#!/usr/bin/env python3
"""Audit whether accepted supported-hold evidence can seed environment collision checks."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile
import xml.etree.ElementTree as ET

import numpy as np

EXPECTED_ARTIFACT_SHA256 = "b5f3bcc6205c57f53e013531d6c5b2bf2bdc0b0e680c664985ea850567bcf1d3"
EXPECTED_MEMBERS = {
    "rubiks_dynamic_supported_hold_telemetry.json": "11b78f5ad0e32e0f33cf1d56b1e3e5edbb0d4db08b19546f9856a009ee606b53",
    "turtlebot3_lime_gripper_test.urdf": "b9f3ee6a3cbdd43dd43dd40e63d089e5865abe6567e6ae98a2d97a9479689eca",
    "rubiks_dynamic_supported_hold_summary.json": "8c3ffff34788a677a2dfa10718c19843703d2fad9fc079a9b46e9bd679e0206f",
    "rubiks_dynamic_supported_hold_evidence.manifest.json": "a14dd262a66af51f73ff357344915cb0be50193043744269a26d37d30abe0622",
}
JOINTS = [f"joint{index}" for index in range(1, 7)]
DAMPING = 1e-4


def sha256_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(text: str | None, default: str = "0 0 0") -> np.ndarray:
    return np.array([float(value) for value in (text or default).split()], dtype=float)


def rotation_from_rpy(value: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = value
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )
    )


def rotation_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cosine, sine = math.cos(angle), math.sin(angle)
    delta = 1.0 - cosine
    return np.array(
        (
            (cosine + x * x * delta, x * y * delta - z * sine, x * z * delta + y * sine),
            (y * x * delta + z * sine, cosine + y * y * delta, y * z * delta - x * sine),
            (z * x * delta - y * sine, z * y * delta + x * sine, cosine + z * z * delta),
        )
    )


def transform(rotation: np.ndarray | None = None, position: np.ndarray | None = None) -> np.ndarray:
    value = np.eye(4)
    value[:3, :3] = np.eye(3) if rotation is None else rotation
    value[:3, 3] = 0.0 if position is None else position
    return value


def configured_initial_positions(urdf_content: bytes) -> list[float | None]:
    root = ET.fromstring(urdf_content)
    found: dict[str, float | None] = {}
    for control in root.findall("ros2_control"):
        for joint in control.findall("joint"):
            name = joint.get("name")
            if name not in JOINTS:
                continue
            initial_value = None
            for state_interface in joint.findall("state_interface"):
                if state_interface.get("name") != "position":
                    continue
                for parameter in state_interface.findall("param"):
                    if parameter.get("name") == "initial_value":
                        initial_value = float((parameter.text or "").strip())
            found[name] = initial_value
    return [found.get(name) for name in JOINTS]


def arm_chain(urdf_path: Path) -> list[dict]:
    root = ET.parse(urdf_path).getroot()
    by_child = {}
    for element in root.findall("joint"):
        child = element.find("child")
        if child is not None and child.get("link"):
            by_child[child.get("link")] = element

    raw_chain = []
    current = "link7"
    while current != "link1":
        element = by_child.get(current)
        if element is None:
            raise ValueError(f"cannot resolve link1-to-link7 chain at {current}")
        raw_chain.append(element)
        current = element.find("parent").get("link")

    chain = []
    for element in reversed(raw_chain):
        if element.get("type") == "fixed":
            continue
        origin = element.find("origin")
        axis = element.find("axis")
        chain.append(
            {
                "name": element.get("name"),
                "xyz": vector(origin.get("xyz") if origin is not None else None),
                "rpy": vector(origin.get("rpy") if origin is not None else None),
                "axis": vector(axis.get("xyz") if axis is not None else "1 0 0"),
            }
        )
    if [joint["name"] for joint in chain] != JOINTS:
        raise ValueError("resolved arm joint order is not joint1 through joint6")
    return chain


def forward_jacobian(chain: list[dict], positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    frame = np.eye(4)
    axes = []
    points = []
    for joint, position in zip(chain, positions):
        frame = frame @ transform(rotation_from_rpy(joint["rpy"]), joint["xyz"])
        axis = frame[:3, :3] @ (joint["axis"] / np.linalg.norm(joint["axis"]))
        axes.append(axis)
        points.append(frame[:3, 3].copy())
        frame = frame @ transform(rotation_from_axis_angle(joint["axis"], float(position)))

    jacobian = np.zeros((6, 6))
    for index, (axis, point) in enumerate(zip(axes, points)):
        jacobian[:3, index] = np.cross(axis, frame[:3, 3] - point)
        jacobian[3:, index] = axis
    return frame, jacobian


def solve_twist(jacobian: np.ndarray, rotation: np.ndarray, lateral: float, yaw: float) -> dict:
    requested = np.r_[
        rotation @ np.array((0.0, lateral, 0.0)),
        rotation @ np.array((0.0, 0.0, yaw)),
    ]
    joint_delta = jacobian.T @ np.linalg.solve(
        jacobian @ jacobian.T + DAMPING**2 * np.eye(6), requested
    )
    achieved = jacobian @ joint_delta
    local_translation = rotation.T @ achieved[:3]
    local_rotation = rotation.T @ achieved[3:]
    return {
        "requested_lateral_m": lateral,
        "requested_yaw_rad": yaw,
        "joint_delta_rad": joint_delta.tolist(),
        "max_abs_joint_delta_rad": float(np.max(np.abs(joint_delta))),
        "achieved_lateral_m": float(local_translation[1]),
        "achieved_yaw_rad": float(local_rotation[2]),
        "lateral_residual_m": float(lateral - local_translation[1]),
        "yaw_residual_rad": float(yaw - local_rotation[2]),
        "twist_residual_norm": float(np.linalg.norm(requested - achieved)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--accepted-artifact-zip", required=True)
    parser.add_argument("--current-urdf", required=True)
    parser.add_argument("--kinematic-summary", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    artifact_path = Path(args.accepted_artifact_zip)
    current_urdf_path = Path(args.current_urdf)
    kinematic_path = Path(args.kinematic_summary)
    output_path = Path(args.output)
    manifest_path = Path(args.manifest)
    errors: list[str] = []

    if sha256_file(artifact_path) != EXPECTED_ARTIFACT_SHA256:
        errors.append("accepted supported-hold artifact digest mismatch")

    with zipfile.ZipFile(artifact_path) as archive:
        members = {name: archive.read(name) for name in EXPECTED_MEMBERS}
    for name, expected_digest in EXPECTED_MEMBERS.items():
        if sha256_bytes(members[name]) != expected_digest:
            errors.append(f"accepted artifact member digest mismatch: {name}")

    telemetry = json.loads(members["rubiks_dynamic_supported_hold_telemetry.json"])
    accepted_summary = json.loads(members["rubiks_dynamic_supported_hold_summary.json"])
    accepted_manifest = json.loads(members["rubiks_dynamic_supported_hold_evidence.manifest.json"])
    configured_positions = configured_initial_positions(members["turtlebot3_lime_gripper_test.urdf"])

    kinematic = json.loads(kinematic_path.read_text(encoding="utf-8"))
    kinematic_seed = np.array(kinematic["ik_method"]["seed_joint_positions_rad"], dtype=float)
    configured_state = np.array(configured_positions, dtype=float)

    chain = arm_chain(current_urdf_path)
    end_frame, jacobian = forward_jacobian(chain, configured_state)
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    rank = int(np.linalg.matrix_rank(jacobian, tol=1e-9))
    condition_number = math.inf if singular_values[-1] <= 1e-12 else float(singular_values[0] / singular_values[-1])

    cases = []
    for label, lateral, yaw in (
        ("plus_y", 0.0005, 0.0),
        ("minus_y", -0.0005, 0.0),
        ("plus_yaw", 0.0, math.radians(1.0)),
        ("mixed", 0.0005, -math.radians(1.0)),
    ):
        result = solve_twist(jacobian, end_frame[:3, :3], lateral, yaw)
        result["label"] = label
        cases.append(result)

    if accepted_summary.get("passed") is not True or telemetry.get("errors") != []:
        errors.append("accepted supported-hold evidence is not passing evidence")
    if telemetry.get("arm_motion_performed") is not False or accepted_manifest.get("arm_motion_performed") is not False:
        errors.append("accepted supported-hold arm-motion boundary changed")
    if telemetry.get("cube_pose_relative_to") != "turtlebot3_lime_gripper_test::link7":
        errors.append("unexpected accepted environment reference frame")
    if telemetry.get("cube_pose_xyz_m") != [-0.019, 0.0, 0.1192]:
        errors.append("accepted cube pose changed")
    if telemetry.get("support_pose_xyz_m") != [-0.019, 0.0, 0.0857]:
        errors.append("accepted support pose changed")
    if any(value is None or abs(float(value)) > 1e-12 for value in configured_positions):
        errors.append(f"configured supported-hold initial arm state is not all zero: {configured_positions}")

    maximum_seed_difference = float(np.max(np.abs(kinematic_seed - configured_state)))
    if maximum_seed_difference < 0.1:
        errors.append("PR #10 kinematic seed unexpectedly matches configured supported-hold state")
    if rank >= 6 or math.isfinite(condition_number):
        errors.append("configured supported-hold state is unexpectedly nonsingular")
    plus_y = next(case for case in cases if case["label"] == "plus_y")
    if abs(float(plus_y["lateral_residual_m"])) < 0.00049:
        errors.append("configured supported-hold state unexpectedly resolves the lateral correction")

    blockers = [
        "accepted physics artifact does not record measured joint1..joint6 samples",
        "configured supported-hold arm initial state is all-zero while PR #10 uses a nonzero numerical seed",
        "all-zero supported-hold state has a rank-deficient Jacobian and cannot produce the accepted lateral correction",
        "placing accepted cube/support poses around the PR #10 seed would assume an unverified whole-arm repositioning",
    ]

    summary = {
        "schema_version": 1,
        "phase": "RUBIK-ENVIRONMENT-COLLISION-PREFLIGHT",
        "writer_lease": "WL-RUBIK-ENVIRONMENT-COLLISION-PREFLIGHT-20260803-01",
        "source_ref": args.source_ref,
        "audit_passed": not errors,
        "errors": errors,
        "decision": "BLOCKED_INPUT_STATE_MISMATCH_AND_SINGULARITY",
        "environment_phase_ready": False,
        "blockers": blockers,
        "accepted_physics_evidence": {
            "workflow_run": 30290131232,
            "head_sha": "a759a6efa2bd945aeff17697da22a19d8b41bb12",
            "artifact_id": 8662639552,
            "artifact_sha256": EXPECTED_ARTIFACT_SHA256,
            "cube_pose_relative_to": telemetry.get("cube_pose_relative_to"),
            "cube_pose_xyz_m": telemetry.get("cube_pose_xyz_m"),
            "support_pose_xyz_m": telemetry.get("support_pose_xyz_m"),
            "arm_motion_performed": telemetry.get("arm_motion_performed"),
            "measured_arm_joint_state_present": False,
            "configured_initial_arm_joint_positions_rad": dict(zip(JOINTS, configured_positions)),
        },
        "kinematic_seed": {
            "joint_positions_rad": dict(zip(JOINTS, kinematic_seed.tolist())),
            "maximum_absolute_difference_from_configured_physics_state_rad": maximum_seed_difference,
        },
        "configured_state_jacobian": {
            "rank": rank,
            "dimension": [6, 6],
            "singular_values": singular_values.tolist(),
            "condition_number": None if not math.isfinite(condition_number) else condition_number,
            "condition_number_class": "infinite" if not math.isfinite(condition_number) else "finite",
            "cases": cases,
        },
        "verified_environment_collision_envelope": {
            "lateral_m": 0.0,
            "yaw_rad": 0.0,
            "reason": "no evidence-consistent common initial arm state exists for the accepted environment pose and PR #10 correction endpoints",
        },
        "next_required_phase": "RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT",
        "safety": {
            "offline_analysis_only": True,
            "environment_collision_checked": False,
            "ros_node_created": False,
            "controller_loaded": False,
            "planning_request_created": False,
            "trajectory_instantiated": False,
            "arm_command_sent": False,
            "gripper_command_sent": False,
            "actuation_authorized": False,
            "production_runtime_modified": False,
        },
    }
    output_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    manifest = {
        "schema_version": 1,
        "phase": summary["phase"],
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "decision": summary["decision"],
        "summary": output_path.name,
        "summary_sha256": sha256_file(output_path),
        "inputs": {
            "accepted_artifact_zip": sha256_file(artifact_path),
            "current_urdf": sha256_file(current_urdf_path),
            "kinematic_summary": sha256_file(kinematic_path),
        },
        "environment_collision_checked": False,
        "actuation_authorized": False,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
