#!/usr/bin/env python3
"""Prepare exact artifact-bound candidate fixture planning-scene inputs."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np

from rubiks_kinematic_preflight import CASES, model, solve, valid

EXPECTED_ARTIFACT_SHA256 = "0f25766bd5e94a10f459538dcbeb293f42af25b780e35ba3a48e270e0b3bcf48"
EXPECTED_MEMBERS = {
    "rubiks_pregrasp_supported_fixture_summary.json": "2aadd67c28771193004299ba1e63f7e756f5c5c458b356235dacb16f5a90f4f8",
    "rubiks_pregrasp_supported_fixture_evidence.manifest.json": "92d651ecae1a8ad285ddb608d185c6acc9d6a8f22f6f398e51da5eb5907e76bf",
    "rubiks_pregrasp_supported_fixture_observation_telemetry.json": "1c96f72da0f1c5a47bd3776d6af704a66d144b486ac3c509010673253f486c6f",
    "rubiks_pregrasp_supported_fixture_arm_telemetry.json": "e8b876a8ee156ce020bd8e91249faacbec9c342ce136d5a7d5bdf09f6d2eaaa4",
    "rubiks_pregrasp_alignment_search.json": "a61c5ca1c44b9cbbfb00022e06ef32092840abf05c379275b28e53b20e63f5ac",
}
JOINT_NAMES = [f"joint{index}" for index in range(1, 7)]
INTERPOLATION_SAMPLES = 100
SUPPORT_ID = "rubiks_support"
CUBE_ID = "rubiks_cube"
SUPPORT_SIZE = [0.075, 0.040, 0.010]
CUBE_SIZE = [0.057, 0.057, 0.057]


def digest_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def digest_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def close_vector(first: list[float], second: list[float], tolerance: float = 1e-12) -> bool:
    return len(first) == len(second) and all(
        math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tolerance)
        for a, b in zip(first, second)
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--accepted-artifact-zip", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--states-tsv", required=True)
    parser.add_argument("--objects-tsv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    artifact_path = Path(args.accepted_artifact_zip)
    urdf_path = Path(args.urdf)
    states_path = Path(args.states_tsv)
    objects_path = Path(args.objects_tsv)
    output_path = Path(args.output)
    errors: list[str] = []

    if digest_file(artifact_path) != EXPECTED_ARTIFACT_SHA256:
        errors.append("accepted PR #15 artifact digest mismatch")

    with zipfile.ZipFile(artifact_path) as archive:
        members = {name: archive.read(name) for name in EXPECTED_MEMBERS}
    for name, expected in EXPECTED_MEMBERS.items():
        if digest_bytes(members[name]) != expected:
            errors.append(f"accepted artifact member digest mismatch: {name}")

    accepted = json.loads(members["rubiks_pregrasp_supported_fixture_summary.json"])
    accepted_manifest = json.loads(
        members["rubiks_pregrasp_supported_fixture_evidence.manifest.json"]
    )
    fixture_telemetry = json.loads(
        members["rubiks_pregrasp_supported_fixture_observation_telemetry.json"]
    )
    arm_telemetry = json.loads(
        members["rubiks_pregrasp_supported_fixture_arm_telemetry.json"]
    )
    search = json.loads(members["rubiks_pregrasp_alignment_search.json"])

    if accepted.get("passed") is not True or accepted.get("decision") != "SUPPORTED_FIXTURE_STATE_ESTABLISHED":
        errors.append("accepted fixture summary is not passing canonical evidence")
    if accepted_manifest.get("source_ref") != "f03af5493108efd436142c1b015e8cc94c049e29":
        errors.append("accepted fixture manifest source head mismatch")
    if fixture_telemetry.get("passed") is not True or arm_telemetry.get("passed") is not True:
        errors.append("accepted fixture/arm telemetry did not pass")

    measured_state = accepted.get("measured_state", {})
    arm_before = measured_state.get("arm_before_fixture") or {}
    measured_arm = [float(value) for value in arm_before.get("position_rad", [])]
    if len(measured_arm) != 6 or not all(math.isfinite(value) for value in measured_arm):
        errors.append("measured candidate arm state is incomplete or non-finite")
    measured_gripper_left = float(measured_state.get("open_gripper_left_joint_position_m", math.nan))
    if not math.isfinite(measured_gripper_left):
        errors.append("measured open gripper state is missing")
    if abs(measured_gripper_left - 0.019) > 0.001:
        errors.append("measured open gripper state differs from accepted open target")

    fixture = accepted.get("fixture", {})
    support_relative = [float(value) for value in fixture.get("support_pose_xyz_m", [])]
    cube_relative = [float(value) for value in fixture.get("cube_pose_xyz_m", [])]
    if not close_vector(support_relative, [-0.019, 0.0, 0.0857]):
        errors.append("accepted support relative transform changed")
    if not close_vector(cube_relative, [-0.019, 0.0, 0.1192]):
        errors.append("accepted cube relative transform changed")
    if fixture.get("reference_frame") != "turtlebot3_lime_gripper_test::link7":
        errors.append("accepted fixture reference frame changed")
    if int(fixture.get("finger_contact_message_count", -1)) != 0:
        errors.append("accepted fixture contains finger contact")
    if int(fixture.get("unexpected_robot_contact_count", -1)) != 0:
        errors.append("accepted fixture contains unintended robot contact")

    chain, _ = model(urdf_path)
    measured = np.array(measured_arm, dtype=float)
    if not valid(chain, measured):
        errors.append("measured candidate violates current effective arm limits")

    correction_cases: list[dict] = []
    states: list[tuple[str, list[float]]] = [("measured_candidate", measured.tolist())]
    for label, lateral, yaw in CASES:
        result = solve(chain, measured, lateral, yaw)
        result["label"] = label
        correction_cases.append(result)
        if result.get("numerically_feasible") is not True:
            errors.append(f"measured-state correction is not feasible: {label}")
        endpoint = np.array(result["preview_joint_positions_rad"], dtype=float)
        for index in range(1, INTERPOLATION_SAMPLES + 1):
            fraction = index / INTERPOLATION_SAMPLES
            position = measured + fraction * (endpoint - measured)
            states.append((f"{label}_{index:03d}", position.tolist()))

    if len(states) != 1 + len(CASES) * INTERPOLATION_SAMPLES:
        errors.append("prepared correction-state matrix size mismatch")

    states_path.write_text(
        "".join(
            label
            + "\t"
            + "\t".join(format(value, ".17g") for value in positions)
            + "\t"
            + format(measured_gripper_left, ".17g")
            + "\n"
            for label, positions in states
        ),
        encoding="utf-8",
    )
    objects_path.write_text(
        "".join(
            (
                object_id
                + "\t"
                + "\t".join(format(value, ".17g") for value in size + relative)
                + "\n"
            )
            for object_id, size, relative in (
                (SUPPORT_ID, SUPPORT_SIZE, support_relative),
                (CUBE_ID, CUBE_SIZE, cube_relative),
            )
        ),
        encoding="utf-8",
    )

    prepared = {
        "schema_version": 1,
        "phase": "RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE",
        "writer_lease": "WL-RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE-20260803-01",
        "source_ref": args.source_ref,
        "passed": not errors,
        "errors": errors,
        "accepted_artifact": {
            "workflow_run": 30761961570,
            "head_sha": "f03af5493108efd436142c1b015e8cc94c049e29",
            "artifact_id": 8837821223,
            "artifact_sha256": EXPECTED_ARTIFACT_SHA256,
            "summary_sha256": EXPECTED_MEMBERS["rubiks_pregrasp_supported_fixture_summary.json"],
            "manifest_sha256": EXPECTED_MEMBERS["rubiks_pregrasp_supported_fixture_evidence.manifest.json"],
        },
        "measured_state": {
            "joint_names": JOINT_NAMES,
            "arm_position_rad": measured_arm,
            "gripper_left_position_m": measured_gripper_left,
            "gripper_right_source": "URDF mimic of measured gripper_left_joint",
        },
        "objects": [
            {
                "id": SUPPORT_ID,
                "type": "box",
                "dimensions_m": SUPPORT_SIZE,
                "relative_to": "link7",
                "relative_translation_m": support_relative,
                "relative_rotation": "identity",
            },
            {
                "id": CUBE_ID,
                "type": "box",
                "dimensions_m": CUBE_SIZE,
                "relative_to": "link7",
                "relative_translation_m": cube_relative,
                "relative_rotation": "identity",
            },
        ],
        "correction_cases": correction_cases,
        "state_matrix": {
            "path": states_path.name,
            "state_count": len(states),
            "interpolation_samples_per_case": INTERPOLATION_SAMPLES,
            "candidate_state_count": 1,
            "correction_case_count": len(CASES),
            "gripper_state_fixed": True,
        },
        "object_matrix": {"path": objects_path.name, "object_count": 2},
        "safety": {
            "offline_only": True,
            "artifact_values_copied_by_hand": False,
            "robot_object_acm_exemption_added": False,
            "ros_node_created": False,
            "controller_loaded": False,
            "planning_request_created": False,
            "trajectory_instantiated": False,
            "command_sent": False,
            "actuation_authorized": False,
            "production_runtime_modified": False,
        },
    }
    output_path.write_text(json.dumps(prepared, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(prepared, indent=2, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
