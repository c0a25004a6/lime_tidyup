#!/usr/bin/env python3
"""Offline, command-free Lime arm kinematic preflight."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

BASE, TIP = "link1", "link7"
MARGIN = math.radians(1)
MAX_COND = 250.0
MAX_STEP = math.radians(5)
DAMP = 1e-4
CASES = (
    ("center", 0.0, 0.0),
    ("minus_y", -0.0005, 0.0),
    ("plus_y", 0.0005, 0.0),
    ("minus_yaw", 0.0, -math.radians(1)),
    ("plus_yaw", 0.0, math.radians(1)),
    ("mixed", 0.0005, -math.radians(1)),
)


def vec(text, default):
    value = np.array([float(x) for x in (text.split() if text else default)], dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError(f"bad vector {value}")
    return value


def rot_rpy(value):
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


def rot_axis(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    d = 1 - c
    return np.array(
        (
            (c + x * x * d, x * y * d - z * s, x * z * d + y * s),
            (y * x * d + z * s, c + y * y * d, y * z * d - x * s),
            (z * x * d - y * s, z * y * d + x * s, c + z * z * d),
        )
    )


def transform(rotation=None, position=None):
    value = np.eye(4)
    value[:3, :3] = rotation if rotation is not None else np.eye(3)
    value[:3, 3] = position if position is not None else 0
    return value


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_interface(element):
    params = {}
    for parameter in element.findall("param"):
        name = parameter.get("name")
        text = (parameter.text or "").strip()
        if not name or not text:
            continue
        try:
            params[name] = float(text)
        except ValueError:
            params[name] = text
    return {"name": element.get("name"), "params": params}


def model(path):
    root = ET.parse(path).getroot()
    by_child = {}
    for element in root.findall("joint"):
        child = element.find("child")
        if child is not None and child.get("link"):
            by_child[child.get("link")] = element

    raw_chain = []
    current = TIP
    while current != BASE:
        element = by_child.get(current)
        if element is None:
            raise ValueError(f"cannot resolve chain at {current}")
        raw_chain.append(element)
        current = element.find("parent").get("link")

    chain = []
    for element in reversed(raw_chain):
        joint_type = element.get("type")
        if joint_type == "fixed":
            continue
        if joint_type not in ("revolute", "continuous"):
            raise ValueError(f"unsupported {joint_type}")
        origin = element.find("origin")
        axis = element.find("axis")
        limit = element.find("limit")
        lower = float(limit.get("lower")) if limit is not None and limit.get("lower") else None
        upper = float(limit.get("upper")) if limit is not None and limit.get("upper") else None
        if joint_type == "revolute" and (lower is None or upper is None):
            raise ValueError(f"URDF limits missing for {element.get('name')}")
        chain.append(
            {
                "name": element.get("name"),
                "type": joint_type,
                "parent": element.find("parent").get("link"),
                "child": element.find("child").get("link"),
                "xyz": vec(origin.get("xyz") if origin is not None else None, (0, 0, 0)),
                "rpy": vec(origin.get("rpy") if origin is not None else None, (0, 0, 0)),
                "axis": vec(axis.get("xyz") if axis is not None else None, (1, 0, 0)),
                "urdf_lower": lower,
                "urdf_upper": upper,
                "effort": float(limit.get("effort")) if limit is not None and limit.get("effort") else None,
                "velocity": float(limit.get("velocity")) if limit is not None and limit.get("velocity") else None,
            }
        )
    if len(chain) != 6:
        raise ValueError(f"expected 6 arm joints, got {len(chain)}")

    interfaces = {}
    for control in root.findall("ros2_control"):
        for joint in control.findall("joint"):
            name = joint.get("name")
            if name:
                interfaces[name] = {
                    "command": [parse_interface(x) for x in joint.findall("command_interface")],
                    "state": [parse_interface(x) for x in joint.findall("state_interface")],
                }

    for joint in chain:
        interface = interfaces.get(joint["name"])
        if interface is None:
            raise ValueError(f"{joint['name']} lacks ros2_control declaration")
        position = next((x for x in interface["command"] if x["name"] == "position"), None)
        if position is None:
            raise ValueError(f"{joint['name']} lacks position command interface")
        control_lower = position["params"].get("min")
        control_upper = position["params"].get("max")
        if control_lower is None or control_upper is None:
            raise ValueError(f"{joint['name']} lacks ros2_control position min/max")
        joint["control_lower"] = float(control_lower)
        joint["control_upper"] = float(control_upper)
        joint["effective_lower"] = max(joint["urdf_lower"], joint["control_lower"])
        joint["effective_upper"] = min(joint["urdf_upper"], joint["control_upper"])
        if joint["effective_lower"] + 2 * MARGIN >= joint["effective_upper"]:
            raise ValueError(f"{joint['name']} has no usable limit intersection")

    return chain, interfaces


def controller(path, names):
    data = yaml.safe_load(path.read_text())
    manager = data.get("controller_manager", {}).get("ros__parameters", {})
    found = []
    for name, declaration in manager.items():
        if isinstance(declaration, dict) and "JointTrajectoryController" in str(declaration.get("type", "")):
            params = data.get(name, {}).get("ros__parameters", {})
            joints = [str(x) for x in params.get("joints", [])]
            found.append((name, declaration.get("type"), joints, params))
    exact = [item for item in found if item[2] == names]
    if not exact:
        raise ValueError(f"no exact trajectory-controller joint order; found={[(x[0], x[2]) for x in found]}")
    name, controller_type, joints, params = exact[0]
    return {
        "name": name,
        "type": controller_type,
        "joints": joints,
        "interface_name": params.get("interface_name"),
        "command_interfaces": params.get("command_interfaces", []),
        "state_interfaces": params.get("state_interfaces", []),
        "action_type": "control_msgs/action/FollowJointTrajectory",
        "action_name": f"/{name}/follow_joint_trajectory",
        "joint_order_exact": True,
    }


def clamp(chain, positions):
    positions = positions.copy()
    for index, joint in enumerate(chain):
        if joint["type"] == "continuous":
            positions[index] = math.atan2(math.sin(positions[index]), math.cos(positions[index]))
        else:
            positions[index] = min(
                joint["effective_upper"] - MARGIN,
                max(joint["effective_lower"] + MARGIN, positions[index]),
            )
    return positions


def valid(chain, positions):
    if not np.isfinite(positions).all():
        return False
    return all(
        joint["type"] == "continuous"
        or joint["effective_lower"] + MARGIN <= position <= joint["effective_upper"] - MARGIN
        for joint, position in zip(chain, positions)
    )


def forward_jacobian(chain, positions):
    frame = np.eye(4)
    axes, points = [], []
    for joint, position in zip(chain, positions):
        frame = frame @ transform(rot_rpy(joint["rpy"]), joint["xyz"])
        axes.append(frame[:3, :3] @ (joint["axis"] / np.linalg.norm(joint["axis"])))
        points.append(frame[:3, 3].copy())
        frame = frame @ transform(rot_axis(joint["axis"], position))
    jacobian = np.zeros((6, len(chain)))
    for index, (axis, point) in enumerate(zip(axes, points)):
        jacobian[:3, index] = np.cross(axis, frame[:3, 3] - point)
        jacobian[3:, index] = axis
    return frame, jacobian


def condition_number(jacobian):
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    if singular_values[-1] <= 1e-9:
        return math.inf
    return float(singular_values[0] / singular_values[-1])


def seed(chain):
    midpoint = np.array(
        [0.0 if joint["type"] == "continuous" else (joint["effective_lower"] + joint["effective_upper"]) / 2 for joint in chain]
    )
    span = np.array(
        [2 * math.pi if joint["type"] == "continuous" else joint["effective_upper"] - joint["effective_lower"] for joint in chain]
    )
    patterns = (
        (0.0, (0, 0, 0, 0, 0, 0)),
        (0.1, (1, -1, 1, -1, 1, -1)),
        (0.1, (-1, 1, -1, 1, -1, 1)),
        (0.2, (1, 1, -1, -1, 1, -1)),
        (0.2, (-1, 1, 1, -1, -1, 1)),
        (0.3, (1, -1, -1, 1, 1, -1)),
    )
    scored = []
    for fraction, signs in patterns:
        positions = clamp(chain, midpoint + fraction * span * np.array(signs))
        number = condition_number(forward_jacobian(chain, positions)[1])
        if valid(chain, positions) and math.isfinite(number):
            scored.append((number, f"deterministic:{fraction}:{signs}", positions))
    if not scored:
        raise ValueError("no finite deterministic seed")
    number, label, positions = min(scored, key=lambda item: (item[0], item[1]))
    if number > MAX_COND:
        raise ValueError(f"best seed condition {number} exceeds {MAX_COND}")
    return label, positions, number


def solve(chain, positions, lateral, yaw):
    initial, jacobian = forward_jacobian(chain, positions)
    rotation = initial[:3, :3]
    delta = np.r_[rotation @ np.array((0, lateral, 0.0)), rotation @ np.array((0.0, 0.0, yaw))]
    joint_delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + DAMP**2 * np.eye(6), delta)
    preview = positions + joint_delta
    final, _ = forward_jacobian(chain, preview)
    relative = np.linalg.inv(initial) @ final
    achieved_lateral = float(relative[1, 3])
    achieved_yaw = math.atan2(relative[1, 0], relative[0, 0])
    yaw_residual = math.atan2(math.sin(yaw - achieved_yaw), math.cos(yaw - achieved_yaw))
    unwanted = float(np.linalg.norm(relative[:3, 3] - np.array((0.0, achieved_lateral, 0.0))))
    feasible = (
        valid(chain, preview)
        and np.isfinite(joint_delta).all()
        and np.max(np.abs(joint_delta)) <= MAX_STEP
        and abs(lateral - achieved_lateral) <= 5e-5
        and abs(yaw_residual) <= math.radians(0.05)
        and unwanted <= 1e-4
    )
    return {
        "requested_lateral_m": lateral,
        "requested_yaw_rad": yaw,
        "joint_delta_rad": joint_delta.tolist(),
        "preview_joint_positions_rad": preview.tolist(),
        "max_abs_joint_delta_rad": float(np.max(np.abs(joint_delta))),
        "achieved_lateral_m": achieved_lateral,
        "achieved_yaw_rad": achieved_yaw,
        "lateral_residual_m": lateral - achieved_lateral,
        "yaw_residual_rad": yaw_residual,
        "unwanted_translation_norm_m": unwanted,
        "effective_position_limits_with_margin": valid(chain, preview),
        "numerically_feasible": bool(feasible),
        "trajectory_instantiated": False,
        "command_authorized": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--controller-yaml", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", default="unknown")
    args = parser.parse_args()
    urdf_path, controller_path, output_path, manifest_path = map(
        Path, (args.urdf, args.controller_yaml, args.output, args.manifest)
    )
    errors = []
    try:
        chain, interfaces = model(urdf_path)
        names = [joint["name"] for joint in chain]
        controller_evidence = controller(controller_path, names)
        for name in names:
            command_names = [x["name"] for x in interfaces[name]["command"]]
            state_names = [x["name"] for x in interfaces[name]["state"]]
            if "position" not in command_names or "position" not in state_names:
                errors.append(f"{name} lacks position ros2_control interface")
        label, positions, number = seed(chain)
        cases = []
        for name, lateral, yaw in CASES:
            result = solve(chain, positions, lateral, yaw)
            result["label"] = name
            cases.append(result)
            if not result["numerically_feasible"]:
                errors.append(f"{name} numerical preflight failed")
        joints = [
            {key: (value.tolist() if isinstance(value, np.ndarray) else value) for key, value in joint.items()}
            | {"ros2_control": interfaces[joint["name"]]}
            for joint in chain
        ]
        summary = {
            "schema_version": 2,
            "phase": "RUBIK-KINEMATIC-PREFLIGHT",
            "writer_lease": "WL-RUBIK-KINEMATIC-PREFLIGHT-20260728-01",
            "source_ref": args.source_ref,
            "passed": not errors,
            "errors": errors,
            "arm_chain": {
                "base_link": BASE,
                "tip_link": TIP,
                "joint_count": 6,
                "joint_order": names,
                "limit_policy": "intersection_of_urdf_and_ros2_control_position_command_limits",
                "joints": joints,
            },
            "controller": controller_evidence,
            "ik_method": {
                "name": "damped_least_squares_geometric_jacobian",
                "damping": DAMP,
                "seed_label": label,
                "seed_joint_positions_rad": positions.tolist(),
                "seed_condition_number": number,
                "max_condition_number": MAX_COND,
                "joint_limit_margin_rad": MARGIN,
                "max_joint_step_rad": MAX_STEP,
            },
            "accepted_observation_cases": cases,
            "upstream_rejected_observation_cases": [
                {"label": "reject_y", "ik_attempted": False, "command_authorized": False},
                {"label": "reject_yaw", "ik_attempted": False, "command_authorized": False},
            ],
            "bounds": {
                "kinematic_preview_lateral_m": 0.001,
                "kinematic_preview_yaw_rad": math.radians(2),
                "collision_verified_lateral_m": 0.0,
                "collision_verified_yaw_rad": 0.0,
                "reason": "no planning scene or collision checker executed",
            },
            "rollback": {
                "behavior": "deterministic_no_op",
                "on_collision_state_unknown": "reject",
                "on_joint_limit_violation": "reject",
                "on_singularity": "reject",
                "on_timeout": "reject",
            },
            "safety": {
                "offline_only": True,
                "simulation_oracle_only": True,
                "production_pose_estimator_claimed": False,
                "controller_loaded": False,
                "action_client_created": False,
                "publisher_created": False,
                "trajectory_instantiated": False,
                "arm_command_sent": False,
                "gripper_command_sent": False,
                "support_removed": False,
                "lift_command_sent": False,
                "ifra_attachment_used": False,
                "grasp_success_claimed": False,
                "collision_clearance_verified": False,
                "actuation_authorized": False,
                "production_runtime_modified": False,
            },
        }
    except Exception as error:
        summary = {
            "schema_version": 2,
            "phase": "RUBIK-KINEMATIC-PREFLIGHT",
            "writer_lease": "WL-RUBIK-KINEMATIC-PREFLIGHT-20260728-01",
            "source_ref": args.source_ref,
            "passed": False,
            "errors": [f"{type(error).__name__}: {error}"],
            "safety": {
                "offline_only": True,
                "action_client_created": False,
                "publisher_created": False,
                "arm_command_sent": False,
                "gripper_command_sent": False,
                "actuation_authorized": False,
                "production_runtime_modified": False,
            },
        }
    output_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    manifest = {
        "schema_version": 2,
        "phase": "RUBIK-KINEMATIC-PREFLIGHT",
        "source_ref": args.source_ref,
        "exact_head_bound": args.source_ref not in ("", "unknown"),
        "summary": output_path.name,
        "summary_sha256": digest(output_path),
        "inputs": {"urdf": digest(urdf_path), "controller_yaml": digest(controller_path)},
        "command_free": True,
        "actuation_authorized": False,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
