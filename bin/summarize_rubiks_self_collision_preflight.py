#!/usr/bin/env python3
"""Summarize command-free MoveIt Core self-collision evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

EXPECTED_LABELS = ["seed", "center", "minus_y", "plus_y", "minus_yaw", "plus_yaw", "mixed"]
EXPECTED_JOINTS = [f"joint{index}" for index in range(1, 7)]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_bool(text: str) -> bool:
    if text == "true":
        return True
    if text == "false":
        return False
    raise ValueError(f"invalid boolean {text!r}")


def parse_results(path: Path) -> tuple[dict, list[dict]]:
    metadata = None
    states: list[dict] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if not raw_line:
            continue
        fields = raw_line.split("\t")
        if fields[0] == "META":
            if len(fields) != 7 or metadata is not None:
                raise ValueError("invalid or duplicate META row")
            metadata = {
                "robot_name": fields[1],
                "group_name": fields[2],
                "active_joint_order": fields[3].split(",") if fields[3] else [],
                "missing_collision_geometry": fields[4].split(",") if fields[4] else [],
                "adjacent_acm_entry_present": parse_bool(fields[5]),
                "adjacent_acm_entry_allowed": parse_bool(fields[6]),
            }
        elif fields[0] == "STATE":
            if len(fields) != 6:
                raise ValueError(f"invalid STATE row: {raw_line}")
            states.append(
                {
                    "label": fields[1],
                    "bounds_ok": parse_bool(fields[2]),
                    "self_collision": parse_bool(fields[3]),
                    "contact_pair_count": int(fields[4]),
                    "contact_pairs": fields[5].split(",") if fields[5] else [],
                }
            )
        else:
            raise ValueError(f"unknown result row: {raw_line}")
    if metadata is None:
        raise ValueError("META row is missing")
    return metadata, states


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-tsv", required=True)
    parser.add_argument("--kinematic-summary", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    results_path = Path(args.results_tsv)
    kinematic_path = Path(args.kinematic_summary)
    urdf_path = Path(args.urdf)
    srdf_path = Path(args.srdf)
    output_path = Path(args.output)
    manifest_path = Path(args.manifest)
    errors: list[str] = []

    try:
        metadata, states = parse_results(results_path)
        kinematic = json.loads(kinematic_path.read_text(encoding="utf-8"))
        srdf_root = ET.parse(srdf_path).getroot()
        arm_group = next((group for group in srdf_root.findall("group") if group.get("name") == "arm"), None)
        semantic_arm_joints = [joint.get("name") for joint in arm_group.findall("joint")] if arm_group is not None else []
        disabled_collisions = [
            {"link1": item.get("link1"), "link2": item.get("link2"), "reason": item.get("reason")}
            for item in srdf_root.findall("disable_collisions")
        ]

        if kinematic.get("passed") is not True:
            errors.append("parent kinematic summary did not pass")
        if kinematic.get("source_ref") != args.source_ref:
            errors.append("parent kinematic summary is not bound to exact head")
        if metadata["robot_name"] != srdf_root.get("name"):
            errors.append("URDF/SRDF robot name mismatch")
        if metadata["group_name"] != "arm":
            errors.append("MoveIt arm group was not used")
        if metadata["active_joint_order"] != EXPECTED_JOINTS:
            errors.append("MoveIt active joint order mismatch")
        if semantic_arm_joints[:6] != EXPECTED_JOINTS:
            errors.append("SRDF semantic arm joint order mismatch")
        if metadata["missing_collision_geometry"]:
            errors.append(f"missing collision geometry: {metadata['missing_collision_geometry']}")
        if metadata["adjacent_acm_entry_present"] is not True or metadata["adjacent_acm_entry_allowed"] is not True:
            errors.append("SRDF allowed-collision matrix was not applied")
        if not disabled_collisions:
            errors.append("SRDF contains no disabled-collision entries")
        if [state["label"] for state in states] != EXPECTED_LABELS:
            errors.append("seed/preview state order mismatch")
        for state in states:
            if state["bounds_ok"] is not True:
                errors.append(f"{state['label']} violates MoveIt bounds")
            if state["self_collision"] is not False:
                errors.append(f"{state['label']} is self-colliding: {state['contact_pairs']}")
            if state["contact_pair_count"] != len(state["contact_pairs"]):
                errors.append(f"{state['label']} contact-pair count mismatch")
            if state["contact_pair_count"] != 0:
                errors.append(f"{state['label']} reported collision contacts")

        summary = {
            "schema_version": 1,
            "phase": "RUBIK-SELF-COLLISION-PREFLIGHT",
            "writer_lease": "WL-RUBIK-SELF-COLLISION-PREFLIGHT-20260803-01",
            "source_ref": args.source_ref,
            "passed": not errors,
            "errors": errors,
            "model": {
                **metadata,
                "semantic_arm_joints": semantic_arm_joints,
                "disabled_collision_entry_count": len(disabled_collisions),
                "required_collision_link_count": 9,
            },
            "states": states,
            "verified_envelope": {
                "self_collision_verified_lateral_m": 0.0005,
                "self_collision_verified_yaw_rad": math.radians(1),
                "self_collision_verified_mixed": True,
                "environment_collision_verified_lateral_m": 0.0,
                "environment_collision_verified_yaw_rad": 0.0,
                "environment_collision_status": "unknown_not_checked",
            },
            "rollback": {
                "behavior": "deterministic_no_op",
                "on_self_collision": "reject",
                "on_missing_geometry": "reject",
                "on_semantic_model_error": "reject",
                "on_environment_state_unknown": "reject",
            },
            "safety": {
                "offline_moveit_core_only": True,
                "ros_node_created": False,
                "service_client_created": False,
                "publisher_created": False,
                "action_client_created": False,
                "controller_loaded": False,
                "planning_request_created": False,
                "trajectory_instantiated": False,
                "arm_command_sent": False,
                "gripper_command_sent": False,
                "gazebo_motion_performed": False,
                "support_removed": False,
                "lift_command_sent": False,
                "ifra_attachment_used": False,
                "self_collision_clearance_verified": not errors,
                "environment_collision_clearance_verified": False,
                "actuation_authorized": False,
                "production_runtime_modified": False,
            },
        }
    except Exception as error:
        summary = {
            "schema_version": 1,
            "phase": "RUBIK-SELF-COLLISION-PREFLIGHT",
            "writer_lease": "WL-RUBIK-SELF-COLLISION-PREFLIGHT-20260803-01",
            "source_ref": args.source_ref,
            "passed": False,
            "errors": [f"{type(error).__name__}: {error}"],
            "safety": {
                "offline_moveit_core_only": True,
                "ros_node_created": False,
                "service_client_created": False,
                "publisher_created": False,
                "action_client_created": False,
                "arm_command_sent": False,
                "gripper_command_sent": False,
                "environment_collision_clearance_verified": False,
                "actuation_authorized": False,
                "production_runtime_modified": False,
            },
        }

    output_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "phase": "RUBIK-SELF-COLLISION-PREFLIGHT",
        "source_ref": args.source_ref,
        "exact_head_bound": bool(args.source_ref),
        "summary": output_path.name,
        "summary_sha256": digest(output_path),
        "inputs": {
            "results_tsv": digest(results_path),
            "kinematic_summary": digest(kinematic_path),
            "urdf": digest(urdf_path),
            "srdf": digest(srdf_path),
        },
        "command_free": True,
        "environment_collision_checked": False,
        "actuation_authorized": False,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
