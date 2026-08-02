#!/usr/bin/env python3
"""Summarize command-free candidate fixture planning-scene collision evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

EXPECTED_OBJECTS = {
    "rubiks_support": {
        "dimensions_m": [0.075, 0.040, 0.010],
        "relative_translation_m": [-0.019, 0.0, 0.0857],
    },
    "rubiks_cube": {
        "dimensions_m": [0.057, 0.057, 0.057],
        "relative_translation_m": [-0.019, 0.0, 0.1192],
    },
}
EXPECTED_STATE_COUNT = 601


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_bool(text: str) -> bool:
    if text == "true":
        return True
    if text == "false":
        return False
    raise ValueError(f"invalid boolean {text!r}")


def close_vector(first: list[float], second: list[float], tolerance: float = 1e-12) -> bool:
    return len(first) == len(second) and all(
        math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tolerance)
        for a, b in zip(first, second)
    )


def parse_results(path: Path) -> tuple[dict, list[dict], list[dict]]:
    metadata = None
    objects: list[dict] = []
    states: list[dict] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if not raw_line:
            continue
        fields = raw_line.split("\t")
        if fields[0] == "META":
            if len(fields) != 6 or metadata is not None:
                raise ValueError("invalid or duplicate META row")
            metadata = {
                "robot_name": fields[1],
                "state_count": int(fields[2]),
                "missing_collision_geometry_count": int(fields[3]),
                "gripper_left_position_m": float(fields[4]),
                "gripper_right_position_m": float(fields[5]),
            }
        elif fields[0] == "OBJECT":
            if len(fields) != 24:
                raise ValueError(f"invalid OBJECT row: {raw_line}")
            objects.append(
                {
                    "id": fields[1],
                    "dimensions_m": [float(value) for value in fields[2:5]],
                    "relative_translation_m": [float(value) for value in fields[5:8]],
                    "world_transform_matrix": [
                        [float(fields[8 + row * 4 + column]) for column in range(4)]
                        for row in range(4)
                    ],
                }
            )
        elif fields[0] == "STATE":
            if len(fields) != 9:
                raise ValueError(f"invalid STATE row: {raw_line}")
            states.append(
                {
                    "label": fields[1],
                    "arm_bounds_ok": parse_bool(fields[2]),
                    "gripper_bounds_ok": parse_bool(fields[3]),
                    "collision": parse_bool(fields[4]),
                    "self_contact_pair_count": int(fields[5]),
                    "world_contact_pair_count": int(fields[6]),
                    "self_contact_pairs": fields[7].split(",") if fields[7] else [],
                    "world_contact_pairs": fields[8].split(",") if fields[8] else [],
                }
            )
        else:
            raise ValueError(f"unknown checker row: {raw_line}")
    if metadata is None:
        raise ValueError("META row is missing")
    return metadata, objects, states


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prepared-summary", required=True)
    parser.add_argument("--checker-results", required=True)
    parser.add_argument("--accepted-artifact-zip", required=True)
    parser.add_argument("--states-tsv", required=True)
    parser.add_argument("--objects-tsv", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    prepared_path = Path(args.prepared_summary)
    results_path = Path(args.checker_results)
    artifact_path = Path(args.accepted_artifact_zip)
    states_path = Path(args.states_tsv)
    objects_path = Path(args.objects_tsv)
    urdf_path = Path(args.urdf)
    srdf_path = Path(args.srdf)
    output_path = Path(args.output)
    manifest_path = Path(args.manifest)
    errors: list[str] = []

    try:
        prepared = json.loads(prepared_path.read_text(encoding="utf-8"))
        metadata, objects, states = parse_results(results_path)
        if prepared.get("passed") is not True or prepared.get("errors") != []:
            errors.append(f"planning-scene input preparation failed: {prepared.get('errors')}")
        if prepared.get("source_ref") != args.source_ref:
            errors.append("prepared input summary is not exact-head bound")
        if prepared.get("phase") != "RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE":
            errors.append("unexpected prepared phase")

        accepted = prepared.get("accepted_artifact", {})
        if accepted.get("artifact_sha256") != "0f25766bd5e94a10f459538dcbeb293f42af25b780e35ba3a48e270e0b3bcf48":
            errors.append("accepted PR #15 artifact binding mismatch")
        if digest(artifact_path) != accepted.get("artifact_sha256"):
            errors.append("downloaded PR #15 artifact digest mismatch")

        if metadata.get("robot_name") != "turtlebot3_lime":
            errors.append("canonical production robot model was not used")
        if metadata.get("state_count") != EXPECTED_STATE_COUNT:
            errors.append("checker state count mismatch")
        if metadata.get("missing_collision_geometry_count") != 0:
            errors.append("required robot collision geometry is missing")
        measured_gripper = float(
            prepared.get("measured_state", {}).get("gripper_left_position_m", math.nan)
        )
        if not math.isclose(
            float(metadata.get("gripper_left_position_m", math.nan)),
            measured_gripper,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            errors.append("checker gripper-left state differs from measured artifact")
        if not math.isclose(
            float(metadata.get("gripper_right_position_m", math.nan)),
            measured_gripper,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            errors.append("URDF mimic did not reproduce measured open gripper state")

        if len(objects) != 2 or {item["id"] for item in objects} != set(EXPECTED_OBJECTS):
            errors.append("planning scene does not contain exact support and cube IDs")
        for item in objects:
            expected = EXPECTED_OBJECTS.get(item["id"])
            if expected is None:
                continue
            if not close_vector(item["dimensions_m"], expected["dimensions_m"]):
                errors.append(f"{item['id']} dimensions mismatch")
            if not close_vector(
                item["relative_translation_m"], expected["relative_translation_m"]
            ):
                errors.append(f"{item['id']} relative transform mismatch")
            matrix = item["world_transform_matrix"]
            if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
                errors.append(f"{item['id']} world transform is malformed")
            if not close_vector(matrix[3], [0.0, 0.0, 0.0, 1.0]):
                errors.append(f"{item['id']} world transform homogeneous row mismatch")

        if len(states) != EXPECTED_STATE_COUNT:
            errors.append("collision result matrix size mismatch")
        if not states or states[0].get("label") != "measured_candidate":
            errors.append("collision result matrix does not begin with measured candidate")
        expected_labels = ["measured_candidate"]
        for case in ("center", "minus_y", "plus_y", "minus_yaw", "plus_yaw", "mixed"):
            expected_labels.extend(f"{case}_{index:03d}" for index in range(1, 101))
        if [state.get("label") for state in states] != expected_labels:
            errors.append("collision result label order mismatch")

        for state in states:
            if state.get("arm_bounds_ok") is not True:
                errors.append(f"{state.get('label')} violates arm bounds")
            if state.get("gripper_bounds_ok") is not True:
                errors.append(f"{state.get('label')} violates gripper bounds")
            if state.get("collision") is not False:
                errors.append(f"{state.get('label')} is in collision")
            if state.get("self_contact_pair_count") != 0 or state.get("self_contact_pairs") != []:
                errors.append(f"{state.get('label')} has self-collision contacts")
            if state.get("world_contact_pair_count") != 0 or state.get("world_contact_pairs") != []:
                errors.append(f"{state.get('label')} has robot-world contacts")

        safety = prepared.get("safety", {})
        if safety.get("offline_only") is not True:
            errors.append("input preparation is not offline-only")
        for key in (
            "artifact_values_copied_by_hand",
            "robot_object_acm_exemption_added",
            "ros_node_created",
            "controller_loaded",
            "planning_request_created",
            "trajectory_instantiated",
            "command_sent",
            "actuation_authorized",
            "production_runtime_modified",
        ):
            if safety.get(key) is not False:
                errors.append(f"{key} must remain false")

        summary = {
            "schema_version": 1,
            "phase": "RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE",
            "writer_lease": "WL-RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE-20260803-01",
            "source_ref": args.source_ref,
            "passed": not errors,
            "errors": errors,
            "decision": "CANDIDATE_FIXTURE_ENVIRONMENT_COLLISION_CLEAR" if not errors else "REJECT",
            "accepted_artifact": accepted,
            "measured_state": prepared.get("measured_state"),
            "planning_scene": {
                "robot_name": metadata.get("robot_name"),
                "objects": objects,
                "object_count": len(objects),
                "robot_object_acm_exemptions_added": False,
                "world_world_support_cube_contact_ignored_by_robot_collision_query": True,
            },
            "collision_matrix": {
                "state_count": len(states),
                "candidate_state_count": 1,
                "correction_case_count": 6,
                "interpolation_samples_per_case": 100,
                "all_arm_bounds_ok": all(state["arm_bounds_ok"] for state in states),
                "all_gripper_bounds_ok": all(state["gripper_bounds_ok"] for state in states),
                "all_self_collision_free": all(
                    state["self_contact_pair_count"] == 0 for state in states
                ),
                "all_robot_world_collision_free": all(
                    state["world_contact_pair_count"] == 0 for state in states
                ),
                "maximum_self_contact_pair_count": max(
                    (state["self_contact_pair_count"] for state in states), default=0
                ),
                "maximum_world_contact_pair_count": max(
                    (state["world_contact_pair_count"] for state in states), default=0
                ),
            },
            "verified_envelope": {
                "lateral_m": 0.0005,
                "yaw_rad": math.radians(1.0),
                "mixed_case_checked": True,
                "supported_fixture_fixed_world_objects": True,
                "gripper_open_and_fixed": True,
                "dynamic_tracking_verified": False,
                "contact_retention_verified": False,
            },
            "next_required_phase": "RUBIK-SUPPORTED-CORRECTION-SIMULATION-TRIAL",
            "rollback": {
                "behavior": "deterministic_no_op",
                "on_collision": "reject",
                "on_bounds_violation": "reject",
                "command_sent": False,
            },
            "safety": {
                "offline_moveit_core_only": True,
                "artifact_values_copied_by_hand": False,
                "robot_object_acm_exemption_added": False,
                "ros_node_created": False,
                "controller_loaded": False,
                "planning_request_created": False,
                "trajectory_instantiated": False,
                "command_sent": False,
                "physical_hardware_used": False,
                "physical_hardware_ready_claimed": False,
                "lift_command_sent": False,
                "support_removed": False,
                "ifra_attachment_used": False,
                "grasp_success_claimed": False,
                "actuation_authorized": False,
                "production_runtime_modified": False,
            },
        }
    except Exception as error:
        summary = {
            "schema_version": 1,
            "phase": "RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE",
            "writer_lease": "WL-RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE-20260803-01",
            "source_ref": args.source_ref,
            "passed": False,
            "errors": [f"{type(error).__name__}: {error}"],
            "decision": "REJECT",
            "safety": {
                "offline_moveit_core_only": True,
                "command_sent": False,
                "actuation_authorized": False,
                "production_runtime_modified": False,
            },
        }

    output_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "phase": summary["phase"],
        "source_ref": args.source_ref,
        "exact_head_bound": bool(args.source_ref),
        "summary": output_path.name,
        "summary_sha256": digest(output_path),
        "inputs": {
            "prepared_summary": digest(prepared_path),
            "checker_results": digest(results_path),
            "accepted_artifact_zip": digest(artifact_path),
            "states_tsv": digest(states_path),
            "objects_tsv": digest(objects_path),
            "urdf": digest(urdf_path),
            "srdf": digest(srdf_path),
        },
        "command_free": True,
        "robot_object_acm_exemption_added": False,
        "actuation_authorized": False,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
