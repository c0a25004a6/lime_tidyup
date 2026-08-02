#!/usr/bin/env python3
"""Summarize deterministic pregrasp search and MoveIt self-collision evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

EXPECTED_TRANSITION_SEGMENTS = 100
EXPECTED_CORRECTIONS = [
    "center",
    "minus_y",
    "plus_y",
    "minus_yaw",
    "plus_yaw",
    "mixed",
]
EXPECTED_LABELS = (
    ["configured_zero"]
    + [f"transition_{index:03d}" for index in range(1, EXPECTED_TRANSITION_SEGMENTS + 1)]
    + [f"correction_{label}" for label in EXPECTED_CORRECTIONS]
)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_bool(text: str) -> bool:
    if text == "true":
        return True
    if text == "false":
        return False
    raise ValueError(f"invalid boolean {text!r}")


def parse_collision_results(path: Path) -> tuple[dict, list[dict]]:
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
    parser.add_argument("--search-summary", required=True)
    parser.add_argument("--states-tsv", required=True)
    parser.add_argument("--collision-results", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--controller-yaml", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    search_path = Path(args.search_summary)
    states_path = Path(args.states_tsv)
    collision_path = Path(args.collision_results)
    urdf_path = Path(args.urdf)
    srdf_path = Path(args.srdf)
    controller_path = Path(args.controller_yaml)
    output_path = Path(args.output)
    manifest_path = Path(args.manifest)

    errors: list[str] = []
    try:
        search = json.loads(search_path.read_text(encoding="utf-8"))
        metadata, states = parse_collision_results(collision_path)
        state_rows = [line for line in states_path.read_text(encoding="utf-8").splitlines() if line]
        state_labels = [line.split("\t", 1)[0] for line in state_rows]

        if search.get("search_passed") is not True:
            errors.append("deterministic pregrasp search did not pass")
        if search.get("source_ref") != args.source_ref:
            errors.append("search summary is not bound to exact head")
        if search.get("phase") != "RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT":
            errors.append("unexpected search phase")
        coverage = search.get("search_coverage", {})
        if coverage.get("method") != "exhaustive_structured_subspace_grid":
            errors.append("unexpected search method")
        if int(coverage.get("candidate_count", 0)) != 28561:
            errors.append("structured grid candidate count mismatch")
        if int(coverage.get("finite_condition_count", 0)) <= 0:
            errors.append("search found no finite-condition candidate")
        if int(coverage.get("correction_feasible_count", 0)) <= 0:
            errors.append("search found no correction-feasible candidate")
        if coverage.get("globally_nearest_claimed") is not False:
            errors.append("search must not claim global nearest posture")

        candidate = search.get("selected_candidate", {})
        if float(candidate.get("jacobian_condition_number", 1e9)) > 250.0:
            errors.append("selected candidate exceeds condition limit")
        if candidate.get("effective_limits_with_margin") is not True:
            errors.append("selected candidate violates effective limits")
        if candidate.get("pre_fixture_posture_only") is not True:
            errors.append("candidate boundary is not pre-fixture-only")
        if float(candidate.get("maximum_absolute_joint_displacement_rad", 1e9)) > 0.3:
            errors.append("selected candidate is outside bounded search displacement")

        corrections = search.get("correction_cases", [])
        if [case.get("label") for case in corrections] != EXPECTED_CORRECTIONS:
            errors.append("correction case order mismatch")
        for case in corrections:
            if case.get("numerically_feasible") is not True:
                errors.append(f"{case.get('label')} correction is not numerically feasible")
            if case.get("effective_position_limits_with_margin") is not True:
                errors.append(f"{case.get('label')} correction violates effective limits")
            if case.get("command_authorized") is not False:
                errors.append(f"{case.get('label')} correction authorized a command")

        if state_labels != EXPECTED_LABELS:
            errors.append("generated state matrix labels mismatch")
        if [state.get("label") for state in states] != EXPECTED_LABELS:
            errors.append("collision result state labels mismatch")
        if len(states) != 107:
            errors.append("unexpected collision state count")
        for state in states:
            if state.get("bounds_ok") is not True:
                errors.append(f"{state.get('label')} violates MoveIt bounds")
            if state.get("self_collision") is not False:
                errors.append(f"{state.get('label')} is self-colliding")
            if state.get("contact_pair_count") != 0 or state.get("contact_pairs") != []:
                errors.append(f"{state.get('label')} has collision contacts")

        if metadata.get("robot_name") != "turtlebot3_lime":
            errors.append("production robot name was not used")
        if metadata.get("group_name") != "arm":
            errors.append("MoveIt arm group was not used")
        if metadata.get("active_joint_order") != [f"joint{index}" for index in range(1, 7)]:
            errors.append("MoveIt active joint order mismatch")
        if metadata.get("missing_collision_geometry") != []:
            errors.append("required collision geometry is missing")
        if metadata.get("adjacent_acm_entry_present") is not True or metadata.get("adjacent_acm_entry_allowed") is not True:
            errors.append("SRDF allowed-collision matrix was not applied")

        environment = search.get("environment", {})
        if environment.get("cube_or_support_added") is not False:
            errors.append("cube/support must not be added in this phase")
        if environment.get("environment_collision_checked") is not False:
            errors.append("environment collision must remain unchecked")
        if environment.get("verified_lateral_m") != 0.0 or environment.get("verified_yaw_rad") != 0.0:
            errors.append("environment collision envelope must remain zero")

        summary = {
            "schema_version": 1,
            "phase": "RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT",
            "writer_lease": "WL-RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT-20260803-01",
            "source_ref": args.source_ref,
            "passed": not errors,
            "errors": errors,
            "search": search,
            "collision_model": metadata,
            "collision_matrix": {
                "state_count": len(states),
                "configured_state_checked": "configured_zero" in state_labels,
                "transition_sample_count": sum(label.startswith("transition_") for label in state_labels),
                "candidate_checked": "transition_100" in state_labels,
                "correction_endpoint_count": sum(label.startswith("correction_") for label in state_labels),
                "all_bounds_ok": all(state["bounds_ok"] for state in states),
                "all_self_collision_free": all(not state["self_collision"] for state in states),
                "maximum_contact_pair_count": max((state["contact_pair_count"] for state in states), default=0),
            },
            "decision": "PREGRASP_CANDIDATE_READY_FOR_NEW_SIMULATION_FIXTURE_EVIDENCE" if not errors else "REJECT",
            "verified_envelope": {
                "zero_to_candidate_self_collision_checked": not errors,
                "candidate_correction_endpoints_self_collision_checked": not errors,
                "environment_collision_lateral_m": 0.0,
                "environment_collision_yaw_rad": 0.0,
                "environment_collision_status": "unknown_no_world_objects_added",
            },
            "next_required_phase": "RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE",
            "rollback": {
                "behavior": "deterministic_no_op",
                "on_search_failure": "reject",
                "on_self_collision": "reject",
                "on_environment_state_unknown": "reject",
            },
            "safety": {
                "offline_only": True,
                "globally_nearest_claimed": False,
                "ros_node_created": False,
                "controller_loaded": False,
                "planning_request_created": False,
                "trajectory_instantiated": False,
                "environment_collision_checked": False,
                "arm_command_sent": False,
                "gripper_command_sent": False,
                "gazebo_motion_performed": False,
                "actuation_authorized": False,
                "production_runtime_modified": False,
            },
        }
    except Exception as error:
        summary = {
            "schema_version": 1,
            "phase": "RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT",
            "writer_lease": "WL-RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT-20260803-01",
            "source_ref": args.source_ref,
            "passed": False,
            "errors": [f"{type(error).__name__}: {error}"],
            "decision": "REJECT",
            "safety": {
                "offline_only": True,
                "environment_collision_checked": False,
                "arm_command_sent": False,
                "gripper_command_sent": False,
                "gazebo_motion_performed": False,
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
            "search_summary": digest(search_path),
            "states_tsv": digest(states_path),
            "collision_results": digest(collision_path),
            "urdf": digest(urdf_path),
            "srdf": digest(srdf_path),
            "controller_yaml": digest(controller_path),
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
