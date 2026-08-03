#!/usr/bin/env python3
"""Extract and audit every recorded empty-scene pregrasp joint-state sample."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import yaml

JOINT_ORDER = [f"joint{i}" for i in range(1, 7)]
EXPECTED_CANDIDATE = [0.0, 0.0, math.radians(10.0), 0.0, math.radians(-5.0), 0.0]
PATH_MAX_ABS_RESIDUAL_RAD = 0.02
PATH_MAX_L2_RESIDUAL_RAD = math.sqrt(6.0) * PATH_MAX_ABS_RESIDUAL_RAD
PATH_MAX_ALPHA_EXCURSION = 1e-4
FALSE_SAFETY_FIELDS = (
    "physical_hardware_used",
    "cube_spawned",
    "support_spawned",
    "fixture_spawned",
    "ifra_attachment_used",
    "grasp_fix_plugin_loaded",
    "gripper_controller_loaded",
    "gripper_command_sent",
    "base_controller_loaded",
    "base_command_sent",
    "lift_command_sent",
    "grasp_success_claimed",
    "environment_clearance_claimed",
    "hardware_readiness_claimed",
    "production_runtime_modified",
)


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_float(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} is not numeric: {value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} is not finite: {value!r}")
    return result


def finite_vector(value: Any, length: int, field: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{field} must contain {length} values")
    return [finite_float(item, f"{field}[{index}]") for index, item in enumerate(value)]


def candidate_matches(candidate: list[float]) -> bool:
    return all(
        math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
        for actual, expected in zip(candidate, EXPECTED_CANDIDATE)
    )


def extract(args: argparse.Namespace) -> int:
    summary_path = Path(args.simulation_summary)
    output_path = Path(args.states_tsv)
    metadata_path = Path(args.metadata)
    summary = load_json(summary_path)
    errors: list[str] = []

    if summary.get("passed") is not True or summary.get("errors") != []:
        errors.append("source simulation summary did not pass")
    if summary.get("source_ref") != args.source_ref:
        errors.append("source simulation summary is not exact-head bound")
    if summary.get("joint_order") != JOINT_ORDER:
        errors.append("source simulation joint order mismatch")

    try:
        candidate = finite_vector(
            summary.get("candidate_joint_positions_rad"), len(JOINT_ORDER), "candidate"
        )
        if not candidate_matches(candidate):
            errors.append("source simulation candidate mismatch")
    except ValueError as exc:
        candidate = []
        errors.append(str(exc))

    evidence = summary.get("evidence", {})
    samples = evidence.get("joint_state_samples", []) if isinstance(evidence, dict) else []
    declared_count = evidence.get("joint_state_sample_count") if isinstance(evidence, dict) else None
    if not isinstance(samples, list) or not samples:
        errors.append("joint-state sample trace is empty")
        samples = []
    if declared_count != len(samples):
        errors.append(
            f"joint-state sample count mismatch: declared={declared_count}, actual={len(samples)}"
        )

    rows: list[tuple[str, list[float]]] = []
    wall_times: list[float] = []
    ros_stamps: list[int] = []
    velocity_complete_count = 0
    for index, sample in enumerate(samples):
        label = f"sample_{index:06d}"
        if not isinstance(sample, dict):
            errors.append(f"{label} is not an object")
            continue
        try:
            positions = finite_vector(sample.get("positions_rad"), len(JOINT_ORDER), f"{label}.positions")
            wall_time = finite_float(sample.get("wall_time_s"), f"{label}.wall_time_s")
            ros_stamp = int(sample.get("ros_stamp_ns"))
            if ros_stamp < 0:
                raise ValueError(f"{label}.ros_stamp_ns is negative")
            velocities = sample.get("velocities_rad_s")
            if velocities is not None:
                finite_vector(velocities, len(JOINT_ORDER), f"{label}.velocities")
                velocity_complete_count += 1
        except (TypeError, ValueError) as exc:
            errors.append(str(exc))
            continue
        rows.append((label, positions))
        wall_times.append(wall_time)
        ros_stamps.append(ros_stamp)

    if len(rows) != len(samples):
        errors.append(f"only {len(rows)} of {len(samples)} samples were valid")
    if len(rows) < 100:
        errors.append("fewer than 100 valid dynamic samples")
    if any(current < previous for previous, current in zip(wall_times, wall_times[1:])):
        errors.append("wall-time sequence is not monotonic")
    if any(current < previous for previous, current in zip(ros_stamps, ros_stamps[1:])):
        errors.append("ROS timestamp sequence is not monotonic")

    safety = summary.get("safety", {})
    if not isinstance(safety, dict):
        safety = {}
        errors.append("safety evidence is not an object")
    if safety.get("simulation_only") is not True:
        errors.append("simulation-only flag missing")
    if safety.get("empty_scene_only") is not True:
        errors.append("empty-scene flag missing")
    if safety.get("candidate_arm_goal_sent") is not True:
        errors.append("candidate goal evidence missing")
    if safety.get("return_to_zero_goal_sent") is not True:
        errors.append("return goal evidence missing")
    for field in FALSE_SAFETY_FIELDS:
        if safety.get(field) is not False:
            errors.append(f"{field} must remain false")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        for label, positions in rows:
            stream.write(label + "\t" + "\t".join(f"{value:.17g}" for value in positions) + "\n")

    metadata = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-DYNAMIC-PATH-AUDIT-EXTRACTION",
        "source_ref": args.source_ref,
        "passed": not errors,
        "errors": errors,
        "joint_order": JOINT_ORDER,
        "candidate_joint_positions_rad": candidate,
        "declared_sample_count": declared_count,
        "extracted_sample_count": len(rows),
        "velocity_complete_sample_count": velocity_complete_count,
        "wall_time_monotonic": not any(
            current < previous for previous, current in zip(wall_times, wall_times[1:])
        ),
        "ros_stamp_monotonic": not any(
            current < previous for previous, current in zip(ros_stamps, ros_stamps[1:])
        ),
        "first_wall_time_s": wall_times[0] if wall_times else None,
        "last_wall_time_s": wall_times[-1] if wall_times else None,
        "first_ros_stamp_ns": ros_stamps[0] if ros_stamps else None,
        "last_ros_stamp_ns": ros_stamps[-1] if ros_stamps else None,
        "states_tsv": output_path.name,
        "states_tsv_sha256": digest(output_path),
        "source_summary_sha256": digest(summary_path),
        "command_added": False,
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return 0 if not errors else 1


def parse_collision_results(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metadata: dict[str, Any] | None = None
    states: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line:
            continue
        fields = line.split("\t")
        if fields[0] == "META":
            if metadata is not None or len(fields) != 7:
                raise ValueError(f"invalid META line {line_number}")
            metadata = {
                "robot_model": fields[1],
                "group": fields[2],
                "active_joints": fields[3].split(",") if fields[3] else [],
                "missing_collision_geometry": fields[4].split(",") if fields[4] else [],
                "adjacent_entry_present": fields[5] == "true",
                "adjacent_allowed": fields[6] == "true",
            }
        elif fields[0] == "STATE":
            if len(fields) != 6:
                raise ValueError(f"invalid STATE line {line_number}")
            states.append(
                {
                    "label": fields[1],
                    "bounds_ok": fields[2] == "true",
                    "self_collision": fields[3] == "true",
                    "contact_pair_count": int(fields[4]),
                    "contact_pairs": fields[5].split(",") if fields[5] else [],
                }
            )
        else:
            raise ValueError(f"unknown collision row at line {line_number}: {fields[0]}")
    if metadata is None:
        raise ValueError("collision results lack META row")
    return metadata, states


def effective_limits(urdf_path: Path) -> dict[str, tuple[float, float]]:
    root = ET.parse(urdf_path).getroot()
    urdf_limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in JOINT_ORDER:
            continue
        limit = joint.find("limit")
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            raise ValueError(f"{name} lacks finite URDF position limits")
        urdf_limits[name] = (float(limit.get("lower")), float(limit.get("upper")))

    control_limits: dict[str, tuple[float, float]] = {}
    for control in root.findall("ros2_control"):
        for joint in control.findall("joint"):
            name = joint.get("name")
            if name not in JOINT_ORDER:
                continue
            for interface in joint.findall("command_interface"):
                if interface.get("name") != "position":
                    continue
                params = {
                    param.get("name"): (param.text or "").strip()
                    for param in interface.findall("param")
                }
                if "min" in params and "max" in params:
                    control_limits[name] = (float(params["min"]), float(params["max"]))

    result: dict[str, tuple[float, float]] = {}
    for name in JOINT_ORDER:
        if name not in urdf_limits or name not in control_limits:
            raise ValueError(f"{name} lacks complete effective limits")
        result[name] = (
            max(urdf_limits[name][0], control_limits[name][0]),
            min(urdf_limits[name][1], control_limits[name][1]),
        )
        if result[name][0] >= result[name][1]:
            raise ValueError(f"{name} effective limit intersection is empty")
    return result


def controller_joint_order(path: Path) -> tuple[str, list[str]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    manager = data.get("controller_manager", {}).get("ros__parameters", {})
    matches: list[tuple[str, list[str]]] = []
    for name, declaration in manager.items():
        if not isinstance(declaration, dict):
            continue
        if "JointTrajectoryController" not in str(declaration.get("type", "")):
            continue
        params = data.get(name, {}).get("ros__parameters", {})
        matches.append((str(name), [str(item) for item in params.get("joints", [])]))
    exact = [item for item in matches if item[1] == JOINT_ORDER]
    if len(exact) != 1:
        raise ValueError(f"expected one exact arm trajectory controller; found={matches}")
    return exact[0]


def parse_states(path: Path) -> list[tuple[str, list[float]]]:
    rows: list[tuple[str, list[float]]] = []
    labels: set[str] = set()
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line:
            continue
        fields = line.split("\t")
        if len(fields) != 7:
            raise ValueError(f"invalid states TSV line {line_number}")
        label = fields[0]
        if not label or label in labels:
            raise ValueError(f"empty or duplicate state label at line {line_number}")
        labels.add(label)
        rows.append((label, [finite_float(float(value), f"line {line_number}") for value in fields[1:]]))
    if not rows:
        raise ValueError("states TSV is empty")
    return rows


def summarize(args: argparse.Namespace) -> int:
    summary_path = Path(args.simulation_summary)
    extraction_path = Path(args.extraction_metadata)
    states_path = Path(args.states_tsv)
    collision_path = Path(args.collision_results)
    urdf_path = Path(args.urdf)
    srdf_path = Path(args.srdf)
    controller_path = Path(args.controller_yaml)
    output_path = Path(args.output)
    manifest_path = Path(args.manifest)
    errors: list[str] = []

    simulation = load_json(summary_path)
    extraction = load_json(extraction_path)
    try:
        rows = parse_states(states_path)
        limits = effective_limits(urdf_path)
        controller_name, controller_joints = controller_joint_order(controller_path)
        collision_meta, collision_states = parse_collision_results(collision_path)
    except Exception as exc:
        errors.append(f"input parsing failed: {type(exc).__name__}: {exc}")
        rows, limits, controller_name, controller_joints = [], {}, "", []
        collision_meta, collision_states = {}, []

    if simulation.get("passed") is not True or simulation.get("errors") != []:
        errors.append("simulation source did not pass")
    if simulation.get("source_ref") != args.source_ref:
        errors.append("simulation source_ref mismatch")
    if extraction.get("passed") is not True or extraction.get("errors") != []:
        errors.append("sample extraction did not pass")
    if extraction.get("source_ref") != args.source_ref:
        errors.append("extraction source_ref mismatch")
    if extraction.get("extracted_sample_count") != len(rows):
        errors.append("extracted sample count mismatch")

    candidate: list[float] = []
    try:
        candidate = finite_vector(
            simulation.get("candidate_joint_positions_rad"), len(JOINT_ORDER), "candidate"
        )
        if not candidate_matches(candidate):
            errors.append("candidate does not match accepted pregrasp state")
    except ValueError as exc:
        errors.append(str(exc))

    candidate_norm_sq = sum(value * value for value in candidate) if candidate else 0.0
    max_abs_residual = 0.0
    max_l2_residual = 0.0
    min_raw_alpha = math.inf
    max_raw_alpha = -math.inf
    max_limit_violation = 0.0
    nearest_endpoint_counts = {"zero": 0, "interior": 0, "candidate": 0}
    if rows and candidate_norm_sq > 0 and limits:
        for label, positions in rows:
            raw_alpha = sum(position * target for position, target in zip(positions, candidate)) / candidate_norm_sq
            alpha = min(1.0, max(0.0, raw_alpha))
            residual = [position - alpha * target for position, target in zip(positions, candidate)]
            max_abs_residual = max(max_abs_residual, max(abs(value) for value in residual))
            max_l2_residual = max(max_l2_residual, math.sqrt(sum(value * value for value in residual)))
            min_raw_alpha = min(min_raw_alpha, raw_alpha)
            max_raw_alpha = max(max_raw_alpha, raw_alpha)
            if alpha <= 0.0:
                nearest_endpoint_counts["zero"] += 1
            elif alpha >= 1.0:
                nearest_endpoint_counts["candidate"] += 1
            else:
                nearest_endpoint_counts["interior"] += 1
            for name, position in zip(JOINT_ORDER, positions):
                lower, upper = limits[name]
                max_limit_violation = max(max_limit_violation, lower - position, position - upper, 0.0)

        if max_abs_residual > PATH_MAX_ABS_RESIDUAL_RAD:
            errors.append(
                f"dynamic path max absolute residual {max_abs_residual} exceeds {PATH_MAX_ABS_RESIDUAL_RAD}"
            )
        if max_l2_residual > PATH_MAX_L2_RESIDUAL_RAD:
            errors.append(
                f"dynamic path L2 residual {max_l2_residual} exceeds {PATH_MAX_L2_RESIDUAL_RAD}"
            )
        if min_raw_alpha < -PATH_MAX_ALPHA_EXCURSION:
            errors.append(f"dynamic path alpha undershoot {min_raw_alpha}")
        if max_raw_alpha > 1.0 + PATH_MAX_ALPHA_EXCURSION:
            errors.append(f"dynamic path alpha overshoot {max_raw_alpha}")
        if max_limit_violation > 0.0:
            errors.append(f"dynamic trace exceeded effective joint limits by {max_limit_violation}")

    expected_labels = [label for label, _ in rows]
    collision_labels = [state.get("label") for state in collision_states]
    if collision_labels != expected_labels:
        errors.append("collision result labels do not exactly match dynamic sample order")
    all_bounds_ok = bool(collision_states) and all(state.get("bounds_ok") is True for state in collision_states)
    all_self_collision_free = bool(collision_states) and all(
        state.get("self_collision") is False for state in collision_states
    )
    maximum_contact_pair_count = max(
        (int(state.get("contact_pair_count", 0)) for state in collision_states), default=-1
    )
    contact_pairs = sorted(
        {
            pair
            for state in collision_states
            for pair in state.get("contact_pairs", [])
            if pair
        }
    )
    if not all_bounds_ok:
        errors.append("one or more MoveIt dynamic states violated bounds")
    if not all_self_collision_free:
        errors.append("one or more MoveIt dynamic states self-collided")
    if maximum_contact_pair_count != 0 or contact_pairs:
        errors.append("MoveIt reported one or more dynamic self-collision contact pairs")

    if collision_meta.get("active_joints") != JOINT_ORDER:
        errors.append("MoveIt active joint order mismatch")
    if collision_meta.get("group") != "arm":
        errors.append("MoveIt arm group mismatch")
    if collision_meta.get("missing_collision_geometry") != []:
        errors.append("required collision geometry is missing")
    if collision_meta.get("adjacent_entry_present") is not True:
        errors.append("SRDF adjacent-collision entry is missing")
    if collision_meta.get("adjacent_allowed") is not True:
        errors.append("SRDF adjacent-collision entry is not allowed")
    if controller_joints != JOINT_ORDER:
        errors.append("controller joint order mismatch")

    safety = simulation.get("safety", {})
    if safety.get("simulation_only") is not True or safety.get("empty_scene_only") is not True:
        errors.append("source safety boundary is not empty-scene simulation-only")
    for field in FALSE_SAFETY_FIELDS:
        if safety.get(field) is not False:
            errors.append(f"{field} must remain false")

    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-DYNAMIC-PATH-AUDIT",
        "writer_lease": "WL-RUBIK-PREGRASP-DYNAMIC-PATH-AUDIT-20260803-01",
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "passed": not errors,
        "errors": errors,
        "joint_order": JOINT_ORDER,
        "controller": {
            "name": controller_name,
            "joint_order": controller_joints,
            "joint_order_exact": controller_joints == JOINT_ORDER,
        },
        "dynamic_trace": {
            "sample_count": len(rows),
            "candidate_joint_positions_rad": candidate,
            "max_abs_path_residual_rad": max_abs_residual,
            "max_l2_path_residual_rad": max_l2_residual,
            "minimum_raw_path_alpha": min_raw_alpha if rows else None,
            "maximum_raw_path_alpha": max_raw_alpha if rows else None,
            "max_effective_limit_violation_rad": max_limit_violation,
            "nearest_segment_region_counts": nearest_endpoint_counts,
            "path_tube_max_abs_limit_rad": PATH_MAX_ABS_RESIDUAL_RAD,
            "path_tube_max_l2_limit_rad": PATH_MAX_L2_RESIDUAL_RAD,
            "path_alpha_excursion_limit": PATH_MAX_ALPHA_EXCURSION,
            "all_finite": extraction.get("passed") is True,
            "wall_time_monotonic": extraction.get("wall_time_monotonic") is True,
            "ros_stamp_monotonic": extraction.get("ros_stamp_monotonic") is True,
        },
        "moveit_self_collision_audit": {
            "state_count": len(collision_states),
            "all_bounds_ok": all_bounds_ok,
            "all_self_collision_free": all_self_collision_free,
            "maximum_contact_pair_count": maximum_contact_pair_count,
            "contact_pairs": contact_pairs,
            "active_joint_order": collision_meta.get("active_joints", []),
            "missing_collision_geometry": collision_meta.get("missing_collision_geometry", []),
            "adjacent_collision_entry_present": collision_meta.get("adjacent_entry_present"),
            "adjacent_collision_allowed": collision_meta.get("adjacent_allowed"),
        },
        "verified_claim": "recorded_empty_scene_joint_trace_within_accepted_joint_path_tube_and_self_collision_free",
        "unverified_claims": [
            "populated_scene_environment_clearance",
            "object_clearance",
            "perception_accuracy",
            "grasp_success",
            "transport_success",
            "placement_success",
            "physical_hardware_readiness",
        ],
        "safety": {
            "same_two_arm_goals_only": True,
            "new_command_path_added": False,
            "simulation_only": True,
            "empty_scene_only": True,
            "physical_hardware_used": False,
            "cube_spawned": False,
            "support_spawned": False,
            "fixture_spawned": False,
            "gripper_command_sent": False,
            "base_command_sent": False,
            "lift_command_sent": False,
            "ifra_attachment_used": False,
            "grasp_success_claimed": False,
            "environment_clearance_claimed": False,
            "hardware_readiness_claimed": False,
            "production_runtime_modified": False,
        },
        "next_required_phase": "RUBIK-PREGRASP-GROUND-PLANE-CLEARANCE-AUDIT",
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    inputs = {}
    for path in (
        summary_path,
        extraction_path,
        states_path,
        collision_path,
        urdf_path,
        srdf_path,
        controller_path,
    ):
        inputs[path.name] = digest(path)
    manifest = {
        "schema_version": 1,
        "phase": summary["phase"],
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "summary": output_path.name,
        "summary_sha256": digest(output_path),
        "inputs": dict(sorted(inputs.items())),
        "dynamic_sample_count": len(rows),
        "moveit_checked_state_count": len(collision_states),
        "same_two_arm_goals_only": True,
        "new_command_path_added": False,
        "simulation_only": True,
        "empty_scene_only": True,
        "physical_hardware_used": False,
        "production_runtime_modified": False,
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if not errors else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    extract_parser = subparsers.add_parser("extract")
    extract_parser.add_argument("--simulation-summary", required=True)
    extract_parser.add_argument("--states-tsv", required=True)
    extract_parser.add_argument("--metadata", required=True)
    extract_parser.add_argument("--source-ref", required=True)
    extract_parser.set_defaults(func=extract)

    summarize_parser = subparsers.add_parser("summarize")
    summarize_parser.add_argument("--simulation-summary", required=True)
    summarize_parser.add_argument("--extraction-metadata", required=True)
    summarize_parser.add_argument("--states-tsv", required=True)
    summarize_parser.add_argument("--collision-results", required=True)
    summarize_parser.add_argument("--urdf", required=True)
    summarize_parser.add_argument("--srdf", required=True)
    summarize_parser.add_argument("--controller-yaml", required=True)
    summarize_parser.add_argument("--output", required=True)
    summarize_parser.add_argument("--manifest", required=True)
    summarize_parser.add_argument("--source-ref", required=True)
    summarize_parser.set_defaults(func=summarize)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
