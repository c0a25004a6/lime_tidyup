#!/usr/bin/env python3
"""Aggregate exact-head empty-scene pregrasp arm-motion repeatability evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

EXPECTED_JOINT_ORDER = [f"joint{i}" for i in range(1, 7)]
EXPECTED_CANDIDATE = [0.0, 0.0, math.radians(10.0), 0.0, math.radians(-5.0), 0.0]
EXPECTED_MODELS = {"ground_plane", "turtlebot3_lime_arm_motion_test"}
EXPECTED_CONTROLLERS = {"joint_state_broadcaster", "arm_controller"}
TRIAL_LABELS = ("trial_01", "trial_02", "trial_03")
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


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not a JSON object")
    return value


def finite_number(value: Any, field: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError(f"{field} is not numeric: {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{field} is not finite: {value!r}")
    return number


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def controller_snapshot(path: Path) -> dict[str, str]:
    controllers: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) >= 3:
            controllers[fields[0]] = fields[-1]
    return controllers


def range_of(values: list[float]) -> float:
    return max(values) - min(values) if values else 0.0


def validate_trial(root: Path, label: str, source_ref: str) -> tuple[dict[str, Any], list[str], dict[str, str]]:
    trial_root = root / label
    errors: list[str] = []
    hashes: dict[str, str] = {}

    status_path = trial_root / "trial_exit_status.txt"
    summary_path = trial_root / "rubiks_pregrasp_simulation_state_summary.json"
    required_files = (
        status_path,
        summary_path,
        trial_root / "rubiks_pregrasp_alignment_search.json",
        trial_root / "rubiks_pregrasp_alignment_states.tsv",
        trial_root / "turtlebot3_lime_arm_motion_test.urdf",
        trial_root / "turtlebot3_lime_pregrasp_search.urdf",
        trial_root / "gazebo_controller_manager.production.yaml",
        trial_root / "pregrasp_controllers_before.txt",
        trial_root / "pregrasp_controllers_after.txt",
        trial_root / "pregrasp_world_models_before.json",
        trial_root / "pregrasp_world_models_after.json",
        trial_root / "pregrasp_actions_after.txt",
    )
    for path in required_files:
        if not path.is_file() or path.stat().st_size == 0:
            errors.append(f"{label}: missing evidence file {path.name}")
        else:
            hashes[str(path.relative_to(root))] = digest(path)

    if errors:
        return {"label": label, "passed": False, "errors": errors}, errors, hashes

    try:
        exit_status = int(status_path.read_text(encoding="utf-8").strip())
    except ValueError:
        exit_status = -1
        errors.append(f"{label}: invalid trial exit status")
    require(exit_status == 0, f"{label}: trial exit status was {exit_status}", errors)

    try:
        summary = load_json(summary_path)
    except Exception as exc:
        errors.append(f"{label}: invalid summary: {exc}")
        return {"label": label, "passed": False, "errors": errors}, errors, hashes

    require(summary.get("passed") is True, f"{label}: summary did not pass", errors)
    require(summary.get("errors") == [], f"{label}: summary contains errors", errors)
    require(summary.get("source_ref") == source_ref, f"{label}: source_ref mismatch", errors)
    require(summary.get("joint_order") == EXPECTED_JOINT_ORDER, f"{label}: joint order mismatch", errors)

    candidate = summary.get("candidate_joint_positions_rad", [])
    candidate_ok = isinstance(candidate, list) and len(candidate) == len(EXPECTED_CANDIDATE)
    if candidate_ok:
        try:
            candidate_ok = all(
                math.isclose(finite_number(actual, f"{label}.candidate"), expected, rel_tol=0.0, abs_tol=1e-12)
                for actual, expected in zip(candidate, EXPECTED_CANDIDATE)
            )
        except ValueError as exc:
            errors.append(str(exc))
            candidate_ok = False
    require(candidate_ok, f"{label}: selected candidate mismatch", errors)

    evidence = summary.get("evidence", {})
    if not isinstance(evidence, dict):
        evidence = {}
        errors.append(f"{label}: evidence is not an object")

    require(int(evidence.get("joint_state_sample_count", 0)) >= 100, f"{label}: insufficient joint-state samples", errors)
    require(int(evidence.get("feedback_sample_count", 0)) >= 10, f"{label}: insufficient feedback samples", errors)

    metrics: dict[str, float] = {}
    for phase in ("candidate", "return"):
        goal = evidence.get(f"{phase}_goal", {})
        feedback = evidence.get(f"{phase}_feedback", {})
        transition = evidence.get(f"{phase}_transition", {})
        hold = evidence.get(f"{phase}_hold", {})
        require(goal.get("accepted") is True, f"{label}: {phase} goal not accepted", errors)
        require(goal.get("succeeded") is True, f"{label}: {phase} goal did not succeed", errors)
        require(goal.get("status") == 4, f"{label}: {phase} goal status mismatch", errors)
        require(goal.get("error_code") == 0, f"{label}: {phase} controller error", errors)
        require(int(feedback.get("sample_count", 0)) >= 5, f"{label}: {phase} feedback count too low", errors)
        require(
            feedback.get("valid_joint_order_count") == feedback.get("sample_count"),
            f"{label}: {phase} feedback joint order mismatch",
            errors,
        )
        require(int(hold.get("sample_count", 0)) >= 20, f"{label}: {phase} hold count too low", errors)
        require(hold.get("all_finite") is True, f"{label}: {phase} hold is non-finite", errors)

        try:
            metrics[f"{phase}_goal_duration_s"] = finite_number(goal.get("duration_s"), f"{label}.{phase}.duration_s")
            metrics[f"{phase}_feedback_max_error_rad"] = finite_number(
                feedback.get("max_abs_position_error_rad"), f"{label}.{phase}.feedback_max_error"
            )
            metrics[f"{phase}_max_overshoot_rad"] = finite_number(
                transition.get("max_overshoot_rad"), f"{label}.{phase}.overshoot"
            )
            metrics[f"{phase}_max_nonmoving_drift_rad"] = finite_number(
                transition.get("max_nonmoving_drift_rad"), f"{label}.{phase}.nonmoving_drift"
            )
            metrics[f"{phase}_hold_final_error_rad"] = finite_number(
                hold.get("max_abs_final_error_rad"), f"{label}.{phase}.hold_final_error"
            )
            metrics[f"{phase}_hold_max_velocity_rad_s"] = finite_number(
                hold.get("max_abs_velocity_rad_s"), f"{label}.{phase}.hold_velocity"
            )
            metrics[f"{phase}_hold_position_range_rad"] = finite_number(
                hold.get("max_position_range_rad"), f"{label}.{phase}.hold_position_range"
            )
        except ValueError as exc:
            errors.append(str(exc))

        if metrics:
            require(metrics.get(f"{phase}_feedback_max_error_rad", math.inf) <= 0.08, f"{label}: {phase} feedback error exceeded", errors)
            require(metrics.get(f"{phase}_max_overshoot_rad", math.inf) <= 0.02, f"{label}: {phase} overshoot exceeded", errors)
            require(metrics.get(f"{phase}_max_nonmoving_drift_rad", math.inf) <= 0.02, f"{label}: {phase} nonmoving drift exceeded", errors)
            require(metrics.get(f"{phase}_hold_final_error_rad", math.inf) <= 0.01, f"{label}: {phase} final error exceeded", errors)
            require(metrics.get(f"{phase}_hold_max_velocity_rad_s", math.inf) <= 0.02, f"{label}: {phase} hold velocity exceeded", errors)
            require(metrics.get(f"{phase}_hold_position_range_rad", math.inf) <= 0.005, f"{label}: {phase} hold stability exceeded", errors)

    world_topics: list[str] = []
    for position in ("before", "after"):
        world = load_json(trial_root / f"pregrasp_world_models_{position}.json")
        require(world.get("status") == "ok", f"{label}: {position} world capture failed", errors)
        require(set(world.get("models", [])) == EXPECTED_MODELS, f"{label}: {position} world set mismatch", errors)
        require(world.get("exact_set_match") is True, f"{label}: {position} world exact-set flag missing", errors)
        require(world.get("forbidden_model_present") is False, f"{label}: {position} forbidden model present", errors)
        require(world.get("pose_count") == 2 and world.get("twist_count") == 2, f"{label}: {position} ModelStates dimensions mismatch", errors)
        discovery = world.get("discovery", {})
        require(discovery.get("api") == "rclpy_node_graph", f"{label}: {position} graph API mismatch", errors)
        require(discovery.get("matching_topic_count") == 1, f"{label}: {position} ModelStates topic ambiguity", errors)
        require(int(discovery.get("publisher_count", 0)) >= 1, f"{label}: {position} ModelStates publisher missing", errors)
        world_topics.append(str(world.get("source_topic", "")))
    require(world_topics[0] == world_topics[1] == "/model_states", f"{label}: ModelStates topic changed", errors)

    for position in ("before", "after"):
        controllers = controller_snapshot(trial_root / f"pregrasp_controllers_{position}.txt")
        require(set(controllers) == EXPECTED_CONTROLLERS, f"{label}: {position} controller set mismatch: {controllers}", errors)
        require(all(state == "active" for state in controllers.values()), f"{label}: {position} controller not active", errors)

    actions = (trial_root / "pregrasp_actions_after.txt").read_text(encoding="utf-8")
    require("/arm_controller/follow_joint_trajectory" in actions, f"{label}: arm action missing", errors)
    require("gripper" not in actions, f"{label}: gripper action appeared", errors)

    safety = summary.get("safety", {})
    require(safety.get("simulation_only") is True, f"{label}: simulation-only flag missing", errors)
    require(safety.get("empty_scene_only") is True, f"{label}: empty-scene flag missing", errors)
    require(safety.get("arm_controller_loaded") is True, f"{label}: arm-controller flag missing", errors)
    require(safety.get("candidate_arm_goal_sent") is True, f"{label}: candidate goal flag missing", errors)
    require(safety.get("return_to_zero_goal_sent") is True, f"{label}: return goal flag missing", errors)
    for field in FALSE_SAFETY_FIELDS:
        require(safety.get(field) is False, f"{label}: {field} must remain false", errors)

    result = {
        "label": label,
        "passed": not errors,
        "errors": errors,
        "source_ref": summary.get("source_ref"),
        "candidate_joint_positions_rad": candidate,
        "joint_state_sample_count": evidence.get("joint_state_sample_count"),
        "feedback_sample_count": evidence.get("feedback_sample_count"),
        "metrics": metrics,
        "model_states_topic": world_topics[0] if world_topics else None,
        "world_models": sorted(EXPECTED_MODELS),
        "active_controllers": sorted(EXPECTED_CONTROLLERS),
    }
    return result, errors, hashes


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    root = Path(args.root)
    output = Path(args.output)
    manifest_path = Path(args.manifest)
    all_errors: list[str] = []
    trials: list[dict[str, Any]] = []
    input_hashes: dict[str, str] = {}

    for label in TRIAL_LABELS:
        try:
            trial, errors, hashes = validate_trial(root, label, args.source_ref)
        except Exception as exc:
            errors = [f"{label}: unexpected aggregator error: {type(exc).__name__}: {exc}"]
            trial = {"label": label, "passed": False, "errors": errors}
            hashes = {}
        trials.append(trial)
        all_errors.extend(errors)
        input_hashes.update(hashes)

    aggregate_metrics: dict[str, dict[str, float]] = {}
    metric_names = sorted(
        {
            name
            for trial in trials
            for name in trial.get("metrics", {})
        }
    )
    for name in metric_names:
        values = [float(trial["metrics"][name]) for trial in trials if name in trial.get("metrics", {})]
        if len(values) == len(TRIAL_LABELS):
            aggregate_metrics[name] = {
                "minimum": min(values),
                "maximum": max(values),
                "range": range_of(values),
                "mean": sum(values) / len(values),
            }

    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-EMPTY-SCENE-REPEATABILITY",
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "trial_count": len(trials),
        "required_trial_count": len(TRIAL_LABELS),
        "fresh_container_per_trial": True,
        "passed": not all_errors and len(trials) == len(TRIAL_LABELS),
        "errors": all_errors,
        "trials": trials,
        "aggregate_metrics": aggregate_metrics,
        "candidate_joint_positions_rad": EXPECTED_CANDIDATE,
        "simulation_only": True,
        "empty_scene_only": True,
        "physical_hardware_used": False,
        "cube_spawned": False,
        "support_spawned": False,
        "gripper_command_sent": False,
        "base_command_sent": False,
        "lift_command_sent": False,
        "grasp_success_claimed": False,
        "hardware_readiness_claimed": False,
        "production_runtime_modified": False,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    manifest = {
        "schema_version": 1,
        "phase": summary["phase"],
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "summary": output.name,
        "summary_sha256": digest(output),
        "inputs": dict(sorted(input_hashes.items())),
        "trial_labels": list(TRIAL_LABELS),
        "trial_count": len(trials),
        "all_trials_passed": summary["passed"],
        "fresh_container_per_trial": True,
        "simulation_only": True,
        "empty_scene_only": True,
        "physical_hardware_used": False,
        "production_runtime_modified": False,
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
