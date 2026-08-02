#!/usr/bin/env python3
"""Summarize bounded empty-scene pregrasp simulation-state evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

JOINT_NAMES = [f"joint{index}" for index in range(1, 7)]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def arm_signature(summary: dict) -> list[dict]:
    chain = summary.get("arm_chain", {})
    result = []
    for joint in chain.get("joints", []):
        result.append(
            {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.get("parent"),
                "child": joint.get("child"),
                "xyz": joint.get("xyz"),
                "rpy": joint.get("rpy"),
                "axis": joint.get("axis"),
                "urdf_lower": joint.get("urdf_lower"),
                "urdf_upper": joint.get("urdf_upper"),
                "control_lower": joint.get("control_lower"),
                "control_upper": joint.get("control_upper"),
                "effective_lower": joint.get("effective_lower"),
                "effective_upper": joint.get("effective_upper"),
            }
        )
    return result


def close_vectors(first: list[float], second: list[float], tolerance: float = 1e-12) -> bool:
    return len(first) == len(second) and all(
        math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tolerance)
        for a, b in zip(first, second)
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--telemetry", required=True)
    parser.add_argument("--search-summary", required=True)
    parser.add_argument("--production-kinematic-summary", required=True)
    parser.add_argument("--test-kinematic-summary", required=True)
    parser.add_argument("--production-urdf", required=True)
    parser.add_argument("--test-urdf", required=True)
    parser.add_argument("--production-controller-yaml", required=True)
    parser.add_argument("--test-controller-yaml", required=True)
    parser.add_argument("--controllers-before", required=True)
    parser.add_argument("--controllers-after", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    telemetry_path = Path(args.telemetry)
    search_path = Path(args.search_summary)
    production_kinematic_path = Path(args.production_kinematic_summary)
    test_kinematic_path = Path(args.test_kinematic_summary)
    production_urdf_path = Path(args.production_urdf)
    test_urdf_path = Path(args.test_urdf)
    production_controller_path = Path(args.production_controller_yaml)
    test_controller_path = Path(args.test_controller_yaml)
    controllers_before_path = Path(args.controllers_before)
    controllers_after_path = Path(args.controllers_after)
    output_path = Path(args.output)
    manifest_path = Path(args.manifest)

    errors: list[str] = []
    try:
        telemetry = json.loads(telemetry_path.read_text(encoding="utf-8"))
        search = json.loads(search_path.read_text(encoding="utf-8"))
        production = json.loads(production_kinematic_path.read_text(encoding="utf-8"))
        test = json.loads(test_kinematic_path.read_text(encoding="utf-8"))
        controllers_before = controllers_before_path.read_text(encoding="utf-8")
        controllers_after = controllers_after_path.read_text(encoding="utf-8")

        if telemetry.get("passed") is not True or telemetry.get("errors") != []:
            errors.append(f"simulation telemetry failed: {telemetry.get('errors')}")
        if telemetry.get("trial_type") != "gazebo_empty_scene_pregrasp_state_alignment":
            errors.append("unexpected simulation trial type")
        if telemetry.get("simulation_only") is not True or telemetry.get("physical_hardware_used") is not False:
            errors.append("simulation/hardware boundary is invalid")
        if telemetry.get("joint_names") != JOINT_NAMES:
            errors.append("telemetry joint order mismatch")

        candidate = search.get("selected_candidate", {}).get("joint_positions_rad", {})
        expected_target = [float(candidate[name]) for name in JOINT_NAMES]
        commanded_target = [float(value) for value in telemetry.get("commanded_target_rad", [])]
        if not close_vectors(commanded_target, expected_target):
            errors.append("commanded target does not equal regenerated deterministic candidate")
        if telemetry.get("goal_sent_count") != 1 or telemetry.get("goal_accepted") is not True:
            errors.append("exactly one accepted arm goal was not observed")
        if telemetry.get("action_status") != 4 or telemetry.get("action_error_code") != 0:
            errors.append("trajectory action did not report successful completion")

        initial = telemetry.get("initial_joint_state") or {}
        final = telemetry.get("final_joint_state") or {}
        initial_positions = [float(value) for value in initial.get("position_rad", [])]
        final_positions = [float(value) for value in final.get("position_rad", [])]
        if len(initial_positions) != 6 or max(abs(value) for value in initial_positions) > 0.01:
            errors.append("initial measured arm state was not near configured zero")
        if len(final_positions) != 6:
            errors.append("final measured arm state is incomplete")
        if float(telemetry.get("maximum_final_joint_error_rad", 1e9)) > 0.01:
            errors.append("final joint tracking error exceeded 0.01 rad")

        joint_samples = telemetry.get("joint_samples", [])
        controller_samples = telemetry.get("controller_samples", [])
        link_samples = telemetry.get("link7_samples", [])
        if len(joint_samples) < 30:
            errors.append("insufficient joint-state samples")
        if len(controller_samples) < 20:
            errors.append("insufficient controller-state samples")
        if len(link_samples) < 20:
            errors.append("insufficient link7 samples")

        comparison = telemetry.get("link7_pose_comparison") or {}
        if float(comparison.get("translation_error_norm_m", 1e9)) > 0.005:
            errors.append("observed link7 translation differed from FK by more than 5 mm")
        if float(comparison.get("rotation_error_rad", 1e9)) > 0.03:
            errors.append("observed link7 orientation differed from FK by more than 0.03 rad")

        model_names = [str(name) for name in telemetry.get("observed_model_names", [])]
        forbidden_models = [
            name
            for name in model_names
            if "rubiks" in name.lower() or "support" in name.lower()
        ]
        if forbidden_models:
            errors.append(f"unexpected cube/support model observed: {forbidden_models}")
        if telemetry.get("cube_or_support_spawned") is not False:
            errors.append("cube/support spawn boundary changed")

        for key in (
            "gripper_controller_loaded",
            "gripper_command_sent",
            "lift_command_sent",
            "support_removed",
            "ifra_attachment_used",
            "grasp_success_claimed",
            "environment_collision_checked",
            "physical_hardware_ready_claimed",
            "production_runtime_modified",
            "actuation_authorized",
        ):
            if telemetry.get(key) is not False:
                errors.append(f"{key} must remain false")

        if production.get("passed") is not True or test.get("passed") is not True:
            errors.append("production/test kinematic extraction did not pass")
        if production.get("source_ref") != args.source_ref or test.get("source_ref") != args.source_ref:
            errors.append("production/test kinematic summaries are not exact-head bound")
        production_signature = arm_signature(production)
        test_signature = arm_signature(test)
        if production_signature != test_signature:
            errors.append("test-only and production arm chain/controller-limit signatures differ")
        if [entry.get("name") for entry in production_signature] != JOINT_NAMES:
            errors.append("production arm signature order mismatch")

        production_controller = production.get("controller", {})
        test_controller = test.get("controller", {})
        for controller in (production_controller, test_controller):
            if controller.get("joints") != JOINT_NAMES:
                errors.append("trajectory controller joint order mismatch")
            if controller.get("action_type") != "control_msgs/action/FollowJointTrajectory":
                errors.append("unexpected trajectory action schema")
            if controller.get("command_interfaces") != ["position"]:
                errors.append("trajectory controller command interface mismatch")
        if production_controller.get("action_name") != test_controller.get("action_name"):
            errors.append("production/test trajectory action names differ")

        if "arm_controller" not in controllers_before or "active" not in controllers_before:
            errors.append("active arm controller missing before trial")
        if "arm_controller" not in controllers_after or "active" not in controllers_after:
            errors.append("active arm controller missing after trial")
        if "gripper_controller" in controllers_before and "gripper_controller" in controllers_before.split("active"):
            pass
        if any(
            "gripper_controller" in line and "active" in line
            for line in controllers_before.splitlines() + controllers_after.splitlines()
        ):
            errors.append("gripper controller was active during trial")

        summary = {
            "schema_version": 1,
            "phase": "RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE",
            "writer_lease": "WL-RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE-20260803-01",
            "source_ref": args.source_ref,
            "passed": not errors,
            "errors": errors,
            "decision": "PREGRASP_SIMULATION_STATE_ESTABLISHED" if not errors else "REJECT",
            "candidate": search.get("selected_candidate"),
            "model_equivalence": {
                "production_robot_name": production.get("arm_chain", {}).get("base_link"),
                "test_robot_model": telemetry.get("robot_model"),
                "arm_signature_equal": production_signature == test_signature,
                "joint_order": JOINT_NAMES,
                "production_controller": production_controller,
                "test_controller": test_controller,
            },
            "tracking": {
                "commanded_target_rad": commanded_target,
                "initial_position_rad": initial_positions,
                "final_position_rad": final_positions,
                "final_error_rad": telemetry.get("final_joint_error_rad"),
                "maximum_final_error_rad": telemetry.get("maximum_final_joint_error_rad"),
                "goal_sent_count": telemetry.get("goal_sent_count"),
                "action_status": telemetry.get("action_status"),
                "action_error_code": telemetry.get("action_error_code"),
                "joint_sample_count": len(joint_samples),
                "controller_sample_count": len(controller_samples),
            },
            "link7": {
                "expected_relative_pose": telemetry.get("expected_link7_relative_pose"),
                "observed_relative_pose": telemetry.get("observed_link7_relative_pose"),
                "comparison": comparison,
                "sample_count": len(link_samples),
            },
            "scene": {
                "observed_model_names": model_names,
                "cube_or_support_spawned": False,
                "gripper_controller_loaded": False,
                "environment_collision_checked": False,
            },
            "next_required_phase": "RUBIK-PREGRASP-SUPPORTED-FIXTURE-OBSERVATION",
            "rollback": {
                "behavior": "stop_trial_and_fail_closed",
                "automatic_second_motion_sent": False,
                "rollback_motion_claimed": False,
            },
            "safety": {
                "simulation_only": True,
                "bounded_simulation_arm_command_sent": True,
                "physical_hardware_used": False,
                "physical_hardware_ready_claimed": False,
                "cube_or_support_spawned": False,
                "gripper_controller_loaded": False,
                "gripper_command_sent": False,
                "lift_command_sent": False,
                "support_removed": False,
                "ifra_attachment_used": False,
                "grasp_success_claimed": False,
                "environment_collision_checked": False,
                "production_runtime_modified": False,
                "actuation_authorized": False,
            },
        }
    except Exception as error:
        summary = {
            "schema_version": 1,
            "phase": "RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE",
            "writer_lease": "WL-RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE-20260803-01",
            "source_ref": args.source_ref,
            "passed": False,
            "errors": [f"{type(error).__name__}: {error}"],
            "decision": "REJECT",
            "safety": {
                "simulation_only": True,
                "physical_hardware_used": False,
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
            "telemetry": digest(telemetry_path),
            "search_summary": digest(search_path),
            "production_kinematic_summary": digest(production_kinematic_path),
            "test_kinematic_summary": digest(test_kinematic_path),
            "production_urdf": digest(production_urdf_path),
            "test_urdf": digest(test_urdf_path),
            "production_controller_yaml": digest(production_controller_path),
            "test_controller_yaml": digest(test_controller_path),
            "controllers_before": digest(controllers_before_path),
            "controllers_after": digest(controllers_after_path),
        },
        "simulation_only": True,
        "bounded_simulation_arm_command_sent": True,
        "physical_hardware_used": False,
        "actuation_authorized": False,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
