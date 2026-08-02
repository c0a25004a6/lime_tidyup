#!/usr/bin/env python3
"""Summarize candidate-aligned supported-fixture simulation observation evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

JOINT_NAMES = [f"joint{index}" for index in range(1, 7)]
EXPECTED_CUBE_POSE = [-0.019, 0.0, 0.1192]
EXPECTED_SUPPORT_POSE = [-0.019, 0.0, 0.0857]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def close_vector(first: list[float], second: list[float], tolerance: float = 1e-12) -> bool:
    return len(first) == len(second) and all(
        math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tolerance)
        for a, b in zip(first, second)
    )


def controller_active(text: str, name: str) -> bool:
    return any(name in line and "active" in line for line in text.splitlines())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-telemetry", required=True)
    parser.add_argument("--arm-telemetry", required=True)
    parser.add_argument("--search-summary", required=True)
    parser.add_argument("--controllers-before-arm", required=True)
    parser.add_argument("--controllers-before-fixture", required=True)
    parser.add_argument("--controllers-after-fixture", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--controller-yaml", required=True)
    parser.add_argument("--cube-sdf", required=True)
    parser.add_argument("--support-sdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    fixture_path = Path(args.fixture_telemetry)
    arm_path = Path(args.arm_telemetry)
    search_path = Path(args.search_summary)
    before_arm_path = Path(args.controllers_before_arm)
    before_fixture_path = Path(args.controllers_before_fixture)
    after_fixture_path = Path(args.controllers_after_fixture)
    urdf_path = Path(args.urdf)
    controller_path = Path(args.controller_yaml)
    cube_path = Path(args.cube_sdf)
    support_path = Path(args.support_sdf)
    output_path = Path(args.output)
    manifest_path = Path(args.manifest)

    errors: list[str] = []
    try:
        fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
        arm = json.loads(arm_path.read_text(encoding="utf-8"))
        search = json.loads(search_path.read_text(encoding="utf-8"))
        before_arm = before_arm_path.read_text(encoding="utf-8")
        before_fixture = before_fixture_path.read_text(encoding="utf-8")
        after_fixture = after_fixture_path.read_text(encoding="utf-8")

        if arm.get("passed") is not True or arm.get("errors") != []:
            errors.append(f"candidate arm alignment failed: {arm.get('errors')}")
        if fixture.get("passed") is not True or fixture.get("errors") != []:
            errors.append(f"supported fixture observation failed: {fixture.get('errors')}")
        if fixture.get("trial_type") != "gazebo_pregrasp_supported_fixture_observation":
            errors.append("unexpected fixture observation trial type")
        if fixture.get("simulation_only") is not True:
            errors.append("fixture observation must remain simulation-only")

        candidate_map = search.get("selected_candidate", {}).get("joint_positions_rad", {})
        expected_target = [float(candidate_map[name]) for name in JOINT_NAMES]
        if not close_vector([float(value) for value in fixture.get("candidate_target_rad", [])], expected_target):
            errors.append("fixture candidate target differs from deterministic search")
        if not close_vector([float(value) for value in arm.get("commanded_target_rad", [])], expected_target):
            errors.append("arm command target differs from deterministic search")
        if fixture.get("candidate_joint_names") != JOINT_NAMES:
            errors.append("fixture arm joint order mismatch")
        if fixture.get("parent_arm_telemetry_passed") is not True:
            errors.append("fixture did not bind passing parent arm telemetry")

        before_error = [float(value) for value in fixture.get("arm_before_fixture_error_rad", [])]
        after_error = [float(value) for value in fixture.get("arm_after_fixture_error_rad", [])]
        if len(before_error) != 6 or max(abs(value) for value in before_error) > 0.01:
            errors.append("arm was not at candidate before fixture spawn")
        if len(after_error) != 6 or max(abs(value) for value in after_error) > 0.01:
            errors.append("arm drifted from candidate after fixture spawn")
        if float(arm.get("maximum_final_joint_error_rad", 1e9)) > 0.01:
            errors.append("parent arm final tracking error exceeded 0.01 rad")

        if fixture.get("gripper_goal_count") != 1:
            errors.append("exactly one gripper-open command was not recorded")
        open_event = fixture.get("open_event") or {}
        if open_event.get("label") != "open_before_supported_fixture":
            errors.append("unexpected gripper command label")
        if open_event.get("reached_goal") is not True or open_event.get("stalled") is not False:
            errors.append("gripper did not reach the open target")
        if not math.isclose(float(open_event.get("target_position_m", -1.0)), 0.019, abs_tol=1e-12):
            errors.append("gripper open target was not 0.019 m")
        if fixture.get("post_fixture_arm_command_count") != 0:
            errors.append("arm command was recorded after fixture spawn")
        if fixture.get("post_fixture_gripper_command_count") != 0:
            errors.append("gripper command was recorded after fixture spawn")

        if fixture.get("support_spawn", {}).get("success") is not True:
            errors.append("support spawn did not succeed")
        if fixture.get("cube_spawn", {}).get("success") is not True:
            errors.append("cube spawn did not succeed")
        if not close_vector(fixture.get("support_pose_xyz_m", []), EXPECTED_SUPPORT_POSE):
            errors.append("support pose differs from accepted PR #7 transform")
        if not close_vector(fixture.get("cube_pose_xyz_m", []), EXPECTED_CUBE_POSE):
            errors.append("cube pose differs from accepted PR #7 transform")
        if fixture.get("reference_frame") != "turtlebot3_lime_gripper_test::link7":
            errors.append("fixture reference frame mismatch")
        if float(fixture.get("fixture_observation_duration_s", 0.0)) < 2.0:
            errors.append("fixture observation duration was below 2.0 s")

        if int(fixture.get("support_contact_message_count", 0)) <= 0:
            errors.append("support contact was not observed")
        if float(fixture.get("support_contact_ratio", 0.0)) < 0.70:
            errors.append("support contact ratio was below 0.70")
        if int(fixture.get("finger_contact_message_count", -1)) != 0:
            errors.append("cube contacted a finger while gripper was open")
        if int(fixture.get("unexpected_robot_contact_count", -1)) != 0:
            errors.append("cube contacted a non-finger robot collision")

        movement = fixture.get("cube_movement") or {}
        if float(movement.get("max_horizontal_displacement_m", 1e9)) > 0.003:
            errors.append("cube horizontal drift exceeded 3 mm")
        if float(movement.get("max_rotation_rad", 1e9)) > 0.10:
            errors.append("cube rotation exceeded 0.10 rad")
        if float(fixture.get("cube_final_linear_speed_m_s", 1e9)) > 0.02:
            errors.append("cube final linear speed exceeded 0.02 m/s")
        if float(fixture.get("cube_final_angular_speed_rad_s", 1e9)) > 0.2:
            errors.append("cube final angular speed exceeded 0.2 rad/s")

        link7_drift = fixture.get("link7_post_fixture_drift") or {}
        if float(link7_drift.get("translation_m", 1e9)) > 0.002:
            errors.append("link7 drift exceeded 2 mm after fixture spawn")
        if float(link7_drift.get("rotation_rad", 1e9)) > 0.01:
            errors.append("link7 rotation exceeded 0.01 rad after fixture spawn")

        models = [str(name) for name in fixture.get("observed_model_names", [])]
        for required in (
            "turtlebot3_lime_gripper_test",
            "rubiks_dynamic_cube_supported_057",
            "rubiks_cube_support_surface",
        ):
            if required not in models:
                errors.append(f"required model was not observed: {required}")

        for key in (
            "gripper_close_command_sent",
            "lift_command_sent",
            "support_removed",
            "ifra_attachment_used",
            "grasp_success_claimed",
            "environment_collision_checked",
            "physical_hardware_ready_claimed",
            "production_runtime_modified",
            "actuation_authorized",
        ):
            if fixture.get(key) is not False:
                errors.append(f"{key} must remain false")

        if not controller_active(before_arm, "arm_controller"):
            errors.append("arm controller was not active before arm alignment")
        if controller_active(before_arm, "gripper_controller"):
            errors.append("gripper controller was active before arm alignment")
        for snapshot_name, snapshot in (
            ("before_fixture", before_fixture),
            ("after_fixture", after_fixture),
        ):
            if not controller_active(snapshot, "arm_controller"):
                errors.append(f"arm controller was not active {snapshot_name}")
            if not controller_active(snapshot, "gripper_controller"):
                errors.append(f"gripper controller was not active {snapshot_name}")

        summary = {
            "schema_version": 1,
            "phase": "RUBIK-PREGRASP-SUPPORTED-FIXTURE-OBSERVATION",
            "writer_lease": "WL-RUBIK-PREGRASP-SUPPORTED-FIXTURE-OBSERVATION-20260803-01",
            "source_ref": args.source_ref,
            "passed": not errors,
            "errors": errors,
            "decision": "SUPPORTED_FIXTURE_STATE_ESTABLISHED" if not errors else "REJECT",
            "candidate": search.get("selected_candidate"),
            "setup_commands": {
                "arm_goal_count": arm.get("goal_sent_count"),
                "arm_action_status": arm.get("action_status"),
                "arm_action_error_code": arm.get("action_error_code"),
                "arm_maximum_final_error_rad": arm.get("maximum_final_joint_error_rad"),
                "gripper_goal_count": fixture.get("gripper_goal_count"),
                "gripper_open_event": open_event,
                "post_fixture_arm_command_count": fixture.get("post_fixture_arm_command_count"),
                "post_fixture_gripper_command_count": fixture.get("post_fixture_gripper_command_count"),
            },
            "measured_state": {
                "arm_before_fixture": fixture.get("arm_before_fixture"),
                "arm_before_fixture_error_rad": before_error,
                "arm_after_fixture": fixture.get("arm_after_fixture"),
                "arm_after_fixture_error_rad": after_error,
                "link7_before_fixture": fixture.get("link7_before_fixture"),
                "link7_after_fixture": fixture.get("link7_after_fixture"),
                "link7_post_fixture_drift": link7_drift,
                "open_gripper_left_joint_position_m": open_event.get("observed_left_joint_position_m"),
                "open_gripper_right_joint_position_m": open_event.get("observed_right_joint_position_m"),
                "open_gripper_link_separation_m": open_event.get("gazebo_link_frame_separation_m"),
            },
            "fixture": {
                "reference_frame": fixture.get("reference_frame"),
                "support_pose_xyz_m": fixture.get("support_pose_xyz_m"),
                "cube_pose_xyz_m": fixture.get("cube_pose_xyz_m"),
                "support_initial_state": fixture.get("support_initial_state"),
                "support_final_state": fixture.get("support_final_state"),
                "cube_initial_state": fixture.get("cube_initial_state"),
                "cube_final_state": fixture.get("cube_final_state"),
                "cube_movement": movement,
                "cube_final_linear_speed_m_s": fixture.get("cube_final_linear_speed_m_s"),
                "cube_final_angular_speed_rad_s": fixture.get("cube_final_angular_speed_rad_s"),
                "support_contact_message_count": fixture.get("support_contact_message_count"),
                "support_contact_ratio": fixture.get("support_contact_ratio"),
                "finger_contact_message_count": fixture.get("finger_contact_message_count"),
                "unexpected_robot_contact_count": fixture.get("unexpected_robot_contact_count"),
                "observation_duration_s": fixture.get("fixture_observation_duration_s"),
                "observed_model_names": models,
            },
            "next_required_phase": "RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE",
            "rollback": {
                "behavior": "stop_trial_and_fail_closed",
                "automatic_post_fixture_motion_sent": False,
                "rollback_motion_claimed": False,
            },
            "safety": {
                "simulation_only": True,
                "bounded_setup_arm_command_sent": True,
                "bounded_setup_gripper_open_command_sent": True,
                "post_fixture_arm_command_sent": False,
                "post_fixture_gripper_command_sent": False,
                "gripper_close_command_sent": False,
                "lift_command_sent": False,
                "support_removed": False,
                "ifra_attachment_used": False,
                "grasp_success_claimed": False,
                "environment_collision_checked": False,
                "physical_hardware_used": False,
                "physical_hardware_ready_claimed": False,
                "production_runtime_modified": False,
                "actuation_authorized": False,
            },
        }
    except Exception as error:
        summary = {
            "schema_version": 1,
            "phase": "RUBIK-PREGRASP-SUPPORTED-FIXTURE-OBSERVATION",
            "writer_lease": "WL-RUBIK-PREGRASP-SUPPORTED-FIXTURE-OBSERVATION-20260803-01",
            "source_ref": args.source_ref,
            "passed": False,
            "errors": [f"{type(error).__name__}: {error}"],
            "decision": "REJECT",
            "safety": {
                "simulation_only": True,
                "post_fixture_arm_command_sent": False,
                "post_fixture_gripper_command_sent": False,
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
            "fixture_telemetry": digest(fixture_path),
            "arm_telemetry": digest(arm_path),
            "search_summary": digest(search_path),
            "controllers_before_arm": digest(before_arm_path),
            "controllers_before_fixture": digest(before_fixture_path),
            "controllers_after_fixture": digest(after_fixture_path),
            "urdf": digest(urdf_path),
            "controller_yaml": digest(controller_path),
            "cube_sdf": digest(cube_path),
            "support_sdf": digest(support_path),
        },
        "simulation_only": True,
        "post_fixture_arm_command_sent": False,
        "post_fixture_gripper_command_sent": False,
        "environment_collision_checked": False,
        "actuation_authorized": False,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
