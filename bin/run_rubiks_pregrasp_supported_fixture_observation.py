#!/usr/bin/env python3
"""Reconstruct the supported Rubik fixture after measured pregrasp alignment."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from gazebo_msgs.msg import LinkStates, ModelStates
from sensor_msgs.msg import JointState

from run_gripper_gauge_calibration import speed
from run_rubiks_dynamic_supported_hold import DynamicSupportedCubeNode, movement_metrics
from run_rubiks_static_cube_contact import quaternion_angle_rad

ARM_JOINTS = [f"joint{index}" for index in range(1, 7)]


def vector_norm(values: list[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def joint_error(target: list[float], actual: list[float]) -> list[float]:
    return [float(expected) - float(observed) for expected, observed in zip(target, actual)]


class SupportedFixtureObservationNode(DynamicSupportedCubeNode):
    def __init__(self, *, link7_name: str, **kwargs) -> None:
        self.link7_name = link7_name
        self.arm_samples: list[dict] = []
        self.latest_arm: dict | None = None
        self.support_samples: list[dict] = []
        self.latest_support_state: dict | None = None
        self.link7_samples: list[dict] = []
        self.latest_link7_state: dict | None = None
        self.model_name_samples: list[dict] = []
        self.gripper_goal_count = 0
        self.fixture_spawn_time_s: float | None = None
        super().__init__(**kwargs)

    def on_joint_state(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        if all(name in values for name in ARM_JOINTS):
            position = [float(values[name]) for name in ARM_JOINTS]
            velocity = (
                [float(velocities[name]) for name in ARM_JOINTS]
                if all(name in velocities for name in ARM_JOINTS)
                else None
            )
            if all(math.isfinite(value) for value in position) and (
                velocity is None or all(math.isfinite(value) for value in velocity)
            ):
                sample = {
                    "t_s": self.now_s(),
                    "phase": self.phase,
                    "position_rad": position,
                    "velocity_rad_s": velocity,
                    "after_fixture_spawn": bool(
                        self.fixture_spawn_time_s is not None
                        and self.now_s() >= self.fixture_spawn_time_s
                    ),
                }
                self.latest_arm = sample
                self.arm_samples.append(sample)
        super().on_joint_state(msg)

    def on_link_states(self, msg: LinkStates) -> None:
        super().on_link_states(msg)
        try:
            index = msg.name.index(self.link7_name)
        except ValueError:
            return
        pose = msg.pose[index]
        twist = msg.twist[index]
        sample = {
            "t_s": self.now_s(),
            "phase": self.phase,
            "position_m": [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ],
            "orientation_xyzw": [
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ],
            "linear_velocity_m_s": [
                float(twist.linear.x),
                float(twist.linear.y),
                float(twist.linear.z),
            ],
            "angular_velocity_rad_s": [
                float(twist.angular.x),
                float(twist.angular.y),
                float(twist.angular.z),
            ],
            "after_fixture_spawn": bool(
                self.fixture_spawn_time_s is not None
                and self.now_s() >= self.fixture_spawn_time_s
            ),
        }
        if all(
            math.isfinite(value)
            for value in (
                sample["position_m"]
                + sample["orientation_xyzw"]
                + sample["linear_velocity_m_s"]
                + sample["angular_velocity_rad_s"]
            )
        ):
            self.latest_link7_state = sample
            self.link7_samples.append(sample)

    def on_model_states(self, msg: ModelStates) -> None:
        super().on_model_states(msg)
        t_s = self.now_s()
        support = self.model_state(msg, self.support_model, t_s)
        if support is not None:
            self.latest_support_state = support
            self.support_samples.append(support)
        self.model_name_samples.append(
            {
                "t_s": t_s,
                "phase": self.phase,
                "model_names": [str(name) for name in msg.name],
                "after_fixture_spawn": bool(
                    self.fixture_spawn_time_s is not None
                    and t_s >= self.fixture_spawn_time_s
                ),
            }
        )

    def send_open_goal(self, *, target: float, max_effort: float, timeout: float) -> dict:
        self.gripper_goal_count += 1
        return self.send_goal(
            label="open_before_supported_fixture",
            target=target,
            max_effort=max_effort,
            timeout=timeout,
            monitor_contact=False,
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--search-summary", required=True)
    parser.add_argument("--arm-telemetry", required=True)
    parser.add_argument("--cube-sdf", required=True)
    parser.add_argument("--support-sdf", required=True)
    parser.add_argument("--reference-frame", default="turtlebot3_lime_gripper_test::link7")
    parser.add_argument("--cube-x", type=float, default=-0.019)
    parser.add_argument("--cube-y", type=float, default=0.0)
    parser.add_argument("--cube-z", type=float, default=0.1192)
    parser.add_argument("--support-x", type=float, default=-0.019)
    parser.add_argument("--support-y", type=float, default=0.0)
    parser.add_argument("--support-z", type=float, default=0.0857)
    parser.add_argument("--open-target", type=float, default=0.019)
    parser.add_argument("--open-effort", type=float, default=0.5)
    parser.add_argument("--pre-spawn-hold-seconds", type=float, default=1.0)
    parser.add_argument("--settle-seconds", type=float, default=2.5)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    search = json.loads(Path(args.search_summary).read_text(encoding="utf-8"))
    arm_telemetry = json.loads(Path(args.arm_telemetry).read_text(encoding="utf-8"))
    target_map = search["selected_candidate"]["joint_positions_rad"]
    target = [float(target_map[name]) for name in ARM_JOINTS]

    robot_model = "turtlebot3_lime_gripper_test"
    cube_model = "rubiks_dynamic_cube_supported_057"
    support_model = "rubiks_cube_support_surface"
    cube_link = f"{cube_model}::cube_link"
    left_link = f"{robot_model}::gripper_left_link"
    right_link = f"{robot_model}::gripper_right_link"

    errors: list[str] = []
    open_event: dict | None = None
    support_spawn: dict | None = None
    cube_spawn: dict | None = None
    arm_before_fixture: dict | None = None
    arm_after_fixture: dict | None = None
    link7_before_fixture: dict | None = None
    link7_after_fixture: dict | None = None
    cube_initial: dict | None = None
    cube_final: dict | None = None
    support_initial: dict | None = None
    support_final: dict | None = None
    fixture_observation_started_s: float | None = None
    fixture_observation_ended_s: float | None = None

    rclpy.init(args=None)
    node = SupportedFixtureObservationNode(
        link7_name=args.reference_frame,
        support_model=support_model,
        action_name="/gripper_controller/gripper_cmd",
        left_joint="gripper_left_joint",
        right_joint="gripper_right_joint",
        robot_model=robot_model,
        gauge_model=cube_model,
        left_link=left_link,
        right_link=right_link,
        gauge_link=cube_link,
        contact_topic="/rubiks_dynamic_cube_hold/contact_states",
    )
    try:
        if arm_telemetry.get("passed") is not True:
            raise RuntimeError("parent arm simulation-state telemetry did not pass")
        if [float(value) for value in arm_telemetry.get("commanded_target_rad", [])] != target:
            raise RuntimeError("parent arm target differs from deterministic candidate")

        node.wait_for(lambda: node.latest_arm is not None, args.timeout, "candidate arm state")
        node.wait_for(lambda: node.latest_link7_state is not None, args.timeout, "candidate link7 state")
        node.wait_for(lambda: node.latest_joint is not None, args.timeout, "gripper joint state")
        node.wait_for(lambda: node.latest_link is not None, args.timeout, "gripper link state")
        if not node.client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError("gripper action server was not available")

        node.phase = "verify_candidate_before_fixture"
        node.spin_for(args.pre_spawn_hold_seconds)
        arm_before_fixture = dict(node.latest_arm or {})
        link7_before_fixture = dict(node.latest_link7_state or {})
        if not arm_before_fixture or not link7_before_fixture:
            raise RuntimeError("candidate arm/link7 telemetry was not observed")
        before_error = joint_error(target, arm_before_fixture["position_rad"])
        if max(abs(value) for value in before_error) > 0.01:
            raise RuntimeError(f"arm was not at accepted candidate: {before_error}")

        node.phase = "open_gripper_before_fixture"
        open_event = node.send_open_goal(
            target=args.open_target,
            max_effort=args.open_effort,
            timeout=args.timeout,
        )
        if not open_event.get("reached_goal") or open_event.get("stalled"):
            raise RuntimeError(f"gripper open command failed: {open_event}")
        node.phase = "open_gripper_settling"
        node.spin_for(0.5)
        if node.gripper_goal_count != 1:
            raise RuntimeError(f"expected one gripper command, got {node.gripper_goal_count}")

        node.phase = "spawn_supported_fixture"
        node.command_target_m = None
        support_spawn = node.spawn_named_model(
            model_name=support_model,
            sdf_text=Path(args.support_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_supported_fixture",
            reference_frame=args.reference_frame,
            pose_xyz=(args.support_x, args.support_y, args.support_z),
            timeout=args.timeout,
        )
        cube_spawn = node.spawn_named_model(
            model_name=cube_model,
            sdf_text=Path(args.cube_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_dynamic_cube_hold",
            reference_frame=args.reference_frame,
            pose_xyz=(args.cube_x, args.cube_y, args.cube_z),
            timeout=args.timeout,
        )
        node.fixture_spawn_time_s = node.now_s()
        fixture_observation_started_s = node.fixture_spawn_time_s
        node.wait_for(lambda: node.latest_support_state is not None, args.timeout, "support model state")
        node.wait_for(lambda: node.latest_gauge_state is not None, args.timeout, "cube model state")
        support_initial = dict(node.latest_support_state or {})
        cube_initial = dict(node.latest_gauge_state or {})

        node.phase = "fixture_settling_no_commands"
        node.spin_for(args.settle_seconds)
        fixture_observation_ended_s = node.now_s()
        support_final = dict(node.latest_support_state or {})
        cube_final = dict(node.latest_gauge_state or {})
        arm_after_fixture = dict(node.latest_arm or {})
        link7_after_fixture = dict(node.latest_link7_state or {})

        if node.gripper_goal_count != 1:
            errors.append("a gripper command was sent after fixture spawn")
        after_error = joint_error(target, arm_after_fixture["position_rad"])
        if max(abs(value) for value in after_error) > 0.01:
            errors.append(f"arm drifted from candidate after fixture spawn: {after_error}")

        post_spawn_arm = [
            sample for sample in node.arm_samples if sample.get("after_fixture_spawn")
        ]
        post_spawn_link7 = [
            sample for sample in node.link7_samples if sample.get("after_fixture_spawn")
        ]
        post_spawn_cube = [
            sample
            for sample in node.gauge_samples
            if float(sample.get("t_s", -1.0)) >= float(node.fixture_spawn_time_s)
        ]
        post_spawn_support = [
            sample
            for sample in node.support_samples
            if float(sample.get("t_s", -1.0)) >= float(node.fixture_spawn_time_s)
        ]
        post_spawn_gripper = [
            sample
            for sample in node.samples
            if float(sample.get("t_s", -1.0)) >= float(node.fixture_spawn_time_s)
        ]

        if len(post_spawn_arm) < 20:
            errors.append("insufficient post-spawn arm samples")
        if len(post_spawn_link7) < 20:
            errors.append("insufficient post-spawn link7 samples")
        if len(post_spawn_cube) < 20:
            errors.append("insufficient post-spawn cube samples")
        if len(post_spawn_support) < 20:
            errors.append("insufficient post-spawn support samples")

        support_contact_samples = [
            sample for sample in post_spawn_gripper if sample.get("support_contact_active")
        ]
        support_contact_ratio = (
            len(support_contact_samples) / len(post_spawn_gripper)
            if post_spawn_gripper
            else 0.0
        )
        if not node.support_contact_messages or support_contact_ratio < 0.70:
            errors.append(
                f"support contact was insufficient: messages={len(node.support_contact_messages)} ratio={support_contact_ratio}"
            )
        if node.contact_messages:
            finger_contacts = [
                event
                for event in node.contact_messages
                if event.get("left_contact") or event.get("right_contact")
            ]
        else:
            finger_contacts = []
        if finger_contacts:
            errors.append("cube contacted a finger while the gripper remained open")
        if node.unexpected_robot_contacts:
            errors.append(
                f"cube contacted non-finger robot collisions: {node.unexpected_robot_contacts}"
            )

        movement = movement_metrics(post_spawn_cube, post_spawn_cube[0]) if post_spawn_cube else {}
        maximum_cube_linear_speed = max(
            (speed(sample) or 0.0 for sample in post_spawn_cube),
            default=math.inf,
        )
        maximum_cube_angular_speed = max(
            (
                vector_norm(sample.get("angular_velocity_rad_s", []))
                for sample in post_spawn_cube
            ),
            default=math.inf,
        )
        final_cube_linear_speed = speed(cube_final)
        final_cube_angular_speed = (
            vector_norm(cube_final.get("angular_velocity_rad_s", []))
            if cube_final is not None
            else None
        )
        if final_cube_linear_speed is None or final_cube_linear_speed > 0.02:
            errors.append(f"cube final linear speed exceeded 0.02 m/s: {final_cube_linear_speed}")
        if final_cube_angular_speed is None or final_cube_angular_speed > 0.2:
            errors.append(f"cube final angular speed exceeded 0.2 rad/s: {final_cube_angular_speed}")
        if float(movement.get("max_horizontal_displacement_m", math.inf)) > 0.003:
            errors.append(f"cube horizontal drift exceeded 3 mm: {movement}")
        if float(movement.get("max_rotation_rad", math.inf)) > 0.10:
            errors.append(f"cube rotation exceeded 0.10 rad: {movement}")

        link7_translation = math.dist(
            link7_before_fixture["position_m"], link7_after_fixture["position_m"]
        )
        link7_rotation = quaternion_angle_rad(
            link7_before_fixture["orientation_xyzw"],
            link7_after_fixture["orientation_xyzw"],
        )
        if link7_translation > 0.002:
            errors.append(f"link7 moved more than 2 mm after fixture spawn: {link7_translation}")
        if link7_rotation > 0.01:
            errors.append(f"link7 rotated more than 0.01 rad after fixture spawn: {link7_rotation}")

        observed_model_names = sorted(
            {
                name
                for sample in node.model_name_samples
                for name in sample.get("model_names", [])
            }
        )
        for required_model in (robot_model, cube_model, support_model):
            if required_model not in observed_model_names:
                errors.append(f"required model was not observed: {required_model}")

        observation_duration = (
            float(fixture_observation_ended_s) - float(fixture_observation_started_s)
            if fixture_observation_started_s is not None
            and fixture_observation_ended_s is not None
            else 0.0
        )
        if observation_duration < 2.0:
            errors.append(f"fixture observation was shorter than 2.0 s: {observation_duration}")

    except Exception as error:
        errors.append(f"{type(error).__name__}: {error}")
    finally:
        observed_model_names = sorted(
            {
                name
                for sample in node.model_name_samples
                for name in sample.get("model_names", [])
            }
        )
        post_spawn_gripper = [
            sample
            for sample in node.samples
            if node.fixture_spawn_time_s is not None
            and float(sample.get("t_s", -1.0)) >= float(node.fixture_spawn_time_s)
        ]
        support_contact_ratio = (
            sum(bool(sample.get("support_contact_active")) for sample in post_spawn_gripper)
            / len(post_spawn_gripper)
            if post_spawn_gripper
            else 0.0
        )
        post_spawn_cube = [
            sample
            for sample in node.gauge_samples
            if node.fixture_spawn_time_s is not None
            and float(sample.get("t_s", -1.0)) >= float(node.fixture_spawn_time_s)
        ]
        movement = movement_metrics(post_spawn_cube, post_spawn_cube[0]) if post_spawn_cube else None
        final_cube_linear_speed = speed(cube_final)
        final_cube_angular_speed = (
            vector_norm(cube_final.get("angular_velocity_rad_s", []))
            if cube_final is not None
            else None
        )
        arm_before_error = (
            joint_error(target, arm_before_fixture["position_rad"])
            if arm_before_fixture is not None
            else None
        )
        arm_after_error = (
            joint_error(target, arm_after_fixture["position_rad"])
            if arm_after_fixture is not None
            else None
        )
        link7_drift = (
            {
                "translation_m": math.dist(
                    link7_before_fixture["position_m"],
                    link7_after_fixture["position_m"],
                ),
                "rotation_rad": quaternion_angle_rad(
                    link7_before_fixture["orientation_xyzw"],
                    link7_after_fixture["orientation_xyzw"],
                ),
            }
            if link7_before_fixture is not None and link7_after_fixture is not None
            else None
        )
        telemetry = {
            "schema_version": 1,
            "trial_type": "gazebo_pregrasp_supported_fixture_observation",
            "writer_lease": "WL-RUBIK-PREGRASP-SUPPORTED-FIXTURE-OBSERVATION-20260803-01",
            "passed": not errors,
            "errors": errors,
            "simulation_only": True,
            "candidate_joint_names": ARM_JOINTS,
            "candidate_target_rad": target,
            "parent_arm_telemetry_passed": arm_telemetry.get("passed"),
            "arm_before_fixture": arm_before_fixture,
            "arm_before_fixture_error_rad": arm_before_error,
            "arm_after_fixture": arm_after_fixture,
            "arm_after_fixture_error_rad": arm_after_error,
            "arm_sample_count": len(node.arm_samples),
            "post_fixture_arm_command_count": 0,
            "gripper_goal_count": node.gripper_goal_count,
            "open_event": open_event,
            "post_fixture_gripper_command_count": 0,
            "fixture_spawn_time_s": node.fixture_spawn_time_s,
            "fixture_observation_started_s": fixture_observation_started_s,
            "fixture_observation_ended_s": fixture_observation_ended_s,
            "fixture_observation_duration_s": (
                float(fixture_observation_ended_s) - float(fixture_observation_started_s)
                if fixture_observation_started_s is not None
                and fixture_observation_ended_s is not None
                else None
            ),
            "reference_frame": args.reference_frame,
            "support_spawn": support_spawn,
            "cube_spawn": cube_spawn,
            "support_pose_xyz_m": [args.support_x, args.support_y, args.support_z],
            "cube_pose_xyz_m": [args.cube_x, args.cube_y, args.cube_z],
            "support_initial_state": support_initial,
            "support_final_state": support_final,
            "cube_initial_state": cube_initial,
            "cube_final_state": cube_final,
            "cube_movement": movement,
            "cube_final_linear_speed_m_s": final_cube_linear_speed,
            "cube_final_angular_speed_rad_s": final_cube_angular_speed,
            "support_contact_message_count": len(node.support_contact_messages),
            "support_contact_ratio": support_contact_ratio,
            "finger_contact_message_count": len(
                [
                    event
                    for event in node.contact_messages
                    if event.get("left_contact") or event.get("right_contact")
                ]
            ),
            "unexpected_robot_contact_count": len(node.unexpected_robot_contacts),
            "unexpected_robot_contacts": node.unexpected_robot_contacts,
            "link7_before_fixture": link7_before_fixture,
            "link7_after_fixture": link7_after_fixture,
            "link7_post_fixture_drift": link7_drift,
            "observed_model_names": observed_model_names,
            "arm_samples": node.arm_samples,
            "gripper_samples": node.samples,
            "link7_samples": node.link7_samples,
            "cube_samples": node.gauge_samples,
            "support_samples": node.support_samples,
            "contact_messages": node.contact_messages,
            "support_contact_messages": node.support_contact_messages,
            "gripper_close_command_sent": False,
            "lift_command_sent": False,
            "support_removed": False,
            "ifra_attachment_used": False,
            "grasp_success_claimed": False,
            "environment_collision_checked": False,
            "physical_hardware_ready_claimed": False,
            "production_runtime_modified": False,
            "actuation_authorized": False,
        }
        output_path.write_text(
            json.dumps(telemetry, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps(telemetry, indent=2, sort_keys=True))
    return 0 if telemetry["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
