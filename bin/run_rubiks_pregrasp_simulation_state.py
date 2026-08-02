#!/usr/bin/env python3
"""Run one bounded empty-scene Gazebo arm move to the accepted pregrasp state."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Callable

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import LinkStates, ModelStates
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from rubiks_kinematic_preflight import forward_jacobian, model

JOINT_NAMES = [f"joint{index}" for index in range(1, 7)]
SUCCESSFUL = int(FollowJointTrajectory.Result.SUCCESSFUL)


def finite_vector(values: object, expected: int) -> list[float] | None:
    try:
        result = [float(value) for value in values]  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if len(result) != expected or not all(math.isfinite(value) for value in result):
        return None
    return result


def duration_message(seconds: float) -> Duration:
    if seconds < 0.0 or not math.isfinite(seconds):
        raise ValueError(f"invalid duration {seconds}")
    whole = int(seconds)
    nanos = int(round((seconds - whole) * 1_000_000_000))
    if nanos >= 1_000_000_000:
        whole += 1
        nanos -= 1_000_000_000
    message = Duration()
    message.sec = whole
    message.nanosec = nanos
    return message


def quaternion_to_rotation(values: list[float]) -> np.ndarray:
    x, y, z, w = values
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=float,
    )


def pose_transform(position: list[float], quaternion: list[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = quaternion_to_rotation(quaternion)
    result[:3, 3] = np.array(position, dtype=float)
    return result


def rotation_angle(rotation: np.ndarray) -> float:
    cosine = max(-1.0, min(1.0, float((np.trace(rotation) - 1.0) / 2.0)))
    return math.acos(cosine)


def relative_pose(first: dict, second: dict) -> dict:
    first_tf = pose_transform(first["position_m"], first["orientation_xyzw"])
    second_tf = pose_transform(second["position_m"], second["orientation_xyzw"])
    relative = np.linalg.inv(first_tf) @ second_tf
    return {
        "translation_m": relative[:3, 3].tolist(),
        "translation_norm_m": float(np.linalg.norm(relative[:3, 3])),
        "rotation_matrix": relative[:3, :3].tolist(),
        "rotation_angle_rad": rotation_angle(relative[:3, :3]),
    }


def maximum_sample_speed(samples: list[dict]) -> float | None:
    direct = []
    for sample in samples:
        velocity = sample.get("velocity_rad_s")
        if isinstance(velocity, list) and len(velocity) == 6:
            direct.extend(abs(float(value)) for value in velocity if math.isfinite(float(value)))
    if direct:
        return max(direct)

    estimated = []
    for previous, current in zip(samples, samples[1:]):
        dt = float(current["t_s"]) - float(previous["t_s"])
        if dt <= 1e-6:
            continue
        for before, after in zip(previous["position_rad"], current["position_rad"]):
            estimated.append(abs((float(after) - float(before)) / dt))
    return max(estimated) if estimated else None


class PregraspSimulationNode(Node):
    def __init__(self, *, action_name: str, robot_model: str) -> None:
        super().__init__("rubiks_pregrasp_simulation_state_evidence")
        self.started = time.monotonic()
        self.phase = "initializing"
        self.action_client = ActionClient(self, FollowJointTrajectory, action_name)
        self.robot_model = robot_model
        self.link7_name = f"{robot_model}::link7"
        self.joint_samples: list[dict] = []
        self.controller_samples: list[dict] = []
        self.link7_samples: list[dict] = []
        self.model_samples: list[dict] = []
        self.latest_joint: dict | None = None
        self.latest_controller: dict | None = None
        self.latest_link7: dict | None = None
        self.latest_models: dict | None = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 100)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/controller_state",
            self.on_controller_state,
            100,
        )
        self.create_subscription(LinkStates, "/link_states", self.on_link_states, 100)
        self.create_subscription(ModelStates, "/model_states", self.on_model_states, 50)

    def now_s(self) -> float:
        return time.monotonic() - self.started

    def on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in JOINT_NAMES):
            return
        velocities = dict(zip(message.name, message.velocity)) if message.velocity else {}
        position = [float(positions[name]) for name in JOINT_NAMES]
        velocity = (
            [float(velocities[name]) for name in JOINT_NAMES]
            if all(name in velocities for name in JOINT_NAMES)
            else None
        )
        if not all(math.isfinite(value) for value in position):
            return
        if velocity is not None and not all(math.isfinite(value) for value in velocity):
            velocity = None
        sample = {
            "t_s": self.now_s(),
            "phase": self.phase,
            "position_rad": position,
            "velocity_rad_s": velocity,
        }
        self.latest_joint = sample
        self.joint_samples.append(sample)

    def on_controller_state(self, message: JointTrajectoryControllerState) -> None:
        if list(message.joint_names) != JOINT_NAMES:
            return
        desired = finite_vector(message.desired.positions, 6)
        actual = finite_vector(message.actual.positions, 6)
        error = finite_vector(message.error.positions, 6)
        actual_velocity = finite_vector(message.actual.velocities, 6)
        sample = {
            "t_s": self.now_s(),
            "phase": self.phase,
            "desired_position_rad": desired,
            "actual_position_rad": actual,
            "error_position_rad": error,
            "actual_velocity_rad_s": actual_velocity,
        }
        self.latest_controller = sample
        self.controller_samples.append(sample)

    def on_link_states(self, message: LinkStates) -> None:
        try:
            index = message.name.index(self.link7_name)
        except ValueError:
            return
        pose = message.pose[index]
        position = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
        orientation = [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ]
        if not all(math.isfinite(value) for value in position + orientation):
            return
        sample = {
            "t_s": self.now_s(),
            "phase": self.phase,
            "position_m": position,
            "orientation_xyzw": orientation,
        }
        self.latest_link7 = sample
        self.link7_samples.append(sample)

    def on_model_states(self, message: ModelStates) -> None:
        sample = {
            "t_s": self.now_s(),
            "phase": self.phase,
            "model_names": [str(name) for name in message.name],
        }
        self.latest_models = sample
        self.model_samples.append(sample)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, predicate: Callable[[], bool], timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"timed out waiting for {description}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--search-summary", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--action-name", default="/arm_controller/follow_joint_trajectory")
    parser.add_argument("--robot-model", default="turtlebot3_lime_gripper_test")
    parser.add_argument("--trajectory-seconds", type=float, default=3.0)
    parser.add_argument("--settle-seconds", type=float, default=1.5)
    parser.add_argument("--stable-hold-seconds", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    search = json.loads(Path(args.search_summary).read_text(encoding="utf-8"))
    candidate_map = search["selected_candidate"]["joint_positions_rad"]
    target = [float(candidate_map[name]) for name in JOINT_NAMES]

    chain, _ = model(Path(args.urdf))
    zero = np.zeros(6, dtype=float)
    zero_frame, _ = forward_jacobian(chain, zero)
    target_frame, _ = forward_jacobian(chain, np.array(target, dtype=float))
    expected_relative = np.linalg.inv(zero_frame) @ target_frame
    expected_pose = {
        "translation_m": expected_relative[:3, 3].tolist(),
        "translation_norm_m": float(np.linalg.norm(expected_relative[:3, 3])),
        "rotation_matrix": expected_relative[:3, :3].tolist(),
        "rotation_angle_rad": rotation_angle(expected_relative[:3, :3]),
    }

    errors: list[str] = []
    goal_sent_count = 0
    goal_accepted = False
    action_status: int | None = None
    action_error_code: int | None = None
    action_error_string: str | None = None
    initial_joint: dict | None = None
    final_joint: dict | None = None
    initial_link7: dict | None = None
    final_link7: dict | None = None
    command_start_s: float | None = None
    action_complete_s: float | None = None
    observed_pose: dict | None = None
    pose_comparison: dict | None = None

    rclpy.init(args=None)
    node = PregraspSimulationNode(action_name=args.action_name, robot_model=args.robot_model)
    try:
        if not node.action_client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError("arm FollowJointTrajectory action server was not available")
        node.wait_for(lambda: node.latest_joint is not None, args.timeout, "initial arm joint state")
        node.wait_for(lambda: node.latest_link7 is not None, args.timeout, "initial link7 pose")
        node.wait_for(lambda: node.latest_controller is not None, args.timeout, "initial controller state")
        node.wait_for(lambda: node.latest_models is not None, args.timeout, "initial model list")

        node.phase = "initial_hold"
        node.spin_for(1.0)
        initial_joint = dict(node.latest_joint or {})
        initial_link7 = dict(node.latest_link7 or {})
        if not initial_joint or not initial_link7:
            raise RuntimeError("initial telemetry disappeared")

        initial_positions = [float(value) for value in initial_joint["position_rad"]]
        if max(abs(value) for value in initial_positions) > 0.01:
            errors.append(f"initial arm state was not near configured zero: {initial_positions}")

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = list(JOINT_NAMES)
        start_point = JointTrajectoryPoint()
        start_point.positions = initial_positions
        start_point.velocities = [0.0] * 6
        start_point.time_from_start = duration_message(0.5)
        target_point = JointTrajectoryPoint()
        target_point.positions = list(target)
        target_point.velocities = [0.0] * 6
        target_point.time_from_start = duration_message(args.trajectory_seconds)
        trajectory_goal.trajectory.points = [start_point, target_point]
        trajectory_goal.goal_time_tolerance = duration_message(2.0)

        node.phase = "trajectory"
        command_start_s = node.now_s()
        goal_sent_count += 1
        goal_future = node.action_client.send_goal_async(trajectory_goal)
        node.wait_for(lambda: goal_future.done(), args.timeout, "trajectory goal response")
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("arm trajectory goal was rejected")
        goal_accepted = True

        result_future = goal_handle.get_result_async()
        node.wait_for(
            lambda: result_future.done(),
            args.trajectory_seconds + args.timeout,
            "trajectory result",
        )
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("trajectory action returned no result")
        action_complete_s = node.now_s()
        action_status = int(wrapped_result.status)
        action_error_code = int(wrapped_result.result.error_code)
        action_error_string = str(wrapped_result.result.error_string)
        if action_status != int(GoalStatus.STATUS_SUCCEEDED):
            errors.append(f"trajectory action status was {action_status}")
        if action_error_code != SUCCESSFUL:
            errors.append(
                f"trajectory result error {action_error_code}: {action_error_string}"
            )

        node.phase = "settling"
        settle_start_s = node.now_s()
        node.spin_for(args.settle_seconds)
        final_joint = dict(node.latest_joint or {})
        final_link7 = dict(node.latest_link7 or {})
        if not final_joint or not final_link7:
            raise RuntimeError("final telemetry was not observed")

        final_positions = [float(value) for value in final_joint["position_rad"]]
        final_error = [target_value - actual for target_value, actual in zip(target, final_positions)]
        max_final_error = max(abs(value) for value in final_error)
        if max_final_error > 0.01:
            errors.append(f"final joint error exceeded 0.01 rad: {final_error}")

        settling_samples = [
            sample
            for sample in node.joint_samples
            if float(sample["t_s"]) >= settle_start_s + max(0.0, args.settle_seconds - args.stable_hold_seconds)
        ]
        settling_speed = maximum_sample_speed(settling_samples)
        settling_duration = (
            float(settling_samples[-1]["t_s"]) - float(settling_samples[0]["t_s"])
            if len(settling_samples) >= 2
            else 0.0
        )
        if settling_duration < args.stable_hold_seconds - 0.1:
            errors.append(f"stable hold interval was too short: {settling_duration}")
        if settling_speed is None or settling_speed > 0.01:
            errors.append(f"settling arm speed exceeded 0.01 rad/s: {settling_speed}")

        observed_pose = relative_pose(initial_link7, final_link7)
        observed_rotation = np.array(observed_pose["rotation_matrix"], dtype=float)
        expected_rotation = expected_relative[:3, :3]
        pose_comparison = {
            "translation_error_vector_m": (
                np.array(observed_pose["translation_m"], dtype=float)
                - expected_relative[:3, 3]
            ).tolist(),
            "translation_error_norm_m": float(
                np.linalg.norm(
                    np.array(observed_pose["translation_m"], dtype=float)
                    - expected_relative[:3, 3]
                )
            ),
            "rotation_error_rad": rotation_angle(expected_rotation.T @ observed_rotation),
        }
        if float(pose_comparison["translation_error_norm_m"]) > 0.005:
            errors.append(f"link7 translation error exceeded 5 mm: {pose_comparison}")
        if float(pose_comparison["rotation_error_rad"]) > 0.03:
            errors.append(f"link7 rotation error exceeded 0.03 rad: {pose_comparison}")

        all_model_names = sorted(
            {
                name
                for sample in node.model_samples
                for name in sample.get("model_names", [])
            }
        )
        forbidden_models = [
            name
            for name in all_model_names
            if "rubiks" in name.lower() or "support" in name.lower()
        ]
        if forbidden_models:
            errors.append(f"unexpected cube/support model observed: {forbidden_models}")

        if goal_sent_count != 1:
            errors.append(f"expected exactly one arm goal, got {goal_sent_count}")
        if len(node.joint_samples) < 30:
            errors.append("insufficient arm joint-state samples")
        if len(node.link7_samples) < 20:
            errors.append("insufficient link7 samples")
        joint_times = [float(sample["t_s"]) for sample in node.joint_samples]
        if any(after <= before for before, after in zip(joint_times, joint_times[1:])):
            errors.append("joint-state sample time was not strictly monotonic")

    except Exception as error:
        errors.append(f"{type(error).__name__}: {error}")
    finally:
        all_model_names = sorted(
            {
                name
                for sample in node.model_samples
                for name in sample.get("model_names", [])
            }
        )
        final_positions = (
            [float(value) for value in final_joint["position_rad"]]
            if final_joint is not None
            else None
        )
        final_error = (
            [target_value - actual for target_value, actual in zip(target, final_positions)]
            if final_positions is not None
            else None
        )
        telemetry = {
            "schema_version": 1,
            "trial_type": "gazebo_empty_scene_pregrasp_state_alignment",
            "writer_lease": "WL-RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE-20260803-01",
            "passed": not errors,
            "errors": errors,
            "simulation_only": True,
            "physical_hardware_used": False,
            "robot_model": args.robot_model,
            "action_name": args.action_name,
            "joint_names": JOINT_NAMES,
            "commanded_target_rad": target,
            "goal_sent_count": goal_sent_count,
            "goal_accepted": goal_accepted,
            "action_status": action_status,
            "action_error_code": action_error_code,
            "action_error_string": action_error_string,
            "command_start_s": command_start_s,
            "action_complete_s": action_complete_s,
            "trajectory_seconds": args.trajectory_seconds,
            "settle_seconds": args.settle_seconds,
            "stable_hold_seconds": args.stable_hold_seconds,
            "initial_joint_state": initial_joint,
            "final_joint_state": final_joint,
            "final_joint_error_rad": final_error,
            "maximum_final_joint_error_rad": (
                max(abs(value) for value in final_error) if final_error is not None else None
            ),
            "joint_samples": node.joint_samples,
            "controller_samples": node.controller_samples,
            "initial_link7_pose": initial_link7,
            "final_link7_pose": final_link7,
            "link7_samples": node.link7_samples,
            "expected_link7_relative_pose": expected_pose,
            "observed_link7_relative_pose": observed_pose,
            "link7_pose_comparison": pose_comparison,
            "observed_model_names": all_model_names,
            "cube_or_support_spawned": False,
            "gripper_controller_loaded": False,
            "gripper_command_sent": False,
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
