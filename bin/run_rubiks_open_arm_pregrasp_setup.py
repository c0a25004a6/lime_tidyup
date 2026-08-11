#!/usr/bin/env python3
"""Open the gripper, move the arm to the exact accepted pregrasp anchor, and hold.

This setup runs before any cube/support is spawned in the dynamic close/hold
isolation gate. It never commands lift/base motion and never claims a grasp.

Gazebo's mimic right gripper joint is not guaranteed to appear in
``/joint_states``. Therefore the setup treats the commanded left joint plus the
physical left/right link separation from ``/link_states`` as the required open
state evidence. If the right mimic joint is published it is checked as an
additional consistency signal, but it is not required for the backend.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from gazebo_msgs.msg import LinkStates
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LEFT = "gripper_left_joint"
RIGHT = "gripper_right_joint"
ROBOT_MODEL = "turtlebot3_lime_gripper_test"
LEFT_LINK = f"{ROBOT_MODEL}::gripper_left_link"
RIGHT_LINK = f"{ROBOT_MODEL}::gripper_right_link"
OPEN_M = 0.019
MIN_OPEN_LINK_SEPARATION_M = 0.076


class SetupNode(Node):
    def __init__(self) -> None:
        super().__init__("rubiks_open_arm_pregrasp_setup")
        self.arm = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 50)
        self.create_subscription(LinkStates, "/link_states", self.on_link_states, 50)
        self.latest: dict[str, float] = {}
        self.latest_link_separation_m: float | None = None
        self.samples: list[dict[str, object]] = []
        self.started = time.monotonic()

    def now_s(self) -> float:
        return time.monotonic() - self.started

    def on_joint_state(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position) and math.isfinite(float(msg.position[index])):
                self.latest[name] = float(msg.position[index])
        self.samples.append({
            "t_s": self.now_s(),
            "arm": [self.latest.get(name) for name in ARM_JOINTS],
            "left_gripper_m": self.latest.get(LEFT),
            "right_gripper_m": self.latest.get(RIGHT),
            "right_gripper_joint_observed": RIGHT in self.latest,
            "physical_link_separation_m": self.latest_link_separation_m,
        })

    def on_link_states(self, msg: LinkStates) -> None:
        try:
            left_index = msg.name.index(LEFT_LINK)
            right_index = msg.name.index(RIGHT_LINK)
        except ValueError:
            return
        left = msg.pose[left_index].position
        right = msg.pose[right_index].position
        separation = math.dist(
            (float(left.x), float(left.y), float(left.z)),
            (float(right.x), float(right.y), float(right.z)),
        )
        if math.isfinite(separation):
            self.latest_link_separation_m = separation

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(
                self, timeout_sec=min(0.05, max(0.0, end - time.monotonic()))
            )

    def wait_for_initial_state(self, timeout: float) -> None:
        end = time.monotonic() + timeout
        needed = set(ARM_JOINTS + [LEFT])
        while time.monotonic() < end:
            if needed.issubset(self.latest) and self.latest_link_separation_m is not None:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        missing = sorted(needed - set(self.latest))
        if self.latest_link_separation_m is None:
            missing.append("physical_gripper_link_separation")
        raise RuntimeError(f"initial state missing: {missing}")

    def wait_future(self, future, timeout: float):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError("action future timed out")
        return future.result()

    def validate_open_state(self, label: str, tolerance: float) -> dict[str, object]:
        left = self.latest.get(LEFT)
        right = self.latest.get(RIGHT)
        separation = self.latest_link_separation_m
        if left is None:
            raise RuntimeError(f"{label}: left gripper joint state missing")
        if abs(left - OPEN_M) > tolerance:
            raise RuntimeError(f"{label}: left gripper not fully open: {left}")
        if right is not None and abs(right - OPEN_M) > tolerance:
            raise RuntimeError(f"{label}: observed right mimic joint inconsistent: {right}")
        if separation is None:
            raise RuntimeError(f"{label}: physical finger-link separation missing")
        if separation < MIN_OPEN_LINK_SEPARATION_M:
            raise RuntimeError(
                f"{label}: physical finger-link separation too small: {separation}"
            )
        return {
            "measured_left_m": left,
            "measured_right_m": right,
            "right_joint_observed": right is not None,
            "physical_link_separation_m": separation,
            "minimum_accepted_link_separation_m": MIN_OPEN_LINK_SEPARATION_M,
        }

    def open_gripper(self, timeout: float) -> dict[str, object]:
        if not self.gripper.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("gripper action server unavailable")
        goal = GripperCommand.Goal()
        goal.command.position = OPEN_M
        goal.command.max_effort = 0.5
        handle = self.wait_future(self.gripper.send_goal_async(goal), timeout)
        if handle is None or not handle.accepted:
            raise RuntimeError("open-gripper goal rejected")
        wrapped = self.wait_future(handle.get_result_async(), timeout)
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"open-gripper action failed: status={getattr(wrapped, 'status', None)}"
            )
        result = wrapped.result
        self.spin_for(0.4)
        measured = self.validate_open_state("open-gripper", 0.0015)
        return {
            "status": wrapped.status,
            "reached_goal": bool(result.reached_goal),
            "stalled": bool(result.stalled),
            "reported_position_m": float(result.position),
            **measured,
        }

    def move_arm(
        self, target: list[float], seconds: float, timeout: float
    ) -> dict[str, object]:
        if not self.arm.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("arm action server unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        whole = int(seconds)
        point.time_from_start.sec = whole
        point.time_from_start.nanosec = int(
            round((seconds - whole) * 1_000_000_000)
        )
        goal.trajectory.points = [point]
        handle = self.wait_future(self.arm.send_goal_async(goal), timeout)
        if handle is None or not handle.accepted:
            raise RuntimeError("pregrasp arm goal rejected")
        wrapped = self.wait_future(handle.get_result_async(), timeout + seconds)
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"pregrasp arm action failed: status={getattr(wrapped, 'status', None)}"
            )
        self.spin_for(0.8)
        measured_arm = [self.latest.get(name) for name in ARM_JOINTS]
        if any(value is None for value in measured_arm):
            raise RuntimeError("final arm joint state missing")
        errors = [float(measured_arm[i]) - target[i] for i in range(6)]
        if max(abs(error) for error in errors) > 0.01:
            raise RuntimeError(f"pregrasp joint error too large: {errors}")
        measured_open = self.validate_open_state("arm-motion", 0.002)
        return {
            "status": wrapped.status,
            "target_rad": list(target),
            "measured_rad": [float(value) for value in measured_arm],
            "error_rad": errors,
            "max_abs_error_rad": max(abs(error) for error in errors),
            **measured_open,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--arm-seconds", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()

    input_data = json.loads(Path(args.input).read_text(encoding="utf-8"))
    target = [float(value) for value in input_data["arm_target_rad"]]
    if len(target) != 6 or not all(math.isfinite(value) for value in target):
        raise SystemExit("invalid arm target")

    rclpy.init()
    node = SetupNode()
    errors: list[str] = []
    initial = None
    open_result = None
    arm_result = None
    try:
        node.wait_for_initial_state(args.timeout)
        initial = {
            **{name: node.latest.get(name) for name in ARM_JOINTS + [LEFT, RIGHT]},
            "right_gripper_joint_observed": RIGHT in node.latest,
            "physical_link_separation_m": node.latest_link_separation_m,
        }
        open_result = node.open_gripper(args.timeout)
        arm_result = node.move_arm(target, args.arm_seconds, args.timeout)
        node.spin_for(1.0)
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        final = {
            **{name: node.latest.get(name) for name in ARM_JOINTS + [LEFT, RIGHT]},
            "right_gripper_joint_observed": RIGHT in node.latest,
            "physical_link_separation_m": node.latest_link_separation_m,
        }
        payload = {
            "schema_version": 2,
            "phase": "RUBIK-PREGRASP-OPEN-ARM-SETUP",
            "input": input_data,
            "mimic_joint_observation_policy": (
                "right_joint_optional; physical left/right link separation required"
            ),
            "gripper_open_before_arm": open_result is not None,
            "open_result": open_result,
            "arm_motion_performed": arm_result is not None,
            "arm_result": arm_result,
            "initial_joint_state": initial,
            "final_joint_state": final,
            "fixture_present": False,
            "base_motion_commanded": False,
            "lift_command_sent": False,
            "attachment_used": False,
            "grasp_success_claimed": False,
            "errors": errors,
            "samples": node.samples,
        }
        Path(args.output).write_text(
            json.dumps(payload, indent=2) + "\n", encoding="utf-8"
        )
        node.destroy_node()
        rclpy.shutdown()

    print(
        json.dumps(
            {key: value for key, value in payload.items() if key != "samples"},
            indent=2,
        )
    )
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
