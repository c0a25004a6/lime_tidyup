#!/usr/bin/env python3
"""Open the gripper, move the arm to the exact accepted pregrasp anchor, and hold.

This setup runs before any cube/support is spawned in the dynamic close/hold
isolation gate. It never commands lift/base motion and never claims a grasp.
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
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LEFT = "gripper_left_joint"
RIGHT = "gripper_right_joint"
OPEN_M = 0.019


class SetupNode(Node):
    def __init__(self) -> None:
        super().__init__("rubiks_open_arm_pregrasp_setup")
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 50)
        self.latest: dict[str, float] = {}
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
        })

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def wait_for_joint_state(self, timeout: float) -> None:
        end = time.monotonic() + timeout
        needed = set(ARM_JOINTS + [LEFT, RIGHT])
        while time.monotonic() < end and not needed.issubset(self.latest):
            rclpy.spin_once(self, timeout_sec=0.05)
        if not needed.issubset(self.latest):
            missing = sorted(needed - set(self.latest))
            raise RuntimeError(f"joint states missing: {missing}")

    def wait_future(self, future, timeout: float):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError("action future timed out")
        return future.result()

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
            raise RuntimeError(f"open-gripper action failed: status={getattr(wrapped, 'status', None)}")
        result = wrapped.result
        self.spin_for(0.4)
        left = self.latest.get(LEFT)
        right = self.latest.get(RIGHT)
        if left is None or right is None:
            raise RuntimeError("open-gripper joint state missing")
        if abs(left - OPEN_M) > 0.0015 or abs(right - OPEN_M) > 0.0015:
            raise RuntimeError(f"gripper did not reach fully open state: left={left}, right={right}")
        return {
            "status": wrapped.status,
            "reached_goal": bool(result.reached_goal),
            "stalled": bool(result.stalled),
            "reported_position_m": float(result.position),
            "measured_left_m": left,
            "measured_right_m": right,
        }

    def move_arm(self, target: list[float], seconds: float, timeout: float) -> dict[str, object]:
        if not self.arm.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("arm action server unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        whole = int(seconds)
        point.time_from_start.sec = whole
        point.time_from_start.nanosec = int(round((seconds - whole) * 1_000_000_000))
        goal.trajectory.points = [point]
        handle = self.wait_future(self.arm.send_goal_async(goal), timeout)
        if handle is None or not handle.accepted:
            raise RuntimeError("pregrasp arm goal rejected")
        wrapped = self.wait_future(handle.get_result_async(), timeout + seconds)
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"pregrasp arm action failed: status={getattr(wrapped, 'status', None)}")
        self.spin_for(0.8)
        measured = [self.latest.get(name) for name in ARM_JOINTS]
        if any(value is None for value in measured):
            raise RuntimeError("final arm joint state missing")
        errors = [float(measured[i]) - target[i] for i in range(6)]
        if max(abs(error) for error in errors) > 0.01:
            raise RuntimeError(f"pregrasp joint error too large: {errors}")
        left = self.latest.get(LEFT)
        right = self.latest.get(RIGHT)
        if left is None or right is None or abs(left - OPEN_M) > 0.002 or abs(right - OPEN_M) > 0.002:
            raise RuntimeError(f"gripper did not remain open during arm motion: {left}, {right}")
        return {
            "status": wrapped.status,
            "target_rad": list(target),
            "measured_rad": [float(value) for value in measured],
            "error_rad": errors,
            "max_abs_error_rad": max(abs(error) for error in errors),
            "measured_left_gripper_m": left,
            "measured_right_gripper_m": right,
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
        node.wait_for_joint_state(args.timeout)
        initial = {name: node.latest.get(name) for name in ARM_JOINTS + [LEFT, RIGHT]}
        open_result = node.open_gripper(args.timeout)
        arm_result = node.move_arm(target, args.arm_seconds, args.timeout)
        node.spin_for(1.0)
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        final = {name: node.latest.get(name) for name in ARM_JOINTS + [LEFT, RIGHT]}
        payload = {
            "schema_version": 1,
            "phase": "RUBIK-PREGRASP-OPEN-ARM-SETUP",
            "input": input_data,
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
        Path(args.output).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps({key: value for key, value in payload.items() if key != "samples"}, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
