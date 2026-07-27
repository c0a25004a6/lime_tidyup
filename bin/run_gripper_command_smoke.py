#!/usr/bin/env python3
"""Command the Lime gripper open-close-open and record joint-state telemetry."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState


class SmokeNode(Node):
    def __init__(self, action_name: str, left_joint: str, right_joint: str) -> None:
        super().__init__("rubiks_gripper_command_smoke")
        self.client = ActionClient(self, GripperCommand, action_name)
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.started = time.monotonic()
        self.samples: list[dict] = []
        self.latest: dict[str, float | bool] = {}
        self.phase = "initial"
        self.command_target_m: float | None = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 50)

    def on_joint_state(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        if self.left_joint not in values:
            return
        left = float(values[self.left_joint])
        right_value = values.get(self.right_joint)
        right_observed = right_value is not None
        right = float(right_value) if right_observed else left
        self.latest = {
            "left": left,
            "right": right,
            "right_observed": right_observed,
        }
        separation = abs((0.021 + left) - (-0.021 - right))
        self.samples.append({
            "t_s": time.monotonic() - self.started,
            "phase": self.phase,
            "command_target_m": self.command_target_m,
            "left_joint_position_m": left,
            "right_joint_position_m": right,
            "right_joint_observed": right_observed,
            "link_frame_separation_m": separation,
            "inner_face_opening_m": None,
        })

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, label: str, target: float, max_effort: float, timeout: float) -> dict:
        self.phase = label
        self.command_target_m = target
        if not self.client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("gripper action server was not available")
        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = max_effort
        send_future = self.client.send_goal_async(goal)
        send_deadline = time.monotonic() + timeout
        while rclpy.ok() and not send_future.done() and time.monotonic() < send_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not send_future.done():
            raise RuntimeError(f"timed out sending gripper target {target}")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"gripper target {target} was rejected")
        result_future = handle.get_result_async()
        result_deadline = time.monotonic() + timeout
        while rclpy.ok() and not result_future.done() and time.monotonic() < result_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done():
            raise RuntimeError(f"timed out executing gripper target {target}")
        wrapped = result_future.result()
        result = wrapped.result
        return {
            "label": label,
            "target_position_m": target,
            "result_status": int(wrapped.status),
            "reached_goal": bool(result.reached_goal),
            "stalled": bool(result.stalled),
            "reported_position_m": float(result.position),
            "reported_effort": float(result.effort),
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action", default="/gripper_controller/gripper_cmd")
    parser.add_argument("--left-joint", default="gripper_left_joint")
    parser.add_argument("--right-joint", default="gripper_right_joint")
    parser.add_argument("--output", required=True)
    parser.add_argument("--hold", type=float, default=1.5)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = SmokeNode(args.action, args.left_joint, args.right_joint)
    events: list[dict] = []
    try:
        node.spin_for(1.0)
        if not node.latest:
            raise RuntimeError("gripper joint was not observed on /joint_states")
        if not bool(node.latest.get("right_observed")):
            raise RuntimeError("right mimic joint was not observed on /joint_states")
        for label, target in (("open_1", 0.019), ("close", -0.010), ("open_2", 0.019)):
            command_t = time.monotonic() - node.started
            outcome = node.send(label, target, max_effort=1.0, timeout=args.timeout)
            node.spin_for(args.hold)
            outcome.update({
                "command_t_s": command_t,
                "hold_end_t_s": time.monotonic() - node.started,
                "observed_left_joint_position_m": node.latest.get("left"),
                "observed_right_joint_position_m": node.latest.get("right"),
                "right_joint_observed": bool(node.latest.get("right_observed")),
            })
            events.append(outcome)
        node.phase = "complete"
        node.command_target_m = None
        node.spin_for(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(node.samples) < 10:
        raise RuntimeError(f"insufficient joint-state samples: {len(node.samples)}")
    payload = {
        "schema_version": 1,
        "trial_type": "fake_hardware_gripper_command_smoke",
        "action_name": args.action,
        "left_joint": args.left_joint,
        "right_joint": args.right_joint,
        "inner_face_opening_calibrated": False,
        "cube_contact": False,
        "grasp_success_claimed": False,
        "sample_count": len(node.samples),
        "events": events,
        "samples": node.samples,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in payload.items() if k != "samples"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
