#!/usr/bin/env python3
"""Record read-only arm and gripper JointState samples for exact-stamp binding."""
from __future__ import annotations

import argparse
import json
import math
import signal
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
GRIPPER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]
REQUIRED_JOINTS = ARM_JOINTS + GRIPPER_JOINTS


def finite(values: list[float]) -> bool:
    return all(math.isfinite(value) for value in values)


class GripperStateObserver(Node):
    def __init__(self) -> None:
        super().__init__("rubiks_gripper_state_observer")
        self.started_monotonic_ns = time.monotonic_ns()
        self.samples: list[dict[str, Any]] = []
        self.messages_seen = 0
        self.messages_missing_required_joints = 0
        self.errors: list[str] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._callback, qos
        )

    def _callback(self, message: JointState) -> None:
        self.messages_seen += 1
        names = [str(name) for name in message.name]
        index = {name: position for position, name in enumerate(names)}
        if any(name not in index for name in REQUIRED_JOINTS):
            self.messages_missing_required_joints += 1
            return
        if len(message.position) < len(names):
            self.errors.append("JointState position array is shorter than name array")
            return
        positions = [float(message.position[index[name]]) for name in REQUIRED_JOINTS]
        velocity_available = len(message.velocity) >= len(names)
        velocities = (
            [float(message.velocity[index[name]]) for name in REQUIRED_JOINTS]
            if velocity_available
            else None
        )
        sample = {
            "monotonic_ns": time.monotonic_ns(),
            "ros_stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec),
            "joint_names": names,
            "required_joint_order": REQUIRED_JOINTS,
            "arm_positions_rad": positions[: len(ARM_JOINTS)],
            "gripper_positions_m": positions[len(ARM_JOINTS) :],
            "arm_velocities_rad_s": (
                None if velocities is None else velocities[: len(ARM_JOINTS)]
            ),
            "gripper_velocities_m_s": (
                None if velocities is None else velocities[len(ARM_JOINTS) :]
            ),
            "position_all_finite": finite(positions),
            "velocity_available": velocity_available,
            "velocity_all_finite": velocities is not None and finite(velocities),
        }
        self.samples.append(sample)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    stop = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init()
    node = GripperStateObserver()
    exit_code = 1
    payload: dict[str, Any] = {}
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.05)
        exit_code = 0 if node.samples else 1
    except ExternalShutdownException:
        exit_code = 0 if node.samples else 1
    except Exception as error:
        node.errors.append(f"{type(error).__name__}: {error}")
    finally:
        passed = exit_code == 0 and not node.errors and bool(node.samples)
        payload = {
            "schema_version": 1,
            "phase": "RUBIK-PREGRASP-GRIPPER-STATE-OBSERVATION",
            "source_ref": args.source_ref,
            "passed": passed,
            "errors": node.errors,
            "started_monotonic_ns": node.started_monotonic_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "topic": "/joint_states",
            "arm_joint_order": ARM_JOINTS,
            "gripper_joint_order": GRIPPER_JOINTS,
            "required_joint_order": REQUIRED_JOINTS,
            "messages_seen": node.messages_seen,
            "messages_missing_required_joints": node.messages_missing_required_joints,
            "sample_count": len(node.samples),
            "samples": node.samples,
            "safety": {
                "read_only_subscription_only": True,
                "publisher_created": False,
                "service_client_created": False,
                "action_client_created": False,
                "controller_loaded": False,
                "command_sent": False,
            },
        }
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(payload, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if payload.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
