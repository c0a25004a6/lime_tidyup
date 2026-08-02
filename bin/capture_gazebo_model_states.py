#!/usr/bin/env python3
"""Capture one read-only Gazebo ModelStates sample and validate the exact model set."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class ModelStateCapture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("rubiks_gazebo_model_state_capture")
        self.message: ModelStates | None = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(ModelStates, topic, self._callback, qos)

    def _callback(self, message: ModelStates) -> None:
        if self.message is None:
            self.message = message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--expected-model", action="append", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = ModelStateCapture(args.topic)
    message: ModelStates | None = None
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.message is None:
            raise SystemExit(f"no ModelStates message received from {args.topic}")
        message = node.message
        names = [str(name) for name in message.name]
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if message is None:
        raise SystemExit("ModelStates capture ended without a message")
    if len(names) != len(set(names)):
        raise SystemExit(f"duplicate model names in ModelStates: {names}")
    expected = sorted(set(args.expected_model))
    if sorted(names) != expected:
        raise SystemExit(f"unexpected world models: {names}; expected {expected}")
    forbidden_tokens = ("cube", "support", "fixture", "rubiks_dynamic", "ifra")
    forbidden = [name for name in names if any(token in name.lower() for token in forbidden_tokens)]
    if forbidden:
        raise SystemExit(f"forbidden fixture model present: {forbidden}")

    payload = {
        "schema_version": 2,
        "source_topic": args.topic,
        "subscription_qos": {
            "history": "KEEP_LAST",
            "depth": 1,
            "reliability": "BEST_EFFORT",
            "durability": "VOLATILE",
        },
        "models": names,
        "expected_models": expected,
        "pose_count": len(message.pose),
        "twist_count": len(message.twist),
        "exact_set_match": True,
        "forbidden_model_present": False,
    }
    if payload["pose_count"] != len(names) or payload["twist_count"] != len(names):
        raise SystemExit(f"ModelStates array length mismatch: {payload}")
    Path(args.output).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
