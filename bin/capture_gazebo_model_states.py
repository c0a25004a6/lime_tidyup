#!/usr/bin/env python3
"""Discover and capture one read-only Gazebo ModelStates sample."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

MODEL_STATES_TYPE = "gazebo_msgs/msg/ModelStates"


def enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    return str(name if name is not None else value)


class ModelStateCapture(Node):
    def __init__(self) -> None:
        super().__init__("rubiks_gazebo_model_state_capture")
        self.message: ModelStates | None = None
        self.subscription = None

    def discover_topic(self, timeout_s: float) -> tuple[str, list[dict[str, Any]], list[dict[str, Any]]]:
        deadline = time.monotonic() + timeout_s
        latest_graph: list[dict[str, Any]] = []
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            graph = self.get_topic_names_and_types(no_demangle=False)
            latest_graph = [
                {"topic": str(topic), "types": sorted(str(type_name) for type_name in type_names)}
                for topic, type_names in sorted(graph)
            ]
            matches = [
                entry["topic"]
                for entry in latest_graph
                if MODEL_STATES_TYPE in entry["types"]
            ]
            if len(matches) == 1:
                topic = matches[0]
                publishers = self.get_publishers_info_by_topic(topic, no_mangle=False)
                publisher_evidence = []
                for publisher in publishers:
                    qos = publisher.qos_profile
                    publisher_evidence.append(
                        {
                            "node_name": str(publisher.node_name),
                            "node_namespace": str(publisher.node_namespace),
                            "topic_type": str(publisher.topic_type),
                            "endpoint_gid_hex": bytes(publisher.endpoint_gid).hex(),
                            "qos": {
                                "history": enum_name(qos.history),
                                "depth": int(qos.depth),
                                "reliability": enum_name(qos.reliability),
                                "durability": enum_name(qos.durability),
                                "liveliness": enum_name(qos.liveliness),
                            },
                        }
                    )
                if not publisher_evidence:
                    time.sleep(0.1)
                    continue
                return topic, latest_graph, publisher_evidence
            if len(matches) > 1:
                raise RuntimeError(f"multiple {MODEL_STATES_TYPE} topics found: {matches}")
        raise RuntimeError(
            f"no unique {MODEL_STATES_TYPE} topic discovered; graph={latest_graph}"
        )

    def subscribe(self, topic: str) -> None:
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
    parser.add_argument("--expected-model", action="append", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--graph-timeout", type=float, default=10.0)
    parser.add_argument("--message-timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = ModelStateCapture()
    message: ModelStates | None = None
    topic = ""
    graph: list[dict[str, Any]] = []
    publishers: list[dict[str, Any]] = []
    try:
        topic, graph, publishers = node.discover_topic(args.graph_timeout)
        node.subscribe(topic)
        deadline = time.monotonic() + args.message_timeout
        while rclpy.ok() and node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.message is None:
            raise RuntimeError(f"no ModelStates message received from discovered topic {topic}")
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
        "schema_version": 3,
        "discovery": {
            "api": "rclpy_node_graph",
            "required_type": MODEL_STATES_TYPE,
            "graph_snapshot": graph,
            "matching_topic_count": 1,
            "publisher_count": len(publishers),
            "publishers": publishers,
        },
        "source_topic": topic,
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
