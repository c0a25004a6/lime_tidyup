#!/usr/bin/env python3
"""Discover and capture one read-only Gazebo ModelStates sample."""
from __future__ import annotations

import argparse
import json
import time
import traceback
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


def safe_int(value: Any) -> int | str | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return str(value)


def safe_gid_hex(value: Any) -> str | None:
    if value is None:
        return None
    try:
        return bytes(value).hex()
    except (TypeError, ValueError):
        try:
            return str(value)
        except Exception:
            return "<unprintable>"


def safe_duration_ns(value: Any) -> int | str | None:
    if value is None:
        return None
    nanoseconds = getattr(value, "nanoseconds", None)
    return safe_int(nanoseconds if nanoseconds is not None else value)


def publisher_to_evidence(publisher: Any) -> dict[str, Any]:
    qos = getattr(publisher, "qos_profile", None)
    evidence: dict[str, Any] = {
        "node_name": str(getattr(publisher, "node_name", "")),
        "node_namespace": str(getattr(publisher, "node_namespace", "")),
        "topic_type": str(getattr(publisher, "topic_type", "")),
        "endpoint_gid_hex": safe_gid_hex(getattr(publisher, "endpoint_gid", None)),
        "endpoint_type": enum_name(getattr(publisher, "endpoint_type", "unknown")),
    }
    if qos is None:
        evidence["qos"] = None
        return evidence
    evidence["qos"] = {
        "history": enum_name(getattr(qos, "history", "unknown")),
        "depth": safe_int(getattr(qos, "depth", None)),
        "reliability": enum_name(getattr(qos, "reliability", "unknown")),
        "durability": enum_name(getattr(qos, "durability", "unknown")),
        "lifespan_ns": safe_duration_ns(getattr(qos, "lifespan", None)),
        "deadline_ns": safe_duration_ns(getattr(qos, "deadline", None)),
        "liveliness": enum_name(getattr(qos, "liveliness", "unknown")),
        "liveliness_lease_duration_ns": safe_duration_ns(
            getattr(qos, "liveliness_lease_duration", None)
        ),
    }
    return evidence


class ModelStateCapture(Node):
    def __init__(self) -> None:
        super().__init__("rubiks_gazebo_model_state_capture")
        self.message: ModelStates | None = None
        self.subscription = None
        self.graph_attempts = 0
        self.graph_errors: list[dict[str, str]] = []
        self.latest_graph: list[dict[str, Any]] = []
        self.latest_matches: list[str] = []
        self.latest_publishers: list[dict[str, Any]] = []

    def _topic_graph(self) -> list[tuple[str, list[str]]]:
        try:
            return self.get_topic_names_and_types(no_demangle=False)
        except TypeError:
            # Some Humble patch levels expose the same API without the keyword.
            return self.get_topic_names_and_types()

    def _publisher_info(self, topic: str) -> list[Any]:
        try:
            return self.get_publishers_info_by_topic(topic, no_mangle=False)
        except TypeError:
            # Some Humble patch levels expose the same API without the keyword.
            return self.get_publishers_info_by_topic(topic)

    def discover_topic(
        self, timeout_s: float
    ) -> tuple[str, list[dict[str, Any]], list[dict[str, Any]]]:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.graph_attempts += 1
            try:
                graph = self._topic_graph()
            except Exception as exc:
                self.graph_errors.append(
                    {
                        "operation": "get_topic_names_and_types",
                        "error_type": type(exc).__name__,
                        "error_message": str(exc),
                    }
                )
                time.sleep(0.1)
                continue

            self.latest_graph = [
                {"topic": str(topic), "types": sorted(str(type_name) for type_name in type_names)}
                for topic, type_names in sorted(graph, key=lambda item: str(item[0]))
            ]
            self.latest_matches = [
                str(entry["topic"])
                for entry in self.latest_graph
                if MODEL_STATES_TYPE in entry["types"]
            ]
            if len(self.latest_matches) > 1:
                raise RuntimeError(
                    f"multiple {MODEL_STATES_TYPE} topics found: {self.latest_matches}"
                )
            if len(self.latest_matches) != 1:
                time.sleep(0.1)
                continue

            topic = self.latest_matches[0]
            try:
                publishers = self._publisher_info(topic)
            except Exception as exc:
                self.graph_errors.append(
                    {
                        "operation": "get_publishers_info_by_topic",
                        "topic": topic,
                        "error_type": type(exc).__name__,
                        "error_message": str(exc),
                    }
                )
                time.sleep(0.1)
                continue

            publisher_evidence: list[dict[str, Any]] = []
            for index, publisher in enumerate(publishers):
                try:
                    publisher_evidence.append(publisher_to_evidence(publisher))
                except Exception as exc:
                    publisher_evidence.append(
                        {
                            "serialization_error": {
                                "index": index,
                                "error_type": type(exc).__name__,
                                "error_message": str(exc),
                            },
                            "topic_type": str(getattr(publisher, "topic_type", "")),
                        }
                    )
            self.latest_publishers = publisher_evidence
            if not publishers:
                time.sleep(0.1)
                continue
            return topic, self.latest_graph, publisher_evidence

        raise RuntimeError(
            f"no unique published {MODEL_STATES_TYPE} topic discovered; "
            f"matches={self.latest_matches}; graph={self.latest_graph}; "
            f"graph_errors={self.graph_errors}"
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


def write_payload(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected-model", action="append", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--graph-timeout", type=float, default=10.0)
    parser.add_argument("--message-timeout", type=float, default=10.0)
    args = parser.parse_args()

    output = Path(args.output)
    started_monotonic = time.monotonic()
    expected = sorted(set(args.expected_model))
    payload: dict[str, Any] = {
        "schema_version": 3,
        "status": "error",
        "phase": "initializing",
        "required_type": MODEL_STATES_TYPE,
        "expected_models": expected,
        "graph_timeout_seconds": args.graph_timeout,
        "message_timeout_seconds": args.message_timeout,
        "errors": [],
    }
    node: ModelStateCapture | None = None
    initialized = False
    exit_code = 1

    try:
        rclpy.init()
        initialized = True
        node = ModelStateCapture()
        payload["phase"] = "discovering_topic"
        topic, graph, publishers = node.discover_topic(args.graph_timeout)
        payload["discovery"] = {
            "api": "rclpy_node_graph",
            "required_type": MODEL_STATES_TYPE,
            "graph_snapshot": graph,
            "graph_attempts": node.graph_attempts,
            "graph_errors": node.graph_errors,
            "matching_topic_count": 1,
            "publisher_count": len(publishers),
            "publishers": publishers,
        }
        payload["source_topic"] = topic
        payload["phase"] = "waiting_for_message"
        node.subscribe(topic)
        deadline = time.monotonic() + args.message_timeout
        while rclpy.ok() and node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.message is None:
            raise RuntimeError(f"no ModelStates message received from discovered topic {topic}")

        message = node.message
        names = [str(name) for name in message.name]
        if len(names) != len(set(names)):
            raise RuntimeError(f"duplicate model names in ModelStates: {names}")
        if sorted(names) != expected:
            raise RuntimeError(f"unexpected world models: {names}; expected {expected}")
        forbidden_tokens = ("cube", "support", "fixture", "rubiks_dynamic", "ifra")
        forbidden = [
            name for name in names
            if any(token in name.lower() for token in forbidden_tokens)
        ]
        if forbidden:
            raise RuntimeError(f"forbidden fixture model present: {forbidden}")
        pose_count = len(message.pose)
        twist_count = len(message.twist)
        if pose_count != len(names) or twist_count != len(names):
            raise RuntimeError(
                f"ModelStates array length mismatch: names={len(names)}, "
                f"poses={pose_count}, twists={twist_count}"
            )

        payload.update(
            {
                "status": "ok",
                "phase": "complete",
                "subscription_qos": {
                    "history": "KEEP_LAST",
                    "depth": 1,
                    "reliability": "BEST_EFFORT",
                    "durability": "VOLATILE",
                },
                "models": names,
                "pose_count": pose_count,
                "twist_count": twist_count,
                "exact_set_match": True,
                "forbidden_model_present": False,
            }
        )
        exit_code = 0
    except Exception as exc:
        payload["errors"].append(
            {
                "phase": payload.get("phase"),
                "error_type": type(exc).__name__,
                "error_message": str(exc),
                "traceback": traceback.format_exc(),
            }
        )
        if node is not None:
            payload.setdefault(
                "discovery",
                {
                    "api": "rclpy_node_graph",
                    "required_type": MODEL_STATES_TYPE,
                    "graph_snapshot": node.latest_graph,
                    "graph_attempts": node.graph_attempts,
                    "graph_errors": node.graph_errors,
                    "matching_topic_count": len(node.latest_matches),
                    "publisher_count": len(node.latest_publishers),
                    "publishers": node.latest_publishers,
                },
            )
    finally:
        payload["elapsed_seconds"] = round(time.monotonic() - started_monotonic, 6)
        try:
            write_payload(output, payload)
        except Exception as exc:
            print(f"failed to write diagnostic payload {output}: {exc}", flush=True)
            exit_code = 2
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if initialized:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    print(json.dumps(payload, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
