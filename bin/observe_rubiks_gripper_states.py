#!/usr/bin/env python3
"""Record read-only arm and gripper state samples from Humble broadcasters."""
from __future__ import annotations

import argparse
import json
import math
import signal
import time
from pathlib import Path
from typing import Any

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
MODEL_GRIPPER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]
SOURCE_ALIASES = {
    "gripper_left_joint": ["gripper_left_joint"],
    "gripper_right_joint": ["gripper_right_joint", "gripper_right_joint_mimic"],
}


def finite(values: list[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def choose_source_name(available: set[str], model_name: str) -> str | None:
    matches = [name for name in SOURCE_ALIASES[model_name] if name in available]
    if len(matches) != 1:
        return None
    return matches[0]


class GripperStateObserver(Node):
    def __init__(self) -> None:
        super().__init__("rubiks_gripper_state_observer")
        self.started_monotonic_ns = time.monotonic_ns()
        self.samples: list[dict[str, Any]] = []
        self.joint_state_messages_seen = 0
        self.joint_state_messages_missing_required_joints = 0
        self.dynamic_messages_seen = 0
        self.dynamic_messages_missing_required_joints = 0
        self.dynamic_messages_missing_required_interfaces = 0
        self.joint_state_joint_names: list[str] = []
        self.dynamic_joint_names: list[str] = []
        self.dynamic_interface_inventory: dict[str, list[str]] = {}
        self.latest_graph_snapshot: list[dict[str, Any]] = []
        self.errors: list[str] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.joint_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_callback, qos
        )
        self.dynamic_subscription = self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self._dynamic_callback, qos
        )

    def _append_sample(
        self,
        *,
        source_topic: str,
        ros_stamp_ns: int,
        arm_positions: list[float],
        gripper_positions: list[float],
        arm_velocities: list[float] | None,
        gripper_velocities: list[float] | None,
        source_joint_names: list[str],
        source_interfaces: dict[str, list[str]],
        gripper_source_joint_names: list[str],
    ) -> None:
        positions = arm_positions + gripper_positions
        velocities = (
            None
            if arm_velocities is None or gripper_velocities is None
            else arm_velocities + gripper_velocities
        )
        self.samples.append(
            {
                "source_topic": source_topic,
                "monotonic_ns": time.monotonic_ns(),
                "ros_stamp_ns": ros_stamp_ns,
                "source_joint_names": source_joint_names,
                "source_interfaces": source_interfaces,
                "arm_joint_order": ARM_JOINTS,
                "model_gripper_joint_order": MODEL_GRIPPER_JOINTS,
                "gripper_source_joint_names": gripper_source_joint_names,
                "right_source_is_ros2_control_mimic_alias": (
                    gripper_source_joint_names[1] == "gripper_right_joint_mimic"
                ),
                "arm_positions_rad": arm_positions,
                "gripper_positions_m": gripper_positions,
                "arm_velocities_rad_s": arm_velocities,
                "gripper_velocities_m_s": gripper_velocities,
                "position_all_finite": finite(positions),
                "velocity_available": velocities is not None,
                "velocity_all_finite": velocities is not None and finite(velocities),
            }
        )

    def _joint_callback(self, message: JointState) -> None:
        self.joint_state_messages_seen += 1
        names = [str(name) for name in message.name]
        self.joint_state_joint_names = names
        index = {name: position for position, name in enumerate(names)}
        available = set(index)
        gripper_sources = [
            choose_source_name(available, model_name) for model_name in MODEL_GRIPPER_JOINTS
        ]
        if any(name not in index for name in ARM_JOINTS) or any(
            source is None for source in gripper_sources
        ):
            self.joint_state_messages_missing_required_joints += 1
            return
        if len(message.position) < len(names):
            self.errors.append("JointState position array is shorter than name array")
            return
        concrete_sources = [str(source) for source in gripper_sources]
        arm_positions = [float(message.position[index[name]]) for name in ARM_JOINTS]
        gripper_positions = [
            float(message.position[index[name]]) for name in concrete_sources
        ]
        velocity_available = len(message.velocity) >= len(names)
        arm_velocities = (
            [float(message.velocity[index[name]]) for name in ARM_JOINTS]
            if velocity_available
            else None
        )
        gripper_velocities = (
            [float(message.velocity[index[name]]) for name in concrete_sources]
            if velocity_available
            else None
        )
        self._append_sample(
            source_topic="/joint_states",
            ros_stamp_ns=stamp_ns(message),
            arm_positions=arm_positions,
            gripper_positions=gripper_positions,
            arm_velocities=arm_velocities,
            gripper_velocities=gripper_velocities,
            source_joint_names=names,
            source_interfaces={name: ["position", "velocity", "effort"] for name in names},
            gripper_source_joint_names=concrete_sources,
        )

    def _dynamic_callback(self, message: DynamicJointState) -> None:
        self.dynamic_messages_seen += 1
        names = [str(name) for name in message.joint_names]
        self.dynamic_joint_names = names
        if len(names) != len(message.interface_values):
            self.errors.append("DynamicJointState joint/interface array length mismatch")
            return
        values_by_joint: dict[str, dict[str, float]] = {}
        inventory: dict[str, list[str]] = {}
        for name, interface_value in zip(names, message.interface_values):
            interfaces = [str(value) for value in interface_value.interface_names]
            inventory[name] = interfaces
            if len(interfaces) != len(interface_value.values):
                self.errors.append(
                    f"DynamicJointState interface/value length mismatch for {name}"
                )
                return
            values_by_joint[name] = {
                interface: float(value)
                for interface, value in zip(interfaces, interface_value.values)
            }
        self.dynamic_interface_inventory = inventory
        available = set(values_by_joint)
        gripper_sources = [
            choose_source_name(available, model_name) for model_name in MODEL_GRIPPER_JOINTS
        ]
        if any(name not in values_by_joint for name in ARM_JOINTS) or any(
            source is None for source in gripper_sources
        ):
            self.dynamic_messages_missing_required_joints += 1
            return
        concrete_sources = [str(source) for source in gripper_sources]
        required_sources = ARM_JOINTS + concrete_sources
        if any(
            "position" not in values_by_joint[name]
            or "velocity" not in values_by_joint[name]
            for name in required_sources
        ):
            self.dynamic_messages_missing_required_interfaces += 1
            return
        self._append_sample(
            source_topic="/dynamic_joint_states",
            ros_stamp_ns=stamp_ns(message),
            arm_positions=[values_by_joint[name]["position"] for name in ARM_JOINTS],
            gripper_positions=[
                values_by_joint[name]["position"] for name in concrete_sources
            ],
            arm_velocities=[values_by_joint[name]["velocity"] for name in ARM_JOINTS],
            gripper_velocities=[
                values_by_joint[name]["velocity"] for name in concrete_sources
            ],
            source_joint_names=names,
            source_interfaces=inventory,
            gripper_source_joint_names=concrete_sources,
        )

    def refresh_graph_snapshot(self) -> None:
        try:
            try:
                graph = self.get_topic_names_and_types(no_demangle=False)
            except TypeError:
                graph = self.get_topic_names_and_types()
            self.latest_graph_snapshot = [
                {"topic": str(topic), "types": sorted(str(value) for value in types)}
                for topic, types in sorted(graph, key=lambda item: str(item[0]))
            ]
        except Exception:
            # Keep the last valid graph snapshot. Shutdown invalidates the Humble node context.
            pass


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
    next_graph_refresh = 0.0
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_graph_refresh:
                node.refresh_graph_snapshot()
                next_graph_refresh = now + 1.0
        exit_code = 0 if node.samples else 1
    except ExternalShutdownException:
        exit_code = 0 if node.samples else 1
    except Exception as error:
        node.errors.append(f"{type(error).__name__}: {error}")
    finally:
        passed = exit_code == 0 and not node.errors and bool(node.samples)
        source_name_sets = sorted(
            {
                tuple(sample["gripper_source_joint_names"])
                for sample in node.samples
            }
        )
        payload = {
            "schema_version": 3,
            "phase": "RUBIK-PREGRASP-GRIPPER-STATE-OBSERVATION",
            "source_ref": args.source_ref,
            "passed": passed,
            "errors": node.errors,
            "started_monotonic_ns": node.started_monotonic_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "observed_topics": ["/joint_states", "/dynamic_joint_states"],
            "topic_graph_snapshot": node.latest_graph_snapshot,
            "arm_joint_order": ARM_JOINTS,
            "gripper_joint_order": MODEL_GRIPPER_JOINTS,
            "gripper_source_alias_contract": SOURCE_ALIASES,
            "observed_gripper_source_joint_name_sets": [list(value) for value in source_name_sets],
            "joint_state_messages_seen": node.joint_state_messages_seen,
            "joint_state_messages_missing_required_joints": (
                node.joint_state_messages_missing_required_joints
            ),
            "joint_state_joint_names": node.joint_state_joint_names,
            "dynamic_messages_seen": node.dynamic_messages_seen,
            "dynamic_messages_missing_required_joints": (
                node.dynamic_messages_missing_required_joints
            ),
            "dynamic_messages_missing_required_interfaces": (
                node.dynamic_messages_missing_required_interfaces
            ),
            "dynamic_joint_names": node.dynamic_joint_names,
            "dynamic_interface_inventory": node.dynamic_interface_inventory,
            "sample_count": len(node.samples),
            "sample_source_counts": {
                topic: sum(
                    1 for sample in node.samples if sample["source_topic"] == topic
                )
                for topic in ("/joint_states", "/dynamic_joint_states")
            },
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
        output.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(payload, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if payload.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
