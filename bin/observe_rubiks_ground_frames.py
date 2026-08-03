#!/usr/bin/env python3
"""Record read-only Gazebo model/link poses paired with arm JointState stamps."""
from __future__ import annotations

import argparse
import json
import math
import signal
import time
from pathlib import Path
from typing import Any

import rclpy
from gazebo_msgs.msg import LinkStates, ModelStates
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]


def pose_dict(pose: Any) -> dict[str, Any]:
    values = [
        float(pose.position.x), float(pose.position.y), float(pose.position.z),
        float(pose.orientation.x), float(pose.orientation.y),
        float(pose.orientation.z), float(pose.orientation.w),
    ]
    return {
        "position_m": values[:3],
        "orientation_xyzw": values[3:],
        "all_finite": all(math.isfinite(value) for value in values),
    }


def twist_dict(twist: Any) -> dict[str, Any]:
    values = [
        float(twist.linear.x), float(twist.linear.y), float(twist.linear.z),
        float(twist.angular.x), float(twist.angular.y), float(twist.angular.z),
    ]
    return {
        "linear_m_s": values[:3],
        "angular_rad_s": values[3:],
        "all_finite": all(math.isfinite(value) for value in values),
    }


class GroundFrameObserver(Node):
    def __init__(self, model_name: str) -> None:
        super().__init__("rubiks_ground_frame_observer")
        self.model_name = model_name
        self.started_monotonic_ns = time.monotonic_ns()
        self.latest_model: dict[str, Any] | None = None
        self.latest_base_link: dict[str, Any] | None = None
        self.latest_base_footprint: dict[str, Any] | None = None
        self.model_names: list[str] = []
        self.link_names: list[str] = []
        self.arm_samples: list[dict[str, Any]] = []
        self.errors: list[str] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.model_sub = self.create_subscription(ModelStates, "/model_states", self._models, qos)
        self.link_sub = self.create_subscription(LinkStates, "/link_states", self._links, qos)
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self._joints, qos)

    def _models(self, message: ModelStates) -> None:
        now = time.monotonic_ns()
        self.model_names = [str(name) for name in message.name]
        matches = [i for i, name in enumerate(message.name) if name == self.model_name]
        if len(matches) != 1:
            self.latest_model = None
            return
        index = matches[0]
        self.latest_model = {
            "monotonic_ns": now,
            "name": self.model_name,
            "pose_world": pose_dict(message.pose[index]),
            "twist_world": twist_dict(message.twist[index]),
        }

    def _links(self, message: LinkStates) -> None:
        now = time.monotonic_ns()
        self.link_names = [str(name) for name in message.name]
        prefix = self.model_name + "::"
        base_links = [i for i, name in enumerate(message.name) if name == prefix + "base_link"]
        footprints = [i for i, name in enumerate(message.name) if name == prefix + "base_footprint"]
        self.latest_base_link = None
        self.latest_base_footprint = None
        if len(base_links) == 1:
            index = base_links[0]
            self.latest_base_link = {
                "monotonic_ns": now,
                "name": str(message.name[index]),
                "pose_world": pose_dict(message.pose[index]),
                "twist_world": twist_dict(message.twist[index]),
            }
        if len(footprints) == 1:
            index = footprints[0]
            self.latest_base_footprint = {
                "monotonic_ns": now,
                "name": str(message.name[index]),
                "pose_world": pose_dict(message.pose[index]),
                "twist_world": twist_dict(message.twist[index]),
            }

    def _joints(self, message: JointState) -> None:
        now = time.monotonic_ns()
        index = {str(name): i for i, name in enumerate(message.name)}
        if any(name not in index for name in ARM_JOINTS):
            return
        positions = [float(message.position[index[name]]) for name in ARM_JOINTS]
        sample: dict[str, Any] = {
            "monotonic_ns": now,
            "ros_stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec),
            "positions_rad": positions,
            "all_finite": all(math.isfinite(value) for value in positions),
            "model_state": self.latest_model,
            "base_link_state": self.latest_base_link,
            "base_footprint_state": self.latest_base_footprint,
        }
        for key in ("model_state", "base_link_state", "base_footprint_state"):
            state = sample[key]
            sample[key + "_age_s"] = (
                None if state is None else (now - int(state["monotonic_ns"])) / 1_000_000_000.0
            )
        self.arm_samples.append(sample)

    def ready(self) -> bool:
        root_observed = self.latest_base_footprint is not None or self.latest_base_link is not None
        return bool(self.arm_samples and self.latest_model and root_observed)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--ready-file", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    stop = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    output = Path(args.output)
    ready_file = Path(args.ready_file)
    rclpy.init()
    node = GroundFrameObserver(args.model)
    exit_code = 1
    payload: dict[str, Any] = {}
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.ready() and not ready_file.exists():
                ready_file.parent.mkdir(parents=True, exist_ok=True)
                ready_file.write_text("ready\n", encoding="utf-8")
        exit_code = 0 if node.ready() else 1
    except ExternalShutdownException:
        # Humble raises this when the process receives the requested shutdown signal.
        exit_code = 0 if node.ready() else 1
    except Exception as error:
        node.errors.append(f"{type(error).__name__}: {error}")
    finally:
        payload = {
            "schema_version": 2,
            "phase": "RUBIK-PREGRASP-GROUND-FRAME-OBSERVATION",
            "source_ref": args.source_ref,
            "model_name": args.model,
            "passed": exit_code == 0 and not node.errors,
            "errors": node.errors,
            "started_monotonic_ns": node.started_monotonic_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "model_names": node.model_names,
            "link_names": node.link_names,
            "arm_joint_order": ARM_JOINTS,
            "arm_sample_count": len(node.arm_samples),
            "base_link_observed": any(
                sample.get("base_link_state") is not None for sample in node.arm_samples
            ),
            "base_footprint_observed": any(
                sample.get("base_footprint_state") is not None for sample in node.arm_samples
            ),
            "root_observation_policy": "prefer_direct_base_footprint_else_infer_from_base_link",
            "samples": node.arm_samples,
            "safety": {
                "read_only_subscriptions_only": True,
                "publisher_created": False,
                "service_client_created": False,
                "action_client_created": False,
                "command_sent": False,
            },
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(payload, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if payload.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
