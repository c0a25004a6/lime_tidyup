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
import xml.etree.ElementTree as ET

import rclpy
from gazebo_msgs.msg import LinkStates, ModelStates
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]


def finite(values: list[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def normalize_quaternion(value: list[float]) -> list[float]:
    norm = math.sqrt(sum(component * component for component in value))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid quaternion")
    result = [component / norm for component in value]
    if result[3] < 0.0:
        result = [-component for component in result]
    return result


def multiply_quaternions(first: list[float], second: list[float]) -> list[float]:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return normalize_quaternion([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def rotate_vector(quaternion: list[float], vector: list[float]) -> list[float]:
    x, y, z, w = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ]


def quaternion_from_rpy(values: list[float]) -> list[float]:
    roll, pitch, yaw = values
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return normalize_quaternion([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def parse_vector(text: str | None) -> list[float]:
    values = [0.0, 0.0, 0.0] if not text else [float(item) for item in text.split()]
    if len(values) != 3 or not finite(values):
        raise ValueError(f"invalid URDF vector: {values}")
    return values


def root_to_base_link(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    by_child: dict[str, ET.Element] = {}
    links = [str(element.get("name")) for element in root.findall("link")]
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is None or not child.get("link"):
            raise ValueError("URDF joint child is missing")
        by_child[str(child.get("link"))] = joint
    roots = sorted(set(links) - set(by_child))
    if roots != ["base_footprint"]:
        raise ValueError(f"unexpected URDF roots: {roots}")

    chain: list[ET.Element] = []
    current = "base_link"
    while current != "base_footprint":
        joint = by_child.get(current)
        if joint is None or joint.get("type") != "fixed":
            raise ValueError(f"invalid fixed root path at {current}")
        chain.append(joint)
        parent = joint.find("parent")
        if parent is None or not parent.get("link"):
            raise ValueError("URDF joint parent is missing")
        current = str(parent.get("link"))

    translation = [0.0, 0.0, 0.0]
    orientation = [0.0, 0.0, 0.0, 1.0]
    joint_names: list[str] = []
    for joint in reversed(chain):
        joint_names.append(str(joint.get("name")))
        origin = joint.find("origin")
        xyz = parse_vector(origin.get("xyz") if origin is not None else None)
        rpy = parse_vector(origin.get("rpy") if origin is not None else None)
        rotated = rotate_vector(orientation, xyz)
        translation = [a + b for a, b in zip(translation, rotated)]
        orientation = multiply_quaternions(orientation, quaternion_from_rpy(rpy))
    return {
        "urdf_robot_name": root.get("name"),
        "root_link": "base_footprint",
        "derived_link": "base_link",
        "fixed_joint_path": joint_names,
        "translation_m": translation,
        "orientation_xyzw": orientation,
    }


def pose_dict(pose: Any) -> dict[str, Any]:
    values = [
        float(pose.position.x), float(pose.position.y), float(pose.position.z),
        float(pose.orientation.x), float(pose.orientation.y),
        float(pose.orientation.z), float(pose.orientation.w),
    ]
    return {
        "position_m": values[:3],
        "orientation_xyzw": normalize_quaternion(values[3:]),
        "all_finite": finite(values),
    }


def twist_dict(twist: Any) -> dict[str, Any]:
    values = [
        float(twist.linear.x), float(twist.linear.y), float(twist.linear.z),
        float(twist.angular.x), float(twist.angular.y), float(twist.angular.z),
    ]
    return {
        "linear_m_s": values[:3],
        "angular_rad_s": values[3:],
        "all_finite": finite(values),
    }


def derive_base_link(root_state: dict[str, Any], binding: dict[str, Any]) -> dict[str, Any]:
    root_pose = root_state["pose_world"]
    root_position = [float(value) for value in root_pose["position_m"]]
    root_orientation = [float(value) for value in root_pose["orientation_xyzw"]]
    translated = rotate_vector(root_orientation, binding["translation_m"])
    position = [a + b for a, b in zip(root_position, translated)]
    orientation = multiply_quaternions(root_orientation, binding["orientation_xyzw"])
    return {
        "monotonic_ns": int(root_state["monotonic_ns"]),
        "name": "derived_from_base_footprint::base_link",
        "pose_world": {
            "position_m": position,
            "orientation_xyzw": orientation,
            "all_finite": finite(position + orientation),
        },
        "twist_world": root_state["twist_world"],
        "derived_from": str(root_state["name"]),
        "fixed_transform_source": "exact_test_urdf",
    }


class GroundFrameObserver(Node):
    def __init__(self, model_name: str, binding: dict[str, Any]) -> None:
        super().__init__("rubiks_ground_frame_observer")
        self.model_name = model_name
        self.binding = binding
        self.started_monotonic_ns = time.monotonic_ns()
        self.latest_model: dict[str, Any] | None = None
        self.latest_base_link: dict[str, Any] | None = None
        self.latest_base_footprint: dict[str, Any] | None = None
        self.base_link_directly_observed = False
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
        self.base_link_directly_observed = False
        if len(footprints) == 1:
            index = footprints[0]
            self.latest_base_footprint = {
                "monotonic_ns": now,
                "name": str(message.name[index]),
                "pose_world": pose_dict(message.pose[index]),
                "twist_world": twist_dict(message.twist[index]),
            }
        if len(base_links) == 1:
            index = base_links[0]
            self.latest_base_link = {
                "monotonic_ns": now,
                "name": str(message.name[index]),
                "pose_world": pose_dict(message.pose[index]),
                "twist_world": twist_dict(message.twist[index]),
            }
            self.base_link_directly_observed = True
        elif self.latest_base_footprint is not None:
            self.latest_base_link = derive_base_link(self.latest_base_footprint, self.binding)

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
            "all_finite": finite(positions),
            "model_state": self.latest_model,
            "base_link_state": self.latest_base_link,
            "base_link_directly_observed": self.base_link_directly_observed,
            "base_footprint_state": self.latest_base_footprint,
        }
        for key in ("model_state", "base_link_state", "base_footprint_state"):
            state = sample[key]
            sample[key + "_age_s"] = (
                None if state is None else (now - int(state["monotonic_ns"])) / 1_000_000_000.0
            )
        self.arm_samples.append(sample)

    def ready(self) -> bool:
        root_observed = self.latest_base_footprint is not None or self.base_link_directly_observed
        return bool(self.arm_samples and self.latest_model and root_observed and self.latest_base_link)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--ready-file", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    binding = root_to_base_link(Path(args.urdf))
    if binding["urdf_robot_name"] != args.model:
        raise ValueError(
            f"observer model/URDF mismatch: {args.model} != {binding['urdf_robot_name']}"
        )
    stop = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    output = Path(args.output)
    ready_file = Path(args.ready_file)
    rclpy.init()
    node = GroundFrameObserver(args.model, binding)
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
        exit_code = 0 if node.ready() else 1
    except Exception as error:
        node.errors.append(f"{type(error).__name__}: {error}")
    finally:
        payload = {
            "schema_version": 3,
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
            "root_to_base_link_urdf_binding": binding,
            "base_link_directly_observed": any(
                sample.get("base_link_directly_observed") is True for sample in node.arm_samples
            ),
            "base_link_available": any(
                sample.get("base_link_state") is not None for sample in node.arm_samples
            ),
            "base_footprint_observed": any(
                sample.get("base_footprint_state") is not None for sample in node.arm_samples
            ),
            "root_observation_policy": (
                "prefer_direct_base_footprint_and_derive_lumped_base_link_from_exact_test_urdf"
            ),
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
