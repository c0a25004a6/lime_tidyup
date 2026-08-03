#!/usr/bin/env python3
"""Run the existing ground-frame observer with reliable JointState delivery."""
from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any

from gazebo_msgs.msg import LinkStates, ModelStates
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

BASE_OBSERVER = Path("/root/bin/observe_rubiks_ground_frames.py")


def load_base() -> Any:
    spec = importlib.util.spec_from_file_location("rubiks_ground_frame_base", BASE_OBSERVER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {BASE_OBSERVER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base = load_base()


class ReliableGroundFrameObserver(base.GroundFrameObserver):
    """Preserve base callbacks while preventing best-effort JointState loss."""

    def __init__(self, model_name: str, binding: dict[str, Any]) -> None:
        Node.__init__(self, "rubiks_ground_frame_observer_reliable")
        self.model_name = model_name
        self.binding = binding
        self.started_monotonic_ns = base.time.monotonic_ns()
        self.latest_model: dict[str, Any] | None = None
        self.latest_base_link: dict[str, Any] | None = None
        self.latest_base_footprint: dict[str, Any] | None = None
        self.base_link_directly_observed = False
        self.model_names: list[str] = []
        self.link_names: list[str] = []
        self.arm_samples: list[dict[str, Any]] = []
        self.errors: list[str] = []

        gazebo_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        joint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=4096,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.model_sub = self.create_subscription(
            ModelStates, "/model_states", self._models, gazebo_qos
        )
        self.link_sub = self.create_subscription(
            LinkStates, "/link_states", self._links, gazebo_qos
        )
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self._joints, joint_qos
        )


base.GroundFrameObserver = ReliableGroundFrameObserver
raise SystemExit(base.main())
