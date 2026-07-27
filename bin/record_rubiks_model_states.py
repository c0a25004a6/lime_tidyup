#!/usr/bin/env python3
"""Record one Gazebo model's pose/twist trajectory as JSON."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates


class Recorder(Node):
    def __init__(self, entity: str, duration: float, max_rate: float) -> None:
        super().__init__("rubiks_model_state_recorder")
        self.entity = entity
        self.duration = duration
        self.min_period = 1.0 / max_rate
        self.first_seen: float | None = None
        self.last_sample = -1e9
        self.samples: list[dict] = []
        self.create_subscription(ModelStates, "/model_states", self.on_states, 20)

    def on_states(self, msg: ModelStates) -> None:
        if self.entity not in msg.name:
            return
        now = time.monotonic()
        if self.first_seen is None:
            self.first_seen = now
        t = now - self.first_seen
        if t - self.last_sample < self.min_period and t < self.duration:
            return
        idx = msg.name.index(self.entity)
        pose = msg.pose[idx]
        twist = msg.twist[idx]
        self.samples.append({
            "t_s": t,
            "position_m": [pose.position.x, pose.position.y, pose.position.z],
            "orientation_xyzw": [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            "linear_velocity_m_s": [twist.linear.x, twist.linear.y, twist.linear.z],
            "angular_velocity_rad_s": [twist.angular.x, twist.angular.y, twist.angular.z],
        })
        self.last_sample = t


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--entity", default="rubiks_cube_settle")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--max-rate", type=float, default=30.0)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--ready-file")
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=None)
    node = Recorder(args.entity, args.duration, args.max_rate)
    if args.ready_file:
        Path(args.ready_file).touch()
    deadline = time.monotonic() + args.wait_timeout
    try:
        while rclpy.ok() and node.first_seen is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.first_seen is None:
            raise RuntimeError(f"entity {args.entity!r} was not observed")
        while rclpy.ok() and node.samples[-1]["t_s"] < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - node.first_seen > args.duration + 2.0:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(node.samples) < 3:
        raise RuntimeError(f"insufficient samples: {len(node.samples)}")
    payload = {
        "schema_version": 1,
        "entity": args.entity,
        "duration_requested_s": args.duration,
        "sample_count": len(node.samples),
        "samples": node.samples,
    }
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: payload[k] for k in payload if k != "samples"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
