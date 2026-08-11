#!/usr/bin/env python3
"""Persist a ROS 2 CompressedImage stream as numbered JPEG evidence frames.

The recorder deliberately does no image decode or re-encode. Gazebo's ROS camera
plugin / image_transport produces the compressed JPEG bytes; this process only
writes those bytes plus timestamp metadata. It never publishes robot commands.
"""
from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class Recorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rubiks_gazebo_compressed_frame_recorder")
        self.args = args
        self.output_dir = Path(args.output_dir)
        self.metadata_path = Path(args.metadata)
        self.ready_path = Path(args.ready_file)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        self.ready_path.parent.mkdir(parents=True, exist_ok=True)
        self.ready_path.unlink(missing_ok=True)

        for old in self.output_dir.glob("frame_*.jpg"):
            old.unlink()

        self.frame_count = 0
        self.first_stamp_s: float | None = None
        self.last_stamp_s: float | None = None
        self.first_wall_s: float | None = None
        self.last_wall_s: float | None = None
        self.formats: set[str] = set()
        self.error: str | None = None
        self.timestamps: list[float] = []
        self.create_subscription(
            CompressedImage,
            args.topic,
            self.on_image,
            qos_profile_sensor_data,
        )

    def on_image(self, msg: CompressedImage) -> None:
        if self.error is not None:
            return
        try:
            payload = bytes(msg.data)
            if len(payload) < 4 or payload[:2] != b"\xff\xd8" or payload[-2:] != b"\xff\xd9":
                raise ValueError(
                    f"compressed frame is not a complete JPEG: bytes={len(payload)}, format={msg.format!r}"
                )
            fmt = str(msg.format)
            if "jpeg" not in fmt.lower() and "jpg" not in fmt.lower():
                raise ValueError(f"unexpected compressed image format: {fmt!r}")

            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            wall = time.monotonic()
            frame_path = self.output_dir / f"frame_{self.frame_count:06d}.jpg"
            frame_path.write_bytes(payload)
            self.timestamps.append(stamp)
            self.formats.add(fmt)
            if self.frame_count == 0:
                self.first_stamp_s = stamp
                self.first_wall_s = wall
                self.ready_path.write_text("first_compressed_frame_saved\n", encoding="utf-8")
            self.last_stamp_s = stamp
            self.last_wall_s = wall
            self.frame_count += 1
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(self.error)

    def finish(self) -> dict[str, object]:
        sim_span = 0.0
        if self.first_stamp_s is not None and self.last_stamp_s is not None:
            sim_span = max(0.0, self.last_stamp_s - self.first_stamp_s)
        wall_span = 0.0
        if self.first_wall_s is not None and self.last_wall_s is not None:
            wall_span = max(0.0, self.last_wall_s - self.first_wall_s)

        positive_intervals = [
            b - a for a, b in zip(self.timestamps, self.timestamps[1:]) if b > a
        ]
        mean_interval = (
            sum(positive_intervals) / len(positive_intervals)
            if positive_intervals
            else None
        )
        simulated_fps = (
            (self.frame_count - 1) / sim_span
            if self.frame_count > 1 and sim_span > 0.0
            else 0.0
        )
        receive_fps_wall = (
            (self.frame_count - 1) / wall_span
            if self.frame_count > 1 and wall_span > 0.0
            else 0.0
        )
        payload: dict[str, object] = {
            "schema_version": 1,
            "capture_source": "GAZEBO_ROS_CAMERA_COMPRESSED_JPEG",
            "topic": self.args.topic,
            "frame_count": self.frame_count,
            "formats": sorted(self.formats),
            "first_ros_stamp_s": self.first_stamp_s,
            "last_ros_stamp_s": self.last_stamp_s,
            "sim_span_s": sim_span,
            "wall_span_s": wall_span,
            "mean_positive_ros_interval_s": mean_interval,
            "simulated_fps": simulated_fps,
            "wall_receive_fps": receive_fps_wall,
            "error": self.error,
            "passed": self.error is None and self.frame_count > 0,
        }
        self.metadata_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--ready-file", required=True)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = Recorder(args)
    stopping = False

    def stop_handler(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        result = node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
