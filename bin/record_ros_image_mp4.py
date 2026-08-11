#!/usr/bin/env python3
"""Record a ROS 2 sensor_msgs/Image topic to MP4 with machine-readable metadata.

This is intentionally test-only evidence plumbing. It does not publish commands
or mutate simulation state.
"""
from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_to_bgr(msg: Image) -> np.ndarray:
    encoding = str(msg.encoding).lower()
    if encoding in {"rgb8", "bgr8"}:
        channels = 3
    elif encoding in {"rgba8", "bgra8"}:
        channels = 4
    elif encoding == "mono8":
        channels = 1
    else:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    minimum_step = int(msg.width) * channels
    if int(msg.step) < minimum_step:
        raise ValueError(
            f"invalid step={msg.step} for width={msg.width}, channels={channels}"
        )
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = int(msg.height) * int(msg.step)
    if raw.size < expected:
        raise ValueError(f"short image buffer: {raw.size} < {expected}")
    rows = raw[:expected].reshape((int(msg.height), int(msg.step)))
    packed = rows[:, :minimum_step]

    if channels == 1:
        gray = packed.reshape((int(msg.height), int(msg.width)))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    image = packed.reshape((int(msg.height), int(msg.width), channels))
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


class Recorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rubiks_physics_video_recorder")
        self.args = args
        self.video_path = Path(args.output)
        self.metadata_path = Path(args.metadata)
        self.ready_path = Path(args.ready_file) if args.ready_file else None
        self.video_path.parent.mkdir(parents=True, exist_ok=True)
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        if self.ready_path:
            self.ready_path.parent.mkdir(parents=True, exist_ok=True)
            self.ready_path.unlink(missing_ok=True)

        self.writer: cv2.VideoWriter | None = None
        self.frame_count = 0
        self.first_monotonic_s: float | None = None
        self.last_monotonic_s: float | None = None
        self.first_ros_stamp_s: float | None = None
        self.last_ros_stamp_s: float | None = None
        self.width: int | None = None
        self.height: int | None = None
        self.encoding: str | None = None
        self.error: str | None = None
        self.started_wall_s = time.time()
        self.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, msg: Image) -> None:
        if self.error is not None:
            return
        try:
            frame = image_to_bgr(msg)
            height, width = frame.shape[:2]
            if self.writer is None:
                fourcc = cv2.VideoWriter_fourcc(*self.args.fourcc)
                writer = cv2.VideoWriter(
                    str(self.video_path), fourcc, float(self.args.fps), (width, height)
                )
                if not writer.isOpened():
                    raise RuntimeError(
                        f"VideoWriter failed to open {self.video_path} with {self.args.fourcc}"
                    )
                self.writer = writer
                self.width = width
                self.height = height
                self.encoding = str(msg.encoding)
            elif width != self.width or height != self.height:
                raise RuntimeError(
                    f"camera resolution changed: {width}x{height} != {self.width}x{self.height}"
                )

            now = time.monotonic()
            ros_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if self.first_monotonic_s is None:
                self.first_monotonic_s = now
                self.first_ros_stamp_s = ros_stamp
            self.last_monotonic_s = now
            self.last_ros_stamp_s = ros_stamp

            # A small overlay makes artifact review self-contained while preserving
            # the underlying Gazebo frame content.
            elapsed = now - self.first_monotonic_s
            cv2.putText(
                frame,
                f"frame={self.frame_count:05d}  t={elapsed:7.3f}s",
                (12, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            self.writer.write(frame)
            self.frame_count += 1
            if self.frame_count == 1 and self.ready_path:
                self.ready_path.write_text("first_frame_recorded\n", encoding="utf-8")
        except Exception as exc:  # recorder failure must be surfaced in metadata
            self.error = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(self.error)

    def close(self) -> dict[str, object]:
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        self.finished_wall_s = time.time()
        observed = 0.0
        if self.first_monotonic_s is not None and self.last_monotonic_s is not None:
            observed = max(0.0, self.last_monotonic_s - self.first_monotonic_s)
        effective_fps = self.frame_count / observed if observed > 0 else 0.0
        payload: dict[str, object] = {
            "schema_version": 1,
            "topic": self.args.topic,
            "video_path": str(self.video_path),
            "codec_fourcc": self.args.fourcc,
            "requested_fps": float(self.args.fps),
            "frame_count": self.frame_count,
            "width": self.width,
            "height": self.height,
            "encoding": self.encoding,
            "observed_seconds": observed,
            "effective_receive_fps": effective_fps,
            "first_ros_stamp_s": self.first_ros_stamp_s,
            "last_ros_stamp_s": self.last_ros_stamp_s,
            "started_wall_s": self.started_wall_s,
            "finished_wall_s": self.finished_wall_s,
            "error": self.error,
            "passed": (
                self.error is None
                and self.frame_count > 0
                and self.video_path.is_file()
                and self.video_path.stat().st_size > 0
            ),
        }
        self.metadata_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--ready-file")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="mp4v")
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
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        payload = node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(payload, indent=2))
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
