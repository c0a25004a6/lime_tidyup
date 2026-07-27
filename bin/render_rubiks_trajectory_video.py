#!/usr/bin/env python3
"""Render Gazebo model-state telemetry into a deterministic MP4 evidence video."""
from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
import subprocess


def fill_rect(buf: bytearray, width: int, height: int, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int]) -> None:
    x0, x1 = sorted((max(0, x0), min(width, x1)))
    y0, y1 = sorted((max(0, y0), min(height, y1)))
    row = bytes(color) * max(0, x1 - x0)
    for y in range(y0, y1):
        start = (y * width + x0) * 3
        buf[start:start + len(row)] = row


def line(buf: bytearray, width: int, height: int, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int], thickness: int = 2) -> None:
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        fill_rect(buf, width, height, x0 - thickness // 2, y0 - thickness // 2, x0 + thickness // 2 + 1, y0 + thickness // 2 + 1, color)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def polygon_outline(buf: bytearray, width: int, height: int, pts: list[tuple[int, int]], color: tuple[int, int, int], thickness: int = 3) -> None:
    for a, b in zip(pts, pts[1:] + pts[:1]):
        line(buf, width, height, a[0], a[1], b[0], b[1], color, thickness)


def yaw_from_quaternion(q: list[float]) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def interpolate(samples: list[dict], times: list[float], t: float) -> dict:
    idx = bisect.bisect_left(times, t)
    if idx <= 0:
        return samples[0]
    if idx >= len(samples):
        return samples[-1]
    a, b = samples[idx - 1], samples[idx]
    span = max(1e-9, b["t_s"] - a["t_s"])
    u = (t - a["t_s"]) / span
    result = {"t_s": t}
    for key in ("position_m", "orientation_xyzw", "linear_velocity_m_s", "angular_velocity_rad_s"):
        result[key] = [(1 - u) * x + u * y for x, y in zip(a[key], b[key])]
    return result


def render_frame(state: dict, edge: float, progress: float, width: int, height: int) -> bytes:
    bg = (17, 22, 30)
    panel = (28, 36, 48)
    grid = (58, 70, 86)
    ground = (120, 132, 145)
    white = (238, 242, 247)
    red = (220, 55, 55)
    green = (45, 190, 90)
    blue = (55, 110, 220)
    yellow = (245, 205, 40)
    buf = bytearray(bytes(bg) * (width * height))
    margin = 40
    gap = 24
    panel_w = (width - 2 * margin - gap) // 2
    panel_h = height - 2 * margin - 34
    left = (margin, margin, margin + panel_w, margin + panel_h)
    right = (margin + panel_w + gap, margin, width - margin, margin + panel_h)
    for p in (left, right):
        fill_rect(buf, width, height, *p, panel)
        polygon_outline(buf, width, height, [(p[0], p[1]), (p[2], p[1]), (p[2], p[3]), (p[0], p[3])], grid, 2)

    def side_px(x: float, z: float) -> tuple[int, int]:
        px = left[0] + int((x + 0.1) / 1.0 * (left[2] - left[0]))
        py = left[3] - int(z / 0.5 * (left[3] - left[1]))
        return px, py

    def top_px(x: float, y: float) -> tuple[int, int]:
        px = right[0] + int((x + 0.1) / 1.0 * (right[2] - right[0]))
        py = right[3] - int((y + 0.5) / 1.0 * (right[3] - right[1]))
        return px, py

    for i in range(11):
        x = -0.1 + i * 0.1
        a, b = side_px(x, 0), side_px(x, 0.5)
        line(buf, width, height, a[0], a[1], b[0], b[1], grid, 1)
        a, b = top_px(x, -0.5), top_px(x, 0.5)
        line(buf, width, height, a[0], a[1], b[0], b[1], grid, 1)
    for i in range(6):
        z = i * 0.1
        a, b = side_px(-0.1, z), side_px(0.9, z)
        line(buf, width, height, a[0], a[1], b[0], b[1], grid, 1)
    for i in range(11):
        y = -0.5 + i * 0.1
        a, b = top_px(-0.1, y), top_px(0.9, y)
        line(buf, width, height, a[0], a[1], b[0], b[1], grid, 1)

    a, b = side_px(-0.1, 0), side_px(0.9, 0)
    line(buf, width, height, a[0], a[1], b[0], b[1], ground, 4)

    x, y, z = state["position_m"]
    side_center = side_px(x, z)
    side_half_x = max(5, int(edge * (left[2] - left[0]) / 2))
    side_half_z = max(5, int(edge / 0.5 * (left[3] - left[1]) / 2))
    fill_rect(buf, width, height, side_center[0] - side_half_x, side_center[1] - side_half_z, side_center[0], side_center[1] + side_half_z, red)
    fill_rect(buf, width, height, side_center[0], side_center[1] - side_half_z, side_center[0] + side_half_x, side_center[1] + side_half_z, blue)
    polygon_outline(buf, width, height, [
        (side_center[0] - side_half_x, side_center[1] - side_half_z),
        (side_center[0] + side_half_x, side_center[1] - side_half_z),
        (side_center[0] + side_half_x, side_center[1] + side_half_z),
        (side_center[0] - side_half_x, side_center[1] + side_half_z),
    ], white, 2)

    yaw = yaw_from_quaternion(state["orientation_xyzw"])
    half = edge / 2
    corners = []
    for lx, ly in [(-half, -half), (half, -half), (half, half), (-half, half)]:
        wx = x + lx * math.cos(yaw) - ly * math.sin(yaw)
        wy = y + lx * math.sin(yaw) + ly * math.cos(yaw)
        corners.append(top_px(wx, wy))
    polygon_outline(buf, width, height, corners, white, 5)
    c = top_px(x, y)
    line(buf, width, height, c[0], c[1], c[0] + int(35 * math.cos(yaw)), c[1] - int(35 * math.sin(yaw)), green, 4)
    line(buf, width, height, c[0], c[1], c[0] - int(28 * math.sin(yaw)), c[1] - int(28 * math.cos(yaw)), yellow, 4)

    bar_y0 = height - 22
    fill_rect(buf, width, height, margin, bar_y0, width - margin, bar_y0 + 8, grid)
    fill_rect(buf, width, height, margin, bar_y0, margin + int((width - 2 * margin) * max(0.0, min(1.0, progress))), bar_y0 + 8, green)
    return bytes(buf)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--parameters", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    args = parser.parse_args()

    trajectory = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    parameters = json.loads(Path(args.parameters).read_text(encoding="utf-8"))
    samples = trajectory["samples"]
    if len(samples) < 3:
        raise SystemExit("trajectory has too few samples")
    times = [float(s["t_s"]) for s in samples]
    duration = max(times[-1], 1.0)
    edge = float(parameters["model"]["edge_length_m"])
    frame_count = max(1, round(duration * args.fps))

    command = [
        "ffmpeg", "-y", "-loglevel", "warning",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{args.width}x{args.height}", "-r", str(args.fps), "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "22",
        "-pix_fmt", "yuv420p", args.output,
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    assert process.stdin is not None
    for index in range(frame_count):
        t = min(duration, index / args.fps)
        state = interpolate(samples, times, t)
        process.stdin.write(render_frame(state, edge, t / duration, args.width, args.height))
    process.stdin.close()
    return_code = process.wait()
    if return_code != 0:
        raise SystemExit(f"ffmpeg exited with {return_code}")
    print(json.dumps({"duration_s": duration, "frames": frame_count, "output": args.output}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
