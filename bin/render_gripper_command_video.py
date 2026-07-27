#!/usr/bin/env python3
"""Render gripper telemetry into a deterministic MP4 evidence video."""
from __future__ import annotations

import argparse
import bisect
import json
from pathlib import Path
import subprocess


def fill_rect(
    buf: bytearray,
    width: int,
    height: int,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    color: tuple[int, int, int],
) -> None:
    x0, x1 = sorted((max(0, x0), min(width, x1)))
    y0, y1 = sorted((max(0, y0), min(height, y1)))
    row = bytes(color) * max(0, x1 - x0)
    for y in range(y0, y1):
        start = (y * width + x0) * 3
        buf[start:start + len(row)] = row


def line(
    buf: bytearray,
    width: int,
    height: int,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    color: tuple[int, int, int],
    thickness: int = 2,
) -> None:
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        fill_rect(
            buf, width, height,
            x0 - thickness // 2, y0 - thickness // 2,
            x0 + thickness // 2 + 1, y0 + thickness // 2 + 1,
            color,
        )
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def outline(
    buf: bytearray,
    width: int,
    height: int,
    rect: tuple[int, int, int, int],
    color: tuple[int, int, int],
    thickness: int = 2,
) -> None:
    x0, y0, x1, y1 = rect
    line(buf, width, height, x0, y0, x1, y0, color, thickness)
    line(buf, width, height, x1, y0, x1, y1, color, thickness)
    line(buf, width, height, x1, y1, x0, y1, color, thickness)
    line(buf, width, height, x0, y1, x0, y0, color, thickness)


def numeric(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def interpolate(samples: list[dict], times: list[float], t: float) -> dict:
    index = bisect.bisect_left(times, t)
    if index <= 0:
        return samples[0]
    if index >= len(samples):
        return samples[-1]
    before, after = samples[index - 1], samples[index]
    span = max(1e-9, float(after["t_s"]) - float(before["t_s"]))
    fraction = (t - float(before["t_s"])) / span
    result = dict(before)
    for key in (
        "left_joint_position_m",
        "right_joint_position_m",
        "joint_derived_link_frame_separation_m",
        "gazebo_link_frame_separation_m",
        "link_frame_separation_m",
    ):
        before_value = before.get(key)
        after_value = after.get(key)
        if numeric(before_value) and numeric(after_value):
            result[key] = (1 - fraction) * float(before_value) + fraction * float(after_value)
        elif fraction >= 0.5:
            result[key] = after_value
    result["t_s"] = t
    if fraction >= 0.5:
        result["phase"] = after.get("phase", before.get("phase", "initial"))
        result["command_target_m"] = after.get("command_target_m")
        result["link_frame_separation_source"] = after.get("link_frame_separation_source")
    return result


def phase_color(phase: str) -> tuple[int, int, int]:
    if phase == "close":
        return (225, 70, 70)
    if phase in {"open_1", "open_2"}:
        return (55, 200, 110)
    if phase == "complete":
        return (80, 145, 235)
    return (150, 160, 175)


def render_frame(
    samples: list[dict],
    state: dict,
    t: float,
    duration: float,
    width: int,
    height: int,
) -> bytes:
    background = (16, 21, 29)
    panel = (27, 35, 47)
    grid = (57, 69, 84)
    white = (238, 242, 247)
    yellow = (245, 205, 45)
    cyan = (55, 200, 220)
    physical = (185, 105, 235)
    buf = bytearray(bytes(background) * (width * height))

    margin = 38
    gap = 26
    panel_width = (width - margin * 2 - gap) // 2
    panel_height = height - margin * 2 - 34
    left_panel = (margin, margin, margin + panel_width, margin + panel_height)
    right_panel = (margin + panel_width + gap, margin, width - margin, margin + panel_height)
    for rect in (left_panel, right_panel):
        fill_rect(buf, width, height, *rect, panel)
        outline(buf, width, height, rect, grid, 2)

    phase = str(state.get("phase", "initial"))
    phase_tint = phase_color(phase)
    fill_rect(
        buf, width, height,
        left_panel[0], left_panel[1], left_panel[2], left_panel[1] + 12,
        phase_tint,
    )

    center_x = (left_panel[0] + left_panel[2]) // 2
    center_y = (left_panel[1] + left_panel[3]) // 2
    usable_width = left_panel[2] - left_panel[0] - 70

    def separation_x(metres: float) -> int:
        return int(metres / 0.09 * usable_width)

    for value in (0.0, 0.022, 0.042, 0.080, 0.09):
        half = separation_x(value) // 2
        line(buf, width, height, center_x - half, center_y - 130, center_x - half, center_y + 130, grid, 1)
        line(buf, width, height, center_x + half, center_y - 130, center_x + half, center_y + 130, grid, 1)

    separation_value = state.get("link_frame_separation_m")
    if not numeric(separation_value):
        raise ValueError("sample does not contain an observed link-frame separation")
    separation = float(separation_value)
    half_px = separation_x(separation) // 2
    left_frame_x = center_x + half_px
    right_frame_x = center_x - half_px
    jaw_top = center_y - 115
    jaw_bottom = center_y + 115
    observed_color = (
        physical
        if state.get("link_frame_separation_source") == "gazebo_link_states"
        else phase_tint
    )
    fill_rect(buf, width, height, left_frame_x - 13, jaw_top, left_frame_x + 13, jaw_bottom, observed_color)
    fill_rect(buf, width, height, right_frame_x - 13, jaw_top, right_frame_x + 13, jaw_bottom, observed_color)
    line(buf, width, height, right_frame_x, center_y, left_frame_x, center_y, white, 3)
    line(buf, width, height, center_x, center_y - 145, center_x, center_y + 145, grid, 2)

    target = state.get("command_target_m")
    if target is not None:
        target_separation = 0.042 + 2 * float(target)
        target_half = separation_x(target_separation) // 2
        for marker_x in (center_x - target_half, center_x + target_half):
            for y in range(jaw_top, jaw_bottom, 14):
                line(buf, width, height, marker_x, y, marker_x, min(y + 7, jaw_bottom), yellow, 2)

    graph_left = right_panel[0] + 28
    graph_right = right_panel[2] - 22
    graph_top = right_panel[1] + 32
    graph_bottom = right_panel[3] - 35
    for index in range(10):
        x = graph_left + index * (graph_right - graph_left) // 9
        line(buf, width, height, x, graph_top, x, graph_bottom, grid, 1)
    for index in range(10):
        y = graph_top + index * (graph_bottom - graph_top) // 9
        line(buf, width, height, graph_left, y, graph_right, y, grid, 1)

    def graph_point(sample: dict) -> tuple[int, int]:
        sample_t = float(sample["t_s"])
        value = sample.get("link_frame_separation_m")
        if not numeric(value):
            raise ValueError("sample does not contain an observed separation")
        x = graph_left + int(sample_t / max(duration, 1e-9) * (graph_right - graph_left))
        y = graph_bottom - int(float(value) / 0.09 * (graph_bottom - graph_top))
        return x, y

    visible = [
        sample for sample in samples
        if float(sample["t_s"]) <= t and numeric(sample.get("link_frame_separation_m"))
    ]
    for before, after in zip(visible, visible[1:]):
        a = graph_point(before)
        b = graph_point(after)
        line(buf, width, height, a[0], a[1], b[0], b[1], cyan, 3)
    current = graph_point(state)
    fill_rect(buf, width, height, current[0] - 5, current[1] - 5, current[0] + 6, current[1] + 6, observed_color)

    for separation_target in (0.022, 0.080):
        y = graph_bottom - int(separation_target / 0.09 * (graph_bottom - graph_top))
        line(buf, width, height, graph_left, y, graph_right, y, yellow, 1)

    progress_y = height - 21
    fill_rect(buf, width, height, margin, progress_y, width - margin, progress_y + 8, grid)
    fill_rect(
        buf, width, height,
        margin, progress_y,
        margin + int((width - 2 * margin) * min(1.0, max(0.0, t / duration))),
        progress_y + 8,
        phase_tint,
    )
    return bytes(buf)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--telemetry", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    args = parser.parse_args()

    telemetry = json.loads(Path(args.telemetry).read_text(encoding="utf-8"))
    samples = [
        sample for sample in telemetry.get("samples", [])
        if numeric(sample.get("link_frame_separation_m"))
    ]
    if len(samples) < 3:
        raise SystemExit("telemetry has too few samples with observed separation")
    times = [float(sample["t_s"]) for sample in samples]
    duration = max(times[-1], 1.0)
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
    for frame_index in range(frame_count):
        t = min(duration, frame_index / args.fps)
        state = interpolate(samples, times, t)
        process.stdin.write(render_frame(samples, state, t, duration, args.width, args.height))
    process.stdin.close()
    return_code = process.wait()
    if return_code != 0:
        raise SystemExit(f"ffmpeg exited with {return_code}")
    print(json.dumps({
        "duration_s": duration,
        "frames": frame_count,
        "source": args.telemetry,
        "output": args.output,
        "inner_face_opening_rendered": False,
        "separation_sources": sorted({
            sample.get("link_frame_separation_source") for sample in samples
            if sample.get("link_frame_separation_source")
        }),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
