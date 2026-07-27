#!/usr/bin/env python3
"""Render calibrated static-cube contact telemetry into deterministic MP4 evidence."""
from __future__ import annotations

import argparse
import bisect
import json
from pathlib import Path
import subprocess

from render_gripper_command_video import fill_rect, line, numeric, outline


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
        "gazebo_link_frame_separation_m",
        "link_frame_separation_m",
        "inner_face_opening_m",
    ):
        a = before.get(key)
        b = after.get(key)
        if numeric(a) and numeric(b):
            result[key] = (1.0 - fraction) * float(a) + fraction * float(b)
        elif fraction >= 0.5:
            result[key] = b
    result["t_s"] = t
    if fraction >= 0.5:
        for key in ("phase", "command_target_m", "contact_active"):
            result[key] = after.get(key)
    return result


def phase_color(phase: str, contact_active: bool) -> tuple[int, int, int]:
    if contact_active or phase == "cube_contact_hold":
        return (225, 70, 70)
    if phase.startswith("close_step_"):
        return (245, 170, 45)
    if phase in {"open_before_cube", "reopen_after_cube_contact"}:
        return (55, 200, 110)
    if phase == "complete":
        return (80, 145, 235)
    return (150, 160, 175)


def render_frame(
    samples: list[dict], state: dict, t: float, duration: float,
    width: int, height: int, cube_width_m: float, offset_m: float,
) -> bytes:
    background = (16, 21, 29)
    panel = (27, 35, 47)
    grid = (57, 69, 84)
    white = (238, 242, 247)
    yellow = (245, 205, 45)
    cyan = (55, 200, 220)
    physical = (185, 105, 235)
    cube_color = (230, 125, 35)
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
    contact_active = bool(state.get("contact_active"))
    tint = phase_color(phase, contact_active)
    fill_rect(buf, width, height, left_panel[0], left_panel[1], left_panel[2], left_panel[1] + 12, tint)

    center_x = (left_panel[0] + left_panel[2]) // 2
    center_y = (left_panel[1] + left_panel[3]) // 2
    usable_width = left_panel[2] - left_panel[0] - 70

    def opening_px(metres: float) -> int:
        return int(metres / 0.09 * usable_width)

    for value in (0.019, 0.057, 0.060, 0.077, 0.09):
        half = opening_px(value) // 2
        line(buf, width, height, center_x - half, center_y - 145, center_x - half, center_y + 145, grid, 1)
        line(buf, width, height, center_x + half, center_y - 145, center_x + half, center_y + 145, grid, 1)

    cube_half = opening_px(cube_width_m) // 2
    cube_rect = (center_x - cube_half, center_y - 75, center_x + cube_half, center_y + 75)
    fill_rect(buf, width, height, *cube_rect, cube_color)
    outline(buf, width, height, cube_rect, white if contact_active else yellow, 3)

    separation_value = state.get("link_frame_separation_m")
    opening_value = state.get("inner_face_opening_m")
    if not numeric(separation_value) or not numeric(opening_value):
        raise ValueError("sample lacks calibrated separation/opening")
    separation = float(separation_value)
    inner_opening = float(opening_value)
    link_half = opening_px(separation) // 2
    inner_half = opening_px(inner_opening) // 2
    jaw_top = center_y - 120
    jaw_bottom = center_y + 120
    for frame_x in (center_x - link_half, center_x + link_half):
        fill_rect(buf, width, height, frame_x - 10, jaw_top, frame_x + 11, jaw_bottom, physical)
    for face_x in (center_x - inner_half, center_x + inner_half):
        line(buf, width, height, face_x, jaw_top, face_x, jaw_bottom, tint if contact_active else white, 5)
    line(buf, width, height, center_x, center_y - 155, center_x, center_y + 155, grid, 2)

    target = state.get("command_target_m")
    if numeric(target):
        target_opening = 0.042 + 2.0 * float(target) - offset_m
        target_half = opening_px(target_opening) // 2
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
        y = graph_top + index * (graph_bottom - graph_top) // 9
        line(buf, width, height, graph_left, y, graph_right, y, grid, 1)

    def graph_point(sample: dict) -> tuple[int, int]:
        value = sample.get("inner_face_opening_m")
        if not numeric(value):
            raise ValueError("sample lacks calibrated inner opening")
        x = graph_left + int(float(sample["t_s"]) / max(duration, 1e-9) * (graph_right - graph_left))
        y = graph_bottom - int(float(value) / 0.09 * (graph_bottom - graph_top))
        return x, y

    visible = [sample for sample in samples if float(sample["t_s"]) <= t and numeric(sample.get("inner_face_opening_m"))]
    for before, after in zip(visible, visible[1:]):
        a = graph_point(before)
        b = graph_point(after)
        line(buf, width, height, a[0], a[1], b[0], b[1], cyan, 3)
    current = graph_point(state)
    fill_rect(buf, width, height, current[0] - 5, current[1] - 5, current[0] + 6, current[1] + 6, tint)

    cube_y = graph_bottom - int(cube_width_m / 0.09 * (graph_bottom - graph_top))
    line(buf, width, height, graph_left, cube_y, graph_right, cube_y, cube_color, 3)
    progress_y = height - 21
    fill_rect(buf, width, height, margin, progress_y, width - margin, progress_y + 8, grid)
    fill_rect(buf, width, height, margin, progress_y, margin + int((width - 2 * margin) * min(1.0, max(0.0, t / duration))), progress_y + 8, tint)
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
    cube_width = float(telemetry["cube_width_m"])
    offset = float(telemetry["combined_inner_face_offset_m"])
    samples = [sample for sample in telemetry.get("samples", []) if numeric(sample.get("inner_face_opening_m"))]
    if len(samples) < 3:
        raise SystemExit("telemetry has too few calibrated samples")
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
        process.stdin.write(render_frame(samples, state, t, duration, args.width, args.height, cube_width, offset))
    process.stdin.close()
    return_code = process.wait()
    if return_code != 0:
        raise SystemExit(f"ffmpeg exited with {return_code}")
    print(json.dumps({
        "duration_s": duration,
        "frames": frame_count,
        "source": args.telemetry,
        "output": args.output,
        "cube_width_m": cube_width,
        "calibrated_inner_faces_rendered": True,
        "raw_gazebo_gui_video": False,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
