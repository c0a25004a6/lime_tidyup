#!/usr/bin/env python3
"""Audit that accepted Rubik contacts are side-dominant and bilateral."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics


def side_metrics(normals: list[list[float]], positions: list[list[float]]) -> dict:
    if not normals or not positions:
        raise ValueError("side has no contact normal/position evidence")
    mean = [statistics.mean(float(n[index]) for n in normals) for index in range(3)]
    mean_abs = [statistics.mean(abs(float(n[index])) for n in normals) for index in range(3)]
    y_positions = [float(position[1]) for position in positions]
    return {
        "normal_count": len(normals),
        "position_count": len(positions),
        "mean_normal_xyz": mean,
        "mean_abs_normal_xyz": mean_abs,
        "mean_contact_y_m": statistics.mean(y_positions),
        "min_contact_y_m": min(y_positions),
        "max_contact_y_m": max(y_positions),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--telemetry", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    telemetry = json.loads(Path(args.telemetry).read_text(encoding="utf-8"))
    left_normals: list[list[float]] = []
    right_normals: list[list[float]] = []
    left_positions: list[list[float]] = []
    right_positions: list[list[float]] = []
    dual_message_count = 0

    for message in telemetry.get("contact_messages", []):
        if not message.get("dual_contact"):
            continue
        dual_message_count += 1
        for state in message.get("states", []):
            pair = f"{state.get('collision1', '')} {state.get('collision2', '')}"
            normals = [[float(value) for value in normal] for normal in state.get("contact_normals", [])]
            positions = [[float(value) for value in position] for position in state.get("contact_positions_m", [])]
            if "gripper_left_link" in pair:
                left_normals.extend(normals)
                left_positions.extend(positions)
            if "gripper_right_link" in pair:
                right_normals.extend(normals)
                right_positions.extend(positions)

    errors: list[str] = []
    try:
        left = side_metrics(left_normals, left_positions)
        right = side_metrics(right_normals, right_positions)
    except ValueError as exc:
        left = {}
        right = {}
        errors.append(str(exc))

    if dual_message_count < 1:
        errors.append("no dual-contact messages")

    if left and right:
        for name, metrics, expected_sign in (
            ("left", left, -1.0),
            ("right", right, 1.0),
        ):
            mean = metrics["mean_normal_xyz"]
            mean_abs = metrics["mean_abs_normal_xyz"]
            if expected_sign * float(mean[1]) < 0.75:
                errors.append(f"{name} Y normal does not point toward its finger: {mean[1]}")
            if float(mean_abs[1]) <= float(mean_abs[2]):
                errors.append(f"{name} contact is not side-dominant: |Y|={mean_abs[1]} |Z|={mean_abs[2]}")
            if float(mean_abs[1]) < 0.75:
                errors.append(f"{name} lateral normal component is too small: {mean_abs[1]}")
            if float(mean_abs[2]) > 0.65:
                errors.append(f"{name} vertical normal component is too large: {mean_abs[2]}")
            if float(mean_abs[0]) > 0.10:
                errors.append(f"{name} depth-axis normal component is too large: {mean_abs[0]}")

        if not -0.030 <= float(left["mean_contact_y_m"]) <= -0.027:
            errors.append(f"left contact is not on the -Y cube side: {left['mean_contact_y_m']}")
        if not 0.027 <= float(right["mean_contact_y_m"]) <= 0.030:
            errors.append(f"right contact is not on the +Y cube side: {right['mean_contact_y_m']}")

    result = {
        "schema_version": 1,
        "audit_type": "rubiks_static_cube_bilateral_side_contact_normals",
        "passed": not errors,
        "errors": errors,
        "dual_contact_message_count": dual_message_count,
        "left": left,
        "right": right,
        "side_dominance_definition": "mean_abs_y > mean_abs_z, mean_abs_y >= 0.75, mean_abs_z <= 0.65",
        "raw_gazebo_gui_video": False,
    }
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
