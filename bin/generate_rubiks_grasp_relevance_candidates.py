#!/usr/bin/env python3
"""Generate deterministic translation candidates constrained by PR24 finger reach."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

COARSE_STEP_M = 0.002
COARSE_RADIUS_STEPS = 40
FINE_STEP_M = 0.00025
FINE_RADIUS_STEPS = 8
TOLERANCE = 1e-12
EXPECTED_PR24_SOURCE = "7d8b9b055cd331d11b89ba275a1ecce7312b4b84"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def overlap(a_min: float, a_max: float, b_min: float, b_max: float) -> float:
    return max(0.0, min(a_max, b_max) - max(a_min, b_min))


def parse_fixture(path: Path) -> dict[str, object]:
    result: dict[str, object] = {}
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 6 and fields[1] == "1", "invalid fixture META")
            result["reference_link"] = fields[2]
            result["anchor_index"] = int(fields[3])
            result["candidate_start_index"] = int(fields[4])
            result["state_count"] = int(fields[5])
        elif fields[0] == "OBJECT":
            require(len(fields) == 8, "invalid fixture OBJECT")
            result[fields[1]] = {
                "size": [float(value) for value in fields[2:5]],
                "offset": [float(value) for value in fields[5:8]],
            }
        else:
            raise ValueError(f"unknown fixture row: {fields[0]}")
    require(result.get("reference_link") == "link7", "fixture reference mismatch")
    require(result.get("state_count") == 2802, "fixture state count mismatch")
    require(result.get("anchor_index") == 1541, "fixture anchor mismatch")
    require("rubiks_cube" in result and "rubiks_support" in result, "fixture objects missing")
    return result


def parse_coarse_selection(path: Path) -> tuple[float, float, float]:
    selected = None
    for raw in path.read_text().splitlines():
        fields = raw.split("\t")
        if fields and fields[0] == "SELECT":
            require(len(fields) >= 7, "invalid SELECT row")
            require(fields[1] == "true", "coarse evaluator found no clear candidate")
            selected = tuple(float(value) for value in fields[2:5])
    require(selected is not None, "coarse SELECT row missing")
    return selected


def relevant(
    delta: tuple[float, float, float],
    cube_size: list[float],
    cube_offset: list[float],
    zero_left_min: list[float],
    zero_left_max: list[float],
    zero_right_min: list[float],
    zero_right_max: list[float],
    lower: float,
    upper: float,
) -> tuple[bool, dict[str, float]]:
    center = [cube_offset[i] + delta[i] for i in range(3)]
    cube_min = [center[i] - cube_size[i] * 0.5 for i in range(3)]
    cube_max = [center[i] + cube_size[i] * 0.5 for i in range(3)]
    left_x = overlap(zero_left_min[0], zero_left_max[0], cube_min[0], cube_max[0])
    left_z = overlap(zero_left_min[2], zero_left_max[2], cube_min[2], cube_max[2])
    right_x = overlap(zero_right_min[0], zero_right_max[0], cube_min[0], cube_max[0])
    right_z = overlap(zero_right_min[2], zero_right_max[2], cube_min[2], cube_max[2])

    left_low = cube_max[1] - zero_left_max[1]
    left_high = cube_max[1] - zero_left_min[1]
    right_low = zero_right_min[1] - cube_min[1]
    right_high = zero_right_max[1] - cube_min[1]
    common_low = max(lower, left_low, right_low)
    common_high = min(upper, left_high, right_high)
    common_exists = common_low <= common_high + TOLERANCE
    positive_xz = min(left_x, left_z, right_x, right_z) > 0.0
    return positive_xz and common_exists, {
        "left_x": left_x,
        "left_z": left_z,
        "right_x": right_x,
        "right_z": right_z,
        "common_low": common_low,
        "common_high": common_high,
    }


def ordered_coarse() -> list[tuple[int, int, int]]:
    values = []
    radius2 = COARSE_RADIUS_STEPS * COARSE_RADIUS_STEPS
    for x in range(-COARSE_RADIUS_STEPS, COARSE_RADIUS_STEPS + 1):
        for y in range(-COARSE_RADIUS_STEPS, COARSE_RADIUS_STEPS + 1):
            for z in range(-COARSE_RADIUS_STEPS, COARSE_RADIUS_STEPS + 1):
                norm2 = x * x + y * y + z * z
                if norm2 <= radius2:
                    values.append((x, y, z))
    values.sort(key=lambda item: (
        item[0] * item[0] + item[1] * item[1] + item[2] * item[2],
        abs(item[2]), abs(item[1]), abs(item[0]), item[2], item[1], item[0],
    ))
    return values


def ordered_fine(center: tuple[float, float, float]) -> list[tuple[int, int, int]]:
    center_units = tuple(int(round(value / FINE_STEP_M)) for value in center)
    values = []
    for dx in range(-FINE_RADIUS_STEPS, FINE_RADIUS_STEPS + 1):
        for dy in range(-FINE_RADIUS_STEPS, FINE_RADIUS_STEPS + 1):
            for dz in range(-FINE_RADIUS_STEPS, FINE_RADIUS_STEPS + 1):
                point = (center_units[0] + dx, center_units[1] + dy, center_units[2] + dz)
                values.append(point)
    values.sort(key=lambda item: (
        item[0] * item[0] + item[1] * item[1] + item[2] * item[2],
        abs(item[2]), abs(item[1]), abs(item[0]), item[2], item[1], item[0],
    ))
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--relevance-summary", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--mode", choices=("coarse", "fine"), required=True)
    parser.add_argument("--coarse-results")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    summary = json.loads(Path(args.relevance_summary).read_text())
    require(summary.get("source_ref") == EXPECTED_PR24_SOURCE, "PR24 summary source mismatch")
    require(summary.get("audit_passed") is True, "PR24 relevance audit failed")
    require(summary.get("decision") == "BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE", "PR24 blocker mismatch")
    fixture = parse_fixture(Path(args.fixture))
    cube = fixture["rubiks_cube"]
    zero = summary.get("evaluated_positions", {}).get("zero", {})
    require(zero, "PR24 zero-position geometry missing")
    left_min = [float(value) for value in zero["left_aabb_min_m"]]
    left_max = [float(value) for value in zero["left_aabb_max_m"]]
    right_min = [float(value) for value in zero["right_aabb_min_m"]]
    right_max = [float(value) for value in zero["right_aabb_max_m"]]
    positions = summary.get("evaluated_positions", {})
    lower = float(positions["lower"]["requested_m"])
    upper = float(positions["upper"]["requested_m"])

    if args.mode == "coarse":
        integer_candidates = ordered_coarse()
        step = COARSE_STEP_M
        center = (0.0, 0.0, 0.0)
    else:
        require(args.coarse_results is not None, "fine mode requires coarse results")
        center = parse_coarse_selection(Path(args.coarse_results))
        integer_candidates = ordered_fine(center)
        step = FINE_STEP_M

    checked = 0
    accepted = []
    for item in integer_candidates:
        delta = tuple(value * step for value in item)
        checked += 1
        ok, metrics = relevant(
            delta,
            cube["size"],
            cube["offset"],
            left_min, left_max, right_min, right_max,
            lower, upper,
        )
        if ok:
            accepted.append((delta, metrics))

    output = Path(args.output)
    with output.open("w", encoding="utf-8") as stream:
        stream.write(
            f"META\t1\t{args.mode}\t{step:.17g}\t{checked}\t{len(accepted)}\t"
            f"{center[0]:.17g}\t{center[1]:.17g}\t{center[2]:.17g}\n"
        )
        for rank, (delta, metrics) in enumerate(accepted):
            stream.write(
                f"CANDIDATE\t{rank}\t{delta[0]:.17g}\t{delta[1]:.17g}\t{delta[2]:.17g}\t"
                f"{math.sqrt(sum(value * value for value in delta)):.17g}\t"
                f"{metrics['left_x']:.17g}\t{metrics['left_z']:.17g}\t"
                f"{metrics['right_x']:.17g}\t{metrics['right_z']:.17g}\t"
                f"{metrics['common_low']:.17g}\t{metrics['common_high']:.17g}\n"
            )
    require(accepted, f"{args.mode} relevance filter produced no candidates")
    print(json.dumps({
        "mode": args.mode,
        "enumerated": checked,
        "relevant": len(accepted),
        "step_m": step,
        "center_m": center,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
