#!/usr/bin/env python3
"""Shared cube-local contact-quality audit for Rubik grasp physics evidence.

The offline exact-mesh search and Gazebo physics gates use the same semantic
criteria: left finger on +Y cube face, right finger on -Y cube face, away from
X/Z edges, Y-dominant local normal, and bounded penetration.
"""
from __future__ import annotations

import bisect
import math
from typing import Iterable

CUBE_HALF_M = 0.0285
FACE_TOLERANCE_M = 0.0010
EDGE_MARGIN_M = 0.0010
NORMAL_Y_MIN = 0.70
MAX_CONTACT_DEPTH_M = 0.0010
MIN_QUALITY_FRACTION = 0.80
MIN_CONTACT_POINTS = 10


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def quat_conjugate(q: list[float]) -> list[float]:
    require(len(q) == 4, "quaternion must be xyzw")
    x, y, z, w = (float(v) for v in q)
    norm2 = x * x + y * y + z * z + w * w
    require(math.isfinite(norm2) and norm2 > 1e-20, "invalid quaternion")
    return [-x / norm2, -y / norm2, -z / norm2, w / norm2]


def rotate(q: list[float], v: list[float]) -> list[float]:
    """Rotate vector by normalized/non-normalized quaternion q (xyzw)."""
    x, y, z, w = (float(value) for value in q)
    vx, vy, vz = (float(value) for value in v)
    norm2 = x * x + y * y + z * z + w * w
    require(math.isfinite(norm2) and norm2 > 1e-20, "invalid quaternion")
    scale = 1.0 / math.sqrt(norm2)
    x *= scale
    y *= scale
    z *= scale
    w *= scale
    # q * v * q^-1, expanded.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ]


def world_to_cube_point(cube_state: dict[str, object], point: list[float]) -> list[float]:
    position = [float(v) for v in cube_state["position_m"]]
    orientation = [float(v) for v in cube_state["orientation_xyzw"]]
    delta = [float(point[i]) - position[i] for i in range(3)]
    return rotate(quat_conjugate(orientation), delta)


def world_to_cube_vector(cube_state: dict[str, object], vector: list[float]) -> list[float]:
    orientation = [float(v) for v in cube_state["orientation_xyzw"]]
    return rotate(quat_conjugate(orientation), [float(v) for v in vector])


def nearest_state(states: list[dict[str, object]], t_s: float) -> dict[str, object]:
    require(bool(states), "cube-state evidence is empty")
    times = [float(state["t_s"]) for state in states]
    index = bisect.bisect_left(times, float(t_s))
    choices = []
    if index < len(states):
        choices.append(states[index])
    if index > 0:
        choices.append(states[index - 1])
    require(bool(choices), "no cube state near contact")
    return min(choices, key=lambda state: abs(float(state["t_s"]) - float(t_s)))


def _finger_from_state(contact_state: dict[str, object]) -> str | None:
    pair = f"{contact_state.get('collision1', '')} {contact_state.get('collision2', '')}"
    has_cube = "cube_link" in pair or "cube_collision" in pair
    if not has_cube:
        return None
    if "gripper_left_link" in pair:
        return "left"
    if "gripper_right_link" in pair:
        return "right"
    return None


def _empty_accumulator() -> dict[str, object]:
    return {
        "total_contact_points": 0,
        "qualified_contact_points": 0,
        "side_face_contact_points": 0,
        "interior_contact_points": 0,
        "y_normal_dominant_points": 0,
        "max_depth_m": 0.0,
        "sum_local_position_m": [0.0, 0.0, 0.0],
        "sum_abs_local_normal": [0.0, 0.0, 0.0],
    }


def audit_contact_quality(
    contact_messages: list[dict[str, object]],
    cube_states: list[dict[str, object]],
    *,
    start_s: float | None = None,
    end_s: float | None = None,
) -> dict[str, object]:
    acc = {"left": _empty_accumulator(), "right": _empty_accumulator()}

    for event in contact_messages:
        t_s = float(event["t_s"])
        if start_s is not None and t_s < float(start_s):
            continue
        if end_s is not None and t_s > float(end_s):
            continue
        cube_state = nearest_state(cube_states, t_s)
        for contact_state in event.get("states", []):
            finger = _finger_from_state(contact_state)
            if finger is None:
                continue
            positions = contact_state.get("contact_positions_m", [])
            normals = contact_state.get("contact_normals", [])
            depths = contact_state.get("depths_m", [])
            count = min(len(positions), len(normals), len(depths))
            for index in range(count):
                local_position = world_to_cube_point(cube_state, positions[index])
                local_normal = world_to_cube_vector(cube_state, normals[index])
                abs_normal = [abs(value) for value in local_normal]
                depth = abs(float(depths[index]))
                positive_y = finger == "left"
                if positive_y:
                    side_face = (
                        CUBE_HALF_M - FACE_TOLERANCE_M
                        <= local_position[1]
                        <= CUBE_HALF_M + FACE_TOLERANCE_M
                    )
                else:
                    side_face = (
                        -CUBE_HALF_M - FACE_TOLERANCE_M
                        <= local_position[1]
                        <= -CUBE_HALF_M + FACE_TOLERANCE_M
                    )
                interior = (
                    abs(local_position[0]) <= CUBE_HALF_M - EDGE_MARGIN_M
                    and abs(local_position[2]) <= CUBE_HALF_M - EDGE_MARGIN_M
                )
                normal_y = (
                    abs_normal[1] >= NORMAL_Y_MIN
                    and abs_normal[1] > abs_normal[0]
                    and abs_normal[1] > abs_normal[2]
                )
                depth_ok = depth <= MAX_CONTACT_DEPTH_M
                qualified = side_face and interior and normal_y and depth_ok

                target = acc[finger]
                target["total_contact_points"] += 1
                target["qualified_contact_points"] += int(qualified)
                target["side_face_contact_points"] += int(side_face)
                target["interior_contact_points"] += int(interior)
                target["y_normal_dominant_points"] += int(normal_y)
                target["max_depth_m"] = max(float(target["max_depth_m"]), depth)
                for axis in range(3):
                    target["sum_local_position_m"][axis] += local_position[axis]
                    target["sum_abs_local_normal"][axis] += abs_normal[axis]

    result: dict[str, object] = {
        "criteria": {
            "cube_half_m": CUBE_HALF_M,
            "face_tolerance_m": FACE_TOLERANCE_M,
            "edge_margin_m": EDGE_MARGIN_M,
            "normal_y_min": NORMAL_Y_MIN,
            "max_contact_depth_m": MAX_CONTACT_DEPTH_M,
            "min_quality_fraction": MIN_QUALITY_FRACTION,
            "min_contact_points": MIN_CONTACT_POINTS,
        }
    }
    passed = True
    for finger in ("left", "right"):
        target = acc[finger]
        total = int(target["total_contact_points"])
        qualified = int(target["qualified_contact_points"])
        fraction = qualified / total if total else 0.0
        mean_position = [
            value / total if total else 0.0
            for value in target["sum_local_position_m"]
        ]
        mean_abs_normal = [
            value / total if total else 0.0
            for value in target["sum_abs_local_normal"]
        ]
        finger_pass = total >= MIN_CONTACT_POINTS and fraction >= MIN_QUALITY_FRACTION
        passed = passed and finger_pass
        result[finger] = {
            "total_contact_points": total,
            "qualified_contact_points": qualified,
            "quality_fraction": fraction,
            "side_face_fraction": (
                int(target["side_face_contact_points"]) / total if total else 0.0
            ),
            "interior_fraction": (
                int(target["interior_contact_points"]) / total if total else 0.0
            ),
            "y_normal_dominant_fraction": (
                int(target["y_normal_dominant_points"]) / total if total else 0.0
            ),
            "max_depth_m": float(target["max_depth_m"]),
            "mean_local_position_m": mean_position,
            "mean_abs_local_normal": mean_abs_normal,
            "passed": finger_pass,
        }
    result["passed"] = passed
    return result


def self_test() -> None:
    cube = [{
        "t_s": 1.0,
        "position_m": [0.0, 0.0, 0.0],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }]
    good = [{
        "t_s": 1.0,
        "states": [
            {
                "collision1": "cube_link::cube_collision",
                "collision2": "robot::gripper_left_link::collision",
                "contact_positions_m": [[0.0, CUBE_HALF_M, 0.0]] * 10,
                "contact_normals": [[0.0, -1.0, 0.0]] * 10,
                "depths_m": [0.00001] * 10,
            },
            {
                "collision1": "cube_link::cube_collision",
                "collision2": "robot::gripper_right_link::collision",
                "contact_positions_m": [[0.0, -CUBE_HALF_M, 0.0]] * 10,
                "contact_normals": [[0.0, 1.0, 0.0]] * 10,
                "depths_m": [0.00001] * 10,
            },
        ],
    }]
    good_result = audit_contact_quality(good, cube)
    require(good_result["passed"] is True, "known-good side contact failed")

    corner = [{
        "t_s": 1.0,
        "states": [
            {
                "collision1": "cube_link::cube_collision",
                "collision2": "robot::gripper_left_link::collision",
                "contact_positions_m": [[-CUBE_HALF_M, CUBE_HALF_M, -CUBE_HALF_M]] * 10,
                "contact_normals": [[1.0, 0.0, 0.0]] * 10,
                "depths_m": [0.00001] * 10,
            },
            {
                "collision1": "cube_link::cube_collision",
                "collision2": "robot::gripper_right_link::collision",
                "contact_positions_m": [[-CUBE_HALF_M, -CUBE_HALF_M, -CUBE_HALF_M]] * 10,
                "contact_normals": [[1.0, 0.0, 0.0]] * 10,
                "depths_m": [0.00001] * 10,
            },
        ],
    }]
    bad_result = audit_contact_quality(corner, cube)
    require(bad_result["passed"] is False, "corner contact incorrectly passed")
    print("rubiks_contact_quality self-test: PASS")


def main() -> int:
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser()
    parser.add_argument("--telemetry")
    parser.add_argument("--output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    require(bool(args.telemetry), "--telemetry is required")
    data = json.loads(Path(args.telemetry).read_text(encoding="utf-8"))
    result = audit_contact_quality(
        data.get("contact_messages", []),
        data.get("cube_state_samples", []),
        start_s=(data.get("hold_result") or {}).get("hold_start_s"),
        end_s=(data.get("hold_result") or {}).get("hold_end_s"),
    )
    if args.output:
        Path(args.output).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
