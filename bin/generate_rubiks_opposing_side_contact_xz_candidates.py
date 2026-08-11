#!/usr/bin/env python3
"""Generate a bounded X/Z translation grid around the accepted PR37 grasp fixture."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

CENTER = (0.063, 0.0, 0.01225)
STEP_M = 0.0005
RADIUS_STEPS = 10
EXPECTED_COUNT = (2 * RADIUS_STEPS + 1) ** 2


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def candidates() -> list[tuple[int, int]]:
    rows = [
        (dx, dz)
        for dx in range(-RADIUS_STEPS, RADIUS_STEPS + 1)
        for dz in range(-RADIUS_STEPS, RADIUS_STEPS + 1)
    ]
    rows.sort(
        key=lambda item: (
            item[0] * item[0] + item[1] * item[1],
            abs(item[0]) + abs(item[1]),
            abs(item[1]),
            abs(item[0]),
            item[1],
            item[0],
        )
    )
    require(len(rows) == EXPECTED_COUNT, "candidate count mismatch")
    require(rows[0] == (0, 0), "accepted PR37 center must be first")
    return rows


def write(path: Path) -> None:
    rows = candidates()
    with path.open("w", encoding="utf-8") as stream:
        stream.write(
            "META\t1\tXZ_OPPOSING_SIDE_CONTACT\t"
            f"{STEP_M:.17g}\t{RADIUS_STEPS}\t{len(rows)}\t"
            f"{CENTER[0]:.17g}\t{CENTER[1]:.17g}\t{CENTER[2]:.17g}\n"
        )
        for rank, (dx, dz) in enumerate(rows):
            x = CENTER[0] + dx * STEP_M
            y = CENTER[1]
            z = CENTER[2] + dz * STEP_M
            distance = STEP_M * math.sqrt(dx * dx + dz * dz)
            stream.write(
                f"CANDIDATE\t{rank}\t{x:.17g}\t{y:.17g}\t{z:.17g}\t"
                f"{distance:.17g}\t{dx}\t{dz}\n"
            )


def self_test() -> None:
    rows = candidates()
    require(len(set(rows)) == EXPECTED_COUNT, "duplicate candidate")
    require(min(dx for dx, _ in rows) == -10 and max(dx for dx, _ in rows) == 10, "X coverage mismatch")
    require(min(dz for _, dz in rows) == -10 and max(dz for _, dz in rows) == 10, "Z coverage mismatch")
    require(max(STEP_M * math.sqrt(dx * dx + dz * dz) for dx, dz in rows) <= math.sqrt(2) * 0.005 + 1e-12, "radius mismatch")
    print("generate_rubiks_opposing_side_contact_xz_candidates self-test: PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    require(bool(args.output), "--output is required")
    write(Path(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
