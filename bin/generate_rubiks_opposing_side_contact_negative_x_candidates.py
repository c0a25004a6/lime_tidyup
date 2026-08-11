#!/usr/bin/env python3
"""Extend the opposing-side contact search only beyond the negative-X boundary.

The previous bounded X/Z search exhausted offsets [-10,+10] steps at 0.5 mm.
Its best contact-quality candidates were all at dx=-10. This generator therefore
covers only the previously untested strip dx=-11..-24 while retaining dz=-10..10
and Y=0.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

CENTER = (0.063, 0.0, 0.01225)
STEP_M = 0.0005
DX_MIN = -24
DX_MAX = -11
DZ_RADIUS = 10
EXPECTED_COUNT = (DX_MAX - DX_MIN + 1) * (2 * DZ_RADIUS + 1)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def candidates() -> list[tuple[int, int]]:
    rows = [
        (dx, dz)
        for dx in range(DX_MIN, DX_MAX + 1)
        for dz in range(-DZ_RADIUS, DZ_RADIUS + 1)
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
    require(all(dx <= -11 for dx, _ in rows), "previous X domain leaked into extension")
    return rows


def write(path: Path) -> None:
    rows = candidates()
    with path.open("w", encoding="utf-8") as stream:
        stream.write(
            "META\t1\tXZ_NEGATIVE_X_EXTENSION\t"
            f"{STEP_M:.17g}\t{len(rows)}\t{DX_MIN}\t{DX_MAX}\t{DZ_RADIUS}\t"
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
    require(EXPECTED_COUNT == 294, "expected 294 directional extension candidates")
    require(len(set(rows)) == EXPECTED_COUNT, "duplicate candidate")
    require(min(dx for dx, _ in rows) == -24, "negative-X lower bound mismatch")
    require(max(dx for dx, _ in rows) == -11, "negative-X upper bound mismatch")
    require(min(dz for _, dz in rows) == -10 and max(dz for _, dz in rows) == 10, "Z coverage mismatch")
    require((-10, 0) not in rows, "previous boundary candidate leaked into extension")
    print("generate_rubiks_opposing_side_contact_negative_x_candidates self-test: PASS")


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
