#!/usr/bin/env python3
"""Write deterministic seed and preview states as a strict TSV matrix."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

EXPECTED_CASES = ["center", "minus_y", "plus_y", "minus_yaw", "plus_yaw", "mixed"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kinematic-summary", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    summary = json.loads(Path(args.kinematic_summary).read_text(encoding="utf-8"))
    if summary.get("passed") is not True:
        raise SystemExit("kinematic summary did not pass")
    seed = summary.get("ik_method", {}).get("seed_joint_positions_rad")
    if not isinstance(seed, list) or len(seed) != 6:
        raise SystemExit("invalid deterministic seed")
    cases = summary.get("accepted_observation_cases", [])
    if [case.get("label") for case in cases] != EXPECTED_CASES:
        raise SystemExit("kinematic preview case order mismatch")

    rows = [("seed", seed)]
    for case in cases:
        positions = case.get("preview_joint_positions_rad")
        if not isinstance(positions, list) or len(positions) != 6:
            raise SystemExit(f"invalid preview state for {case.get('label')}")
        rows.append((case["label"], positions))

    output = "".join(
        label + "\t" + "\t".join(format(float(value), ".17g") for value in positions) + "\n"
        for label, positions in rows
    )
    Path(args.output).write_text(output, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
