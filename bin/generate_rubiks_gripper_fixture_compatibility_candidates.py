#!/usr/bin/env python3
"""Generate the first deterministic local X-axis compatibility scan around PR26."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

PR27_SOURCE = "bffc9619efe3db79ab89cb282a93e4d1f1ae6ca1"
CENTER = (0.06325, 0.0, 0.01225)
STEP_M = 0.00025
RADIUS_STEPS = 8


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pr27-summary", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    summary = json.loads(Path(args.pr27_summary).read_text())
    require(summary.get("source_ref") == PR27_SOURCE, "PR27 source mismatch")
    require(summary.get("audit_passed") is True, "PR27 audit failed")
    require(summary.get("decision") == "BLOCKED_FORBIDDEN_COLLISION_BEFORE_DUAL_CONTACT", "PR27 blocker mismatch")
    require(summary.get("selected_translation_link7_m") == list(CENTER), "PR27 center mismatch")

    offsets = list(range(-RADIUS_STEPS, RADIUS_STEPS + 1))
    offsets.sort(key=lambda value: (abs(value), value))
    output = Path(args.output)
    with output.open("w", encoding="utf-8") as stream:
        stream.write(
            f"META\t1\tX\t{STEP_M:.17g}\t{RADIUS_STEPS}\t{len(offsets)}\t"
            f"{CENTER[0]:.17g}\t{CENTER[1]:.17g}\t{CENTER[2]:.17g}\n"
        )
        for rank, offset in enumerate(offsets):
            x = CENTER[0] + offset * STEP_M
            stream.write(
                f"CANDIDATE\t{rank}\t{x:.17g}\t{CENTER[1]:.17g}\t{CENTER[2]:.17g}\t"
                f"{abs(offset) * STEP_M:.17g}\t{offset}\n"
            )
    print(json.dumps({
        "axis": "X", "candidate_count": len(offsets), "center_m": CENTER,
        "step_m": STEP_M, "radius_m": RADIUS_STEPS * STEP_M,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
