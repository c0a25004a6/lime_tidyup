#!/usr/bin/env python3
"""Run the repeatability aggregator for the ±0.025 mm matrix."""
from __future__ import annotations

import summarize_rubiks_dynamic_hold_repeatability as base

TRIALS = (
    ("minus_0p025mm", -0.000025),
    ("center", 0.0),
    ("plus_0p025mm", 0.000025),
)


def main() -> int:
    base.TRIALS = TRIALS
    return base.main()


if __name__ == "__main__":
    raise SystemExit(main())
