#!/usr/bin/env python3
"""Run the repeatability aggregator for the conservative ±0.05 mm matrix."""
from __future__ import annotations

import summarize_rubiks_dynamic_hold_repeatability as base

TRIALS = (
    ("minus_0p05mm", -0.00005),
    ("center", 0.0),
    ("plus_0p05mm", 0.00005),
)


def main() -> int:
    base.TRIALS = TRIALS
    return base.main()


if __name__ == "__main__":
    raise SystemExit(main())
