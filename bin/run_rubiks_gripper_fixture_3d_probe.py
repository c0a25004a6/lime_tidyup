#!/usr/bin/env python3
"""Run a bounded distance-first prefix of the 3-D Rubik fixture scan.

This is an early positive probe only. A negative result does not replace the
full 4,896-candidate expansion.
"""
from __future__ import annotations

import argparse
import concurrent.futures
from pathlib import Path

from run_rubiks_gripper_fixture_3d_compatibility_scan import (
    evaluate_candidate,
    read_candidates,
    require,
    write_outputs,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidates", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--sweep-executable", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--states", required=True)
    parser.add_argument("--work-dir", required=True)
    parser.add_argument("--scan-output", required=True)
    parser.add_argument("--compatible-candidates", required=True)
    parser.add_argument("--limit", type=int, default=256)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    require(1 <= args.limit <= 4896, "--limit must be in [1, 4896]")
    require(1 <= args.workers <= 32, "--workers must be in [1, 32]")

    full_candidates = read_candidates(Path(args.candidates))
    candidates = full_candidates[: args.limit]
    fixture_text = Path(args.fixture).read_text(encoding="utf-8")
    work = Path(args.work_dir)
    work.mkdir(parents=True, exist_ok=True)

    rows = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = [
            executor.submit(
                evaluate_candidate,
                candidate,
                fixture_text,
                args.sweep_executable,
                args.urdf,
                args.srdf,
                args.states,
                work,
            )
            for candidate in candidates
        ]
        for completed, future in enumerate(concurrent.futures.as_completed(futures), start=1):
            rows.append(future.result())
            if completed % 32 == 0 or completed == len(futures):
                print(f"probe_completed={completed}/{len(futures)}", flush=True)

    compatible_count = write_outputs(
        Path(args.scan_output),
        Path(args.compatible_candidates),
        rows,
    )
    print(
        f"probe_scanned={len(rows)} compatible={compatible_count} "
        "negative_is_not_exhaustive=true"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
