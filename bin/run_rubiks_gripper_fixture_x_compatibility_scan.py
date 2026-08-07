#!/usr/bin/env python3
"""Reuse the PR27 sweep executable across a bounded local X translation scan."""
from __future__ import annotations

import argparse
import math
import subprocess
from pathlib import Path

CENTER = (0.06325, 0.0, 0.01225)
POSITIVE = "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def read_candidates(path: Path) -> list[dict[str, object]]:
    rows = []
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 9 and fields[1:3] == ["1", "X"], "invalid candidate META")
            require(int(fields[5]) == 17, "unexpected X candidate count")
            continue
        require(fields[0] == "CANDIDATE" and len(fields) == 7, "invalid candidate row")
        rows.append({
            "rank": int(fields[1]),
            "translation": [float(value) for value in fields[2:5]],
            "distance": float(fields[5]),
            "offset_steps": int(fields[6]),
        })
    require(len(rows) == 17, "X candidate matrix incomplete")
    return rows


def shifted_fixture(source: Path, destination: Path, translation: list[float]) -> None:
    shift = [translation[index] - CENTER[index] for index in range(3)]
    output = []
    for raw in source.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "OBJECT":
            require(len(fields) == 8, "invalid fixture OBJECT row")
            offset = [float(value) for value in fields[5:8]]
            effective = [offset[index] + shift[index] for index in range(3)]
            fields[5:8] = [f"{value:.17g}" for value in effective]
        output.append("\t".join(fields))
    destination.write_text("\n".join(output) + "\n")


def parse_decision(path: Path) -> dict[str, object]:
    decision = None
    for raw in path.read_text().splitlines():
        fields = raw.split("\t")
        if fields[0] == "DECISION":
            require(len(fields) == 11, "invalid sweep DECISION row")
            decision = {
                "decision": fields[1],
                "open_clear": fields[2] == "true",
                "first_open_index": int(fields[3]),
                "desired_contact": fields[4] == "true",
                "first_contact_index": int(fields[5]),
                "first_contact_q": float(fields[6]),
                "dual_contact": fields[7] == "true",
                "first_dual_index": int(fields[8]),
                "first_dual_q": float(fields[9]),
                "forbidden_before_dual": fields[10] == "true",
            }
    require(decision is not None, "sweep decision missing")
    return decision


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
    args = parser.parse_args()

    candidates = read_candidates(Path(args.candidates))
    work = Path(args.work_dir)
    work.mkdir(parents=True, exist_ok=True)
    scan_rows = []
    compatible = []
    for candidate in candidates:
        rank = int(candidate["rank"])
        translation = list(candidate["translation"])
        fixture_path = work / f"fixture_{rank:02d}.tsv"
        result_path = work / f"sweep_{rank:02d}.tsv"
        shifted_fixture(Path(args.fixture), fixture_path, translation)
        subprocess.run([
            args.sweep_executable,
            args.urdf, args.srdf, args.states, str(fixture_path),
            f"{CENTER[0]:.17g}", f"{CENTER[1]:.17g}", f"{CENTER[2]:.17g}",
            str(result_path),
        ], check=True)
        decision = parse_decision(result_path)
        scan_rows.append((candidate, decision))
        if decision["decision"] == POSITIVE:
            compatible.append(candidate)

    with Path(args.scan_output).open("w", encoding="utf-8") as stream:
        stream.write(f"META\t1\tX\t{len(candidates)}\t{len(compatible)}\n")
        for candidate, decision in scan_rows:
            translation = candidate["translation"]
            stream.write(
                f"SCAN\t{candidate['rank']}\t{translation[0]:.17g}\t{translation[1]:.17g}\t{translation[2]:.17g}\t"
                f"{candidate['distance']:.17g}\t{candidate['offset_steps']}\t{decision['decision']}\t"
                f"{str(decision['open_clear']).lower()}\t{decision['first_contact_q']:.17g}\t"
                f"{str(decision['dual_contact']).lower()}\t{decision['first_dual_q']:.17g}\t"
                f"{str(decision['forbidden_before_dual']).lower()}\n"
            )

    with Path(args.compatible_candidates).open("w", encoding="utf-8") as stream:
        stream.write(f"META\t1\tX_COMPATIBLE\t{len(compatible)}\n")
        for rank, candidate in enumerate(compatible):
            x, y, z = candidate["translation"]
            norm = math.sqrt(x * x + y * y + z * z)
            stream.write(
                f"CANDIDATE\t{rank}\t{x:.17g}\t{y:.17g}\t{z:.17g}\t{norm:.17g}\t0\t0\t0\t0\t0\t0\n"
            )
    print(f"scanned={len(candidates)} compatible={len(compatible)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
