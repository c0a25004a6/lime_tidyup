#!/usr/bin/env python3
"""Parallel offline gripper-sweep scan for the remaining local 3-D fixture grid.

Consumes the 4,896-candidate file emitted by prepare_rubiks_grasp_followup.py.
Each worker invokes the already accepted PR27 collision-only sweep executable;
no ROS node, Gazebo process, controller, or robot command is created here.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import math
import os
import subprocess
import tempfile
from pathlib import Path

CENTER = (0.06325, 0.0, 0.01225)
EXPECTED_COUNT = 4896
POSITIVE = "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def read_candidates(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    meta_seen = False
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 10, "invalid 3-D candidate META")
            require(fields[1:3] == ["1", "XYZ_REMAINING"], "unexpected candidate mode")
            require(int(fields[5]) == EXPECTED_COUNT, "unexpected remaining candidate count")
            require(int(fields[6]) == 17, "previous X-line count mismatch")
            require(
                all(abs(float(fields[7 + i]) - CENTER[i]) <= 1e-12 for i in range(3)),
                "candidate center mismatch",
            )
            meta_seen = True
            continue
        require(fields[0] == "CANDIDATE" and len(fields) == 9, "invalid 3-D candidate row")
        translation = [float(value) for value in fields[2:5]]
        offset_steps = [int(value) for value in fields[6:9]]
        require(not (offset_steps[1] == 0 and offset_steps[2] == 0), "prior X-line candidate leaked into 3-D scan")
        rows.append({
            "rank": int(fields[1]),
            "translation": translation,
            "distance": float(fields[5]),
            "offset_steps": offset_steps,
        })
    require(meta_seen, "3-D candidate META missing")
    require(len(rows) == EXPECTED_COUNT, "3-D candidate matrix incomplete")
    require([int(row["rank"]) for row in rows] == list(range(EXPECTED_COUNT)), "candidate ranks are not contiguous")
    return rows


def shifted_fixture(source_text: str, translation: list[float]) -> str:
    shift = [translation[index] - CENTER[index] for index in range(3)]
    output: list[str] = []
    object_count = 0
    for raw in source_text.splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "OBJECT":
            require(len(fields) == 8, "invalid fixture OBJECT row")
            offset = [float(value) for value in fields[5:8]]
            effective = [offset[index] + shift[index] for index in range(3)]
            fields[5:8] = [f"{value:.17g}" for value in effective]
            object_count += 1
        output.append("\t".join(fields))
    require(object_count == 2, "expected cube and support fixture objects")
    return "\n".join(output) + "\n"


def parse_decision(path: Path) -> dict[str, object]:
    decision: dict[str, object] | None = None
    for raw in path.read_text(encoding="utf-8").splitlines():
        fields = raw.split("\t")
        if fields[0] != "DECISION":
            continue
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


def evaluate_candidate(
    candidate: dict[str, object],
    fixture_text: str,
    sweep_executable: str,
    urdf: str,
    srdf: str,
    states: str,
    work_dir: Path,
) -> tuple[dict[str, object], dict[str, object]]:
    rank = int(candidate["rank"])
    translation = list(candidate["translation"])
    fixture_path = work_dir / f"fixture_{rank:04d}.tsv"
    result_path = work_dir / f"sweep_{rank:04d}.tsv"

    # Preserve completed per-candidate evidence so an interrupted large scan can resume.
    if result_path.is_file() and result_path.stat().st_size > 0:
        return candidate, parse_decision(result_path)

    fixture_path.write_text(shifted_fixture(fixture_text, translation), encoding="utf-8")
    subprocess.run(
        [
            sweep_executable,
            urdf,
            srdf,
            states,
            str(fixture_path),
            f"{CENTER[0]:.17g}",
            f"{CENTER[1]:.17g}",
            f"{CENTER[2]:.17g}",
            str(result_path),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return candidate, parse_decision(result_path)


def write_outputs(
    scan_output: Path,
    compatible_output: Path,
    rows: list[tuple[dict[str, object], dict[str, object]]],
) -> int:
    rows.sort(key=lambda item: int(item[0]["rank"]))
    compatible = [candidate for candidate, decision in rows if decision["decision"] == POSITIVE]

    with scan_output.open("w", encoding="utf-8") as stream:
        stream.write(f"META\t1\tXYZ_REMAINING\t{len(rows)}\t{len(compatible)}\n")
        for candidate, decision in rows:
            x, y, z = candidate["translation"]
            dx, dy, dz = candidate["offset_steps"]
            stream.write(
                f"SCAN\t{candidate['rank']}\t{x:.17g}\t{y:.17g}\t{z:.17g}\t"
                f"{candidate['distance']:.17g}\t{dx}\t{dy}\t{dz}\t{decision['decision']}\t"
                f"{str(decision['open_clear']).lower()}\t{decision['first_contact_q']:.17g}\t"
                f"{str(decision['dual_contact']).lower()}\t{decision['first_dual_q']:.17g}\t"
                f"{str(decision['forbidden_before_dual']).lower()}\n"
            )

    with compatible_output.open("w", encoding="utf-8") as stream:
        stream.write(f"META\t1\tXYZ_COMPATIBLE\t{len(compatible)}\n")
        for output_rank, candidate in enumerate(compatible):
            x, y, z = candidate["translation"]
            norm = math.sqrt(x * x + y * y + z * z)
            dx, dy, dz = candidate["offset_steps"]
            stream.write(
                f"CANDIDATE\t{output_rank}\t{x:.17g}\t{y:.17g}\t{z:.17g}\t"
                f"{norm:.17g}\t{dx}\t{dy}\t{dz}\t0\t0\t0\n"
            )
    return len(compatible)


def self_test() -> None:
    from prepare_rubiks_grasp_followup import write_3d_candidates

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        candidates_path = root / "candidates.tsv"
        write_3d_candidates(candidates_path)
        rows = read_candidates(candidates_path)
        require(len(rows) == EXPECTED_COUNT, "self-test candidate count mismatch")
        require(rows[0]["offset_steps"] in ([0, -1, 0], [0, 0, -1], [0, 0, 1], [0, 1, 0]), "unexpected nearest candidate")

        fixture = (
            "META\t1\tlink7\t1541\t1043\t2802\n"
            "OBJECT\trubiks_cube\t0.057\t0.057\t0.057\t-0.019\t0\t0.1192\n"
            "OBJECT\trubiks_support\t0.075\t0.040\t0.010\t-0.019\t0\t0.0857\n"
        )
        shifted = shifted_fixture(fixture, [CENTER[0] + 0.001, CENTER[1] - 0.0005, CENTER[2] + 0.00025])
        object_rows = [line.split("\t") for line in shifted.splitlines() if line.startswith("OBJECT\t")]
        cube = next(row for row in object_rows if row[1] == "rubiks_cube")
        require(abs(float(cube[5]) - (-0.018)) <= 1e-12, "fixture X shift mismatch")
        require(abs(float(cube[6]) - (-0.0005)) <= 1e-12, "fixture Y shift mismatch")
        require(abs(float(cube[7]) - 0.11945) <= 1e-12, "fixture Z shift mismatch")

    print("run_rubiks_gripper_fixture_3d_compatibility_scan self-test: PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidates")
    parser.add_argument("--fixture")
    parser.add_argument("--sweep-executable")
    parser.add_argument("--urdf")
    parser.add_argument("--srdf")
    parser.add_argument("--states")
    parser.add_argument("--work-dir")
    parser.add_argument("--scan-output")
    parser.add_argument("--compatible-candidates")
    parser.add_argument("--workers", type=int, default=max(1, min(8, os.cpu_count() or 1)))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0

    required = (
        "candidates", "fixture", "sweep_executable", "urdf", "srdf", "states",
        "work_dir", "scan_output", "compatible_candidates",
    )
    for name in required:
        require(bool(getattr(args, name)), f"--{name.replace('_', '-')} is required")
    require(1 <= args.workers <= 32, "--workers must be in [1, 32]")

    candidates = read_candidates(Path(args.candidates))
    fixture_text = Path(args.fixture).read_text(encoding="utf-8")
    work = Path(args.work_dir)
    work.mkdir(parents=True, exist_ok=True)
    rows: list[tuple[dict[str, object], dict[str, object]]] = []

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
            if completed % 100 == 0 or completed == len(futures):
                print(f"completed={completed}/{len(futures)}", flush=True)

    compatible_count = write_outputs(
        Path(args.scan_output),
        Path(args.compatible_candidates),
        rows,
    )
    print(f"scanned={len(rows)} compatible={compatible_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
