#!/usr/bin/env python3
"""Search support Y width while keeping the accepted PR26 cube pose unchanged.

The accepted 40 mm-wide support is reduced only along Y. Cube pose, support
center/height, arm anchor, gripper limits, and the PR26 fixture translation stay
fixed. The objective is the *largest* support width whose offline gripper sweep
has an open-clear prefix and reaches bilateral cube contact before any forbidden
collision.

This script starts no ROS/Gazebo process and sends no command.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import tempfile
from pathlib import Path

CENTER = (0.06325, 0.0, 0.01225)
ORIGINAL_SUPPORT_WIDTH_M = 0.040
MIN_SUPPORT_WIDTH_M = 0.010
COARSE_STEP_M = 0.001
FINE_STEP_M = 0.00025
POSITIVE = "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND"
EXPECTED_BASELINE = "BLOCKED_FORBIDDEN_COLLISION_BEFORE_DUAL_CONTACT"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def fixture_with_support_width(source: str, width_m: float) -> str:
    require(MIN_SUPPORT_WIDTH_M <= width_m <= ORIGINAL_SUPPORT_WIDTH_M, "support width outside declared range")
    output: list[str] = []
    cube_seen = False
    support_seen = False
    for raw in source.splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "OBJECT":
            require(len(fields) == 8, "invalid fixture OBJECT row")
            if fields[1] == "rubiks_cube":
                cube_seen = True
                require(abs(float(fields[2]) - 0.057) <= 1e-12, "cube X dimension changed")
                require(abs(float(fields[3]) - 0.057) <= 1e-12, "cube Y dimension changed")
                require(abs(float(fields[4]) - 0.057) <= 1e-12, "cube Z dimension changed")
            elif fields[1] == "rubiks_support":
                support_seen = True
                require(abs(float(fields[2]) - 0.075) <= 1e-12, "support X dimension changed")
                require(abs(float(fields[3]) - ORIGINAL_SUPPORT_WIDTH_M) <= 1e-12, "unexpected input support width")
                require(abs(float(fields[4]) - 0.010) <= 1e-12, "support Z dimension changed")
                fields[3] = f"{width_m:.17g}"
            else:
                raise ValueError(f"unexpected fixture object: {fields[1]}")
        output.append("\t".join(fields))
    require(cube_seen and support_seen, "cube/support fixture incomplete")
    return "\n".join(output) + "\n"


def parse_decision(path: Path) -> dict[str, object]:
    decision: dict[str, object] | None = None
    first_forbidden_pairs: str | None = None
    for raw in path.read_text(encoding="utf-8").splitlines():
        fields = raw.split("\t")
        if fields[0] == "STATE" and len(fields) >= 15:
            # STATE schema is inherited from the accepted PR27 evaluator. The
            # last contact-pair fields are preserved verbatim for diagnostics.
            if fields[3] == "FORBIDDEN" and first_forbidden_pairs is None:
                nonempty = [value for value in fields[13:] if value]
                first_forbidden_pairs = ",".join(nonempty)
        elif fields[0] == "DECISION":
            require(len(fields) == 11, "invalid sweep DECISION row")
            decision = {
                "decision": fields[1],
                "open_clear": fields[2] == "true",
                "first_open_index": int(fields[3]),
                "desired_contact": fields[4] == "true",
                "first_contact_index": int(fields[5]),
                "first_contact_q_m": float(fields[6]),
                "dual_contact": fields[7] == "true",
                "first_dual_index": int(fields[8]),
                "first_dual_q_m": float(fields[9]),
                "forbidden_before_dual": fields[10] == "true",
            }
    require(decision is not None, "sweep decision missing")
    decision["first_forbidden_pairs"] = first_forbidden_pairs
    return decision


def run_width(
    width_m: float,
    source_fixture: str,
    sweep_executable: str,
    urdf: str,
    srdf: str,
    states: str,
    work_dir: Path,
) -> dict[str, object]:
    key = int(round(width_m * 1_000_000))
    fixture_path = work_dir / f"fixture_support_y_{key:05d}um.tsv"
    result_path = work_dir / f"sweep_support_y_{key:05d}um.tsv"
    fixture_path.write_text(fixture_with_support_width(source_fixture, width_m), encoding="utf-8")
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
    result = parse_decision(result_path)
    result["support_width_m"] = width_m
    result["fixture_path"] = str(fixture_path)
    result["result_path"] = str(result_path)
    return result


def grid_descending(high: float, low: float, step: float) -> list[float]:
    count = int(round((high - low) / step))
    values = [high - index * step for index in range(count + 1)]
    require(abs(values[-1] - low) <= 1e-12, "declared width grid does not close exactly")
    return values


def build_summary(results: list[dict[str, object]]) -> dict[str, object]:
    baseline = next(result for result in results if abs(float(result["support_width_m"]) - ORIGINAL_SUPPORT_WIDTH_M) <= 1e-12)
    require(baseline["decision"] == EXPECTED_BASELINE, "40 mm baseline did not reproduce PR27 blocker")
    positives = [result for result in results if result["decision"] == POSITIVE]
    selected = max(positives, key=lambda result: float(result["support_width_m"])) if positives else None
    return {
        "schema_version": 1,
        "phase": "RUBIK-SUPPORT-WIDTH-GRIPPER-COMPATIBILITY-SEARCH",
        "center_translation_link7_m": list(CENTER),
        "cube_pose_changed": False,
        "support_center_changed": False,
        "support_xz_dimensions_changed": False,
        "original_support_width_m": ORIGINAL_SUPPORT_WIDTH_M,
        "minimum_support_width_m": MIN_SUPPORT_WIDTH_M,
        "coarse_step_m": COARSE_STEP_M,
        "fine_step_m": FINE_STEP_M,
        "baseline_reproduced": True,
        "candidate_found": selected is not None,
        "decision": (
            "SUPPORT_WIDTH_GRIPPER_COMPATIBLE_CANDIDATE_FOUND"
            if selected is not None
            else "NO_SUPPORT_WIDTH_GRIPPER_COMPATIBLE_CANDIDATE_IN_DECLARED_RANGE"
        ),
        "selected": None if selected is None else {
            "support_width_m": selected["support_width_m"],
            "open_clear": selected["open_clear"],
            "dual_contact": selected["dual_contact"],
            "first_dual_q_m": selected["first_dual_q_m"],
            "forbidden_before_dual": selected["forbidden_before_dual"],
        },
        "evaluated": [
            {
                "support_width_m": result["support_width_m"],
                "decision": result["decision"],
                "open_clear": result["open_clear"],
                "dual_contact": result["dual_contact"],
                "first_dual_q_m": result["first_dual_q_m"],
                "forbidden_before_dual": result["forbidden_before_dual"],
                "first_forbidden_pairs": result["first_forbidden_pairs"],
            }
            for result in sorted(results, key=lambda item: float(item["support_width_m"]), reverse=True)
        ],
        "safety": {
            "offline_analysis_only": True,
            "gazebo_started": False,
            "ros_node_created": False,
            "controller_loaded": False,
            "command_sent": False,
            "attachment_used": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
            "grasp_success_claimed": False,
            "lifting_or_transport_claimed": False,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--sweep-executable", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--states", required=True)
    parser.add_argument("--work-dir", required=True)
    parser.add_argument("--selected-fixture", required=True)
    parser.add_argument("--summary", required=True)
    args = parser.parse_args()

    source_fixture = Path(args.fixture).read_text(encoding="utf-8")
    work = Path(args.work_dir)
    work.mkdir(parents=True, exist_ok=True)

    results: list[dict[str, object]] = []
    by_key: dict[int, dict[str, object]] = {}
    for width in grid_descending(ORIGINAL_SUPPORT_WIDTH_M, MIN_SUPPORT_WIDTH_M, COARSE_STEP_M):
        result = run_width(width, source_fixture, args.sweep_executable, args.urdf, args.srdf, args.states, work)
        results.append(result)
        by_key[int(round(width * 1_000_000))] = result
        print(f"coarse width={width:.6f} decision={result['decision']}", flush=True)

    coarse_positive = [result for result in results if result["decision"] == POSITIVE]
    if coarse_positive:
        best_coarse = max(coarse_positive, key=lambda result: float(result["support_width_m"]))
        positive_width = float(best_coarse["support_width_m"])
        upper = min(ORIGINAL_SUPPORT_WIDTH_M, positive_width + COARSE_STEP_M)
        for width in grid_descending(upper, positive_width, FINE_STEP_M):
            key = int(round(width * 1_000_000))
            if key in by_key:
                continue
            result = run_width(width, source_fixture, args.sweep_executable, args.urdf, args.srdf, args.states, work)
            results.append(result)
            by_key[key] = result
            print(f"fine width={width:.6f} decision={result['decision']}", flush=True)

    summary = build_summary(results)
    Path(args.summary).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if summary["candidate_found"]:
        width = float(summary["selected"]["support_width_m"])
        Path(args.selected_fixture).write_text(fixture_with_support_width(source_fixture, width), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
