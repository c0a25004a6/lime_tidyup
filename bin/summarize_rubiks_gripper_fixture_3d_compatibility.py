#!/usr/bin/env python3
"""Summarize the remaining local 3-D Rubik fixture compatibility expansion."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

EXPECTED_COUNT = 4896
POSITIVE_SWEEP = "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND"
POSITIVE = "GRIPPER_FIXTURE_COMPATIBLE_FULL_PATH_CLEAR_TRANSLATION_FOUND"
NEGATIVE = "NO_SWEEP_COMPATIBLE_FULL_PATH_CLEAR_TRANSLATION_IN_LOCAL_3D_DOMAIN"
NEXT_POSITIVE = "RUBIK-PREGRASP-CORRECTED-FIXTURE-STATIC-GAZEBO-NO-ACTUATION-EVIDENCE"
NEXT_NEGATIVE = "RUBIK-PREGRASP-GRIPPER-FIXTURE-GEOMETRY-REDESIGN"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def read_scan(path: Path) -> tuple[int, list[dict[str, object]]]:
    compatible_count = -1
    rows: list[dict[str, object]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 5 and fields[1:3] == ["1", "XYZ_REMAINING"], "invalid 3-D scan META")
            require(int(fields[3]) == EXPECTED_COUNT, "3-D scan coverage mismatch")
            compatible_count = int(fields[4])
            continue
        require(fields[0] == "SCAN" and len(fields) == 15, "invalid 3-D scan row")
        rows.append({
            "rank": int(fields[1]),
            "translation_m": [float(fields[2]), float(fields[3]), float(fields[4])],
            "distance_from_center_m": float(fields[5]),
            "offset_steps": [int(fields[6]), int(fields[7]), int(fields[8])],
            "decision": fields[9],
            "open_clear": fields[10] == "true",
            "first_contact_q_m": float(fields[11]),
            "dual_contact": fields[12] == "true",
            "first_dual_q_m": float(fields[13]),
            "forbidden_before_dual": fields[14] == "true",
        })
    require(compatible_count >= 0, "3-D scan META missing")
    require(len(rows) == EXPECTED_COUNT, "3-D scan rows incomplete")
    require(sum(row["decision"] == POSITIVE_SWEEP for row in rows) == compatible_count, "compatible count mismatch")
    return compatible_count, rows


def read_full_path(path: Path | None) -> dict[str, object] | None:
    if path is None or not path.is_file() or path.stat().st_size == 0:
        return None
    selected: dict[str, object] | None = None
    candidate_rows = 0
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "CANDIDATE":
            candidate_rows += 1
        elif fields[0] == "SELECT":
            require(len(fields) == 8, "invalid full-path SELECT row")
            selected = {
                "found": fields[1] == "true",
                "translation_m": [float(fields[2]), float(fields[3]), float(fields[4])],
                "norm_m": float(fields[5]),
                "total_states_checked": int(fields[6]),
                "selected_states_checked": int(fields[7]),
                "candidate_rows_evaluated": candidate_rows,
            }
    require(selected is not None, "full-path SELECT missing")
    return selected


def find_scan_row(rows: list[dict[str, object]], translation: list[float]) -> dict[str, object]:
    for row in rows:
        values = row["translation_m"]
        if all(abs(float(values[i]) - translation[i]) <= 1e-12 for i in range(3)):
            return row
    raise ValueError("selected full-path translation missing from 3-D scan")


def build_summary(
    input_summary: dict[str, object],
    compatible_count: int,
    scan_rows: list[dict[str, object]],
    full_path: dict[str, object] | None,
    source_ref: str,
) -> dict[str, object]:
    require(len(source_ref) == 40 and all(c in "0123456789abcdef" for c in source_ref.lower()), "invalid source ref")
    require(input_summary.get("audit_passed") is True and input_summary.get("errors") == [], "input PR28 evidence failed audit")
    require(input_summary.get("candidate_found") is False, "3-D expansion requires negative X result")
    require(input_summary.get("next_required_phase") == "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-3D-EXPANSION", "input next phase mismatch")

    found = bool(full_path is not None and full_path["found"])
    selected_candidate = None
    if found:
        translation = list(full_path["translation_m"])
        sweep = find_scan_row(scan_rows, translation)
        require(sweep["decision"] == POSITIVE_SWEEP, "full-path selection was not sweep-compatible")
        require(full_path["selected_states_checked"] == 2802, "selected candidate lacks full 2,802-state clearance")
        selected_candidate = {
            "translation_m": translation,
            "distance_from_center_m": sweep["distance_from_center_m"],
            "offset_steps": sweep["offset_steps"],
            "selected_states_checked": full_path["selected_states_checked"],
            "full_path_candidates_evaluated": full_path["candidate_rows_evaluated"],
            "sweep": {
                "open_clear": sweep["open_clear"],
                "dual_contact": sweep["dual_contact"],
                "first_dual_q_m": sweep["first_dual_q_m"],
                "forbidden_before_dual": sweep["forbidden_before_dual"],
            },
        }

    return {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-3D-EXPANSION",
        "source_ref": source_ref,
        "input_x_source_ref": input_summary.get("source_ref"),
        "audit_passed": True,
        "errors": [],
        "exact_head_bound": True,
        "candidate_found": found,
        "decision": POSITIVE if found else NEGATIVE,
        "next_required_phase": NEXT_POSITIVE if found else NEXT_NEGATIVE,
        "grid": {
            "candidate_count": EXPECTED_COUNT,
            "sweep_compatible_count": compatible_count,
            "step_m": 0.00025,
            "radius_per_axis_m": 0.002,
            "previous_x_line_excluded": True,
        },
        "full_path": {
            "performed": full_path is not None,
            "candidate_rows_evaluated": 0 if full_path is None else full_path["candidate_rows_evaluated"],
            "total_states_checked": 0 if full_path is None else full_path["total_states_checked"],
        },
        "selected_candidate": selected_candidate,
        "safety": {
            "offline_analysis_only": True,
            "gazebo_started": False,
            "ros_node_created": False,
            "object_spawned": False,
            "controller_loaded": False,
            "planning_request_created": False,
            "trajectory_instantiated": False,
            "command_sent": False,
            "attachment_used": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
        },
        "claim_limits": {
            "sampled_translation_grid_only": True,
            "sampled_gripper_positions_only": True,
            "continuous_translation_optimum_claimed": False,
            "continuous_gripper_clearance_claimed": False,
            "force_closure_claimed": False,
            "grasp_success_claimed": False,
            "lifting_or_transport_claimed": False,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-x-summary", required=True)
    parser.add_argument("--scan", required=True)
    parser.add_argument("--full-path-results")
    parser.add_argument("--source-ref", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    input_summary = json.loads(Path(args.input_x_summary).read_text(encoding="utf-8"))
    compatible_count, scan_rows = read_scan(Path(args.scan))
    full_path = read_full_path(Path(args.full_path_results)) if args.full_path_results else None
    if compatible_count == 0:
        require(full_path is None, "full-path evidence supplied with zero sweep-compatible candidates")
    else:
        require(full_path is not None, "full-path evidence required for sweep-compatible candidates")
    summary = build_summary(input_summary, compatible_count, scan_rows, full_path, args.source_ref)
    Path(args.output).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
