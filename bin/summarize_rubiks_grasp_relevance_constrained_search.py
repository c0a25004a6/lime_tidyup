#!/usr/bin/env python3
"""Summarize relevance-filtered full-path fixture translation search."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

PR23_SOURCE = "fa78493ca98b5c9701ccf2f6da5ae0ee3a2cbcee"
PR24_SOURCE = "7d8b9b055cd331d11b89ba275a1ecce7312b4b84"
STATE_COUNT = 2802


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_candidates(path: Path) -> dict[str, object]:
    meta = None
    rows = []
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 9 and fields[1] == "1", "invalid candidate META")
            meta = {
                "mode": fields[2],
                "step_m": float(fields[3]),
                "enumerated": int(fields[4]),
                "relevant": int(fields[5]),
                "center_m": [float(value) for value in fields[6:9]],
            }
        elif fields[0] == "CANDIDATE":
            require(len(fields) == 12, "invalid candidate row")
            row = {
                "rank": int(fields[1]),
                "translation_m": [float(value) for value in fields[2:5]],
                "norm_m": float(fields[5]),
                "left_x_overlap_m": float(fields[6]),
                "left_z_overlap_m": float(fields[7]),
                "right_x_overlap_m": float(fields[8]),
                "right_z_overlap_m": float(fields[9]),
                "common_low_m": float(fields[10]),
                "common_high_m": float(fields[11]),
            }
            require(min(row["left_x_overlap_m"], row["left_z_overlap_m"], row["right_x_overlap_m"], row["right_z_overlap_m"]) > 0.0, "candidate lost XZ reach")
            require(row["common_low_m"] <= row["common_high_m"] + 1e-12, "candidate lost mimic interval")
            rows.append(row)
        else:
            raise ValueError(f"unknown candidate row: {fields[0]}")
    require(meta is not None and rows, "candidate file incomplete")
    require(meta["relevant"] == len(rows), "candidate count mismatch")
    for index, row in enumerate(rows):
        require(row["rank"] == index, "candidate rank mismatch")
    return {"meta": meta, "rows": rows}


def parse_eval(path: Path) -> dict[str, object]:
    meta = baseline = selected = None
    rows = []
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 6 and fields[1] == "1", "invalid evaluator META")
            meta = {"robot_name": fields[2], "state_count": int(fields[3]), "candidate_count": int(fields[4]), "anchor_index": int(fields[5])}
        elif fields[0] == "BASELINE":
            require(len(fields) == 4, "invalid BASELINE row")
            baseline = {"clear": fields[1] == "true", "first_collision_index": int(fields[2]), "pairs": fields[3].split(",") if fields[3] else []}
        elif fields[0] == "CANDIDATE":
            require(len(fields) == 13, "invalid evaluator CANDIDATE row")
            rows.append({
                "rank": int(fields[1]),
                "translation_m": [float(value) for value in fields[2:5]],
                "norm_m": float(fields[5]),
                "clear": fields[6] == "true",
                "states_checked": int(fields[7]),
                "first_collision_index": int(fields[8]),
                "first_collision_phase": fields[9],
                "cube_collision": fields[10] == "true",
                "support_collision": fields[11] == "true",
                "pairs": fields[12].split(",") if fields[12] else [],
            })
        elif fields[0] == "SELECT":
            require(len(fields) == 8, "invalid SELECT row")
            selected = {
                "found": fields[1] == "true",
                "translation_m": [float(value) for value in fields[2:5]],
                "norm_m": float(fields[5]),
                "total_state_checks": int(fields[6]),
                "selected_states_checked": int(fields[7]),
            }
        else:
            raise ValueError(f"unknown evaluator row: {fields[0]}")
    require(meta and baseline and selected, "evaluator output incomplete")
    require(meta["robot_name"] == "turtlebot3_lime", "robot identity mismatch")
    require(meta["state_count"] == STATE_COUNT and meta["anchor_index"] == 1541, "state identity mismatch")
    expected = sorted(["gripper_left_link|rubiks_cube", "gripper_left_link|rubiks_support", "gripper_right_link|rubiks_cube", "gripper_right_link|rubiks_support"])
    require(not baseline["clear"] and baseline["first_collision_index"] == 0 and sorted(baseline["pairs"]) == expected, "PR22 blocker baseline mismatch")
    if selected["found"]:
        require(rows and rows[-1]["clear"], "selected clear row missing")
        require(rows[-1]["states_checked"] == STATE_COUNT and selected["selected_states_checked"] == STATE_COUNT, "selected candidate not full-path checked")
        require(rows[-1]["translation_m"] == selected["translation_m"], "selected translation mismatch")
    return {"meta": meta, "baseline": baseline, "rows": rows, "selected": selected}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pr23-summary", required=True)
    parser.add_argument("--pr24-summary", required=True)
    parser.add_argument("--coarse-candidates", required=True)
    parser.add_argument("--coarse-results", required=True)
    parser.add_argument("--fine-candidates")
    parser.add_argument("--fine-results")
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    pr23 = json.loads(Path(args.pr23_summary).read_text())
    pr24 = json.loads(Path(args.pr24_summary).read_text())
    require(pr23.get("source_ref") == PR23_SOURCE and pr23.get("audit_passed") is True and pr23.get("decision") == "FULL_PATH_CLEAR_TRANSLATION_FOUND", "PR23 evidence mismatch")
    require(pr24.get("source_ref") == PR24_SOURCE and pr24.get("audit_passed") is True and pr24.get("decision") == "BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE", "PR24 evidence mismatch")

    coarse_candidates = parse_candidates(Path(args.coarse_candidates))
    coarse = parse_eval(Path(args.coarse_results))
    require(coarse["meta"]["candidate_count"] == coarse_candidates["meta"]["relevant"], "coarse coverage mismatch")

    fine_candidates = fine = None
    if coarse["selected"]["found"]:
        require(args.fine_candidates and args.fine_results, "coarse clear requires fine search")
        fine_candidates = parse_candidates(Path(args.fine_candidates))
        fine = parse_eval(Path(args.fine_results))
        require(fine["meta"]["candidate_count"] == fine_candidates["meta"]["relevant"], "fine coverage mismatch")

    found = bool(fine and fine["selected"]["found"])
    if found:
        decision = "GRASP_RELEVANT_FULL_PATH_CLEAR_TRANSLATION_FOUND"
        next_phase = "RUBIK-PREGRASP-CORRECTED-FIXTURE-OFFLINE-GRIPPER-SWEEP-COLLISION-PREFLIGHT"
        selected = fine["selected"]
    elif coarse["selected"]["found"]:
        decision = "BLOCKED_FINE_REFINEMENT_LOST_FULL_PATH_CLEAR_TRANSLATION"
        next_phase = "RUBIK-PREGRASP-GRASP-RELEVANCE-CONSTRAINED-FIXTURE-SEARCH-EXPANSION"
        selected = None
    else:
        decision = "NO_GRASP_RELEVANT_FULL_PATH_CLEAR_TRANSLATION_IN_COARSE_DOMAIN"
        next_phase = "RUBIK-PREGRASP-GRASP-RELEVANCE-CONSTRAINED-FIXTURE-SEARCH-EXPANSION"
        selected = None

    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-GRASP-RELEVANCE-CONSTRAINED-FIXTURE-SEARCH",
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "audit_passed": True,
        "errors": [],
        "decision": decision,
        "candidate_found": found,
        "selected_candidate": selected,
        "coarse": {"generator": coarse_candidates["meta"], "evaluated_before_stop": len(coarse["rows"]), "selected": coarse["selected"]},
        "fine": None if fine is None else {"generator": fine_candidates["meta"], "evaluated_before_stop": len(fine["rows"]), "selected": fine["selected"]},
        "full_path_state_count": STATE_COUNT,
        "relevance_constraints": {"positive_left_right_xz_aabb_overlap_required": True, "common_exact_limit_mimic_side_plane_interval_required": True, "cube_support_rigid_transform_preserved": True},
        "next_required_phase": next_phase,
        "claim_limits": {"grid_search_only": True, "continuous_space_optimum_claimed": False, "continuous_between_sample_clearance_claimed": False, "exact_mesh_contact_claimed": False, "force_closure_claimed": False, "grasp_success_claimed": False, "hardware_readiness_claimed": False},
        "safety": {"offline_analysis_only": True, "accepted_states_unchanged": True, "gazebo_started": False, "ros_node_created": False, "object_spawned": False, "controller_loaded": False, "planning_request_created": False, "trajectory_instantiated": False, "command_sent": False, "attachment_used": False, "physical_hardware_used": False, "production_runtime_modified": False},
    }
    output = Path(args.output)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    inputs = {"pr23_summary": sha256(Path(args.pr23_summary)), "pr24_summary": sha256(Path(args.pr24_summary)), "coarse_candidates": sha256(Path(args.coarse_candidates)), "coarse_results": sha256(Path(args.coarse_results))}
    if args.fine_candidates:
        inputs["fine_candidates"] = sha256(Path(args.fine_candidates))
    if args.fine_results:
        inputs["fine_results"] = sha256(Path(args.fine_results))
    manifest = {"schema_version": 1, "source_ref": args.source_ref, "decision": decision, "candidate_found": found, "selected_translation_m": None if not found else selected["translation_m"], "inputs": inputs, "summary": {"path": output.name, "sha256": sha256(output)}}
    Path(args.manifest).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
