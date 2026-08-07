#!/usr/bin/env python3
"""Summarize local X compatibility sweep and passive full-path evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

PR27_SOURCE = "bffc9619efe3db79ab89cb282a93e4d1f1ae6ca1"
CENTER = [0.06325, 0.0, 0.01225]
POSITIVE_SWEEP = "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_scan(path: Path) -> dict[str, object]:
    meta = None
    rows = []
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 5 and fields[1:3] == ["1", "X"], "invalid scan META")
            meta = {"candidate_count": int(fields[3]), "compatible_count": int(fields[4])}
        elif fields[0] == "SCAN":
            require(len(fields) == 13, "invalid SCAN row")
            rows.append({
                "rank": int(fields[1]),
                "translation_m": [float(value) for value in fields[2:5]],
                "distance_from_center_m": float(fields[5]),
                "offset_steps": int(fields[6]),
                "sweep_decision": fields[7],
                "open_clear": fields[8] == "true",
                "first_contact_q_m": float(fields[9]),
                "dual_contact": fields[10] == "true",
                "first_dual_q_m": float(fields[11]),
                "forbidden_before_dual": fields[12] == "true",
            })
        else:
            raise ValueError(f"unknown scan row: {fields[0]}")
    require(meta is not None and len(rows) == meta["candidate_count"] == 17, "scan coverage mismatch")
    compatible = [row for row in rows if row["sweep_decision"] == POSITIVE_SWEEP]
    require(len(compatible) == meta["compatible_count"], "compatible count mismatch")
    for row in compatible:
        require(row["open_clear"] and row["dual_contact"] and not row["forbidden_before_dual"], "invalid positive sweep row")
    return {"meta": meta, "rows": rows, "compatible": compatible}


def parse_full_path(path: Path | None) -> dict[str, object] | None:
    if path is None or not path.is_file() or path.stat().st_size == 0:
        return None
    selected = None
    baseline = None
    for raw in path.read_text().splitlines():
        fields = raw.split("\t")
        if fields[0] == "BASELINE":
            require(len(fields) == 4, "invalid full-path BASELINE")
            baseline = {"clear": fields[1] == "true", "first_collision_index": int(fields[2]), "pairs": fields[3].split(",") if fields[3] else []}
        elif fields[0] == "SELECT":
            require(len(fields) == 8, "invalid full-path SELECT")
            selected = {
                "found": fields[1] == "true",
                "translation_m": [float(value) for value in fields[2:5]],
                "norm_m": float(fields[5]),
                "total_state_checks": int(fields[6]),
                "selected_states_checked": int(fields[7]),
            }
    require(baseline is not None and selected is not None, "full-path evidence incomplete")
    require(not baseline["clear"] and baseline["first_collision_index"] == 0, "PR22 baseline not reproduced")
    if selected["found"]:
        require(selected["selected_states_checked"] == 2802, "selected candidate lacks 2,802-state evidence")
    return {"baseline": baseline, "selected": selected}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pr27-summary", required=True)
    parser.add_argument("--candidates", required=True)
    parser.add_argument("--scan", required=True)
    parser.add_argument("--compatible-candidates", required=True)
    parser.add_argument("--full-path-results")
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    pr27 = json.loads(Path(args.pr27_summary).read_text())
    require(pr27.get("source_ref") == PR27_SOURCE, "PR27 source mismatch")
    require(pr27.get("audit_passed") is True and pr27.get("errors") == [], "PR27 evidence failed")
    require(pr27.get("decision") == "BLOCKED_FORBIDDEN_COLLISION_BEFORE_DUAL_CONTACT", "PR27 decision mismatch")
    require(pr27.get("selected_translation_link7_m") == CENTER, "PR27 center mismatch")

    scan = parse_scan(Path(args.scan))
    full_path_path = Path(args.full_path_results) if args.full_path_results else None
    full_path = parse_full_path(full_path_path)
    if not scan["compatible"]:
        require(full_path is None, "full path should not run without sweep-compatible candidates")
        decision = "NO_SWEEP_COMPATIBLE_TRANSLATION_IN_LOCAL_X_DOMAIN"
        selected = None
        next_phase = "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-3D-EXPANSION"
    else:
        require(full_path is not None, "sweep-compatible candidates require full-path evaluation")
        if full_path["selected"]["found"]:
            selected = full_path["selected"]
            match = next((row for row in scan["compatible"] if row["translation_m"] == selected["translation_m"]), None)
            require(match is not None, "full-path selection is not sweep-compatible")
            selected = {**selected, "sweep": match}
            decision = "GRIPPER_FIXTURE_COMPATIBLE_FULL_PATH_CLEAR_TRANSLATION_FOUND"
            next_phase = "RUBIK-PREGRASP-CORRECTED-FIXTURE-STATIC-GAZEBO-NO-ACTUATION-EVIDENCE"
        else:
            selected = None
            decision = "NO_SWEEP_COMPATIBLE_FULL_PATH_CLEAR_TRANSLATION_IN_LOCAL_X_DOMAIN"
            next_phase = "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-3D-EXPANSION"

    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-CORRECTION",
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "audit_passed": True,
        "errors": [],
        "decision": decision,
        "candidate_found": selected is not None,
        "center_translation_link7_m": CENTER,
        "x_scan": {
            "step_m": 0.00025,
            "radius_m": 0.002,
            "candidate_count": scan["meta"]["candidate_count"],
            "sweep_compatible_count": scan["meta"]["compatible_count"],
        },
        "selected_candidate": selected,
        "next_required_phase": next_phase,
        "claim_limits": {
            "x_axis_local_search_only": True,
            "continuous_translation_optimum_claimed": False,
            "continuous_gripper_clearance_claimed": False,
            "exact_surface_touch_claimed": False,
            "force_closure_claimed": False,
            "grasp_success_claimed": False,
            "hardware_readiness_claimed": False,
        },
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
    }
    output = Path(args.output)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    inputs = {
        "pr27_summary": digest(Path(args.pr27_summary)),
        "candidates": digest(Path(args.candidates)),
        "scan": digest(Path(args.scan)),
        "compatible_candidates": digest(Path(args.compatible_candidates)),
    }
    if full_path_path and full_path_path.is_file():
        inputs["full_path_results"] = digest(full_path_path)
    manifest = {
        "schema_version": 1,
        "source_ref": args.source_ref,
        "decision": decision,
        "candidate_found": selected is not None,
        "selected_translation_m": None if selected is None else selected["translation_m"],
        "inputs": inputs,
        "summary": {"path": output.name, "sha256": digest(output)},
    }
    Path(args.manifest).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
