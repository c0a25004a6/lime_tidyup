#!/usr/bin/env python3
"""Summarize command-free corrected-fixture gripper sweep evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

PR26_SOURCE = "3ecd9f6533815a0cee89085dc9cc165077c6f510"
EXPECTED_TRANSLATION = [0.06325, 0.0, 0.01225]
EXPECTED_STATES = 2802
EXPECTED_ANCHOR = 1541
ALLOWED_DECISIONS = {
    "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND",
    "BLOCKED_NO_COLLISION_FREE_OPENING_STATE",
    "BLOCKED_FORBIDDEN_COLLISION_BEFORE_DUAL_CONTACT",
    "BLOCKED_NO_DUAL_FINGER_CUBE_CONTACT_IN_LIMITS",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def close_vector(actual: list[float], expected: list[float], tolerance: float = 1e-12) -> bool:
    return len(actual) == len(expected) and all(
        math.isfinite(value) and abs(value - target) <= tolerance
        for value, target in zip(actual, expected)
    )


def parse_results(path: Path) -> dict[str, object]:
    meta = None
    rows = []
    decision = None
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 12 and fields[1] == "1", "invalid META row")
            meta = {
                "robot_name": fields[2],
                "anchor_index": int(fields[3]),
                "accepted_state_count": int(fields[4]),
                "lower_m": float(fields[5]),
                "upper_m": float(fields[6]),
                "step_m": float(fields[7]),
                "sweep_state_count": int(fields[8]),
                "translation_m": [float(value) for value in fields[9:12]],
            }
        elif fields[0] == "STATE":
            require(len(fields) == 17, "invalid STATE row")
            rows.append({
                "index": int(fields[1]),
                "left_q_m": float(fields[2]),
                "right_q_m": float(fields[3]),
                "classification": fields[4],
                "bounds_ok": fields[5] == "true",
                "self_collision": fields[6] == "true",
                "cube_collision": fields[7] == "true",
                "support_collision": fields[8] == "true",
                "ground_collision": fields[9] == "true",
                "left_cube_contact": fields[10] == "true",
                "right_cube_contact": fields[11] == "true",
                "forbidden_cube_contact": fields[12] == "true",
                "cube_pairs": fields[13].split(",") if fields[13] else [],
                "support_pairs": fields[14].split(",") if fields[14] else [],
                "ground_pairs": fields[15].split(",") if fields[15] else [],
                "self_pairs": fields[16].split(",") if fields[16] else [],
            })
        elif fields[0] == "DECISION":
            require(len(fields) == 11, "invalid DECISION row")
            decision = {
                "decision": fields[1],
                "open_clear_found": fields[2] == "true",
                "first_open_index": int(fields[3]),
                "desired_contact_found": fields[4] == "true",
                "first_contact_index": int(fields[5]),
                "first_contact_q_m": float(fields[6]),
                "dual_contact_found": fields[7] == "true",
                "first_dual_index": int(fields[8]),
                "first_dual_q_m": float(fields[9]),
                "forbidden_before_dual": fields[10] == "true",
            }
        else:
            raise ValueError(f"unknown result row: {fields[0]}")
    require(meta is not None and rows and decision is not None, "sweep evidence incomplete")
    require(meta["sweep_state_count"] == len(rows), "sweep state count mismatch")
    for index, row in enumerate(rows):
        require(row["index"] == index, "sweep index mismatch")
        require(math.isfinite(row["left_q_m"]) and math.isfinite(row["right_q_m"]), "nonfinite gripper state")
        require(abs(row["left_q_m"] - row["right_q_m"]) <= 1e-12, "MoveIt mimic mismatch")
        if index:
            require(row["left_q_m"] <= rows[index - 1]["left_q_m"] + 1e-15, "sweep is not closing monotonically")
        if row["classification"] == "OPEN_CLEAR":
            require(not row["cube_collision"] and not row["support_collision"] and not row["ground_collision"] and not row["self_collision"], "open-clear row contains collision")
        if row["classification"] == "DUAL_FINGER_CUBE":
            require(row["left_cube_contact"] and row["right_cube_contact"], "dual row lacks both finger contacts")
        if not row["forbidden_cube_contact"]:
            allowed = {"gripper_left_link|rubiks_cube", "gripper_right_link|rubiks_cube"}
            require(set(row["cube_pairs"]).issubset(allowed), "unexpected cube contact pair")
    return {"meta": meta, "rows": rows, "decision": decision}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pr26-summary", required=True)
    parser.add_argument("--results", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--states", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    pr26 = json.loads(Path(args.pr26_summary).read_text())
    require(pr26.get("source_ref") == PR26_SOURCE, "PR26 source mismatch")
    require(pr26.get("audit_passed") is True and pr26.get("errors") == [], "PR26 evidence failed")
    require(pr26.get("decision") == "GRASP_RELEVANT_FULL_PATH_CLEAR_TRANSLATION_FOUND", "PR26 decision mismatch")
    require(pr26.get("candidate_found") is True, "PR26 selected candidate missing")
    selected = pr26.get("selected_candidate", {})
    require(close_vector(selected.get("translation_m", []), EXPECTED_TRANSLATION), "PR26 selected translation mismatch")
    require(selected.get("selected_states_checked") == EXPECTED_STATES, "PR26 full-path count mismatch")

    parsed = parse_results(Path(args.results))
    meta = parsed["meta"]
    rows = parsed["rows"]
    evidence = parsed["decision"]
    require(meta["robot_name"] == "turtlebot3_lime", "MoveIt model mismatch")
    require(meta["anchor_index"] == EXPECTED_ANCHOR, "anchor mismatch")
    require(meta["accepted_state_count"] == EXPECTED_STATES, "accepted state count mismatch")
    require(close_vector(meta["translation_m"], EXPECTED_TRANSLATION), "sweep translation mismatch")
    require(abs(meta["lower_m"] + 0.01) <= 1e-12 and abs(meta["upper_m"] - 0.019) <= 1e-12, "gripper limits mismatch")
    require(abs(meta["step_m"] - 0.00001) <= 1e-15, "sweep step mismatch")
    require(evidence["decision"] in ALLOWED_DECISIONS, "unexpected sweep decision")

    counts = {}
    for row in rows:
        counts[row["classification"]] = counts.get(row["classification"], 0) + 1
    positive = evidence["decision"] == "OFFLINE_GRIPPER_SWEEP_OPEN_TO_DUAL_CONTACT_PATH_FOUND"
    if positive:
        require(evidence["open_clear_found"] is True, "positive result lacks open-clear state")
        require(evidence["dual_contact_found"] is True, "positive result lacks dual contact")
        require(evidence["forbidden_before_dual"] is False, "positive prefix contains forbidden collision")
        require(evidence["first_open_index"] <= evidence["first_contact_index"] <= evidence["first_dual_index"], "contact ordering mismatch")
        prefix = rows[: evidence["first_dual_index"] + 1]
        require(all(row["classification"] != "FORBIDDEN" for row in prefix), "positive prefix contains forbidden row")
    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-CORRECTED-FIXTURE-OFFLINE-GRIPPER-SWEEP-COLLISION-PREFLIGHT",
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "audit_passed": True,
        "errors": [],
        "decision": evidence["decision"],
        "open_to_dual_contact_path_found": positive,
        "selected_translation_link7_m": EXPECTED_TRANSLATION,
        "anchor_index": EXPECTED_ANCHOR,
        "sweep": {
            **meta,
            "classification_counts": counts,
            "first_open_index": evidence["first_open_index"],
            "first_contact_index": evidence["first_contact_index"],
            "first_contact_q_m": evidence["first_contact_q_m"],
            "first_dual_index": evidence["first_dual_index"],
            "first_dual_q_m": evidence["first_dual_q_m"],
            "forbidden_before_dual": evidence["forbidden_before_dual"],
        },
        "next_required_phase": (
            "RUBIK-PREGRASP-CORRECTED-FIXTURE-GRIPPER-CONTACT-BOUNDARY-REFINEMENT"
            if positive else
            "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-CORRECTION"
        ),
        "claim_limits": {
            "sampled_gripper_positions_only": True,
            "gripper_step_m": 0.00001,
            "continuous_gripper_clearance_claimed": False,
            "exact_surface_touch_claimed": False,
            "contact_normal_claimed": False,
            "force_closure_claimed": False,
            "friction_claimed": False,
            "grasp_success_claimed": False,
            "lifting_or_transport_claimed": False,
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
    paths = [Path(args.pr26_summary), Path(args.results), Path(args.urdf), Path(args.srdf), Path(args.states), Path(args.fixture)]
    manifest = {
        "schema_version": 1,
        "source_ref": args.source_ref,
        "decision": summary["decision"],
        "open_to_dual_contact_path_found": positive,
        "selected_translation_link7_m": EXPECTED_TRANSLATION,
        "inputs": {path.name: digest(path) for path in paths},
        "summary": {"path": output.name, "sha256": digest(output)},
    }
    Path(args.manifest).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
