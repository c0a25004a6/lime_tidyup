#!/usr/bin/env python3
"""Summarize deterministic offline fixture-translation search evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

EXPECTED_STATE_COUNT = 2802
EXPECTED_COARSE_CANDIDATE_COUNT = 267761
EXPECTED_FINE_CANDIDATE_COUNT = 4913
EXPECTED_BASELINE_PAIRS = {
    "gripper_left_link|rubiks_cube",
    "gripper_right_link|rubiks_cube",
    "gripper_left_link|rubiks_support",
    "gripper_right_link|rubiks_support",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def boolean(text: str) -> bool:
    require(text in ("true", "false"), f"invalid boolean: {text}")
    return text == "true"


def finite_number(text: str) -> float:
    value = float(text)
    require(math.isfinite(value), f"nonfinite number: {text}")
    return value


def parse_search_row(fields: list[str]) -> dict[str, Any]:
    require(len(fields) == 18, f"invalid {fields[0]} row")
    return {
        "step_m": finite_number(fields[1]),
        "radius_m": finite_number(fields[2]),
        "candidate_count": int(fields[3]),
        "candidates_checked": int(fields[4]),
        "found": boolean(fields[5]),
        "selected_translation_link7_m": [finite_number(value) for value in fields[6:9]],
        "selected_translation_norm_m": finite_number(fields[9]),
        "total_state_checks": int(fields[10]),
        "maximum_states_checked_per_candidate": int(fields[11]),
        "rejected_cube_only": int(fields[12]),
        "rejected_support_only": int(fields[13]),
        "rejected_both": int(fields[14]),
        "rejected_by_phase": {
            "approach": int(fields[15]),
            "candidate_hold": int(fields[16]),
            "return_and_zero_hold": int(fields[17]),
        },
    }


def parse_results(path: Path) -> dict[str, Any]:
    meta = None
    baseline = None
    coarse = None
    fine = None
    selected = None
    states: list[dict[str, Any]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        label = fields[0]
        if label == "META":
            require(len(fields) == 8 and fields[1] == "1", "invalid META row")
            meta = {
                "robot_name": fields[2],
                "root_link": fields[3],
                "state_count": int(fields[4]),
                "anchor_index": int(fields[5]),
                "candidate_start_index": int(fields[6]),
                "reference_link": fields[7],
            }
        elif label == "BASELINE":
            require(len(fields) == 8, "invalid BASELINE row")
            baseline = {
                "clear": boolean(fields[1]),
                "first_collision_state_index": int(fields[2]),
                "first_collision_phase": fields[3],
                "cube_collision": boolean(fields[4]),
                "support_collision": boolean(fields[5]),
                "states_checked": int(fields[6]),
                "contact_pairs": sorted(fields[7].split(",") if fields[7] else []),
            }
        elif label == "COARSE":
            coarse = parse_search_row(fields)
        elif label == "FINE":
            fine = parse_search_row(fields)
        elif label == "SELECT":
            require(len(fields) == 7, "invalid SELECT row")
            selected = {
                "found": boolean(fields[1]),
                "translation_link7_m": [finite_number(value) for value in fields[2:5]],
                "translation_norm_m": finite_number(fields[5]),
                "decision": fields[6],
            }
        elif label == "STATE":
            require(len(fields) == 10, "invalid STATE row")
            states.append({
                "label": fields[1],
                "phase": fields[2],
                "bounds_ok": boolean(fields[3]),
                "self_collision": boolean(fields[4]),
                "self_pairs": fields[5].split(",") if fields[5] else [],
                "cube_collision": boolean(fields[6]),
                "support_collision": boolean(fields[7]),
                "fixture_pair_count": int(fields[8]),
                "fixture_pairs": fields[9].split(",") if fields[9] else [],
            })
        else:
            raise ValueError(f"unknown result row: {label}")

    require(meta is not None, "META evidence missing")
    require(baseline is not None, "BASELINE evidence missing")
    require(coarse is not None and fine is not None, "search evidence missing")
    require(selected is not None, "selection evidence missing")
    return {
        "meta": meta,
        "baseline": baseline,
        "coarse_search": coarse,
        "fine_search": fine,
        "selection": selected,
        "states": states,
    }


def validate_search_stats(search: dict[str, Any], *, candidate_count: int, state_count: int) -> None:
    require(search["candidate_count"] == candidate_count, "search candidate count mismatch")
    require(0 <= search["candidates_checked"] <= candidate_count, "search checked count invalid")
    require(search["total_state_checks"] >= search["candidates_checked"], "search state-check count too small")
    require(0 <= search["maximum_states_checked_per_candidate"] <= state_count, "maximum states checked invalid")
    rejected = search["rejected_cube_only"] + search["rejected_support_only"] + search["rejected_both"]
    expected_rejected = search["candidates_checked"] - (1 if search["found"] else 0)
    require(rejected == expected_rejected, "search rejection count mismatch")
    require(sum(search["rejected_by_phase"].values()) == expected_rejected, "search rejection phase count mismatch")
    if search["found"]:
        require(search["candidates_checked"] >= 1, "found search checked no candidates")
        require(search["maximum_states_checked_per_candidate"] == state_count, "clear candidate did not traverse full path")
    else:
        require(search["candidates_checked"] == candidate_count, "incomplete negative search")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binding", required=True)
    parser.add_argument("--accepted-summary", required=True)
    parser.add_argument("--accepted-manifest", required=True)
    parser.add_argument("--results", required=True)
    parser.add_argument("--states", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    paths = {
        "binding": Path(args.binding),
        "accepted_summary": Path(args.accepted_summary),
        "accepted_manifest": Path(args.accepted_manifest),
        "results": Path(args.results),
        "states": Path(args.states),
        "fixture": Path(args.fixture),
        "urdf": Path(args.urdf),
        "srdf": Path(args.srdf),
    }
    binding = json.loads(paths["binding"].read_text())
    accepted_summary = json.loads(paths["accepted_summary"].read_text())
    accepted_manifest = json.loads(paths["accepted_manifest"].read_text())
    evidence = parse_results(paths["results"])

    require(binding.get("source_ref") == args.source_ref, "binding source mismatch")
    require(binding.get("passed") is True and binding.get("errors") == [], "input binding failed")
    require(accepted_summary.get("source_ref") == binding.get("accepted_source"), "accepted summary source mismatch")
    require(accepted_summary.get("decision") == "BLOCKED_ROBOT_FIXTURE_COLLISION", "accepted blocker decision mismatch")
    require(accepted_manifest.get("source_ref") == binding.get("accepted_source"), "accepted manifest source mismatch")
    require(accepted_manifest.get("decision") == "BLOCKED_ROBOT_FIXTURE_COLLISION", "accepted manifest blocker mismatch")

    meta = evidence["meta"]
    baseline = evidence["baseline"]
    coarse = evidence["coarse_search"]
    fine = evidence["fine_search"]
    selection = evidence["selection"]
    states = evidence["states"]

    require(meta == {
        "robot_name": "turtlebot3_lime",
        "root_link": "base_footprint",
        "state_count": EXPECTED_STATE_COUNT,
        "anchor_index": 1541,
        "candidate_start_index": 1043,
        "reference_link": "link7",
    }, "search META contract mismatch")
    require(baseline["clear"] is False, "baseline unexpectedly clear")
    require(baseline["first_collision_state_index"] == 0, "baseline first collision mismatch")
    require(baseline["first_collision_phase"] == "approach", "baseline first collision phase mismatch")
    require(baseline["cube_collision"] is True and baseline["support_collision"] is True, "baseline object collision mismatch")
    require(baseline["states_checked"] == 1, "baseline did not fail at first state")
    require(set(baseline["contact_pairs"]) == EXPECTED_BASELINE_PAIRS, "baseline contact-pair set mismatch")

    require(math.isclose(coarse["step_m"], 0.002, abs_tol=1e-15), "coarse step mismatch")
    require(math.isclose(coarse["radius_m"], 0.08, abs_tol=1e-15), "coarse radius mismatch")
    validate_search_stats(coarse, candidate_count=EXPECTED_COARSE_CANDIDATE_COUNT, state_count=EXPECTED_STATE_COUNT)

    found = selection["found"]
    require(coarse["found"] is found, "coarse/selection found mismatch")
    if found:
        require(math.isclose(fine["step_m"], 0.00025, abs_tol=1e-15), "fine step mismatch")
        require(math.isclose(fine["radius_m"], 0.002, abs_tol=1e-15), "fine radius mismatch")
        validate_search_stats(fine, candidate_count=EXPECTED_FINE_CANDIDATE_COUNT, state_count=EXPECTED_STATE_COUNT)
        require(fine["found"] is True, "fine search lost coarse clear candidate")
        require(selection["decision"] == "FULL_PATH_CLEAR_TRANSLATION_FOUND", "positive selection decision mismatch")
        require(len(states) == EXPECTED_STATE_COUNT, "selected candidate state matrix incomplete")
        for index, state in enumerate(states):
            require(state["label"] == f"sample_{index:06d}", f"selected state label mismatch at {index}")
            expected_phase = "approach" if index < 1043 else ("candidate_hold" if index <= 1541 else "return_and_zero_hold")
            require(state["phase"] == expected_phase, f"selected state phase mismatch at {index}")
            require(state["bounds_ok"] is True, f"selected state bounds failure at {index}")
            require(state["self_collision"] is False and state["self_pairs"] == [], f"selected state self collision at {index}")
            require(state["cube_collision"] is False, f"selected state cube collision at {index}")
            require(state["support_collision"] is False, f"selected state support collision at {index}")
            require(state["fixture_pair_count"] == 0 and state["fixture_pairs"] == [], f"selected state fixture contacts at {index}")
    else:
        require(fine["candidate_count"] == 0 and fine["candidates_checked"] == 0, "negative search unexpectedly ran fine grid")
        require(fine["found"] is False, "negative fine search marked found")
        require(selection["decision"] == "NO_FULL_PATH_CLEAR_TRANSLATION_WITHIN_COARSE_RADIUS", "negative selection decision mismatch")
        require(states == [], "negative search unexpectedly emitted selected states")

    selected_delta = selection["translation_link7_m"]
    require(len(selected_delta) == 3, "selected translation dimension mismatch")
    recomputed_norm = math.sqrt(sum(value * value for value in selected_delta))
    require(math.isclose(recomputed_norm, selection["translation_norm_m"], rel_tol=0.0, abs_tol=1e-12), "selected translation norm mismatch")
    if found:
        require(all(abs(value) <= 0.0820000001 for value in selected_delta), "selected translation outside search/refinement domain")
        require(selection["translation_norm_m"] <= coarse["selected_translation_norm_m"] + 1e-12, "fine result is farther than coarse result")

    cube_nominal = [-0.019, 0.0, 0.1192]
    support_nominal = [-0.019, 0.0, 0.0857]
    cube_corrected = [a + b for a, b in zip(cube_nominal, selected_delta)]
    support_corrected = [a + b for a, b in zip(support_nominal, selected_delta)]
    relative_before = [a - b for a, b in zip(cube_nominal, support_nominal)]
    relative_after = [a - b for a, b in zip(cube_corrected, support_corrected)]
    require(max(abs(a - b) for a, b in zip(relative_before, relative_after)) <= 1e-15, "cube/support relative transform changed")

    decision = selection["decision"]
    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-FIXTURE-RELATIVE-ALIGNMENT-CORRECTION-PREFLIGHT",
        "writer_lease": "WL-RUBIK-PREGRASP-FIXTURE-ALIGNMENT-SEARCH-20260804-01",
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "audit_passed": True,
        "errors": [],
        "decision": decision,
        "correction_candidate_found": found,
        "correction_phase_ready": found,
        "accepted_blocker_source": binding.get("accepted_source"),
        "input_binding": binding,
        "baseline_reproduction": baseline,
        "coarse_search": coarse,
        "fine_search": fine,
        "selected_candidate": {
            **selection,
            "cube_offset_from_link7_m": cube_corrected if found else None,
            "support_offset_from_link7_m": support_corrected if found else None,
            "cube_support_relative_transform_preserved": True,
            "translation_within_cube_half_extent_box": found and all(abs(value) <= 0.0285 for value in selected_delta),
            "translation_norm_less_than_cube_edge": found and selection["translation_norm_m"] <= 0.057,
            "full_recorded_path_clear": found,
        },
        "selected_path_audit": {
            "state_count": len(states),
            "all_bounds_ok": found and all(state["bounds_ok"] for state in states),
            "all_self_collision_free": found and all(not state["self_collision"] for state in states),
            "all_cube_clear": found and all(not state["cube_collision"] for state in states),
            "all_support_clear": found and all(not state["support_collision"] for state in states),
            "contact_pair_count": sum(state["fixture_pair_count"] for state in states),
        },
        "claim_limits": {
            "nearest_clear_candidate_on_coarse_grid_then_local_fine_grid_only": True,
            "continuous_global_optimum_claimed": False,
            "rotation_search_performed": False,
            "grasp_relevance_claimed": False,
            "contact_quality_claimed": False,
            "distance_margin_claimed": False,
            "gazebo_stability_claimed": False,
            "actuation_or_hardware_readiness_claimed": False,
        },
        "safety": {
            "offline_analysis_only": True,
            "accepted_passive_finger_states_unchanged": True,
            "cube_support_rigid_relation_preserved": True,
            "gazebo_started": False,
            "ros_node_created": False,
            "object_spawned": False,
            "controller_loaded": False,
            "planning_request_created": False,
            "trajectory_instantiated": False,
            "arm_command_sent": False,
            "gripper_command_sent": False,
            "attachment_used": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
        },
        "next_required_phase": (
            "RUBIK-PREGRASP-CORRECTED-FIXTURE-GEOMETRIC-RELEVANCE-PREFLIGHT"
            if found else
            "RUBIK-PREGRASP-ACTIVE-GRIPPER-OPEN-STATE-OFFLINE-PREFLIGHT"
        ),
    }
    output = Path(args.output)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    manifest = {
        "schema_version": 1,
        "phase": summary["phase"],
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "summary": output.name,
        "summary_sha256": sha256(output),
        "inputs": {path.name: sha256(path) for path in paths.values()},
        "decision": decision,
        "correction_candidate_found": found,
        "state_count": len(states),
        "coarse_candidate_count": coarse["candidate_count"],
        "coarse_candidates_checked": coarse["candidates_checked"],
        "fine_candidate_count": fine["candidate_count"],
        "fine_candidates_checked": fine["candidates_checked"],
        "offline_analysis_only": True,
        "physical_hardware_used": False,
        "production_runtime_modified": False,
    }
    Path(args.manifest).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
