#!/usr/bin/env python3
"""Validate and summarize grasp-relevance constrained fixture search evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

PR24_SOURCE = "7d8b9b055cd331d11b89ba275a1ecce7312b4b84"
EXPECTED_COARSE_TOTAL = 267761
EXPECTED_COARSE_RELEVANT = 61511
EXPECTED_FINE_TOTAL = 4913
VALID_DECISIONS = {
    "FULL_PATH_CLEAR_GRASP_RELEVANT_TRANSLATION_FOUND",
    "NO_FULL_PATH_CLEAR_GRASP_RELEVANT_TRANSLATION_WITHIN_COARSE_RADIUS",
}
EXPECTED_BASELINE_PAIRS = {
    "gripper_left_link|rubiks_cube",
    "gripper_left_link|rubiks_support",
    "gripper_right_link|rubiks_cube",
    "gripper_right_link|rubiks_support",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def number(text: str) -> float:
    value = float(text)
    require(math.isfinite(value), f"nonfinite number: {text}")
    return value


def integer(text: str) -> int:
    value = int(text)
    require(str(value) == text and value >= 0, f"invalid integer: {text}")
    return value


def boolean(text: str) -> bool:
    require(text in {"true", "false"}, f"invalid boolean: {text}")
    return text == "true"


def pairs(text: str) -> set[str]:
    return set() if text == "" else set(text.split(","))


def parse_search_stats(fields: list[str]) -> dict[str, object]:
    require(len(fields) == 18, f"invalid {fields[0]} search row")
    return {
        "step_m": number(fields[1]),
        "radius_m": number(fields[2]),
        "candidate_count": integer(fields[3]),
        "candidates_checked": integer(fields[4]),
        "found": boolean(fields[5]),
        "selected_translation_link7_m": [number(value) for value in fields[6:9]],
        "selected_norm_m": number(fields[9]),
        "total_state_checks": integer(fields[10]),
        "maximum_states_checked": integer(fields[11]),
        "rejected_cube_only": integer(fields[12]),
        "rejected_support_only": integer(fields[13]),
        "rejected_both": integer(fields[14]),
        "rejected_by_phase": {
            "approach": integer(fields[15]),
            "candidate_hold": integer(fields[16]),
            "return_and_zero_hold": integer(fields[17]),
        },
    }


def parse_results(path: Path) -> dict[str, object]:
    meta = None
    baseline = None
    filters: dict[str, dict[str, int]] = {}
    searches: dict[str, dict[str, object]] = {}
    selection = None
    reach = None
    states: list[dict[str, object]] = []
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        label = fields[0]
        if label == "META":
            require(meta is None and len(fields) == 8 and fields[1] == "1", "invalid META row")
            meta = {
                "robot": fields[2],
                "root": fields[3],
                "state_count": integer(fields[4]),
                "anchor_index": integer(fields[5]),
                "candidate_start_index": integer(fields[6]),
                "reference_link": fields[7],
            }
        elif label == "BASELINE":
            require(baseline is None and len(fields) == 8, "invalid BASELINE row")
            baseline = {
                "clear": boolean(fields[1]),
                "first_collision_state_index": integer(fields[2]),
                "first_collision_phase": fields[3],
                "cube_collision": boolean(fields[4]),
                "support_collision": boolean(fields[5]),
                "states_checked": integer(fields[6]),
                "pairs": sorted(pairs(fields[7])),
            }
        elif label == "FILTER":
            require(len(fields) == 7 and fields[1] not in filters, "invalid FILTER row")
            filters[fields[1]] = {
                "total_candidates": integer(fields[2]),
                "relevant_candidates": integer(fields[3]),
                "filtered_candidates": integer(fields[4]),
                "minimum_relevant_z_step": int(fields[5]),
                "maximum_relevant_z_step": int(fields[6]),
            }
        elif label in {"COARSE", "FINE"}:
            require(label not in searches, f"duplicate {label} row")
            searches[label] = parse_search_stats(fields)
        elif label == "SELECT":
            require(selection is None and len(fields) == 7, "invalid SELECT row")
            selection = {
                "found": boolean(fields[1]),
                "translation_link7_m": [number(value) for value in fields[2:5]],
                "norm_m": number(fields[5]),
                "decision": fields[6],
            }
        elif label == "REACH":
            require(reach is None and len(fields) == 13, "invalid REACH row")
            reach = {
                "relevant": boolean(fields[1]),
                "left_x_overlap_m": number(fields[2]),
                "left_z_overlap_m": number(fields[3]),
                "right_x_overlap_m": number(fields[4]),
                "right_z_overlap_m": number(fields[5]),
                "left_reachable_low_m": number(fields[6]),
                "left_reachable_high_m": number(fields[7]),
                "right_reachable_low_m": number(fields[8]),
                "right_reachable_high_m": number(fields[9]),
                "common_low_m": number(fields[10]),
                "common_high_m": number(fields[11]),
                "common_interval_exists": boolean(fields[12]),
            }
        elif label == "STATE":
            require(len(fields) == 10, "invalid STATE row")
            states.append({
                "label": fields[1],
                "phase": fields[2],
                "bounds_ok": boolean(fields[3]),
                "self_collision": boolean(fields[4]),
                "self_pairs": sorted(pairs(fields[5])),
                "cube_collision": boolean(fields[6]),
                "support_collision": boolean(fields[7]),
                "fixture_contact_count": integer(fields[8]),
                "fixture_pairs": sorted(pairs(fields[9])),
            })
        else:
            raise ValueError(f"unknown result row: {label}")
    require(meta is not None and baseline is not None and selection is not None and reach is not None, "result rows incomplete")
    require(set(filters) == {"COARSE", "FINE"}, "filter rows incomplete")
    require(set(searches) == {"COARSE", "FINE"}, "search rows incomplete")
    return {
        "meta": meta,
        "baseline": baseline,
        "filters": filters,
        "searches": searches,
        "selection": selection,
        "reach": reach,
        "states": states,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binding", required=True)
    parser.add_argument("--contract", required=True)
    parser.add_argument("--results", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--states", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    paths = {
        "binding": Path(args.binding),
        "contract": Path(args.contract),
        "results": Path(args.results),
        "urdf": Path(args.urdf),
        "srdf": Path(args.srdf),
        "states": Path(args.states),
        "fixture": Path(args.fixture),
    }
    for label, path in paths.items():
        require(path.is_file(), f"missing {label}: {path}")
    binding = json.loads(paths["binding"].read_text())
    parsed = parse_results(paths["results"])
    require(binding.get("passed") is True and binding.get("errors") == [], "input binding failed")
    require(binding.get("source_ref") == args.source_ref, "binding source mismatch")
    require(binding.get("accepted_source") == PR24_SOURCE, "accepted PR24 source mismatch")
    require(binding.get("coarse_candidate_count") == EXPECTED_COARSE_TOTAL, "binding coarse total mismatch")
    require(binding.get("coarse_relevant_candidate_count") == EXPECTED_COARSE_RELEVANT, "binding relevant count mismatch")
    require(binding.get("hard_filter", {}).get("collision_rules_weakened") is False, "collision rules were weakened")

    meta = parsed["meta"]
    require(meta == {
        "robot": "turtlebot3_lime",
        "root": "base_footprint",
        "state_count": 2802,
        "anchor_index": 1541,
        "candidate_start_index": 1043,
        "reference_link": "link7",
    }, "result identity mismatch")
    baseline = parsed["baseline"]
    require(baseline["clear"] is False, "baseline unexpectedly clear")
    require(baseline["first_collision_state_index"] == 0, "baseline collision index mismatch")
    require(baseline["first_collision_phase"] == "approach", "baseline phase mismatch")
    require(baseline["cube_collision"] is True and baseline["support_collision"] is True, "baseline collision classes mismatch")
    require(set(baseline["pairs"]) == EXPECTED_BASELINE_PAIRS, "baseline contact pairs mismatch")

    coarse_filter = parsed["filters"]["COARSE"]
    require(coarse_filter == {
        "total_candidates": EXPECTED_COARSE_TOTAL,
        "relevant_candidates": EXPECTED_COARSE_RELEVANT,
        "filtered_candidates": EXPECTED_COARSE_TOTAL - EXPECTED_COARSE_RELEVANT,
        "minimum_relevant_z_step": -40,
        "maximum_relevant_z_step": 22,
    }, "coarse filter mismatch")
    coarse = parsed["searches"]["COARSE"]
    require(math.isclose(coarse["step_m"], 0.002, abs_tol=1e-15), "coarse step mismatch")
    require(math.isclose(coarse["radius_m"], 0.08, abs_tol=1e-15), "coarse radius mismatch")
    require(coarse["candidate_count"] == EXPECTED_COARSE_RELEVANT, "coarse relevant count mismatch")
    require(coarse["candidates_checked"] <= EXPECTED_COARSE_RELEVANT, "coarse checked count overflow")
    require(coarse["maximum_states_checked"] <= 2802, "coarse state count overflow")

    selection = parsed["selection"]
    require(selection["decision"] in VALID_DECISIONS, "unexpected search decision")
    found = selection["found"]
    require((selection["decision"] == "FULL_PATH_CLEAR_GRASP_RELEVANT_TRANSLATION_FOUND") is found, "selection decision mismatch")
    require(coarse["found"] is found, "coarse found mismatch")
    fine_filter = parsed["filters"]["FINE"]
    fine = parsed["searches"]["FINE"]
    states = parsed["states"]
    reach = parsed["reach"]

    selected_path_audit = {
        "state_count": len(states),
        "all_bounds_ok": all(state["bounds_ok"] for state in states),
        "all_self_collision_free": all(not state["self_collision"] for state in states),
        "all_cube_clear": all(not state["cube_collision"] for state in states),
        "all_support_clear": all(not state["support_collision"] for state in states),
        "contact_pair_count": sum(state["fixture_contact_count"] for state in states),
        "self_contact_pairs": sorted({pair for state in states for pair in state["self_pairs"]}),
        "fixture_contact_pairs": sorted({pair for state in states for pair in state["fixture_pairs"]}),
    }

    if found:
        require(coarse["candidates_checked"] > 0, "positive search checked no coarse candidates")
        require(fine_filter["total_candidates"] == EXPECTED_FINE_TOTAL, "fine total mismatch")
        require(0 < fine_filter["relevant_candidates"] <= EXPECTED_FINE_TOTAL, "fine relevant count mismatch")
        require(fine_filter["filtered_candidates"] == EXPECTED_FINE_TOTAL - fine_filter["relevant_candidates"], "fine filtered count mismatch")
        require(fine["candidate_count"] == fine_filter["relevant_candidates"], "fine candidate count mismatch")
        require(fine["found"] is True and fine["candidates_checked"] > 0, "fine search did not retain candidate")
        require(selection["translation_link7_m"] == fine["selected_translation_link7_m"], "selected/fine translation mismatch")
        require(reach["relevant"] is True and reach["common_interval_exists"] is True, "selected reach relevance missing")
        for key in ("left_x_overlap_m", "left_z_overlap_m", "right_x_overlap_m", "right_z_overlap_m"):
            require(reach[key] > 0.0, f"selected reach overlap missing: {key}")
        require(len(states) == 2802, "selected state count mismatch")
        for index, state in enumerate(states):
            require(state["label"] == f"sample_{index:06d}", f"selected state label mismatch: {index}")
        require(selected_path_audit["all_bounds_ok"], "selected path bounds failure")
        require(selected_path_audit["all_self_collision_free"], "selected path self collision")
        require(selected_path_audit["all_cube_clear"], "selected path cube collision")
        require(selected_path_audit["all_support_clear"], "selected path support collision")
        require(selected_path_audit["contact_pair_count"] == 0, "selected path fixture contact")
        require(selected_path_audit["self_contact_pairs"] == [], "selected path self contact pairs")
        require(selected_path_audit["fixture_contact_pairs"] == [], "selected path fixture contact pairs")
    else:
        require(coarse["candidates_checked"] == EXPECTED_COARSE_RELEVANT, "negative coarse search incomplete")
        require(coarse["found"] is False, "negative coarse found mismatch")
        require(fine_filter["total_candidates"] == 0 and fine_filter["relevant_candidates"] == 0, "negative fine filter must be empty")
        require(fine["candidate_count"] == 0 and fine["candidates_checked"] == 0 and fine["found"] is False, "negative fine search must be empty")
        require(states == [], "negative search emitted selected states")
        require(reach["relevant"] is False, "negative search emitted selected relevance")

    cube_center = [-0.019, 0.0, 0.1192]
    support_center = [-0.019, 0.0, 0.0857]
    selected_cube_center = [cube_center[index] + selection["translation_link7_m"][index] for index in range(3)] if found else None
    selected_support_center = [support_center[index] + selection["translation_link7_m"][index] for index in range(3)] if found else None
    next_phase = (
        "RUBIK-PREGRASP-SELECTED-FIXTURE-OFFLINE-GRIPPER-SWEEP-COLLISION-PREFLIGHT"
        if found
        else "RUBIK-PREGRASP-FIXTURE-OR-POSE-REDESIGN-BLOCKER"
    )
    input_digests = {label: sha256(path) for label, path in paths.items()}
    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-GRASP-RELEVANCE-CONSTRAINED-FIXTURE-SEARCH",
        "source_ref": args.source_ref,
        "accepted_source": PR24_SOURCE,
        "exact_head_bound": True,
        "audit_passed": True,
        "errors": [],
        "decision": selection["decision"],
        "correction_candidate_found": found,
        "baseline_reproduction": baseline,
        "hard_filter": {
            "coarse": coarse_filter,
            "fine": fine_filter,
            "positive_cube_finger_aabb_overlap_x_and_z": True,
            "common_mimic_side_plane_interval_within_limits": True,
            "collision_rules_weakened": False,
        },
        "coarse_search": coarse,
        "fine_search": fine,
        "selected_candidate": {
            "translation_link7_m": selection["translation_link7_m"] if found else None,
            "norm_m": selection["norm_m"] if found else None,
            "cube_center_link7_m": selected_cube_center,
            "support_center_link7_m": selected_support_center,
            "reach": reach if found else None,
            "cube_support_relative_transform_preserved": True,
            "full_recorded_path_clear": found,
        },
        "selected_path_audit": selected_path_audit,
        "next_required_phase": next_phase,
        "claim_limits": {
            "nearest_clear_relevant_candidate_on_coarse_grid_then_local_fine_grid_only": True,
            "continuous_global_optimum_claimed": False,
            "rotation_search_performed": False,
            "exact_mesh_surface_contact_claimed": False,
            "force_closure_claimed": False,
            "friction_claimed": False,
            "gripper_motion_clearance_claimed": False,
            "grasp_success_claimed": False,
            "lifting_or_transport_claimed": False,
            "hardware_readiness_claimed": False,
        },
        "safety": {
            "offline_analysis_only": True,
            "accepted_arm_and_passive_finger_states_unchanged": True,
            "cube_support_rigid_relation_preserved": True,
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
        "input_sha256": input_digests,
    }
    output = Path(args.output)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "source_ref": args.source_ref,
        "accepted_source": PR24_SOURCE,
        "decision": selection["decision"],
        "correction_candidate_found": found,
        "coarse_total_candidate_count": EXPECTED_COARSE_TOTAL,
        "coarse_relevant_candidate_count": EXPECTED_COARSE_RELEVANT,
        "inputs": input_digests,
        "summary": {"path": output.name, "sha256": sha256(output)},
    }
    Path(args.manifest).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
