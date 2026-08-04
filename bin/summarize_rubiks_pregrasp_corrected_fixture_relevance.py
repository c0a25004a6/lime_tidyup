#!/usr/bin/env python3
"""Validate and summarize the corrected fixture geometric relevance preflight."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

EXPECTED_ACCEPTED_SOURCE = "fa78493ca98b5c9701ccf2f6da5ae0ee3a2cbcee"
VALID_DECISIONS = {
    "CORRECTED_FIXTURE_GEOMETRICALLY_RELEVANT",
    "BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE",
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


def boolean(text: str) -> bool:
    require(text in {"true", "false"}, f"invalid boolean: {text}")
    return text == "true"


def parse_results(path: Path) -> dict[str, object]:
    meta: list[str] | None = None
    cube: list[float] | None = None
    meshes: dict[str, dict[str, object]] = {}
    positions: dict[str, dict[str, object]] = {}
    interval: dict[str, object] | None = None
    decision: dict[str, object] | None = None
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 6 and fields[1] == "1", "invalid META row")
            meta = fields[1:]
        elif fields[0] == "CUBE":
            require(len(fields) == 7, "invalid CUBE row")
            cube = [number(value) for value in fields[1:]]
        elif fields[0] == "MESH":
            require(len(fields) == 9, "invalid MESH row")
            require(fields[1] not in meshes, "duplicate MESH row")
            meshes[fields[1]] = {
                "shape_count": int(fields[2]),
                "local_aabb_center_m": [number(value) for value in fields[3:6]],
                "local_aabb_extents_m": [number(value) for value in fields[6:9]],
            }
        elif fields[0] == "POSITION":
            require(len(fields) == 23, "invalid POSITION row")
            require(fields[1] not in positions, "duplicate POSITION row")
            positions[fields[1]] = {
                "requested_m": number(fields[2]),
                "modeled_left_m": number(fields[3]),
                "modeled_right_m": number(fields[4]),
                "left_aabb_min_m": [number(value) for value in fields[5:8]],
                "left_aabb_max_m": [number(value) for value in fields[8:11]],
                "right_aabb_min_m": [number(value) for value in fields[11:14]],
                "right_aabb_max_m": [number(value) for value in fields[14:17]],
                "left_x_overlap_m": number(fields[17]),
                "left_z_overlap_m": number(fields[18]),
                "right_x_overlap_m": number(fields[19]),
                "right_z_overlap_m": number(fields[20]),
                "left_cube_side_inside_aabb": boolean(fields[21]),
                "right_cube_side_inside_aabb": boolean(fields[22]),
            }
        elif fields[0] == "INTERVAL":
            require(len(fields) == 11, "invalid INTERVAL row")
            interval = {
                "left_reachable_low_m": number(fields[1]),
                "left_reachable_high_m": number(fields[2]),
                "right_reachable_low_m": number(fields[3]),
                "right_reachable_high_m": number(fields[4]),
                "common_low_m": number(fields[5]),
                "common_high_m": number(fields[6]),
                "common_interval_exists": boolean(fields[7]),
                "representative_m": number(fields[8]),
                "representative_left_side_inside": boolean(fields[9]),
                "representative_right_side_inside": boolean(fields[10]),
            }
        elif fields[0] == "DECISION":
            require(len(fields) == 5, "invalid DECISION row")
            decision = {
                "value": fields[1],
                "positive_xz_overlap": boolean(fields[2]),
                "simultaneous_side_plane_reachability": boolean(fields[3]),
                "linearity_residual_m": number(fields[4]),
            }
        else:
            raise ValueError(f"unknown result row: {fields[0]}")
    require(meta is not None and cube is not None and interval is not None and decision is not None, "result rows are incomplete")
    require(set(meshes) == {"gripper_left_link", "gripper_right_link"}, "mesh rows mismatch")
    require(set(positions) == {"lower", "zero", "passive", "upper"}, "position rows mismatch")
    return {
        "meta": meta,
        "cube": cube,
        "meshes": meshes,
        "positions": positions,
        "interval": interval,
        "decision": decision,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binding", required=True)
    parser.add_argument("--mesh-provenance", required=True)
    parser.add_argument("--results", required=True)
    parser.add_argument("--contract", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    paths = {
        "binding": Path(args.binding),
        "mesh_provenance": Path(args.mesh_provenance),
        "results": Path(args.results),
        "contract": Path(args.contract),
        "urdf": Path(args.urdf),
        "srdf": Path(args.srdf),
    }
    for label, path in paths.items():
        require(path.is_file(), f"missing {label}: {path}")

    binding = json.loads(paths["binding"].read_text())
    provenance = json.loads(paths["mesh_provenance"].read_text())
    parsed = parse_results(paths["results"])
    require(binding.get("passed") is True and binding.get("errors") == [], "input binding failed")
    require(binding.get("source_ref") == args.source_ref, "binding exact-head mismatch")
    require(binding.get("accepted_source") == EXPECTED_ACCEPTED_SOURCE, "accepted PR23 source mismatch")
    require(binding.get("state_count") == 2802 and binding.get("anchor_index") == 1541, "binding state identity mismatch")
    require(binding.get("selected_translation_link7_m") == [0.0, 0.0, 0.04575], "binding translation mismatch")
    require(binding.get("corrected_cube_center_link7_m") == [-0.019, 0.0, 0.16495], "binding cube mismatch")
    require(provenance.get("passed") is True and provenance.get("errors") == [], "mesh provenance failed")
    require(provenance.get("source_ref") == args.source_ref, "mesh provenance exact-head mismatch")
    meshes = provenance.get("meshes", {})
    require(set(meshes) == {"gripper_left_link", "gripper_right_link"}, "mesh provenance coverage mismatch")
    for link, entry in meshes.items():
        require(entry.get("scale") == [0.001, 0.001, 0.001], f"mesh scale mismatch: {link}")
        require(isinstance(entry.get("sha256"), str) and len(entry["sha256"]) == 64, f"mesh digest missing: {link}")
        require(isinstance(entry.get("size_bytes"), int) and entry["size_bytes"] > 0, f"mesh size missing: {link}")
    source_repository = provenance.get("source_repository", {})
    require(source_repository.get("clean") is True, "external source clone is dirty")
    require(isinstance(source_repository.get("commit"), str) and len(source_repository["commit"]) == 40, "external source commit missing")

    require(parsed["meta"] == ["1", "turtlebot3_lime", "base_footprint", "sample_001541", "link7"], "result identity mismatch")
    require(parsed["cube"] == [0.057, 0.057, 0.057, -0.019, 0.0, 0.16495], "result cube contract mismatch")
    for link, entry in parsed["meshes"].items():
        require(entry["shape_count"] == 1, f"unexpected collision shape count: {link}")
        require(all(value > 0.0 for value in entry["local_aabb_extents_m"]), f"invalid mesh extents: {link}")
    positions = parsed["positions"]
    contract = binding["gripper_joint_contract"]
    expected_positions = {
        "lower": contract["lower_m"],
        "zero": 0.0,
        "passive": binding["anchor_state"]["left_gripper_position_m"],
        "upper": contract["upper_m"],
    }
    for label, expected in expected_positions.items():
        row = positions[label]
        require(math.isclose(row["requested_m"], expected, abs_tol=1e-15), f"requested position mismatch: {label}")
        require(math.isclose(row["modeled_left_m"], expected, abs_tol=1e-12), f"left mimic mismatch: {label}")
        require(math.isclose(row["modeled_right_m"], expected, abs_tol=1e-12), f"right mimic mismatch: {label}")

    decision = parsed["decision"]
    require(decision["value"] in VALID_DECISIONS, "unexpected relevance decision")
    require(decision["linearity_residual_m"] <= 1e-12, "gripper AABB motion linearity mismatch")
    interval = parsed["interval"]
    representative_reaches = (
        interval["common_interval_exists"]
        and interval["representative_left_side_inside"]
        and interval["representative_right_side_inside"]
    )
    require(decision["simultaneous_side_plane_reachability"] is representative_reaches, "reachability decision mismatch")
    positive_xz = all(
        positions["zero"][key] > 0.0
        for key in ("left_x_overlap_m", "left_z_overlap_m", "right_x_overlap_m", "right_z_overlap_m")
    )
    require(decision["positive_xz_overlap"] is positive_xz, "XZ-overlap decision mismatch")
    relevant = positive_xz and representative_reaches
    expected_decision = (
        "CORRECTED_FIXTURE_GEOMETRICALLY_RELEVANT"
        if relevant
        else "BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE"
    )
    require(decision["value"] == expected_decision, "relevance decision is inconsistent")

    next_phase = (
        "RUBIK-PREGRASP-CORRECTED-FIXTURE-OFFLINE-GRIPPER-SWEEP-COLLISION-PREFLIGHT"
        if relevant
        else "RUBIK-PREGRASP-GRASP-RELEVANCE-CONSTRAINED-FIXTURE-SEARCH"
    )
    input_digests = {label: sha256(path) for label, path in paths.items()}
    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-CORRECTED-FIXTURE-GEOMETRIC-RELEVANCE-PREFLIGHT",
        "source_ref": args.source_ref,
        "accepted_source": EXPECTED_ACCEPTED_SOURCE,
        "exact_head_bound": True,
        "audit_passed": True,
        "errors": [],
        "decision": decision["value"],
        "corrected_fixture_geometrically_relevant": relevant,
        "correction_translation_link7_m": binding["selected_translation_link7_m"],
        "cube_center_link7_m": binding["corrected_cube_center_link7_m"],
        "cube_size_m": parsed["cube"][:3],
        "mesh_provenance": provenance,
        "collision_mesh_aabb": parsed["meshes"],
        "evaluated_positions": positions,
        "simultaneous_side_plane_interval": interval,
        "positive_xz_contact_patch_overlap": positive_xz,
        "simultaneous_mimic_side_plane_reachability": representative_reaches,
        "next_required_phase": next_phase,
        "claim_limits": {
            "collision_mesh_aabb_envelopes_only": True,
            "exact_mesh_surface_contact_claimed": False,
            "contact_normal_claimed": False,
            "force_closure_claimed": False,
            "friction_claimed": False,
            "support_clearance_during_gripper_motion_claimed": False,
            "ground_clearance_during_gripper_motion_claimed": False,
            "grasp_success_claimed": False,
            "lifting_or_transport_claimed": False,
            "hardware_readiness_claimed": False,
        },
        "safety": {
            "offline_analysis_only": True,
            "accepted_arm_and_passive_finger_states_unchanged": True,
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
        "accepted_source": EXPECTED_ACCEPTED_SOURCE,
        "decision": decision["value"],
        "corrected_fixture_geometrically_relevant": relevant,
        "external_source_commit": source_repository["commit"],
        "mesh_sha256": {link: entry["sha256"] for link, entry in meshes.items()},
        "inputs": input_digests,
        "summary": {"path": output.name, "sha256": sha256(output)},
    }
    Path(args.manifest).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
