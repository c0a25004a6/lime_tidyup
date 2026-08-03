#!/usr/bin/env python3
"""Summarize fixed-world cube/support collision evidence for the recorded path."""
from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValueError(message)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binding", required=True)
    parser.add_argument("--results", required=True)
    parser.add_argument("--states", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    binding_path = Path(args.binding)
    result_path = Path(args.results)
    binding = json.loads(binding_path.read_text())
    require(binding.get("source_ref") == args.source_ref and binding.get("passed") is True, "input binding failed")

    meta = None
    rows = []
    for raw in result_path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 9, "invalid META row")
            meta = {
                "robot_name": fields[1], "root_link": fields[2],
                "state_count": int(fields[3]), "anchor_index": int(fields[4]),
                "candidate_start_index": int(fields[5]), "reference_link": fields[6],
                "cube_object": fields[7], "support_object": fields[8],
            }
        elif fields[0] == "STATE":
            require(len(fields) == 13, "invalid STATE row")
            rows.append({
                "label": fields[1], "phase": fields[2], "bounds_ok": fields[3] == "true",
                "self_collision": fields[4] == "true", "self_pair_count": int(fields[5]),
                "self_pairs": fields[6].split(",") if fields[6] else [],
                "cube_collision": fields[7] == "true", "cube_pair_count": int(fields[8]),
                "cube_pairs": fields[9].split(",") if fields[9] else [],
                "support_collision": fields[10] == "true", "support_pair_count": int(fields[11]),
                "support_pairs": fields[12].split(",") if fields[12] else [],
            })
        else:
            raise ValueError("unknown result row")
    require(meta is not None and rows, "fixture collision matrix incomplete")
    require(meta["state_count"] == len(rows) == int(binding["state_count"]), "state count mismatch")
    require(meta["anchor_index"] == int(binding["fixture_anchor_state_index"]), "anchor mismatch")
    require(meta["candidate_start_index"] == int(binding["candidate_hold_start_index"]), "candidate interval mismatch")
    require(meta["robot_name"] == "turtlebot3_lime" and meta["root_link"] == "base_footprint", "robot model mismatch")
    require(meta["reference_link"] == "link7", "fixture reference mismatch")
    require(meta["cube_object"] == "rubiks_cube" and meta["support_object"] == "rubiks_support", "fixture object mismatch")

    pair_counts = {"cube": Counter(), "support": Counter(), "self": Counter()}
    phase_counts = defaultdict(lambda: {"states": 0, "cube_collision_states": 0, "support_collision_states": 0})
    first = {"cube": None, "support": None, "self": None, "bounds": None}
    for index, row in enumerate(rows):
        require(row["label"] == f"sample_{index:06d}", f"state label mismatch at {index}")
        phase_counts[row["phase"]]["states"] += 1
        if not row["bounds_ok"] and first["bounds"] is None: first["bounds"] = index
        if row["self_collision"] and first["self"] is None: first["self"] = index
        if row["cube_collision"]:
            phase_counts[row["phase"]]["cube_collision_states"] += 1
            if first["cube"] is None: first["cube"] = index
        if row["support_collision"]:
            phase_counts[row["phase"]]["support_collision_states"] += 1
            if first["support"] is None: first["support"] = index
        pair_counts["self"].update(row["self_pairs"])
        pair_counts["cube"].update(row["cube_pairs"])
        pair_counts["support"].update(row["support_pairs"])

    bounds_ok = all(row["bounds_ok"] for row in rows)
    self_clear = all(not row["self_collision"] and row["self_pair_count"] == 0 for row in rows)
    cube_clear = all(not row["cube_collision"] and row["cube_pair_count"] == 0 for row in rows)
    support_clear = all(not row["support_collision"] and row["support_pair_count"] == 0 for row in rows)
    fixture_ready = bounds_ok and self_clear and cube_clear and support_clear
    decision = "FIXTURE_PLACEMENT_CLEAR" if fixture_ready else "BLOCKED_ROBOT_FIXTURE_COLLISION"
    next_phase = (
        "RUBIK-PREGRASP-FIXTURE-SCENE-SIMULATION-EVIDENCE"
        if fixture_ready else "RUBIK-PREGRASP-FIXTURE-RELATIVE-ALIGNMENT-CORRECTION-PREFLIGHT"
    )

    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-FIXTURE-PLACEMENT-OFFLINE-COLLISION-PREFLIGHT",
        "writer_lease": "WL-RUBIK-PREGRASP-FIXTURE-OFFLINE-COLLISION-20260804-01",
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "audit_passed": True,
        "errors": [],
        "decision": decision,
        "fixture_phase_ready": fixture_ready,
        "input_binding": binding,
        "moveit_audit": {
            **meta,
            "all_bounds_ok": bounds_ok,
            "all_self_collision_free": self_clear,
            "all_cube_clear": cube_clear,
            "all_support_clear": support_clear,
            "first_failure_state_index": first,
            "phase_counts": dict(phase_counts),
            "self_contact_pair_counts": dict(sorted(pair_counts["self"].items())),
            "cube_contact_pair_counts": dict(sorted(pair_counts["cube"].items())),
            "support_contact_pair_counts": dict(sorted(pair_counts["support"].items())),
            "cube_collision_state_count": sum(row["cube_collision"] for row in rows),
            "support_collision_state_count": sum(row["support_collision"] for row in rows),
            "self_collision_state_count": sum(row["self_collision"] for row in rows),
            "bounds_failure_state_count": sum(not row["bounds_ok"] for row in rows),
        },
        "claim_limits": {
            "fixed_fixture_world_pose_only": True,
            "fixture_anchored_at_selected_pose_hold": True,
            "intended_cube_support_surface_contact_only": True,
            "distance_or_margin_claimed": False,
            "gazebo_fixture_stability_claimed": False,
            "grasp_or_contact_quality_claimed": False,
            "actuation_or_hardware_readiness_claimed": False,
        },
        "safety": {
            "offline_analysis_only": True,
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
        "next_required_phase": next_phase,
    }
    output = Path(args.output)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    inputs = [Path(args.binding), Path(args.results), Path(args.states), Path(args.fixture), Path(args.urdf), Path(args.srdf)]
    manifest = {
        "schema_version": 1, "phase": summary["phase"], "source_ref": args.source_ref,
        "exact_head_bound": True, "summary": output.name, "summary_sha256": sha(output),
        "inputs": {path.name: sha(path) for path in inputs}, "state_count": len(rows),
        "fixture_phase_ready": fixture_ready, "decision": decision,
        "offline_analysis_only": True, "physical_hardware_used": False,
        "production_runtime_modified": False,
    }
    Path(args.manifest).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
