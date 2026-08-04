#!/usr/bin/env python3
"""Bind accepted PR22 fixture-collision evidence to an offline translation search."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

PR22_SOURCE = "b0b9dcf507e883a409d322e43120407d03aa0c19"
PR22_FILES = {
    "rubiks_pregrasp_fixture_offline_summary.json": "45bd489513e15a8d3bdbd241bf6b0dd0a81ea78d5fb888c46e605024e1191e25",
    "rubiks_pregrasp_fixture_offline_evidence.manifest.json": "2e566f8e6419aaa53d315c82501c3c5c838873388102fcb3bd005ec372f93668",
    "rubiks_pregrasp_fixture_input_binding.json": "fc18b2d5243ce15960bb90eb2c5d7c96c95fa46add2bde50638a4ec2df3ac966",
    "rubiks_pregrasp_fixture_contract.tsv": "aa0c7014fc6543ceb7effea230f3d3767b13fb527a7dbb278910f8b262732b80",
    "rubiks_pregrasp_fixture_collision_results.tsv": "198e27f7731ac5d31eddb2a0b327ca19402956770ad1273634d8afaef2eb46a0",
    "pregrasp/rubiks_pregrasp_gripper_ground_states.tsv": "baed5397ad5c40835778fefc3e302ad1716cc8af278fe856b77fbb79fe106cf3",
    "turtlebot3_lime_fixture_current.urdf": "58c77314a7af7f743064a1f382bc7741edb806c8f2563180ff76580ccad126b8",
    "turtlebot3_lime_fixture_current.srdf": "fd3778f7c8156687ab8301f01aa9dc915c40179fa0d6f6bd1ceefd77efc71d1f",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_files(root: Path) -> dict[str, str]:
    verified: dict[str, str] = {}
    for relative, expected in PR22_FILES.items():
        path = root / relative
        require(path.is_file(), f"accepted PR22 file missing: {relative}")
        actual = sha256(path)
        require(actual == expected, f"accepted PR22 digest mismatch: {relative}")
        verified[relative] = actual
    return verified


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--accepted-dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    root = Path(args.accepted_dir)
    verified = verify_files(root)
    summary = json.loads((root / "rubiks_pregrasp_fixture_offline_summary.json").read_text())
    manifest = json.loads((root / "rubiks_pregrasp_fixture_offline_evidence.manifest.json").read_text())
    binding = json.loads((root / "rubiks_pregrasp_fixture_input_binding.json").read_text())

    require(summary.get("source_ref") == PR22_SOURCE, "accepted PR22 summary source mismatch")
    require(summary.get("audit_passed") is True and summary.get("errors") == [], "accepted PR22 audit failed")
    require(summary.get("decision") == "BLOCKED_ROBOT_FIXTURE_COLLISION", "accepted PR22 decision mismatch")
    require(summary.get("fixture_phase_ready") is False, "accepted PR22 unexpectedly marked ready")
    require(summary.get("next_required_phase") == "RUBIK-PREGRASP-FIXTURE-RELATIVE-ALIGNMENT-CORRECTION-PREFLIGHT", "accepted PR22 next phase mismatch")
    audit = summary.get("moveit_audit", {})
    require(audit.get("state_count") == 2802, "accepted state count mismatch")
    require(audit.get("all_bounds_ok") is True, "accepted bounds evidence failed")
    require(audit.get("all_self_collision_free") is True, "accepted self-collision evidence failed")
    require(audit.get("cube_collision_state_count") == 2802, "accepted cube blocker count mismatch")
    require(audit.get("support_collision_state_count") == 2802, "accepted support blocker count mismatch")
    require(audit.get("first_failure_state_index", {}).get("cube") == 0, "accepted cube first failure mismatch")
    require(audit.get("first_failure_state_index", {}).get("support") == 0, "accepted support first failure mismatch")

    require(manifest.get("source_ref") == PR22_SOURCE, "accepted manifest source mismatch")
    require(manifest.get("decision") == "BLOCKED_ROBOT_FIXTURE_COLLISION", "accepted manifest decision mismatch")
    require(manifest.get("state_count") == 2802, "accepted manifest state count mismatch")
    require(manifest.get("offline_analysis_only") is True, "accepted manifest offline boundary missing")

    require(binding.get("source_ref") == PR22_SOURCE, "accepted input binding source mismatch")
    require(binding.get("passed") is True and binding.get("errors") == [], "accepted input binding failed")
    require(binding.get("state_count") == 2802, "accepted input binding state count mismatch")
    require(binding.get("fixture_anchor_state_index") == 1541, "accepted fixture anchor mismatch")
    require(binding.get("candidate_hold_start_index") == 1043, "accepted candidate start mismatch")
    require(binding.get("fixture_reference_link") == "link7", "accepted reference link mismatch")
    require(binding.get("intended_fixture_contact", {}).get("robot_fixture_contact_allowed") is False, "robot-fixture contact policy mismatch")

    payload = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-FIXTURE-RELATIVE-ALIGNMENT-SEARCH-INPUT-BINDING",
        "source_ref": args.source_ref,
        "accepted_source": PR22_SOURCE,
        "passed": True,
        "errors": [],
        "verified_files": verified,
        "state_count": 2802,
        "fixture_anchor_state_index": 1541,
        "candidate_hold_start_index": 1043,
        "fixture_reference_link": "link7",
        "baseline_decision": "BLOCKED_ROBOT_FIXTURE_COLLISION",
        "baseline_contact_state_counts": {"cube": 2802, "support": 2802},
        "search_contract": {
            "transformation": "rigid translation only in selected-pose link7 frame",
            "cube_support_relative_transform_preserved": True,
            "objects_fixed_in_world_during_full_path_validation": True,
            "candidate_order": "squared Euclidean norm, then abs(z), abs(y), abs(x), then signed z, y, x",
            "coarse_step_m": 0.002,
            "coarse_radius_m": 0.08,
            "coarse_domain": "closed Euclidean sphere",
            "coarse_candidate_must_clear_all_2802_states": True,
            "fine_step_m": 0.00025,
            "fine_local_radius_m": 0.002,
            "fine_domain": "local cube around first coarse full-path-clear candidate",
            "fine_candidate_must_clear_all_2802_states": True,
            "continuous_global_optimum_claimed": False,
            "grasp_relevance_claimed": False,
        },
        "safety": {
            "offline_binding_only": True,
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
    Path(args.output).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
