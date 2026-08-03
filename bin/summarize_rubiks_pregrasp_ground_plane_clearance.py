#!/usr/bin/env python3
"""Summarize exact-head MoveIt ground-plane clearance evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

JOINTS = [f"joint{i}" for i in range(1, 7)]
AUDITED_LINKS = [f"link{i}" for i in range(1, 8)]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValueError(message)


def parse_results(path: Path) -> dict[str, Any]:
    meta = None
    states = []
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 8, f"bad META row: {raw}")
            meta = {
                "robot_name":fields[1], "root_link":fields[2],
                "active_joint_order":fields[3].split(",") if fields[3] else [],
                "audited_ground_links":fields[4].split(",") if fields[4] else [],
                "missing_collision_geometry":fields[5].split(",") if fields[5] else [],
                "srdf_disabled_collision_pair_count":int(fields[6]),
                "ground_allowed_non_audited_link_count":int(fields[7]),
            }
        elif fields[0] == "STATE":
            require(len(fields) == 10, f"bad STATE row: {raw}")
            states.append({
                "label":fields[1], "bounds_ok":fields[2]=="true",
                "self_collision":fields[3]=="true", "ground_collision":fields[4]=="true",
                "ground_contact_pair_count":int(fields[5]),
                "ground_contact_pairs":fields[6].split(",") if fields[6] else [],
                "self_contact_pair_count":int(fields[7]),
                "self_contact_pairs":fields[8].split(",") if fields[8] else [],
                "ground_distance_m":None if fields[9]=="NA" else float(fields[9]),
            })
        else:
            raise ValueError(f"unknown result row: {raw}")
    require(meta is not None and states, "collision results incomplete")
    ground_pairs = sorted({p for s in states for p in s["ground_contact_pairs"]})
    self_pairs = sorted({p for s in states for p in s["self_contact_pairs"]})
    distances = [s["ground_distance_m"] for s in states if s["ground_distance_m"] is not None]
    return {
        **meta, "state_count":len(states),
        "all_bounds_ok":all(s["bounds_ok"] for s in states),
        "all_self_collision_free":all(not s["self_collision"] for s in states),
        "all_audited_links_ground_clear":all(not s["ground_collision"] for s in states),
        "maximum_ground_contact_pair_count":max(s["ground_contact_pair_count"] for s in states),
        "maximum_self_contact_pair_count":max(s["self_contact_pair_count"] for s in states),
        "ground_contact_pairs":ground_pairs, "self_contact_pairs":self_pairs,
        "finite_ground_distance_count":len(distances),
        "minimum_reported_ground_distance_m":min(distances) if distances else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation-summary", required=True)
    parser.add_argument("--ground-frame-trace", required=True)
    parser.add_argument("--frame-binding", required=True)
    parser.add_argument("--states-tsv", required=True)
    parser.add_argument("--collision-results", required=True)
    parser.add_argument("--simulation-urdf", required=True)
    parser.add_argument("--moveit-urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--base-runner", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    binding = json.loads(Path(args.frame_binding).read_text())
    require(binding.get("source_ref") == args.source_ref and binding.get("passed") is True, "frame binding failed")
    collision = parse_results(Path(args.collision_results))
    errors = []
    if collision["state_count"] != int(binding.get("state_count", 0)): errors.append("state count mismatch")
    if collision["robot_name"] != "turtlebot3_lime": errors.append("MoveIt robot name mismatch")
    if collision["root_link"] != "base_footprint": errors.append("MoveIt root link mismatch")
    if collision["active_joint_order"] != JOINTS: errors.append("joint order mismatch")
    if collision["audited_ground_links"] != AUDITED_LINKS: errors.append("audited link set mismatch")
    if collision["missing_collision_geometry"]: errors.append("audited collision geometry missing")
    if not collision["all_bounds_ok"]: errors.append("MoveIt bounds failure")
    if not collision["all_self_collision_free"] or collision["self_contact_pairs"]: errors.append("self collision detected")
    if not collision["all_audited_links_ground_clear"] or collision["ground_contact_pairs"]: errors.append("arm-ground collision detected")
    if collision["maximum_ground_contact_pair_count"] != 0: errors.append("ground contacts reported")
    if collision["maximum_self_contact_pair_count"] != 0: errors.append("self contacts reported")

    summary = {
        "schema_version":1, "phase":"RUBIK-PREGRASP-GROUND-PLANE-CLEARANCE-AUDIT",
        "writer_lease":"WL-RUBIK-PREGRASP-GROUND-PLANE-CLEARANCE-AUDIT-20260803-01",
        "source_ref":args.source_ref, "exact_head_bound":True, "passed":not errors, "errors":errors,
        "frame_binding":binding, "moveit_ground_plane_audit":collision,
        "verified_claim":"recorded_pregrasp_arm_chain_states_clear_of_exact_ground_plane_under_observed_root_transforms" if not errors else None,
        "claim_limits":{"audited_links_only":AUDITED_LINKS,"ground_model_only":"ground_plane",
            "populated_scene_clearance_claimed":False,"cube_or_support_clearance_claimed":False,
            "gripper_ground_clearance_claimed":False,"continuous_between_sample_clearance_claimed":False,
            "hardware_readiness_claimed":False},
        "safety":{"same_two_arm_goals_only":True,"new_command_path_added":False,"read_only_frame_observer":True,
            "offline_ground_object_only":True,"physical_hardware_used":False,"cube_spawned":False,
            "support_spawned":False,"fixture_spawned":False,"gripper_command_sent":False,
            "base_command_sent":False,"lift_command_sent":False,"ifra_attachment_used":False,
            "production_runtime_modified":False},
        "next_required_phase":"RUBIK-PREGRASP-GRIPPER-STATE-AND-GROUND-CLEARANCE-EVIDENCE",
    }
    output = Path(args.output)
    output.write_text(json.dumps(summary,indent=2,sort_keys=True)+"\n")
    inputs = {}
    for name in (args.simulation_summary,args.ground_frame_trace,args.frame_binding,args.states_tsv,
                 args.collision_results,args.simulation_urdf,args.moveit_urdf,args.srdf,args.world,args.base_runner):
        path = Path(name); inputs[path.name] = digest(path)
    manifest = {
        "schema_version":1,"phase":summary["phase"],"source_ref":args.source_ref,"exact_head_bound":True,
        "summary":output.name,"summary_sha256":digest(output),"inputs":inputs,
        "state_count":binding.get("state_count"),"moveit_checked_state_count":collision["state_count"],
        "audited_ground_links":AUDITED_LINKS,"ground_model_only":True,"all_states_passed":not errors,
        "new_command_path_added":False,"physical_hardware_used":False,"production_runtime_modified":False,
    }
    Path(args.manifest).write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n")
    print(json.dumps(summary,indent=2,sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
