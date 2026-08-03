#!/usr/bin/env python3
"""Summarize measured passive finger-state floor-clearance evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

ARM = [f"joint{i}" for i in range(1, 7)]
FINGER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]
FINGER_LINKS = ["gripper_left_link", "gripper_right_link"]


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValueError(message)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_results(path: Path) -> dict[str, Any]:
    meta = None
    rows = []
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 10, "invalid META row")
            meta = {
                "robot_name": fields[1],
                "root_link": fields[2],
                "arm_joint_order": fields[3].split(","),
                "finger_joint_order": fields[4].split(","),
                "audited_links": fields[5].split(","),
                "missing_geometry": fields[6].split(",") if fields[6] else [],
                "srdf_disabled_pair_count": int(fields[7]),
                "allowed_non_audited_link_count": int(fields[8]),
                "maximum_allowed_model_mimic_residual_m": float(fields[9]),
            }
        elif fields[0] == "STATE":
            require(len(fields) == 14, "invalid STATE row")
            rows.append({
                "label": fields[1],
                "bounds_ok": fields[2] == "true",
                "model_mimic_match": fields[3] == "true",
                "model_mimic_residual_m": float(fields[4]),
                "left_m": float(fields[5]),
                "right_m": float(fields[6]),
                "modeled_right_m": float(fields[7]),
                "self_collision": fields[8] == "true",
                "floor_collision": fields[9] == "true",
                "floor_pair_count": int(fields[10]),
                "floor_pairs": fields[11].split(",") if fields[11] else [],
                "self_pair_count": int(fields[12]),
                "self_pairs": fields[13].split(",") if fields[13] else [],
            })
        else:
            raise ValueError("unknown result row")
    require(meta is not None and rows, "result matrix is incomplete")
    return {
        **meta,
        "state_count": len(rows),
        "all_bounds_ok": all(row["bounds_ok"] for row in rows),
        "all_model_mimic_matches": all(row["model_mimic_match"] for row in rows),
        "maximum_model_mimic_residual_m": max(row["model_mimic_residual_m"] for row in rows),
        "all_self_collision_free": all(not row["self_collision"] for row in rows),
        "all_audited_links_floor_clear": all(not row["floor_collision"] for row in rows),
        "maximum_floor_pair_count": max(row["floor_pair_count"] for row in rows),
        "maximum_self_pair_count": max(row["self_pair_count"] for row in rows),
        "floor_pairs": sorted({pair for row in rows for pair in row["floor_pairs"]}),
        "self_pairs": sorted({pair for row in rows for pair in row["self_pairs"]}),
        "left_min_m": min(row["left_m"] for row in rows),
        "left_max_m": max(row["left_m"] for row in rows),
        "right_min_m": min(row["right_m"] for row in rows),
        "right_max_m": max(row["right_m"] for row in rows),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation-summary", required=True)
    parser.add_argument("--ground-summary", required=True)
    parser.add_argument("--ground-manifest", required=True)
    parser.add_argument("--state-trace", required=True)
    parser.add_argument("--state-binding", required=True)
    parser.add_argument("--states-tsv", required=True)
    parser.add_argument("--collision-results", required=True)
    parser.add_argument("--simulation-urdf", required=True)
    parser.add_argument("--moveit-urdf", required=True)
    parser.add_argument("--srdf", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--controllers-before", required=True)
    parser.add_argument("--controllers-after", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    paths = {name: Path(getattr(args, name)) for name in (
        "simulation_summary", "ground_summary", "ground_manifest", "state_trace",
        "state_binding", "states_tsv", "collision_results", "simulation_urdf",
        "moveit_urdf", "srdf", "world", "controllers_before", "controllers_after",
    )}
    simulation = json.loads(paths["simulation_summary"].read_text())
    ground = json.loads(paths["ground_summary"].read_text())
    ground_manifest = json.loads(paths["ground_manifest"].read_text())
    trace = json.loads(paths["state_trace"].read_text())
    binding = json.loads(paths["state_binding"].read_text())
    result = parse_results(paths["collision_results"])

    for label, evidence in (("simulation", simulation), ("ground", ground),
                            ("ground_manifest", ground_manifest), ("trace", trace),
                            ("binding", binding)):
        require(evidence.get("source_ref") == args.source_ref, f"{label} source mismatch")
    require(simulation.get("passed") is True, "simulation failed")
    require(ground.get("passed") is True, "upstream floor audit failed")
    require(ground_manifest.get("all_states_passed") is True, "upstream manifest failed")
    require(trace.get("passed") is True and trace.get("errors") == [], "passive state trace failed")
    require(binding.get("passed") is True and binding.get("errors") == [], "passive state binding failed")

    for label in ("controllers_before", "controllers_after"):
        text = paths[label].read_text()
        require("arm_controller" in text and "active" in text, f"arm controller missing in {label}")
        require("joint_state_broadcaster" in text and "active" in text, f"state broadcaster missing in {label}")
        require("gripper_controller" not in text, f"finger actuator appeared in {label}")
        require(not any(token in text for token in ("wheel", "diff_drive", "base_controller")), f"base actuator appeared in {label}")

    errors = []
    checks = (
        (result["state_count"] == int(binding.get("state_count", 0)), "state count mismatch"),
        (result["robot_name"] == "turtlebot3_lime", "robot name mismatch"),
        (result["root_link"] == "base_footprint", "root link mismatch"),
        (result["arm_joint_order"] == ARM, "arm order mismatch"),
        (result["finger_joint_order"] == FINGER_JOINTS, "finger order mismatch"),
        (result["audited_links"] == FINGER_LINKS, "audited link set mismatch"),
        (result["missing_geometry"] == [], "finger geometry missing"),
        (result["all_bounds_ok"], "bounds failure"),
        (result["all_model_mimic_matches"], "model mimic mismatch"),
        (result["maximum_model_mimic_residual_m"] <= result["maximum_allowed_model_mimic_residual_m"], "model mimic residual exceeded"),
        (result["all_self_collision_free"] and result["self_pairs"] == [], "self collision detected"),
        (result["all_audited_links_floor_clear"] and result["floor_pairs"] == [], "finger-floor collision detected"),
        (result["maximum_floor_pair_count"] == 0, "floor contacts reported"),
        (result["maximum_self_pair_count"] == 0, "self contacts reported"),
    )
    errors.extend(message for ok, message in checks if not ok)

    summary = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-GRIPPER-STATE-AND-GROUND-CLEARANCE-EVIDENCE",
        "writer_lease": "WL-RUBIK-PREGRASP-GRIPPER-STATE-GROUND-CLEARANCE-20260803-01",
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "passed": not errors,
        "errors": errors,
        "upstream_arm_floor_audit_passed": True,
        "passive_state_binding": binding,
        "moveit_passive_finger_floor_audit": result,
        "verified_claim": (
            "measured_sampled_passive_finger_states_within_limits_mimic_consistent_self_collision_free_and_floor_clear"
            if not errors else None
        ),
        "claim_limits": {
            "sampled_states_only": True,
            "audited_links_only": FINGER_LINKS,
            "continuous_between_samples_claimed": False,
            "object_clearance_claimed": False,
            "contact_or_force_claimed": False,
            "actuation_capability_claimed": False,
            "hardware_readiness_claimed": False,
        },
        "safety": {
            "same_two_arm_goals_only": True,
            "passive_observer_only": True,
            "offline_floor_object_only": True,
            "finger_actuator_loaded": False,
            "finger_actuation_performed": False,
            "new_actuation_path_added": False,
            "object_spawned": False,
            "base_actuation_performed": False,
            "lift_performed": False,
            "attachment_used": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
        },
        "next_required_phase": "RUBIK-PREGRASP-FIXTURE-PLACEMENT-OFFLINE-COLLISION-PREFLIGHT",
    }
    output = Path(args.output)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    manifest = {
        "schema_version": 1,
        "phase": summary["phase"],
        "source_ref": args.source_ref,
        "exact_head_bound": True,
        "summary": output.name,
        "summary_sha256": sha(output),
        "inputs": {path.name: sha(path) for path in paths.values()},
        "state_count": binding.get("state_count"),
        "moveit_checked_state_count": result["state_count"],
        "measured_joint_order": FINGER_JOINTS,
        "audited_links": FINGER_LINKS,
        "all_states_passed": not errors,
        "passive_observation_only": True,
        "physical_hardware_used": False,
        "production_runtime_modified": False,
    }
    Path(args.manifest).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
