#!/usr/bin/env python3
"""Bind accepted PR23 correction evidence to a command-free geometry audit."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PR23_SOURCE = "fa78493ca98b5c9701ccf2f6da5ae0ee3a2cbcee"
PR23_FILES = {
    "rubiks_pregrasp_fixture_alignment_search_summary.json": "8a259ba59232519bb47f72ce4512bae6b44f05d4da2bfd40e1c7a97d88ed1ffc",
    "rubiks_pregrasp_fixture_alignment_search_evidence.manifest.json": "4218ea47f2be70c2f1435701b155a67750b066598707efaea16b8113bedc4d93",
    "rubiks_pregrasp_fixture_alignment_search_results.tsv": "69eee3b5965c3d47ac6a01e1d5985f8e2197736ef0868bad147be44ab0a75c12",
    "rubiks_pregrasp_fixture_alignment_search_input_binding.json": "5aea00c7dfa5a47aa7d1b3ee15b1de4909b86884eb76bd13a8c6a7f31c07fb5e",
    "accepted/pregrasp/rubiks_pregrasp_gripper_ground_states.tsv": "baed5397ad5c40835778fefc3e302ad1716cc8af278fe856b77fbb79fe106cf3",
    "accepted/rubiks_pregrasp_fixture_contract.tsv": "aa0c7014fc6543ceb7effea230f3d3767b13fb527a7dbb278910f8b262732b80",
    "accepted/turtlebot3_lime_fixture_current.urdf": "58c77314a7af7f743064a1f382bc7741edb806c8f2563180ff76580ccad126b8",
    "accepted/turtlebot3_lime_fixture_current.srdf": "fd3778f7c8156687ab8301f01aa9dc915c40179fa0d6f6bd1ceefd77efc71d1f",
}

EXPECTED_TRANSLATION = [0.0, 0.0, 0.04575]
EXPECTED_CUBE_CENTER = [-0.019, 0.0, 0.16495]
EXPECTED_SUPPORT_CENTER = [-0.019, 0.0, 0.13145]
ANCHOR_INDEX = 1541
STATE_COUNT = 2802


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def close_vector(actual: list[float], expected: list[float], tolerance: float = 1e-12) -> bool:
    return len(actual) == len(expected) and all(
        math.isfinite(value) and abs(value - target) <= tolerance
        for value, target in zip(actual, expected)
    )


def verify_files(root: Path) -> dict[str, str]:
    verified: dict[str, str] = {}
    for relative, expected in PR23_FILES.items():
        path = root / relative
        require(path.is_file(), f"accepted PR23 file missing: {relative}")
        actual = sha256(path)
        require(actual == expected, f"accepted PR23 digest mismatch: {relative}")
        verified[relative] = actual
    return verified


def parse_joint_contract(urdf_path: Path) -> dict[str, object]:
    root = ET.parse(urdf_path).getroot()
    joints = {str(item.get("name")): item for item in root.findall("joint")}
    left = joints.get("gripper_left_joint")
    right = joints.get("gripper_right_joint")
    require(left is not None and right is not None, "gripper joints missing")
    require(left.get("type") == "prismatic" and right.get("type") == "prismatic", "gripper joints are not prismatic")

    def axis(joint: ET.Element) -> list[float]:
        element = joint.find("axis")
        require(element is not None and element.get("xyz"), "gripper axis missing")
        values = [float(value) for value in str(element.get("xyz")).split()]
        require(len(values) == 3 and all(math.isfinite(value) for value in values), "invalid gripper axis")
        return values

    left_axis = axis(left)
    right_axis = axis(right)
    require(close_vector(left_axis, [0.0, 1.0, 0.0]), "left gripper axis mismatch")
    require(close_vector(right_axis, [0.0, -1.0, 0.0]), "right gripper axis mismatch")
    limit = left.find("limit")
    require(limit is not None and limit.get("lower") and limit.get("upper"), "left gripper limits missing")
    lower = float(str(limit.get("lower")))
    upper = float(str(limit.get("upper")))
    require(math.isfinite(lower) and math.isfinite(upper) and lower < upper, "invalid gripper limits")
    right_limit = right.find("limit")
    require(right_limit is not None, "right gripper limits missing")
    require(abs(float(str(right_limit.get("lower"))) - lower) <= 1e-15, "right lower limit mismatch")
    require(abs(float(str(right_limit.get("upper"))) - upper) <= 1e-15, "right upper limit mismatch")
    mimic = right.find("mimic")
    require(mimic is not None, "right mimic declaration missing")
    require(mimic.get("joint") == "gripper_left_joint", "right mimic source mismatch")
    require(abs(float(mimic.get("multiplier", "1")) - 1.0) <= 1e-15, "right mimic multiplier mismatch")
    return {
        "left_axis": left_axis,
        "right_axis": right_axis,
        "lower_m": lower,
        "upper_m": upper,
        "mimic_source": "gripper_left_joint",
        "mimic_multiplier": 1.0,
    }


def read_anchor_state(states_path: Path) -> dict[str, object]:
    lines = [line for line in states_path.read_text().splitlines() if line]
    require(len(lines) == STATE_COUNT, "accepted state count mismatch")
    fields = lines[ANCHOR_INDEX].split("\t")
    require(len(fields) == 16 and fields[0] == f"sample_{ANCHOR_INDEX:06d}", "anchor state row mismatch")
    values = [float(value) for value in fields[1:]]
    require(all(math.isfinite(value) for value in values), "anchor state contains nonfinite values")
    return {
        "label": fields[0],
        "root_pose_xyz_xyzw": values[:7],
        "arm_positions_rad": values[7:13],
        "left_gripper_position_m": values[13],
        "right_gripper_position_m": values[14],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--accepted-dir", required=True)
    parser.add_argument("--contract-tsv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    root = Path(args.accepted_dir)
    verified = verify_files(root)
    summary = json.loads((root / "rubiks_pregrasp_fixture_alignment_search_summary.json").read_text())
    manifest = json.loads((root / "rubiks_pregrasp_fixture_alignment_search_evidence.manifest.json").read_text())
    require(summary.get("source_ref") == PR23_SOURCE, "accepted PR23 summary source mismatch")
    require(summary.get("audit_passed") is True and summary.get("errors") == [], "accepted PR23 audit failed")
    require(summary.get("decision") == "FULL_PATH_CLEAR_TRANSLATION_FOUND", "accepted correction decision mismatch")
    require(summary.get("correction_candidate_found") is True, "accepted correction candidate missing")
    require(summary.get("correction_phase_ready") is True, "accepted correction phase not ready")
    require(summary.get("next_required_phase") == "RUBIK-PREGRASP-CORRECTED-FIXTURE-GEOMETRIC-RELEVANCE-PREFLIGHT", "accepted next phase mismatch")
    selected = summary.get("selected_candidate", {})
    require(close_vector(selected.get("translation_link7_m", []), EXPECTED_TRANSLATION), "selected translation mismatch")
    require(close_vector(selected.get("cube_offset_from_link7_m", []), EXPECTED_CUBE_CENTER), "corrected cube center mismatch")
    require(close_vector(selected.get("support_offset_from_link7_m", []), EXPECTED_SUPPORT_CENTER), "corrected support center mismatch")
    require(selected.get("full_recorded_path_clear") is True, "selected candidate path not clear")
    require(selected.get("cube_support_relative_transform_preserved") is True, "fixture relative transform changed")
    require(selected.get("translation_within_cube_half_extent_box") is False, "expected relevance concern is absent")
    require(selected.get("translation_norm_less_than_cube_edge") is True, "selected translation exceeds cube edge")
    require(manifest.get("source_ref") == PR23_SOURCE and manifest.get("decision") == "FULL_PATH_CLEAR_TRANSLATION_FOUND", "accepted PR23 manifest mismatch")
    require(manifest.get("state_count") == STATE_COUNT, "accepted PR23 manifest state count mismatch")

    urdf_path = root / "accepted/turtlebot3_lime_fixture_current.urdf"
    states_path = root / "accepted/pregrasp/rubiks_pregrasp_gripper_ground_states.tsv"
    joint = parse_joint_contract(urdf_path)
    anchor = read_anchor_state(states_path)
    passive = float(anchor["left_gripper_position_m"])
    require(joint["lower_m"] <= passive <= joint["upper_m"], "passive gripper position outside limits")

    contract_path = Path(args.contract_tsv)
    with contract_path.open("w", encoding="utf-8") as stream:
        stream.write(f"META\t1\t{STATE_COUNT}\t{ANCHOR_INDEX}\tlink7\n")
        stream.write("CUBE\t0.057\t0.057\t0.057\t" + "\t".join(f"{value:.17g}" for value in EXPECTED_CUBE_CENTER) + "\n")
        stream.write("SUPPORT\t0.075\t0.040\t0.010\t" + "\t".join(f"{value:.17g}" for value in EXPECTED_SUPPORT_CENTER) + "\n")
        stream.write("TRANSLATION\t" + "\t".join(f"{value:.17g}" for value in EXPECTED_TRANSLATION) + "\n")
        stream.write(f"GRIPPER\t{joint['lower_m']:.17g}\t0\t{passive:.17g}\t{joint['upper_m']:.17g}\n")
        stream.write("ANCHOR\t" + str(anchor["label"]) + "\t" + "\t".join(f"{value:.17g}" for value in anchor["arm_positions_rad"]) + "\n")

    payload = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-CORRECTED-FIXTURE-GEOMETRIC-RELEVANCE-INPUT-BINDING",
        "source_ref": args.source_ref,
        "accepted_source": PR23_SOURCE,
        "passed": True,
        "errors": [],
        "verified_files": verified,
        "state_count": STATE_COUNT,
        "anchor_index": ANCHOR_INDEX,
        "anchor_state": anchor,
        "gripper_joint_contract": joint,
        "selected_translation_link7_m": EXPECTED_TRANSLATION,
        "corrected_cube_center_link7_m": EXPECTED_CUBE_CENTER,
        "corrected_support_center_link7_m": EXPECTED_SUPPORT_CENTER,
        "contract_tsv": contract_path.name,
        "audit_contract": {
            "collision_mesh_aabb_envelopes_only": True,
            "gripper_positions_evaluated": [joint["lower_m"], 0.0, passive, joint["upper_m"]],
            "necessary_xz_contact_patch_overlap_required": True,
            "simultaneous_mimic_side_plane_reachability_required": True,
            "mesh_contact_or_force_claimed": False,
            "grasp_success_claimed": False,
        },
        "safety": {
            "offline_binding_only": True,
            "accepted_states_unchanged": True,
            "joint_state_changed_in_runtime": False,
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
