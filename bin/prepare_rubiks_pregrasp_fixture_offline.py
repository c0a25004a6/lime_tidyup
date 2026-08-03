#!/usr/bin/env python3
"""Bind accepted supported fixture geometry to exact passive pregrasp states."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PR7_SOURCE = "a759a6efa2bd945aeff17697da22a19d8b41bb12"
PR21_SOURCE = "2c388efe00f5e4ce76e11fe78f8574c9e11be752"
PR7_FILES = {
    "rubiks_dynamic_cube_supported_057.sdf": "e67b672bf5d9c0c9030a93a0bdef35474a8541d7ceca2fcf49ad95921fc09f6b",
    "rubiks_cube_support_surface.sdf": "aa2722085c5180af7e6cb54d1bad964cb203f6606e1ac9dd8bed1a2aee257147",
    "rubiks_dynamic_supported_hold_telemetry.json": "11b78f5ad0e32e0f33cf1d56b1e3e5edbb0d4db08b19546f9856a009ee606b53",
    "rubiks_dynamic_supported_hold_summary.json": "8c3ffff34788a677a2dfa10718c19843703d2fad9fc079a9b46e9bd679e0206f",
    "rubiks_dynamic_supported_hold_evidence.manifest.json": "a14dd262a66af51f73ff357344915cb0be50193043744269a26d37d30abe0622",
}
PR21_FILES = {
    "rubiks_pregrasp_simulation_state_summary.json": "3447a9a5a5a9aabb09d25917034fc4a930a58f51aa562fdc322ace2bf8578d71",
    "rubiks_pregrasp_gripper_ground_states.tsv": "baed5397ad5c40835778fefc3e302ad1716cc8af278fe856b77fbb79fe106cf3",
    "turtlebot3_lime_pregrasp_search.urdf": "58c77314a7af7f743064a1f382bc7741edb806c8f2563180ff76580ccad126b8",
    "turtlebot3_lime.srdf": "fd3778f7c8156687ab8301f01aa9dc915c40179fa0d6f6bd1ceefd77efc71d1f",
    "rubiks_pregrasp_gripper_ground_clearance_summary.json": "a8c6a663fef295ab19e0b8c12f37195e63ee424b52bcb055e56329632e5520e5",
    "rubiks_pregrasp_gripper_ground_clearance_evidence.manifest.json": "18dea8c85b935e53883123ff5111ad31619a61b50de84a055930cac3a99c0ca1",
}
ARM = [f"joint{i}" for i in range(1, 7)]
FINGERS = ["gripper_left_joint", "gripper_right_joint"]


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValueError(message)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_files(root: Path, expected: dict[str, str]) -> None:
    for name, digest in expected.items():
        path = root / name
        require(path.is_file(), f"missing accepted evidence file: {name}")
        require(sha(path) == digest, f"accepted evidence digest mismatch: {name}")


def vector(text: str | None, count: int) -> list[float]:
    values = [float(value) for value in (text or "").split()]
    require(len(values) == count and all(math.isfinite(value) for value in values), "invalid vector")
    return values


def sdf_box(path: Path, model_name: str, link_name: str, collision_name: str, static: bool) -> dict:
    root = ET.parse(path).getroot()
    require(root.tag == "sdf" and root.get("version") == "1.7", f"unexpected SDF: {path.name}")
    models = root.findall("model")
    require(len(models) == 1 and models[0].get("name") == model_name, f"model mismatch: {path.name}")
    model = models[0]
    require((model.findtext("static") or "false").strip().lower() == str(static).lower(), f"static flag mismatch: {path.name}")
    links = model.findall("link")
    require(len(links) == 1 and links[0].get("name") == link_name, f"link mismatch: {path.name}")
    collisions = links[0].findall("collision")
    require(len(collisions) == 1 and collisions[0].get("name") == collision_name, f"collision mismatch: {path.name}")
    size = vector(collisions[0].findtext("geometry/box/size"), 3)
    return {
        "model_name": model_name,
        "link_name": link_name,
        "collision_name": collision_name,
        "static": static,
        "size_m": size,
        "sdf_sha256": sha(path),
    }


def read_states(path: Path) -> list[dict]:
    rows = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        require(len(fields) == 16, f"invalid passive state row: {fields[0] if fields else ''}")
        values = [float(value) for value in fields[1:]]
        require(all(math.isfinite(value) for value in values), f"nonfinite passive state: {fields[0]}")
        rows.append({"label": fields[0], "values": values})
    require(len(rows) >= 100, "passive state matrix is too small")
    for index, row in enumerate(rows):
        require(row["label"] == f"sample_{index:06d}", f"state label mismatch at {index}")
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--supported-dir", required=True)
    parser.add_argument("--pregrasp-dir", required=True)
    parser.add_argument("--current-urdf", required=True)
    parser.add_argument("--current-srdf", required=True)
    parser.add_argument("--fixture-tsv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    supported = Path(args.supported_dir)
    pregrasp = Path(args.pregrasp_dir)
    verify_files(supported, PR7_FILES)
    verify_files(pregrasp, PR21_FILES)
    require(sha(Path(args.current_urdf)) == PR21_FILES["turtlebot3_lime_pregrasp_search.urdf"], "current URDF differs from accepted PR21 model")
    require(sha(Path(args.current_srdf)) == PR21_FILES["turtlebot3_lime.srdf"], "current SRDF differs from accepted PR21 model")

    telemetry = json.loads((supported / "rubiks_dynamic_supported_hold_telemetry.json").read_text())
    supported_summary = json.loads((supported / "rubiks_dynamic_supported_hold_summary.json").read_text())
    pregrasp_summary = json.loads((pregrasp / "rubiks_pregrasp_simulation_state_summary.json").read_text())
    passive_summary = json.loads((pregrasp / "rubiks_pregrasp_gripper_ground_clearance_summary.json").read_text())
    passive_manifest = json.loads((pregrasp / "rubiks_pregrasp_gripper_ground_clearance_evidence.manifest.json").read_text())

    require(supported_summary.get("passed") is True and supported_summary.get("errors") == [], "accepted supported hold failed")
    require(telemetry.get("errors") == [] and telemetry.get("unexpected_robot_contact_count") == 0, "accepted supported hold contains unexpected contact")
    require(telemetry.get("arm_motion_performed") is False and telemetry.get("cube_lifted") is False, "accepted fixture safety mismatch")
    require(passive_summary.get("source_ref") == PR21_SOURCE and passive_summary.get("passed") is True, "accepted PR21 summary mismatch")
    require(passive_manifest.get("source_ref") == PR21_SOURCE and passive_manifest.get("all_states_passed") is True, "accepted PR21 manifest mismatch")
    require(pregrasp_summary.get("source_ref") == PR21_SOURCE and pregrasp_summary.get("passed") is True, "accepted PR21 simulation mismatch")
    require(pregrasp_summary.get("joint_order") == ARM, "accepted arm order mismatch")

    cube = sdf_box(supported / "rubiks_dynamic_cube_supported_057.sdf", "rubiks_dynamic_cube_supported_057", "cube_link", "cube_collision", False)
    support = sdf_box(supported / "rubiks_cube_support_surface.sdf", "rubiks_cube_support_surface", "support_link", "support_collision", True)
    require(cube["size_m"] == [0.057, 0.057, 0.057], "canonical cube size mismatch")
    require(support["size_m"] == [0.075, 0.04, 0.01], "support size mismatch")
    require(math.isclose(float(telemetry.get("cube_mass_kg")), 0.09, abs_tol=1e-12), "cube mass mismatch")
    require(telemetry.get("cube_pose_relative_to") == "turtlebot3_lime_gripper_test::link7", "fixture reference frame mismatch")
    cube_offset = [float(value) for value in telemetry.get("cube_pose_xyz_m", [])]
    support_offset = [float(value) for value in telemetry.get("support_pose_xyz_m", [])]
    require(len(cube_offset) == len(support_offset) == 3, "fixture offset missing")
    require(all(math.isfinite(value) for value in cube_offset + support_offset), "fixture offset nonfinite")
    require(cube_offset[:2] == support_offset[:2] == [-0.019, 0.0], "fixture lateral alignment mismatch")
    expected_vertical = cube["size_m"][2] / 2.0 + support["size_m"][2] / 2.0
    vertical_separation = cube_offset[2] - support_offset[2]
    require(math.isclose(vertical_separation, expected_vertical, abs_tol=1e-12), "cube/support surface contact mismatch")
    require(support["size_m"][0] >= cube["size_m"][0], "support is too short in X")
    require(support["size_m"][1] > 0.0 and cube["size_m"][1] > support["size_m"][1], "unexpected support Y coverage")

    states = read_states(pregrasp / "rubiks_pregrasp_gripper_ground_states.tsv")
    evidence = pregrasp_summary.get("evidence", {})
    candidate_goal = evidence.get("candidate_goal", {})
    return_goal = evidence.get("return_goal", {})
    candidate_start = int(candidate_goal.get("sample_end_index", -1))
    anchor_index = int(return_goal.get("sample_start_index", -1)) - 1
    require(0 <= candidate_start <= anchor_index < len(states), "candidate anchor interval invalid")
    require(anchor_index - candidate_start + 1 >= 400, "candidate hold interval too short")
    candidate = [float(value) for value in pregrasp_summary.get("candidate_joint_positions_rad", [])]
    anchor_arm = states[anchor_index]["values"][7:13]
    require(len(candidate) == 6 and max(abs(a - b) for a, b in zip(anchor_arm, candidate)) <= 1e-3, "anchor is not at selected pose")

    fixture_path = Path(args.fixture_tsv)
    fixture_path.write_text(
        "META\t1\tlink7\t" + str(anchor_index) + "\t" + str(candidate_start) + "\t" + str(len(states)) + "\n"
        + "OBJECT\trubiks_cube\t" + "\t".join(f"{value:.17g}" for value in cube["size_m"] + cube_offset) + "\n"
        + "OBJECT\trubiks_support\t" + "\t".join(f"{value:.17g}" for value in support["size_m"] + support_offset) + "\n",
        encoding="utf-8",
    )

    payload = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-FIXTURE-PLACEMENT-OFFLINE-INPUT-BINDING",
        "source_ref": args.source_ref,
        "accepted_sources": {"supported_hold_head": PR7_SOURCE, "passive_pregrasp_head": PR21_SOURCE},
        "passed": True,
        "errors": [],
        "state_count": len(states),
        "candidate_hold_start_index": candidate_start,
        "fixture_anchor_state_index": anchor_index,
        "fixture_reference_link": "link7",
        "fixture_objects": {"cube": {**cube, "offset_from_link7_m": cube_offset, "mass_kg": 0.09}, "support": {**support, "offset_from_link7_m": support_offset}},
        "intended_fixture_contact": {
            "pair": ["rubiks_cube", "rubiks_support"],
            "center_vertical_separation_m": vertical_separation,
            "half_height_sum_m": expected_vertical,
            "x_overlap_m": min(cube["size_m"][0], support["size_m"][0]),
            "y_overlap_m": min(cube["size_m"][1], support["size_m"][1]),
            "support_y_overhang_per_side_m": (cube["size_m"][1] - support["size_m"][1]) / 2.0,
            "robot_fixture_contact_allowed": False,
        },
        "fixture_policy": "anchor accepted link7-relative transforms at final selected-pose hold state, then keep both objects fixed in world for the complete recorded path",
        "current_model_exactly_matches_accepted_pregrasp": True,
        "fixture_tsv": fixture_path.name,
        "safety": {
            "offline_evidence_binding_only": True,
            "gazebo_started": False,
            "object_spawned": False,
            "ros_node_created": False,
            "controller_loaded": False,
            "command_sent": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
        },
    }
    Path(args.output).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
