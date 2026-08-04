#!/usr/bin/env python3
"""Bind accepted PR24 evidence and emit the constrained-search contract."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

PR24_SOURCE = "7d8b9b055cd331d11b89ba275a1ecce7312b4b84"
EXTERNAL_SOURCE = "3363b27aae6d37598dc142cf4dddb342fef3047f"
EXPECTED_FILES = {
    "rubiks_pregrasp_corrected_fixture_geometric_relevance_summary.json": "f72908a2a1993341946fee90f2e36e9d00926b3e69b65d4d4817257aaf64aa54",
    "rubiks_pregrasp_corrected_fixture_geometric_relevance_evidence.manifest.json": "b6e300d20d2032c294f8613a22d94b5338092d3a49157b0fa42e9e7751a1d579",
    "rubiks_pregrasp_corrected_fixture_mesh_provenance.json": "d58fe9076f5759045a08e5569be4615df206656081bfd42e0a4c75691e32e959",
    "rubiks_pregrasp_corrected_fixture_relevance_results.tsv": "3ccec85e4a0a2fe3fd839b620c16153326376ef55a44362e45a93a47c40c71dd",
    "rubiks_pregrasp_corrected_fixture_relevance_input_binding.json": "e3505f89a5b85bb275c77e677785b0c53e4460208fc7147bd1462d094211a2e5",
    "rubiks_pregrasp_corrected_fixture_relevance_contract.tsv": "7e8ea4073415c19a3b729421debc23c47518ae147be767f668dfb74f546d1244",
    "accepted/accepted/turtlebot3_lime_fixture_current.urdf": "58c77314a7af7f743064a1f382bc7741edb806c8f2563180ff76580ccad126b8",
    "accepted/accepted/turtlebot3_lime_fixture_current.srdf": "fd3778f7c8156687ab8301f01aa9dc915c40179fa0d6f6bd1ceefd77efc71d1f",
    "accepted/accepted/pregrasp/rubiks_pregrasp_gripper_ground_states.tsv": "baed5397ad5c40835778fefc3e302ad1716cc8af278fe856b77fbb79fe106cf3",
    "accepted/accepted/rubiks_pregrasp_fixture_contract.tsv": "aa0c7014fc6543ceb7effea230f3d3767b13fb527a7dbb278910f8b262732b80",
}
COARSE_STEP_M = 0.002
COARSE_RADIUS_STEPS = 40
EXPECTED_COARSE_COUNT = 267761
EXPECTED_RELEVANT_COARSE_COUNT = 61511
FINE_STEP_M = 0.00025
FINE_RADIUS_STEPS = 8
EXPECTED_FINE_COUNT = 4913


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def number(text: str) -> float:
    value = float(text)
    require(math.isfinite(value), f"nonfinite number: {text}")
    return value


def parse_fixture(path: Path) -> dict[str, object]:
    meta = None
    objects: dict[str, dict[str, list[float]]] = {}
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 6 and fields[1] == "1", "invalid fixture META")
            meta = {
                "reference_link": fields[2],
                "anchor_index": int(fields[3]),
                "candidate_start_index": int(fields[4]),
                "state_count": int(fields[5]),
            }
        elif fields[0] == "OBJECT":
            require(len(fields) == 8 and fields[1] not in objects, "invalid fixture OBJECT")
            objects[fields[1]] = {
                "size": [number(value) for value in fields[2:5]],
                "center": [number(value) for value in fields[5:8]],
            }
        else:
            raise ValueError(f"unknown fixture row: {fields[0]}")
    require(meta is not None, "fixture META missing")
    require(set(objects) == {"rubiks_cube", "rubiks_support"}, "fixture objects mismatch")
    return {"meta": meta, "objects": objects}


def parse_relevance_results(path: Path) -> dict[str, object]:
    positions: dict[str, dict[str, object]] = {}
    decision = None
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "POSITION":
            require(len(fields) == 23 and fields[1] not in positions, "invalid POSITION row")
            positions[fields[1]] = {
                "position": number(fields[2]),
                "left_min": [number(value) for value in fields[5:8]],
                "left_max": [number(value) for value in fields[8:11]],
                "right_min": [number(value) for value in fields[11:14]],
                "right_max": [number(value) for value in fields[14:17]],
            }
        elif fields[0] == "DECISION":
            require(len(fields) == 5 and decision is None, "invalid DECISION row")
            decision = fields[1]
    require(set(positions) == {"lower", "zero", "passive", "upper"}, "position coverage mismatch")
    require(decision == "BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE", "PR24 blocker mismatch")
    return {"positions": positions, "decision": decision}


def positive_overlap(first_min: float, first_max: float, second_min: float, second_max: float) -> bool:
    return min(first_max, second_max) - max(first_min, second_min) > 0.0


def relevant(
    delta: tuple[float, float, float],
    cube: dict[str, list[float]],
    left_min: list[float],
    left_max: list[float],
    right_min: list[float],
    right_max: list[float],
    lower: float,
    upper: float,
) -> bool:
    center = [cube["center"][axis] + delta[axis] for axis in range(3)]
    half = [value * 0.5 for value in cube["size"]]
    cube_min = [center[axis] - half[axis] for axis in range(3)]
    cube_max = [center[axis] + half[axis] for axis in range(3)]
    if not (
        positive_overlap(left_min[0], left_max[0], cube_min[0], cube_max[0])
        and positive_overlap(left_min[2], left_max[2], cube_min[2], cube_max[2])
        and positive_overlap(right_min[0], right_max[0], cube_min[0], cube_max[0])
        and positive_overlap(right_min[2], right_max[2], cube_min[2], cube_max[2])
    ):
        return False
    left_low = cube_max[1] - left_max[1]
    left_high = cube_max[1] - left_min[1]
    right_low = right_min[1] - cube_min[1]
    right_high = right_max[1] - cube_min[1]
    return max(lower, left_low, right_low) <= min(upper, left_high, right_high) + 1e-12


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--accepted-dir", required=True)
    parser.add_argument("--contract-tsv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    root = Path(args.accepted_dir)
    require(root.is_dir(), "accepted PR24 artifact directory is missing")
    observed: dict[str, str] = {}
    for relative, expected in EXPECTED_FILES.items():
        path = root / relative
        require(path.is_file(), f"accepted input missing: {relative}")
        digest = sha256(path)
        require(digest == expected, f"accepted input digest mismatch: {relative}")
        observed[relative] = digest

    summary = json.loads((root / "rubiks_pregrasp_corrected_fixture_geometric_relevance_summary.json").read_text())
    manifest = json.loads((root / "rubiks_pregrasp_corrected_fixture_geometric_relevance_evidence.manifest.json").read_text())
    provenance = json.loads((root / "rubiks_pregrasp_corrected_fixture_mesh_provenance.json").read_text())
    require(summary.get("source_ref") == PR24_SOURCE, "PR24 summary source mismatch")
    require(summary.get("audit_passed") is True and summary.get("errors") == [], "PR24 audit failed")
    require(summary.get("decision") == "BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE", "PR24 decision mismatch")
    require(summary.get("corrected_fixture_geometrically_relevant") is False, "PR24 relevance mismatch")
    require(summary.get("positive_xz_contact_patch_overlap") is False, "PR24 XZ blocker missing")
    require(summary.get("simultaneous_mimic_side_plane_reachability") is True, "PR24 Y reach mismatch")
    require(manifest.get("source_ref") == PR24_SOURCE, "PR24 manifest source mismatch")
    require(manifest.get("decision") == summary.get("decision"), "PR24 manifest decision mismatch")
    source_repository = provenance.get("source_repository", {})
    require(source_repository.get("commit") == EXTERNAL_SOURCE, "external source commit mismatch")
    require(source_repository.get("expected_commit") == EXTERNAL_SOURCE, "external expected commit mismatch")
    require(source_repository.get("exact_commit_bound") is True, "external source is not exact-bound")

    fixture = parse_fixture(root / "accepted/accepted/rubiks_pregrasp_fixture_contract.tsv")
    relevance = parse_relevance_results(root / "rubiks_pregrasp_corrected_fixture_relevance_results.tsv")
    meta = fixture["meta"]
    require(meta == {
        "reference_link": "link7",
        "anchor_index": 1541,
        "candidate_start_index": 1043,
        "state_count": 2802,
    }, "fixture identity mismatch")
    cube = fixture["objects"]["rubiks_cube"]
    support = fixture["objects"]["rubiks_support"]
    require(cube == {"size": [0.057, 0.057, 0.057], "center": [-0.019, 0.0, 0.1192]}, "cube contract mismatch")
    require(support == {"size": [0.075, 0.04, 0.01], "center": [-0.019, 0.0, 0.0857]}, "support contract mismatch")
    zero = relevance["positions"]["zero"]
    lower = relevance["positions"]["lower"]["position"]
    upper = relevance["positions"]["upper"]["position"]
    require(math.isclose(lower, -0.01, abs_tol=1e-15), "lower gripper limit mismatch")
    require(math.isclose(upper, 0.019, abs_tol=1e-15), "upper gripper limit mismatch")

    total = 0
    relevant_count = 0
    relevant_z_steps: list[int] = []
    radius2 = COARSE_RADIUS_STEPS * COARSE_RADIUS_STEPS
    for x in range(-COARSE_RADIUS_STEPS, COARSE_RADIUS_STEPS + 1):
        for y in range(-COARSE_RADIUS_STEPS, COARSE_RADIUS_STEPS + 1):
            for z in range(-COARSE_RADIUS_STEPS, COARSE_RADIUS_STEPS + 1):
                if x * x + y * y + z * z > radius2:
                    continue
                total += 1
                delta = (x * COARSE_STEP_M, y * COARSE_STEP_M, z * COARSE_STEP_M)
                if relevant(
                    delta, cube,
                    zero["left_min"], zero["left_max"],
                    zero["right_min"], zero["right_max"],
                    lower, upper,
                ):
                    relevant_count += 1
                    relevant_z_steps.append(z)
    require(total == EXPECTED_COARSE_COUNT, "coarse candidate count mismatch")
    require(relevant_count == EXPECTED_RELEVANT_COARSE_COUNT, "relevant coarse candidate count mismatch")
    require(min(relevant_z_steps) == -40 and max(relevant_z_steps) == 22, "relevant Z range mismatch")

    contract_path = Path(args.contract_tsv)
    with contract_path.open("w", encoding="utf-8") as stream:
        stream.write(f"META\t1\t{meta['state_count']}\t{meta['anchor_index']}\t{meta['candidate_start_index']}\t{meta['reference_link']}\n")
        stream.write("CUBE\t" + "\t".join(f"{value:.17g}" for value in (*cube["size"], *cube["center"])) + "\n")
        stream.write("SUPPORT\t" + "\t".join(f"{value:.17g}" for value in (*support["size"], *support["center"])) + "\n")
        stream.write("LEFT_ZERO\t" + "\t".join(f"{value:.17g}" for value in (*zero["left_min"], *zero["left_max"])) + "\n")
        stream.write("RIGHT_ZERO\t" + "\t".join(f"{value:.17g}" for value in (*zero["right_min"], *zero["right_max"])) + "\n")
        stream.write(f"GRIPPER\t{lower:.17g}\t{upper:.17g}\n")
        stream.write(
            f"COARSE\t{COARSE_STEP_M:.17g}\t{COARSE_RADIUS_STEPS * COARSE_STEP_M:.17g}"
            f"\t{EXPECTED_COARSE_COUNT}\t{EXPECTED_RELEVANT_COARSE_COUNT}\t-40\t22\n"
        )
        stream.write(
            f"FINE\t{FINE_STEP_M:.17g}\t{FINE_RADIUS_STEPS * FINE_STEP_M:.17g}"
            f"\t{EXPECTED_FINE_COUNT}\n"
        )
        stream.write("PR24_BLOCKED\t0\t0\t0.045750000000000001\n")
        stream.write(f"EXTERNAL\t{EXTERNAL_SOURCE}\n")

    binding = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-GRASP-RELEVANCE-CONSTRAINED-FIXTURE-SEARCH-INPUT",
        "source_ref": args.source_ref,
        "accepted_source": PR24_SOURCE,
        "passed": True,
        "errors": [],
        "input_sha256": observed,
        "state_count": meta["state_count"],
        "anchor_index": meta["anchor_index"],
        "candidate_start_index": meta["candidate_start_index"],
        "reference_link": meta["reference_link"],
        "coarse_candidate_count": total,
        "coarse_relevant_candidate_count": relevant_count,
        "coarse_relevant_z_step_range": [-40, 22],
        "hard_filter": {
            "positive_cube_finger_aabb_overlap_x_and_z": True,
            "common_mimic_side_plane_interval_within_limits": True,
            "collision_rules_weakened": False,
        },
        "external_source_commit": EXTERNAL_SOURCE,
        "contract": {"path": contract_path.name, "sha256": sha256(contract_path)},
        "safety": {
            "offline_analysis_only": True,
            "gazebo_started": False,
            "ros_node_created": False,
            "controller_loaded": False,
            "planning_request_created": False,
            "trajectory_instantiated": False,
            "command_sent": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
        },
    }
    Path(args.output).write_text(json.dumps(binding, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(binding, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
