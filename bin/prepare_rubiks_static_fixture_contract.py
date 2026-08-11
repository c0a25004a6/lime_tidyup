#!/usr/bin/env python3
"""Prepare corrected Rubik fixture inputs for the post-PR28 static Gazebo gate.

This helper is deliberately command-free. For a positive PR #28 result it
applies the selected rigid translation to the accepted cube/support fixture TSV
and emits an observation-only contract. It does not start ROS/Gazebo or send
arm/gripper commands.
"""
from __future__ import annotations

import argparse
import json
import math
import tempfile
from pathlib import Path

CENTER = (0.06325, 0.0, 0.01225)
POSITIVE = "GRIPPER_FIXTURE_COMPATIBLE_FULL_PATH_CLEAR_TRANSLATION_FOUND"
EXPECTED_PHASE = "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-CORRECTION"
NEXT_PHASE = "RUBIK-PREGRASP-CORRECTED-FIXTURE-STATIC-GAZEBO-NO-ACTUATION-EVIDENCE"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def vec3(value: object, name: str) -> list[float]:
    require(isinstance(value, list) and len(value) == 3, f"{name} must be a vec3")
    result = [float(item) for item in value]
    require(all(math.isfinite(item) for item in result), f"{name} must be finite")
    return result


def validate_summary(summary: dict[str, object]) -> list[float]:
    require(summary.get("phase") == EXPECTED_PHASE, "unexpected PR28 phase")
    require(summary.get("audit_passed") is True and summary.get("errors") == [], "PR28 audit failed")
    require(summary.get("exact_head_bound") is True, "PR28 evidence is not exact-head bound")
    require(summary.get("decision") == POSITIVE, "static fixture contract requires a positive PR28 result")
    require(summary.get("candidate_found") is True, "positive PR28 result has no candidate")
    require(vec3(summary.get("center_translation_link7_m"), "center") == list(CENTER), "PR28 center mismatch")
    require(summary.get("next_required_phase") == NEXT_PHASE, "PR28 next phase mismatch")

    selected = summary.get("selected_candidate")
    require(isinstance(selected, dict), "selected candidate missing")
    translation = vec3(selected.get("translation_m"), "selected translation")
    require(int(selected.get("selected_states_checked", -1)) == 2802, "selected candidate lacks 2,802-state evidence")
    sweep = selected.get("sweep")
    require(isinstance(sweep, dict), "selected sweep missing")
    require(sweep.get("open_clear") is True, "selected sweep is not open-clear")
    require(sweep.get("dual_contact") is True, "selected sweep lacks dual finger-cube contact")
    require(sweep.get("forbidden_before_dual") is False, "selected sweep has forbidden collision before dual contact")

    safety = summary.get("safety")
    require(isinstance(safety, dict), "PR28 safety block missing")
    for key in (
        "gazebo_started",
        "ros_node_created",
        "object_spawned",
        "controller_loaded",
        "planning_request_created",
        "trajectory_instantiated",
        "command_sent",
        "attachment_used",
        "physical_hardware_used",
        "production_runtime_modified",
    ):
        require(safety.get(key) is False, f"PR28 safety boundary violated: {key}")
    return translation


def shift_fixture(source: Path, destination: Path, selected_translation: list[float]) -> list[dict[str, object]]:
    shift = [selected_translation[index] - CENTER[index] for index in range(3)]
    output: list[str] = []
    objects: list[dict[str, object]] = []
    for raw in source.read_text(encoding="utf-8").splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] != "OBJECT":
            output.append(raw)
            continue
        require(len(fields) == 8, "invalid fixture OBJECT row")
        original = [float(item) for item in fields[5:8]]
        require(all(math.isfinite(item) for item in original), "fixture offset is not finite")
        corrected = [original[index] + shift[index] for index in range(3)]
        fields[5:8] = [f"{item:.17g}" for item in corrected]
        output.append("\t".join(fields))
        objects.append({
            "kind": fields[1],
            "name": fields[2],
            "original_offset_link7_m": original,
            "corrected_offset_link7_m": corrected,
        })
    require(len(objects) == 2, "expected exactly cube and support OBJECT rows")
    names = {str(item["name"]).lower() for item in objects}
    require(any("cube" in name for name in names), "cube OBJECT row missing")
    require(any("support" in name for name in names), "support OBJECT row missing")
    destination.write_text("\n".join(output) + "\n", encoding="utf-8")
    return objects


def build_contract(summary: dict[str, object], source_fixture: Path, corrected_fixture: Path) -> dict[str, object]:
    selected = validate_summary(summary)
    objects = shift_fixture(source_fixture, corrected_fixture, selected)
    shift = [selected[index] - CENTER[index] for index in range(3)]
    return {
        "schema_version": 1,
        "phase": NEXT_PHASE,
        "input_phase": EXPECTED_PHASE,
        "input_source_ref": summary.get("source_ref"),
        "selected_translation_link7_m": selected,
        "translation_delta_from_pr28_center_m": shift,
        "corrected_fixture_path": corrected_fixture.name,
        "objects": objects,
        "observation_contract": {
            "gazebo_allowed": True,
            "ros_state_observation_allowed": True,
            "fixture_spawn_allowed": True,
            "arm_command_allowed": False,
            "gripper_command_allowed": False,
            "base_command_allowed": False,
            "controller_state_change_allowed": False,
            "lift_allowed": False,
            "attachment_allowed": False,
            "physical_hardware_allowed": False,
            "production_runtime_modification_allowed": False,
        },
        "required_observations": [
            "exact robot arm/finger initial state before fixture spawn",
            "link7 pose before and after fixture spawn",
            "cube and support model poses after spawn",
            "cube-support contact coverage and settling",
            "zero finger-cube contact while open/passive before any later close gate",
            "zero non-finger robot-fixture contact",
            "cube translation/rotation/speed during settling",
        ],
        "grasp_success_claimed": False,
    }


def fixture_text() -> str:
    return (
        "META\t1\tfixture\n"
        "OBJECT\tBOX\trubiks_cube\t0.057\t0.057\t-0.019\t0\t0.1192\n"
        "OBJECT\tBOX\trubiks_support\t0.075\t0.040\t-0.019\t0\t0.0857\n"
    )


def positive_summary(translation: list[float]) -> dict[str, object]:
    return {
        "phase": EXPECTED_PHASE,
        "source_ref": "a" * 40,
        "exact_head_bound": True,
        "audit_passed": True,
        "errors": [],
        "decision": POSITIVE,
        "candidate_found": True,
        "center_translation_link7_m": list(CENTER),
        "selected_candidate": {
            "translation_m": translation,
            "selected_states_checked": 2802,
            "sweep": {
                "open_clear": True,
                "dual_contact": True,
                "forbidden_before_dual": False,
            },
        },
        "next_required_phase": NEXT_PHASE,
        "safety": {
            "offline_analysis_only": True,
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


def self_test() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        source = root / "fixture.tsv"
        corrected = root / "corrected.tsv"
        source.write_text(fixture_text(), encoding="utf-8")
        selected = [CENTER[0] - 0.0005, CENTER[1] + 0.00025, CENTER[2] + 0.001]
        contract = build_contract(positive_summary(selected), source, corrected)
        require(contract["translation_delta_from_pr28_center_m"] == [-0.0005, 0.00025, 0.001], "translation delta mismatch")
        objects = contract["objects"]
        require(isinstance(objects, list) and len(objects) == 2, "object contract count mismatch")
        cube = next(item for item in objects if "cube" in str(item["name"]).lower())
        require(abs(cube["corrected_offset_link7_m"][0] - (-0.0195)) < 1e-12, "cube X correction mismatch")
        require(abs(cube["corrected_offset_link7_m"][1] - 0.00025) < 1e-12, "cube Y correction mismatch")
        require(abs(cube["corrected_offset_link7_m"][2] - 0.1202) < 1e-12, "cube Z correction mismatch")
        require(contract["observation_contract"]["arm_command_allowed"] is False, "arm command accidentally allowed")
        require(contract["observation_contract"]["gripper_command_allowed"] is False, "gripper command accidentally allowed")
        require(contract["observation_contract"]["lift_allowed"] is False, "lift accidentally allowed")

        broken = positive_summary(selected)
        broken["selected_candidate"]["sweep"]["forbidden_before_dual"] = True
        try:
            build_contract(broken, source, corrected)
        except ValueError:
            pass
        else:
            raise AssertionError("forbidden-collision candidate was accepted")
    print("prepare_rubiks_static_fixture_contract self-test: PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pr28-summary")
    parser.add_argument("--fixture")
    parser.add_argument("--corrected-fixture")
    parser.add_argument("--output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    require(bool(args.pr28_summary), "--pr28-summary is required")
    require(bool(args.fixture), "--fixture is required")
    require(bool(args.corrected_fixture), "--corrected-fixture is required")
    require(bool(args.output), "--output is required")
    summary = json.loads(Path(args.pr28_summary).read_text(encoding="utf-8"))
    contract = build_contract(summary, Path(args.fixture), Path(args.corrected_fixture))
    Path(args.output).write_text(json.dumps(contract, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(contract, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
