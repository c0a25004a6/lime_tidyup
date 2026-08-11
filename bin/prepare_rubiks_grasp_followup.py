#!/usr/bin/env python3
"""Prepare the next Rubik grasp phase from PR #28 compatibility evidence.

This tool is intentionally command-free. It validates the exact PR #28 summary
and either:

* emits a static-Gazebo/no-actuation follow-up plan when an X-only compatible
  full-path-clear fixture translation exists; or
* emits the deterministic remaining local 3-D translation grid when the X-only
  search has no usable solution.

It never starts ROS/Gazebo, never creates a controller/action client, and never
sends a robot command.
"""
from __future__ import annotations

import argparse
import json
import math
import tempfile
from pathlib import Path
from typing import Iterable

CENTER = (0.06325, 0.0, 0.01225)
STEP_M = 0.00025
RADIUS_STEPS = 8
PRIOR_X_COUNT = 17
FULL_CUBE_COUNT = (2 * RADIUS_STEPS + 1) ** 3
REMAINING_3D_COUNT = FULL_CUBE_COUNT - PRIOR_X_COUNT

POSITIVE = "GRIPPER_FIXTURE_COMPATIBLE_FULL_PATH_CLEAR_TRANSLATION_FOUND"
NEGATIVE = {
    "NO_SWEEP_COMPATIBLE_TRANSLATION_IN_LOCAL_X_DOMAIN",
    "NO_SWEEP_COMPATIBLE_FULL_PATH_CLEAR_TRANSLATION_IN_LOCAL_X_DOMAIN",
}

STATIC_PHASE = "RUBIK-PREGRASP-CORRECTED-FIXTURE-STATIC-GAZEBO-NO-ACTUATION-EVIDENCE"
EXPANSION_PHASE = "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-3D-EXPANSION"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _finite_vec3(value: object, name: str) -> list[float]:
    require(isinstance(value, list) and len(value) == 3, f"{name} must be vec3")
    result = [float(item) for item in value]
    require(all(math.isfinite(item) for item in result), f"{name} must be finite")
    return result


def validate_pr28(summary: dict[str, object]) -> str:
    require(summary.get("phase") == "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-CORRECTION", "unexpected phase")
    require(summary.get("audit_passed") is True, "PR28 audit did not pass")
    require(summary.get("errors") == [], "PR28 summary contains errors")
    require(summary.get("exact_head_bound") is True, "PR28 summary is not exact-head bound")
    require(_finite_vec3(summary.get("center_translation_link7_m"), "center") == list(CENTER), "center mismatch")

    x_scan = summary.get("x_scan")
    require(isinstance(x_scan, dict), "x_scan missing")
    require(int(x_scan.get("candidate_count", -1)) == PRIOR_X_COUNT, "X scan coverage mismatch")
    require(abs(float(x_scan.get("step_m", -1.0)) - STEP_M) < 1e-12, "X scan step mismatch")
    require(abs(float(x_scan.get("radius_m", -1.0)) - RADIUS_STEPS * STEP_M) < 1e-12, "X scan radius mismatch")

    safety = summary.get("safety")
    require(isinstance(safety, dict), "safety block missing")
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

    decision = str(summary.get("decision"))
    require(decision == POSITIVE or decision in NEGATIVE, f"unexpected PR28 decision: {decision}")

    if decision == POSITIVE:
        require(summary.get("candidate_found") is True, "positive decision without candidate")
        selected = summary.get("selected_candidate")
        require(isinstance(selected, dict), "selected candidate missing")
        _finite_vec3(selected.get("translation_m"), "selected translation")
        require(int(selected.get("selected_states_checked", -1)) == 2802, "selected candidate lacks 2,802-state evidence")
        sweep = selected.get("sweep")
        require(isinstance(sweep, dict), "selected sweep missing")
        require(sweep.get("open_clear") is True, "selected sweep lacks open-clear prefix")
        require(sweep.get("dual_contact") is True, "selected sweep lacks dual contact")
        require(sweep.get("forbidden_before_dual") is False, "selected sweep has forbidden pre-contact state")
        require(summary.get("next_required_phase") == STATIC_PHASE, "positive next phase mismatch")
    else:
        require(summary.get("candidate_found") is False, "negative decision unexpectedly has candidate")
        require(summary.get("selected_candidate") is None, "negative decision has selected candidate")
        require(summary.get("next_required_phase") == EXPANSION_PHASE, "negative next phase mismatch")

    return decision


def candidate_offsets() -> list[tuple[int, int, int]]:
    """Return the remaining 3-D local grid in deterministic distance-first order.

    The 17 points on the already-tested X line (dy=dz=0) are excluded, so no
    PR #28 sweep work is repeated.
    """
    rows: list[tuple[int, int, int]] = []
    for dx in range(-RADIUS_STEPS, RADIUS_STEPS + 1):
        for dy in range(-RADIUS_STEPS, RADIUS_STEPS + 1):
            for dz in range(-RADIUS_STEPS, RADIUS_STEPS + 1):
                if dy == 0 and dz == 0:
                    continue
                rows.append((dx, dy, dz))
    rows.sort(
        key=lambda item: (
            item[0] * item[0] + item[1] * item[1] + item[2] * item[2],
            abs(item[0]) + abs(item[1]) + abs(item[2]),
            abs(item[2]),
            abs(item[1]),
            abs(item[0]),
            item[2],
            item[1],
            item[0],
        )
    )
    require(len(rows) == REMAINING_3D_COUNT, "remaining 3-D grid coverage mismatch")
    require(len(set(rows)) == len(rows), "duplicate 3-D candidate")
    return rows


def write_3d_candidates(path: Path) -> None:
    rows = candidate_offsets()
    with path.open("w", encoding="utf-8") as stream:
        stream.write(
            "META\t1\tXYZ_REMAINING\t"
            f"{STEP_M:.17g}\t{RADIUS_STEPS}\t{len(rows)}\t{PRIOR_X_COUNT}\t"
            f"{CENTER[0]:.17g}\t{CENTER[1]:.17g}\t{CENTER[2]:.17g}\n"
        )
        for rank, (dx, dy, dz) in enumerate(rows):
            translation = (
                CENTER[0] + dx * STEP_M,
                CENTER[1] + dy * STEP_M,
                CENTER[2] + dz * STEP_M,
            )
            distance = STEP_M * math.sqrt(dx * dx + dy * dy + dz * dz)
            stream.write(
                f"CANDIDATE\t{rank}\t"
                f"{translation[0]:.17g}\t{translation[1]:.17g}\t{translation[2]:.17g}\t"
                f"{distance:.17g}\t{dx}\t{dy}\t{dz}\n"
            )


def build_plan(summary: dict[str, object], candidates_output: Path | None) -> dict[str, object]:
    decision = validate_pr28(summary)
    source_ref = str(summary.get("source_ref", ""))
    require(len(source_ref) == 40 and all(char in "0123456789abcdef" for char in source_ref.lower()), "invalid PR28 source_ref")

    common = {
        "schema_version": 1,
        "input_phase": summary["phase"],
        "input_source_ref": source_ref,
        "input_decision": decision,
        "actuation_authorized": False,
        "physical_hardware_authorized": False,
        "attachment_authorized": False,
        "production_runtime_modification_authorized": False,
    }

    if decision == POSITIVE:
        require(candidates_output is None, "3-D candidate output must not be requested for positive X result")
        selected = summary["selected_candidate"]
        assert isinstance(selected, dict)
        return {
            **common,
            "route": "STATIC_GAZEBO_NO_ACTUATION",
            "next_phase": STATIC_PHASE,
            "selected_translation_link7_m": _finite_vec3(selected.get("translation_m"), "selected translation"),
            "selected_states_checked": int(selected["selected_states_checked"]),
            "gazebo_allowed_in_next_phase": True,
            "ros_observation_allowed_in_next_phase": True,
            "controller_command_allowed_in_next_phase": False,
            "gripper_command_allowed_in_next_phase": False,
            "arm_command_allowed_in_next_phase": False,
            "lift_allowed_in_next_phase": False,
        }

    require(candidates_output is not None, "negative X result requires --candidates-output")
    write_3d_candidates(candidates_output)
    return {
        **common,
        "route": "LOCAL_3D_EXPANSION",
        "next_phase": EXPANSION_PHASE,
        "grid": {
            "center_translation_link7_m": list(CENTER),
            "step_m": STEP_M,
            "radius_per_axis_m": RADIUS_STEPS * STEP_M,
            "full_cube_candidate_count": FULL_CUBE_COUNT,
            "previous_x_line_candidate_count": PRIOR_X_COUNT,
            "remaining_candidate_count": REMAINING_3D_COUNT,
            "previous_x_line_excluded": True,
            "ordering": "distance_squared_then_l1_then_abs_z_y_x_then_signed_z_y_x",
            "candidates_path": candidates_output.name,
        },
        "gazebo_allowed_in_next_phase": False,
        "ros_observation_allowed_in_next_phase": False,
        "controller_command_allowed_in_next_phase": False,
        "gripper_command_allowed_in_next_phase": False,
        "arm_command_allowed_in_next_phase": False,
        "lift_allowed_in_next_phase": False,
    }


def _fixture_summary(decision: str, source_ref: str = "a" * 40) -> dict[str, object]:
    base: dict[str, object] = {
        "phase": "RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-CORRECTION",
        "source_ref": source_ref,
        "exact_head_bound": True,
        "audit_passed": True,
        "errors": [],
        "decision": decision,
        "center_translation_link7_m": list(CENTER),
        "x_scan": {
            "step_m": STEP_M,
            "radius_m": RADIUS_STEPS * STEP_M,
            "candidate_count": PRIOR_X_COUNT,
            "sweep_compatible_count": 0,
        },
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
    if decision == POSITIVE:
        base.update({
            "candidate_found": True,
            "selected_candidate": {
                "translation_m": [0.063, 0.0, 0.01225],
                "selected_states_checked": 2802,
                "sweep": {
                    "open_clear": True,
                    "dual_contact": True,
                    "forbidden_before_dual": False,
                },
            },
            "next_required_phase": STATIC_PHASE,
        })
    else:
        base.update({
            "candidate_found": False,
            "selected_candidate": None,
            "next_required_phase": EXPANSION_PHASE,
        })
    return base


def self_test() -> None:
    rows = candidate_offsets()
    require(len(rows) == 4896, "expected 4,896 untested 3-D candidates")
    require(all(not (dy == 0 and dz == 0) for _, dy, dz in rows), "prior X line leaked into 3-D grid")
    require(rows[0][0] ** 2 + rows[0][1] ** 2 + rows[0][2] ** 2 == 1, "nearest 3-D shell should start one step away")

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        candidates = root / "remaining.tsv"
        negative_plan = build_plan(_fixture_summary(next(iter(sorted(NEGATIVE)))), candidates)
        require(negative_plan["route"] == "LOCAL_3D_EXPANSION", "negative routing failed")
        lines = candidates.read_text().splitlines()
        require(len(lines) == REMAINING_3D_COUNT + 1, "3-D candidate file row count mismatch")
        require(lines[0].split("\t")[5] == str(REMAINING_3D_COUNT), "3-D META count mismatch")

        positive_plan = build_plan(_fixture_summary(POSITIVE), None)
        require(positive_plan["route"] == "STATIC_GAZEBO_NO_ACTUATION", "positive routing failed")
        require(positive_plan["actuation_authorized"] is False, "positive route accidentally authorized actuation")
        require(positive_plan["lift_allowed_in_next_phase"] is False, "positive route accidentally authorized lift")

        broken = _fixture_summary(POSITIVE)
        broken["safety"]["command_sent"] = True  # type: ignore[index]
        try:
            build_plan(broken, None)
        except ValueError:
            pass
        else:
            raise AssertionError("unsafe PR28 summary was accepted")

    print("prepare_rubiks_grasp_followup self-test: PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pr28-summary")
    parser.add_argument("--output")
    parser.add_argument("--candidates-output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0

    require(bool(args.pr28_summary), "--pr28-summary is required")
    require(bool(args.output), "--output is required")
    summary = json.loads(Path(args.pr28_summary).read_text())
    candidates_output = Path(args.candidates_output) if args.candidates_output else None
    plan = build_plan(summary, candidates_output)
    Path(args.output).write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n")
    print(json.dumps(plan, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
