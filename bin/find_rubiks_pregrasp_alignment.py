#!/usr/bin/env python3
"""Deterministically search a command-free nonsingular Lime pregrasp posture."""
from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np

from rubiks_kinematic_preflight import (
    CASES,
    MAX_COND,
    condition_number,
    controller,
    forward_jacobian,
    model,
    solve,
    valid,
)

GRID_STEP_DEG = 2.5
GRID_LIMIT_DEG = 15.0
VARIED_JOINT_INDICES = (0, 1, 2, 4)
FIXED_JOINT_INDICES = (3, 5)
INTERPOLATION_SEGMENTS = 100


def rotation_distance(first: np.ndarray, second: np.ndarray) -> float:
    relative = first.T @ second
    cosine = max(-1.0, min(1.0, float((np.trace(relative) - 1.0) / 2.0)))
    return math.acos(cosine)


def initial_positions(interfaces: dict, names: list[str]) -> np.ndarray:
    positions = []
    for name in names:
        position_state = next(
            (entry for entry in interfaces[name]["state"] if entry["name"] == "position"),
            None,
        )
        if position_state is None or "initial_value" not in position_state["params"]:
            raise ValueError(f"{name} lacks a configured position initial_value")
        positions.append(float(position_state["params"]["initial_value"]))
    return np.array(positions, dtype=float)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--controller-yaml", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--states-tsv", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    urdf_path = Path(args.urdf)
    controller_path = Path(args.controller_yaml)
    output_path = Path(args.output)
    states_path = Path(args.states_tsv)

    chain, interfaces = model(urdf_path)
    names = [joint["name"] for joint in chain]
    controller_evidence = controller(controller_path, names)
    configured = initial_positions(interfaces, names)
    if not np.allclose(configured, np.zeros(6), rtol=0.0, atol=1e-12):
        raise SystemExit(f"configured evidence-bound state is not all-zero: {configured.tolist()}")
    if not valid(chain, configured):
        raise SystemExit("configured evidence-bound state violates effective limits")

    configured_frame, configured_jacobian = forward_jacobian(chain, configured)
    configured_condition = condition_number(configured_jacobian)
    if math.isfinite(configured_condition):
        raise SystemExit("configured evidence-bound state is unexpectedly nonsingular")

    grid_count_each_side = int(round(GRID_LIMIT_DEG / GRID_STEP_DEG))
    grid_values = [math.radians(index * GRID_STEP_DEG) for index in range(-grid_count_each_side, grid_count_each_side + 1)]
    candidate_count = 0
    finite_condition_count = 0
    correction_feasible_count = 0
    scored = []

    for varied_values in itertools.product(grid_values, repeat=len(VARIED_JOINT_INDICES)):
        positions = np.zeros(6, dtype=float)
        for index, value in zip(VARIED_JOINT_INDICES, varied_values):
            positions[index] = value
        candidate_count += 1
        if not valid(chain, positions):
            continue

        frame, jacobian = forward_jacobian(chain, positions)
        number = condition_number(jacobian)
        if not math.isfinite(number) or number > MAX_COND:
            continue
        finite_condition_count += 1

        cases = []
        all_cases_feasible = True
        for label, lateral, yaw in CASES:
            result = solve(chain, positions, lateral, yaw)
            result["label"] = label
            cases.append(result)
            if result["numerically_feasible"] is not True:
                all_cases_feasible = False
        if not all_cases_feasible:
            continue
        correction_feasible_count += 1

        translation = float(np.linalg.norm(frame[:3, 3] - configured_frame[:3, 3]))
        orientation = rotation_distance(configured_frame[:3, :3], frame[:3, :3])
        objective = (
            float(np.max(np.abs(positions))),
            float(np.linalg.norm(positions)),
            number,
            translation,
            orientation,
            tuple(float(value) for value in positions),
        )
        scored.append((objective, positions.copy(), frame.copy(), cases))

    if not scored:
        raise SystemExit("structured search found no numerically feasible pregrasp posture")

    objective, candidate, candidate_frame, cases = min(scored, key=lambda item: item[0])
    selected_condition = condition_number(forward_jacobian(chain, candidate)[1])

    rows: list[tuple[str, np.ndarray]] = [("configured_zero", configured)]
    for segment in range(1, INTERPOLATION_SEGMENTS + 1):
        fraction = segment / INTERPOLATION_SEGMENTS
        rows.append((f"transition_{segment:03d}", configured + fraction * (candidate - configured)))
    for result in cases:
        rows.append((f"correction_{result['label']}", np.array(result["preview_joint_positions_rad"], dtype=float)))

    states_path.write_text(
        "".join(
            label + "\t" + "\t".join(format(float(value), ".17g") for value in positions) + "\n"
            for label, positions in rows
        ),
        encoding="utf-8",
    )

    result = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT",
        "writer_lease": "WL-RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT-20260803-01",
        "source_ref": args.source_ref,
        "search_passed": True,
        "controller": controller_evidence,
        "configured_state": {
            "joint_positions_rad": dict(zip(names, configured.tolist())),
            "jacobian_condition_class": "infinite",
        },
        "search_coverage": {
            "method": "exhaustive_structured_subspace_grid",
            "grid_step_deg": GRID_STEP_DEG,
            "grid_limit_deg": GRID_LIMIT_DEG,
            "varied_joints": [names[index] for index in VARIED_JOINT_INDICES],
            "fixed_joints": [names[index] for index in FIXED_JOINT_INDICES],
            "candidate_count": candidate_count,
            "finite_condition_count": finite_condition_count,
            "correction_feasible_count": correction_feasible_count,
            "globally_nearest_claimed": False,
            "selection_order": [
                "minimum maximum absolute joint displacement",
                "minimum joint-space norm",
                "minimum condition number",
                "minimum link7 translation",
                "minimum link7 orientation change",
                "lexicographic joint vector",
            ],
        },
        "selected_candidate": {
            "joint_positions_rad": dict(zip(names, candidate.tolist())),
            "joint_positions_deg": dict(zip(names, np.degrees(candidate).tolist())),
            "maximum_absolute_joint_displacement_rad": objective[0],
            "joint_space_norm_rad": objective[1],
            "jacobian_condition_number": selected_condition,
            "link7_translation_from_configured_m": objective[3],
            "link7_orientation_change_rad": objective[4],
            "effective_limits_with_margin": valid(chain, candidate),
            "pre_fixture_posture_only": True,
        },
        "correction_cases": cases,
        "self_collision_state_matrix": {
            "path": states_path.name,
            "state_count": len(rows),
            "interpolation_segments": INTERPOLATION_SEGMENTS,
            "includes_configured_state": True,
            "includes_candidate": True,
            "includes_all_correction_endpoints": True,
        },
        "environment": {
            "cube_or_support_added": False,
            "environment_collision_checked": False,
            "verified_lateral_m": 0.0,
            "verified_yaw_rad": 0.0,
            "later_fixture_strategy": "respawn accepted cube and support transforms relative to link7 after pregrasp alignment",
        },
        "safety": {
            "offline_search_only": True,
            "globally_nearest_claimed": False,
            "trajectory_instantiated": False,
            "planning_request_created": False,
            "controller_loaded": False,
            "arm_command_sent": False,
            "gripper_command_sent": False,
            "gazebo_motion_performed": False,
            "actuation_authorized": False,
            "production_runtime_modified": False,
        },
    }
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
