#!/usr/bin/env python3
"""Bind measured gripper states to exact ground-audit root/arm states."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
GRIPPER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]
GRIPPER_LINKS = ["gripper_left_link", "gripper_right_link"]
RIGHT_SOURCE_ALIAS = "gripper_right_joint_mimic"
RIGHT_ALIAS_TO_MODEL_MULTIPLIER = -1.0
MAX_ARM_MATCH_ERROR_RAD = 1e-9
MAX_MIMIC_POSITION_RESIDUAL_M = 1e-6
MAX_MIMIC_VELOCITY_RESIDUAL_M_S = 1e-5


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def parse_interface(element: ET.Element) -> dict[str, Any]:
    params: dict[str, float | str] = {}
    for parameter in element.findall("param"):
        name = parameter.get("name")
        text = (parameter.text or "").strip()
        if not name or not text:
            continue
        try:
            params[name] = float(text)
        except ValueError:
            params[name] = text
    return {"name": element.get("name"), "params": params}


def urdf_contract(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    joints = {str(joint.get("name")): joint for joint in root.findall("joint")}
    controls: dict[str, dict[str, Any]] = {}
    for control in root.findall("ros2_control"):
        for joint in control.findall("joint"):
            name = str(joint.get("name"))
            controls[name] = {
                "parameters": {
                    str(param.get("name")): (param.text or "").strip()
                    for param in joint.findall("param")
                    if param.get("name")
                },
                "command": [
                    parse_interface(item) for item in joint.findall("command_interface")
                ],
                "state": [
                    parse_interface(item) for item in joint.findall("state_interface")
                ],
            }

    result: dict[str, Any] = {
        "robot_name": root.get("name"),
        "gripper_joint_order": GRIPPER_JOINTS,
        "gripper_links": GRIPPER_LINKS,
        "joints": {},
        "collision_geometry_sha256": {},
        "missing_collision_geometry": [],
    }
    for name in GRIPPER_JOINTS:
        joint = joints.get(name)
        require(joint is not None, f"{name} is missing")
        require(joint.get("type") == "prismatic", f"{name} is not prismatic")
        parent = joint.find("parent")
        child = joint.find("child")
        limit = joint.find("limit")
        axis = joint.find("axis")
        require(
            parent is not None and parent.get("link") == "link7",
            f"{name} parent mismatch",
        )
        require(child is not None and child.get("link"), f"{name} child missing")
        require(
            limit is not None and limit.get("lower") and limit.get("upper"),
            f"{name} limits missing",
        )
        urdf_lower = float(limit.get("lower"))
        urdf_upper = float(limit.get("upper"))
        axis_values = [
            float(value)
            for value in (axis.get("xyz") if axis is not None else "").split()
        ]
        require(
            len(axis_values) == 3 and all(math.isfinite(value) for value in axis_values),
            f"{name} axis invalid",
        )

        control = controls.get(name)
        require(control is not None, f"{name} ros2_control declaration missing")
        command = next(
            (item for item in control["command"] if item["name"] == "position"),
            None,
        )
        require(command is not None, f"{name} position command interface missing")
        control_lower = command["params"].get("min")
        control_upper = command["params"].get("max")
        require(
            isinstance(control_lower, float) and isinstance(control_upper, float),
            f"{name} control limits missing",
        )
        state_names = [item["name"] for item in control["state"]]
        require(
            "position" in state_names and "velocity" in state_names,
            f"{name} state interfaces incomplete",
        )
        effective_lower = max(urdf_lower, float(control_lower))
        effective_upper = min(urdf_upper, float(control_upper))
        require(effective_lower <= effective_upper, f"{name} limit intersection empty")

        mimic = joint.find("mimic")
        mimic_evidence = None
        if mimic is not None:
            mimic_evidence = {
                "joint": mimic.get("joint"),
                "multiplier": float(mimic.get("multiplier", "1")),
                "offset": float(mimic.get("offset", "0")),
            }
        result["joints"][name] = {
            "type": "prismatic",
            "parent": parent.get("link"),
            "child": child.get("link"),
            "axis": axis_values,
            "urdf_lower_m": urdf_lower,
            "urdf_upper_m": urdf_upper,
            "control_lower_m": float(control_lower),
            "control_upper_m": float(control_upper),
            "effective_lower_m": effective_lower,
            "effective_upper_m": effective_upper,
            "state_interfaces": state_names,
            "mimic": mimic_evidence,
            "ros2_control_parameters": control["parameters"],
        }

    left = result["joints"]["gripper_left_joint"]
    right = result["joints"]["gripper_right_joint"]
    require(left["axis"] == [0.0, 1.0, 0.0], "left gripper axis mismatch")
    require(right["axis"] == [0.0, -1.0, 0.0], "right gripper axis mismatch")
    require(
        right["mimic"]
        == {"joint": "gripper_left_joint", "multiplier": 1.0, "offset": 0.0},
        "right mimic contract mismatch",
    )
    require(
        right["ros2_control_parameters"].get("mimic") == "gripper_left_joint",
        "right ros2_control mimic source mismatch",
    )
    require(
        float(right["ros2_control_parameters"].get("multiplier", "nan")) == 1.0,
        "right ros2_control mimic multiplier mismatch",
    )
    result["right_hardware_alias_binding"] = {
        "source_joint_name": RIGHT_SOURCE_ALIAS,
        "model_joint_name": "gripper_right_joint",
        "source_to_model_position_multiplier": RIGHT_ALIAS_TO_MODEL_MULTIPLIER,
        "source_to_model_velocity_multiplier": RIGHT_ALIAS_TO_MODEL_MULTIPLIER,
        "derivation": "exact_opposed_urdf_axes_and_legacy_ros2_control_mimic_suffix",
        "left_axis": left["axis"],
        "right_axis": right["axis"],
        "model_mimic_multiplier": right["mimic"]["multiplier"],
    }

    links = {str(link.get("name")): link for link in root.findall("link")}
    for name in GRIPPER_LINKS:
        link = links.get(name)
        collisions = [] if link is None else link.findall("collision")
        if not collisions:
            result["missing_collision_geometry"].append(name)
            continue
        data = b"".join(
            ET.tostring(element, encoding="utf-8") for element in collisions
        )
        result["collision_geometry_sha256"][name] = digest_bytes(data)
    return result


def read_ground_states(path: Path) -> list[dict[str, Any]]:
    states = []
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        require(len(fields) == 14, f"invalid ground-state row: {raw}")
        values = np.asarray([float(value) for value in fields[1:]], dtype=float)
        require(
            values.shape == (13,) and np.isfinite(values).all(),
            f"invalid ground-state values: {fields[0]}",
        )
        states.append({"label": fields[0], "values": values})
    require(states, "ground-state matrix is empty")
    return states


def match_samples(
    simulation: dict[str, Any],
    trace: dict[str, Any],
    ground_states: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    simulation_samples = simulation.get("evidence", {}).get("joint_state_samples", [])
    observer_samples = trace.get("samples", [])
    require(
        len(simulation_samples) == len(ground_states) and len(simulation_samples) >= 100,
        "upstream state count mismatch",
    )
    by_stamp: dict[int, list[dict[str, Any]]] = {}
    for sample in observer_samples:
        by_stamp.setdefault(int(sample.get("ros_stamp_ns", -1)), []).append(sample)

    used: set[int] = set()
    rows = []
    arm_errors = []
    left_values = []
    raw_right_values = []
    model_right_values = []
    left_velocities = []
    raw_right_velocities = []
    model_right_velocities = []
    mimic_positions = []
    mimic_velocities = []
    raw_alias_position_residuals = []
    raw_alias_velocity_residuals = []
    source_topics: dict[str, int] = {}
    missing_velocity_count = 0

    for index, (simulation_sample, ground_state) in enumerate(
        zip(simulation_samples, ground_states)
    ):
        require(
            ground_state["label"] == f"sample_{index:06d}",
            f"ground-state label mismatch at {index}",
        )
        stamp = int(simulation_sample.get("ros_stamp_ns", -1))
        arm = np.asarray(simulation_sample.get("positions_rad"), dtype=float)
        require(
            arm.shape == (6,) and np.isfinite(arm).all(),
            f"invalid simulation arm sample {index}",
        )
        require(
            np.max(np.abs(ground_state["values"][7:] - arm))
            <= MAX_ARM_MATCH_ERROR_RAD,
            f"ground/arm mismatch at {index}",
        )
        candidates = []
        for candidate in by_stamp.get(stamp, []):
            if id(candidate) in used:
                continue
            observed_arm = np.asarray(candidate.get("arm_positions_rad"), dtype=float)
            if observed_arm.shape == (6,) and np.isfinite(observed_arm).all():
                candidates.append(
                    (float(np.max(np.abs(observed_arm - arm))), candidate)
                )
        require(candidates, f"no gripper observation match for sample {index}")
        arm_error, candidate = min(candidates, key=lambda item: item[0])
        require(
            arm_error <= MAX_ARM_MATCH_ERROR_RAD,
            f"arm match error at {index}: {arm_error}",
        )
        require(
            candidate.get("gripper_source_joint_names")
            == ["gripper_left_joint", RIGHT_SOURCE_ALIAS],
            f"gripper source alias mismatch at {index}",
        )
        require(
            candidate.get("right_source_is_ros2_control_mimic_alias") is True,
            f"right alias flag missing at {index}",
        )
        used.add(id(candidate))
        arm_errors.append(arm_error)
        source_topic = str(candidate.get("source_topic"))
        source_topics[source_topic] = source_topics.get(source_topic, 0) + 1

        raw_gripper = np.asarray(candidate.get("gripper_positions_m"), dtype=float)
        require(
            raw_gripper.shape == (2,) and np.isfinite(raw_gripper).all(),
            f"invalid gripper positions at {index}",
        )
        raw_velocity = candidate.get("gripper_velocities_m_s")
        if raw_velocity is None:
            missing_velocity_count += 1
            gripper_velocity = np.asarray([math.nan, math.nan])
        else:
            gripper_velocity = np.asarray(raw_velocity, dtype=float)
            require(
                gripper_velocity.shape == (2,)
                and np.isfinite(gripper_velocity).all(),
                f"invalid gripper velocity at {index}",
            )

        left = float(raw_gripper[0])
        raw_right = float(raw_gripper[1])
        model_right = RIGHT_ALIAS_TO_MODEL_MULTIPLIER * raw_right
        left_values.append(left)
        raw_right_values.append(raw_right)
        model_right_values.append(model_right)
        raw_alias_position_residuals.append(abs(raw_right + left))
        mimic_positions.append(abs(model_right - left))

        if raw_velocity is not None:
            left_velocity = float(gripper_velocity[0])
            raw_right_velocity = float(gripper_velocity[1])
            model_right_velocity = (
                RIGHT_ALIAS_TO_MODEL_MULTIPLIER * raw_right_velocity
            )
            left_velocities.append(left_velocity)
            raw_right_velocities.append(raw_right_velocity)
            model_right_velocities.append(model_right_velocity)
            raw_alias_velocity_residuals.append(
                abs(raw_right_velocity + left_velocity)
            )
            mimic_velocities.append(abs(model_right_velocity - left_velocity))

        rows.append(
            {
                "label": ground_state["label"],
                "ground_values": ground_state["values"].tolist(),
                "raw_gripper_source_positions_m": [left, raw_right],
                "gripper_positions_m": [left, model_right],
            }
        )

    require(
        missing_velocity_count == 0,
        f"{missing_velocity_count} matched samples lack gripper velocity",
    )
    require(
        max(raw_alias_position_residuals) <= MAX_MIMIC_POSITION_RESIDUAL_M,
        "raw right alias anti-symmetry residual exceeded",
    )
    require(
        max(raw_alias_velocity_residuals) <= MAX_MIMIC_VELOCITY_RESIDUAL_M_S,
        "raw right alias velocity anti-symmetry residual exceeded",
    )
    require(
        max(mimic_positions) <= MAX_MIMIC_POSITION_RESIDUAL_M,
        "normalized gripper mimic position residual exceeded",
    )
    require(
        max(mimic_velocities) <= MAX_MIMIC_VELOCITY_RESIDUAL_M_S,
        "normalized gripper mimic velocity residual exceeded",
    )
    return rows, {
        "simulation_sample_count": len(simulation_samples),
        "observer_sample_count": len(observer_samples),
        "matched_sample_count": len(rows),
        "all_samples_matched": len(rows) == len(simulation_samples),
        "matched_source_topic_counts": source_topics,
        "maximum_arm_match_error_rad": max(arm_errors),
        "missing_velocity_count": missing_velocity_count,
        "right_source_joint_name": RIGHT_SOURCE_ALIAS,
        "right_source_to_model_multiplier": RIGHT_ALIAS_TO_MODEL_MULTIPLIER,
        "left_position_min_m": min(left_values),
        "left_position_max_m": max(left_values),
        "left_position_range_m": max(left_values) - min(left_values),
        "raw_right_alias_position_min_m": min(raw_right_values),
        "raw_right_alias_position_max_m": max(raw_right_values),
        "model_right_position_min_m": min(model_right_values),
        "model_right_position_max_m": max(model_right_values),
        "model_right_position_range_m": max(model_right_values)
        - min(model_right_values),
        "maximum_abs_left_velocity_m_s": max(
            abs(value) for value in left_velocities
        ),
        "maximum_abs_raw_right_alias_velocity_m_s": max(
            abs(value) for value in raw_right_velocities
        ),
        "maximum_abs_model_right_velocity_m_s": max(
            abs(value) for value in model_right_velocities
        ),
        "maximum_raw_alias_position_antisymmetry_residual_m": max(
            raw_alias_position_residuals
        ),
        "maximum_raw_alias_velocity_antisymmetry_residual_m_s": max(
            raw_alias_velocity_residuals
        ),
        "maximum_mimic_position_residual_m": max(mimic_positions),
        "maximum_mimic_velocity_residual_m_s": max(mimic_velocities),
        "maximum_allowed_mimic_position_residual_m": (
            MAX_MIMIC_POSITION_RESIDUAL_M
        ),
        "maximum_allowed_mimic_velocity_residual_m_s": (
            MAX_MIMIC_VELOCITY_RESIDUAL_M_S
        ),
    }


def write_states(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            values = row["ground_values"] + row["gripper_positions_m"]
            stream.write(
                row["label"]
                + "\t"
                + "\t".join(f"{value:.17g}" for value in values)
                + "\n"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation-summary", required=True)
    parser.add_argument("--gripper-trace", required=True)
    parser.add_argument("--ground-states", required=True)
    parser.add_argument("--simulation-urdf", required=True)
    parser.add_argument("--moveit-urdf", required=True)
    parser.add_argument("--output-states", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    simulation = json.loads(Path(args.simulation_summary).read_text())
    trace = json.loads(Path(args.gripper_trace).read_text())
    require(
        simulation.get("source_ref") == args.source_ref,
        "simulation source_ref mismatch",
    )
    require(
        trace.get("source_ref") == args.source_ref,
        "gripper trace source_ref mismatch",
    )
    require(
        simulation.get("passed") is True and trace.get("passed") is True,
        "upstream evidence failed",
    )
    require(simulation.get("joint_order") == ARM_JOINTS, "simulation arm order mismatch")
    require(trace.get("arm_joint_order") == ARM_JOINTS, "observer arm order mismatch")
    require(
        trace.get("gripper_joint_order") == GRIPPER_JOINTS,
        "observer gripper order mismatch",
    )
    require(
        trace.get("observed_gripper_source_joint_name_sets")
        == [["gripper_left_joint", RIGHT_SOURCE_ALIAS]],
        "observer gripper source alias mismatch",
    )
    require(trace.get("errors") == [], "gripper observer contains errors")
    safety = trace.get("safety", {})
    require(
        safety.get("read_only_subscription_only") is True,
        "gripper observer is not read-only",
    )
    for key in (
        "publisher_created",
        "service_client_created",
        "action_client_created",
        "controller_loaded",
        "command_sent",
    ):
        require(safety.get(key) is False, f"observer {key} must be false")
    simulation_safety = simulation.get("safety", {})
    require(
        simulation_safety.get("gripper_controller_loaded") is False,
        "gripper controller was loaded",
    )
    require(
        simulation_safety.get("gripper_command_sent") is False,
        "gripper command was sent",
    )

    simulation_contract = urdf_contract(Path(args.simulation_urdf))
    moveit_contract = urdf_contract(Path(args.moveit_urdf))
    require(
        simulation_contract["robot_name"] == "turtlebot3_lime_arm_motion_test",
        "simulation URDF name mismatch",
    )
    require(
        moveit_contract["robot_name"] == "turtlebot3_lime",
        "MoveIt URDF name mismatch",
    )
    require(
        not simulation_contract["missing_collision_geometry"],
        "simulation gripper geometry missing",
    )
    require(
        not moveit_contract["missing_collision_geometry"],
        "MoveIt gripper geometry missing",
    )
    require(
        simulation_contract["collision_geometry_sha256"]
        == moveit_contract["collision_geometry_sha256"],
        "gripper collision geometry mismatch",
    )
    require(
        simulation_contract["joints"] == moveit_contract["joints"],
        "gripper joint contract mismatch",
    )
    require(
        simulation_contract["right_hardware_alias_binding"]
        == moveit_contract["right_hardware_alias_binding"],
        "right hardware alias binding mismatch",
    )

    ground_states = read_ground_states(Path(args.ground_states))
    rows, evidence = match_samples(simulation, trace, ground_states)
    limits = simulation_contract["joints"]
    for row in rows:
        for name, value in zip(GRIPPER_JOINTS, row["gripper_positions_m"]):
            contract = limits[name]
            require(
                contract["effective_lower_m"]
                <= value
                <= contract["effective_upper_m"],
                f"{name} effective limit violation",
            )
    write_states(Path(args.output_states), rows)

    payload = {
        "schema_version": 2,
        "phase": "RUBIK-PREGRASP-GRIPPER-STATE-BINDING",
        "source_ref": args.source_ref,
        "passed": True,
        "errors": [],
        "state_count": len(rows),
        "joint_contract": simulation_contract,
        "simulation_moveit_joint_contract_exact_match": True,
        "simulation_moveit_collision_geometry_exact_match": True,
        "right_hardware_alias_binding_exact_match": True,
        "measured_state_evidence": evidence,
        "output_states_tsv": Path(args.output_states).name,
        "safety": {
            "read_only_observation_only": True,
            "coordinate_normalization_only": True,
            "raw_measurements_preserved_in_evidence": True,
            "gripper_controller_loaded": False,
            "gripper_command_sent": False,
            "new_command_path_added": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
        },
    }
    Path(args.output).write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
