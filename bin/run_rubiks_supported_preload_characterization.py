#!/usr/bin/env python3
"""Characterize supported Rubik contact under a bounded deeper gripper preload.

The cube remains on its static support for the entire trial. The gripper first
reaches the validated bilateral-contact state, then optionally closes to a
slightly smaller position target. No support removal or arm motion occurs.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

import rclpy

import run_rubiks_dynamic_opposing_side_hold_v2  # noqa: F401
from rubiks_contact_quality import audit_contact_quality
from run_gripper_gauge_calibration import speed
from run_rubiks_dynamic_supported_hold import (
    DynamicSupportedCubeNode,
    movement_metrics,
    ratio,
)

OPEN_M = 0.019


def finger_force_stats(events: list[dict], finger: str) -> dict[str, float | int | None]:
    values: list[float] = []
    token = f"gripper_{finger}_link"
    for event in events:
        for state in event.get("states", []):
            pair = f"{state.get('collision1', '')} {state.get('collision2', '')}"
            if token in pair:
                value = state.get("total_force_magnitude_n")
                if value is not None and math.isfinite(float(value)):
                    values.append(float(value))
    if not values:
        return {"sample_count": 0, "minimum_n": None, "median_n": None, "mean_n": None, "maximum_n": None}
    return {
        "sample_count": len(values),
        "minimum_n": min(values),
        "median_n": statistics.median(values),
        "mean_n": statistics.mean(values),
        "maximum_n": max(values),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cube-sdf", required=True)
    parser.add_argument("--support-sdf", required=True)
    parser.add_argument("--preload-target", type=float, required=True)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    parser.add_argument("--close-start", type=float, default=0.0185)
    parser.add_argument("--close-stop", type=float, default=0.0060)
    parser.add_argument("--close-step", type=float, default=0.00025)
    parser.add_argument("--step-hold", type=float, default=0.12)
    parser.add_argument("--max-effort", type=float, default=0.25)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    if not (0.0065 - 1e-12 <= args.preload_target <= 0.00725 + 1e-12):
        raise SystemExit("preload target outside declared 0.0065..0.00725 m domain")

    input_data = json.loads(Path(args.input).read_text(encoding="utf-8"))
    cube_xyz = tuple(float(v) for v in input_data["cube_effective_offset_link7_m"])
    support_xyz = tuple(float(v) for v in input_data["support_effective_offset_link7_m"])
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    robot_model = "turtlebot3_lime_gripper_test"
    cube_model = "rubiks_dynamic_cube_supported_057"
    support_model = "rubiks_cube_support_surface"

    rclpy.init(args=None)
    node = DynamicSupportedCubeNode(
        support_model=support_model,
        action_name="/gripper_controller/gripper_cmd",
        left_joint="gripper_left_joint",
        right_joint="gripper_right_joint",
        robot_model=robot_model,
        gauge_model=cube_model,
        left_link=f"{robot_model}::gripper_left_link",
        right_link=f"{robot_model}::gripper_right_link",
        gauge_link=f"{cube_model}::cube_link",
        contact_topic="/rubiks_dynamic_cube_hold/contact_states",
    )

    errors: list[str] = []
    events: list[dict] = []
    bilateral_target = None
    preload_event = None
    contact_quality = None
    hold_result = None
    force_result = None
    support_contact_before_close = False

    try:
        node.spin_for(1.0)
        if node.latest_joint is None or node.latest_link is None:
            raise RuntimeError("initial gripper telemetry unavailable")
        open_event = node.send_goal(
            label="preload_open_before_fixture",
            target=OPEN_M,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(open_event)
        if not open_event["reached_goal"] or open_event["stalled"]:
            raise RuntimeError("gripper failed to open")

        node.phase = "preload_spawn_support"
        node.spawn_named_model(
            model_name=support_model,
            sdf_text=Path(args.support_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_cube_support",
            reference_frame=f"{robot_model}::link7",
            pose_xyz=support_xyz,
            timeout=args.timeout,
        )
        node.phase = "preload_spawn_cube"
        node.spawn_named_model(
            model_name=cube_model,
            sdf_text=Path(args.cube_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_dynamic_cube_hold",
            reference_frame=f"{robot_model}::link7",
            pose_xyz=cube_xyz,
            timeout=args.timeout,
        )
        node.wait_for(lambda: node.latest_gauge_state is not None, args.timeout, "cube state")
        node.phase = "preload_settle"
        settle_start = node.now_s()
        node.spin_for(args.settle_seconds)
        settle_end = node.now_s()
        settled = dict(node.latest_gauge_state) if node.latest_gauge_state else None
        if settled is None:
            raise RuntimeError("settled cube state missing")
        settled_speed = speed(settled)
        if settled_speed is None or settled_speed > 0.02:
            raise RuntimeError(f"cube did not settle: {settled_speed}")
        support_contact_before_close = any(
            settle_start <= float(event["t_s"]) <= settle_end
            for event in node.support_contact_messages
        )
        if not support_contact_before_close:
            raise RuntimeError("support contact absent before close")
        if node.contact_messages or node.unexpected_robot_contacts:
            raise RuntimeError("robot-cube contact while gripper open")

        target = args.close_start
        step_index = 0
        bilateral_event = None
        while target >= args.close_stop - 1e-12:
            event = node.send_goal(
                label=f"opposing_side_close_step_preload_{step_index:02d}",
                target=target,
                max_effort=args.max_effort,
                timeout=args.timeout,
                monitor_contact=True,
            )
            events.append(event)
            if event["contact_observed"]:
                bilateral_event = event
                break
            if not event["reached_goal"] or event["stalled"]:
                raise RuntimeError(f"close step failed before bilateral contact: {target:.6f}")
            node.spin_for(args.step_hold)
            target -= args.close_step
            step_index += 1
        if bilateral_event is None:
            raise RuntimeError("bilateral contact not reached")
        bilateral_target = float(bilateral_event["target_position_m"])
        if args.preload_target > bilateral_target + 1e-12:
            raise RuntimeError(
                f"requested preload {args.preload_target} is more open than bilateral target {bilateral_target}"
            )

        if args.preload_target < bilateral_target - 1e-12:
            preload_event = node.send_goal(
                label="supported_preload_target",
                target=args.preload_target,
                max_effort=args.max_effort,
                timeout=args.timeout,
                monitor_contact=False,
            )
            events.append(preload_event)
            if not preload_event["reached_goal"] or preload_event["stalled"]:
                raise RuntimeError(f"preload target failed: {args.preload_target}")
        else:
            preload_event = bilateral_event

        node.phase = "supported_preload_hold"
        node.command_target_m = args.preload_target
        hold_start = node.now_s()
        node.spin_for(args.hold_seconds)
        hold_end = node.now_s()
        hold_contacts = [
            event for event in node.contact_messages
            if hold_start <= float(event["t_s"]) <= hold_end
        ]
        if not any(event.get("dual_contact") for event in hold_contacts):
            raise RuntimeError("bilateral contact absent during preload hold")
        if node.unexpected_robot_contacts:
            raise RuntimeError("non-finger robot-cube contact during preload hold")

        contact_quality = audit_contact_quality(
            hold_contacts,
            node.gauge_samples,
            start_s=hold_start,
            end_s=hold_end,
        )
        support_ratio = ratio(node.samples, hold_start, hold_end, "support_contact_active")
        dual_ratio = ratio(node.samples, hold_start, hold_end, "dual_finger_contact_active")
        hold_states = [
            state for state in node.gauge_samples
            if hold_start <= float(state["t_s"]) <= hold_end
        ]
        movement = movement_metrics(hold_states, settled)
        force_result = {
            "left": finger_force_stats(hold_contacts, "left"),
            "right": finger_force_stats(hold_contacts, "right"),
        }
        hold_result = {
            "start_s": hold_start,
            "end_s": hold_end,
            "observed_seconds": hold_end - hold_start,
            "support_contact_ratio": support_ratio,
            "bilateral_finger_contact_ratio": dual_ratio,
            "movement": movement,
            "left_joint_position_m": (
                node.latest_joint.get("left_joint_position_m") if node.latest_joint else None
            ),
            "physical_link_separation_m": (
                node.latest_link.get("separation_m") if node.latest_link else None
            ),
        }

        if contact_quality.get("passed") is not True:
            raise RuntimeError("preload contact quality/penetration gate failed")
        if support_ratio < 0.75 or dual_ratio < 0.65:
            raise RuntimeError(
                f"preload hold ratios failed: support={support_ratio}, dual={dual_ratio}"
            )
        if float(movement["max_horizontal_displacement_m"]) > 0.003:
            raise RuntimeError(f"preload cube horizontal movement too large: {movement}")
        if float(movement["max_abs_vertical_displacement_m"]) > 0.003:
            raise RuntimeError(f"preload cube vertical movement too large: {movement}")
        if float(movement["max_rotation_rad"]) > 0.10:
            raise RuntimeError(f"preload cube rotation too large: {movement}")
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        payload = {
            "schema_version": 1,
            "phase": "RUBIK-SUPPORTED-GRIPPER-PRELOAD-CHARACTERIZATION",
            "input": input_data,
            "requested_preload_target_m": args.preload_target,
            "bilateral_contact_target_m": bilateral_target,
            "preload_delta_from_bilateral_m": (
                None if bilateral_target is None else bilateral_target - args.preload_target
            ),
            "support_contact_before_close": support_contact_before_close,
            "contact_quality": contact_quality,
            "contact_force": force_result,
            "hold_result": hold_result,
            "support_removed": False,
            "arm_motion_performed": False,
            "arm_command_sent": False,
            "base_command_sent": False,
            "lift_command_sent": False,
            "attachment_used": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
            "grasp_success_claimed": False,
            "passed": not errors,
            "errors": errors,
            "events": events,
            "contact_messages": node.contact_messages,
            "support_contact_messages": node.support_contact_messages,
            "samples": node.samples,
            "cube_state_samples": node.gauge_samples,
        }
        output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps({
        k: v for k, v in payload.items()
        if k not in {"samples", "cube_state_samples", "contact_messages", "support_contact_messages"}
    }, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
