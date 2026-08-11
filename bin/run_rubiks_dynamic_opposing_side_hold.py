#!/usr/bin/env python3
"""Gazebo supported close/hold/reopen gate for the selected opposing-side fixture.

This gate deliberately does not reuse the old scalar link-separation calibration as
an acceptance criterion. Contact quality is measured directly in the moving cube
frame from Gazebo contact positions/normals.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy

from rubiks_contact_quality import audit_contact_quality
from run_gripper_gauge_calibration import speed
from run_rubiks_dynamic_supported_hold import (
    DynamicSupportedCubeNode,
    movement_metrics,
    ratio,
    state_distance,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cube-sdf", required=True)
    parser.add_argument("--support-sdf", required=True)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--release-settle-seconds", type=float, default=1.5)
    parser.add_argument("--close-start", type=float, default=0.0185)
    parser.add_argument("--close-stop", type=float, default=0.0060)
    parser.add_argument("--close-step", type=float, default=0.00025)
    parser.add_argument("--step-hold", type=float, default=0.12)
    parser.add_argument("--max-effort", type=float, default=0.25)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    gate = json.loads(Path(args.input).read_text(encoding="utf-8"))
    cube_xyz = tuple(float(v) for v in gate["cube_effective_offset_link7_m"])
    support_xyz = tuple(float(v) for v in gate["support_effective_offset_link7_m"])
    if len(cube_xyz) != 3 or len(support_xyz) != 3:
        raise SystemExit("invalid fixture offsets")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    errors: list[str] = []
    events: list[dict] = []
    support_spawn_result = None
    cube_spawn_result = None
    initial_robot_state = None
    settled_cube_state = None
    contact_result = None
    contact_quality = None
    hold_result = None
    release_result = None
    support_contact_before_close = False
    contact_cleared_after_reopen = False
    hold_start = None
    hold_end = None

    robot_model = "turtlebot3_lime_gripper_test"
    cube_model = "rubiks_dynamic_cube_supported_057"
    support_model = "rubiks_cube_support_surface"
    node = None

    rclpy.init(args=None)
    try:
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
        node.spin_for(1.0)
        if node.latest_joint is None or node.latest_link is None:
            raise RuntimeError("initial gripper joint/link telemetry was not observed")
        initial_robot_state = node.latest_robot_state

        open_event = node.send_goal(
            label="opposing_side_open_before_fixture",
            target=0.019,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(open_event)
        if not open_event["reached_goal"] or open_event["stalled"]:
            raise RuntimeError("gripper did not reach fully open before fixture spawn")
        observed_left = open_event.get("observed_left_joint_position_m")
        separation = open_event.get("gazebo_link_frame_separation_m")
        if observed_left is None or abs(float(observed_left) - 0.019) > 0.002:
            raise RuntimeError(f"left gripper was not physically open: {observed_left}")
        if separation is None or float(separation) < 0.076:
            raise RuntimeError(f"physical finger separation too small while open: {separation}")

        node.phase = "spawn_opposing_side_support"
        support_spawn_result = node.spawn_named_model(
            model_name=support_model,
            sdf_text=Path(args.support_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_cube_support",
            reference_frame=f"{robot_model}::link7",
            pose_xyz=support_xyz,
            timeout=args.timeout,
        )
        node.phase = "spawn_opposing_side_cube"
        cube_spawn_result = node.spawn_named_model(
            model_name=cube_model,
            sdf_text=Path(args.cube_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_dynamic_cube_hold",
            reference_frame=f"{robot_model}::link7",
            pose_xyz=cube_xyz,
            timeout=args.timeout,
        )
        node.wait_for(lambda: node.latest_gauge_state is not None, args.timeout, "dynamic cube model state")

        node.phase = "opposing_side_settle_on_support"
        settle_start = node.now_s()
        node.spin_for(args.settle_seconds)
        settle_end = node.now_s()
        settled_cube_state = dict(node.latest_gauge_state) if node.latest_gauge_state else None
        if settled_cube_state is None:
            raise RuntimeError("settled dynamic cube state was not observed")
        support_contact_before_close = any(
            settle_start <= float(event["t_s"]) <= settle_end
            for event in node.support_contact_messages
        )
        if not support_contact_before_close:
            raise RuntimeError("cube did not establish support contact before close")
        settled_speed = speed(settled_cube_state)
        if settled_speed is None or settled_speed > 0.02:
            raise RuntimeError(f"cube did not settle on support: speed={settled_speed}")
        if node.contact_messages:
            raise RuntimeError("cube contacted finger while gripper was fully open")
        if node.unexpected_robot_contacts:
            raise RuntimeError("cube contacted non-finger robot collision while open")

        target = args.close_start
        step_index = 0
        contact_event = None
        while target >= args.close_stop - 1e-12:
            event = node.send_goal(
                label=f"opposing_side_close_step_{step_index:02d}",
                target=target,
                max_effort=args.max_effort,
                timeout=args.timeout,
                monitor_contact=True,
            )
            events.append(event)
            if event["contact_observed"]:
                contact_event = event
                break
            if not event["reached_goal"] or event["stalled"]:
                raise RuntimeError(f"close step {target:.6f} failed before cube contact")
            node.spin_for(args.step_hold)
            target -= args.close_step
            step_index += 1
        if contact_event is None:
            raise RuntimeError("no cube contact was observed in bounded close sweep")

        hold_start = node.now_s()
        node.phase = "opposing_side_supported_hold"
        node.command_target_m = contact_event["target_position_m"]
        node.spin_for(args.hold_seconds)
        hold_end = node.now_s()

        hold_contacts = [
            event for event in node.contact_messages
            if hold_start <= float(event["t_s"]) <= hold_end
        ]
        dual_contacts = [event for event in hold_contacts if event.get("dual_contact")]
        if not dual_contacts:
            raise RuntimeError("bilateral finger contact was not maintained during hold")
        if node.unexpected_robot_contacts:
            raise RuntimeError("dynamic cube contacted a non-finger robot collision")

        contact_quality = audit_contact_quality(
            hold_contacts,
            node.gauge_samples,
            start_s=hold_start,
            end_s=hold_end,
        )
        if contact_quality.get("passed") is not True:
            raise RuntimeError(
                "moving-cube opposing-side contact quality failed: "
                + json.dumps({k: contact_quality.get(k) for k in ("left", "right")}, sort_keys=True)
            )

        support_ratio = ratio(node.samples, hold_start, hold_end, "support_contact_active")
        dual_ratio = ratio(node.samples, hold_start, hold_end, "dual_finger_contact_active")
        hold_states = [state for state in node.gauge_samples if hold_start <= float(state["t_s"]) <= hold_end]
        movement = movement_metrics(hold_states, settled_cube_state)
        if support_ratio < 0.75:
            raise RuntimeError(f"support contact coverage too low during hold: {support_ratio}")
        if dual_ratio < 0.65:
            raise RuntimeError(f"bilateral finger coverage too low during hold: {dual_ratio}")
        if float(movement["max_horizontal_displacement_m"]) > 0.003:
            raise RuntimeError(f"cube slid too far during hold: {movement}")
        if float(movement["max_vertical_rise_m"]) > 0.0015:
            raise RuntimeError(f"cube rose from support during no-lift hold: {movement}")
        if float(movement["max_rotation_rad"]) > 0.10:
            raise RuntimeError(f"cube rotated too far during hold: {movement}")

        contact_result = {
            "close_contact_target_m": float(contact_event["target_position_m"]),
            "dual_contact_message_count": len(dual_contacts),
            "direct_cube_frame_quality_used": True,
            "legacy_scalar_opening_gate_used": False,
            "legacy_scalar_opening_note": (
                "The old 3 mm gauge calibration is not transferable across the sloped STL contact region; "
                "acceptance uses moving-cube contact positions/normals instead."
            ),
        }
        hold_result = {
            "hold_start_s": hold_start,
            "hold_end_s": hold_end,
            "requested_hold_seconds": args.hold_seconds,
            "observed_hold_seconds": hold_end - hold_start,
            "support_contact_ratio": support_ratio,
            "bilateral_finger_contact_ratio": dual_ratio,
            "movement": movement,
        }

        reopen_started = node.now_s()
        reopen_event = node.send_goal(
            label="opposing_side_reopen_after_hold",
            target=0.019,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(reopen_event)
        if not reopen_event["reached_goal"] or reopen_event["stalled"]:
            raise RuntimeError("gripper did not reopen after hold")
        node.phase = "opposing_side_release_settle"
        node.command_target_m = None
        release_start = node.now_s()
        node.spin_for(args.release_settle_seconds)
        release_end = node.now_s()

        late_finger_contacts = [
            event for event in node.contact_messages
            if float(event["t_s"]) >= reopen_started + 0.30
        ]
        contact_cleared_after_reopen = not late_finger_contacts
        if not contact_cleared_after_reopen:
            raise RuntimeError("finger contact did not clear after reopen")
        release_support_ratio = ratio(node.samples, release_start, release_end, "support_contact_active")
        final_cube_state = node.latest_gauge_state
        final_cube_speed = speed(final_cube_state)
        final_offset = state_distance(settled_cube_state, final_cube_state)
        if release_support_ratio < 0.75:
            raise RuntimeError(f"support contact did not persist after release: {release_support_ratio}")
        if final_cube_speed is None or final_cube_speed > 0.02:
            raise RuntimeError(f"cube did not settle after release: speed={final_cube_speed}")
        if float(final_offset["horizontal_displacement_m"]) > 0.004:
            raise RuntimeError(f"final horizontal offset too large: {final_offset}")
        if abs(float(final_offset["vertical_displacement_m"])) > 0.002:
            raise RuntimeError(f"final vertical offset too large: {final_offset}")
        if float(final_offset["rotation_rad"]) > 0.12:
            raise RuntimeError(f"final rotation too large: {final_offset}")
        release_result = {
            "release_start_s": release_start,
            "release_end_s": release_end,
            "support_contact_ratio": release_support_ratio,
            "final_cube_linear_speed_m_s": final_cube_speed,
            "final_offset_from_preclose_settled_state": final_offset,
        }
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        if node is not None:
            final_robot_state = node.latest_robot_state
            final_cube_state = node.latest_gauge_state
            max_vertical_rise = (
                hold_result["movement"]["max_vertical_rise_m"] if hold_result else None
            )
            payload = {
                "schema_version": 1,
                "phase": "RUBIK-OPPOSING-SIDE-DYNAMIC-SUPPORTED-HOLD",
                "input": gate,
                "cube_model": cube_model,
                "support_model": support_model,
                "cube_dynamic": True,
                "cube_mass_kg": 0.09,
                "support_static": True,
                "support_spawn_result": support_spawn_result,
                "cube_spawn_result": cube_spawn_result,
                "support_contact_before_close": support_contact_before_close,
                "contact_cleared_after_reopen": contact_cleared_after_reopen,
                "contact_result": contact_result,
                "contact_quality": contact_quality,
                "hold_result": hold_result,
                "release_result": release_result,
                "arm_motion_performed": False,
                "base_motion_commanded": False,
                "lift_command_sent": False,
                "cube_lifted": bool(max_vertical_rise is not None and float(max_vertical_rise) > 0.0015),
                "ifra_attachment_used": False,
                "grasp_success_claimed": False,
                "passed": not errors,
                "errors": errors,
                "events": events,
                "contact_messages": node.contact_messages,
                "support_contact_messages": node.support_contact_messages,
                "samples": node.samples,
                "cube_state_samples": node.gauge_samples,
                "initial_robot_state": initial_robot_state,
                "final_robot_state": final_robot_state,
                "settled_cube_state": settled_cube_state,
                "final_cube_state": final_cube_state,
            }
            output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
            node.destroy_node()
        else:
            payload = {"schema_version":1,"phase":"RUBIK-OPPOSING-SIDE-DYNAMIC-SUPPORTED-HOLD","passed":False,"errors":errors}
            output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        rclpy.shutdown()

    print(json.dumps({k:v for k,v in payload.items() if k not in {"samples","cube_state_samples","contact_messages","support_contact_messages"}}, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
