#!/usr/bin/env python3
"""Run the selected Rubik fixture through open arm travel and supported hold.

Sequence: exact anchor/open -> spawn supported cube -> open arm anchor->zero->anchor
with the fixture physically present -> bounded close through unilateral precursor
contact -> bilateral 2 s hold -> reopen. No lift/base/attachment is used.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

# Import for the validated side effects used by PR #45: first-observation
# readiness is 3 s, and bounded close labels beginning opposing_side_close_step_
# treat unilateral contact only as a precursor until bilateral contact.
import run_rubiks_dynamic_opposing_side_hold_v2  # noqa: F401
from rubiks_contact_quality import audit_contact_quality
from run_gripper_gauge_calibration import speed
from run_rubiks_dynamic_supported_hold import (
    DynamicSupportedCubeNode,
    movement_metrics,
    ratio,
    state_distance,
)

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
OPEN_M = 0.019


class IntegratedNode(DynamicSupportedCubeNode):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.arm_joint_latest: dict[str, float] = {}
        self.arm_motion_windows: list[dict[str, object]] = []

    def on_joint_state(self, msg) -> None:
        for index, name in enumerate(msg.name):
            if name in ARM_JOINTS and index < len(msg.position):
                value = float(msg.position[index])
                if math.isfinite(value):
                    self.arm_joint_latest[name] = value
        super().on_joint_state(msg)

    def move_open_arm(
        self, label: str, target: list[float], seconds: float, timeout: float
    ) -> dict[str, object]:
        if len(target) != 6 or not all(math.isfinite(v) for v in target):
            raise RuntimeError("invalid arm target")
        if not self.arm_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("arm action server unavailable")

        before_contacts = len(self.contact_messages)
        before_unexpected = len(self.unexpected_robot_contacts)
        start = self.now_s()
        self.phase = label

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        whole = int(seconds)
        point.time_from_start.sec = whole
        point.time_from_start.nanosec = int(round((seconds - whole) * 1e9))
        goal.trajectory.points = [point]

        send_future = self.arm_client.send_goal_async(goal)
        self.wait_for(lambda: send_future.done(), timeout, f"{label} acceptance")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label} goal rejected")
        result_future = handle.get_result_async()
        self.wait_for(
            lambda: result_future.done(), timeout + seconds, f"{label} result"
        )
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{label} action failed: status={getattr(wrapped, 'status', None)}"
            )
        self.spin_for(0.6)
        end = self.now_s()

        measured = [self.arm_joint_latest.get(name) for name in ARM_JOINTS]
        if any(value is None for value in measured):
            raise RuntimeError(f"{label} final arm state missing")
        errors = [float(measured[i]) - target[i] for i in range(6)]
        max_error = max(abs(v) for v in errors)
        if max_error > 0.01:
            raise RuntimeError(f"{label} arm error too large: {errors}")

        left = self.latest_joint.get("left_joint_position_m") if self.latest_joint else None
        separation = self.latest_link.get("separation_m") if self.latest_link else None
        if left is None or abs(float(left) - OPEN_M) > 0.002:
            raise RuntimeError(f"{label}: left gripper not open: {left}")
        if separation is None or float(separation) < 0.076:
            raise RuntimeError(f"{label}: physical opening too small: {separation}")

        finger_events = self.contact_messages[before_contacts:]
        unexpected = self.unexpected_robot_contacts[before_unexpected:]
        if finger_events:
            raise RuntimeError(f"{label}: premature finger-cube contact while open")
        if unexpected:
            raise RuntimeError(f"{label}: non-finger robot-cube contact")

        window = {
            "label": label,
            "start_s": start,
            "end_s": end,
            "duration_s": end - start,
            "target_rad": list(target),
            "measured_rad": [float(v) for v in measured],
            "max_abs_error_rad": max_error,
            "left_gripper_m": float(left),
            "physical_link_separation_m": float(separation),
            "finger_cube_contact_count": len(finger_events),
            "nonfinger_robot_cube_contact_count": len(unexpected),
        }
        self.arm_motion_windows.append(window)
        return window


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cube-sdf", required=True)
    parser.add_argument("--support-sdf", required=True)
    parser.add_argument("--arm-seconds", type=float, default=3.0)
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

    input_data = json.loads(Path(args.input).read_text(encoding="utf-8"))
    anchor = [float(v) for v in input_data["arm_target_rad"]]
    cube_xyz = tuple(float(v) for v in input_data["cube_effective_offset_link7_m"])
    support_xyz = tuple(float(v) for v in input_data["support_effective_offset_link7_m"])
    if len(anchor) != 6 or len(cube_xyz) != 3 or len(support_xyz) != 3:
        raise SystemExit("invalid integrated input geometry")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    robot_model = "turtlebot3_lime_gripper_test"
    cube_model = "rubiks_dynamic_cube_supported_057"
    support_model = "rubiks_cube_support_surface"

    rclpy.init(args=None)
    node = IntegratedNode(
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
    events: list[dict[str, object]] = []
    settled_cube_state = None
    preclose_cube_state = None
    open_cycle_start = None
    open_cycle_end = None
    open_cycle_support_ratio = None
    approach_fixture_movement = None
    contact_result = None
    contact_quality = None
    hold_result = None
    release_result = None
    support_contact_before_open_cycle = False
    contact_cleared_after_reopen = False

    try:
        node.spin_for(1.0)
        if node.latest_joint is None or node.latest_link is None:
            raise RuntimeError("initial gripper telemetry unavailable")

        open_event = node.send_goal(
            label="integrated_open_at_anchor",
            target=OPEN_M,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(open_event)
        if not open_event["reached_goal"] or open_event["stalled"]:
            raise RuntimeError("failed to establish fully-open anchor state")

        node.phase = "integrated_spawn_support"
        node.spawn_named_model(
            model_name=support_model,
            sdf_text=Path(args.support_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_cube_support",
            reference_frame=f"{robot_model}::link7",
            pose_xyz=support_xyz,
            timeout=args.timeout,
        )
        node.phase = "integrated_spawn_cube"
        node.spawn_named_model(
            model_name=cube_model,
            sdf_text=Path(args.cube_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_dynamic_cube_hold",
            reference_frame=f"{robot_model}::link7",
            pose_xyz=cube_xyz,
            timeout=args.timeout,
        )
        node.wait_for(lambda: node.latest_gauge_state is not None, args.timeout, "cube state")

        node.phase = "integrated_anchor_settle"
        settle_start = node.now_s()
        node.spin_for(args.settle_seconds)
        settle_end = node.now_s()
        settled_cube_state = dict(node.latest_gauge_state) if node.latest_gauge_state else None
        if settled_cube_state is None:
            raise RuntimeError("cube state missing after anchor settle")
        settled_speed = speed(settled_cube_state)
        if settled_speed is None or settled_speed > 0.02:
            raise RuntimeError(f"cube did not settle at anchor: {settled_speed}")
        support_contact_before_open_cycle = any(
            settle_start <= float(event["t_s"]) <= settle_end
            for event in node.support_contact_messages
        )
        if not support_contact_before_open_cycle:
            raise RuntimeError("support contact absent before open arm cycle")
        if node.contact_messages or node.unexpected_robot_contacts:
            raise RuntimeError("robot-cube contact exists before open arm cycle")

        open_cycle_start = node.now_s()
        zero_window = node.move_open_arm(
            "integrated_open_return_zero", [0.0] * 6, args.arm_seconds, args.timeout
        )
        events.append(zero_window)
        node.phase = "integrated_zero_hold"
        node.spin_for(0.8)
        if node.contact_messages or node.unexpected_robot_contacts:
            raise RuntimeError("robot-cube contact observed at open zero hold")

        approach_window = node.move_open_arm(
            "integrated_open_approach_anchor", anchor, args.arm_seconds, args.timeout
        )
        events.append(approach_window)
        node.phase = "integrated_preclose_settle"
        node.spin_for(0.8)
        open_cycle_end = node.now_s()
        if node.contact_messages or node.unexpected_robot_contacts:
            raise RuntimeError("premature robot-cube contact after open approach")

        open_cycle_support_ratio = ratio(
            node.samples, open_cycle_start, open_cycle_end, "support_contact_active"
        )
        if open_cycle_support_ratio < 0.75:
            raise RuntimeError(
                f"support contact too low during open arm cycle: {open_cycle_support_ratio}"
            )
        preclose_cube_state = dict(node.latest_gauge_state) if node.latest_gauge_state else None
        if preclose_cube_state is None:
            raise RuntimeError("cube state missing before close")
        approach_fixture_movement = state_distance(settled_cube_state, preclose_cube_state)
        if float(approach_fixture_movement["horizontal_displacement_m"]) > 0.003:
            raise RuntimeError(f"cube moved too far during open arm cycle: {approach_fixture_movement}")
        if abs(float(approach_fixture_movement["vertical_displacement_m"])) > 0.002:
            raise RuntimeError(f"cube vertical drift too large: {approach_fixture_movement}")
        if float(approach_fixture_movement["rotation_rad"]) > 0.10:
            raise RuntimeError(f"cube rotation too large: {approach_fixture_movement}")

        target = args.close_start
        step_index = 0
        contact_event = None
        while target >= args.close_stop - 1e-12:
            event = node.send_goal(
                # Prefix is required by the validated PR #45 unilateral-precursor policy.
                label=f"opposing_side_close_step_integrated_{step_index:02d}",
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
                raise RuntimeError(f"close step failed before bilateral contact: {target:.6f}")
            node.spin_for(args.step_hold)
            target -= args.close_step
            step_index += 1
        if contact_event is None:
            raise RuntimeError("no bilateral cube contact in bounded close")

        hold_start = node.now_s()
        node.phase = "integrated_supported_hold"
        node.command_target_m = contact_event["target_position_m"]
        node.spin_for(args.hold_seconds)
        hold_end = node.now_s()
        hold_contacts = [
            event for event in node.contact_messages
            if hold_start <= float(event["t_s"]) <= hold_end
        ]
        dual = [event for event in hold_contacts if event.get("dual_contact")]
        if not dual:
            raise RuntimeError("bilateral cube contact not maintained in integrated hold")
        if node.unexpected_robot_contacts:
            raise RuntimeError("non-finger robot-cube contact in integrated hold")

        contact_quality = audit_contact_quality(
            hold_contacts, node.gauge_samples, start_s=hold_start, end_s=hold_end
        )
        if contact_quality.get("passed") is not True:
            raise RuntimeError(
                "integrated moving-cube contact quality failed: "
                + json.dumps({k: contact_quality.get(k) for k in ("left", "right")}, sort_keys=True)
            )

        support_ratio = ratio(node.samples, hold_start, hold_end, "support_contact_active")
        dual_ratio = ratio(node.samples, hold_start, hold_end, "dual_finger_contact_active")
        hold_states = [
            state for state in node.gauge_samples
            if hold_start <= float(state["t_s"]) <= hold_end
        ]
        movement = movement_metrics(hold_states, preclose_cube_state)
        if support_ratio < 0.75 or dual_ratio < 0.65:
            raise RuntimeError(
                f"integrated hold ratios failed: support={support_ratio}, dual={dual_ratio}"
            )
        if float(movement["max_horizontal_displacement_m"]) > 0.003:
            raise RuntimeError(f"integrated hold horizontal motion too large: {movement}")
        if float(movement["max_vertical_rise_m"]) > 0.0015:
            raise RuntimeError(f"integrated no-lift hold rose too far: {movement}")
        if float(movement["max_rotation_rad"]) > 0.10:
            raise RuntimeError(f"integrated hold rotation too large: {movement}")

        contact_result = {
            "bilateral_target_m": float(contact_event["target_position_m"]),
            "dual_contact_message_count": len(dual),
            "direct_cube_frame_quality_used": True,
            "legacy_scalar_opening_gate_used": False,
        }
        hold_result = {
            "hold_start_s": hold_start,
            "hold_end_s": hold_end,
            "observed_hold_seconds": hold_end - hold_start,
            "support_contact_ratio": support_ratio,
            "bilateral_finger_contact_ratio": dual_ratio,
            "movement": movement,
        }

        reopen_started = node.now_s()
        reopen = node.send_goal(
            label="integrated_reopen",
            target=OPEN_M,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(reopen)
        if not reopen["reached_goal"] or reopen["stalled"]:
            raise RuntimeError("integrated reopen failed")
        node.phase = "integrated_release_settle"
        release_start = node.now_s()
        node.spin_for(args.release_settle_seconds)
        release_end = node.now_s()
        late_contacts = [
            event for event in node.contact_messages
            if float(event["t_s"]) >= reopen_started + 0.30
        ]
        contact_cleared_after_reopen = not late_contacts
        if not contact_cleared_after_reopen:
            raise RuntimeError("finger contact did not clear after integrated reopen")
        release_support = ratio(node.samples, release_start, release_end, "support_contact_active")
        if release_support < 0.75:
            raise RuntimeError(f"support contact did not persist after release: {release_support}")
        final_state = node.latest_gauge_state
        final_speed = speed(final_state)
        if final_speed is None or final_speed > 0.02:
            raise RuntimeError(f"cube did not settle after release: {final_speed}")
        final_offset = state_distance(preclose_cube_state, final_state)
        if float(final_offset["horizontal_displacement_m"]) > 0.004:
            raise RuntimeError(f"final horizontal offset too large: {final_offset}")
        if abs(float(final_offset["vertical_displacement_m"])) > 0.002:
            raise RuntimeError(f"final vertical offset too large: {final_offset}")
        if float(final_offset["rotation_rad"]) > 0.12:
            raise RuntimeError(f"final rotation too large: {final_offset}")
        release_result = {
            "release_start_s": release_start,
            "release_end_s": release_end,
            "support_contact_ratio": release_support,
            "final_cube_speed_m_s": final_speed,
            "final_offset": final_offset,
        }
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        payload = {
            "schema_version": 1,
            "phase": "RUBIK-INTEGRATED-OPPOSING-SIDE-DYNAMIC-NO-LIFT",
            "input": input_data,
            "fixture_present_during_open_arm_cycle": True,
            "support_contact_before_open_cycle": support_contact_before_open_cycle,
            "open_arm_cycle_start_s": open_cycle_start,
            "open_arm_cycle_end_s": open_cycle_end,
            "open_arm_cycle_support_contact_ratio": open_cycle_support_ratio,
            "open_arm_motion_windows": node.arm_motion_windows,
            "approach_fixture_movement": approach_fixture_movement,
            "contact_result": contact_result,
            "contact_quality": contact_quality,
            "hold_result": hold_result,
            "release_result": release_result,
            "contact_cleared_after_reopen": contact_cleared_after_reopen,
            "unexpected_robot_contacts": node.unexpected_robot_contacts,
            "lift_command_sent": False,
            "attachment_used": False,
            "base_command_sent": False,
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
        if k not in {"samples", "cube_state_samples", "contact_messages", "support_contact_messages", "unexpected_robot_contacts"}
    }, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
