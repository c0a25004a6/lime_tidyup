#!/usr/bin/env python3
"""Run a no-lift close/hold/release trial with the canonical 90 g Rubik cube on support."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from run_gripper_gauge_calibration import force_magnitude, speed
from run_rubiks_static_cube_contact import CubeContactNode, quaternion_angle_rad


class DynamicSupportedCubeNode(CubeContactNode):
    def __init__(self, *, support_model: str, **kwargs) -> None:
        self.support_model = support_model
        self.support_contact_messages: list[dict] = []
        self.last_support_contact_t: float | None = None
        self.last_dual_finger_contact_t: float | None = None
        super().__init__(**kwargs)

    def on_contacts(self, msg: ContactsState) -> None:
        now = self.now_s()
        for state in msg.states:
            collision1 = str(state.collision1_name)
            collision2 = str(state.collision2_name)
            pair = f"{collision1} {collision2}"
            if self.gauge_model not in pair or self.support_model not in pair:
                continue
            depths = [float(value) for value in state.depths]
            support_event = {
                "t_s": now,
                "phase": self.phase,
                "collision1": collision1,
                "collision2": collision2,
                "max_depth_m": max(depths, default=0.0),
                "max_force_magnitude_n": force_magnitude(state),
                "contact_positions_m": [
                    [float(point.x), float(point.y), float(point.z)]
                    for point in state.contact_positions
                ],
                "contact_normals": [
                    [float(normal.x), float(normal.y), float(normal.z)]
                    for normal in state.contact_normals
                ],
            }
            self.support_contact_messages.append(support_event)
            self.last_support_contact_t = now

        before = len(self.contact_messages)
        super().on_contacts(msg)
        for event in self.contact_messages[before:]:
            if event.get("dual_contact"):
                self.last_dual_finger_contact_t = float(event["t_s"])

    def on_joint_state(self, msg: JointState) -> None:
        before = len(self.samples)
        super().on_joint_state(msg)
        if len(self.samples) == before:
            return
        sample = self.samples[-1]
        now = float(sample["t_s"])
        sample["support_contact_active"] = bool(
            self.last_support_contact_t is not None
            and now - self.last_support_contact_t <= 0.08
        )
        sample["finger_contact_active"] = bool(
            self.last_contact_t is not None and now - self.last_contact_t <= 0.08
        )
        sample["dual_finger_contact_active"] = bool(
            self.last_dual_finger_contact_t is not None
            and now - self.last_dual_finger_contact_t <= 0.08
        )
        if self.latest_gauge_state is not None:
            sample["cube_position_m"] = list(self.latest_gauge_state["position_m"])
            sample["cube_orientation_xyzw"] = list(
                self.latest_gauge_state["orientation_xyzw"]
            )
            sample["cube_linear_velocity_m_s"] = list(
                self.latest_gauge_state["linear_velocity_m_s"]
            )

    def spawn_named_model(
        self,
        *,
        model_name: str,
        sdf_text: str,
        namespace: str,
        reference_frame: str,
        pose_xyz: tuple[float, float, float],
        timeout: float,
    ) -> dict:
        if not self.spawn_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/spawn_entity service was not available")
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = sdf_text
        request.robot_namespace = namespace
        request.reference_frame = reference_frame
        request.initial_pose = Pose()
        request.initial_pose.position.x = pose_xyz[0]
        request.initial_pose.position.y = pose_xyz[1]
        request.initial_pose.position.z = pose_xyz[2]
        request.initial_pose.orientation.w = 1.0
        future = self.spawn_client.call_async(request)
        self.wait_for(lambda: future.done(), timeout, f"{model_name} spawn response")
        response = future.result()
        if response is None or not response.success:
            status = response.status_message if response is not None else "no response"
            raise RuntimeError(f"{model_name} spawn failed: {status}")
        return {"success": True, "status_message": str(response.status_message)}


def ratio(samples: list[dict], start: float, end: float, key: str) -> float:
    selected = [sample for sample in samples if start <= float(sample["t_s"]) <= end]
    if not selected:
        return 0.0
    return sum(bool(sample.get(key)) for sample in selected) / len(selected)


def movement_metrics(states: list[dict], reference: dict) -> dict:
    if not states:
        return {
            "sample_count": 0,
            "max_horizontal_displacement_m": None,
            "max_vertical_rise_m": None,
            "max_abs_vertical_displacement_m": None,
            "max_rotation_rad": None,
        }
    reference_position = reference["position_m"]
    reference_orientation = reference["orientation_xyzw"]
    horizontal = []
    vertical_rise = []
    vertical_abs = []
    rotations = []
    for state in states:
        position = state["position_m"]
        dx = float(position[0]) - float(reference_position[0])
        dy = float(position[1]) - float(reference_position[1])
        dz = float(position[2]) - float(reference_position[2])
        horizontal.append(math.hypot(dx, dy))
        vertical_rise.append(dz)
        vertical_abs.append(abs(dz))
        rotations.append(
            quaternion_angle_rad(reference_orientation, state["orientation_xyzw"])
        )
    return {
        "sample_count": len(states),
        "max_horizontal_displacement_m": max(horizontal),
        "max_vertical_rise_m": max(vertical_rise),
        "max_abs_vertical_displacement_m": max(vertical_abs),
        "max_rotation_rad": max(rotations),
    }


def state_distance(a: dict | None, b: dict | None) -> dict:
    if a is None or b is None:
        return {
            "horizontal_displacement_m": None,
            "vertical_displacement_m": None,
            "rotation_rad": None,
        }
    dx = float(b["position_m"][0]) - float(a["position_m"][0])
    dy = float(b["position_m"][1]) - float(a["position_m"][1])
    dz = float(b["position_m"][2]) - float(a["position_m"][2])
    return {
        "horizontal_displacement_m": math.hypot(dx, dy),
        "vertical_displacement_m": dz,
        "rotation_rad": quaternion_angle_rad(
            a["orientation_xyzw"], b["orientation_xyzw"]
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cube-sdf", required=True)
    parser.add_argument("--support-sdf", required=True)
    parser.add_argument("--cube-width", type=float, default=0.057)
    parser.add_argument("--cube-mass", type=float, default=0.09)
    parser.add_argument(
        "--combined-inner-face-offset", type=float, default=0.003000000004097683
    )
    parser.add_argument(
        "--calibration-uncertainty", type=float, default=0.0003279557798182653
    )
    parser.add_argument(
        "--reference-frame", default="turtlebot3_lime_gripper_test::link7"
    )
    parser.add_argument("--cube-x", type=float, default=-0.019)
    parser.add_argument("--cube-y", type=float, default=0.0)
    parser.add_argument("--cube-z", type=float, default=0.1192)
    parser.add_argument("--support-x", type=float, default=-0.019)
    parser.add_argument("--support-y", type=float, default=0.0)
    parser.add_argument("--support-z", type=float, default=0.0857)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--release-settle-seconds", type=float, default=1.5)
    parser.add_argument("--close-start", type=float, default=0.0185)
    parser.add_argument("--close-stop", type=float, default=0.0060)
    parser.add_argument("--close-step", type=float, default=0.00025)
    parser.add_argument("--step-hold", type=float, default=0.12)
    parser.add_argument("--max-effort", type=float, default=0.25)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    errors: list[str] = []
    events: list[dict] = []
    support_spawn_result: dict | None = None
    cube_spawn_result: dict | None = None
    initial_robot_state: dict | None = None
    settled_cube_state: dict | None = None
    contact_result: dict | None = None
    hold_result: dict | None = None
    release_result: dict | None = None
    contact_cleared_after_reopen = False
    support_contact_before_close = False

    robot_model = "turtlebot3_lime_gripper_test"
    cube_model = "rubiks_dynamic_cube_supported_057"
    support_model = "rubiks_cube_support_surface"
    left_link = f"{robot_model}::gripper_left_link"
    right_link = f"{robot_model}::gripper_right_link"
    cube_link = f"{cube_model}::cube_link"

    rclpy.init(args=None)
    node = DynamicSupportedCubeNode(
        support_model=support_model,
        action_name="/gripper_controller/gripper_cmd",
        left_joint="gripper_left_joint",
        right_joint="gripper_right_joint",
        robot_model=robot_model,
        gauge_model=cube_model,
        left_link=left_link,
        right_link=right_link,
        gauge_link=cube_link,
        contact_topic="/rubiks_dynamic_cube_hold/contact_states",
    )
    try:
        node.spin_for(1.0)
        if node.latest_joint is None or node.latest_link is None:
            raise RuntimeError("initial gripper joint/link telemetry was not observed")
        initial_robot_state = node.latest_robot_state

        open_event = node.send_goal(
            label="open_before_cube",
            target=0.019,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(open_event)
        if not open_event["reached_goal"] or open_event["stalled"]:
            raise RuntimeError("gripper did not reach the open pre-cube position")

        node.phase = "spawn_support"
        support_spawn_result = node.spawn_named_model(
            model_name=support_model,
            sdf_text=Path(args.support_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_cube_support",
            reference_frame=args.reference_frame,
            pose_xyz=(args.support_x, args.support_y, args.support_z),
            timeout=args.timeout,
        )

        node.phase = "spawn_dynamic_cube"
        cube_spawn_result = node.spawn_named_model(
            model_name=cube_model,
            sdf_text=Path(args.cube_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_dynamic_cube_hold",
            reference_frame=args.reference_frame,
            pose_xyz=(args.cube_x, args.cube_y, args.cube_z),
            timeout=args.timeout,
        )
        node.wait_for(
            lambda: node.latest_gauge_state is not None,
            args.timeout,
            "dynamic cube model state",
        )

        node.phase = "settle_on_support"
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
            raise RuntimeError("cube did not establish support contact before closing")
        settled_speed = speed(settled_cube_state)
        if settled_speed is None or settled_speed > 0.02:
            raise RuntimeError(f"cube did not settle on support: speed={settled_speed}")
        if node.contact_messages:
            raise RuntimeError("cube contacted a finger while the gripper was fully open")
        if node.unexpected_robot_contacts:
            raise RuntimeError("cube initially contacted a non-finger robot collision")

        target = args.close_start
        step_index = 0
        contact_event: dict | None = None
        while target >= args.close_stop - 1e-12:
            event = node.send_goal(
                label=f"close_step_{step_index:02d}",
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
                raise RuntimeError(
                    f"close step {target:.6f} failed before dynamic cube contact"
                )
            node.spin_for(args.step_hold)
            target -= args.close_step
            step_index += 1
        if contact_event is None:
            raise RuntimeError("no finger contact was observed in the bounded close sweep")

        hold_start = node.now_s()
        node.phase = "cube_contact_hold"
        node.command_target_m = contact_event["target_position_m"]
        node.spin_for(args.hold_seconds)
        hold_end = node.now_s()

        hold_contacts = [
            event
            for event in node.contact_messages
            if hold_start <= float(event["t_s"]) <= hold_end
        ]
        dual_contacts = [
            event
            for event in hold_contacts
            if event.get("dual_contact")
            and event.get("link_frame_separation_m") is not None
        ]
        if not dual_contacts:
            raise RuntimeError("bilateral finger contact was not maintained during hold")
        if node.unexpected_robot_contacts:
            raise RuntimeError("dynamic cube contacted a non-finger robot collision")

        separations = [float(event["link_frame_separation_m"]) for event in dual_contacts]
        inner_openings = [
            separation - args.combined_inner_face_offset for separation in separations
        ]
        contact_opening = max(inner_openings)
        opening_error = contact_opening - args.cube_width
        tolerance = max(0.001, 3.0 * args.calibration_uncertainty)
        if abs(opening_error) > tolerance:
            raise RuntimeError(
                f"calibrated dynamic contact opening error {opening_error} exceeds {tolerance}"
            )

        support_ratio = ratio(
            node.samples, hold_start, hold_end, "support_contact_active"
        )
        dual_ratio = ratio(
            node.samples, hold_start, hold_end, "dual_finger_contact_active"
        )
        hold_states = [
            state
            for state in node.gauge_samples
            if hold_start <= float(state["t_s"]) <= hold_end
        ]
        movement = movement_metrics(hold_states, settled_cube_state)
        if support_ratio < 0.75:
            raise RuntimeError(
                f"support contact coverage during hold is too low: {support_ratio}"
            )
        if dual_ratio < 0.65:
            raise RuntimeError(
                f"bilateral finger contact coverage during hold is too low: {dual_ratio}"
            )
        if float(movement["max_horizontal_displacement_m"]) > 0.003:
            raise RuntimeError(f"cube slid too far during hold: {movement}")
        if float(movement["max_vertical_rise_m"]) > 0.0015:
            raise RuntimeError(f"cube rose from support during hold: {movement}")
        if float(movement["max_rotation_rad"]) > 0.10:
            raise RuntimeError(f"cube rotated too far during hold: {movement}")

        contact_result = {
            "method": "maximum dual-contact calibrated inner-face opening",
            "cube_width_m": args.cube_width,
            "combined_inner_face_offset_m": args.combined_inner_face_offset,
            "calibration_uncertainty_m": args.calibration_uncertainty,
            "acceptance_tolerance_m": tolerance,
            "dual_contact_sample_count": len(dual_contacts),
            "dual_contact_link_separation_min_m": min(separations),
            "dual_contact_link_separation_max_m": max(separations),
            "dual_contact_inner_opening_min_m": min(inner_openings),
            "dual_contact_inner_opening_max_m": max(inner_openings),
            "accepted_contact_inner_opening_m": contact_opening,
            "opening_error_m": opening_error,
            "max_contact_depth_m": max(
                float(event.get("max_depth_m", 0.0)) for event in dual_contacts
            ),
            "max_force_magnitude_n": max(
                float(event.get("max_force_magnitude_n", 0.0))
                for event in dual_contacts
            ),
        }
        hold_result = {
            "requested_hold_seconds": args.hold_seconds,
            "observed_hold_seconds": hold_end - hold_start,
            "support_contact_ratio": support_ratio,
            "bilateral_finger_contact_ratio": dual_ratio,
            "support_contact_message_count": sum(
                hold_start <= float(event["t_s"]) <= hold_end
                for event in node.support_contact_messages
            ),
            "finger_contact_message_count": len(hold_contacts),
            "movement": movement,
        }

        reopen_started = node.now_s()
        reopen_event = node.send_goal(
            label="reopen_after_cube_contact",
            target=0.019,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(reopen_event)
        if not reopen_event["reached_goal"] or reopen_event["stalled"]:
            raise RuntimeError("gripper did not reopen after dynamic hold")

        release_start = node.now_s()
        node.phase = "complete"
        node.command_target_m = None
        node.spin_for(args.release_settle_seconds)
        release_end = node.now_s()
        late_finger_contacts = [
            event
            for event in node.contact_messages
            if float(event["t_s"]) >= reopen_started + 0.30
        ]
        contact_cleared_after_reopen = not late_finger_contacts
        if not contact_cleared_after_reopen:
            raise RuntimeError("finger contact did not clear after reopening")
        post_support_ratio = ratio(
            node.samples, release_start, release_end, "support_contact_active"
        )
        final_cube_state = node.latest_gauge_state
        final_cube_speed = speed(final_cube_state)
        final_offset = state_distance(settled_cube_state, final_cube_state)
        if post_support_ratio < 0.75:
            raise RuntimeError(
                f"support contact did not persist after release: {post_support_ratio}"
            )
        if final_cube_speed is None or final_cube_speed > 0.02:
            raise RuntimeError(f"cube did not settle after release: speed={final_cube_speed}")
        if float(final_offset["horizontal_displacement_m"]) > 0.004:
            raise RuntimeError(f"cube final horizontal offset too large: {final_offset}")
        if abs(float(final_offset["vertical_displacement_m"])) > 0.002:
            raise RuntimeError(f"cube final vertical offset too large: {final_offset}")
        if float(final_offset["rotation_rad"]) > 0.12:
            raise RuntimeError(f"cube final rotation too large: {final_offset}")
        release_result = {
            "requested_settle_seconds": args.release_settle_seconds,
            "observed_settle_seconds": release_end - release_start,
            "support_contact_ratio": post_support_ratio,
            "final_cube_linear_speed_m_s": final_cube_speed,
            "final_offset_from_preclose_settled_state": final_offset,
        }
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        final_robot_state = node.latest_robot_state
        final_cube_state = node.latest_gauge_state
        for sample in node.samples:
            separation = sample.get("link_frame_separation_m")
            sample["cube_width_m"] = args.cube_width
            sample["calibrated_combined_inner_face_offset_m"] = (
                args.combined_inner_face_offset
            )
            sample["inner_face_opening_m"] = (
                float(separation) - args.combined_inner_face_offset
                if separation is not None
                else None
            )
        final_offset = state_distance(settled_cube_state, final_cube_state)
        max_vertical_rise = (
            hold_result["movement"]["max_vertical_rise_m"]
            if hold_result is not None
            else None
        )
        payload = {
            "schema_version": 1,
            "trial_type": "gazebo_dynamic_supported_rubiks_cube_hold_release",
            "hardware_backend": "gazebo_ros2_control",
            "cube_model": cube_model,
            "support_model": support_model,
            "source_cube_collision_size_m": [0.057, 0.057, 0.057],
            "cube_width_m": args.cube_width,
            "cube_mass_kg": args.cube_mass,
            "cube_dynamic": True,
            "cube_mass_dynamics_tested": True,
            "support_static": True,
            "cube_pose_relative_to": args.reference_frame,
            "cube_pose_xyz_m": [args.cube_x, args.cube_y, args.cube_z],
            "support_pose_xyz_m": [args.support_x, args.support_y, args.support_z],
            "support_spawn_result": support_spawn_result,
            "cube_spawn_result": cube_spawn_result,
            "cube_spawned": final_cube_state is not None,
            "support_contact_before_close": support_contact_before_close,
            "initial_open_finger_contact": bool(
                node.contact_messages
                and node.contact_messages[0].get("phase")
                in {"spawn_dynamic_cube", "settle_on_support"}
            ),
            "left_cube_contact_observed": any(
                event.get("left_contact") for event in node.contact_messages
            ),
            "right_cube_contact_observed": any(
                event.get("right_contact") for event in node.contact_messages
            ),
            "dual_cube_contact_observed": any(
                event.get("dual_contact") for event in node.contact_messages
            ),
            "unexpected_robot_contact_count": len(node.unexpected_robot_contacts),
            "unexpected_robot_contacts": node.unexpected_robot_contacts,
            "contact_cleared_after_reopen": contact_cleared_after_reopen,
            "contact_result": contact_result,
            "hold_result": hold_result,
            "release_result": release_result,
            "calibration_applied": True,
            "combined_inner_face_offset_m": args.combined_inner_face_offset,
            "calibration_uncertainty_m": args.calibration_uncertainty,
            "arm_motion_performed": False,
            "lift_command_sent": False,
            "cube_lifted": bool(
                max_vertical_rise is not None and float(max_vertical_rise) > 0.0015
            ),
            "ifra_attachment_used": False,
            "grasp_success_claimed": False,
            "errors": errors,
            "events": events,
            "contact_messages": node.contact_messages,
            "support_contact_messages": node.support_contact_messages,
            "samples": node.samples,
            "link_state_samples": node.link_samples,
            "initial_robot_state": initial_robot_state,
            "final_robot_state": final_robot_state,
            "robot_state_samples": node.robot_samples,
            "settled_cube_state": settled_cube_state,
            "final_cube_state": final_cube_state,
            "cube_state_samples": node.gauge_samples,
            "final_offset_from_preclose_settled_state": final_offset,
            "robot_base_displacement_m": (
                math.dist(
                    initial_robot_state["position_m"], final_robot_state["position_m"]
                )
                if initial_robot_state is not None and final_robot_state is not None
                else None
            ),
            "robot_final_linear_speed_m_s": speed(final_robot_state),
        }
        output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()

    print(
        json.dumps(
            {
                key: value
                for key, value in payload.items()
                if key
                not in {
                    "samples",
                    "link_state_samples",
                    "robot_state_samples",
                    "cube_state_samples",
                    "contact_messages",
                    "support_contact_messages",
                    "unexpected_robot_contacts",
                }
            },
            indent=2,
        )
    )
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
