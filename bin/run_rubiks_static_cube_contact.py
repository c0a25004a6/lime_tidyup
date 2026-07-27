#!/usr/bin/env python3
"""Run a calibrated no-lift side-contact trial against a stationary 57 mm cube."""
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

from run_gripper_gauge_calibration import GaugeCalibrationNode, speed


def quaternion_angle_rad(a: list[float], b: list[float]) -> float:
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


class CubeContactNode(GaugeCalibrationNode):
    def __init__(self, **kwargs) -> None:
        self.unexpected_robot_contacts: list[dict] = []
        super().__init__(**kwargs)

    def on_contacts(self, msg: ContactsState) -> None:
        for state in msg.states:
            collision1 = str(state.collision1_name)
            collision2 = str(state.collision2_name)
            pair = f"{collision1} {collision2}"
            if self.gauge_model not in pair or self.robot_model not in pair:
                continue
            finger_contact = (
                "gripper_left_link" in pair or "gripper_right_link" in pair
            )
            if not finger_contact:
                self.unexpected_robot_contacts.append({
                    "t_s": self.now_s(),
                    "phase": self.phase,
                    "collision1": collision1,
                    "collision2": collision2,
                })
        super().on_contacts(msg)

    def spawn_cube(
        self,
        *,
        sdf_text: str,
        reference_frame: str,
        pose_xyz: tuple[float, float, float],
        timeout: float,
    ) -> dict:
        if not self.spawn_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/spawn_entity service was not available")
        request = SpawnEntity.Request()
        request.name = self.gauge_model
        request.xml = sdf_text
        request.robot_namespace = "/rubiks_cube_contact"
        request.reference_frame = reference_frame
        request.initial_pose = Pose()
        request.initial_pose.position.x = pose_xyz[0]
        request.initial_pose.position.y = pose_xyz[1]
        request.initial_pose.position.z = pose_xyz[2]
        request.initial_pose.orientation.w = 1.0
        future = self.spawn_client.call_async(request)
        self.wait_for(lambda: future.done(), timeout, "cube spawn response")
        response = future.result()
        if response is None or not response.success:
            status = response.status_message if response is not None else "no response"
            raise RuntimeError(f"cube spawn failed: {status}")
        return {"success": True, "status_message": str(response.status_message)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cube-sdf", required=True)
    parser.add_argument("--cube-width", type=float, default=0.057)
    parser.add_argument("--combined-inner-face-offset", type=float, default=0.003000000004097683)
    parser.add_argument("--calibration-uncertainty", type=float, default=0.0003279557798182653)
    parser.add_argument("--reference-frame", default="turtlebot3_lime_gripper_test::link7")
    parser.add_argument("--cube-x", type=float, default=-0.019)
    parser.add_argument("--cube-y", type=float, default=0.0)
    parser.add_argument("--cube-z", type=float, default=0.1007)
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
    spawn_result: dict | None = None
    initial_robot_state: dict | None = None
    initial_cube_state: dict | None = None
    contact_cleared_after_reopen = False
    contact_result: dict | None = None

    robot_model = "turtlebot3_lime_gripper_test"
    cube_model = "rubiks_static_cube_contact_057"
    left_link = f"{robot_model}::gripper_left_link"
    right_link = f"{robot_model}::gripper_right_link"
    cube_link = f"{cube_model}::cube_link"

    rclpy.init(args=None)
    node = CubeContactNode(
        action_name="/gripper_controller/gripper_cmd",
        left_joint="gripper_left_joint",
        right_joint="gripper_right_joint",
        robot_model=robot_model,
        gauge_model=cube_model,
        left_link=left_link,
        right_link=right_link,
        gauge_link=cube_link,
        contact_topic="/rubiks_cube_contact/contact_states",
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

        node.phase = "spawn_static_cube"
        node.command_target_m = None
        spawn_result = node.spawn_cube(
            sdf_text=Path(args.cube_sdf).read_text(encoding="utf-8"),
            reference_frame=args.reference_frame,
            pose_xyz=(args.cube_x, args.cube_y, args.cube_z),
            timeout=args.timeout,
        )
        node.wait_for(
            lambda: node.latest_gauge_state is not None,
            args.timeout,
            "static cube model state",
        )
        initial_cube_state = dict(node.latest_gauge_state)
        accepted_before = len(node.contact_messages)
        unexpected_before = len(node.unexpected_robot_contacts)
        node.spin_for(0.8)
        if len(node.contact_messages) > accepted_before:
            raise RuntimeError("cube contacted a finger while the gripper was fully open")
        if len(node.unexpected_robot_contacts) > unexpected_before:
            raise RuntimeError("cube initially intersected a non-finger robot collision")

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
                raise RuntimeError(f"close step {target:.6f} failed before cube contact")
            node.spin_for(args.step_hold)
            target -= args.close_step
            step_index += 1
        if contact_event is None:
            raise RuntimeError("no cube contact was observed in the bounded close sweep")

        node.phase = "cube_contact_hold"
        node.command_target_m = contact_event["target_position_m"]
        node.spin_for(0.35)
        dual_contacts = [
            event for event in node.contact_messages
            if event.get("dual_contact")
            and event.get("link_frame_separation_m") is not None
        ]
        if not dual_contacts:
            raise RuntimeError("cube was not independently contacted by both fingers")
        if node.unexpected_robot_contacts:
            raise RuntimeError("cube contacted a non-finger robot collision")

        separations = [float(event["link_frame_separation_m"]) for event in dual_contacts]
        inner_openings = [
            separation - args.combined_inner_face_offset for separation in separations
        ]
        contact_opening = max(inner_openings)
        opening_error = contact_opening - args.cube_width
        tolerance = max(0.001, 3.0 * args.calibration_uncertainty)
        if abs(opening_error) > tolerance:
            raise RuntimeError(
                f"calibrated contact opening error {opening_error} exceeds {tolerance}"
            )
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
                float(event.get("max_force_magnitude_n", 0.0)) for event in dual_contacts
            ),
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
            raise RuntimeError("gripper did not reopen after cube contact")
        node.phase = "complete"
        node.command_target_m = None
        node.spin_for(0.8)
        late_contacts = [
            event for event in node.contact_messages
            if float(event["t_s"]) >= reopen_started + 0.30
        ]
        contact_cleared_after_reopen = not late_contacts
        if not contact_cleared_after_reopen:
            raise RuntimeError("cube contact did not clear after reopening")
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
                if separation is not None else None
            )
        cube_displacement = (
            math.dist(initial_cube_state["position_m"], final_cube_state["position_m"])
            if initial_cube_state is not None and final_cube_state is not None
            else None
        )
        cube_rotation = (
            quaternion_angle_rad(
                initial_cube_state["orientation_xyzw"],
                final_cube_state["orientation_xyzw"],
            )
            if initial_cube_state is not None and final_cube_state is not None
            else None
        )
        payload = {
            "schema_version": 1,
            "trial_type": "gazebo_static_rubiks_cube_side_contact",
            "hardware_backend": "gazebo_ros2_control",
            "cube_model": cube_model,
            "source_cube_collision_size_m": [0.057, 0.057, 0.057],
            "cube_width_m": args.cube_width,
            "cube_static": True,
            "cube_mass_dynamics_tested": False,
            "cube_pose_relative_to": args.reference_frame,
            "cube_pose_xyz_m": [args.cube_x, args.cube_y, args.cube_z],
            "spawn_result": spawn_result,
            "cube_spawned": final_cube_state is not None,
            "initial_open_contact": bool(
                node.contact_messages and node.contact_messages[0]["phase"] == "spawn_static_cube"
            ),
            "cube_contact": bool(node.contact_messages),
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
            "calibration_applied": True,
            "combined_inner_face_offset_m": args.combined_inner_face_offset,
            "calibration_uncertainty_m": args.calibration_uncertainty,
            "arm_motion_performed": False,
            "lift_command_sent": False,
            "cube_lifted": False,
            "ifra_attachment_used": False,
            "grasp_success_claimed": False,
            "errors": errors,
            "events": events,
            "contact_messages": node.contact_messages,
            "samples": node.samples,
            "link_state_samples": node.link_samples,
            "initial_robot_state": initial_robot_state,
            "final_robot_state": final_robot_state,
            "robot_state_samples": node.robot_samples,
            "initial_cube_state": initial_cube_state,
            "final_cube_state": final_cube_state,
            "cube_state_samples": node.gauge_samples,
            "cube_displacement_m": cube_displacement,
            "cube_rotation_rad": cube_rotation,
            "robot_base_displacement_m": (
                math.dist(initial_robot_state["position_m"], final_robot_state["position_m"])
                if initial_robot_state is not None and final_robot_state is not None
                else None
            ),
            "robot_final_linear_speed_m_s": speed(final_robot_state),
        }
        output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps({
        key: value for key, value in payload.items()
        if key not in {
            "samples", "link_state_samples", "robot_state_samples",
            "cube_state_samples", "contact_messages", "unexpected_robot_contacts",
        }
    }, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
