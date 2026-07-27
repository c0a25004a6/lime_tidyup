#!/usr/bin/env python3
"""Calibrate Lime gripper inner-face opening with a known-width Gazebo gauge."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from gazebo_msgs.msg import ContactsState, LinkStates, ModelStates
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


def vector(values: object) -> list[float]:
    return [float(value) for value in values]  # type: ignore[arg-type]


def force_magnitude(state: object) -> float:
    wrench = state.total_wrench
    force = wrench.force
    return math.sqrt(float(force.x) ** 2 + float(force.y) ** 2 + float(force.z) ** 2)


class GaugeCalibrationNode(Node):
    def __init__(
        self,
        *,
        action_name: str,
        left_joint: str,
        right_joint: str,
        robot_model: str,
        gauge_model: str,
        left_link: str,
        right_link: str,
        gauge_link: str,
        contact_topic: str,
    ) -> None:
        super().__init__("rubiks_gripper_gauge_calibration")
        self.client = ActionClient(self, GripperCommand, action_name)
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.robot_model = robot_model
        self.gauge_model = gauge_model
        self.left_link = left_link
        self.right_link = right_link
        self.gauge_link = gauge_link
        self.started = time.monotonic()
        self.phase = "initial"
        self.command_target_m: float | None = None
        self.latest_joint: dict | None = None
        self.latest_link: dict | None = None
        self.latest_robot_state: dict | None = None
        self.latest_gauge_state: dict | None = None
        self.samples: list[dict] = []
        self.link_samples: list[dict] = []
        self.robot_samples: list[dict] = []
        self.gauge_samples: list[dict] = []
        self.contact_messages: list[dict] = []
        self.command_contacts: list[dict] = []
        self.last_contact_t: float | None = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 100)
        self.create_subscription(LinkStates, "/link_states", self.on_link_states, 100)
        self.create_subscription(ModelStates, "/model_states", self.on_model_states, 50)
        self.create_subscription(ContactsState, contact_topic, self.on_contacts, 100)

    def now_s(self) -> float:
        return time.monotonic() - self.started

    def on_joint_state(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        if self.left_joint not in values:
            return
        left = float(values[self.left_joint])
        right_value = values.get(self.right_joint)
        right = float(right_value) if right_value is not None else None
        separation = self.latest_link.get("separation_m") if self.latest_link else None
        contact_active = bool(
            self.last_contact_t is not None and self.now_s() - self.last_contact_t <= 0.08
        )
        sample = {
            "t_s": self.now_s(),
            "phase": self.phase,
            "command_target_m": self.command_target_m,
            "left_joint_position_m": left,
            "right_joint_position_m": right,
            "right_joint_observed": right is not None,
            "gazebo_link_frame_separation_m": separation,
            "link_frame_separation_m": separation,
            "link_frame_separation_source": "gazebo_link_states" if separation is not None else None,
            "contact_active": contact_active,
            "gauge_width_m": None,
            "inner_face_opening_m": None,
        }
        self.latest_joint = sample
        self.samples.append(sample)

    def on_link_states(self, msg: LinkStates) -> None:
        try:
            left_index = msg.name.index(self.left_link)
            right_index = msg.name.index(self.right_link)
        except ValueError:
            return
        left_pose = msg.pose[left_index]
        right_pose = msg.pose[right_index]
        left_position = vector((left_pose.position.x, left_pose.position.y, left_pose.position.z))
        right_position = vector((right_pose.position.x, right_pose.position.y, right_pose.position.z))
        gauge_position = None
        if self.gauge_link in msg.name:
            gauge_pose = msg.pose[msg.name.index(self.gauge_link)]
            gauge_position = vector(
                (gauge_pose.position.x, gauge_pose.position.y, gauge_pose.position.z)
            )
        sample = {
            "t_s": self.now_s(),
            "phase": self.phase,
            "left_position_m": left_position,
            "right_position_m": right_position,
            "gauge_position_m": gauge_position,
            "separation_m": math.dist(left_position, right_position),
        }
        self.latest_link = sample
        self.link_samples.append(sample)

    @staticmethod
    def model_state(msg: ModelStates, name: str, t_s: float) -> dict | None:
        try:
            index = msg.name.index(name)
        except ValueError:
            return None
        pose = msg.pose[index]
        twist = msg.twist[index]
        return {
            "t_s": t_s,
            "position_m": vector((pose.position.x, pose.position.y, pose.position.z)),
            "orientation_xyzw": vector(
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            ),
            "linear_velocity_m_s": vector((twist.linear.x, twist.linear.y, twist.linear.z)),
            "angular_velocity_rad_s": vector(
                (twist.angular.x, twist.angular.y, twist.angular.z)
            ),
        }

    def on_model_states(self, msg: ModelStates) -> None:
        t_s = self.now_s()
        robot = self.model_state(msg, self.robot_model, t_s)
        if robot is not None:
            self.latest_robot_state = robot
            self.robot_samples.append(robot)
        gauge = self.model_state(msg, self.gauge_model, t_s)
        if gauge is not None:
            self.latest_gauge_state = gauge
            self.gauge_samples.append(gauge)

    def on_contacts(self, msg: ContactsState) -> None:
        if not msg.states:
            return
        states: list[dict] = []
        left_contact = False
        right_contact = False
        max_depth_m = 0.0
        max_force_n = 0.0
        for state in msg.states:
            collision1 = str(state.collision1_name)
            collision2 = str(state.collision2_name)
            pair = f"{collision1} {collision2}"
            touches_gauge = self.gauge_model in pair or "gauge_collision" in pair
            touches_left = "gripper_left_link" in pair
            touches_right = "gripper_right_link" in pair
            if not touches_gauge or not (touches_left or touches_right):
                continue
            depths = [float(value) for value in state.depths]
            state_force = force_magnitude(state)
            max_depth_m = max(max_depth_m, max(depths, default=0.0))
            max_force_n = max(max_force_n, state_force)
            left_contact = left_contact or touches_left
            right_contact = right_contact or touches_right
            states.append({
                "collision1": collision1,
                "collision2": collision2,
                "depths_m": depths,
                "contact_positions_m": [
                    vector((point.x, point.y, point.z)) for point in state.contact_positions
                ],
                "contact_normals": [
                    vector((normal.x, normal.y, normal.z)) for normal in state.contact_normals
                ],
                "total_force_magnitude_n": state_force,
            })
        if not states:
            return
        sample = {
            "t_s": self.now_s(),
            "phase": self.phase,
            "command_target_m": self.command_target_m,
            "left_contact": left_contact,
            "right_contact": right_contact,
            "dual_contact": left_contact and right_contact,
            "max_depth_m": max_depth_m,
            "max_force_magnitude_n": max_force_n,
            "link_frame_separation_m": (
                self.latest_link.get("separation_m") if self.latest_link else None
            ),
            "left_joint_position_m": (
                self.latest_joint.get("left_joint_position_m") if self.latest_joint else None
            ),
            "states": states,
        }
        self.last_contact_t = float(sample["t_s"])
        self.contact_messages.append(sample)
        self.command_contacts.append(sample)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, predicate, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"timed out waiting for {description}")

    def spawn_gauge(
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
        request.robot_namespace = "/rubiks_gauge"
        request.reference_frame = reference_frame
        request.initial_pose = Pose()
        request.initial_pose.position.x = pose_xyz[0]
        request.initial_pose.position.y = pose_xyz[1]
        request.initial_pose.position.z = pose_xyz[2]
        request.initial_pose.orientation.w = 1.0
        future = self.spawn_client.call_async(request)
        self.wait_for(lambda: future.done(), timeout, "gauge spawn response")
        response = future.result()
        if response is None or not response.success:
            status = response.status_message if response is not None else "no response"
            raise RuntimeError(f"gauge spawn failed: {status}")
        return {"success": True, "status_message": str(response.status_message)}

    def send_goal(
        self,
        *,
        label: str,
        target: float,
        max_effort: float,
        timeout: float,
        monitor_contact: bool,
        contact_settle_s: float = 0.15,
    ) -> dict:
        self.phase = label
        self.command_target_m = target
        self.command_contacts = []
        if not self.client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("gripper action server was not available")
        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = max_effort
        send_future = self.client.send_goal_async(goal)
        self.wait_for(lambda: send_future.done(), timeout, f"{label} goal acceptance")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label} goal was rejected")
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        contact_deadline: float | None = None
        cancelled_on_contact = False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if result_future.done():
                break
            if monitor_contact and self.command_contacts:
                if contact_deadline is None:
                    contact_deadline = time.monotonic() + contact_settle_s
                dual = any(event.get("dual_contact") for event in self.command_contacts)
                if dual or time.monotonic() >= contact_deadline:
                    cancel_future = handle.cancel_goal_async()
                    self.wait_for(
                        lambda: cancel_future.done(), 3.0, f"{label} contact cancellation"
                    )
                    cancelled_on_contact = True
                    break
        if not result_future.done() and not cancelled_on_contact:
            cancel_future = handle.cancel_goal_async()
            self.wait_for(lambda: cancel_future.done(), 3.0, f"{label} timeout cancellation")
            raise RuntimeError(f"{label} timed out")
        wrapped = result_future.result() if result_future.done() else None
        result = wrapped.result if wrapped is not None else None
        self.spin_for(0.08)
        return {
            "label": label,
            "target_position_m": target,
            "max_effort": max_effort,
            "result_status": int(wrapped.status) if wrapped is not None else None,
            "reached_goal": bool(result.reached_goal) if result is not None else False,
            "stalled": bool(result.stalled) if result is not None else False,
            "reported_position_m": float(result.position) if result is not None else None,
            "reported_effort": float(result.effort) if result is not None else None,
            "cancelled_on_contact": cancelled_on_contact,
            "contact_observed": bool(self.command_contacts),
            "dual_contact_observed": any(
                event.get("dual_contact") for event in self.command_contacts
            ),
            "contact_message_count": len(self.command_contacts),
            "observed_left_joint_position_m": (
                self.latest_joint.get("left_joint_position_m") if self.latest_joint else None
            ),
            "observed_right_joint_position_m": (
                self.latest_joint.get("right_joint_position_m") if self.latest_joint else None
            ),
            "gazebo_link_frame_separation_m": (
                self.latest_link.get("separation_m") if self.latest_link else None
            ),
        }


def speed(state: dict | None) -> float | None:
    if state is None:
        return None
    return math.sqrt(sum(float(value) ** 2 for value in state["linear_velocity_m_s"]))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--gauge-sdf", required=True)
    parser.add_argument("--gauge-width", type=float, default=0.057)
    parser.add_argument("--reference-frame", default="turtlebot3_lime_gripper_test::link7")
    parser.add_argument("--gauge-x", type=float, default=-0.019)
    parser.add_argument("--gauge-y", type=float, default=0.0)
    parser.add_argument("--gauge-z", type=float, default=0.1007)
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
    calibration: dict | None = None
    initial_contact = False
    contact_cleared_after_reopen = False

    robot_model = "turtlebot3_lime_gripper_test"
    gauge_model = "rubiks_static_gauge_057"
    left_link = f"{robot_model}::gripper_left_link"
    right_link = f"{robot_model}::gripper_right_link"
    gauge_link = f"{gauge_model}::gauge_link"

    rclpy.init(args=None)
    node = GaugeCalibrationNode(
        action_name="/gripper_controller/gripper_cmd",
        left_joint="gripper_left_joint",
        right_joint="gripper_right_joint",
        robot_model=robot_model,
        gauge_model=gauge_model,
        left_link=left_link,
        right_link=right_link,
        gauge_link=gauge_link,
        contact_topic="/rubiks_gauge/contact_states",
    )
    try:
        node.spin_for(1.0)
        if node.latest_joint is None or node.latest_link is None:
            raise RuntimeError("initial gripper joint/link telemetry was not observed")
        initial_robot_state = node.latest_robot_state

        open_event = node.send_goal(
            label="open_before_gauge",
            target=0.019,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(open_event)
        if not open_event["reached_goal"] or open_event["stalled"]:
            raise RuntimeError("gripper did not reach the open pre-gauge position")

        node.phase = "spawn_gauge"
        node.command_target_m = None
        spawn_result = node.spawn_gauge(
            sdf_text=Path(args.gauge_sdf).read_text(encoding="utf-8"),
            reference_frame=args.reference_frame,
            pose_xyz=(args.gauge_x, args.gauge_y, args.gauge_z),
            timeout=args.timeout,
        )
        node.wait_for(
            lambda: node.latest_gauge_state is not None,
            args.timeout,
            "gauge model state",
        )
        contacts_before = len(node.contact_messages)
        node.spin_for(0.8)
        initial_contact = len(node.contact_messages) > contacts_before
        if initial_contact:
            raise RuntimeError("gauge contacted a finger while the gripper was fully open")

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
                    f"close step {target:.6f} failed before gauge contact"
                )
            node.spin_for(args.step_hold)
            target -= args.close_step
            step_index += 1
        if contact_event is None:
            raise RuntimeError("no gauge contact was observed in the bounded close sweep")

        node.phase = "contact_hold"
        node.command_target_m = contact_event["target_position_m"]
        node.spin_for(0.35)
        dual_contacts = [
            event for event in node.contact_messages
            if event.get("dual_contact")
            and event.get("link_frame_separation_m") is not None
        ]
        if not dual_contacts:
            raise RuntimeError("gauge was not independently contacted by both fingers")
        separations = [float(event["link_frame_separation_m"]) for event in dual_contacts]
        calibration_separation = max(separations)
        combined_offset = calibration_separation - args.gauge_width
        max_depth = max(float(event.get("max_depth_m", 0.0)) for event in dual_contacts)
        spread = max(separations) - min(separations)
        uncertainty = max(0.00025, spread / 2.0 + max_depth)
        calibration = {
            "method": "maximum dual-contact link-frame separation",
            "gauge_width_m": args.gauge_width,
            "dual_contact_sample_count": len(dual_contacts),
            "dual_contact_separation_min_m": min(separations),
            "dual_contact_separation_max_m": max(separations),
            "dual_contact_separation_median_m": statistics.median(separations),
            "calibration_link_frame_separation_m": calibration_separation,
            "combined_inner_face_offset_m": combined_offset,
            "symmetric_per_finger_inner_face_offset_m": combined_offset / 2.0,
            "estimated_uncertainty_m": uncertainty,
            "max_contact_depth_m": max_depth,
            "predicted_open_inner_face_opening_m": 0.080 - combined_offset,
            "predicted_closed_inner_face_opening_m": 0.022 - combined_offset,
            "formula": "inner_face_opening_m = link_frame_separation_m - combined_inner_face_offset_m",
        }
        if not 0.001 <= combined_offset <= 0.010:
            raise RuntimeError(f"implausible combined inner-face offset: {combined_offset}")
        if calibration["predicted_open_inner_face_opening_m"] <= args.gauge_width:
            raise RuntimeError("calibrated full-open inner-face opening does not clear the gauge")

        reopen_started = node.now_s()
        reopen_event = node.send_goal(
            label="reopen_after_contact",
            target=0.019,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(reopen_event)
        if not reopen_event["reached_goal"] or reopen_event["stalled"]:
            raise RuntimeError("gripper did not reopen after gauge contact")
        node.phase = "complete"
        node.command_target_m = None
        node.spin_for(0.8)
        late_contacts = [
            event for event in node.contact_messages
            if float(event["t_s"]) >= reopen_started + 0.30
        ]
        contact_cleared_after_reopen = not late_contacts
        if not contact_cleared_after_reopen:
            raise RuntimeError("gauge contact did not clear after reopening")
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        final_robot_state = node.latest_robot_state
        final_gauge_state = node.latest_gauge_state
        if calibration is not None:
            combined_offset = float(calibration["combined_inner_face_offset_m"])
            for sample in node.samples:
                separation = sample.get("link_frame_separation_m")
                sample["gauge_width_m"] = args.gauge_width
                sample["calibrated_combined_inner_face_offset_m"] = combined_offset
                sample["inner_face_opening_m"] = (
                    float(separation) - combined_offset if separation is not None else None
                )
        payload = {
            "schema_version": 1,
            "trial_type": "gazebo_static_gauge_inner_face_calibration",
            "hardware_backend": "gazebo_ros2_control",
            "gauge_model": gauge_model,
            "gauge_width_m": args.gauge_width,
            "gauge_pose_relative_to": args.reference_frame,
            "gauge_pose_xyz_m": [args.gauge_x, args.gauge_y, args.gauge_z],
            "spawn_result": spawn_result,
            "gauge_spawned": final_gauge_state is not None,
            "initial_open_contact": initial_contact,
            "gauge_contact_observed": bool(node.contact_messages),
            "left_gauge_contact_observed": any(
                event.get("left_contact") for event in node.contact_messages
            ),
            "right_gauge_contact_observed": any(
                event.get("right_contact") for event in node.contact_messages
            ),
            "dual_gauge_contact_observed": any(
                event.get("dual_contact") for event in node.contact_messages
            ),
            "contact_cleared_after_reopen": contact_cleared_after_reopen,
            "calibration": calibration,
            "inner_face_opening_calibrated": calibration is not None and not errors,
            "cube_spawned": False,
            "cube_contact": False,
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
            "final_gauge_state": final_gauge_state,
            "gauge_state_samples": node.gauge_samples,
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

    print(json.dumps({
        key: value for key, value in payload.items()
        if key not in {
            "samples", "link_state_samples", "robot_state_samples",
            "gauge_state_samples", "contact_messages",
        }
    }, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
