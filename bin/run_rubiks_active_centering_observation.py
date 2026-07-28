#!/usr/bin/env python3
"""Observe cube pose relative to link7 and emit a command-free centering proposal."""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import statistics
import time
from typing import Iterable

import rclpy
from gazebo_msgs.msg import ContactsState, LinkStates, ModelStates
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def vector3(x: float, y: float, z: float) -> Vector3:
    return float(x), float(y), float(z)


def quaternion(x: float, y: float, z: float, w: float) -> Quaternion:
    values = (float(x), float(y), float(z), float(w))
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError("zero-length quaternion")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def quat_conjugate(q: Quaternion) -> Quaternion:
    return -q[0], -q[1], -q[2], q[3]


def quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q: Quaternion, value: Vector3) -> Vector3:
    pure: Quaternion = (value[0], value[1], value[2], 0.0)
    rotated = quat_multiply(quat_multiply(q, pure), quat_conjugate(q))
    return rotated[0], rotated[1], rotated[2]


def relative_pose(
    reference_position: Vector3,
    reference_orientation: Quaternion,
    target_position: Vector3,
    target_orientation: Quaternion,
) -> tuple[Vector3, Quaternion]:
    inverse = quat_conjugate(reference_orientation)
    delta = (
        target_position[0] - reference_position[0],
        target_position[1] - reference_position[1],
        target_position[2] - reference_position[2],
    )
    return rotate_vector(inverse, delta), quaternion(*quat_multiply(inverse, target_orientation))


def yaw_from_quaternion(q: Quaternion) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quaternion(yaw: float) -> Quaternion:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def magnitude(values: Iterable[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def median_angle(values: list[float]) -> float:
    if not values:
        raise ValueError("cannot take median of no angles")
    sine = statistics.median(math.sin(value) for value in values)
    cosine = statistics.median(math.cos(value) for value in values)
    return math.atan2(sine, cosine)


class ObservationNode(Node):
    def __init__(
        self,
        *,
        robot_model: str,
        link_name: str,
        cube_model: str,
        support_model: str,
        contact_topic: str,
    ) -> None:
        super().__init__("rubiks_active_centering_observation")
        self.robot_model = robot_model
        self.link_name = link_name
        self.cube_model = cube_model
        self.support_model = support_model
        self.started = time.monotonic()
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.latest_link: dict | None = None
        self.latest_cube: dict | None = None
        self.latest_support: dict | None = None
        self.relative_samples: deque[dict] = deque(maxlen=1000)
        self.contact_messages: list[dict] = []
        self.robot_contact_messages: list[dict] = []
        self.support_contact_messages: list[dict] = []
        self.create_subscription(LinkStates, "/link_states", self.on_link_states, 100)
        self.create_subscription(ModelStates, "/model_states", self.on_model_states, 100)
        self.create_subscription(ContactsState, contact_topic, self.on_contacts, 100)

    def now_s(self) -> float:
        return time.monotonic() - self.started

    @staticmethod
    def pose_record(pose, twist, t_s: float) -> dict:
        return {
            "t_s": t_s,
            "position_m": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
            "orientation_xyzw": [
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ],
            "linear_velocity_m_s": [
                float(twist.linear.x), float(twist.linear.y), float(twist.linear.z)
            ],
            "angular_velocity_rad_s": [
                float(twist.angular.x), float(twist.angular.y), float(twist.angular.z)
            ],
        }

    def on_link_states(self, msg: LinkStates) -> None:
        try:
            index = msg.name.index(self.link_name)
        except ValueError:
            return
        pose = msg.pose[index]
        twist = msg.twist[index]
        self.latest_link = self.pose_record(pose, twist, self.now_s())
        self.record_relative_sample()

    def on_model_states(self, msg: ModelStates) -> None:
        t_s = self.now_s()
        for name, attribute in (
            (self.cube_model, "latest_cube"),
            (self.support_model, "latest_support"),
        ):
            try:
                index = msg.name.index(name)
            except ValueError:
                continue
            setattr(self, attribute, self.pose_record(msg.pose[index], msg.twist[index], t_s))
        self.record_relative_sample()

    def record_relative_sample(self) -> None:
        if self.latest_link is None or self.latest_cube is None:
            return
        link_position = vector3(*self.latest_link["position_m"])
        link_orientation = quaternion(*self.latest_link["orientation_xyzw"])
        cube_position = vector3(*self.latest_cube["position_m"])
        cube_orientation = quaternion(*self.latest_cube["orientation_xyzw"])
        position, orientation = relative_pose(
            link_position, link_orientation, cube_position, cube_orientation
        )
        sample = {
            "t_s": self.now_s(),
            "relative_position_m": list(position),
            "relative_orientation_xyzw": list(orientation),
            "relative_yaw_rad": yaw_from_quaternion(orientation),
            "cube_linear_speed_m_s": magnitude(self.latest_cube["linear_velocity_m_s"]),
            "cube_angular_speed_rad_s": magnitude(self.latest_cube["angular_velocity_rad_s"]),
            "link_linear_speed_m_s": magnitude(self.latest_link["linear_velocity_m_s"]),
            "link_angular_speed_rad_s": magnitude(self.latest_link["angular_velocity_rad_s"]),
        }
        if not self.relative_samples or sample["t_s"] > self.relative_samples[-1]["t_s"] + 1e-5:
            self.relative_samples.append(sample)

    def on_contacts(self, msg: ContactsState) -> None:
        if not msg.states:
            return
        for state in msg.states:
            collision1 = str(state.collision1_name)
            collision2 = str(state.collision2_name)
            pair = f"{collision1} {collision2}"
            record = {
                "t_s": self.now_s(),
                "collision1": collision1,
                "collision2": collision2,
                "robot_contact": self.robot_model in pair,
                "support_contact": self.support_model in pair or "support_collision" in pair,
            }
            self.contact_messages.append(record)
            if record["robot_contact"]:
                self.robot_contact_messages.append(record)
            if record["support_contact"]:
                self.support_contact_messages.append(record)

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

    def spawn(
        self,
        *,
        name: str,
        sdf_text: str,
        namespace: str,
        reference_frame: str,
        xyz: Vector3,
        yaw: float,
        timeout: float,
    ) -> dict:
        if not self.spawn_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/spawn_entity service was not available")
        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf_text
        request.robot_namespace = namespace
        request.reference_frame = reference_frame
        request.initial_pose = Pose()
        request.initial_pose.position.x = xyz[0]
        request.initial_pose.position.y = xyz[1]
        request.initial_pose.position.z = xyz[2]
        q = yaw_quaternion(yaw)
        request.initial_pose.orientation.x = q[0]
        request.initial_pose.orientation.y = q[1]
        request.initial_pose.orientation.z = q[2]
        request.initial_pose.orientation.w = q[3]
        future = self.spawn_client.call_async(request)
        self.wait_for(lambda: future.done(), timeout, f"{name} spawn response")
        response = future.result()
        if response is None or not response.success:
            message = response.status_message if response is not None else "no response"
            raise RuntimeError(f"{name} spawn failed: {message}")
        return {"success": True, "status_message": str(response.status_message)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cube-sdf", required=True)
    parser.add_argument("--support-sdf", required=True)
    parser.add_argument("--requested-y", type=float, required=True)
    parser.add_argument("--requested-yaw", type=float, required=True)
    parser.add_argument("--expected-decision", choices=("accept", "reject"), required=True)
    parser.add_argument("--max-lateral", type=float, default=0.001)
    parser.add_argument("--max-yaw", type=float, default=math.radians(2.0))
    parser.add_argument("--reference-frame", default="turtlebot3_lime_gripper_test::link7")
    parser.add_argument("--fixture-x", type=float, default=0.25)
    parser.add_argument("--support-z", type=float, default=0.10)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--observe-seconds", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    errors: list[str] = []
    robot_model = "turtlebot3_lime_gripper_test"
    cube_model = "rubiks_active_centering_cube"
    support_model = "rubiks_active_centering_support"
    cube_z = args.support_z + 0.005 + 0.0285 + 0.008

    rclpy.init(args=None)
    node = ObservationNode(
        robot_model=robot_model,
        link_name=args.reference_frame,
        cube_model=cube_model,
        support_model=support_model,
        contact_topic="/rubiks_dynamic_cube_hold/contact_states",
    )
    support_spawn: dict | None = None
    cube_spawn: dict | None = None
    try:
        node.wait_for(lambda: node.latest_link is not None, args.timeout, "link7 state")
        initial_link = dict(node.latest_link) if node.latest_link else None
        support_spawn = node.spawn(
            name=support_model,
            sdf_text=Path(args.support_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_active_centering_support",
            reference_frame=args.reference_frame,
            xyz=(args.fixture_x, 0.0, args.support_z),
            yaw=0.0,
            timeout=args.timeout,
        )
        cube_spawn = node.spawn(
            name=cube_model,
            sdf_text=Path(args.cube_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_dynamic_cube_hold",
            reference_frame=args.reference_frame,
            xyz=(args.fixture_x, args.requested_y, cube_z),
            yaw=args.requested_yaw,
            timeout=args.timeout,
        )
        node.wait_for(
            lambda: node.latest_cube is not None and node.latest_support is not None,
            args.timeout,
            "cube and support states",
        )
        node.spin_for(args.settle_seconds)
        observation_start = node.now_s()
        node.spin_for(args.observe_seconds)
        observation_end = node.now_s()
        final_link = dict(node.latest_link) if node.latest_link else None
        final_cube = dict(node.latest_cube) if node.latest_cube else None

        samples = [
            sample
            for sample in node.relative_samples
            if observation_start <= float(sample["t_s"]) <= observation_end
        ]
        if len(samples) < 20:
            raise RuntimeError(f"insufficient relative-pose samples: {len(samples)}")
        lateral_values = [float(sample["relative_position_m"][1]) for sample in samples]
        yaw_values = [float(sample["relative_yaw_rad"]) for sample in samples]
        measured_y = statistics.median(lateral_values)
        measured_yaw = median_angle(yaw_values)
        max_cube_speed = max(float(sample["cube_linear_speed_m_s"]) for sample in samples)
        max_cube_angular_speed = max(
            float(sample["cube_angular_speed_rad_s"]) for sample in samples
        )
        pose_error_y = measured_y - args.requested_y
        pose_error_yaw = wrap_angle(measured_yaw - args.requested_yaw)
        within_lateral = abs(measured_y) <= args.max_lateral
        within_yaw = abs(measured_yaw) <= args.max_yaw
        accepted = within_lateral and within_yaw
        decision = "accept" if accepted else "reject"
        reasons: list[str] = []
        if not within_lateral:
            reasons.append("lateral correction exceeds configured bound")
        if not within_yaw:
            reasons.append("yaw correction exceeds configured bound")

        proposal = {
            "decision": decision,
            "command_authorized": False,
            "proposed_link7_lateral_shift_m": measured_y if accepted else None,
            "proposed_link7_yaw_rotation_rad": measured_yaw if accepted else None,
            "preview_saturated_lateral_shift_m": max(
                -args.max_lateral, min(args.max_lateral, measured_y)
            ),
            "preview_saturated_yaw_rotation_rad": max(
                -args.max_yaw, min(args.max_yaw, measured_yaw)
            ),
            "predicted_residual_lateral_m": 0.0 if accepted else measured_y,
            "predicted_residual_yaw_rad": 0.0 if accepted else measured_yaw,
            "reasons": reasons,
        }

        if decision != args.expected_decision:
            errors.append(
                f"decision mismatch: expected {args.expected_decision}, observed {decision}"
            )
        if abs(pose_error_y) > 0.00015:
            errors.append(f"relative Y observation error too large: {pose_error_y}")
        if abs(pose_error_yaw) > math.radians(0.15):
            errors.append(f"relative yaw observation error too large: {pose_error_yaw}")
        if max_cube_speed > 0.02:
            errors.append(f"cube did not settle: max speed {max_cube_speed}")
        if max_cube_angular_speed > 0.2:
            errors.append(
                f"cube did not settle: max angular speed {max_cube_angular_speed}"
            )
        if not node.support_contact_messages:
            errors.append("support contact was not observed")
        if node.robot_contact_messages:
            errors.append("cube contacted the robot during observation")

        if initial_link is None or final_link is None or final_cube is None:
            errors.append("final pose evidence is incomplete")
            link_displacement = None
        else:
            link_displacement = math.dist(
                initial_link["position_m"], final_link["position_m"]
            )
            if link_displacement > 0.005:
                errors.append(f"link7 moved unexpectedly: {link_displacement}")

        payload = {
            "schema_version": 1,
            "trial_type": "gazebo_active_centering_observation_dry_run",
            "observation_source": "Gazebo /model_states and /link_states ground truth",
            "simulation_oracle_only": True,
            "production_pose_estimator_claimed": False,
            "robot_model": robot_model,
            "reference_link": args.reference_frame,
            "cube_model": cube_model,
            "support_model": support_model,
            "requested_relative_y_m": args.requested_y,
            "requested_relative_yaw_rad": args.requested_yaw,
            "expected_decision": args.expected_decision,
            "measured_relative_y_m": measured_y,
            "measured_relative_yaw_rad": measured_yaw,
            "observation_error_y_m": pose_error_y,
            "observation_error_yaw_rad": pose_error_yaw,
            "maximum_lateral_correction_m": args.max_lateral,
            "maximum_yaw_correction_rad": args.max_yaw,
            "proposal": proposal,
            "sample_count": len(samples),
            "observation_duration_s": observation_end - observation_start,
            "maximum_cube_linear_speed_m_s": max_cube_speed,
            "maximum_cube_angular_speed_rad_s": max_cube_angular_speed,
            "support_contact_observed": bool(node.support_contact_messages),
            "robot_contact_observed": bool(node.robot_contact_messages),
            "link7_displacement_m": link_displacement,
            "support_spawn": support_spawn,
            "cube_spawn": cube_spawn,
            "initial_link_state": initial_link,
            "final_link_state": final_link,
            "final_cube_state": final_cube,
            "relative_samples": samples,
            "contact_messages": node.contact_messages,
            "passed": not errors,
            "errors": errors,
            "arm_controller_loaded": False,
            "arm_command_sent": False,
            "gripper_controller_loaded": False,
            "gripper_command_sent": False,
            "support_removed": False,
            "lift_command_sent": False,
            "cube_lifted": False,
            "ifra_attachment_used": False,
            "grasp_success_claimed": False,
        }
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
        payload = {
            "schema_version": 1,
            "trial_type": "gazebo_active_centering_observation_dry_run",
            "simulation_oracle_only": True,
            "passed": False,
            "errors": errors,
            "requested_relative_y_m": args.requested_y,
            "requested_relative_yaw_rad": args.requested_yaw,
            "expected_decision": args.expected_decision,
            "support_spawn": support_spawn,
            "cube_spawn": cube_spawn,
            "arm_controller_loaded": False,
            "arm_command_sent": False,
            "gripper_controller_loaded": False,
            "gripper_command_sent": False,
            "support_removed": False,
            "lift_command_sent": False,
            "cube_lifted": False,
            "ifra_attachment_used": False,
            "grasp_success_claimed": False,
        }
    finally:
        output_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps({key: value for key, value in payload.items() if key not in {"relative_samples", "contact_messages"}}, indent=2))
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
