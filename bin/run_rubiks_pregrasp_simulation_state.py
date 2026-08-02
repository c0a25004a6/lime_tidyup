#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f"joint{i}" for i in range(1, 7)]
INITIAL_POSITION_TOL = 0.01
FINAL_POSITION_TOL = 0.01
HOLD_VELOCITY_TOL = 0.02
HOLD_RANGE_TOL = 0.005
OVERSHOOT_TOL = 0.02
MAX_FEEDBACK_ERROR = 0.08
MIN_FEEDBACK_SAMPLES = 5
MIN_HOLD_SAMPLES = 20


class EvidenceNode(Node):
    def __init__(self) -> None:
        super().__init__("rubiks_pregrasp_simulation_state_evidence")
        self.started = time.monotonic()
        self.samples: list[dict[str, Any]] = []
        self.feedback: list[dict[str, Any]] = []
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, qos_profile_sensor_data
        )
        self.action_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

    def _joint_state_callback(self, message: JointState) -> None:
        index = {name: i for i, name in enumerate(message.name)}
        if any(name not in index for name in JOINTS):
            return
        positions = [float(message.position[index[name]]) for name in JOINTS]
        velocities = None
        if len(message.velocity) == len(message.name):
            velocities = [float(message.velocity[index[name]]) for name in JOINTS]
        self.samples.append(
            {
                "wall_time_s": time.monotonic() - self.started,
                "ros_stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec),
                "positions_rad": positions,
                "velocities_rad_s": velocities,
            }
        )

    def feedback_callback(self, message: FollowJointTrajectory.FeedbackMessage) -> None:
        feedback = message.feedback
        event: dict[str, Any] = {
            "wall_time_s": time.monotonic() - self.started,
            "joint_names": list(feedback.joint_names),
        }
        for key, point in (
            ("desired", feedback.desired),
            ("actual", feedback.actual),
            ("error", feedback.error),
        ):
            event[key] = {
                "positions_rad": [float(value) for value in point.positions],
                "velocities_rad_s": [float(value) for value in point.velocities],
            }
        self.feedback.append(event)

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))

    def wait_for_joint_samples(self, minimum: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.samples) >= minimum:
                return True
        return False

    def send_goal(self, target: list[float], duration_s: float, timeout_s: float) -> dict[str, Any]:
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return {"accepted": False, "status": None, "error_code": None, "error_string": "action server unavailable"}

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = [0.0] * len(JOINTS)
        seconds = int(duration_s)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(round((duration_s - seconds) * 1_000_000_000))
        goal.trajectory.points = [point]
        goal.goal_time_tolerance.sec = 1

        feedback_start = len(self.feedback)
        sample_start = len(self.samples)
        sent_at = time.monotonic() - self.started
        send_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            return {"accepted": False, "status": None, "error_code": None, "error_string": "goal acceptance timeout"}
        handle = send_future.result()
        if not handle.accepted:
            return {"accepted": False, "status": None, "error_code": None, "error_string": "goal rejected"}

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        if not result_future.done() or result_future.result() is None:
            return {"accepted": True, "status": None, "error_code": None, "error_string": "goal result timeout"}
        wrapped = result_future.result()
        result = wrapped.result
        finished_at = time.monotonic() - self.started
        return {
            "accepted": True,
            "status": int(wrapped.status),
            "succeeded": wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and int(result.error_code) == int(FollowJointTrajectory.Result.SUCCESSFUL),
            "error_code": int(result.error_code),
            "error_string": str(result.error_string),
            "sent_at_s": sent_at,
            "finished_at_s": finished_at,
            "duration_s": finished_at - sent_at,
            "feedback_start_index": feedback_start,
            "feedback_end_index": len(self.feedback),
            "sample_start_index": sample_start,
            "sample_end_index": len(self.samples),
        }


def finite(values: list[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def phase_stats(samples: list[dict[str, Any]], target: list[float]) -> dict[str, Any]:
    if not samples:
        return {"sample_count": 0}
    positions = [sample["positions_rad"] for sample in samples]
    velocities = [sample["velocities_rad_s"] for sample in samples if sample["velocities_rad_s"] is not None]
    final = positions[-1]
    per_joint_range = [max(row[i] for row in positions) - min(row[i] for row in positions) for i in range(6)]
    errors = [[row[i] - target[i] for i in range(6)] for row in positions]
    return {
        "sample_count": len(samples),
        "velocity_sample_count": len(velocities),
        "final_positions_rad": final,
        "final_error_rad": [final[i] - target[i] for i in range(6)],
        "max_abs_final_error_rad": max(abs(final[i] - target[i]) for i in range(6)),
        "max_abs_error_rad": max(abs(value) for row in errors for value in row),
        "position_range_rad": per_joint_range,
        "max_position_range_rad": max(per_joint_range),
        "max_abs_velocity_rad_s": max((abs(value) for row in velocities for value in row), default=None),
        "all_finite": all(finite(row) for row in positions) and all(finite(row) for row in velocities),
    }


def feedback_stats(events: list[dict[str, Any]]) -> dict[str, Any]:
    errors: list[float] = []
    actual_velocities: list[float] = []
    valid_order_count = 0
    for event in events:
        if event.get("joint_names") == JOINTS:
            valid_order_count += 1
        error_positions = event.get("error", {}).get("positions_rad", [])
        actual_velocity = event.get("actual", {}).get("velocities_rad_s", [])
        if len(error_positions) == 6:
            errors.extend(abs(float(value)) for value in error_positions)
        if len(actual_velocity) == 6:
            actual_velocities.extend(abs(float(value)) for value in actual_velocity)
    return {
        "sample_count": len(events),
        "valid_joint_order_count": valid_order_count,
        "max_abs_position_error_rad": max(errors, default=None),
        "max_abs_actual_velocity_rad_s": max(actual_velocities, default=None),
    }


def overshoot_stats(samples: list[dict[str, Any]], start: list[float], target: list[float]) -> dict[str, Any]:
    overshoots: list[float] = []
    drifts: list[float] = []
    for i in range(6):
        values = [sample["positions_rad"][i] for sample in samples]
        if not values:
            overshoots.append(0.0)
            drifts.append(0.0)
            continue
        delta = target[i] - start[i]
        if delta > 1e-12:
            overshoots.append(max(0.0, max(values) - target[i]))
            drifts.append(0.0)
        elif delta < -1e-12:
            overshoots.append(max(0.0, target[i] - min(values)))
            drifts.append(0.0)
        else:
            overshoots.append(0.0)
            drifts.append(max(abs(value - start[i]) for value in values))
    return {
        "per_joint_overshoot_rad": overshoots,
        "max_overshoot_rad": max(overshoots),
        "per_joint_nonmoving_drift_rad": drifts,
        "max_nonmoving_drift_rad": max(drifts),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-json", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    parser.add_argument("--trajectory-seconds", type=float, default=3.0)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    args = parser.parse_args()

    search = json.loads(Path(args.candidate_json).read_text(encoding="utf-8"))
    selected = search.get("selected_candidate", {}).get("joint_positions_rad", {})
    candidate = [float(selected[name]) for name in JOINTS]
    zero = [0.0] * 6
    errors: list[str] = []
    raw: dict[str, Any] = {}

    rclpy.init()
    node = EvidenceNode()
    try:
        if not node.wait_for_joint_samples(30, 12.0):
            errors.append("insufficient initial joint-state samples")
        node.spin_for(1.0)
        initial_samples = node.samples[-max(MIN_HOLD_SAMPLES, min(len(node.samples), 200)) :]
        initial = phase_stats(initial_samples, zero)
        raw["initial"] = initial
        if initial.get("sample_count", 0) < MIN_HOLD_SAMPLES:
            errors.append("initial hold sample count is too low")
        if initial.get("max_abs_final_error_rad", math.inf) > INITIAL_POSITION_TOL:
            errors.append("initial arm state is not near zero")
        if initial.get("max_abs_velocity_rad_s") is None or initial["max_abs_velocity_rad_s"] > HOLD_VELOCITY_TOL:
            errors.append("initial arm state is not stationary")
        if initial.get("max_position_range_rad", math.inf) > HOLD_RANGE_TOL:
            errors.append("initial arm state is unstable")

        candidate_start_sample = len(node.samples)
        candidate_start_feedback = len(node.feedback)
        candidate_result = node.send_goal(candidate, args.trajectory_seconds, args.trajectory_seconds + 10.0)
        node.spin_for(args.hold_seconds)
        candidate_all_samples = node.samples[candidate_start_sample:]
        candidate_hold_samples = [sample for sample in candidate_all_samples if sample["wall_time_s"] >= candidate_result.get("finished_at_s", math.inf)]
        candidate_feedback = node.feedback[candidate_start_feedback:]
        raw["candidate_goal"] = candidate_result
        raw["candidate_feedback"] = feedback_stats(candidate_feedback)
        raw["candidate_transition"] = overshoot_stats(candidate_all_samples, zero, candidate)
        raw["candidate_hold"] = phase_stats(candidate_hold_samples, candidate)

        if candidate_result.get("succeeded") is not True:
            errors.append(f"candidate goal failed: {candidate_result}")
        if raw["candidate_feedback"]["sample_count"] < MIN_FEEDBACK_SAMPLES:
            errors.append("candidate feedback sample count is too low")
        if raw["candidate_feedback"]["valid_joint_order_count"] != raw["candidate_feedback"]["sample_count"]:
            errors.append("candidate feedback joint order mismatch")
        feedback_error = raw["candidate_feedback"].get("max_abs_position_error_rad")
        if feedback_error is None or feedback_error > MAX_FEEDBACK_ERROR:
            errors.append("candidate feedback tracking error exceeds limit")
        if raw["candidate_transition"]["max_overshoot_rad"] > OVERSHOOT_TOL:
            errors.append("candidate overshoot exceeds limit")
        if raw["candidate_transition"]["max_nonmoving_drift_rad"] > OVERSHOOT_TOL:
            errors.append("candidate nonmoving-joint drift exceeds limit")
        hold = raw["candidate_hold"]
        if hold.get("sample_count", 0) < MIN_HOLD_SAMPLES:
            errors.append("candidate hold sample count is too low")
        if hold.get("max_abs_final_error_rad", math.inf) > FINAL_POSITION_TOL:
            errors.append("candidate final position error exceeds limit")
        if hold.get("max_abs_velocity_rad_s") is None or hold["max_abs_velocity_rad_s"] > HOLD_VELOCITY_TOL:
            errors.append("candidate hold velocity exceeds limit")
        if hold.get("max_position_range_rad", math.inf) > HOLD_RANGE_TOL:
            errors.append("candidate hold stability exceeds limit")

        return_start_sample = len(node.samples)
        return_start_feedback = len(node.feedback)
        return_result = node.send_goal(zero, args.trajectory_seconds, args.trajectory_seconds + 10.0)
        node.spin_for(args.hold_seconds)
        return_all_samples = node.samples[return_start_sample:]
        return_hold_samples = [sample for sample in return_all_samples if sample["wall_time_s"] >= return_result.get("finished_at_s", math.inf)]
        return_feedback = node.feedback[return_start_feedback:]
        raw["return_goal"] = return_result
        raw["return_feedback"] = feedback_stats(return_feedback)
        raw["return_transition"] = overshoot_stats(return_all_samples, candidate, zero)
        raw["return_hold"] = phase_stats(return_hold_samples, zero)

        if return_result.get("succeeded") is not True:
            errors.append(f"return goal failed: {return_result}")
        if raw["return_feedback"]["sample_count"] < MIN_FEEDBACK_SAMPLES:
            errors.append("return feedback sample count is too low")
        if raw["return_feedback"]["valid_joint_order_count"] != raw["return_feedback"]["sample_count"]:
            errors.append("return feedback joint order mismatch")
        feedback_error = raw["return_feedback"].get("max_abs_position_error_rad")
        if feedback_error is None or feedback_error > MAX_FEEDBACK_ERROR:
            errors.append("return feedback tracking error exceeds limit")
        if raw["return_transition"]["max_overshoot_rad"] > OVERSHOOT_TOL:
            errors.append("return overshoot exceeds limit")
        if raw["return_transition"]["max_nonmoving_drift_rad"] > OVERSHOOT_TOL:
            errors.append("return nonmoving-joint drift exceeds limit")
        hold = raw["return_hold"]
        if hold.get("sample_count", 0) < MIN_HOLD_SAMPLES:
            errors.append("return hold sample count is too low")
        if hold.get("max_abs_final_error_rad", math.inf) > FINAL_POSITION_TOL:
            errors.append("return final position error exceeds limit")
        if hold.get("max_abs_velocity_rad_s") is None or hold["max_abs_velocity_rad_s"] > HOLD_VELOCITY_TOL:
            errors.append("return hold velocity exceeds limit")
        if hold.get("max_position_range_rad", math.inf) > HOLD_RANGE_TOL:
            errors.append("return hold stability exceeds limit")
    except Exception as error:
        errors.append(f"{type(error).__name__}: {error}")
    finally:
        raw["joint_state_sample_count"] = len(node.samples)
        raw["feedback_sample_count"] = len(node.feedback)
        raw["joint_state_samples"] = node.samples
        raw["feedback_samples"] = node.feedback
        node.destroy_node()
        rclpy.shutdown()

    telemetry = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE",
        "writer_lease": "WL-RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE-20260803-01",
        "source_ref": args.source_ref,
        "passed": not errors,
        "errors": errors,
        "joint_order": JOINTS,
        "candidate_joint_positions_rad": candidate,
        "trajectory_seconds": args.trajectory_seconds,
        "hold_seconds": args.hold_seconds,
        "thresholds": {
            "initial_position_tolerance_rad": INITIAL_POSITION_TOL,
            "final_position_tolerance_rad": FINAL_POSITION_TOL,
            "hold_velocity_tolerance_rad_s": HOLD_VELOCITY_TOL,
            "hold_range_tolerance_rad": HOLD_RANGE_TOL,
            "overshoot_tolerance_rad": OVERSHOOT_TOL,
            "maximum_feedback_error_rad": MAX_FEEDBACK_ERROR,
        },
        "evidence": raw,
        "safety": {
            "simulation_only": True,
            "physical_hardware_used": False,
            "empty_scene_only": True,
            "cube_spawned": False,
            "support_spawned": False,
            "fixture_spawned": False,
            "ifra_attachment_used": False,
            "grasp_fix_plugin_loaded": False,
            "gripper_controller_loaded": False,
            "gripper_command_sent": False,
            "base_controller_loaded": False,
            "base_command_sent": False,
            "arm_controller_loaded": True,
            "candidate_arm_goal_sent": True,
            "return_to_zero_goal_sent": True,
            "lift_command_sent": False,
            "grasp_success_claimed": False,
            "environment_clearance_claimed": False,
            "hardware_readiness_claimed": False,
            "production_runtime_modified": False,
        },
    }
    Path(args.output).write_text(json.dumps(telemetry, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(telemetry, indent=2, sort_keys=True))
    return 0 if telemetry["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
