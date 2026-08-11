#!/usr/bin/env python3
"""Test pure Gazebo grasp retention by removing support under a fixed arm.

The robot starts at the accepted anchor. The cube is closed on while supported,
then the static support model is deleted for a short bounded interval. The arm
never moves and no attachment is used. The trial ends with the gripper still
closed; it does not intentionally drop the cube after the unsupported interval.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity

import run_rubiks_dynamic_opposing_side_hold_v2  # noqa: F401
from rubiks_contact_quality import audit_contact_quality
from run_gripper_gauge_calibration import speed
from run_rubiks_dynamic_supported_hold import (
    DynamicSupportedCubeNode,
    movement_metrics,
    ratio,
)

OPEN_M = 0.019


class UnsupportedRetentionNode(DynamicSupportedCubeNode):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")
        self.latest_models: set[str] = set()

    def on_model_states(self, msg: ModelStates) -> None:
        self.latest_models = set(str(name) for name in msg.name)
        super().on_model_states(msg)

    def delete_model(self, name: str, timeout: float) -> dict[str, object]:
        if not self.delete_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/delete_entity service unavailable")
        request = DeleteEntity.Request()
        request.name = name
        future = self.delete_client.call_async(request)
        self.wait_for(lambda: future.done(), timeout, f"delete {name} response")
        response = future.result()
        if response is None or not response.success:
            status = response.status_message if response is not None else "no response"
            raise RuntimeError(f"delete {name} failed: {status}")
        self.wait_for(
            lambda: name not in self.latest_models,
            timeout,
            f"{name} removal from /model_states",
        )
        return {"success": True, "status_message": str(response.status_message)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cube-sdf", required=True)
    parser.add_argument("--support-sdf", required=True)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--supported-hold-seconds", type=float, default=1.0)
    parser.add_argument("--unsupported-seconds", type=float, default=1.0)
    parser.add_argument("--close-start", type=float, default=0.0185)
    parser.add_argument("--close-stop", type=float, default=0.0060)
    parser.add_argument("--close-step", type=float, default=0.00025)
    parser.add_argument("--step-hold", type=float, default=0.12)
    parser.add_argument("--max-effort", type=float, default=0.25)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    input_data = json.loads(Path(args.input).read_text(encoding="utf-8"))
    cube_xyz = tuple(float(v) for v in input_data["cube_effective_offset_link7_m"])
    support_xyz = tuple(float(v) for v in input_data["support_effective_offset_link7_m"])
    if len(cube_xyz) != 3 or len(support_xyz) != 3:
        raise SystemExit("invalid fixture geometry")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    robot_model = "turtlebot3_lime_gripper_test"
    cube_model = "rubiks_dynamic_cube_supported_057"
    support_model = "rubiks_cube_support_surface"

    rclpy.init(args=None)
    node = UnsupportedRetentionNode(
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
    supported_contact_quality = None
    unsupported_contact_quality = None
    supported_hold_result = None
    unsupported_result = None
    support_delete_result = None
    support_contact_before_close = False
    support_removed = False
    unsupported_start = None
    unsupported_end = None

    try:
        node.spin_for(1.0)
        if node.latest_joint is None or node.latest_link is None:
            raise RuntimeError("initial gripper telemetry unavailable")

        open_event = node.send_goal(
            label="unsupported_open_before_fixture",
            target=OPEN_M,
            max_effort=0.5,
            timeout=args.timeout,
            monitor_contact=False,
        )
        events.append(open_event)
        if not open_event["reached_goal"] or open_event["stalled"]:
            raise RuntimeError("gripper did not establish fully open state")

        node.phase = "unsupported_spawn_support"
        node.spawn_named_model(
            model_name=support_model,
            sdf_text=Path(args.support_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_cube_support",
            reference_frame=f"{robot_model}::link7",
            pose_xyz=support_xyz,
            timeout=args.timeout,
        )
        node.phase = "unsupported_spawn_cube"
        node.spawn_named_model(
            model_name=cube_model,
            sdf_text=Path(args.cube_sdf).read_text(encoding="utf-8"),
            namespace="/rubiks_dynamic_cube_hold",
            reference_frame=f"{robot_model}::link7",
            pose_xyz=cube_xyz,
            timeout=args.timeout,
        )
        node.wait_for(lambda: node.latest_gauge_state is not None, args.timeout, "cube state")
        node.wait_for(lambda: support_model in node.latest_models, args.timeout, "support model state")

        node.phase = "unsupported_settle_on_support"
        settle_start = node.now_s()
        node.spin_for(args.settle_seconds)
        settle_end = node.now_s()
        settled = dict(node.latest_gauge_state) if node.latest_gauge_state else None
        if settled is None:
            raise RuntimeError("cube state missing after support settle")
        settled_speed = speed(settled)
        if settled_speed is None or settled_speed > 0.02:
            raise RuntimeError(f"cube did not settle on support: {settled_speed}")
        support_contact_before_close = any(
            settle_start <= float(event["t_s"]) <= settle_end
            for event in node.support_contact_messages
        )
        if not support_contact_before_close:
            raise RuntimeError("support contact absent before close")
        if node.contact_messages or node.unexpected_robot_contacts:
            raise RuntimeError("robot-cube contact exists while gripper is open")

        target = args.close_start
        step_index = 0
        bilateral_event = None
        while target >= args.close_stop - 1e-12:
            event = node.send_goal(
                label=f"opposing_side_close_step_unsupported_{step_index:02d}",
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
            raise RuntimeError("no bilateral contact in bounded close")

        node.phase = "unsupported_prewithdraw_supported_hold"
        node.command_target_m = bilateral_event["target_position_m"]
        supported_start = node.now_s()
        node.spin_for(args.supported_hold_seconds)
        supported_end = node.now_s()
        supported_contacts = [
            event for event in node.contact_messages
            if supported_start <= float(event["t_s"]) <= supported_end
        ]
        supported_contact_quality = audit_contact_quality(
            supported_contacts,
            node.gauge_samples,
            start_s=supported_start,
            end_s=supported_end,
        )
        if supported_contact_quality.get("passed") is not True:
            raise RuntimeError("supported pre-withdraw contact quality failed")
        supported_support_ratio = ratio(
            node.samples, supported_start, supported_end, "support_contact_active"
        )
        supported_dual_ratio = ratio(
            node.samples, supported_start, supported_end, "dual_finger_contact_active"
        )
        if supported_support_ratio < 0.75 or supported_dual_ratio < 0.65:
            raise RuntimeError(
                f"supported pre-withdraw ratios failed: support={supported_support_ratio}, dual={supported_dual_ratio}"
            )
        if node.unexpected_robot_contacts:
            raise RuntimeError("non-finger robot-cube contact before support removal")
        supported_hold_result = {
            "start_s": supported_start,
            "end_s": supported_end,
            "support_contact_ratio": supported_support_ratio,
            "bilateral_finger_contact_ratio": supported_dual_ratio,
        }

        prewithdraw_state = dict(node.latest_gauge_state) if node.latest_gauge_state else None
        if prewithdraw_state is None:
            raise RuntimeError("cube state missing before support removal")

        node.phase = "unsupported_delete_support"
        support_delete_result = node.delete_model(support_model, args.timeout)
        support_removed = True
        unsupported_start = node.now_s()
        node.phase = "unsupported_gravity_retention"
        node.spin_for(args.unsupported_seconds)
        unsupported_end = node.now_s()

        unsupported_contacts = [
            event for event in node.contact_messages
            if unsupported_start <= float(event["t_s"]) <= unsupported_end
        ]
        unsupported_contact_quality = audit_contact_quality(
            unsupported_contacts,
            node.gauge_samples,
            start_s=unsupported_start,
            end_s=unsupported_end,
        )
        if unsupported_contact_quality.get("passed") is not True:
            raise RuntimeError("unsupported moving-cube contact quality failed")
        unsupported_dual_ratio = ratio(
            node.samples,
            unsupported_start,
            unsupported_end,
            "dual_finger_contact_active",
        )
        if unsupported_dual_ratio < 0.65:
            raise RuntimeError(
                f"bilateral contact coverage too low without support: {unsupported_dual_ratio}"
            )
        if node.unexpected_robot_contacts:
            raise RuntimeError("non-finger robot-cube contact during unsupported retention")

        unsupported_states = [
            state for state in node.gauge_samples
            if unsupported_start <= float(state["t_s"]) <= unsupported_end
        ]
        movement = movement_metrics(unsupported_states, prewithdraw_state)
        if float(movement["max_horizontal_displacement_m"]) > 0.003:
            raise RuntimeError(f"unsupported horizontal motion too large: {movement}")
        if float(movement["max_abs_vertical_displacement_m"]) > 0.003:
            raise RuntimeError(f"unsupported vertical motion too large: {movement}")
        if float(movement["max_rotation_rad"]) > 0.10:
            raise RuntimeError(f"unsupported rotation too large: {movement}")
        final_state = node.latest_gauge_state
        final_speed = speed(final_state)
        if final_speed is None or final_speed > 0.02:
            raise RuntimeError(f"cube not stably retained after support removal: {final_speed}")

        unsupported_result = {
            "requested_seconds": args.unsupported_seconds,
            "observed_seconds": unsupported_end - unsupported_start,
            "bilateral_finger_contact_ratio": unsupported_dual_ratio,
            "movement": movement,
            "final_cube_speed_m_s": final_speed,
            "support_model_absent": support_model not in node.latest_models,
        }
        if unsupported_result["support_model_absent"] is not True:
            raise RuntimeError("support model reappeared during unsupported interval")
    except Exception as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        payload = {
            "schema_version": 1,
            "phase": "RUBIK-FIXED-ARM-UNSUPPORTED-GRAVITY-RETENTION",
            "input": input_data,
            "support_contact_before_close": support_contact_before_close,
            "supported_hold_result": supported_hold_result,
            "supported_contact_quality": supported_contact_quality,
            "support_delete_result": support_delete_result,
            "support_removed": support_removed,
            "unsupported_start_s": unsupported_start,
            "unsupported_end_s": unsupported_end,
            "unsupported_result": unsupported_result,
            "unsupported_contact_quality": unsupported_contact_quality,
            "arm_motion_performed": False,
            "arm_command_sent": False,
            "base_command_sent": False,
            "lift_command_sent": False,
            "reopen_after_support_removal": False,
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
