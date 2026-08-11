#!/usr/bin/env python3
"""Run the opposing-side hold gate while treating unilateral contact as a bounded precursor.

Acceptance criteria are unchanged. During the close sweep only, a completed
0.25 mm step with unilateral cube contact does not end the sweep. The sweep
continues within the existing 0.0185 -> 0.0060 m domain until bilateral contact
is observed. No step is allowed to exceed the existing target domain/effort.
"""
from __future__ import annotations

from run_gripper_gauge_calibration import GaugeCalibrationNode
import run_rubiks_dynamic_opposing_side_hold as implementation

_BASE_SEND_GOAL = GaugeCalibrationNode.send_goal


def _send_goal_with_unilateral_precursor(
    self,
    *,
    label: str,
    target: float,
    max_effort: float,
    timeout: float,
    monitor_contact: bool,
    contact_settle_s: float = 0.15,
):
    close_step = label.startswith("opposing_side_close_step_")
    event = _BASE_SEND_GOAL(
        self,
        label=label,
        target=target,
        max_effort=max_effort,
        timeout=timeout,
        # The base helper cancels after any first contact. For this geometry the
        # right finger contacts first, while the exact-mesh dual-contact state
        # lies several bounded 0.25 mm steps later. Let completed close steps
        # finish; the outer gate still stops immediately once dual is observed.
        monitor_contact=False if close_step else monitor_contact,
        contact_settle_s=contact_settle_s,
    )
    if close_step:
        raw_contact = bool(event.get("contact_observed"))
        dual = bool(event.get("dual_contact_observed"))
        event["raw_contact_observed"] = raw_contact
        event["unilateral_precursor_contact"] = raw_contact and not dual
        # The v1 outer loop advances on contact_observed. Expose only bilateral
        # contact to that control decision while retaining raw evidence above.
        event["contact_observed"] = dual
    return event


implementation.DynamicSupportedCubeNode.send_goal = _send_goal_with_unilateral_precursor

if __name__ == "__main__":
    raise SystemExit(implementation.main())
