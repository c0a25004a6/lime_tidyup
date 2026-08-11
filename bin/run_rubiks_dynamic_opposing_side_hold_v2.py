#!/usr/bin/env python3
"""Run the opposing-side hold gate with bounded precursor contact and robust readiness.

Acceptance criteria are unchanged. During the close sweep only, a completed
0.25 mm step with unilateral cube contact does not end the sweep. The first
observation window is also extended from 1 s to 3 s so DDS/Gazebo subscription
startup latency is not misclassified as a physical repeatability failure.
"""
from __future__ import annotations

from run_gripper_gauge_calibration import GaugeCalibrationNode
import run_rubiks_dynamic_opposing_side_hold as implementation

_BASE_SEND_GOAL = GaugeCalibrationNode.send_goal
_BASE_SPIN_FOR = implementation.DynamicSupportedCubeNode.spin_for


def _spin_for_with_initial_readiness(self, seconds: float) -> None:
    first = not bool(getattr(self, "_opposing_side_initial_spin_completed", False))
    if first:
        self._opposing_side_initial_spin_completed = True
        _BASE_SPIN_FOR(self, max(float(seconds), 3.0))
        return
    _BASE_SPIN_FOR(self, seconds)


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
        monitor_contact=False if close_step else monitor_contact,
        contact_settle_s=contact_settle_s,
    )
    if close_step:
        raw_contact = bool(event.get("contact_observed"))
        dual = bool(event.get("dual_contact_observed"))
        event["raw_contact_observed"] = raw_contact
        event["unilateral_precursor_contact"] = raw_contact and not dual
        event["contact_observed"] = dual
    return event


implementation.DynamicSupportedCubeNode.spin_for = _spin_for_with_initial_readiness
implementation.DynamicSupportedCubeNode.send_goal = _send_goal_with_unilateral_precursor

if __name__ == "__main__":
    raise SystemExit(implementation.main())
