#!/usr/bin/env python3
"""Retry unsupported retention with the reviewed q=0.00675 supported preload.

This wrapper preserves the PR #47 support-removal physics and all acceptance
thresholds. It changes only the close stopping point: bilateral contact at
q=0.00725 and q=0.00700 remains a bounded precursor, and the trial begins its
supported hold only when the commanded close target reaches q=0.00675.

The reviewed 1 mm global penetration ceiling is also enforced on each fresh
audit. If the supported preload exceeds it, the parent trial fails before the
support is deleted; the same ceiling remains active during unsupported hold.
"""
from __future__ import annotations

import run_rubiks_dynamic_opposing_side_hold_v2  # noqa: F401
from run_rubiks_dynamic_supported_hold import DynamicSupportedCubeNode
import run_rubiks_unsupported_retention as implementation

SELECTED_PRELOAD_M = 0.00675
MAX_GLOBAL_DEPTH_M = 0.001
_BASE_SEND_GOAL = DynamicSupportedCubeNode.send_goal
_BASE_AUDIT = implementation.audit_contact_quality


def _send_goal_to_reviewed_preload(
    self,
    *,
    label: str,
    target: float,
    max_effort: float,
    timeout: float,
    monitor_contact: bool,
    contact_settle_s: float = 0.15,
):
    event = _BASE_SEND_GOAL(
        self,
        label=label,
        target=target,
        max_effort=max_effort,
        timeout=timeout,
        monitor_contact=monitor_contact,
        contact_settle_s=contact_settle_s,
    )
    if label.startswith("opposing_side_close_step_unsupported_"):
        dual = bool(event.get("contact_observed"))
        event["reviewed_preload_target_m"] = SELECTED_PRELOAD_M
        if dual and target > SELECTED_PRELOAD_M + 1e-12:
            event["bilateral_preload_precursor"] = True
            event["contact_observed"] = False
        elif dual:
            event["bilateral_preload_precursor"] = False
            event["preload_target_reached"] = True
    return event


def _audit_with_global_depth_limit(*args, **kwargs):
    result = _BASE_AUDIT(*args, **kwargs)
    left_depth = float((result.get("left") or {}).get("max_depth_m", 999.0))
    right_depth = float((result.get("right") or {}).get("max_depth_m", 999.0))
    global_depth_passed = (
        left_depth <= MAX_GLOBAL_DEPTH_M and right_depth <= MAX_GLOBAL_DEPTH_M
    )
    result["global_max_depth_limit_m"] = MAX_GLOBAL_DEPTH_M
    result["global_depth_passed"] = global_depth_passed
    if not global_depth_passed:
        result["passed"] = False
    return result


DynamicSupportedCubeNode.send_goal = _send_goal_to_reviewed_preload
implementation.audit_contact_quality = _audit_with_global_depth_limit

if __name__ == "__main__":
    raise SystemExit(implementation.main())
