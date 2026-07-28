#!/usr/bin/env python3
"""Aggregate active-centering pose-observation dry-run trials."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

TRIALS = (
    ("center", 0.0, 0.0, "accept"),
    ("minus_y", -0.0005, 0.0, "accept"),
    ("plus_y", 0.0005, 0.0, "accept"),
    ("minus_yaw", 0.0, -math.radians(1.0), "accept"),
    ("plus_yaw", 0.0, math.radians(1.0), "accept"),
    ("mixed", 0.0005, -math.radians(1.0), "accept"),
    ("reject_y", 0.0015, 0.0, "reject"),
    ("reject_yaw", 0.0, math.radians(3.0), "reject"),
)


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected object in {path}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    args = parser.parse_args()

    root = Path(args.root)
    errors: list[str] = []
    records: list[dict[str, Any]] = []
    y_errors: list[float] = []
    yaw_errors: list[float] = []

    for label, requested_y, requested_yaw, expected_decision in TRIALS:
        trial_errors: list[str] = []
        trial_root = root / label
        try:
            summary = load(trial_root / "active_centering_observation_summary.json")
        except (FileNotFoundError, json.JSONDecodeError, ValueError) as exc:
            summary = {}
            trial_errors.append(f"missing or invalid summary: {exc}")

        if summary:
            if summary.get("passed") is not True:
                trial_errors.append("trial summary did not pass")
            trial_errors.extend(str(error) for error in summary.get("errors", []))
            if not math.isclose(
                float(summary.get("requested_relative_y_m", math.inf)),
                requested_y,
                abs_tol=1e-12,
            ):
                trial_errors.append("requested Y does not match matrix")
            if not math.isclose(
                float(summary.get("requested_relative_yaw_rad", math.inf)),
                requested_yaw,
                abs_tol=1e-12,
            ):
                trial_errors.append("requested yaw does not match matrix")
            if summary.get("expected_decision") != expected_decision:
                trial_errors.append("expected decision does not match matrix")
            proposal = summary.get("proposal") or {}
            if proposal.get("decision") != expected_decision:
                trial_errors.append("observed decision does not match matrix")
            if proposal.get("command_authorized") is not False:
                trial_errors.append("dry-run authorized a command")
            if summary.get("simulation_oracle_only") is not True:
                trial_errors.append("simulation oracle boundary is missing")
            for key in (
                "arm_command_sent",
                "gripper_command_sent",
                "support_removed",
                "lift_command_sent",
                "ifra_attachment_used",
                "grasp_success_claimed",
            ):
                if summary.get(key) is not False:
                    trial_errors.append(f"{key} must remain false")
            y_error = float(summary.get("observation_error_y_m", math.inf))
            yaw_error = float(summary.get("observation_error_yaw_rad", math.inf))
            if not math.isfinite(y_error) or abs(y_error) > 0.00015:
                trial_errors.append("Y observation error exceeds 0.15 mm")
            else:
                y_errors.append(abs(y_error))
            if not math.isfinite(yaw_error) or abs(yaw_error) > math.radians(0.15):
                trial_errors.append("yaw observation error exceeds 0.15 degrees")
            else:
                yaw_errors.append(abs(yaw_error))

            if expected_decision == "accept":
                lateral = proposal.get("proposed_link7_lateral_shift_m")
                yaw = proposal.get("proposed_link7_yaw_rotation_rad")
                if lateral is None or yaw is None:
                    trial_errors.append("accepted case lacks a correction proposal")
                else:
                    if abs(float(lateral) - float(summary["measured_relative_y_m"])) > 1e-12:
                        trial_errors.append("lateral proposal does not match observed error")
                    if abs(float(yaw) - float(summary["measured_relative_yaw_rad"])) > 1e-12:
                        trial_errors.append("yaw proposal does not match observed error")
            else:
                if proposal.get("proposed_link7_lateral_shift_m") is not None:
                    trial_errors.append("rejected case exposed an executable lateral proposal")
                if proposal.get("proposed_link7_yaw_rotation_rad") is not None:
                    trial_errors.append("rejected case exposed an executable yaw proposal")

        errors.extend(f"{label}: {error}" for error in trial_errors)
        records.append(
            {
                "label": label,
                "requested_relative_y_m": requested_y,
                "requested_relative_yaw_rad": requested_yaw,
                "expected_decision": expected_decision,
                "passed": not trial_errors,
                "measured_relative_y_m": summary.get("measured_relative_y_m"),
                "measured_relative_yaw_rad": summary.get("measured_relative_yaw_rad"),
                "observation_error_y_m": summary.get("observation_error_y_m"),
                "observation_error_yaw_rad": summary.get("observation_error_yaw_rad"),
                "proposal": summary.get("proposal"),
                "sample_count": summary.get("sample_count"),
                "errors": trial_errors,
            }
        )

    if len(records) != len(TRIALS):
        errors.append("not all matrix trials produced records")
    if not all(record["passed"] for record in records):
        errors.append("one or more active-centering observation trials failed")

    output = {
        "schema_version": 1,
        "gate_type": "gazebo_active_centering_observation_matrix",
        "passed": not errors,
        "errors": errors,
        "trial_count": len(records),
        "required_trial_count": len(TRIALS),
        "accepted_case_count": sum(
            record["expected_decision"] == "accept" for record in records
        ),
        "rejected_case_count": sum(
            record["expected_decision"] == "reject" for record in records
        ),
        "maximum_abs_observation_error_y_m": max(y_errors, default=None),
        "maximum_abs_observation_error_yaw_rad": max(yaw_errors, default=None),
        "simulation_oracle_only": True,
        "production_pose_estimator_claimed": False,
        "trials": records,
        "arm_command_sent": False,
        "gripper_command_sent": False,
        "support_removed": False,
        "lift_command_sent": False,
        "ifra_attachment_used": False,
        "grasp_success_claimed": False,
    }
    Path(args.output).write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")

    manifest = {
        "schema_version": 1,
        "evidence_type": "gazebo_active_centering_observation_matrix",
        "simulation_oracle_only": True,
        "production_pose_estimator_claimed": False,
        "fresh_gazebo_process_per_trial": True,
        "contact_free_fixture": True,
        "production_urdf_modified": False,
        "arm_command_sent": False,
        "gripper_command_sent": False,
        "support_removed": False,
        "lift_command_sent": False,
        "ifra_attachment_used": False,
        "grasp_success_claimed": False,
        "review_required": True,
        "trial_labels": [trial[0] for trial in TRIALS],
        "summary": Path(args.output).name,
    }
    Path(args.manifest).write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(output, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
