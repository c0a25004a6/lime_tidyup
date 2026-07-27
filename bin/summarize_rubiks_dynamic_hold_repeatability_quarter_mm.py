#!/usr/bin/env python3
"""Run the repeatability aggregator for the accepted ±0.25 mm matrix."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback

import summarize_rubiks_dynamic_hold_repeatability as base


TRIALS = (
    ("minus_0p25mm", -0.00025),
    ("center", 0.0),
    ("plus_0p25mm", 0.00025),
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    return parser.parse_args()


def write_diagnostic_failure(exc: Exception) -> None:
    args = arguments()
    root = Path(args.root)
    trials = []
    for label, offset in TRIALS:
        trial_root = root / label
        record = {
            "label": label,
            "requested_cube_y_m": offset,
            "trial_exit_status": None,
            "summary_present": False,
            "summary_passed": False,
            "summary_errors": [],
        }
        status_path = trial_root / "trial_exit_status.txt"
        if status_path.is_file():
            try:
                record["trial_exit_status"] = int(status_path.read_text().strip())
            except ValueError:
                record["trial_exit_status"] = status_path.read_text().strip()
        summary_path = trial_root / "rubiks_dynamic_supported_hold_summary.json"
        if summary_path.is_file():
            record["summary_present"] = True
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                record["summary_passed"] = summary.get("passed") is True
                record["summary_errors"] = summary.get("errors", [])
                record["event_labels"] = summary.get("event_labels", [])
            except json.JSONDecodeError as error:
                record["summary_errors"] = [f"invalid summary JSON: {error}"]
        trials.append(record)

    output = {
        "schema_version": 1,
        "gate_type": "gazebo_dynamic_supported_hold_lateral_repeatability",
        "passed": False,
        "errors": [
            f"aggregator exception: {type(exc).__name__}: {exc}",
            traceback.format_exc(),
        ],
        "trial_count": len(trials),
        "required_trial_count": len(TRIALS),
        "offset_axis": "cube Y relative to link7",
        "offsets_m": [offset for _, offset in TRIALS],
        "same_physics_and_acceptance_thresholds": True,
        "trials": trials,
        "arm_motion_performed": False,
        "lift_command_sent": False,
        "support_removed": False,
        "ifra_attachment_used": False,
        "grasp_success_claimed": False,
    }
    Path(args.output).write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "evidence_type": "gazebo_dynamic_supported_hold_lateral_repeatability",
        "source_pr": 7,
        "trial_labels": [label for label, _ in TRIALS],
        "trial_offsets_m": [offset for _, offset in TRIALS],
        "fresh_gazebo_process_per_trial": True,
        "same_support_effort_timing_calibration_thresholds": True,
        "production_urdf_modified": False,
        "arm_motion_performed": False,
        "lift_command_sent": False,
        "support_removed": False,
        "ifra_attachment_used": False,
        "grasp_success_claimed": False,
        "review_required": True,
        "summary": Path(args.output).name,
        "trial_directories": [label for label, _ in TRIALS],
        "aggregator_exception_recorded": True,
    }
    Path(args.manifest).write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(output, indent=2))


def main() -> int:
    base.TRIALS = TRIALS
    try:
        return base.main()
    except Exception as exc:
        write_diagnostic_failure(exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
