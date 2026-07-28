#!/usr/bin/env python3
"""Aggregate no-lift dynamic supported-hold trials into one gate result."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

TRIALS = (
    ("minus_0p5mm", -0.0005),
    ("center", 0.0),
    ("plus_0p5mm", 0.0005),
)


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected object in {path}")
    return value


def as_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def at_least(value: Any, minimum: float, message: str, errors: list[str]) -> None:
    number = as_float(value)
    require(number is not None and number >= minimum, message, errors)


def at_most(value: Any, maximum: float, message: str, errors: list[str]) -> None:
    number = as_float(value)
    require(number is not None and number <= maximum, message, errors)


def abs_at_most(value: Any, maximum: float, message: str, errors: list[str]) -> None:
    number = as_float(value)
    require(number is not None and abs(number) <= maximum, message, errors)


def aggregate_passed(records: list[dict[str, Any]]) -> dict[str, Any]:
    passed = [record for record in records if record.get("passed") is True]
    result: dict[str, Any] = {
        "available_passed_trial_count": len(passed),
        "all_required_trials_available": len(passed) == len(TRIALS),
    }
    if not passed:
        return result

    def values(path: tuple[str, ...]) -> list[float]:
        output: list[float] = []
        for record in passed:
            value: Any = record
            for key in path:
                if not isinstance(value, dict):
                    value = None
                    break
                value = value.get(key)
            number = as_float(value)
            if number is not None:
                output.append(number)
        return output

    metric_specs = {
        "minimum_support_contact_ratio": (min, ("hold_result", "support_contact_ratio")),
        "minimum_bilateral_finger_contact_ratio": (
            min,
            ("hold_result", "bilateral_finger_contact_ratio"),
        ),
        "maximum_hold_horizontal_displacement_m": (
            max,
            ("hold_result", "movement", "max_horizontal_displacement_m"),
        ),
        "maximum_hold_vertical_rise_m": (
            max,
            ("hold_result", "movement", "max_vertical_rise_m"),
        ),
        "maximum_hold_rotation_rad": (
            max,
            ("hold_result", "movement", "max_rotation_rad"),
        ),
        "maximum_final_horizontal_offset_m": (
            max,
            (
                "release_result",
                "final_offset_from_preclose_settled_state",
                "horizontal_displacement_m",
            ),
        ),
        "maximum_final_rotation_rad": (
            max,
            (
                "release_result",
                "final_offset_from_preclose_settled_state",
                "rotation_rad",
            ),
        ),
    }
    for name, (operation, path) in metric_specs.items():
        found = values(path)
        if found:
            result[name] = operation(found)

    opening_errors = values(("contact_result", "opening_error_m"))
    if opening_errors:
        result["maximum_abs_opening_error_m"] = max(abs(value) for value in opening_errors)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    args = parser.parse_args()

    root = Path(args.root)
    errors: list[str] = []
    records: list[dict[str, Any]] = []

    for label, requested_y in TRIALS:
        trial_root = root / label
        trial_errors: list[str] = []
        summary: dict[str, Any] = {}
        audit: dict[str, Any] = {}
        telemetry: dict[str, Any] = {}
        probe: dict[str, Any] = {}
        try:
            summary = load_json(trial_root / "rubiks_dynamic_supported_hold_summary.json")
            audit = load_json(trial_root / "rubiks_dynamic_supported_hold_normal_audit.json")
            telemetry = load_json(trial_root / "rubiks_dynamic_supported_hold_telemetry.json")
            probe = load_json(
                trial_root / "rubiks_dynamic_supported_hold_telemetry_evidence.ffprobe.json"
            )
        except (FileNotFoundError, json.JSONDecodeError, ValueError) as exc:
            trial_errors.append(f"missing or invalid evidence: {exc}")

        if summary:
            require(summary.get("passed") is True, "summary did not pass", trial_errors)
            for error in summary.get("errors", []):
                message = f"trial summary: {error}"
                if message not in trial_errors:
                    trial_errors.append(message)
        if audit:
            require(audit.get("passed") is True, "side-normal audit did not pass", trial_errors)
        if telemetry:
            require(
                telemetry.get("trial_type")
                == "gazebo_dynamic_supported_rubiks_cube_hold_release",
                "unexpected trial_type",
                trial_errors,
            )
            require(
                telemetry.get("source_cube_collision_size_m") == [0.057, 0.057, 0.057],
                "cube collision differs from canonical 57 mm",
                trial_errors,
            )
            mass = as_float(telemetry.get("cube_mass_kg"))
            require(
                mass is not None and math.isclose(mass, 0.09, abs_tol=1e-12),
                "cube mass differs from canonical 90 g",
                trial_errors,
            )
            pose = telemetry.get("cube_pose_xyz_m") or []
            require(
                isinstance(pose, list) and len(pose) == 3,
                "cube pose is missing",
                trial_errors,
            )
            if isinstance(pose, list) and len(pose) == 3:
                pose_y = as_float(pose[1])
                require(
                    pose_y is not None
                    and math.isclose(pose_y, requested_y, abs_tol=1e-9),
                    f"requested Y offset {requested_y} was not applied: {pose[1]}",
                    trial_errors,
                )
            for key in (
                "cube_dynamic",
                "cube_mass_dynamics_tested",
                "support_static",
                "support_contact_before_close",
                "left_cube_contact_observed",
                "right_cube_contact_observed",
                "dual_cube_contact_observed",
                "contact_cleared_after_reopen",
            ):
                require(telemetry.get(key) is True, f"{key} must be true", trial_errors)
            require(
                telemetry.get("initial_open_finger_contact") is False,
                "finger contact existed while fully open",
                trial_errors,
            )
            require(
                int(telemetry.get("unexpected_robot_contact_count", -1)) == 0,
                "cube contacted a non-finger robot collision",
                trial_errors,
            )
            for key in (
                "arm_motion_performed",
                "lift_command_sent",
                "cube_lifted",
                "ifra_attachment_used",
                "grasp_success_claimed",
            ):
                require(telemetry.get(key) is False, f"{key} must remain false", trial_errors)

        contact = summary.get("contact_result") or {}
        hold = summary.get("hold_result") or {}
        movement = hold.get("movement") or {}
        release = summary.get("release_result") or {}
        final_offset = release.get("final_offset_from_preclose_settled_state") or {}

        opening_error = as_float(contact.get("opening_error_m"))
        tolerance = as_float(contact.get("acceptance_tolerance_m"))
        require(
            opening_error is not None
            and tolerance is not None
            and abs(opening_error) <= tolerance,
            "calibrated opening error exceeds tolerance or is missing",
            trial_errors,
        )
        at_least(
            hold.get("support_contact_ratio"),
            0.75,
            "support contact coverage is below 75%",
            trial_errors,
        )
        at_least(
            hold.get("bilateral_finger_contact_ratio"),
            0.65,
            "bilateral finger contact coverage is below 65%",
            trial_errors,
        )
        at_most(
            movement.get("max_horizontal_displacement_m"),
            0.003,
            "hold horizontal displacement exceeds 3 mm or is missing",
            trial_errors,
        )
        at_most(
            movement.get("max_vertical_rise_m"),
            0.0015,
            "hold vertical rise exceeds 1.5 mm or is missing",
            trial_errors,
        )
        at_most(
            movement.get("max_rotation_rad"),
            0.10,
            "hold rotation exceeds 0.10 rad or is missing",
            trial_errors,
        )
        at_least(
            release.get("support_contact_ratio"),
            0.75,
            "post-release support contact coverage is below 75%",
            trial_errors,
        )
        at_most(
            release.get("final_cube_linear_speed_m_s"),
            0.02,
            "post-release cube speed exceeds 0.02 m/s or is missing",
            trial_errors,
        )
        at_most(
            final_offset.get("horizontal_displacement_m"),
            0.004,
            "final horizontal offset exceeds 4 mm or is missing",
            trial_errors,
        )
        abs_at_most(
            final_offset.get("vertical_displacement_m"),
            0.002,
            "final vertical offset exceeds 2 mm or is missing",
            trial_errors,
        )
        at_most(
            final_offset.get("rotation_rad"),
            0.12,
            "final rotation exceeds 0.12 rad or is missing",
            trial_errors,
        )

        duration = as_float((probe.get("format") or {}).get("duration"))
        size = as_float((probe.get("format") or {}).get("size"))
        require(
            duration is not None and duration >= 7.0,
            "telemetry video is too short or missing",
            trial_errors,
        )
        require(
            size is not None and size >= 10000,
            "telemetry video is unexpectedly small or missing",
            trial_errors,
        )

        errors.extend(f"{label}: {error}" for error in trial_errors)
        records.append(
            {
                "label": label,
                "requested_cube_y_m": requested_y,
                "passed": not trial_errors,
                "sample_count": summary.get("sample_count"),
                "finger_contact_message_count": summary.get(
                    "finger_contact_message_count"
                ),
                "support_contact_message_count": summary.get(
                    "support_contact_message_count"
                ),
                "contact_result": contact,
                "hold_result": hold,
                "release_result": release,
                "normal_audit": audit,
                "video_duration_s": duration,
                "video_size_bytes": int(size) if size is not None else None,
                "errors": trial_errors,
            }
        )

    require(len(records) == len(TRIALS), "not all required trials produced records", errors)
    all_passed = len(records) == len(TRIALS) and all(
        record["passed"] for record in records
    )
    require(all_passed, "one or more repeatability trials failed", errors)

    output = {
        "schema_version": 2,
        "gate_type": "gazebo_dynamic_supported_hold_lateral_repeatability",
        "passed": not errors,
        "errors": errors,
        "trial_count": len(records),
        "required_trial_count": len(TRIALS),
        "offset_axis": "cube Y relative to link7",
        "offsets_m": [offset for _, offset in TRIALS],
        "same_physics_and_acceptance_thresholds": True,
        "aggregate": aggregate_passed(records),
        "trials": records,
        "arm_motion_performed": False,
        "lift_command_sent": False,
        "support_removed": False,
        "ifra_attachment_used": False,
        "grasp_success_claimed": False,
    }
    Path(args.output).write_text(
        json.dumps(output, indent=2) + "\n", encoding="utf-8"
    )

    manifest = {
        "schema_version": 2,
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
        "partial_failure_is_reported_without_aggregator_exception": True,
    }
    Path(args.manifest).write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(output, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
