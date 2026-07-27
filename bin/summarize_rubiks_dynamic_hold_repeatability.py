#!/usr/bin/env python3
"""Aggregate three no-lift dynamic supported-hold trials into one gate result."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


TRIALS = (
    ("minus_0p5mm", -0.0005),
    ("center", 0.0),
    ("plus_0p5mm", 0.0005),
)


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def load_json(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    args = parser.parse_args()

    root = Path(args.root)
    errors: list[str] = []
    records: list[dict] = []

    for label, requested_y in TRIALS:
        trial_root = root / label
        try:
            summary = load_json(trial_root / "rubiks_dynamic_supported_hold_summary.json")
            audit = load_json(
                trial_root / "rubiks_dynamic_supported_hold_normal_audit.json"
            )
            telemetry = load_json(
                trial_root / "rubiks_dynamic_supported_hold_telemetry.json"
            )
            probe = load_json(
                trial_root
                / "rubiks_dynamic_supported_hold_telemetry_evidence.ffprobe.json"
            )
        except (FileNotFoundError, json.JSONDecodeError) as exc:
            errors.append(f"{label}: missing or invalid evidence: {exc}")
            continue

        trial_errors: list[str] = []
        require(summary.get("passed") is True, "summary did not pass", trial_errors)
        require(audit.get("passed") is True, "side-normal audit did not pass", trial_errors)
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
        require(
            math.isclose(float(telemetry.get("cube_mass_kg", -1)), 0.09, abs_tol=1e-12),
            "cube mass differs from canonical 90 g",
            trial_errors,
        )
        pose = telemetry.get("cube_pose_xyz_m") or []
        require(len(pose) == 3, "cube pose is missing", trial_errors)
        if len(pose) == 3:
            require(
                math.isclose(float(pose[1]), requested_y, abs_tol=1e-9),
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
        require(
            abs(float(contact.get("opening_error_m", 1.0)))
            <= float(contact.get("acceptance_tolerance_m", 0.0)),
            "calibrated opening error exceeds tolerance",
            trial_errors,
        )
        require(
            float(hold.get("support_contact_ratio", 0.0)) >= 0.75,
            "support contact coverage is below 75%",
            trial_errors,
        )
        require(
            float(hold.get("bilateral_finger_contact_ratio", 0.0)) >= 0.65,
            "bilateral finger contact coverage is below 65%",
            trial_errors,
        )
        require(
            float(movement.get("max_horizontal_displacement_m", 1.0)) <= 0.003,
            "hold horizontal displacement exceeds 3 mm",
            trial_errors,
        )
        require(
            float(movement.get("max_vertical_rise_m", 1.0)) <= 0.0015,
            "hold vertical rise exceeds 1.5 mm",
            trial_errors,
        )
        require(
            float(movement.get("max_rotation_rad", 1.0)) <= 0.10,
            "hold rotation exceeds 0.10 rad",
            trial_errors,
        )
        require(
            float(release.get("support_contact_ratio", 0.0)) >= 0.75,
            "post-release support contact coverage is below 75%",
            trial_errors,
        )
        require(
            float(release.get("final_cube_linear_speed_m_s", 1.0)) <= 0.02,
            "post-release cube speed exceeds 0.02 m/s",
            trial_errors,
        )
        require(
            float(final_offset.get("horizontal_displacement_m", 1.0)) <= 0.004,
            "final horizontal offset exceeds 4 mm",
            trial_errors,
        )
        require(
            abs(float(final_offset.get("vertical_displacement_m", 1.0))) <= 0.002,
            "final vertical offset exceeds 2 mm",
            trial_errors,
        )
        require(
            float(final_offset.get("rotation_rad", 1.0)) <= 0.12,
            "final rotation exceeds 0.12 rad",
            trial_errors,
        )

        duration = float((probe.get("format") or {}).get("duration", 0.0))
        size = int((probe.get("format") or {}).get("size", 0))
        require(duration >= 7.0, "telemetry video is too short", trial_errors)
        require(size >= 10000, "telemetry video is unexpectedly small", trial_errors)

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
                "video_size_bytes": size,
                "errors": trial_errors,
            }
        )

    require(len(records) == len(TRIALS), "not all three trials produced evidence", errors)
    all_passed = len(records) == len(TRIALS) and all(
        record["passed"] for record in records
    )
    require(all_passed, "one or more repeatability trials failed", errors)

    if records:
        aggregate = {
            "minimum_support_contact_ratio": min(
                float(record["hold_result"]["support_contact_ratio"])
                for record in records
            ),
            "minimum_bilateral_finger_contact_ratio": min(
                float(record["hold_result"]["bilateral_finger_contact_ratio"])
                for record in records
            ),
            "maximum_abs_opening_error_m": max(
                abs(float(record["contact_result"]["opening_error_m"]))
                for record in records
            ),
            "maximum_hold_horizontal_displacement_m": max(
                float(
                    record["hold_result"]["movement"][
                        "max_horizontal_displacement_m"
                    ]
                )
                for record in records
            ),
            "maximum_hold_vertical_rise_m": max(
                float(record["hold_result"]["movement"]["max_vertical_rise_m"])
                for record in records
            ),
            "maximum_hold_rotation_rad": max(
                float(record["hold_result"]["movement"]["max_rotation_rad"])
                for record in records
            ),
            "maximum_final_horizontal_offset_m": max(
                float(
                    record["release_result"][
                        "final_offset_from_preclose_settled_state"
                    ]["horizontal_displacement_m"]
                )
                for record in records
            ),
            "maximum_final_rotation_rad": max(
                float(
                    record["release_result"][
                        "final_offset_from_preclose_settled_state"
                    ]["rotation_rad"]
                )
                for record in records
            ),
        }
    else:
        aggregate = {}

    output = {
        "schema_version": 1,
        "gate_type": "gazebo_dynamic_supported_hold_lateral_repeatability",
        "passed": not errors,
        "errors": errors,
        "trial_count": len(records),
        "required_trial_count": len(TRIALS),
        "offset_axis": "cube Y relative to link7",
        "offsets_m": [offset for _, offset in TRIALS],
        "same_physics_and_acceptance_thresholds": True,
        "aggregate": aggregate,
        "trials": records,
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
    }
    Path(args.manifest).write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(output, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
