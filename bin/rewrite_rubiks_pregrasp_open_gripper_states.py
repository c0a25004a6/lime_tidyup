#!/usr/bin/env python3
"""Rewrite only the passive gripper columns of accepted pregrasp states.

Root pose and joint1..joint6 are copied byte-for-byte from the accepted 2,802
state matrix. Both finger positions are replaced with the declared fully-open
joint value so the same arm trajectory can be audited for the intended runtime
order: open gripper -> approach -> close.

No ROS/Gazebo process is started and no command is sent.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

EXPECTED_STATES = 2802
EXPECTED_FIELDS = 16
OPEN_M = 0.019
PASSIVE_MAX_M = 0.0001


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rewrite(source: Path, destination: Path) -> dict[str, object]:
    input_lines = source.read_text(encoding="utf-8").splitlines()
    require(len(input_lines) == EXPECTED_STATES, "accepted state count mismatch")

    output_lines: list[str] = []
    passive_left: list[float] = []
    passive_right: list[float] = []
    preserved_prefixes: list[str] = []

    for index, raw in enumerate(input_lines):
        fields = raw.split("\t")
        require(len(fields) == EXPECTED_FIELDS, f"state {index} field count mismatch")
        require(fields[0] == f"sample_{index:06d}", f"state {index} label mismatch")
        numeric = [float(value) for value in fields[1:]]
        require(all(math.isfinite(value) for value in numeric), f"state {index} contains non-finite value")

        left = float(fields[14])
        right = float(fields[15])
        passive_left.append(left)
        passive_right.append(right)
        preserved_prefixes.append("\t".join(fields[:14]))

        # Preserve label, root pose and all six arm joints exactly as text.
        output_lines.append("\t".join(fields[:14] + [f"{OPEN_M:.17g}", f"{OPEN_M:.17g}"]))

    require(max(abs(value) for value in passive_left) < PASSIVE_MAX_M, "source left finger was not passive-near-zero")
    require(max(abs(value) for value in passive_right) < PASSIVE_MAX_M, "source right finger was not passive-near-zero")

    destination.write_text("\n".join(output_lines) + "\n", encoding="utf-8")

    # Re-read and prove only fields 14/15 changed.
    rewritten = destination.read_text(encoding="utf-8").splitlines()
    require(len(rewritten) == EXPECTED_STATES, "rewritten state count mismatch")
    for index, raw in enumerate(rewritten):
        fields = raw.split("\t")
        require("\t".join(fields[:14]) == preserved_prefixes[index], f"state {index} root/arm prefix changed")
        require(abs(float(fields[14]) - OPEN_M) <= 1e-15, f"state {index} left finger not fully open")
        require(abs(float(fields[15]) - OPEN_M) <= 1e-15, f"state {index} right finger not fully open")

    return {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-OPEN-GRIPPER-STATE-REWRITE",
        "state_count": EXPECTED_STATES,
        "open_gripper_position_m": OPEN_M,
        "source_sha256": sha256(source),
        "output_sha256": sha256(destination),
        "passive_left_min_m": min(passive_left),
        "passive_left_max_m": max(passive_left),
        "passive_right_min_m": min(passive_right),
        "passive_right_max_m": max(passive_right),
        "root_and_arm_text_preserved_exactly": True,
        "changed_fields": ["gripper_left_joint", "gripper_right_joint"],
        "safety": {
            "offline_rewrite_only": True,
            "ros_node_created": False,
            "gazebo_started": False,
            "controller_loaded": False,
            "command_sent": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
        },
    }


def self_test() -> None:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        source = root / "source.tsv"
        output = root / "output.tsv"
        rows = []
        for index in range(EXPECTED_STATES):
            rows.append(
                "\t".join(
                    [f"sample_{index:06d}"]
                    + ["0"] * 3
                    + ["0", "0", "0", "1"]
                    + ["0"] * 6
                    + ["0.000012", "0.000012"]
                )
            )
        source.write_text("\n".join(rows) + "\n", encoding="utf-8")
        summary = rewrite(source, output)
        require(summary["root_and_arm_text_preserved_exactly"] is True, "prefix preservation failed")
        first = output.read_text().splitlines()[0].split("\t")
        require(first[14:] == ["0.019", "0.019"], "open rewrite failed")
    print("rewrite_rubiks_pregrasp_open_gripper_states self-test: PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input")
    parser.add_argument("--output")
    parser.add_argument("--summary")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0

    require(bool(args.input), "--input is required")
    require(bool(args.output), "--output is required")
    require(bool(args.summary), "--summary is required")
    summary = rewrite(Path(args.input), Path(args.output))
    Path(args.summary).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
