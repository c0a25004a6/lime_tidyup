#!/usr/bin/env python3
"""Fail if package:// resources referenced by an accepted Rubik URDF are missing.

This is an offline preflight guard. It creates no ROS node and sends no command.
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


REQUIRED_MESH_BASENAMES = {
    "gripper_left_palm.stl",
    "gripper_right_palm.stl",
    "link7.stl",
}


def resolve_resources(urdf_path: Path) -> list[Path]:
    text = urdf_path.read_text(encoding="utf-8")
    refs = sorted(set(re.findall(r'package://([^/]+)/([^"<]+)', text)))
    if not refs:
        raise RuntimeError("accepted URDF contains no package:// resources")

    resolved: list[Path] = []
    for package, relative in refs:
        share = Path(get_package_share_directory(package))
        target = share / relative
        if not target.is_file():
            raise RuntimeError(
                f"unresolved package resource: package://{package}/{relative} -> {target}"
            )
        resolved.append(target)

    basenames = {path.name for path in resolved}
    missing_required = REQUIRED_MESH_BASENAMES - basenames
    if missing_required:
        raise RuntimeError(
            "required Lime collision meshes were not referenced/resolved: "
            + ", ".join(sorted(missing_required))
        )
    return resolved


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    args = parser.parse_args()

    resolved = resolve_resources(Path(args.urdf))
    print(f"resolved_package_resources={len(resolved)}")
    for path in resolved:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
