#!/usr/bin/env python3
"""Capture exact source and digest provenance for gripper collision meshes."""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

EXPECTED_LINKS = ("gripper_left_link", "gripper_right_link")
EXPECTED_PACKAGE = "package://turtlebot3_lime_description/"
EXPECTED_SCALE = [0.001, 0.001, 0.001]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--package-share", required=True)
    parser.add_argument("--source-repo", required=True)
    parser.add_argument("--expected-source-commit", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()

    urdf_path = Path(args.urdf)
    package_share = Path(args.package_share).resolve()
    source_repo = Path(args.source_repo).resolve()
    require(urdf_path.is_file(), "URDF is missing")
    require(package_share.is_dir(), "package share is missing")
    require((source_repo / ".git").exists(), "source repository is missing")
    require(len(args.expected_source_commit) == 40, "expected source commit is invalid")

    root = ET.parse(urdf_path).getroot()
    links = {str(link.get("name")): link for link in root.findall("link")}
    meshes: dict[str, object] = {}
    for link_name in EXPECTED_LINKS:
        link = links.get(link_name)
        require(link is not None, f"missing link: {link_name}")
        collisions = link.findall("collision")
        require(len(collisions) == 1, f"collision count mismatch: {link_name}")
        mesh = collisions[0].find("geometry/mesh")
        require(mesh is not None, f"collision is not a mesh: {link_name}")
        filename = str(mesh.get("filename", ""))
        scale = [float(value) for value in str(mesh.get("scale", "")).split()]
        require(filename.startswith(EXPECTED_PACKAGE), f"unexpected mesh package: {link_name}")
        require(scale == EXPECTED_SCALE, f"unexpected mesh scale: {link_name}")
        relative = filename[len(EXPECTED_PACKAGE):]
        resolved = (package_share / relative).resolve()
        require(resolved.is_file(), f"mesh file missing: {resolved}")
        require(package_share in resolved.parents, f"mesh escaped package share: {resolved}")
        meshes[link_name] = {
            "resource": filename,
            "relative_to_package_share": relative,
            "resolved_path": str(resolved),
            "scale": scale,
            "size_bytes": resolved.stat().st_size,
            "sha256": sha256(resolved),
        }

    commit = git(source_repo, "rev-parse", "HEAD")
    require(commit == args.expected_source_commit, "external source commit mismatch")
    status = git(source_repo, "status", "--porcelain")
    require(status == "", "source repository is dirty")
    remote = git(source_repo, "remote", "get-url", "origin")
    require("ROBOTIS-JAPAN-GIT/turtlebot3_lime" in remote, "unexpected source repository origin")
    payload = {
        "schema_version": 1,
        "phase": "RUBIK-PREGRASP-CORRECTED-FIXTURE-GEOMETRIC-RELEVANCE-MESH-PROVENANCE",
        "source_ref": args.source_ref,
        "passed": True,
        "errors": [],
        "source_repository": {
            "path": str(source_repo),
            "commit": commit,
            "expected_commit": args.expected_source_commit,
            "origin": remote,
            "clean": True,
            "exact_commit_bound": True,
        },
        "urdf": {
            "path": str(urdf_path.resolve()),
            "sha256": sha256(urdf_path),
        },
        "package_share": str(package_share),
        "meshes": meshes,
        "safety": {
            "read_only_filesystem_inspection": True,
            "gazebo_started": False,
            "ros_node_created": False,
            "controller_loaded": False,
            "planning_request_created": False,
            "command_sent": False,
            "physical_hardware_used": False,
            "production_runtime_modified": False,
        },
    }
    Path(args.output).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
