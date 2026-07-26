#!/usr/bin/env python3
"""Safe preflight checks for Rubik's Cube grasp work in lime_tidyup."""

from __future__ import annotations

import argparse
import ctypes.util
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

PLUGINS = (
    "libgazebo_grasp_fix.so",
    "libgazebo_link_attacher.so",
    "libgazebo_ros_state.so",
)
SEARCH_ROOTS = (
    "/project/lib_ws/install",
    "/project/lib_ws/build",
    "/root/turtlebot3_ws/install",
    "/opt/ros/humble",
    "/usr/lib/x86_64-linux-gnu",
    "/usr/local/lib",
)


class PreflightError(RuntimeError):
    pass


def default_parameters() -> Path:
    candidates = (
        Path(__file__).resolve().parents[1]
        / "project/resource/model_editor_models/rubiks_cube/parameters.json",
        Path("/root/.gazebo/models/rubiks_cube/parameters.json"),
        Path("/project/resource/model_editor_models/rubiks_cube/parameters.json"),
    )
    return next((path for path in candidates if path.is_file()), candidates[0])


def load_config(path: Path) -> dict:
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PreflightError(f"cannot read {path}: {exc}") from exc
    if config.get("schema_version") != 1:
        raise PreflightError("parameters.json schema_version must be 1")
    model = config["model"]
    edge = float(model["edge_length_m"])
    mass = float(model["mass_kg"])
    gap = float(model["tile_gap_m"])
    thickness = float(model["tile_thickness_m"])
    if not 0.02 <= edge <= 0.08:
        raise PreflightError("edge_length_m must be in [0.02, 0.08]")
    if not 0.005 <= mass <= 0.5:
        raise PreflightError("mass_kg must be in [0.005, 0.5]")
    if not 0 <= gap < edge / 3 or not 0 < thickness < edge / 10:
        raise PreflightError("invalid tile gap or thickness")
    for side in ("left_joint", "right_joint"):
        joint = config["gripper"][side]
        if float(joint["lower_m"]) >= float(joint["upper_m"]):
            raise PreflightError(f"{side}: lower_m must be below upper_m")
    return config


def frame_separation(config: dict, left_q: float, right_q: float | None = None) -> float:
    right_q = left_q if right_q is None else right_q
    left = config["gripper"]["left_joint"]
    right = config["gripper"]["right_joint"]
    left_y = float(left["origin_y_m"]) + float(left["axis_y"]) * left_q
    right_y = float(right["origin_y_m"]) + float(right["axis_y"]) * right_q
    return abs(left_y - right_y)


def inner_opening(config: dict, left_q: float, right_q: float | None = None) -> float | None:
    offset = config["gripper"].get("inner_face_offset_from_link_frame_m")
    if offset is None:
        return None
    return frame_separation(config, left_q, right_q) - 2 * float(offset)


def calibrate_offset(separation: float, reference_width: float) -> float:
    if reference_width <= 0 or reference_width > separation:
        raise PreflightError("reference width must be positive and no wider than frame separation")
    return (separation - reference_width) / 2


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def find_plugins(extra_roots: list[str], runtime_ros: bool) -> dict:
    raw_roots = list(extra_roots)
    raw_roots += [p for p in os.environ.get("GAZEBO_PLUGIN_PATH", "").split(":") if p]
    raw_roots += list(SEARCH_ROOTS)
    roots, seen = [], set()
    for raw in raw_roots:
        path = Path(raw).expanduser()
        if path.exists() and path not in seen:
            roots.append(path)
            seen.add(path)

    found = {name: [] for name in PLUGINS}
    for root in roots:
        if root.is_file():
            if root.name in found:
                found[root.name].append(str(root.resolve()))
            continue
        for current, dirs, files in os.walk(root):
            dirs[:] = [d for d in dirs if d not in {".git", "include", "share", "src", "__pycache__"}]
            for name in set(files).intersection(found):
                found[name].append(str((Path(current) / name).resolve()))

    try:
        result = subprocess.run(
            ["ldconfig", "-p"], check=False, text=True, capture_output=True, timeout=5
        )
        for name in PLUGINS:
            for line in result.stdout.splitlines():
                if name in line and "=>" in line:
                    found[name].append(line.split("=>", 1)[1].strip())
    except (OSError, subprocess.TimeoutExpired):
        pass

    plugins = {}
    for name in PLUGINS:
        short = name.removeprefix("lib").removesuffix(".so")
        dynamic = ctypes.util.find_library(short)
        plugins[name] = {
            "found": bool(found[name] or dynamic),
            "paths": sorted(set(found[name])),
            "ctypes_find_library": dynamic,
        }

    runtime = {"checked": False, "matching_services": []}
    if runtime_ros and shutil.which("ros2"):
        try:
            result = subprocess.run(
                ["ros2", "service", "list"],
                check=False,
                text=True,
                capture_output=True,
                timeout=8,
            )
            runtime = {
                "checked": True,
                "returncode": result.returncode,
                "matching_services": [
                    line for line in result.stdout.splitlines()
                    if any(key in line.lower() for key in ("attach", "detach", "entity_state"))
                ],
                "stderr": result.stderr.strip(),
            }
        except (OSError, subprocess.TimeoutExpired) as exc:
            runtime = {"checked": True, "error": str(exc), "matching_services": []}

    required = "libgazebo_grasp_fix.so"
    return {
        "schema_version": 1,
        "timestamp_unix": time.time(),
        "gazebo_plugin_path": os.environ.get("GAZEBO_PLUGIN_PATH", ""),
        "search_roots": [str(path) for path in roots],
        "plugins": plugins,
        "required_plugin_found": plugins[required]["found"],
        "runtime_ros": runtime,
    }


def read_joint_state(topic: str, left_name: str, right_name: str, timeout: float):
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        raise PreflightError("rclpy is unavailable; use --joint-position-m") from exc

    values = {}
    rclpy.init(args=None)
    node = Node("rubiks_gripper_opening_probe")

    def callback(message):
        for name, value in zip(message.name, message.position):
            if name in (left_name, right_name):
                values[name] = float(value)

    subscription = node.create_subscription(JointState, topic, callback, 10)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and left_name not in values:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if left_name not in values:
        raise PreflightError(f"{left_name} was not observed on {topic}")
    return values[left_name], values.get(right_name)


def material(rgba: str) -> str:
    return (
        "<material>"
        f"<ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
        "<specular>0.05 0.05 0.05 1</specular>"
        "</material>"
    )


def visual(name: str, pose: str, size: str, rgba: str) -> str:
    return (
        f'<visual name="{name}"><pose>{pose}</pose>'
        f"<geometry><box><size>{size}</size></box></geometry>"
        f"{material(rgba)}</visual>"
    )


def model_sdf(config: dict) -> str:
    model = config["model"]
    edge = float(model["edge_length_m"])
    mass = float(model["mass_kg"])
    thickness = float(model["tile_thickness_m"])
    mu = float(model["friction"]["mu"])
    mu2 = float(model["friction"]["mu2"])
    restitution = float(model["restitution_coefficient"])
    inertia = mass * edge * edge / 6
    h = edge / 2 + thickness / 2
    e, t = f"{edge:.9g}", f"{thickness:.9g}"
    faces = (
        visual("face_px_red", f"{h:.9g} 0 0 0 0 0", f"{t} {e} {e}", "0.8 0.02 0.02 1"),
        visual("face_nx_orange", f"{-h:.9g} 0 0 0 0 0", f"{t} {e} {e}", "1 0.22 0 1"),
        visual("face_py_blue", f"0 {h:.9g} 0 0 0 0", f"{e} {t} {e}", "0.02 0.12 0.85 1"),
        visual("face_ny_green", f"0 {-h:.9g} 0 0 0 0", f"{e} {t} {e}", "0.02 0.65 0.1 1"),
        visual("face_pz_white", f"0 0 {h:.9g} 0 0 0", f"{e} {e} {t}", "0.95 0.95 0.95 1"),
        visual("face_nz_yellow", f"0 0 {-h:.9g} 0 0 0", f"{e} {e} {t}", "0.95 0.85 0.02 1"),
    )
    return f'''<?xml version="1.0"?>
<!-- Generated by bin/rubiks_grasp_preflight.py; edit parameters.json instead. -->
<sdf version="1.7">
  <model name="{model['name']}">
    <static>false</static>
    <self_collide>false</self_collide>
    <allow_auto_disable>false</allow_auto_disable>
    <link name="cube_link">
      <inertial>
        <mass>{mass:.9g}</mass>
        <inertia>
          <ixx>{inertia:.12g}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{inertia:.12g}</iyy><iyz>0</iyz><izz>{inertia:.12g}</izz>
        </inertia>
      </inertial>
      <collision name="cube_collision">
        <geometry><box><size>{e} {e} {e}</size></box></geometry>
        <surface>
          <friction><ode><mu>{mu:.9g}</mu><mu2>{mu2:.9g}</mu2></ode></friction>
          <bounce><restitution_coefficient>{restitution:.9g}</restitution_coefficient></bounce>
          <contact><ode><max_vel>0.01</max_vel><min_depth>0.0001</min_depth></ode></contact>
        </surface>
        <max_contacts>20</max_contacts>
      </collision>
      <visual name="black_body">
        <geometry><box><size>{e} {e} {e}</size></box></geometry>
        {material("0.005 0.005 0.005 1")}
      </visual>
      {chr(10).join(faces)}
    </link>
  </model>
</sdf>
'''


def validate_model(config: dict, path: Path) -> dict:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise PreflightError(f"cannot parse {path}: {exc}") from exc
    edge = float(config["model"]["edge_length_m"])
    mass = float(config["model"]["mass_kg"])
    inertia = mass * edge * edge / 6
    size = [float(v) for v in root.findtext(".//collision/geometry/box/size", "").split()]
    actual_mass = float(root.findtext(".//inertial/mass", "nan"))
    diagonal = [
        float(root.findtext(f".//inertial/inertia/{axis}", "nan"))
        for axis in ("ixx", "iyy", "izz")
    ]
    checks = {
        "edge": len(size) == 3 and all(math.isclose(v, edge, abs_tol=1e-10) for v in size),
        "mass": math.isclose(actual_mass, mass, abs_tol=1e-10),
        "inertia": all(math.isclose(v, inertia, rel_tol=1e-9, abs_tol=1e-12) for v in diagonal),
        "one_collision": len(root.findall(".//collision")) == 1,
        "six_face_visuals": len(root.findall(".//visual")) == 7,
    }
    return {"model": str(path), "checks": checks, "valid": all(checks.values())}


def command_probe(args):
    report = find_plugins(args.search_root, args.runtime_ros)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.report:
        write_json(Path(args.report), report)
    return 0 if report["required_plugin_found"] or args.allow_missing else 3


def command_opening(args):
    path = Path(args.parameters).resolve() if args.parameters else default_parameters()
    config = load_config(path)
    if args.subscribe:
        left_q, right_q = read_joint_state(
            args.topic,
            config["gripper"]["left_joint"]["name"],
            config["gripper"]["right_joint"]["name"],
            args.timeout,
        )
    else:
        left_q, right_q = args.joint_position_m, args.right_joint_position_m
    separation = frame_separation(config, left_q, right_q)
    result = {
        "left_joint_position_m": left_q,
        "right_joint_position_m": left_q if right_q is None else right_q,
        "link_frame_separation_m": separation,
        "inner_face_opening_m": inner_opening(config, left_q, right_q),
        "mesh_calibrated": config["gripper"].get("inner_face_offset_from_link_frame_m") is not None,
    }
    if args.reference_width_m is not None:
        offset = calibrate_offset(separation, args.reference_width_m)
        result["calibration"] = {
            "reference_width_m": args.reference_width_m,
            "inner_face_offset_from_link_frame_m": offset,
        }
        if args.write_calibration:
            config["gripper"]["inner_face_offset_from_link_frame_m"] = offset
            config["gripper"]["calibration_status"] = "gauge_measured"
            config["gripper"]["calibration_reference_width_m"] = args.reference_width_m
            write_json(path, config)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


def command_generate(args):
    parameters = Path(args.parameters).resolve() if args.parameters else default_parameters()
    config = load_config(parameters)
    output = Path(args.output).resolve() if args.output else parameters.with_name("model.sdf")
    output.write_text(model_sdf(config), encoding="utf-8")
    report = validate_model(config, output)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["valid"] else 4


def command_validate(args):
    parameters = Path(args.parameters).resolve() if args.parameters else default_parameters()
    config = load_config(parameters)
    model = Path(args.model).resolve() if args.model else parameters.with_name("model.sdf")
    report = validate_model(config, model)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["valid"] else 4


def command_self_test(_):
    config = {
        "schema_version": 1,
        "model": {
            "name": "rubiks_cube", "edge_length_m": 0.057, "mass_kg": 0.09,
            "tile_gap_m": 0.001, "tile_thickness_m": 0.0005,
            "restitution_coefficient": 0, "friction": {"mu": 1, "mu2": 1},
        },
        "gripper": {
            "left_joint": {"origin_y_m": 0.021, "axis_y": 1, "lower_m": -0.01, "upper_m": 0.019},
            "right_joint": {"origin_y_m": -0.021, "axis_y": -1, "lower_m": -0.01, "upper_m": 0.019},
            "inner_face_offset_from_link_frame_m": None,
        },
    }
    assert math.isclose(frame_separation(config, -0.01), 0.022)
    assert math.isclose(frame_separation(config, 0.019), 0.08)
    assert math.isclose(calibrate_offset(0.064, 0.057), 0.0035)
    root = ET.fromstring(model_sdf(config))
    assert len(root.findall(".//visual")) == 7
    assert len(root.findall(".//collision")) == 1
    print("rubiks_grasp_preflight self-test: PASS")
    return 0


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(required=True)

    probe = commands.add_parser("probe-plugin")
    probe.add_argument("--search-root", action="append", default=[])
    probe.add_argument("--runtime-ros", action="store_true")
    probe.add_argument("--allow-missing", action="store_true")
    probe.add_argument("--report")
    probe.set_defaults(func=command_probe)

    opening = commands.add_parser("opening")
    opening.add_argument("--parameters")
    source = opening.add_mutually_exclusive_group(required=True)
    source.add_argument("--joint-position-m", type=float)
    source.add_argument("--subscribe", action="store_true")
    opening.add_argument("--right-joint-position-m", type=float)
    opening.add_argument("--topic", default="/joint_states")
    opening.add_argument("--timeout", type=float, default=5)
    opening.add_argument("--reference-width-m", type=float)
    opening.add_argument("--write-calibration", action="store_true")
    opening.set_defaults(func=command_opening)

    generate = commands.add_parser("generate-model")
    generate.add_argument("--parameters")
    generate.add_argument("--output")
    generate.set_defaults(func=command_generate)

    validate = commands.add_parser("validate-model")
    validate.add_argument("--parameters")
    validate.add_argument("--model")
    validate.set_defaults(func=command_validate)

    test = commands.add_parser("self-test")
    test.set_defaults(func=command_self_test)
    return root


def main() -> int:
    args = parser().parse_args()
    try:
        return args.func(args)
    except PreflightError as exc:
        print(f"preflight error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
