#!/usr/bin/env python3
"""Bind Gazebo model/link observations to MoveIt root poses for ground audit."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

JOINTS = [f"joint{i}" for i in range(1, 7)]
AUDITED_LINKS = [f"link{i}" for i in range(1, 8)]
MAX_AGE_S = 0.05
MAX_POSITION_ERROR_RAD = 1e-9
MAX_FIXED_RESIDUAL_M = 1e-5
MAX_FIXED_RESIDUAL_RAD = 1e-5


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValueError(message)


def vec(text: str | None, count: int, default: list[float]) -> np.ndarray:
    value = np.asarray(default if text is None else [float(x) for x in text.split()], dtype=float)
    require(value.shape == (count,) and np.isfinite(value).all(), f"bad vector {value}")
    return value


def rpy(r: np.ndarray) -> np.ndarray:
    a, b, c = r
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return np.asarray([
        [cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa],
        [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa],
        [-sb, cb * sa, cb * ca],
    ])


def tf(rotation: np.ndarray | None = None, translation: np.ndarray | None = None) -> np.ndarray:
    value = np.eye(4)
    if rotation is not None:
        value[:3, :3] = rotation
    if translation is not None:
        value[:3, 3] = translation
    return value


def quat_rotation(xyzw: list[float]) -> np.ndarray:
    x, y, z, w = [float(v) for v in xyzw]
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    require(math.isfinite(norm) and norm > 1e-12, "invalid quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.asarray([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def matrix_quat(r: np.ndarray) -> list[float]:
    t = float(np.trace(r))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        q = [(r[2,1]-r[1,2])/s, (r[0,2]-r[2,0])/s, (r[1,0]-r[0,1])/s, 0.25*s]
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(1+r[0,0]-r[1,1]-r[2,2])*2
            q = [0.25*s, (r[0,1]+r[1,0])/s, (r[0,2]+r[2,0])/s, (r[2,1]-r[1,2])/s]
        elif i == 1:
            s = math.sqrt(1+r[1,1]-r[0,0]-r[2,2])*2
            q = [(r[0,1]+r[1,0])/s, 0.25*s, (r[1,2]+r[2,1])/s, (r[0,2]-r[2,0])/s]
        else:
            s = math.sqrt(1+r[2,2]-r[0,0]-r[1,1])*2
            q = [(r[0,2]+r[2,0])/s, (r[1,2]+r[2,1])/s, 0.25*s, (r[1,0]-r[0,1])/s]
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q *= -1
    return q.tolist()


def pose_matrix(pose: dict[str, Any]) -> np.ndarray:
    require(pose.get("all_finite") is True, "pose marked non-finite")
    p = np.asarray(pose.get("position_m"), dtype=float)
    require(p.shape == (3,) and np.isfinite(p).all(), "bad pose position")
    return tf(quat_rotation(pose.get("orientation_xyzw")), p)


def angle(r: np.ndarray) -> float:
    return math.acos(max(-1.0, min(1.0, (float(np.trace(r)) - 1.0) / 2.0)))


def residual(reference: np.ndarray, current: np.ndarray) -> tuple[float, float]:
    d = np.linalg.inv(reference) @ current
    return float(np.linalg.norm(d[:3, 3])), angle(d[:3, :3])


def urdf_binding(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    links = [str(x.get("name")) for x in root.findall("link")]
    by_child: dict[str, ET.Element] = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        require(child is not None and child.get("link"), "joint child missing")
        by_child[str(child.get("link"))] = joint
    roots = sorted(set(links) - set(by_child))
    require(roots == ["base_footprint"], f"unexpected roots {roots}")
    chain: list[ET.Element] = []
    current = "base_link"
    while current != "base_footprint":
        joint = by_child.get(current)
        require(joint is not None and joint.get("type") == "fixed", f"bad root path at {current}")
        chain.append(joint)
        current = str(joint.find("parent").get("link"))
    root_to_base = np.eye(4)
    names = []
    for joint in reversed(chain):
        names.append(str(joint.get("name")))
        origin = joint.find("origin")
        xyz = vec(origin.get("xyz") if origin is not None else None, 3, [0,0,0])
        rotation = vec(origin.get("rpy") if origin is not None else None, 3, [0,0,0])
        root_to_base = root_to_base @ tf(rpy(rotation), xyz)
    hashes: dict[str, str] = {}
    missing = []
    for name in AUDITED_LINKS:
        link = next((x for x in root.findall("link") if x.get("name") == name), None)
        if link is None or not link.findall("collision"):
            missing.append(name)
            continue
        data = b"".join(ET.tostring(x, encoding="utf-8") for x in link.findall("collision"))
        hashes[name] = hashlib.sha256(data).hexdigest()
    return {
        "robot_name": root.get("name"), "root_link": "base_footprint", "observed_link": "base_link",
        "fixed_joint_path": names, "root_to_base_link_matrix": root_to_base.tolist(),
        "audited_ground_links": AUDITED_LINKS, "missing_audited_collision_geometry": missing,
        "audited_collision_geometry_sha256": hashes,
    }


def identity_pose(element: ET.Element | None) -> bool:
    return element is None or not (element.text or "").strip() or np.max(np.abs(vec(element.text, 6, [0]*6))) <= 1e-12


def world_binding(path: Path) -> dict[str, Any]:
    sdf = ET.parse(path).getroot()
    require(sdf.tag == "sdf" and sdf.get("version") == "1.7", "unexpected SDF")
    worlds = sdf.findall("world")
    require(len(worlds) == 1 and worlds[0].get("name") == "default", "unexpected world")
    models = worlds[0].findall("model")
    require([x.get("name") for x in models] == ["ground_plane"], "world model set mismatch")
    model = models[0]
    require((model.findtext("static") or "").strip().lower() == "true" and identity_pose(model.find("pose")), "ground model mismatch")
    links = model.findall("link")
    require(len(links) == 1 and links[0].get("name") == "link" and identity_pose(links[0].find("pose")), "ground link mismatch")
    collisions = links[0].findall("collision")
    require(len(collisions) == 1 and collisions[0].get("name") == "collision" and identity_pose(collisions[0].find("pose")), "ground collision mismatch")
    plane = collisions[0].find("geometry/plane")
    require(plane is not None, "ground is not plane")
    normal, size = vec(plane.findtext("normal"), 3, [0,0,1]), vec(plane.findtext("size"), 2, [0,0])
    require(np.max(np.abs(normal - [0,0,1])) <= 1e-12 and np.max(np.abs(size - [6,6])) <= 1e-12, "plane geometry mismatch")
    return {"sdf_version":"1.7", "world_name":"default", "model_name":"ground_plane", "model_static":True,
            "link_name":"link", "collision_name":"collision", "pose_world":[0.0]*6,
            "plane_normal_world":normal.tolist(), "declared_size_m":size.tolist(),
            "plane_equation_world":[0.0,0.0,1.0,0.0], "moveit_representation":"shapes::Plane"}


def spawn_binding(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    fragment = '-x 0.0 -y 0.0 -z 0.03 -Y 0.0'
    require(text.count(fragment) == 1 and text.count("spawn_entity.py") == 1, "spawn contract mismatch")
    require('-entity "${MODEL_NAME}"' in text and '-file "${MOTION_URDF}"' in text, "spawn binding missing")
    return {"source":path.name, "requested_model_pose_world":{"position_m":[0,0,0.03],"rpy_rad":[0,0,0]}, "exact_command_fragment":fragment}


def match(simulation: dict[str, Any], frames: dict[str, Any], root_to_base: np.ndarray) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    source = simulation.get("evidence", {}).get("joint_state_samples", [])
    observed = frames.get("samples", [])
    require(simulation.get("joint_order") == JOINTS and frames.get("arm_joint_order") == JOINTS, "joint order mismatch")
    require(len(source) >= 100 and len(observed) >= len(source), "insufficient samples")
    by_stamp: dict[int, list[dict[str, Any]]] = {}
    for item in observed:
        by_stamp.setdefault(int(item.get("ros_stamp_ns", -1)), []).append(item)
    rows, model_to_base, footprint_residuals = [], [], []
    used: set[int] = set()
    ages_model, ages_link, position_errors = [], [], []
    for index, item in enumerate(source):
        stamp = int(item.get("ros_stamp_ns", -1))
        positions = np.asarray(item.get("positions_rad"), dtype=float)
        require(positions.shape == (6,) and np.isfinite(positions).all(), f"bad state {index}")
        candidates = []
        for candidate in by_stamp.get(stamp, []):
            if id(candidate) in used:
                continue
            p = np.asarray(candidate.get("positions_rad"), dtype=float)
            if p.shape == (6,) and np.isfinite(p).all():
                candidates.append((float(np.max(np.abs(p-positions))), candidate))
        require(candidates, f"no frame match for sample {index}")
        error, candidate = min(candidates, key=lambda x:x[0])
        require(error <= MAX_POSITION_ERROR_RAD, f"position mismatch {error}")
        used.add(id(candidate)); position_errors.append(error)
        model_state, base_state = candidate.get("model_state"), candidate.get("base_link_state")
        require(model_state is not None and base_state is not None, f"missing pose {index}")
        ma, la = float(candidate.get("model_state_age_s")), float(candidate.get("base_link_state_age_s"))
        require(0 <= ma <= MAX_AGE_S and 0 <= la <= MAX_AGE_S, f"stale pose {index}: {ma},{la}")
        ages_model.append(ma); ages_link.append(la)
        world_model, world_base = pose_matrix(model_state["pose_world"]), pose_matrix(base_state["pose_world"])
        world_root = world_base @ np.linalg.inv(root_to_base)
        model_to_base.append(np.linalg.inv(world_model) @ world_base)
        if candidate.get("base_footprint_state") is not None:
            footprint_residuals.append(residual(world_root, pose_matrix(candidate["base_footprint_state"]["pose_world"])))
        rows.append({"label":f"sample_{index:06d}", "positions":positions.tolist(), "world_root":world_root})
    reference = model_to_base[0]
    fixed = [residual(reference, x) for x in model_to_base]
    max_fixed_m, max_fixed_rad = max(x[0] for x in fixed), max(x[1] for x in fixed)
    require(max_fixed_m <= MAX_FIXED_RESIDUAL_M and max_fixed_rad <= MAX_FIXED_RESIDUAL_RAD, "model/base transform not fixed")
    max_fp_m = max((x[0] for x in footprint_residuals), default=0.0)
    max_fp_rad = max((x[1] for x in footprint_residuals), default=0.0)
    require(max_fp_m <= MAX_FIXED_RESIDUAL_M and max_fp_rad <= MAX_FIXED_RESIDUAL_RAD, "root observation mismatch")
    root_positions = np.asarray([x["world_root"][:3,3] for x in rows])
    return rows, {
        "simulation_sample_count":len(source), "observer_sample_count":len(observed), "matched_sample_count":len(rows),
        "all_samples_matched":len(rows)==len(source), "maximum_position_match_error_rad":max(position_errors),
        "maximum_model_state_age_s":max(ages_model), "maximum_base_link_state_age_s":max(ages_link),
        "maximum_allowed_frame_age_s":MAX_AGE_S, "model_to_base_link_matrix_reference":reference.tolist(),
        "maximum_model_to_base_link_translation_residual_m":max_fixed_m,
        "maximum_model_to_base_link_rotation_residual_rad":max_fixed_rad,
        "base_footprint_directly_observed":bool(footprint_residuals),
        "maximum_inferred_to_observed_root_translation_residual_m":max_fp_m,
        "maximum_inferred_to_observed_root_rotation_residual_rad":max_fp_rad,
        "root_position_world_min_m":np.min(root_positions,axis=0).tolist(),
        "root_position_world_max_m":np.max(root_positions,axis=0).tolist(),
    }


def write_states(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            matrix = row["world_root"]
            values = matrix[:3,3].tolist() + matrix_quat(matrix[:3,:3]) + row["positions"]
            stream.write(row["label"] + "\t" + "\t".join(f"{x:.17g}" for x in values) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation-summary", required=True)
    parser.add_argument("--ground-frame-trace", required=True)
    parser.add_argument("--simulation-urdf", required=True)
    parser.add_argument("--moveit-urdf", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--base-runner", required=True)
    parser.add_argument("--states-tsv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--source-ref", required=True)
    args = parser.parse_args()
    simulation = json.loads(Path(args.simulation_summary).read_text())
    frames = json.loads(Path(args.ground_frame_trace).read_text())
    require(simulation.get("source_ref") == args.source_ref and frames.get("source_ref") == args.source_ref, "source_ref mismatch")
    require(simulation.get("passed") is True and frames.get("passed") is True, "upstream evidence failed")
    safety = frames.get("safety", {})
    require(safety.get("read_only_subscriptions_only") is True and all(safety.get(k) is False for k in ("publisher_created","service_client_created","action_client_created","command_sent")), "observer safety mismatch")
    sim_urdf, moveit_urdf = urdf_binding(Path(args.simulation_urdf)), urdf_binding(Path(args.moveit_urdf))
    require(sim_urdf["robot_name"] == "turtlebot3_lime_arm_motion_test" and moveit_urdf["robot_name"] == "turtlebot3_lime", "URDF names mismatch")
    require(not sim_urdf["missing_audited_collision_geometry"] and not moveit_urdf["missing_audited_collision_geometry"], "collision geometry missing")
    sim_tf, moveit_tf = np.asarray(sim_urdf["root_to_base_link_matrix"]), np.asarray(moveit_urdf["root_to_base_link_matrix"])
    require(np.max(np.abs(sim_tf-moveit_tf)) <= 1e-12, "root transform mismatch")
    require(sim_urdf["audited_collision_geometry_sha256"] == moveit_urdf["audited_collision_geometry_sha256"], "collision geometry mismatch")
    rows, dynamic = match(simulation, frames, sim_tf)
    write_states(Path(args.states_tsv), rows)
    payload = {
        "schema_version":1, "phase":"RUBIK-PREGRASP-GROUND-PLANE-FRAME-BINDING", "source_ref":args.source_ref,
        "passed":True, "errors":[], "state_count":len(rows), "world":world_binding(Path(args.world)),
        "spawn_contract":spawn_binding(Path(args.base_runner)),
        "robot_model_binding":{"simulation_urdf":sim_urdf,"moveit_urdf":moveit_urdf,"root_transform_exact_match":True,"audited_collision_geometry_exact_match":True},
        "dynamic_frame_binding":dynamic, "output_states_tsv":Path(args.states_tsv).name,
        "safety":{"offline_transform_derivation_only":True,"same_two_arm_goals_only":True,"new_command_path_added":False,"ground_object_added_to_gazebo":False,"production_runtime_modified":False,"physical_hardware_used":False},
    }
    Path(args.output).write_text(json.dumps(payload,indent=2,sort_keys=True)+"\n")
    print(json.dumps(payload,indent=2,sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
