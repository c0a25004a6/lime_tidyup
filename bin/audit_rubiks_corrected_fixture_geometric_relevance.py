#!/usr/bin/env python3
"""Audit necessary two-finger relevance from exact URDF-referenced STL vertices."""
from __future__ import annotations

import argparse
import math
import struct
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

TOLERANCE = 1e-12
EXPECTED_LINKS = ("gripper_left_link", "gripper_right_link")
EXPECTED_PACKAGE = "package://turtlebot3_lime_description/"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def number(text: str) -> float:
    value = float(text)
    require(math.isfinite(value), f"nonfinite number: {text}")
    return value


def vector(text: str, count: int = 3) -> list[float]:
    values = [number(item) for item in text.split()]
    require(len(values) == count, f"invalid vector: {text}")
    return values


def close_vector(actual: list[float], expected: list[float], tolerance: float = TOLERANCE) -> bool:
    return len(actual) == len(expected) and all(abs(a - b) <= tolerance for a, b in zip(actual, expected))


@dataclass(frozen=True)
class Aabb:
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    @property
    def center(self) -> tuple[float, float, float]:
        return tuple((low + high) * 0.5 for low, high in zip(self.minimum, self.maximum))

    @property
    def extents(self) -> tuple[float, float, float]:
        return tuple(high - low for low, high in zip(self.minimum, self.maximum))

    def translated(self, delta: list[float]) -> "Aabb":
        return Aabb(
            tuple(value + shift for value, shift in zip(self.minimum, delta)),
            tuple(value + shift for value, shift in zip(self.maximum, delta)),
        )


@dataclass(frozen=True)
class MeshContract:
    link: str
    resource: str
    path: Path
    scale: tuple[float, float, float]
    collision_origin: tuple[float, float, float]
    joint_origin: tuple[float, float, float]
    axis: tuple[float, float, float]
    aabb_local: Aabb
    triangle_count: int
    vertex_count: int
    stl_format: str


def stl_vertices(path: Path) -> tuple[list[tuple[float, float, float]], int, str]:
    data = path.read_bytes()
    require(len(data) >= 15, f"STL is too small: {path}")
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + triangle_count * 50:
            vertices: list[tuple[float, float, float]] = []
            for index in range(triangle_count):
                values = struct.unpack_from("<12fH", data, 84 + index * 50)
                for start in (3, 6, 9):
                    point = (float(values[start]), float(values[start + 1]), float(values[start + 2]))
                    require(all(math.isfinite(value) for value in point), f"binary STL contains nonfinite vertex: {path}")
                    vertices.append(point)
            require(vertices, f"binary STL has no vertices: {path}")
            return vertices, triangle_count, "binary"
    text = data.decode("ascii")
    vertices = []
    for raw in text.splitlines():
        fields = raw.strip().split()
        if len(fields) == 4 and fields[0] == "vertex":
            point = tuple(number(value) for value in fields[1:4])
            vertices.append(point)
    require(vertices and len(vertices) % 3 == 0, f"invalid ASCII STL: {path}")
    return vertices, len(vertices) // 3, "ascii"


def aabb_from_vertices(
    vertices: list[tuple[float, float, float]],
    scale: list[float],
    origin: list[float],
) -> Aabb:
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    for vertex in vertices:
        point = [vertex[index] * scale[index] + origin[index] for index in range(3)]
        for index, value in enumerate(point):
            minimum[index] = min(minimum[index], value)
            maximum[index] = max(maximum[index], value)
    require(all(math.isfinite(value) for value in minimum + maximum), "mesh AABB is nonfinite")
    require(all(high > low for low, high in zip(minimum, maximum)), "mesh AABB has nonpositive extent")
    return Aabb(tuple(minimum), tuple(maximum))


def read_contract(path: Path) -> dict[str, object]:
    result: dict[str, object] = {}
    for raw in path.read_text().splitlines():
        if not raw:
            continue
        fields = raw.split("\t")
        if fields[0] == "META":
            require(len(fields) == 5 and fields[1] == "1", "invalid META row")
            result["state_count"] = int(fields[2])
            result["anchor_index"] = int(fields[3])
            result["reference_link"] = fields[4]
        elif fields[0] in {"CUBE", "SUPPORT"}:
            require(len(fields) == 7, f"invalid {fields[0]} row")
            result[fields[0].lower()] = {
                "size": [number(value) for value in fields[1:4]],
                "center": [number(value) for value in fields[4:7]],
            }
        elif fields[0] == "TRANSLATION":
            require(len(fields) == 4, "invalid TRANSLATION row")
            result["translation"] = [number(value) for value in fields[1:4]]
        elif fields[0] == "GRIPPER":
            require(len(fields) == 5, "invalid GRIPPER row")
            result["gripper"] = [number(value) for value in fields[1:5]]
        elif fields[0] == "ANCHOR":
            require(len(fields) == 8, "invalid ANCHOR row")
            result["anchor_label"] = fields[1]
            result["anchor_arm"] = [number(value) for value in fields[2:8]]
        else:
            raise ValueError(f"unknown contract row: {fields[0]}")
    require(
        set(result) == {
            "state_count", "anchor_index", "reference_link", "cube", "support",
            "translation", "gripper", "anchor_label", "anchor_arm",
        },
        "contract is incomplete",
    )
    require(result["state_count"] == 2802, "state count mismatch")
    require(result["anchor_index"] == 1541, "anchor index mismatch")
    require(result["reference_link"] == "link7", "reference link mismatch")
    require(result["anchor_label"] == "sample_001541", "anchor label mismatch")
    return result


def load_mesh_contracts(urdf_path: Path, package_share: Path) -> dict[str, MeshContract]:
    root = ET.parse(urdf_path).getroot()
    links = {str(element.get("name")): element for element in root.findall("link")}
    joints = {str(element.get("name")): element for element in root.findall("joint")}
    result: dict[str, MeshContract] = {}
    expected = {
        "gripper_left_link": ("gripper_left_joint", [0.0, 1.0, 0.0]),
        "gripper_right_link": ("gripper_right_joint", [0.0, -1.0, 0.0]),
    }
    for link_name, (joint_name, expected_axis) in expected.items():
        link = links.get(link_name)
        joint = joints.get(joint_name)
        require(link is not None and joint is not None, f"missing URDF geometry: {link_name}")
        require(joint.get("type") == "prismatic", f"joint is not prismatic: {joint_name}")
        parent = joint.find("parent")
        child = joint.find("child")
        require(parent is not None and parent.get("link") == "link7", f"parent mismatch: {joint_name}")
        require(child is not None and child.get("link") == link_name, f"child mismatch: {joint_name}")
        joint_origin_element = joint.find("origin")
        require(joint_origin_element is not None, f"joint origin missing: {joint_name}")
        joint_origin = vector(str(joint_origin_element.get("xyz", "")))
        require(close_vector(vector(str(joint_origin_element.get("rpy", "0 0 0"))), [0.0, 0.0, 0.0]), f"joint rotation mismatch: {joint_name}")
        axis_element = joint.find("axis")
        require(axis_element is not None, f"joint axis missing: {joint_name}")
        axis = vector(str(axis_element.get("xyz", "")))
        require(close_vector(axis, expected_axis), f"joint axis mismatch: {joint_name}")

        collisions = link.findall("collision")
        require(len(collisions) == 1, f"collision count mismatch: {link_name}")
        collision = collisions[0]
        collision_origin_element = collision.find("origin")
        collision_origin = [0.0, 0.0, 0.0]
        if collision_origin_element is not None:
            collision_origin = vector(str(collision_origin_element.get("xyz", "0 0 0")))
            require(close_vector(vector(str(collision_origin_element.get("rpy", "0 0 0"))), [0.0, 0.0, 0.0]), f"collision rotation mismatch: {link_name}")
        mesh = collision.find("geometry/mesh")
        require(mesh is not None, f"collision mesh missing: {link_name}")
        resource = str(mesh.get("filename", ""))
        require(resource.startswith(EXPECTED_PACKAGE), f"mesh package mismatch: {link_name}")
        scale = vector(str(mesh.get("scale", "")))
        require(close_vector(scale, [0.001, 0.001, 0.001]), f"mesh scale mismatch: {link_name}")
        mesh_path = (package_share / resource[len(EXPECTED_PACKAGE):]).resolve()
        require(mesh_path.is_file(), f"mesh file missing: {mesh_path}")
        require(package_share.resolve() in mesh_path.parents, f"mesh escaped package share: {mesh_path}")
        vertices, triangle_count, stl_format = stl_vertices(mesh_path)
        local_aabb = aabb_from_vertices(vertices, scale, collision_origin)
        result[link_name] = MeshContract(
            link=link_name,
            resource=resource,
            path=mesh_path,
            scale=tuple(scale),
            collision_origin=tuple(collision_origin),
            joint_origin=tuple(joint_origin),
            axis=tuple(axis),
            aabb_local=local_aabb,
            triangle_count=triangle_count,
            vertex_count=len(vertices),
            stl_format=stl_format,
        )
    return result


def transformed(mesh: MeshContract, position: float) -> Aabb:
    delta = [mesh.joint_origin[index] + mesh.axis[index] * position for index in range(3)]
    return mesh.aabb_local.translated(delta)


def overlap(first_min: float, first_max: float, second_min: float, second_max: float) -> float:
    return max(0.0, min(first_max, second_max) - max(first_min, second_min))


def contains(minimum: float, maximum: float, value: float) -> bool:
    return minimum - TOLERANCE <= value <= maximum + TOLERANCE


def write_aabb(stream, value: Aabb) -> None:
    stream.write("\t" + "\t".join(f"{item:.17g}" for item in (*value.minimum, *value.maximum)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--package-share", required=True)
    parser.add_argument("--contract", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    urdf_path = Path(args.urdf)
    package_share = Path(args.package_share).resolve()
    contract = read_contract(Path(args.contract))
    meshes = load_mesh_contracts(urdf_path, package_share)
    cube = contract["cube"]
    cube_min = [cube["center"][index] - cube["size"][index] * 0.5 for index in range(3)]
    cube_max = [cube["center"][index] + cube["size"][index] * 0.5 for index in range(3)]
    lower, zero, passive, upper = contract["gripper"]
    positions = (("lower", lower), ("zero", zero), ("passive", passive), ("upper", upper))

    audits: dict[str, dict[str, object]] = {}
    for label, position in positions:
        left = transformed(meshes["gripper_left_link"], position)
        right = transformed(meshes["gripper_right_link"], position)
        audits[label] = {
            "position": position,
            "left": left,
            "right": right,
            "left_x": overlap(left.minimum[0], left.maximum[0], cube_min[0], cube_max[0]),
            "left_z": overlap(left.minimum[2], left.maximum[2], cube_min[2], cube_max[2]),
            "right_x": overlap(right.minimum[0], right.maximum[0], cube_min[0], cube_max[0]),
            "right_z": overlap(right.minimum[2], right.maximum[2], cube_min[2], cube_max[2]),
            "left_side": contains(left.minimum[1], left.maximum[1], cube_max[1]),
            "right_side": contains(right.minimum[1], right.maximum[1], cube_min[1]),
        }

    zero_left = audits["zero"]["left"]
    zero_right = audits["zero"]["right"]
    left_low = cube_max[1] - zero_left.maximum[1]
    left_high = cube_max[1] - zero_left.minimum[1]
    right_low = zero_right.minimum[1] - cube_min[1]
    right_high = zero_right.maximum[1] - cube_min[1]
    common_low = max(lower, left_low, right_low)
    common_high = min(upper, left_high, right_high)
    common_exists = common_low <= common_high + TOLERANCE
    representative = (common_low + common_high) * 0.5 if common_exists else zero
    representative_left = transformed(meshes["gripper_left_link"], representative)
    representative_right = transformed(meshes["gripper_right_link"], representative)
    representative_left_side = contains(representative_left.minimum[1], representative_left.maximum[1], cube_max[1])
    representative_right_side = contains(representative_right.minimum[1], representative_right.maximum[1], cube_min[1])
    positive_xz = all(audits["zero"][key] > 0.0 for key in ("left_x", "left_z", "right_x", "right_z"))
    simultaneous = common_exists and representative_left_side and representative_right_side
    relevant = positive_xz and simultaneous
    decision = (
        "CORRECTED_FIXTURE_GEOMETRICALLY_RELEVANT"
        if relevant
        else "BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE"
    )

    with Path(args.output).open("w", encoding="utf-8") as stream:
        stream.write(f"META\t1\tturtlebot3_lime\tbase_footprint\t{contract['anchor_label']}\tlink7\n")
        stream.write("CUBE\t" + "\t".join(f"{value:.17g}" for value in (*cube["size"], *cube["center"])) + "\n")
        for link_name in EXPECTED_LINKS:
            mesh = meshes[link_name]
            stream.write(
                f"MESH\t{link_name}\t1\t"
                + "\t".join(f"{value:.17g}" for value in (*mesh.aabb_local.center, *mesh.aabb_local.extents))
                + "\n"
            )
        for label, _ in positions:
            audit = audits[label]
            stream.write(f"POSITION\t{label}\t{audit['position']:.17g}\t{audit['position']:.17g}\t{audit['position']:.17g}")
            write_aabb(stream, audit["left"])
            write_aabb(stream, audit["right"])
            stream.write(
                f"\t{audit['left_x']:.17g}\t{audit['left_z']:.17g}"
                f"\t{audit['right_x']:.17g}\t{audit['right_z']:.17g}"
                f"\t{'true' if audit['left_side'] else 'false'}"
                f"\t{'true' if audit['right_side'] else 'false'}\n"
            )
        stream.write(
            f"INTERVAL\t{left_low:.17g}\t{left_high:.17g}\t{right_low:.17g}\t{right_high:.17g}"
            f"\t{common_low:.17g}\t{common_high:.17g}\t{'true' if common_exists else 'false'}"
            f"\t{representative:.17g}\t{'true' if representative_left_side else 'false'}"
            f"\t{'true' if representative_right_side else 'false'}\n"
        )
        stream.write(
            f"DECISION\t{decision}\t{'true' if positive_xz else 'false'}"
            f"\t{'true' if simultaneous else 'false'}\t0\n"
        )
        for link_name in EXPECTED_LINKS:
            mesh = meshes[link_name]
            stream.write(
                f"STL\t{link_name}\t{mesh.stl_format}\t{mesh.triangle_count}\t{mesh.vertex_count}"
                f"\t{mesh.resource}\n"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
