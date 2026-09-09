#!/usr/bin/env python3
"""Spawn a deterministic 57 mm Rubik-style Cube8 target and sim camera.

Simulation evidence only. The spawned cube is static and this module does not
command the TurtleBot, arm, gripper, or any physical hardware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


CUBE_SIDE_M = 0.057
CAMERA_MODEL = "cube8_sim_camera"
TRUTH_MODEL = "cube8_truth"
CAMERA_FRAME = "cube8_sim_camera_optical_frame"

# Gazebo camera link uses +X forward/+Y left/+Z up. ROS optical is
# +Z forward/+X right/+Y down. This quaternion is world <- optical for a
# camera link at identity orientation.
WORLD_FROM_CAMERA_XYZ = (0.0, 0.0, 0.45)
WORLD_FROM_CAMERA_QXYZW = (0.5, -0.5, 0.5, -0.5)

# Canonical Cube8 truth in the ROS optical frame.
CAMERA_FROM_CUBE_XYZ = (0.02, 0.01, 0.65)
CAMERA_FROM_CUBE_QXYZW = (
    0.1106179384,
    -0.1224724734,
    0.1743586059,
    0.9707539540,
)

# Exact composition of the two transforms above, used to spawn truth.
WORLD_FROM_CUBE_XYZ = (0.65, -0.02, 0.44)
WORLD_FROM_CUBE_QXYZW = (
    0.4041249420,
    -0.4560110746,
    0.3922704070,
    -0.6891014866,
)


@dataclass(frozen=True)
class Pose:
    xyz: tuple[float, float, float]
    qxyzw: tuple[float, float, float, float]


def _normalize_quaternion(values):
    q = tuple(float(v) for v in values)
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError("invalid quaternion")
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        raise ValueError("zero quaternion")
    return tuple(v / n for v in q)


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(q, v):
    qx, qy, qz, qw = _normalize_quaternion(q)
    vx, vy, vz = v
    return (
        (1 - 2 * (qy * qy + qz * qz)) * vx
        + 2 * (qx * qy - qz * qw) * vy
        + 2 * (qx * qz + qy * qw) * vz,
        2 * (qx * qy + qz * qw) * vx
        + (1 - 2 * (qx * qx + qz * qz)) * vy
        + 2 * (qy * qz - qx * qw) * vz,
        2 * (qx * qz - qy * qw) * vx
        + 2 * (qy * qz + qx * qw) * vy
        + (1 - 2 * (qx * qx + qy * qy)) * vz,
    )


def compose_pose(world_from_parent: Pose, parent_from_child: Pose) -> Pose:
    rotated = _rotate_vector(world_from_parent.qxyzw, parent_from_child.xyz)
    xyz = tuple(world_from_parent.xyz[i] + rotated[i] for i in range(3))
    q = _normalize_quaternion(
        _quat_multiply(
            _normalize_quaternion(world_from_parent.qxyzw),
            _normalize_quaternion(parent_from_child.qxyzw),
        )
    )
    return Pose(xyz=xyz, qxyzw=q)


def build_rubiks_sdf(entity_name: str = TRUTH_MODEL) -> str:
    """Return a 57 mm outer-envelope, 3x3x3 Rubik-style visual model."""
    cubie = 0.018
    pitch = 0.019
    sticker = 0.0155
    sticker_thickness = 0.0005
    sticker_center = CUBE_SIDE_M / 2.0 - sticker_thickness / 2.0
    positions = (-pitch, 0.0, pitch)
    colors = {
        "+x": (1.0, 0.0, 0.0, 1.0),
        "-x": (1.0, 0.35, 0.0, 1.0),
        "+y": (0.0, 0.75, 0.0, 1.0),
        "-y": (0.0, 0.2, 1.0, 1.0),
        "+z": (1.0, 1.0, 1.0, 1.0),
        "-z": (1.0, 0.9, 0.0, 1.0),
    }
    out = [
        "<?xml version='1.0'?>",
        "<sdf version='1.6'>",
        f"  <model name='{entity_name}'>",
        "    <static>true</static>",
        "    <link name='body'>",
    ]
    for ix, x in enumerate(positions):
        for iy, y in enumerate(positions):
            for iz, z in enumerate(positions):
                out.extend(
                    [
                        f"      <visual name='cubie_{ix}{iy}{iz}'>",
                        f"        <pose>{x} {y} {z} 0 0 0</pose>",
                        f"        <geometry><box><size>{cubie} {cubie} {cubie}</size></box></geometry>",
                        "        <material><ambient>0.015 0.015 0.015 1</ambient><diffuse>0.025 0.025 0.025 1</diffuse></material>",
                        "      </visual>",
                    ]
                )
    for face, rgba in colors.items():
        for a, u in enumerate(positions):
            for b, v in enumerate(positions):
                sign = 1.0 if face[0] == "+" else -1.0
                if face[1] == "x":
                    xyz = (sign * sticker_center, u, v)
                    size = (sticker_thickness, sticker, sticker)
                elif face[1] == "y":
                    xyz = (u, sign * sticker_center, v)
                    size = (sticker, sticker_thickness, sticker)
                else:
                    xyz = (u, v, sign * sticker_center)
                    size = (sticker, sticker, sticker_thickness)
                r, g, blue, alpha = rgba
                label = face.replace("+", "p").replace("-", "n")
                out.extend(
                    [
                        f"      <visual name='sticker_{label}_{a}{b}'>",
                        f"        <pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0</pose>",
                        f"        <geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>",
                        f"        <material><ambient>{r} {g} {blue} {alpha}</ambient><diffuse>{r} {g} {blue} {alpha}</diffuse></material>",
                        "      </visual>",
                    ]
                )
    out.extend(["    </link>", "  </model>", "</sdf>"])
    return "\n".join(out) + "\n"


def build_camera_sdf(entity_name: str = CAMERA_MODEL) -> str:
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{entity_name}'>
    <static>true</static>
    <link name='camera_link'>
      <sensor name='cube8_sim_camera' type='camera'>
        <always_on>true</always_on>
        <update_rate>20</update_rate>
        <visualize>true</visualize>
        <camera>
          <horizontal_fov>1.085595</horizontal_fov>
          <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>5.0</far></clip>
        </camera>
        <plugin name='cube8_sim_camera_ros' filename='libgazebo_ros_camera.so'>
          <ros><namespace>/cube8_sim</namespace></ros>
          <camera_name>camera</camera_name>
          <frame_name>{CAMERA_FRAME}</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
"""


def main(args=None):
    import rclpy
    from gazebo_msgs.srv import SpawnEntity
    from geometry_msgs.msg import Pose as RosPose
    from rclpy.node import Node

    def ros_pose(pose: Pose) -> RosPose:
        msg = RosPose()
        msg.position.x, msg.position.y, msg.position.z = pose.xyz
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = pose.qxyzw
        return msg

    class Setup(Node):
        def __init__(self):
            super().__init__("cube8_sim_setup")
            self.client = self.create_client(SpawnEntity, "/spawn_entity")

        def spawn(self, name: str, xml: str, pose: Pose):
            if not self.client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError("/spawn_entity unavailable")
            request = SpawnEntity.Request()
            request.name = name
            request.xml = xml
            request.robot_namespace = ""
            request.initial_pose = ros_pose(pose)
            request.reference_frame = "world"
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            result = future.result()
            if result is None or not result.success:
                status = "no response" if result is None else result.status_message
                raise RuntimeError(f"spawn {name!r} failed: {status}")
            self.get_logger().info(f"spawned {name}")

    camera_pose = Pose(WORLD_FROM_CAMERA_XYZ, (0.0, 0.0, 0.0, 1.0))
    cube_pose = Pose(WORLD_FROM_CUBE_XYZ, WORLD_FROM_CUBE_QXYZW)
    rclpy.init(args=args)
    node = Setup()
    try:
        node.spawn(CAMERA_MODEL, build_camera_sdf(), camera_pose)
        node.spawn(TRUTH_MODEL, build_rubiks_sdf(), cube_pose)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
