#!/usr/bin/env python3
"""Fail-closed Cube8 sim PnP result -> Gazebo proxy bridge."""
from __future__ import annotations

import json
import math
import time

from .cube8_sim_setup import (
    CAMERA_FRAME,
    CUBE_SIDE_M,
    Pose,
    WORLD_FROM_CAMERA_QXYZW,
    WORLD_FROM_CAMERA_XYZ,
    compose_pose,
)


PROXY_MODEL = "cube8_pose_proxy"


def select_world_pose(payload, now=None, max_age_sec=2.0):
    if payload.get("physical_robot_authority") is not False:
        raise ValueError("payload must explicitly deny physical robot authority")
    timestamp = float(payload.get("timestamp", math.nan))
    if not math.isfinite(timestamp):
        raise ValueError("missing/non-finite timestamp")
    age = (time.time() if now is None else float(now)) - timestamp
    if age < -1.0 or age > float(max_age_sec):
        raise ValueError(f"stale/future payload age={age:.3f}s")
    detections = payload.get("detections")
    if not isinstance(detections, list):
        raise ValueError("detections must be a list")
    accepted = [d for d in detections if isinstance(d, dict) and d.get("pnp", {}).get("status") == "accepted"]
    if len(accepted) != 1:
        raise ValueError(f"exactly one accepted PnP result required, got {len(accepted)}")
    pnp = accepted[0]["pnp"]
    if str(pnp.get("frame_id", "")) != CAMERA_FRAME:
        raise ValueError(f"unexpected PnP frame {pnp.get('frame_id')!r}")
    xyz = tuple(float(v) for v in pnp.get("translation_m", ()))
    q = tuple(float(v) for v in pnp.get("orientation_xyzw", ()))
    if len(xyz) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in (*xyz, *q)):
        raise ValueError("invalid PnP pose")
    world_from_camera = Pose(WORLD_FROM_CAMERA_XYZ, WORLD_FROM_CAMERA_QXYZW)
    return compose_pose(world_from_camera, Pose(xyz, q))


def proxy_sdf(side_length_m=CUBE_SIDE_M):
    side = float(side_length_m)
    if not math.isfinite(side) or side <= 0:
        raise ValueError("invalid proxy side length")
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{PROXY_MODEL}'>
    <static>true</static>
    <link name='body'>
      <visual name='proxy'>
        <geometry><box><size>{side} {side} {side}</size></box></geometry>
        <material><ambient>0.1 0.75 1 0.45</ambient><diffuse>0.1 0.75 1 0.45</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def main(args=None):
    import rclpy
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState, SpawnEntity
    from geometry_msgs.msg import Pose as RosPose
    from rclpy.node import Node
    from std_msgs.msg import String

    def ros_pose(pose):
        msg = RosPose()
        msg.position.x, msg.position.y, msg.position.z = pose.xyz
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = pose.qxyzw
        return msg

    class Bridge(Node):
        def __init__(self):
            super().__init__("cube8_sim_gazebo_bridge")
            self.spawned = False
            self.spawn_inflight = False
            self.spawn = self.create_client(SpawnEntity, "/spawn_entity")
            self.set_state = self.create_client(SetEntityState, "/gazebo/set_entity_state")
            self.create_subscription(String, "/cube8_pose_result", self.on_result, 10)
            self.get_logger().warning("diagnostic sim bridge only; no physical robot authority")

        def on_result(self, msg):
            try:
                world_pose = select_world_pose(json.loads(msg.data))
            except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                self.get_logger().warning(f"Cube8 sim bridge rejected result: {exc}")
                return
            if not self.spawned:
                if self.spawn_inflight:
                    return
                if not self.spawn.wait_for_service(timeout_sec=0.05):
                    return
                request = SpawnEntity.Request()
                request.name = PROXY_MODEL
                request.xml = proxy_sdf()
                request.robot_namespace = ""
                request.initial_pose = ros_pose(world_pose)
                request.reference_frame = "world"
                self.spawn_inflight = True
                future = self.spawn.call_async(request)
                future.add_done_callback(self.on_spawn)
                return
            if not self.set_state.wait_for_service(timeout_sec=0.05):
                return
            request = SetEntityState.Request()
            request.state = EntityState()
            request.state.name = PROXY_MODEL
            request.state.pose = ros_pose(world_pose)
            request.state.reference_frame = "world"
            self.set_state.call_async(request)

        def on_spawn(self, future):
            self.spawn_inflight = False
            try:
                result = future.result()
                self.spawned = bool(result.success)
                if self.spawned:
                    self.get_logger().info(f"spawned {PROXY_MODEL}")
                else:
                    self.get_logger().error(f"proxy spawn failed: {result.status_message}")
            except Exception as exc:
                self.get_logger().error(f"proxy spawn service failed: {exc}")

    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
