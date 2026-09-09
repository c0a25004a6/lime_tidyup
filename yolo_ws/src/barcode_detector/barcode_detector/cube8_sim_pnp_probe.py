#!/usr/bin/env python3
"""CameraInfo -> synthetic sim truth projection -> existing Cube8 PnP core.

This intentionally bypasses learned perception so the first sim gate isolates
camera calibration, Cube8 geometry/PnP, frame composition, and Gazebo bridging.
"""
from __future__ import annotations

import json
import math
import time

from .cube8_geometry import PoseKeypoint2D, cube_vertices
from .cube8_pnp import CameraModel, estimate_cube_pose
from .cube8_sim_setup import (
    CAMERA_FRAME,
    CAMERA_FROM_CUBE_QXYZW,
    CAMERA_FROM_CUBE_XYZ,
    CUBE_SIDE_M,
)


def _normalize(q):
    n = math.sqrt(sum(float(v) * float(v) for v in q))
    if n < 1e-12:
        raise ValueError("zero quaternion")
    return tuple(float(v) / n for v in q)


def _rotate(q, v):
    qx, qy, qz, qw = _normalize(q)
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


def project_truth(camera_k, side_length_m=CUBE_SIDE_M):
    if len(camera_k) != 9:
        raise ValueError("CameraInfo K must contain 9 values")
    fx, fy = float(camera_k[0]), float(camera_k[4])
    cx, cy = float(camera_k[2]), float(camera_k[5])
    if fx <= 0 or fy <= 0:
        raise ValueError("invalid CameraInfo focal length")
    points = []
    for vertex in cube_vertices(side_length_m):
        rx, ry, rz = _rotate(
            CAMERA_FROM_CUBE_QXYZW,
            (vertex.x, vertex.y, vertex.z),
        )
        x = CAMERA_FROM_CUBE_XYZ[0] + rx
        y = CAMERA_FROM_CUBE_XYZ[1] + ry
        z = CAMERA_FROM_CUBE_XYZ[2] + rz
        if z <= 0:
            raise ValueError("truth vertex is behind sim camera")
        points.append(PoseKeypoint2D(vertex.label, fx * x / z + cx, fy * y / z + cy, 1.0, 2))
    return tuple(points)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import String

    class Probe(Node):
        def __init__(self):
            super().__init__("cube8_sim_pnp_probe")
            self.declare_parameter("camera_info_topic", "/cube8_sim/camera/camera_info")
            self.declare_parameter("result_topic", "/cube8_pose_result")
            self.declare_parameter("publish_period_sec", 0.25)
            self.camera_info = None
            self.publisher = self.create_publisher(String, str(self.get_parameter("result_topic").value), 10)
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self.on_camera_info,
                10,
            )
            self.create_timer(float(self.get_parameter("publish_period_sec").value), self.publish_probe)
            self.get_logger().warning(
                "Cube8 sim PnP probe uses Gazebo truth projection; learned perception is intentionally bypassed"
            )

        def on_camera_info(self, msg):
            self.camera_info = msg

        def publish_probe(self):
            ci = self.camera_info
            if ci is None:
                return
            if ci.header.frame_id != CAMERA_FRAME:
                self.get_logger().error(
                    f"unexpected CameraInfo frame {ci.header.frame_id!r}; expected {CAMERA_FRAME!r}"
                )
                return
            if any(abs(float(v)) > 1e-12 for v in ci.d):
                self.get_logger().error("sim probe currently requires zero-distortion CameraInfo")
                return
            try:
                keypoints = project_truth(tuple(ci.k), CUBE_SIDE_M)
                pose = estimate_cube_pose(
                    keypoints,
                    CameraModel(tuple(ci.k), tuple(ci.d)),
                    side_length_m=CUBE_SIDE_M,
                    minimum_keypoint_confidence=0.5,
                    minimum_correspondences=4,
                    minimum_inliers=4,
                    maximum_reprojection_error_px=1.0,
                )
            except Exception as exc:
                self.get_logger().error(f"sim PnP probe rejected: {type(exc).__name__}: {exc}")
                return
            frame = ci.header.frame_id
            translation_error = math.sqrt(
                sum((pose.translation_m[i] - CAMERA_FROM_CUBE_XYZ[i]) ** 2 for i in range(3))
            )
            truth_q = _normalize(CAMERA_FROM_CUBE_QXYZW)
            dot = abs(sum(pose.orientation_xyzw[i] * truth_q[i] for i in range(4)))
            dot = max(-1.0, min(1.0, dot))
            angle_error = 2.0 * math.acos(dot)
            payload = {
                "timestamp": time.time(),
                "frame_id": frame,
                "pipeline": "gazebo_truth_projection+cube8_pnp",
                "perception_frontend": "sim_truth_projection",
                "retained_model_qualification": "not_used",
                "two_face_geometry_qualification": "not_used",
                "physical_robot_authority": False,
                "sim_probe": {
                    "truth_projection_only": True,
                    "translation_error_m": translation_error,
                    "orientation_error_rad": angle_error,
                },
                "detections": [
                    {
                        "class_id": -1,
                        "class_name": "cube_sim_truth",
                        "detector_confidence": 1.0,
                        "perception_frontend": "sim_truth_projection",
                        "pnp": {
                            "status": "accepted",
                            "frame_id": frame,
                            "semantic_frame": "color_fixed_cube_axis",
                            **pose.as_dict(),
                        },
                        "grasp": {
                            "status": "not_computed",
                            "cube_symmetry_equivalent_pose": False,
                            "execution_authorized": False,
                        },
                    }
                ],
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.publisher.publish(msg)

    rclpy.init(args=args)
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
