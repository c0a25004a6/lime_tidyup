#!/usr/bin/env python3
"""Compare Gazebo truth Cube8 pose against the bridged proxy pose."""
from __future__ import annotations

import json
import math


def pose_error(a, b):
    translation = math.sqrt(
        (a.position.x - b.position.x) ** 2
        + (a.position.y - b.position.y) ** 2
        + (a.position.z - b.position.z) ** 2
    )
    qa = (a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w)
    qb = (b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w)
    na = math.sqrt(sum(v * v for v in qa))
    nb = math.sqrt(sum(v * v for v in qb))
    if na < 1e-12 or nb < 1e-12:
        raise ValueError("zero quaternion in Gazebo ModelStates")
    dot = abs(sum((qa[i] / na) * (qb[i] / nb) for i in range(4)))
    angle = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
    return translation, angle


def main(args=None):
    import rclpy
    from gazebo_msgs.msg import ModelStates
    from rclpy.node import Node
    from std_msgs.msg import String

    class Compare(Node):
        def __init__(self):
            super().__init__("cube8_sim_compare")
            self.declare_parameter("truth_model", "cube8_truth")
            self.declare_parameter("proxy_model", "cube8_pose_proxy")
            self.declare_parameter("translation_tolerance_m", 0.001)
            self.declare_parameter("orientation_tolerance_rad", 0.01)
            self.publisher = self.create_publisher(String, "/cube8_sim/roundtrip_status", 10)
            self.create_subscription(ModelStates, "/gazebo/model_states", self.on_states, 10)
            self.last_status = None

        def on_states(self, msg):
            truth_name = str(self.get_parameter("truth_model").value)
            proxy_name = str(self.get_parameter("proxy_model").value)
            try:
                truth = msg.pose[msg.name.index(truth_name)]
                proxy = msg.pose[msg.name.index(proxy_name)]
            except ValueError:
                return
            translation, angle = pose_error(truth, proxy)
            passed = (
                translation <= float(self.get_parameter("translation_tolerance_m").value)
                and angle <= float(self.get_parameter("orientation_tolerance_rad").value)
            )
            status = "PASS" if passed else "FAIL"
            payload = {
                "status": status,
                "truth_model": truth_name,
                "proxy_model": proxy_name,
                "translation_error_m": translation,
                "orientation_error_rad": angle,
                "physical_robot_authority": False,
            }
            out = String()
            out.data = json.dumps(payload)
            self.publisher.publish(out)
            if status != self.last_status:
                log = self.get_logger().info if passed else self.get_logger().error
                log(
                    f"CUBE8_SIM_ROUNDTRIP_{status} translation={translation:.6f}m "
                    f"orientation={angle:.6f}rad"
                )
                self.last_status = status

    rclpy.init(args=args)
    node = Compare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
