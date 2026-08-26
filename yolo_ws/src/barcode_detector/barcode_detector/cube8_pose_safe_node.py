"""Fail-closed executable wrapper for Cube8 grasp candidate generation.

The underlying two-face frontend may produce a useful view-local PnP pose while
physical color-fixed cube orientation/chirality is still unresolved.  Keep that
pose as diagnostic evidence, but return no pose object to the base grasp stage
until the PnP result explicitly marks color-fixed semantics resolved.
"""
from __future__ import annotations

import rclpy

from .cube8_pose_node import Cube8PoseNode as _BaseCube8PoseNode

_FRONTEND_TWO_FACE = "two_face_geometry"


class Cube8PoseNode(_BaseCube8PoseNode):
    def _process_two_face(self, frame, image_msg, bbox, base):
        item, pose = super()._process_two_face(frame, image_msg, bbox, base)
        if (
            pose is not None
            and item.get("pnp", {}).get("color_fixed_semantics_resolved") is not True
        ):
            item["grasp"] = {
                "status": "withheld_unresolved_cube_semantics",
                "reason": "color-fixed cube orientation/chirality is unresolved",
                "pose_semantic_frame": item.get("pnp", {}).get("semantic_frame", ""),
                "requires_color_fixed_semantics": True,
                "execution_authorized": False,
                "candidates": [],
            }
            # The base _process only generates grasp candidates when pose is not None.
            # Preserve the accepted PnP payload in item, but suppress the grasp input.
            return item, None
        return item, pose


def main(args=None):
    rclpy.init(args=args)
    node = Cube8PoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
