"""Safe view-local seven-point Cube8 PnP adapter.

The view-local cube frame treats the three currently visible axis families as +X/+Y/+Z.
Local vertex 0 is therefore the fully hidden opposite corner.  It must remain absent
from PnP observations; any projective hidden estimate is auxiliary evidence only.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence

from .cube8_pnp import CameraModel, Cube8PnpError, CubePose3D
from .cube8_pnp_assist import estimate_pose_from_observed_vertices

VIEW_LOCAL_POSE_FRAME = "view_local_cube_symmetry"

@dataclass(frozen=True)
class ViewLocalCubePose:
    pose: CubePose3D
    semantic_frame: str = VIEW_LOCAL_POSE_FRAME
    color_fixed_semantics_resolved: bool = False
    hidden_vertex_used_as_pnp_observation: bool = False


def indexed_observations_from_view_local_points(
    points: Sequence[Sequence[float] | None],
) -> dict[int, tuple[float, float]]:
    """Convert 8 local slots into exactly seven observed PnP correspondences."""
    if len(points) != 8:
        raise Cube8PnpError("view-local points must contain exactly 8 slots")
    if points[0] is not None:
        raise Cube8PnpError("view-local hidden vertex 0 must remain unobserved")
    observations: dict[int, tuple[float, float]] = {}
    for index in range(1, 8):
        raw = points[index]
        if raw is None or len(raw) < 2:
            raise Cube8PnpError(f"view-local visible vertex {index} is missing")
        x, y = float(raw[0]), float(raw[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise Cube8PnpError(f"view-local visible vertex {index} is non-finite")
        observations[index] = (x, y)
    return observations


def estimate_view_local_pose(
    points: Sequence[Sequence[float] | None],
    camera: CameraModel,
    *,
    side_length_m: float = 0.057,
    maximum_reprojection_error_px: float = 4.0,
) -> ViewLocalCubePose:
    observations = indexed_observations_from_view_local_points(points)
    pose = estimate_pose_from_observed_vertices(
        observations,
        camera,
        side_length_m=side_length_m,
        maximum_reprojection_error_px=maximum_reprojection_error_px,
    )
    if pose.correspondence_count != 7:
        raise Cube8PnpError(
            f"view-local PnP must use exactly 7 correspondences, got {pose.correspondence_count}"
        )
    if len(pose.inlier_mask) != 8 or pose.inlier_mask[0] != 0:
        raise Cube8PnpError("view-local hidden vertex 0 unexpectedly entered the PnP inlier set")
    return ViewLocalCubePose(pose)
