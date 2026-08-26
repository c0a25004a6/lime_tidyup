"""Safe view-local Cube8 PnP adapters with explicit observation provenance.

The view-local cube frame treats the three currently visible axis families as +X/+Y/+Z.
Local vertex 0 is always the fully hidden opposite corner. Some front-ends may also
derive local vertex 1 projectively; derived points must remain outside the PnP solve.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from .cube8_pnp import CameraModel, Cube8PnpError, CubePose3D
from .cube8_pnp_assist import estimate_pose_from_observed_vertices

VIEW_LOCAL_POSE_FRAME = "view_local_cube_symmetry"

@dataclass(frozen=True)
class ViewLocalCubePose:
    pose: CubePose3D
    semantic_frame: str = VIEW_LOCAL_POSE_FRAME
    color_fixed_semantics_resolved: bool = False
    hidden_vertex_used_as_pnp_observation: bool = False
    pnp_observation_indices: tuple[int, ...] = (1,2,3,4,5,6,7)
    derived_vertex_indices: tuple[int, ...] = ()

def indexed_observations_from_view_local_points(
    points: Sequence[Sequence[float] | None],
) -> dict[int, tuple[float, float]]:
    """Convert 8 local slots into the legacy seven-observation view-local contract."""
    if len(points) != 8:
        raise Cube8PnpError("view-local points must contain exactly 8 slots")
    if points[0] is not None:
        raise Cube8PnpError("view-local hidden vertex 0 must remain unobserved")
    observations={}
    for index in range(1,8):
        raw=points[index]
        if raw is None or len(raw) < 2:
            raise Cube8PnpError(f"view-local visible vertex {index} is missing")
        x,y=float(raw[0]),float(raw[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise Cube8PnpError(f"view-local visible vertex {index} is non-finite")
        observations[index]=(x,y)
    return observations

def estimate_view_local_pose_from_observations(
    observations: Mapping[int,Sequence[float]],
    camera: CameraModel,
    *,
    derived_vertex_indices: Sequence[int] = (),
    side_length_m: float = 0.057,
    maximum_reprojection_error_px: float = 4.0,
) -> ViewLocalCubePose:
    """Solve view-local PnP using only explicitly observed local vertices.

    Vertex 0 is always forbidden. Any additional projectively-derived vertices must
    be listed in ``derived_vertex_indices`` and are also forbidden from observations.
    """
    normalized={}
    for index,raw in observations.items():
        if isinstance(index,bool) or not isinstance(index,int) or not 1 <= index < 8:
            raise Cube8PnpError("view-local PnP observations must use integer indices 1..7")
        if len(raw) < 2:
            raise Cube8PnpError(f"view-local observation {index} must contain x,y")
        x,y=float(raw[0]),float(raw[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise Cube8PnpError(f"view-local observation {index} is non-finite")
        normalized[index]=(x,y)
    raw_derived=tuple(derived_vertex_indices)
    if any(isinstance(index,bool) or not isinstance(index,int) or not 1 <= index < 8 for index in raw_derived):
        raise Cube8PnpError("derived view-local indices must be integers in 1..7")
    derived=tuple(sorted(set(raw_derived)))
    overlap=sorted(set(normalized)&set(derived))
    if overlap:
        raise Cube8PnpError(f"derived vertices cannot enter PnP observations: {overlap}")
    pose=estimate_pose_from_observed_vertices(
        normalized,camera,
        side_length_m=side_length_m,
        maximum_reprojection_error_px=maximum_reprojection_error_px,
    )
    if pose.correspondence_count != len(normalized):
        raise Cube8PnpError(
            f"view-local PnP correspondence mismatch: expected {len(normalized)}, got {pose.correspondence_count}"
        )
    if len(pose.inlier_mask) != 8 or pose.inlier_mask[0] != 0:
        raise Cube8PnpError("view-local hidden vertex 0 unexpectedly entered the PnP inlier set")
    for index in derived:
        if pose.inlier_mask[index] != 0:
            raise Cube8PnpError(f"derived view-local vertex {index} unexpectedly entered PnP")
    return ViewLocalCubePose(
        pose=pose,
        pnp_observation_indices=tuple(sorted(normalized)),
        derived_vertex_indices=derived,
    )

def estimate_view_local_pose(
    points: Sequence[Sequence[float] | None],
    camera: CameraModel,
    *,
    side_length_m: float = 0.057,
    maximum_reprojection_error_px: float = 4.0,
) -> ViewLocalCubePose:
    """Backward-compatible seven-observation view-local PnP."""
    observations=indexed_observations_from_view_local_points(points)
    result=estimate_view_local_pose_from_observations(
        observations,camera,
        side_length_m=side_length_m,
        maximum_reprojection_error_px=maximum_reprojection_error_px,
    )
    if result.pose.correspondence_count != 7:
        raise Cube8PnpError(
            f"view-local PnP must use exactly 7 correspondences, got {result.pose.correspondence_count}"
        )
    return result
