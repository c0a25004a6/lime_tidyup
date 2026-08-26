"""Sparse RGB-D cross-checks for Cube8 pose without full point-cloud processing.

Depth pixels must already be aligned to the color image used by the supplied camera
intrinsics. This module samples only tiny patches around PnP-predicted visible face
centers. It does not run plane fitting, point-cloud construction, or authorize robot
motion.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import Sequence


class CubeDepthAssistError(ValueError):
    pass


@dataclass(frozen=True)
class FaceDepthPrediction:
    face_id: str
    pixel_x: float
    pixel_y: float
    predicted_depth_m: float


@dataclass(frozen=True)
class FaceDepthObservation:
    face_id: str
    pixel_x: float
    pixel_y: float
    predicted_depth_m: float
    observed_depth_m: float
    support: int
    mad_m: float

    @property
    def disagreement_m(self) -> float:
        return self.observed_depth_m - self.predicted_depth_m

    @property
    def abs_disagreement_m(self) -> float:
        return abs(self.disagreement_m)


@dataclass(frozen=True)
class SparseDepthCheck:
    observations: tuple[FaceDepthObservation, ...]
    maximum_abs_disagreement_m: float
    median_abs_disagreement_m: float
    passed: bool
    failed_faces: tuple[str, ...]
    authorizes_robot_motion: bool = False


_FACES = (
    ("x_neg", (-1.0, 0.0, 0.0)),
    ("x_pos", (+1.0, 0.0, 0.0)),
    ("y_neg", (0.0, -1.0, 0.0)),
    ("y_pos", (0.0, +1.0, 0.0)),
    ("z_neg", (0.0, 0.0, -1.0)),
    ("z_pos", (0.0, 0.0, +1.0)),
)


def predict_visible_face_depths(
    *,
    rotation_vector: Sequence[float],
    translation_m: Sequence[float],
    side_length_m: float,
    camera_k: Sequence[float],
    camera_d: Sequence[float] = (),
    facing_epsilon: float = 1e-9,
) -> tuple[FaceDepthPrediction, ...]:
    import cv2
    import numpy as np

    rvec = _vec3(rotation_vector, "rotation_vector")
    tvec = _vec3(translation_m, "translation_m")
    side = float(side_length_m)
    if not math.isfinite(side) or side <= 0.0:
        raise CubeDepthAssistError("side_length_m must be finite and positive")
    if len(camera_k) != 9 or not all(math.isfinite(float(v)) for v in camera_k):
        raise CubeDepthAssistError("camera_k must contain 9 finite values")
    if float(camera_k[0]) <= 0.0 or float(camera_k[4]) <= 0.0:
        raise CubeDepthAssistError("camera focal lengths must be positive")
    if not all(math.isfinite(float(v)) for v in camera_d):
        raise CubeDepthAssistError("camera distortion must be finite")
    epsilon = float(facing_epsilon)
    if not math.isfinite(epsilon) or epsilon < 0.0:
        raise CubeDepthAssistError("facing_epsilon must be finite and non-negative")

    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    K = np.asarray(camera_k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(tuple(float(v) for v in camera_d), dtype=np.float64)
    half = side / 2.0

    predictions: list[FaceDepthPrediction] = []
    for face_id, normal_raw in _FACES:
        normal_obj = np.asarray(normal_raw, dtype=np.float64)
        center_obj = normal_obj * half
        center_cam = R @ center_obj + t
        normal_cam = R @ normal_obj
        if center_cam[2] <= 0.0:
            continue
        if float(normal_cam @ center_cam) >= -epsilon:
            continue
        projected, _ = cv2.projectPoints(
            center_obj.reshape(1, 3),
            np.asarray(rvec, dtype=np.float64).reshape(3, 1),
            np.asarray(tvec, dtype=np.float64).reshape(3, 1),
            K,
            d,
        )
        u, v = (float(value) for value in projected.reshape(2))
        if not all(math.isfinite(value) for value in (u, v, float(center_cam[2]))):
            raise CubeDepthAssistError("face projection produced non-finite values")
        predictions.append(FaceDepthPrediction(face_id, u, v, float(center_cam[2])))
    return tuple(predictions)


def sample_aligned_depth_patches(
    depth_image,
    predictions: Sequence[FaceDepthPrediction],
    *,
    depth_scale: float,
    radius_px: int = 1,
    minimum_support_per_face: int = 3,
    minimum_depth_m: float = 0.05,
    maximum_depth_m: float = 8.0,
) -> tuple[FaceDepthObservation, ...]:
    scale = float(depth_scale)
    if not math.isfinite(scale) or scale <= 0.0:
        raise CubeDepthAssistError("depth_scale must be finite and positive")
    if isinstance(radius_px, bool) or int(radius_px) != radius_px or radius_px < 0:
        raise CubeDepthAssistError("radius_px must be a non-negative integer")
    if minimum_support_per_face <= 0:
        raise CubeDepthAssistError("minimum_support_per_face must be positive")
    min_depth = float(minimum_depth_m)
    max_depth = float(maximum_depth_m)
    if not (math.isfinite(min_depth) and math.isfinite(max_depth) and 0.0 < min_depth < max_depth):
        raise CubeDepthAssistError("invalid depth range")

    try:
        height = len(depth_image)
        width = len(depth_image[0]) if height else 0
    except Exception as exc:
        raise CubeDepthAssistError("depth_image must be a 2D indexable image") from exc
    if height <= 0 or width <= 0:
        return ()

    observations: list[FaceDepthObservation] = []
    for prediction in predictions:
        cx = int(round(float(prediction.pixel_x)))
        cy = int(round(float(prediction.pixel_y)))
        values: list[float] = []
        for y in range(max(0, cy - radius_px), min(height - 1, cy + radius_px) + 1):
            row = depth_image[y]
            for x in range(max(0, cx - radius_px), min(width - 1, cx + radius_px) + 1):
                try:
                    raw = float(row[x])
                except (TypeError, ValueError):
                    continue
                depth_m = raw * scale
                if math.isfinite(depth_m) and min_depth <= depth_m <= max_depth:
                    values.append(depth_m)
        if len(values) < int(minimum_support_per_face):
            continue
        observed = float(median(values))
        mad = float(median(abs(value - observed) for value in values))
        observations.append(
            FaceDepthObservation(
                prediction.face_id,
                float(prediction.pixel_x),
                float(prediction.pixel_y),
                float(prediction.predicted_depth_m),
                observed,
                len(values),
                mad,
            )
        )
    return tuple(observations)


def check_sparse_depth_consistency(
    observations: Sequence[FaceDepthObservation],
    *,
    maximum_abs_disagreement_m: float,
    minimum_face_count: int = 1,
) -> SparseDepthCheck:
    limit = float(maximum_abs_disagreement_m)
    if not math.isfinite(limit) or limit <= 0.0:
        raise CubeDepthAssistError("maximum_abs_disagreement_m must be finite and positive")
    if minimum_face_count <= 0:
        raise CubeDepthAssistError("minimum_face_count must be positive")
    obs = tuple(observations)
    if len(obs) < int(minimum_face_count):
        return SparseDepthCheck(obs, limit, math.inf, False, ("insufficient_face_support",), False)
    errors = [item.abs_disagreement_m for item in obs]
    failed_faces = tuple(item.face_id for item in obs if item.abs_disagreement_m > limit)
    return SparseDepthCheck(obs, limit, float(median(errors)), not failed_faces, failed_faces, False)


def _vec3(values: Sequence[float], name: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise CubeDepthAssistError(f"{name} must contain 3 values")
    result = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise CubeDepthAssistError(f"{name} must be finite")
    return result  # type: ignore[return-value]
