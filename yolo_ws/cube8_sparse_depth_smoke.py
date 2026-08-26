#!/usr/bin/env python3
"""Smoke sparse RealSense-style aligned depth checks without camera or robot IO."""
import numpy as np

from barcode_detector.cube8_depth_assist import (
    check_sparse_depth_consistency,
    predict_visible_face_depths,
    sample_aligned_depth_patches,
)


def main() -> int:
    camera_k=(600.0,0.0,320.0,0.0,600.0,240.0,0.0,0.0,1.0)
    predictions=predict_visible_face_depths(
        rotation_vector=(0.20,-0.25,0.10),
        translation_m=(0.02,-0.01,0.80),
        side_length_m=0.057,
        camera_k=camera_k,
    )
    assert 1 <= len(predictions) <= 3

    depth=np.zeros((480,640),dtype=np.uint16)
    scale=0.001
    for prediction in predictions:
        cx=int(round(prediction.pixel_x)); cy=int(round(prediction.pixel_y))
        raw=int(round(prediction.predicted_depth_m/scale))
        depth[max(0,cy-1):min(480,cy+2),max(0,cx-1):min(640,cx+2)]=raw
    observations=sample_aligned_depth_patches(depth,predictions,depth_scale=scale)
    check=check_sparse_depth_consistency(observations,maximum_abs_disagreement_m=0.003)
    assert check.passed and not check.authorizes_robot_motion

    bad=depth.copy()
    prediction=predictions[0]
    cx=int(round(prediction.pixel_x)); cy=int(round(prediction.pixel_y))
    bad[max(0,cy-1):min(480,cy+2),max(0,cx-1):min(640,cx+2)]=int(round((prediction.predicted_depth_m+0.05)/scale))
    bad_observations=sample_aligned_depth_patches(bad,predictions,depth_scale=scale)
    bad_check=check_sparse_depth_consistency(bad_observations,maximum_abs_disagreement_m=0.01)
    assert not bad_check.passed and prediction.face_id in bad_check.failed_faces

    print(f"CUBE8_SPARSE_DEPTH faces={len(predictions)} support={sum(o.support for o in observations)} passed={check.passed}")
    print("CUBE8_SPARSE_DEPTH full_point_cloud=false plane_fit=false robot_authority=false")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
