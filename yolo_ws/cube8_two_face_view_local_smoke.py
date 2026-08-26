#!/usr/bin/env python3
"""Synthetic contract for two-face view-local geometry and six-observation PnP."""
from __future__ import annotations
import math

import cv2
import numpy as np

from barcode_detector.cube8_geometry import cube_vertices
from barcode_detector.cube8_pnp import CameraModel, Cube8PnpError
from barcode_detector.cube8_pnp_assist import project_vertex
from barcode_detector.cube8_two_face_view_local import (
    Cube8TwoFaceGeometryError,
    recover_view_local_from_yz_quads,
)
from barcode_detector.cube8_view_local_pnp import estimate_view_local_pose_from_observations

def variants(quad):
    items=list(quad)
    for shift in range(4):
        rotated=items[shift:]+items[:shift]
        yield rotated
        yield list(reversed(rotated))

def distance(a,b):
    return math.hypot(float(a[0])-float(b[0]),float(a[1])-float(b[1]))

def main() -> int:
    side=0.057
    vertices=cube_vertices(side)
    obj=np.asarray([(v.x,v.y,v.z) for v in vertices],dtype=np.float64)
    K=np.asarray([[620.0,0.0,320.0],[0.0,615.0,240.0],[0.0,0.0,1.0]],dtype=np.float64)
    rvec=np.asarray([0.23,-0.31,0.14],dtype=np.float64).reshape(3,1)
    tvec=np.asarray([0.03,-0.015,0.78],dtype=np.float64).reshape(3,1)
    projected,_=cv2.projectPoints(obj,rvec,tvec,K,np.zeros(5,dtype=np.float64))
    truth=projected.reshape(-1,2)
    bbox=(
        float(np.min(truth[:,0])-10.0),float(np.min(truth[:,1])-10.0),
        float(np.max(truth[:,0])+10.0),float(np.max(truth[:,1])+10.0),
    )
    y_quad=[truth[i] for i in (2,3,7,6)]
    z_quad=[truth[i] for i in (4,5,6,7)]

    max_variant_error=0.0
    for y_variant in variants(y_quad):
        for z_variant in variants(z_quad):
            geometry=recover_view_local_from_yz_quads(y_variant,z_variant,bbox)
            assert geometry.gate_passed,geometry.gate_failures
            error=max(distance(geometry.points[i],truth[i]) for i in range(1,8))
            max_variant_error=max(max_variant_error,error)
    assert max_variant_error < 1e-6,max_variant_error

    geometry=recover_view_local_from_yz_quads(y_quad,z_quad,bbox)
    assert geometry.points[0] is None
    assert geometry.point_sources[0]=="hidden_unobserved"
    assert geometry.point_sources[1]=="derived_projective_from_two_faces"
    assert geometry.observed_indices==(2,3,4,5,6,7)
    assert geometry.derived_indices==(1,)
    observations=geometry.pnp_observations()
    assert tuple(sorted(observations))==(2,3,4,5,6,7)
    assert 0 not in observations and 1 not in observations

    camera=CameraModel(tuple(float(v) for v in K.reshape(-1)),())
    result=estimate_view_local_pose_from_observations(
        observations,camera,derived_vertex_indices=geometry.derived_indices,side_length_m=side
    )
    pose=result.pose
    assert pose.correspondence_count==6
    assert pose.inlier_count==6
    assert pose.inlier_mask[0]==0 and pose.inlier_mask[1]==0
    assert result.pnp_observation_indices==(2,3,4,5,6,7)
    assert result.derived_vertex_indices==(1,)
    translation_error=math.sqrt(sum((pose.translation_m[i]-float(tvec[i,0]))**2 for i in range(3)))
    assert translation_error < 1e-5,translation_error
    p1=project_vertex(pose,1,camera)
    p0=project_vertex(pose,0,camera)
    derived_a_error=distance(p1,truth[1])
    hidden_error=distance(p0,truth[0])
    assert derived_a_error < 1e-3,derived_a_error
    assert hidden_error < 1e-3,hidden_error

    shifted=[(float(point[0]+30.0),float(point[1]+30.0)) for point in z_quad]
    rejected=recover_view_local_from_yz_quads(y_quad,shifted,bbox)
    assert not rejected.gate_passed
    try:
        rejected.pnp_observations()
        raise AssertionError("gate-failed geometry exposed PnP observations")
    except Cube8TwoFaceGeometryError:
        pass
    try:
        estimate_view_local_pose_from_observations(
            {1:tuple(truth[1]),**observations},camera,
            derived_vertex_indices=(1,),side_length_m=side,
        )
        raise AssertionError("derived vertex 1 entered PnP")
    except Cube8PnpError:
        pass

    print(
        "CUBE8_TWO_FACE_VIEW_LOCAL",
        f"variant_max_px={max_variant_error:.9g}",
        f"correspondences={pose.correspondence_count}",
        f"inliers={pose.inlier_count}",
        f"reproj_px={pose.reprojection_error_px:.9g}",
        f"translation_error_m={translation_error:.9g}",
        f"derived_a_reprojection_px={derived_a_error:.9g}",
        f"hidden_reprojection_px={hidden_error:.9g}",
    )
    print(
        "CUBE8_TWO_FACE_VIEW_LOCAL",
        "observed=2,3,4,5,6,7","derived=1","hidden=0",
        "color_semantics_required=false","robot_authority=false",
    )
    return 0

if __name__=="__main__":
    raise SystemExit(main())
