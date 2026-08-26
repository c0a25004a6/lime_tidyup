#!/usr/bin/env python3
"""Synthetic dev2 smoke for view-local Cube8 seven-point PnP."""
import math
import numpy as np
import cv2

from barcode_detector.cube8_geometry import cube_vertices
from barcode_detector.cube8_pnp import CameraModel
from barcode_detector.cube8_pnp_assist import project_vertex
from barcode_detector.cube8_view_local_pnp import estimate_view_local_pose


def main() -> int:
    side=0.057
    camera=CameraModel((620.0,0.0,320.0,0.0,615.0,240.0,0.0,0.0,1.0),())
    true_rvec=np.asarray((0.22,-0.17,0.11),dtype=np.float64).reshape(3,1)
    true_tvec=np.asarray((0.035,-0.018,0.78),dtype=np.float64).reshape(3,1)
    vertices=cube_vertices(side)
    obj=np.asarray([(v.x,v.y,v.z) for v in vertices],dtype=np.float64)
    projected,_=cv2.projectPoints(obj,true_rvec,true_tvec,np.asarray(camera.k,dtype=np.float64).reshape(3,3),np.asarray(camera.d,dtype=np.float64))
    image=projected.reshape(-1,2)
    points=[None]+[(float(image[i,0]),float(image[i,1])) for i in range(1,8)]
    result=estimate_view_local_pose(points,camera,side_length_m=side,maximum_reprojection_error_px=0.5)
    pose=result.pose
    translation_error=math.sqrt(sum((pose.translation_m[i]-float(true_tvec[i,0]))**2 for i in range(3)))
    hidden=project_vertex(pose,0,camera)
    hidden_error=math.hypot(hidden[0]-float(image[0,0]),hidden[1]-float(image[0,1]))
    assert pose.correspondence_count==7
    assert pose.inlier_count==7
    assert pose.inlier_mask[0]==0
    assert pose.reprojection_error_px < 1e-3
    assert translation_error < 1e-5
    assert hidden_error < 1e-3
    assert not result.hidden_vertex_used_as_pnp_observation
    assert not result.color_fixed_semantics_resolved
    print(f"CUBE8_VIEW_LOCAL_PNP correspondences={pose.correspondence_count} inliers={pose.inlier_count} reproj_px={pose.reprojection_error_px:.9g} translation_error_m={translation_error:.9g} hidden_reprojection_error_px={hidden_error:.9g}")
    print("CUBE8_VIEW_LOCAL_PNP synthetic_only=true color_semantics_required=false hidden_observed=false robot_authority=false")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
