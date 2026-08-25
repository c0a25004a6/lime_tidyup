#!/usr/bin/env python3
"""Smoke projective hidden-corner derivation plus independent seven-point PnP."""
from __future__ import annotations
import math
import cv2
import numpy as np
from barcode_detector.cube8_geometry import cube_vertices
from barcode_detector.cube8_pnp import CameraModel
from barcode_detector.cube8_pnp_assist import estimate_pose_from_observed_vertices, project_vertex
from barcode_detector.cube8_projective_hidden import derive_hidden_vertex

def main():
    observed={
        1:(445.4269579038999,430.8904695393965),
        2:(655.8452200975654,608.2459519265391),
        3:(878.9271163422362,440.2784065187869),
        4:(665.2172970629673,54.436436544922515),
        5:(409.2040641360062,171.12057423250215),
        6:(654.2455376369813,335.83724936593774),
        7:(915.0199115044248,178.60564159292034),
    }
    hidden=derive_hidden_vertex(
        observed,hidden_index=0,
        vanishing_x=(2553.05167128,-816.24000403),
        vanishing_y=(-933.94942234,-731.74608472),
        vanishing_z=(664.00891530,1998.43536404),
    )
    assert abs(hidden.x-664.75012113)<1e-4 and abs(hidden.y-301.45397680)<1e-4
    assert hidden.axis_line_rms_error_px < 0.304
    assert hidden.pairwise_intersection_spread_px < 1.10

    camera=CameraModel((600.0,0.0,320.0,0.0,600.0,240.0,0.0,0.0,1.0),())
    side=0.057
    rvec=np.asarray([0.17,-0.24,0.31],dtype=np.float64).reshape(3,1)
    tvec=np.asarray([0.04,-0.02,0.80],dtype=np.float64).reshape(3,1)
    vertices=np.asarray([(v.x,v.y,v.z) for v in cube_vertices(side)],dtype=np.float64)
    projected,_=cv2.projectPoints(vertices,rvec,tvec,np.asarray(camera.k).reshape(3,3),np.zeros(0))
    projected=projected.reshape(8,2)
    observations={i:tuple(float(v) for v in projected[i]) for i in range(1,8)}
    pose=estimate_pose_from_observed_vertices(observations,camera,side_length_m=side)
    assert pose.correspondence_count==7 and pose.inlier_mask[0]==0
    predicted=project_vertex(pose,0,camera)
    error=math.hypot(predicted[0]-projected[0,0],predicted[1]-projected[0,1])
    assert error < 0.01, error
    print(f"CUBE8_DEV2_HIDDEN x={hidden.x:.6f} y={hidden.y:.6f} rms={hidden.axis_line_rms_error_px:.6f} spread={hidden.pairwise_intersection_spread_px:.6f}")
    print(f"CUBE8_DEV2_PNP correspondences={pose.correspondence_count} hidden_error_px={error:.9f} reproj_px={pose.reprojection_error_px:.9f}")
    return 0

if __name__=="__main__":
    raise SystemExit(main())
