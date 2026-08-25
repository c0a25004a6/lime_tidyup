#!/usr/bin/env python3
"""Synthetic fail-closed smoke for Cube8 PnP and geometric grasp output."""
import cv2
import numpy as np
from barcode_detector.cube8_geometry import cube_vertices, ordered_keypoints
from barcode_detector.cube8_pnp import CameraModel, estimate_cube_pose
from barcode_detector.cube8_grasp import rank_cube_grasp_candidates

side=0.057
K=np.array([[600.,0.,320.],[0.,600.,240.],[0.,0.,1.]],dtype=np.float64)
obj=np.array([(v.x,v.y,v.z) for v in cube_vertices(side)],dtype=np.float64)
rvec=np.array([[0.12],[-0.08],[0.18]],dtype=np.float64)
tvec=np.array([[0.04],[-0.02],[0.80]],dtype=np.float64)
image,_=cv2.projectPoints(obj,rvec,tvec,K,np.zeros(5))
keypoints=ordered_keypoints(image.reshape(-1,2).tolist(),[1.0]*8)
pose=estimate_cube_pose(
    keypoints,
    CameraModel(tuple(K.reshape(-1)),(0.,0.,0.,0.,0.)),
    side_length_m=side,
)
assert np.linalg.norm(np.array(pose.translation_m)-tvec.reshape(-1)) < 0.01, pose
grasps=rank_cube_grasp_candidates(
    translation_m=pose.translation_m,
    rotation_xyzw=pose.orientation_xyzw,
    side_length_m=side,
    table_z_m=-10.0,
)
assert grasps
assert grasps[0].as_dict()["execution_authorized"] is False
assert len(grasps[0].grasp_frame_xyzw) == 4
assert len(grasps[0].grasp_frame_rpy_rad) == 3
print("synthetic Cube8 PnP/grasp smoke PASS")
print(pose.as_dict())
print(grasps[0].as_dict())
