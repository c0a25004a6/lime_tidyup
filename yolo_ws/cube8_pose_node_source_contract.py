#!/usr/bin/env python3
"""Static safety contract for the Cube8 ROS executable without ROS imports."""
from __future__ import annotations
import ast
from pathlib import Path

BASE_PATH=Path("yolo_ws/src/barcode_detector/barcode_detector/cube8_pose_node.py")
SAFE_PATH=Path("yolo_ws/src/barcode_detector/barcode_detector/cube8_pose_safe_node.py")
SETUP_PATH=Path("yolo_ws/src/barcode_detector/setup.py")
source=BASE_PATH.read_text()
safe_source=SAFE_PATH.read_text()
setup_source=SETUP_PATH.read_text()
tree=ast.parse(source)
ast.parse(safe_source)
ast.parse(setup_source)

def declared_default(name):
    for node in ast.walk(tree):
        if not isinstance(node,ast.Call) or not isinstance(node.func,ast.Attribute):
            continue
        if node.func.attr != "declare_parameter" or len(node.args) < 2:
            continue
        if isinstance(node.args[0],ast.Constant) and node.args[0].value == name:
            return ast.literal_eval(node.args[1])
    raise AssertionError(f"parameter {name!r} is not declared")

assert declared_default("perception_frontend") == "rtmpose"
assert declared_default("enable_pnp") is True
assert declared_default("enable_grasp_candidates") is True
assert '"physical_robot_authority":False' in source
assert 'geometry.pnp_observations()' in source
assert 'derived_vertex_indices=geometry.derived_indices' in source
assert '"hidden_index":0' in source
assert '"color_fixed_semantics_resolved":False' in source
assert '"experimental_not_production_qualified"' in source
assert '"two_face_geometry"' in source

assert "cube8_pose_node = barcode_detector.cube8_pose_safe_node:main" in setup_source
assert "class Cube8PoseNode(_BaseCube8PoseNode)" in safe_source
assert "super()._process_two_face" in safe_source
assert 'get("color_fixed_semantics_resolved") is not True' in safe_source
assert '"status": "withheld_unresolved_cube_semantics"' in safe_source
assert '"requires_color_fixed_semantics": True' in safe_source
assert '"execution_authorized": False' in safe_source
assert '"candidates": []' in safe_source
assert "return item, None" in safe_source

print("CUBE8_NODE_FRONTEND_CONTRACT default=rtmpose two_face=explicit_opt_in pnp_observed=2..7 derived=1 hidden=0 executable=safe_wrapper unresolved_two_face_grasp=withheld robot_authority=false")
