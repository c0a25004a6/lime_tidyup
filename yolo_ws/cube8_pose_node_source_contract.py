#!/usr/bin/env python3
"""Static safety contract for the ROS Cube8 frontend switch without ROS imports."""
from __future__ import annotations
import ast
from pathlib import Path

PATH=Path("yolo_ws/src/barcode_detector/barcode_detector/cube8_pose_node.py")
source=PATH.read_text()
tree=ast.parse(source)

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
print("CUBE8_NODE_FRONTEND_CONTRACT default=rtmpose two_face=explicit_opt_in pnp_observed=2..7 derived=1 hidden=0 robot_authority=false")
