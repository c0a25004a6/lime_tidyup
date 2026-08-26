#!/usr/bin/env python3
"""Run the two-face color frontend on the exact dev2 sample and emit evidence artifacts."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2

from barcode_detector.cube8_two_face_color_frontend import recover_two_face_view_local_from_bgr

EXPECTED_SAMPLE_SHA256="48eb4331e729900ac82c630e582ca54deff6080d5ad13b11628290495e451841"
DETECTOR_BBOX=(396.77117919921875,137.01947021484375,914.8598022460938,616.1307373046875)
DETECTOR_MODEL_SHA256="9a888f153380a0cf9ed51fdfb311ec48f5402bb13ad7009fa9b8ee2d9f182ea9"
DETECTOR_SOURCE_COMMIT="99d5d5be6b5253752d0b36137199286f9c78e277"
REFERENCE_COLOR_LINE={
    1:(445.4269579038999,430.8904695393965),
    2:(655.8452200975654,608.2459519265391),
    3:(878.9271163422362,440.2784065187869),
    4:(665.2172970629673,54.436436544922515),
    5:(409.2040641360062,171.12057423250215),
    6:(654.2455376369813,335.83724936593774),
    7:(915.0199115044248,178.60564159292034),
}

def sha256(path:Path)->str:
    h=hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda:handle.read(1024*1024),b""):
            h.update(chunk)
    return h.hexdigest()

def distance(a,b):
    return math.hypot(float(a[0])-float(b[0]),float(a[1])-float(b[1]))

def main()->int:
    parser=argparse.ArgumentParser()
    parser.add_argument("--image",default="yolo_ws/sample.png")
    parser.add_argument("--json",default="artifacts/cube8-two-face-exact/sample-two-face.json")
    parser.add_argument("--debug-image",default="artifacts/cube8-two-face-exact/sample-two-face-debug.jpg")
    args=parser.parse_args()
    image_path=Path(args.image)
    actual_sha=sha256(image_path)
    if actual_sha != EXPECTED_SAMPLE_SHA256:
        raise SystemExit(f"sample SHA mismatch: {actual_sha}")
    frame=cv2.imread(str(image_path),cv2.IMREAD_COLOR)
    if frame is None:
        raise SystemExit("cannot decode exact sample")
    observation=recover_two_face_view_local_from_bgr(frame,DETECTOR_BBOX)
    geometry=observation.geometry
    if not geometry.gate_passed:
        raise SystemExit("exact sample two-face geometry gate failed: "+",".join(geometry.gate_failures))
    if observation.visible_y_color != "blue" or observation.visible_z_color != "white":
        raise SystemExit(
            f"unexpected exact-sample Y/Z colors: {observation.visible_y_color}/{observation.visible_z_color}"
        )
    points={index:geometry.points[index] for index in range(1,8)}
    errors={index:distance(points[index],REFERENCE_COLOR_LINE[index]) for index in range(1,8)}
    observed_errors=[errors[index] for index in geometry.observed_indices]
    derived_error=errors[1]

    payload={
        "schema":"cube8-two-face-exact-sample-v1",
        "sample_sha256":actual_sha,
        "detector_bbox_xyxy":list(DETECTOR_BBOX),
        "detector_bbox_provenance":{
            "source":"legacy_actual_yolo_detection",
            "source_commit":DETECTOR_SOURCE_COMMIT,
            "model_sha256":DETECTOR_MODEL_SHA256,
            "detector_not_rerun_in_this_diagnostic":True,
        },
        "selected_visible_colors":{
            "y":observation.visible_y_color,
            "z":observation.visible_z_color,
            "x":"unresolved_not_required",
        },
        "seed_support_pixels":dict(observation.support_pixels),
        "search_box_xyxy":list(observation.search_box_xyxy),
        "face_quads":{
            color:{
                "vertices":[list(point) for point in quad.vertices],
                "component_area_px":quad.component_area_px,
                "mask_support_px":quad.mask_support_px,
            }
            for color,quad in observation.face_quads.items()
        },
        "view_local_geometry":{
            "semantic_frame":geometry.semantic_frame,
            "color_fixed_semantics_resolved":geometry.color_fixed_semantics_resolved,
            "points":[None if point is None else list(point) for point in geometry.points],
            "point_sources":list(geometry.point_sources),
            "observed_indices":list(geometry.observed_indices),
            "derived_indices":list(geometry.derived_indices),
            "hidden_index":0,
            "vanishing_x":list(geometry.vanishing_x),
            "vanishing_y":list(geometry.vanishing_y),
            "vanishing_z":list(geometry.vanishing_z),
            "shared_edge_score_over_bbox_diag":geometry.shared_edge_score_over_bbox_diag,
            "gate_passed":geometry.gate_passed,
            "gate_failures":list(geometry.gate_failures),
        },
        "reference_color_line_comparison":{
            "reference_is_ground_truth":False,
            "reference_description":"previous exact-sample color-line/projective diagnostic",
            "per_vertex_error_px":{str(index):errors[index] for index in range(1,8)},
            "observed_2_to_7_mean_error_px":sum(observed_errors)/len(observed_errors),
            "observed_2_to_7_max_error_px":max(observed_errors),
            "derived_1_error_px":derived_error,
        },
        "claim_boundary":{
            "pnp_run":False,
            "camera_info_used":False,
            "hidden_vertex_observed":False,
            "derived_vertex_used_as_independent_observation":False,
            "physical_robot_authority":False,
            "production_qualified":False,
        },
    }
    json_path=Path(args.json); json_path.parent.mkdir(parents=True,exist_ok=True)
    json_path.write_text(json.dumps(payload,indent=2,sort_keys=True)+"\n")

    debug=frame.copy()
    x1,y1,x2,y2=(int(round(v)) for v in DETECTOR_BBOX)
    cv2.rectangle(debug,(x1,y1),(x2,y2),(255,0,255),2)
    for color,quad in observation.face_quads.items():
        poly=[(int(round(p[0])),int(round(p[1]))) for p in quad.vertices]
        line_color=(255,255,0) if color in ("blue","green") else (0,255,255)
        for i in range(4):
            cv2.line(debug,poly[i],poly[(i+1)%4],line_color,2,cv2.LINE_AA)
        cx=int(round(sum(p[0] for p in poly)/4)); cy=int(round(sum(p[1] for p in poly)/4))
        cv2.putText(debug,f"{color} face",(cx-35,cy),cv2.FONT_HERSHEY_SIMPLEX,0.5,line_color,2,cv2.LINE_AA)
    for index in geometry.observed_indices:
        p=geometry.points[index]
        pt=(int(round(p[0])),int(round(p[1])))
        cv2.circle(debug,pt,6,(0,255,0),-1,cv2.LINE_AA)
        cv2.putText(debug,f"O{index}",(pt[0]+7,pt[1]-7),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,255,0),2,cv2.LINE_AA)
    p=geometry.points[1]
    pt=(int(round(p[0])),int(round(p[1])))
    cv2.circle(debug,pt,7,(0,165,255),2,cv2.LINE_AA)
    cv2.putText(debug,"D1 projective",(pt[0]+8,pt[1]-8),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,165,255),2,cv2.LINE_AA)
    cv2.putText(debug,"O2..O7 observed | D1 derived | 0 hidden/not drawn | PnP OFF",
                (25,35),cv2.FONT_HERSHEY_SIMPLEX,0.62,(0,0,0),3,cv2.LINE_AA)
    cv2.putText(debug,"O2..O7 observed | D1 derived | 0 hidden/not drawn | PnP OFF",
                (25,35),cv2.FONT_HERSHEY_SIMPLEX,0.62,(255,255,255),1,cv2.LINE_AA)
    debug_path=Path(args.debug_image); debug_path.parent.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(str(debug_path),debug):
        raise SystemExit("failed to write debug image")

    print(
        "CUBE8_TWO_FACE_EXACT",
        f"colors={observation.visible_y_color}/{observation.visible_z_color}",
        f"gate={geometry.gate_passed}",
        f"shared={geometry.shared_edge_score_over_bbox_diag:.9g}",
        f"observed_ref_mean_px={sum(observed_errors)/len(observed_errors):.9g}",
        f"observed_ref_max_px={max(observed_errors):.9g}",
        f"derived1_ref_px={derived_error:.9g}",
    )
    print(
        "CUBE8_TWO_FACE_EXACT",
        "observed=2,3,4,5,6,7","derived=1","hidden=0",
        "pnp=false","camera_info=false","robot_authority=false","production_qualified=false",
    )
    return 0

if __name__=="__main__":
    raise SystemExit(main())
