#!/usr/bin/env python3
"""Offline sample visualization for the dev2 YOLO-bbox + Cube8 RTMPose pipeline."""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path

ROOT=Path(__file__).resolve().parent
SRC=ROOT/"src"/"barcode_detector"
sys.path.insert(0,str(SRC))
from barcode_detector.cube8_geometry import CUBE_EDGES
from barcode_detector.cube8_rtmpose import infer_rtmpose_mmpose_compatible, ordered_cube_keypoints_from_rtmpose

def parse_args():
    p=argparse.ArgumentParser()
    p.add_argument("--image",default="sample.png")
    p.add_argument("--detector",default="runs/pose/train-2/weights/best.pt")
    p.add_argument("--rtmpose",default="models/cube8-rtmpose-retained.onnx")
    p.add_argument("--output-image",default="sample_cube8_debug.jpg")
    p.add_argument("--output-json",default="sample_cube8_debug.json")
    p.add_argument("--conf",type=float,default=0.25)
    return p.parse_args()

def main():
    import cv2
    from ultralytics import YOLO
    from rtmlib.tools.pose_estimation.rtmpose import RTMPose
    args=parse_args()
    frame=cv2.imread(args.image,cv2.IMREAD_COLOR)
    if frame is None: raise SystemExit(f"cannot decode {args.image}")
    det=YOLO(args.detector).predict(source=frame,conf=args.conf,save=False,verbose=False)[0]
    if det.boxes is None or len(det.boxes)==0: raise SystemExit("no detector boxes")
    candidates=[]
    for i,(box,cls,conf) in enumerate(zip(det.boxes.xyxy.cpu().tolist(),det.boxes.cls.cpu().tolist(),det.boxes.conf.cpu().tolist())):
        name=str(det.names[int(cls)])
        if "cube" in name.lower(): candidates.append((float(conf),i,box,int(cls),name))
    if not candidates: raise SystemExit("no cube-class bbox")
    confidence,_,bbox,class_id,class_name=max(candidates,key=lambda x:x[0])
    model=RTMPose(args.rtmpose,model_input_size=(192,256),backend="onnxruntime",device="cpu")
    keypoints,scores=infer_rtmpose_mmpose_compatible(model,frame,[bbox])
    ordered=ordered_cube_keypoints_from_rtmpose(keypoints,scores,visible_confidence=0.50)
    out=frame.copy()
    pts=[(int(round(p.x)),int(round(p.y))) for p in ordered]
    x1,y1,x2,y2=[int(round(v)) for v in bbox]
    cv2.rectangle(out,(x1,y1),(x2,y2),(255,0,255),3)
    for a,b in CUBE_EDGES: cv2.line(out,pts[a],pts[b],(0,230,255),2,cv2.LINE_AA)
    for i,(p,pt) in enumerate(zip(ordered,pts)):
        cv2.circle(out,pt,6,(0,255,80),-1,cv2.LINE_AA)
        cv2.putText(out,f"k{i} {p.confidence:.2f}",(pt[0]+7,pt[1]-7),cv2.FONT_HERSHEY_SIMPLEX,0.45,(20,20,255),1,cv2.LINE_AA)
    cv2.putText(out,f"YOLO bbox: {class_name} {confidence:.3f}",(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.65,(255,0,255),2,cv2.LINE_AA)
    cv2.putText(out,"Cube8 RTMPose: 8 ordered vertices | PnP not run (sample has no CameraInfo)",(10,50),
                cv2.FONT_HERSHEY_SIMPLEX,0.48,(0,255,255),1,cv2.LINE_AA)
    cv2.imwrite(args.output_image,out)
    payload={
        "pipeline":"yolo_bbox+cube8_rtmpose",
        "image":args.image,
        "detector":{"class_id":class_id,"class_name":class_name,"confidence":confidence,"bbox_xyxy":bbox},
        "rtmpose":{"keypoints":[{"index":i,"label":p.label,"xy":[p.x,p.y],"confidence":p.confidence,"visibility":p.visibility}
                                for i,p in enumerate(ordered)]},
        "pnp":{"status":"not_run","reason":"sample image has no matching CameraInfo"},
        "grasp":{"status":"not_computed","execution_authorized":False},
        "retained_model_qualification":"do_not_promote",
    }
    Path(args.output_json).write_text(json.dumps(payload,indent=2)+"\n")
    print(json.dumps(payload,indent=2))
if __name__=="__main__": main()
