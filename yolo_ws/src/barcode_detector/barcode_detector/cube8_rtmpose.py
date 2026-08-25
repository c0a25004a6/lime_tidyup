"""Detector-neutral Cube8 RTMPose runtime adapted from lime-interactive-transport."""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Any, Sequence
from .cube8_geometry import PoseKeypoint2D, ordered_keypoints

class Cube8RtmPoseError(ValueError):
    pass

@dataclass(frozen=True)
class ImageBBox:
    x0: float
    y0: float
    x1: float
    y1: float
    def as_list(self) -> list[float]:
        return [self.x0,self.y0,self.x1,self.y1]

def infer_rtmpose_mmpose_compatible(model: Any, image_bgr: Any,
                                    bboxes: Sequence[Sequence[float]],
                                    *, simcc_split_ratio: float = 2.0):
    """Run RTMLib raw ONNX inference using the pinned Cube8/MMPose semantics."""
    import numpy as np
    image=np.asarray(image_bgr)
    if image.ndim != 3 or image.shape[2] != 3:
        raise Cube8RtmPoseError(f"expected HxWx3 BGR image, got {image.shape}")
    ratio=float(simcc_split_ratio)
    if not math.isfinite(ratio) or ratio <= 0:
        raise Cube8RtmPoseError("simcc_split_ratio must be positive")
    rgb=np.ascontiguousarray(image[..., ::-1])
    boxes=list(bboxes) or [[0.0,0.0,float(image.shape[1]),float(image.shape[0])]]
    input_size=np.asarray(getattr(model,"model_input_size",()),dtype=np.float64)
    if input_size.shape != (2,) or not np.isfinite(input_size).all() or (input_size <= 0).any():
        raise Cube8RtmPoseError("invalid RTMPose model_input_size")
    kp_batches=[]
    score_batches=[]
    for bbox in boxes:
        try:
            resized,center,scale=model.preprocess(rgb,list(bbox))
            outputs=model.inference(resized)
        except Exception as exc:
            raise Cube8RtmPoseError(f"RTMPose inference failed: {type(exc).__name__}: {exc}") from exc
        if not isinstance(outputs,(list,tuple)) or len(outputs) != 2:
            raise Cube8RtmPoseError("SimCC ONNX must return [simcc_x, simcc_y]")
        sx=np.asarray(outputs[0],dtype=np.float64)
        sy=np.asarray(outputs[1],dtype=np.float64)
        if sx.ndim != 3 or sy.ndim != 3 or sx.shape[0] != 1 or sy.shape[0] != 1 or sx.shape[1] != sy.shape[1]:
            raise Cube8RtmPoseError(f"unexpected SimCC shapes x={sx.shape} y={sy.shape}")
        if sx.shape[1] != 8:
            raise Cube8RtmPoseError(f"Cube8 model must emit 8 keypoints, got {sx.shape[1]}")
        if not np.isfinite(sx).all() or not np.isfinite(sy).all():
            raise Cube8RtmPoseError("non-finite SimCC output")
        xloc=np.argmax(sx[0],axis=1)
        yloc=np.argmax(sy[0],axis=1)
        maxx=np.max(sx[0],axis=1)
        maxy=np.max(sy[0],axis=1)
        scores=np.minimum(maxx,maxy)
        points=np.stack((xloc,yloc),axis=-1).astype(np.float64)
        points[scores <= 0.0]=-1.0
        points/=ratio
        center=np.asarray(center,dtype=np.float64).reshape(-1)
        scale=np.asarray(scale,dtype=np.float64).reshape(-1)
        if center.shape != (2,) or scale.shape != (2,) or not np.isfinite(center).all() or not np.isfinite(scale).all() or (scale <= 0).any():
            raise Cube8RtmPoseError("invalid preprocess center/scale")
        points=points/input_size*scale + center - scale/2.0
        kp_batches.append(points)
        score_batches.append(scores)
    return np.stack(kp_batches,axis=0),np.stack(score_batches,axis=0)

def ordered_cube_keypoints_from_rtmpose(keypoints: Any, scores: Any,
                                        *, visible_confidence: float = 0.50) -> tuple[PoseKeypoint2D,...]:
    def plain(v):
        for method in ("detach","cpu","numpy"):
            if hasattr(v,method):
                v=getattr(v,method)()
        if hasattr(v,"tolist"):
            v=v.tolist()
        return v
    pts=plain(keypoints); scr=plain(scores)
    if isinstance(pts,list) and len(pts)==1: pts=pts[0]
    if isinstance(scr,list) and len(scr)==1: scr=scr[0]
    return ordered_keypoints(pts,scr,visible_confidence=visible_confidence)

def expand_detection_bbox(*, center_x: float, center_y: float, size_x: float, size_y: float,
                          expansion: float, image_width: int, image_height: int) -> ImageBBox:
    vals=[float(center_x),float(center_y),float(size_x),float(size_y),float(expansion)]
    if not all(math.isfinite(v) for v in vals):
        raise Cube8RtmPoseError("bbox contains non-finite values")
    if size_x <= 0 or size_y <= 0 or expansion < 1.0 or image_width <= 0 or image_height <= 0:
        raise Cube8RtmPoseError("invalid bbox geometry")
    hw=0.5*float(size_x)*float(expansion); hh=0.5*float(size_y)*float(expansion)
    x0=max(0.0,float(center_x)-hw); y0=max(0.0,float(center_y)-hh)
    x1=min(float(image_width),float(center_x)+hw); y1=min(float(image_height),float(center_y)+hh)
    if x1 <= x0 or y1 <= y0:
        raise Cube8RtmPoseError("expanded bbox is empty")
    return ImageBBox(x0,y0,x1,y1)
