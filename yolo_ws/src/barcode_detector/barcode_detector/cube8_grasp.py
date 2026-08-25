"""Pure geometric grasp candidates from a validated Cube8 3-D pose."""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence

class Cube8GraspError(ValueError): pass

@dataclass(frozen=True)
class GraspCandidate:
    face_id: str
    closing_axis_id: str
    grasp_position_m: tuple[float,float,float]
    pregrasp_position_m: tuple[float,float,float]
    approach_direction: tuple[float,float,float]
    closing_direction: tuple[float,float,float]
    opening_width_m: float
    grasp_frame_xyzw: tuple[float,float,float,float]
    grasp_frame_rpy_rad: tuple[float,float,float]
    score: float
    def as_dict(self):
        return {
            "face_id":self.face_id,
            "closing_axis_id":self.closing_axis_id,
            "grasp_position_m":list(self.grasp_position_m),
            "pregrasp_position_m":list(self.pregrasp_position_m),
            "approach_direction":list(self.approach_direction),
            "closing_direction":list(self.closing_direction),
            "opening_width_m":self.opening_width_m,
            "grasp_frame_convention":"x=closing, z=-approach, y=z_cross_x",
            "grasp_frame_xyzw":list(self.grasp_frame_xyzw),
            "grasp_frame_rpy_rad":list(self.grasp_frame_rpy_rad),
            "score":self.score,
            "execution_authorized":False,
        }

def rank_cube_grasp_candidates(*, translation_m: Sequence[float], rotation_xyzw: Sequence[float],
                               side_length_m: float=0.057, finger_clearance_m: float=0.003,
                               pregrasp_standoff_m: float=0.10, table_z_m: float=0.0,
                               table_clearance_m: float=0.01) -> tuple[GraspCandidate,...]:
    center=_vec(translation_m,3,"translation_m")
    R=_rotation_matrix_xyzw(*_vec(rotation_xyzw,4,"rotation_xyzw"))
    side=_finite(side_length_m,"side_length_m")
    if not 0.04 <= side <= 0.08: raise Cube8GraspError("side length outside supported envelope")
    clearance=_finite(finger_clearance_m,"finger_clearance_m")
    standoff=_finite(pregrasp_standoff_m,"pregrasp_standoff_m")
    if not 0 <= clearance <= 0.02 or not 0 < standoff <= 0.5: raise Cube8GraspError("invalid grasp clearance/standoff")
    axes={"X":_col(R,0),"Y":_col(R,1),"Z":_col(R,2)}
    tangents={"X":("Y","Z"),"Y":("X","Z"),"Z":("X","Y")}
    opening=side+2*clearance
    out=[]
    for axis_id in ("X","Y","Z"):
        axis=axes[axis_id]
        for sign,txt in ((1.0,"+"),(-1.0,"-")):
            outward=_scale(axis,sign)
            pre=_add(center,_scale(outward,standoff))
            if pre[2] < float(table_z_m)+float(table_clearance_m): continue
            approach=_scale(outward,-1.0)
            for close_id in tangents[axis_id]:
                closing=axes[close_id]
                q,rpy=_grasp_frame(closing,approach)
                top=max(0.0,outward[2]); side_app=1.0-abs(outward[2]); horiz=1.0-abs(closing[2])
                score=3*top+side_app+0.75*horiz+min(0.5,max(0.0,pre[2]-float(table_z_m)))
                out.append(GraspCandidate(
                    f"{txt}{axis_id}",close_id,center,pre,approach,closing,opening,q,rpy,score))
    out.sort(key=lambda c:(-c.score,c.face_id,c.closing_axis_id))
    return tuple(out)

def _grasp_frame(closing, approach):
    import numpy as np
    x=np.asarray(closing,dtype=float); x=x/np.linalg.norm(x)
    z=-np.asarray(approach,dtype=float); z=z/np.linalg.norm(z)
    y=np.cross(z,x)
    n=np.linalg.norm(y)
    if n < 1e-9: raise Cube8GraspError("closing and approach axes are degenerate")
    y=y/n; x=np.cross(y,z); x=x/np.linalg.norm(x)
    R=np.column_stack((x,y,z))
    return _quat(R),_rpy(R)

def _quat(R):
    m00,m01,m02=R[0]; m10,m11,m12=R[1]; m20,m21,m22=R[2]
    tr=m00+m11+m22
    if tr>0:
        s=math.sqrt(tr+1)*2; w=.25*s; x=(m21-m12)/s; y=(m02-m20)/s; z=(m10-m01)/s
    elif m00>m11 and m00>m22:
        s=math.sqrt(1+m00-m11-m22)*2; w=(m21-m12)/s; x=.25*s; y=(m01+m10)/s; z=(m02+m20)/s
    elif m11>m22:
        s=math.sqrt(1+m11-m00-m22)*2; w=(m02-m20)/s; x=(m01+m10)/s; y=.25*s; z=(m12+m21)/s
    else:
        s=math.sqrt(1+m22-m00-m11)*2; w=(m10-m01)/s; x=(m02+m20)/s; y=(m12+m21)/s; z=.25*s
    n=math.sqrt(x*x+y*y+z*z+w*w)
    return (x/n,y/n,z/n,w/n)

def _rpy(R):
    sy=math.sqrt(R[0,0]**2+R[1,0]**2)
    if sy > 1e-9:
        roll=math.atan2(R[2,1],R[2,2]); pitch=math.atan2(-R[2,0],sy); yaw=math.atan2(R[1,0],R[0,0])
    else:
        roll=math.atan2(-R[1,2],R[1,1]); pitch=math.atan2(-R[2,0],sy); yaw=0.0
    return (float(roll),float(pitch),float(yaw))

def _rotation_matrix_xyzw(x,y,z,w):
    n=math.sqrt(x*x+y*y+z*z+w*w)
    if n<1e-12: raise Cube8GraspError("zero quaternion")
    x,y,z,w=x/n,y/n,z/n,w/n
    return (
        (1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
        (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
        (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)),
    )
def _vec(v,n,name):
    if not isinstance(v,(list,tuple)) or len(v)!=n: raise Cube8GraspError(f"{name} must contain {n} values")
    return tuple(_finite(x,name) for x in v)
def _finite(v,name):
    x=float(v)
    if not math.isfinite(x): raise Cube8GraspError(f"{name} must be finite")
    return x
def _col(M,i): return tuple(float(M[r][i]) for r in range(3))
def _scale(v,s): return tuple(float(x)*float(s) for x in v)
def _add(a,b): return tuple(float(a[i])+float(b[i]) for i in range(3))
