"""Rigid pose composition for moving camera-frame Cube8 evidence into robot/world frames."""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence

class PoseTransformError(ValueError): pass

@dataclass(frozen=True)
class RigidTransform:
    translation_m: tuple[float,float,float]
    rotation_xyzw: tuple[float,float,float,float]
    def validate(self):
        if not all(math.isfinite(float(v)) for v in (*self.translation_m,*self.rotation_xyzw)):
            raise PoseTransformError("non-finite rigid transform")
        _normalize_quaternion(self.rotation_xyzw)

def _normalize_quaternion(q):
    if len(q)!=4: raise PoseTransformError("quaternion must contain 4 values")
    q=tuple(float(v) for v in q)
    n=math.sqrt(sum(v*v for v in q))
    if not all(math.isfinite(v) for v in q) or n<=1e-12: raise PoseTransformError("invalid quaternion")
    return tuple(v/n for v in q)

def _multiply(a,b):
    ax,ay,az,aw=_normalize_quaternion(a); bx,by,bz,bw=_normalize_quaternion(b)
    return _normalize_quaternion((
        aw*bx+ax*bw+ay*bz-az*by,
        aw*by-ax*bz+ay*bw+az*bx,
        aw*bz+ax*by-ay*bx+az*bw,
        aw*bw-ax*bx-ay*by-az*bz))

def _rotate(q,v):
    if len(v)!=3: raise PoseTransformError("vector must contain 3 values")
    vx,vy,vz=(float(x) for x in v)
    qx,qy,qz,qw=_normalize_quaternion(q)
    dot=qx*vx+qy*vy+qz*vz; uu=qx*qx+qy*qy+qz*qz
    cx=qy*vz-qz*vy; cy=qz*vx-qx*vz; cz=qx*vy-qy*vx
    return (
        2*dot*qx+(qw*qw-uu)*vx+2*qw*cx,
        2*dot*qy+(qw*qw-uu)*vy+2*qw*cy,
        2*dot*qz+(qw*qw-uu)*vz+2*qw*cz)

def transform_pose(transform: RigidTransform, position_m: Sequence[float],
                   orientation_xyzw: Sequence[float]):
    transform.validate()
    if len(position_m)!=3: raise PoseTransformError("position must contain 3 values")
    rotated=_rotate(transform.rotation_xyzw,position_m)
    position=tuple(rotated[i]+transform.translation_m[i] for i in range(3))
    orientation=_multiply(transform.rotation_xyzw,orientation_xyzw)
    return position,orientation
