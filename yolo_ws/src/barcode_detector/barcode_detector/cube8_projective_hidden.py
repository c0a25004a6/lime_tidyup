"""Projectively derive one fully hidden Cube8 corner from seven observed vertices."""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Mapping, Sequence

_X_NEIGHBOR = (1,0,3,2,5,4,7,6)
_Y_NEIGHBOR = (3,2,1,0,7,6,5,4)
_Z_NEIGHBOR = (4,5,6,7,0,1,2,3)

class Cube8HiddenGeometryError(ValueError):
    pass

@dataclass(frozen=True)
class HiddenVertex2D:
    index: int
    x: float
    y: float
    axis_line_rms_error_px: float
    pairwise_intersection_spread_px: float

def derive_hidden_vertex(
    observed_vertices: Mapping[int, Sequence[float]],
    *,
    hidden_index: int,
    vanishing_x: Sequence[float],
    vanishing_y: Sequence[float],
    vanishing_z: Sequence[float],
) -> HiddenVertex2D:
    """Return a dependent hidden-corner estimate; never an independent observation."""
    if isinstance(hidden_index,bool) or not isinstance(hidden_index,int) or not 0 <= hidden_index < 8:
        raise Cube8HiddenGeometryError("hidden_index must be an integer in [0,7]")
    if hidden_index in observed_vertices:
        raise Cube8HiddenGeometryError("hidden vertex must not be supplied as an observed point")
    invalid=[i for i in observed_vertices if isinstance(i,bool) or not isinstance(i,int) or not 0 <= i < 8]
    if invalid:
        raise Cube8HiddenGeometryError(f"observed vertex indices outside [0,7]: {invalid}")
    x_neighbor=_point(observed_vertices,_X_NEIGHBOR[hidden_index])
    y_neighbor=_point(observed_vertices,_Y_NEIGHBOR[hidden_index])
    z_neighbor=_point(observed_vertices,_Z_NEIGHBOR[hidden_index])
    vx=_finite_point(vanishing_x,"vanishing_x")
    vy=_finite_point(vanishing_y,"vanishing_y")
    vz=_finite_point(vanishing_z,"vanishing_z")
    axis_lines=(_line(x_neighbor,vx),_line(y_neighbor,vy),_line(z_neighbor,vz))
    hidden=_least_squares(axis_lines)
    pairwise=(
        _intersection(axis_lines[0],axis_lines[1]),
        _intersection(axis_lines[0],axis_lines[2]),
        _intersection(axis_lines[1],axis_lines[2]),
    )
    residuals=[_distance(hidden,line) for line in axis_lines]
    rms=math.sqrt(sum(value*value for value in residuals)/3.0)
    spread=max(math.hypot(a[0]-b[0],a[1]-b[1])
               for i,a in enumerate(pairwise) for b in pairwise[i+1:])
    return HiddenVertex2D(hidden_index,hidden[0],hidden[1],rms,spread)

def _point(points,index):
    if index not in points:
        raise Cube8HiddenGeometryError(f"required hidden-neighbor vertex missing: {index}")
    return _finite_point(points[index],f"vertex_{index}")

def _finite_point(raw,name):
    if len(raw)!=2:
        raise Cube8HiddenGeometryError(f"{name} must contain x,y")
    point=(float(raw[0]),float(raw[1]))
    if not all(math.isfinite(v) for v in point):
        raise Cube8HiddenGeometryError(f"{name} is non-finite")
    return point

def _line(a,b):
    x0,y0=_finite_point(a,"line_a"); x1,y1=_finite_point(b,"line_b")
    A=y0-y1; B=x1-x0; C=x0*y1-x1*y0
    n=math.hypot(A,B)
    if n<=1e-12: raise Cube8HiddenGeometryError("degenerate hidden-axis line")
    return (A/n,B/n,C/n)

def _intersection(first,second):
    a0,b0,c0=first; a1,b1,c1=second
    den=a0*b1-b0*a1
    if abs(den)<=1e-12: raise Cube8HiddenGeometryError("hidden-axis lines are parallel")
    return ((b0*c1-c0*b1)/den,(c0*a1-a0*c1)/den)

def _least_squares(lines):
    aa=sum(a*a for a,_,_ in lines); ab=sum(a*b for a,b,_ in lines); bb=sum(b*b for _,b,_ in lines)
    ac=sum(a*c for a,_,c in lines); bc=sum(b*c for _,b,c in lines)
    det=aa*bb-ab*ab
    if abs(det)<=1e-12: raise Cube8HiddenGeometryError("hidden-axis line system is rank-deficient")
    point=((-ac*bb+ab*bc)/det,(-aa*bc+ab*ac)/det)
    if not all(math.isfinite(v) for v in point): raise Cube8HiddenGeometryError("hidden estimate is non-finite")
    return point

def _distance(point,line):
    a,b,c=line
    return abs(a*point[0]+b*point[1]+c)
