#!/usr/bin/env python3
"""Offline, command-free Lime arm kinematic preflight."""
from __future__ import annotations
import argparse, hashlib, json, math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml

BASE, TIP = "link1", "link7"
MARGIN, MAX_COND, MAX_STEP, DAMP = math.radians(1), 250.0, math.radians(5), 1e-4
CASES = (("center",0.,0.),("minus_y",-.0005,0.),("plus_y",.0005,0.),
         ("minus_yaw",0.,-math.radians(1)),("plus_yaw",0.,math.radians(1)),
         ("mixed",.0005,-math.radians(1)))

def vec(text, default):
    a=np.array([float(x) for x in (text.split() if text else default)],float)
    if a.shape!=(3,) or not np.isfinite(a).all(): raise ValueError(f"bad vector {a}")
    return a

def rot_rpy(v):
    r,p,y=v; cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array(((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
                     (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),(-sp,cp*sr,cp*cr)))

def rot_axis(a,t):
    a=a/np.linalg.norm(a); x,y,z=a; c,s=math.cos(t),math.sin(t); d=1-c
    return np.array(((c+x*x*d,x*y*d-z*s,x*z*d+y*s),(y*x*d+z*s,c+y*y*d,y*z*d-x*s),(z*x*d-y*s,z*y*d+x*s,c+z*z*d)))

def tf(R=None,p=None):
    T=np.eye(4); T[:3,:3]=R if R is not None else np.eye(3); T[:3,3]=p if p is not None else 0; return T

def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def model(path):
    root=ET.parse(path).getroot(); by_child={}
    for e in root.findall("joint"):
        c=e.find("child")
        if c is not None and c.get("link"): by_child[c.get("link")]=e
    chain=[]; cur=TIP
    while cur!=BASE:
        e=by_child.get(cur)
        if e is None: raise ValueError(f"cannot resolve chain at {cur}")
        chain.append(e); cur=e.find("parent").get("link")
    out=[]
    for e in reversed(chain):
        typ=e.get("type")
        if typ=="fixed": continue
        if typ not in ("revolute","continuous"): raise ValueError(f"unsupported {typ}")
        o,a,l=e.find("origin"),e.find("axis"),e.find("limit")
        lo=float(l.get("lower")) if l is not None and l.get("lower") else None
        hi=float(l.get("upper")) if l is not None and l.get("upper") else None
        if typ=="revolute" and (lo is None or hi is None): raise ValueError(f"limits missing for {e.get('name')}")
        out.append(dict(name=e.get("name"),type=typ,parent=e.find("parent").get("link"),child=e.find("child").get("link"),
                        xyz=vec(o.get("xyz") if o is not None else None,(0,0,0)),
                        rpy=vec(o.get("rpy") if o is not None else None,(0,0,0)),
                        axis=vec(a.get("xyz") if a is not None else None,(1,0,0)),lower=lo,upper=hi,
                        effort=float(l.get("effort")) if l is not None and l.get("effort") else None,
                        velocity=float(l.get("velocity")) if l is not None and l.get("velocity") else None))
    if len(out)!=6: raise ValueError(f"expected 6 arm joints, got {len(out)}")
    interfaces={}
    for ctl in root.findall("ros2_control"):
        for j in ctl.findall("joint"):
            n=j.get("name");
            if n: interfaces[n]={"command":[x.get("name") for x in j.findall("command_interface")],"state":[x.get("name") for x in j.findall("state_interface")]}
    return out,interfaces

def controller(path,names):
    d=yaml.safe_load(path.read_text()); mgr=d.get("controller_manager",{}).get("ros__parameters",{})
    found=[]
    for name,decl in mgr.items():
        if isinstance(decl,dict) and "JointTrajectoryController" in str(decl.get("type","")):
            p=d.get(name,{}).get("ros__parameters",{}); joints=[str(x) for x in p.get("joints",[])]
            found.append((name,decl.get("type"),joints,p))
    exact=[x for x in found if x[2]==names]
    if not exact: raise ValueError(f"no exact trajectory-controller joint order; found={[(x[0],x[2]) for x in found]}")
    n,t,j,p=exact[0]
    return {"name":n,"type":t,"joints":j,"command_interfaces":p.get("command_interfaces",[]),"state_interfaces":p.get("state_interfaces",[]),
            "action_type":"control_msgs/action/FollowJointTrajectory","action_name":f"/{n}/follow_joint_trajectory","joint_order_exact":True}

def clamp(chain,q):
    q=q.copy()
    for i,j in enumerate(chain):
        if j["type"]=="continuous": q[i]=math.atan2(math.sin(q[i]),math.cos(q[i]))
        else: q[i]=min(j["upper"]-MARGIN,max(j["lower"]+MARGIN,q[i]))
    return q

def valid(chain,q):
    if not np.isfinite(q).all(): return False
    return all(j["type"]=="continuous" or j["lower"]+MARGIN<=x<=j["upper"]-MARGIN for j,x in zip(chain,q))

def fk_j(chain,q):
    T=np.eye(4); axes=[]; pts=[]
    for j,x in zip(chain,q):
        T=T@tf(rot_rpy(j["rpy"]),j["xyz"]); axes.append(T[:3,:3]@(j["axis"]/np.linalg.norm(j["axis"]))); pts.append(T[:3,3].copy()); T=T@tf(rot_axis(j["axis"],x))
    J=np.zeros((6,len(chain)))
    for i,(a,p) in enumerate(zip(axes,pts)): J[:3,i]=np.cross(a,T[:3,3]-p); J[3:,i]=a
    return T,J

def cond(J):
    s=np.linalg.svd(J,compute_uv=False); return math.inf if s[-1]<=1e-9 else float(s[0]/s[-1])

def seed(chain):
    mid=np.array([0. if j["type"]=="continuous" else (j["lower"]+j["upper"])/2 for j in chain])
    span=np.array([2*math.pi if j["type"]=="continuous" else j["upper"]-j["lower"] for j in chain])
    patterns=((0,(0,0,0,0,0,0)),(.1,(1,-1,1,-1,1,-1)),(.1,(-1,1,-1,1,-1,1)),(.2,(1,1,-1,-1,1,-1)),(.2,(-1,1,1,-1,-1,1)),(.3,(1,-1,-1,1,1,-1)))
    scored=[]
    for f,s in patterns:
        q=clamp(chain,mid+f*span*np.array(s)); c=cond(fk_j(chain,q)[1])
        if valid(chain,q) and math.isfinite(c): scored.append((c,f"deterministic:{f}:{s}",q))
    if not scored: raise ValueError("no finite deterministic seed")
    c,label,q=min(scored,key=lambda x:(x[0],x[1]))
    if c>MAX_COND: raise ValueError(f"best seed condition {c} exceeds {MAX_COND}")
    return label,q,c

def solve(chain,q,dy,dyaw):
    A,J=fk_j(chain,q); R=A[:3,:3]; dx=np.r_[R@np.array((0,dy,0.)),R@np.array((0,0,dyaw))]
    dq=J.T@np.linalg.solve(J@J.T+DAMP**2*np.eye(6),dx); qp=q+dq; B,_=fk_j(chain,qp); rel=np.linalg.inv(A)@B
    ay=float(rel[1,3]); yaw=math.atan2(rel[1,0],rel[0,0]); yr=math.atan2(math.sin(dyaw-yaw),math.cos(dyaw-yaw))
    unwanted=float(np.linalg.norm(rel[:3,3]-np.array((0,ay,0.)))); ok=valid(chain,qp) and np.isfinite(dq).all() and np.max(np.abs(dq))<=MAX_STEP and abs(dy-ay)<=5e-5 and abs(yr)<=math.radians(.05) and unwanted<=1e-4
    return {"requested_lateral_m":dy,"requested_yaw_rad":dyaw,"joint_delta_rad":dq.tolist(),"preview_joint_positions_rad":qp.tolist(),"max_abs_joint_delta_rad":float(np.max(np.abs(dq))),
            "achieved_lateral_m":ay,"achieved_yaw_rad":yaw,"lateral_residual_m":dy-ay,"yaw_residual_rad":yr,"unwanted_translation_norm_m":unwanted,
            "position_limits_with_margin":valid(chain,qp),"numerically_feasible":bool(ok),"trajectory_instantiated":False,"command_authorized":False}

def main():
    a=argparse.ArgumentParser(); a.add_argument("--urdf",required=True); a.add_argument("--controller-yaml",required=True); a.add_argument("--output",required=True); a.add_argument("--manifest",required=True); a.add_argument("--source-ref",default="unknown"); x=a.parse_args()
    up,cp,op,mp=map(Path,(x.urdf,x.controller_yaml,x.output,x.manifest)); errors=[]
    try:
        chain,ifs=model(up); names=[j["name"] for j in chain]; ctl=controller(cp,names)
        for n in names:
            if n not in ifs or "position" not in ifs[n]["command"] or "position" not in ifs[n]["state"]: errors.append(f"{n} lacks position ros2_control interface")
        label,q,c=seed(chain); cases=[]
        for name,dy,yaw in CASES:
            r=solve(chain,q,dy,yaw); r["label"]=name; cases.append(r)
            if not r["numerically_feasible"]: errors.append(f"{name} numerical preflight failed")
        joints=[{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in j.items()}|{"ros2_control":ifs.get(j["name"],{})} for j in chain]
        summary={"schema_version":1,"phase":"RUBIK-KINEMATIC-PREFLIGHT","writer_lease":"WL-RUBIK-KINEMATIC-PREFLIGHT-20260728-01","source_ref":x.source_ref,"passed":not errors,"errors":errors,
                 "arm_chain":{"base_link":BASE,"tip_link":TIP,"joint_count":6,"joint_order":names,"joints":joints},"controller":ctl,
                 "ik_method":{"name":"damped_least_squares_geometric_jacobian","damping":DAMP,"seed_label":label,"seed_joint_positions_rad":q.tolist(),"seed_condition_number":c,"max_condition_number":MAX_COND,"joint_limit_margin_rad":MARGIN,"max_joint_step_rad":MAX_STEP},
                 "accepted_observation_cases":cases,"upstream_rejected_observation_cases":[{"label":"reject_y","ik_attempted":False,"command_authorized":False},{"label":"reject_yaw","ik_attempted":False,"command_authorized":False}],
                 "bounds":{"kinematic_preview_lateral_m":.001,"kinematic_preview_yaw_rad":math.radians(2),"collision_verified_lateral_m":0.,"collision_verified_yaw_rad":0.,"reason":"no planning scene or collision checker executed"},
                 "rollback":{"behavior":"deterministic_no_op","on_collision_state_unknown":"reject","on_joint_limit_violation":"reject","on_singularity":"reject","on_timeout":"reject"},
                 "safety":{"offline_only":True,"simulation_oracle_only":True,"production_pose_estimator_claimed":False,"controller_loaded":False,"action_client_created":False,"publisher_created":False,"trajectory_instantiated":False,"arm_command_sent":False,"gripper_command_sent":False,"support_removed":False,"lift_command_sent":False,"ifra_attachment_used":False,"grasp_success_claimed":False,"collision_clearance_verified":False,"actuation_authorized":False,"production_runtime_modified":False}}
    except Exception as e:
        summary={"schema_version":1,"phase":"RUBIK-KINEMATIC-PREFLIGHT","writer_lease":"WL-RUBIK-KINEMATIC-PREFLIGHT-20260728-01","source_ref":x.source_ref,"passed":False,"errors":[f"{type(e).__name__}: {e}"],"safety":{"offline_only":True,"action_client_created":False,"publisher_created":False,"arm_command_sent":False,"gripper_command_sent":False,"actuation_authorized":False,"production_runtime_modified":False}}
    op.write_text(json.dumps(summary,indent=2,sort_keys=True)+"\n")
    manifest={"schema_version":1,"phase":"RUBIK-KINEMATIC-PREFLIGHT","source_ref":x.source_ref,"exact_head_bound":x.source_ref not in ("","unknown"),"summary":op.name,"summary_sha256":digest(op),"inputs":{"urdf":digest(up),"controller_yaml":digest(cp)},"command_free":True,"actuation_authorized":False}
    mp.write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n"); print(json.dumps(summary,indent=2,sort_keys=True)); return 0 if summary.get("passed") else 1
if __name__=="__main__": raise SystemExit(main())
