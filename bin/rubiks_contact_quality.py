#!/usr/bin/env python3
"""Shared cube-local contact-quality audit for Rubik grasp physics evidence."""
from __future__ import annotations

import bisect
import math

CUBE_HALF_M = 0.0285
FACE_TOLERANCE_M = 0.0010
EDGE_MARGIN_M = 0.0010
NORMAL_Y_MIN = 0.70
MAX_CONTACT_DEPTH_M = 0.0010
MIN_QUALITY_FRACTION = 0.80
MIN_CONTACT_POINTS = 10


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def quat_conjugate(q: list[float]) -> list[float]:
    require(len(q) == 4, "quaternion must be xyzw")
    x, y, z, w = (float(v) for v in q)
    norm2 = x*x + y*y + z*z + w*w
    require(math.isfinite(norm2) and norm2 > 1e-20, "invalid quaternion")
    return [-x/norm2, -y/norm2, -z/norm2, w/norm2]


def rotate(q: list[float], v: list[float]) -> list[float]:
    x, y, z, w = (float(value) for value in q)
    vx, vy, vz = (float(value) for value in v)
    norm2 = x*x + y*y + z*z + w*w
    require(math.isfinite(norm2) and norm2 > 1e-20, "invalid quaternion")
    scale = 1.0 / math.sqrt(norm2)
    x *= scale; y *= scale; z *= scale; w *= scale
    tx = 2.0 * (y*vz - z*vy)
    ty = 2.0 * (z*vx - x*vz)
    tz = 2.0 * (x*vy - y*vx)
    return [vx + w*tx + (y*tz-z*ty), vy + w*ty + (z*tx-x*tz), vz + w*tz + (x*ty-y*tx)]


def world_to_cube_point(cube_state: dict[str, object], point: list[float]) -> list[float]:
    position = [float(v) for v in cube_state["position_m"]]
    orientation = [float(v) for v in cube_state["orientation_xyzw"]]
    return rotate(quat_conjugate(orientation), [float(point[i])-position[i] for i in range(3)])


def world_to_cube_vector(cube_state: dict[str, object], vector: list[float]) -> list[float]:
    orientation = [float(v) for v in cube_state["orientation_xyzw"]]
    return rotate(quat_conjugate(orientation), [float(v) for v in vector])


def nearest_state(states: list[dict[str, object]], t_s: float) -> dict[str, object]:
    require(bool(states), "cube-state evidence is empty")
    times = [float(state["t_s"]) for state in states]
    index = bisect.bisect_left(times, float(t_s))
    choices = []
    if index < len(states): choices.append(states[index])
    if index > 0: choices.append(states[index-1])
    require(bool(choices), "no cube state near contact")
    return min(choices, key=lambda state: abs(float(state["t_s"])-float(t_s)))


def _finger_from_state(contact_state: dict[str, object]) -> str | None:
    pair = f"{contact_state.get('collision1','')} {contact_state.get('collision2','')}"
    if "cube_link" not in pair and "cube_collision" not in pair:
        return None
    if "gripper_left_link" in pair: return "left"
    if "gripper_right_link" in pair: return "right"
    return None


def _empty() -> dict[str, object]:
    return {"total":0,"qualified":0,"side":0,"interior":0,"ynormal":0,"max_depth":0.0,"sum_pos":[0.0]*3,"sum_abs_normal":[0.0]*3}


def audit_contact_quality(contact_messages: list[dict[str, object]], cube_states: list[dict[str, object]], *, start_s: float | None=None, end_s: float | None=None) -> dict[str, object]:
    acc = {"left": _empty(), "right": _empty()}
    for event in contact_messages:
        t_s = float(event["t_s"])
        if start_s is not None and t_s < float(start_s): continue
        if end_s is not None and t_s > float(end_s): continue
        cube_state = nearest_state(cube_states, t_s)
        for state in event.get("states", []):
            finger = _finger_from_state(state)
            if finger is None: continue
            positions = state.get("contact_positions_m", [])
            normals = state.get("contact_normals", [])
            depths = state.get("depths_m", [])
            for i in range(min(len(positions),len(normals),len(depths))):
                p = world_to_cube_point(cube_state, positions[i])
                n = world_to_cube_vector(cube_state, normals[i])
                an = [abs(v) for v in n]
                depth = abs(float(depths[i]))
                side = ((CUBE_HALF_M-FACE_TOLERANCE_M <= p[1] <= CUBE_HALF_M+FACE_TOLERANCE_M) if finger=="left" else (-CUBE_HALF_M-FACE_TOLERANCE_M <= p[1] <= -CUBE_HALF_M+FACE_TOLERANCE_M))
                interior = abs(p[0]) <= CUBE_HALF_M-EDGE_MARGIN_M and abs(p[2]) <= CUBE_HALF_M-EDGE_MARGIN_M
                ynormal = an[1] >= NORMAL_Y_MIN and an[1] > an[0] and an[1] > an[2]
                qualified = side and interior and ynormal and depth <= MAX_CONTACT_DEPTH_M
                a = acc[finger]
                a["total"] += 1; a["qualified"] += int(qualified); a["side"] += int(side); a["interior"] += int(interior); a["ynormal"] += int(ynormal)
                a["max_depth"] = max(float(a["max_depth"]), depth)
                for axis in range(3):
                    a["sum_pos"][axis] += p[axis]; a["sum_abs_normal"][axis] += an[axis]
    result: dict[str, object] = {"criteria":{"cube_half_m":CUBE_HALF_M,"face_tolerance_m":FACE_TOLERANCE_M,"edge_margin_m":EDGE_MARGIN_M,"normal_y_min":NORMAL_Y_MIN,"max_contact_depth_m":MAX_CONTACT_DEPTH_M,"min_quality_fraction":MIN_QUALITY_FRACTION,"min_contact_points":MIN_CONTACT_POINTS}}
    passed = True
    for finger in ("left","right"):
        a=acc[finger]; total=int(a["total"]); qualified=int(a["qualified"]); fraction=qualified/total if total else 0.0
        finger_pass=total>=MIN_CONTACT_POINTS and fraction>=MIN_QUALITY_FRACTION
        passed = passed and finger_pass
        result[finger]={"total_contact_points":total,"qualified_contact_points":qualified,"quality_fraction":fraction,"side_face_fraction":int(a["side"])/total if total else 0.0,"interior_fraction":int(a["interior"])/total if total else 0.0,"y_normal_dominant_fraction":int(a["ynormal"])/total if total else 0.0,"max_depth_m":float(a["max_depth"]),"mean_local_position_m":[v/total if total else 0.0 for v in a["sum_pos"]],"mean_abs_local_normal":[v/total if total else 0.0 for v in a["sum_abs_normal"]],"passed":finger_pass}
    result["passed"] = passed
    return result


def self_test() -> None:
    cube=[{"t_s":1.0,"position_m":[0.0,0.0,0.0],"orientation_xyzw":[0.0,0.0,0.0,1.0]}]
    def state(finger: str, y: float, p: list[float], n: list[float]):
        return {"collision1":"cube_link::cube_collision","collision2":f"robot::gripper_{finger}_link::collision","contact_positions_m":[p]*10,"contact_normals":[n]*10,"depths_m":[1e-5]*10}
    good=[{"t_s":1.0,"states":[state("left",CUBE_HALF_M,[0.0,CUBE_HALF_M,0.0],[0.0,-1.0,0.0]),state("right",-CUBE_HALF_M,[0.0,-CUBE_HALF_M,0.0],[0.0,1.0,0.0])]}]
    require(audit_contact_quality(good,cube)["passed"] is True,"known-good side contact failed")
    corner=[{"t_s":1.0,"states":[state("left",CUBE_HALF_M,[-CUBE_HALF_M,CUBE_HALF_M,-CUBE_HALF_M],[1.0,0.0,0.0]),state("right",-CUBE_HALF_M,[-CUBE_HALF_M,-CUBE_HALF_M,-CUBE_HALF_M],[1.0,0.0,0.0])]}]
    require(audit_contact_quality(corner,cube)["passed"] is False,"corner contact incorrectly passed")
    print("rubiks_contact_quality self-test: PASS")


if __name__ == "__main__":
    import argparse, json
    from pathlib import Path
    parser=argparse.ArgumentParser(); parser.add_argument("--telemetry"); parser.add_argument("--output"); parser.add_argument("--self-test",action="store_true"); args=parser.parse_args()
    if args.self_test: self_test(); raise SystemExit(0)
    require(bool(args.telemetry),"--telemetry is required")
    data=json.loads(Path(args.telemetry).read_text())
    result=audit_contact_quality(data.get("contact_messages",[]),data.get("cube_state_samples",[]),start_s=(data.get("hold_result") or {}).get("hold_start_s"),end_s=(data.get("hold_result") or {}).get("hold_end_s"))
    if args.output: Path(args.output).write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2)); raise SystemExit(0 if result["passed"] else 1)
