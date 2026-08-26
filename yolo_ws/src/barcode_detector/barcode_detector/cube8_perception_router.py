"""Cost-aware routing for Cube8 perception components.

This module only decides which already-existing perception component should run.
It does not embed detector, color, projective-geometry, PnP, or depth thresholds.
Callers re-plan after each stage so expensive RealSense work is only requested
when the current goal or prior evidence actually needs it.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Mapping


class Cube8RoutingError(ValueError):
    pass


class ComponentPolicy(str, Enum):
    OFF = "off"
    ON_DEMAND = "on_demand"
    ALWAYS = "always"


class PerceptionComponent(str, Enum):
    DETECTOR = "detector"
    COLOR_GEOMETRY = "color_geometry"
    PNP = "pnp"
    SPARSE_DEPTH = "sparse_depth"
    DEPTH_PLANE = "depth_plane"
    LEARNED_FALLBACK = "learned_fallback"


class PerceptionGoal(IntEnum):
    BBOX = 1
    CUBE_2D = 2
    CUBE_3D = 3
    GRASP_VERIFY = 4


class RobotPhase(str, Enum):
    SEARCH = "search"
    APPROACH = "approach"
    PREGRASP = "pregrasp"
    EXECUTION_CHECK = "execution_check"


class CheckState(str, Enum):
    NOT_RUN = "not_run"
    PASS = "pass"
    FAIL = "fail"


@dataclass(frozen=True)
class CascadePolicies:
    detector: ComponentPolicy = ComponentPolicy.ON_DEMAND
    color_geometry: ComponentPolicy = ComponentPolicy.ON_DEMAND
    pnp: ComponentPolicy = ComponentPolicy.ON_DEMAND
    sparse_depth: ComponentPolicy = ComponentPolicy.ON_DEMAND
    depth_plane: ComponentPolicy = ComponentPolicy.ON_DEMAND
    learned_fallback: ComponentPolicy = ComponentPolicy.OFF

    def for_component(self, component: PerceptionComponent) -> ComponentPolicy:
        return {
            PerceptionComponent.DETECTOR: self.detector,
            PerceptionComponent.COLOR_GEOMETRY: self.color_geometry,
            PerceptionComponent.PNP: self.pnp,
            PerceptionComponent.SPARSE_DEPTH: self.sparse_depth,
            PerceptionComponent.DEPTH_PLANE: self.depth_plane,
            PerceptionComponent.LEARNED_FALLBACK: self.learned_fallback,
        }[component]


@dataclass(frozen=True)
class PerceptionCapabilities:
    camera_intrinsics: bool = False
    depth_stream: bool = False
    learned_model: bool = False


@dataclass(frozen=True)
class PerceptionEvidence:
    track_reusable: bool = False
    detector: CheckState = CheckState.NOT_RUN
    color_geometry: CheckState = CheckState.NOT_RUN
    pnp: CheckState = CheckState.NOT_RUN
    sparse_depth: CheckState = CheckState.NOT_RUN
    depth_plane: CheckState = CheckState.NOT_RUN
    learned_fallback: CheckState = CheckState.NOT_RUN


@dataclass(frozen=True)
class PerceptionRequest:
    goal: PerceptionGoal
    phase: RobotPhase = RobotPhase.APPROACH
    force_reacquire: bool = False


@dataclass(frozen=True)
class PerceptionPlan:
    run: tuple[PerceptionComponent, ...]
    reasons: Mapping[PerceptionComponent, str]
    blocked: tuple[str, ...]
    satisfied: bool
    authorizes_robot_motion: bool = False


_ORDER = (
    PerceptionComponent.DETECTOR,
    PerceptionComponent.COLOR_GEOMETRY,
    PerceptionComponent.LEARNED_FALLBACK,
    PerceptionComponent.PNP,
    PerceptionComponent.SPARSE_DEPTH,
    PerceptionComponent.DEPTH_PLANE,
)


def plan_cube8_perception(
    request: PerceptionRequest,
    evidence: PerceptionEvidence,
    capabilities: PerceptionCapabilities,
    policies: CascadePolicies = CascadePolicies(),
) -> PerceptionPlan:
    if not isinstance(request.goal, PerceptionGoal) or not isinstance(request.phase, RobotPhase):
        raise Cube8RoutingError("request contains an invalid goal or robot phase")

    run: set[PerceptionComponent] = set()
    reasons: dict[PerceptionComponent, str] = {}
    blocked: list[str] = []

    def capability_available(component: PerceptionComponent) -> bool:
        if component is PerceptionComponent.PNP:
            return capabilities.camera_intrinsics
        if component in (PerceptionComponent.SPARSE_DEPTH, PerceptionComponent.DEPTH_PLANE):
            return capabilities.depth_stream
        if component is PerceptionComponent.LEARNED_FALLBACK:
            return capabilities.learned_model
        return True

    def request_component(component: PerceptionComponent, reason: str, *, required: bool = False) -> bool:
        policy = policies.for_component(component)
        if policy is ComponentPolicy.OFF:
            if required:
                blocked.append(f"{component.value}_disabled")
            return False
        if not capability_available(component):
            if required or policy is ComponentPolicy.ALWAYS:
                blocked.append(f"{component.value}_unavailable")
            return False
        run.add(component)
        reasons.setdefault(component, reason)
        return True

    for component in _ORDER:
        if policies.for_component(component) is ComponentPolicy.ALWAYS:
            request_component(component, "policy_always")

    bbox_ready = (
        evidence.track_reusable and not request.force_reacquire
    ) or evidence.detector is CheckState.PASS
    if request.force_reacquire or not bbox_ready:
        request_component(
            PerceptionComponent.DETECTOR,
            "reacquire_required" if request.force_reacquire else "no_reusable_roi",
            required=True,
        )

    if request.goal is PerceptionGoal.BBOX:
        return _finish(run, reasons, blocked, bbox_ready and not run)

    geometry_ready = evidence.color_geometry is CheckState.PASS
    learned_ready = evidence.learned_fallback is CheckState.PASS
    observations_ready = geometry_ready or learned_ready
    if not observations_ready:
        if evidence.color_geometry is CheckState.FAIL:
            observations_ready = request_component(
                PerceptionComponent.LEARNED_FALLBACK,
                "color_geometry_failed",
                required=True,
            )
        else:
            observations_ready = request_component(
                PerceptionComponent.COLOR_GEOMETRY,
                "cube_vertices_required",
                required=True,
            )

    if request.goal is PerceptionGoal.CUBE_2D:
        return _finish(run, reasons, blocked, (geometry_ready or learned_ready) and not run)

    pnp_ready = evidence.pnp is CheckState.PASS
    if evidence.pnp is CheckState.NOT_RUN:
        request_component(PerceptionComponent.PNP, "metric_cube_pose_required", required=True)

    close_to_grasp = request.phase in (
        RobotPhase.PREGRASP,
        RobotPhase.EXECUTION_CHECK,
    ) or request.goal is PerceptionGoal.GRASP_VERIFY

    sparse_ready = evidence.sparse_depth is CheckState.PASS
    if close_to_grasp and evidence.sparse_depth is CheckState.NOT_RUN:
        request_component(
            PerceptionComponent.SPARSE_DEPTH,
            "pregrasp_metric_depth_crosscheck",
            required=request.goal is PerceptionGoal.GRASP_VERIFY,
        )
    elif evidence.pnp is CheckState.FAIL and evidence.sparse_depth is CheckState.NOT_RUN:
        request_component(PerceptionComponent.SPARSE_DEPTH, "pnp_failed_request_independent_depth")

    need_plane = evidence.sparse_depth is CheckState.FAIL or (
        request.goal is PerceptionGoal.GRASP_VERIFY
        and evidence.pnp is CheckState.FAIL
        and evidence.sparse_depth is not CheckState.NOT_RUN
    )
    if need_plane and evidence.depth_plane is not CheckState.PASS:
        request_component(
            PerceptionComponent.DEPTH_PLANE,
            "cheap_depth_or_pnp_crosscheck_failed",
            required=request.goal is PerceptionGoal.GRASP_VERIFY,
        )

    if request.goal is PerceptionGoal.CUBE_3D:
        satisfied = pnp_ready and observations_ready
    else:
        depth_verified = sparse_ready or evidence.depth_plane is CheckState.PASS
        satisfied = pnp_ready and observations_ready and depth_verified
    if run:
        satisfied = False
    return _finish(run, reasons, blocked, satisfied)


def _finish(run, reasons, blocked, satisfied) -> PerceptionPlan:
    ordered = tuple(component for component in _ORDER if component in run)
    return PerceptionPlan(
        run=ordered,
        reasons={component: reasons[component] for component in ordered},
        blocked=tuple(dict.fromkeys(blocked)),
        satisfied=bool(satisfied and not blocked),
        authorizes_robot_motion=False,
    )
