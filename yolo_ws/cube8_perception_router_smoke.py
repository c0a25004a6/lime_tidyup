#!/usr/bin/env python3
"""Smoke the cost-aware Cube8 perception cascade without camera or robot IO."""
from barcode_detector.cube8_perception_router import (
    CascadePolicies,
    CheckState,
    ComponentPolicy,
    PerceptionCapabilities,
    PerceptionComponent,
    PerceptionEvidence,
    PerceptionGoal,
    PerceptionRequest,
    RobotPhase,
    plan_cube8_perception,
)


def main() -> int:
    track = plan_cube8_perception(
        PerceptionRequest(PerceptionGoal.BBOX, RobotPhase.SEARCH),
        PerceptionEvidence(track_reusable=True),
        PerceptionCapabilities(),
    )
    assert track.run == () and track.satisfied

    two_d = plan_cube8_perception(
        PerceptionRequest(PerceptionGoal.CUBE_2D),
        PerceptionEvidence(track_reusable=True),
        PerceptionCapabilities(),
    )
    assert two_d.run == (PerceptionComponent.COLOR_GEOMETRY,)

    three_d = plan_cube8_perception(
        PerceptionRequest(PerceptionGoal.CUBE_3D),
        PerceptionEvidence(track_reusable=True, color_geometry=CheckState.PASS),
        PerceptionCapabilities(camera_intrinsics=True, depth_stream=True),
    )
    assert three_d.run == (PerceptionComponent.PNP,)

    pregrasp = plan_cube8_perception(
        PerceptionRequest(PerceptionGoal.GRASP_VERIFY, RobotPhase.PREGRASP),
        PerceptionEvidence(track_reusable=True, color_geometry=CheckState.PASS),
        PerceptionCapabilities(camera_intrinsics=True, depth_stream=True),
    )
    assert pregrasp.run == (PerceptionComponent.PNP, PerceptionComponent.SPARSE_DEPTH)
    assert not pregrasp.authorizes_robot_motion

    escalated = plan_cube8_perception(
        PerceptionRequest(PerceptionGoal.GRASP_VERIFY, RobotPhase.PREGRASP),
        PerceptionEvidence(
            track_reusable=True,
            color_geometry=CheckState.PASS,
            pnp=CheckState.PASS,
            sparse_depth=CheckState.FAIL,
        ),
        PerceptionCapabilities(camera_intrinsics=True, depth_stream=True),
    )
    assert escalated.run == (PerceptionComponent.DEPTH_PLANE,)

    policies = CascadePolicies(learned_fallback=ComponentPolicy.ON_DEMAND)
    learned = plan_cube8_perception(
        PerceptionRequest(PerceptionGoal.CUBE_2D),
        PerceptionEvidence(track_reusable=True, color_geometry=CheckState.FAIL),
        PerceptionCapabilities(learned_model=True),
        policies,
    )
    assert learned.run == (PerceptionComponent.LEARNED_FALLBACK,)

    no_calibration = plan_cube8_perception(
        PerceptionRequest(PerceptionGoal.CUBE_3D),
        PerceptionEvidence(track_reusable=True, color_geometry=CheckState.PASS),
        PerceptionCapabilities(camera_intrinsics=False),
    )
    assert no_calibration.run == ()
    assert no_calibration.blocked == ("pnp_unavailable",)

    print("CUBE8_ROUTER track=reuse 2d=color_geometry 3d=pnp pregrasp=pnp+sparse_depth")
    print("CUBE8_ROUTER escalation=depth_plane learned=explicit_opt_in robot_authority=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
