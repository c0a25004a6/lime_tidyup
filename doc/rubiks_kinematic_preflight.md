# Rubik command-free kinematic preflight

## Authority

```text
lane_id: RUBIK-KINEMATIC-PREFLIGHT
writer_lease: WL-RUBIK-KINEMATIC-PREFLIGHT-20260728-01
parent_pr: #9
exact_base: 64b88c9b176ca138571562158f98a2bf2daf32b8
authority: documentation/test-design/evidence-only
```

This phase consumes the accepted active-centering observation matrix without loading a controller or sending a command. It does not authorize a later actuation phase.

## Canonical model and controller resolution

The evidence harness builds the current Lime Docker image and resolves the installed upstream packages from that exact image. It then:

1. expands the test-only Lime Xacro, which retains the upstream manipulator and `ros2_control` definitions;
2. walks the rendered URDF chain from `link1` to `link7` instead of hard-coding arm joint names;
3. records joint ordering, axes, origins, URDF position/velocity/effort limits, and ROS 2 command/state interfaces;
4. records the position-command minimum and maximum embedded in each rendered `ros2_control` joint;
5. uses the intersection of URDF and `ros2_control` position-command limits as the effective numerical limit;
6. copies and parses the installed production `gazebo_controller_manager.yaml`;
7. requires one `JointTrajectoryController` whose joint order exactly matches the resolved arm chain;
8. records the expected `control_msgs/action/FollowJointTrajectory` endpoint as schema evidence only.

The upstream definitions do not use identical position ranges for every arm joint. Treating the URDF range alone as authoritative can produce a preview that the hardware/control interface would reject. The preflight therefore fails when either source is missing or their intersection cannot preserve the required one-degree margin.

No controller is spawned and no action client is created.

## Offline Jacobian method

The preflight implements a dependency-light geometric Jacobian directly from the rendered URDF. A deterministic seed is selected from bounded deterministic candidates. Seeds are rejected when they violate a one-degree effective joint-limit margin or produce a non-finite or ill-conditioned Jacobian.

Accepted observation cases from PR #9 are converted into a local `link7` twist and evaluated with damped least squares:

```text
dq = J^T (J J^T + lambda^2 I)^-1 dx
```

Recorded checks include:

- finite joint deltas;
- effective position limits with margin;
- maximum per-joint preview step;
- nonlinear forward-kinematics residual;
- unwanted translation;
- deterministic handling of the two upstream-rejected observation cases.

The result is a numerical preview only. No trajectory message is instantiated.

## Bounds and collision status

The observation envelope remains:

- lateral preview: at most `1.0 mm`;
- yaw preview: at most `2 degrees`.

This phase does not load a MoveIt planning scene and does not perform self/environment collision checking. Therefore the only collision-verified actuation bound is deliberately:

```text
collision_verified_lateral_m = 0.0
collision_verified_yaw_rad = 0.0
```

Any unknown collision state deterministically rejects and produces a no-op. Numerical IK feasibility must not be described as collision-free feasibility.

## Fail-closed behavior

The preflight rejects on:

- missing or malformed URDF/controller evidence;
- a non-six-DOF `link1` to `link7` chain;
- controller joint-order mismatch;
- missing position command/state interfaces;
- missing `ros2_control` position-command minimum or maximum;
- unusable URDF/control-limit intersection;
- non-finite input or output;
- singularity or condition-number limit;
- effective joint-limit margin violation;
- excessive preview joint step;
- residual outside tolerance;
- unknown collision state;
- timeout in any future consumer.

Rollback behavior is always `deterministic_no_op`; no prior proposal, default pose, or saturated preview may be substituted.

## Safety boundary

- offline analysis only;
- Gazebo observation remains a simulation oracle only;
- no production pose-estimator claim;
- no controller spawner;
- no action client;
- no publisher;
- no trajectory message;
- no arm or gripper command;
- no lift or support removal;
- no IFRA attachment;
- no grasp-success claim;
- no production URDF/controller/runtime modification;
- no physical-hardware readiness claim;
- `actuation_authorized=false`.

## Next permissible gate

After exact-head evidence passes, the next safe phase may validate self-collision and environment-collision state offline or through a read-only MoveIt planning-scene check. It must remain command-free and establish a nonzero collision-verified correction envelope before any Gazebo arm trajectory can be considered.
