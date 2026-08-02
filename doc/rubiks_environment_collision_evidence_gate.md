# Rubik environment-collision evidence gate

## Authority

```text
lane_id: RUBIK-ENVIRONMENT-COLLISION-PREFLIGHT
writer_lease: WL-RUBIK-ENVIRONMENT-COLLISION-PREFLIGHT-20260803-01
parent_pr: #11
exact_base: e15aefedcf416733bed53f9553026ba478af2fec
authority: documentation/test-design/evidence-only
```

## Decision

The environment-collision phase is intentionally stopped before creating a planning scene:

```text
BLOCKED_INPUT_STATE_MISMATCH_AND_SINGULARITY
```

The accepted supported-hold artifact and the command-free kinematic evidence do not share an evidence-consistent initial arm state.

## Accepted physics source

The audit downloads and verifies the exact accepted PR #7 artifact:

- workflow run: `30290131232`;
- exact head: `a759a6efa2bd945aeff17697da22a19d8b41bb12`;
- artifact ID: `8662639552`;
- artifact SHA-256: `b5f3bcc6205c57f53e013531d6c5b2bf2bdc0b0e680c664985ea850567bcf1d3`.

The artifact records:

- cube pose `[-0.019, 0.0, 0.1192] m` relative to `turtlebot3_lime_gripper_test::link7`;
- support pose `[-0.019, 0.0, 0.0857] m` relative to the same frame;
- `arm_motion_performed=false`;
- no lift, support removal, IFRA attachment, or grasp-success claim.

Its rendered ROS 2 Control definition configures `joint1` through `joint6` with position-state initial values of `0.0 rad`. However, the artifact does not preserve measured six-axis arm joint samples. It records gripper and link/model telemetry but not the exact measured `joint1` through `joint6` state.

## Incompatibility with PR #10

PR #10 selected a deterministic numerical seed solely to obtain a finite, well-conditioned Jacobian:

```text
joint1 = -1.1309733553
joint2 =  0.5896769411
joint3 =  0.6848671985
joint4 = -0.6283185307
joint5 = -0.6333450790
joint6 =  0.8922123136
```

The maximum difference from the supported-hold configured state is approximately `1.13097 rad`. Applying the PR #7 cube/support poses around this seed would silently assume a large, unverified whole-arm repositioning before the correction begins.

That assumption is prohibited. The accepted physics evidence establishes the environment relative to the fixed supported-hold state, not relative to any arbitrary nonsingular posture.

## Configured supported-hold Jacobian

Using the exact production URDF at the configured all-zero arm state, the geometric Jacobian is rank deficient:

- matrix dimension: `6 x 6`;
- rank: `3`;
- condition number: infinite;
- a requested `+0.5 mm` local-Y correction produces `0.0 mm` at first order;
- the lateral residual remains the full `0.5 mm`;
- a yaw-only correction is numerically available, but a mixed lateral/yaw request retains the full lateral residual.

Therefore the accepted lateral active-centering proposal cannot be converted into a valid correction at the evidence-bound supported-hold state.

## Why environment collision is not run

A planning scene needs one common state for:

1. the robot joint configuration;
2. the world-fixed cube and support poses;
3. the correction start and endpoints;
4. interpolation between them.

No such common evidence-bound state currently exists. Running collision checks anyway would produce a precise-looking answer for an invented scene.

The verified environment-collision envelope remains:

```text
lateral_m = 0.0
yaw_rad = 0.0
```

## Next required phase

The next safe phase is `RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT`.

It must remain command-free and:

1. define or discover a nearby nonsingular pregrasp posture within effective URDF/ROS 2 Control limits;
2. preserve an explicit transform from the supported cube/support scene to that posture;
3. check self-collision and environment collision along the complete transition from the evidence-bound supported-hold state;
4. establish whether the cube can remain supported and outside unintended robot collisions during that transition;
5. record measured or exact simulated `joint1` through `joint6` state in any later physics evidence;
6. reject rather than invent any missing transform, joint state, or contact allowance.

## Safety boundary

- offline evidence audit only;
- no planning scene is created in this phase;
- no ROS node, controller, publisher, service client, or action client;
- no trajectory message or execution;
- no Gazebo motion;
- no arm or gripper command;
- no lift, support removal, or IFRA attachment;
- no production runtime modification;
- `actuation_authorized=false`.
