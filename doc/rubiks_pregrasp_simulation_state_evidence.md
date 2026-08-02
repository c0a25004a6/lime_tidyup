# Rubik pregrasp simulation-state evidence

## Authority

```text
lane_id: RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE
writer_lease: WL-RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE-20260803-01
parent_pr: #13
exact_base: 80c8deb9b5416f31c2c9a6d223bf8aa3492390d3
authority: simulation-only bounded actuation evidence
```

This phase is the first authorization for arm motion in this lane. It is limited to an empty headless Gazebo scene and does not authorize a cube, support, gripper command, physical hardware, or production runtime change.

## Exact candidate regeneration

The workflow does not hard-code an unverified candidate. Inside the exact-head Lime image it expands the production simulation URDF, resolves the installed production controller definition, and re-runs the PR #13 structured search. The selected posture must remain:

```text
joint1 = 0 degrees
joint2 = 0 degrees
joint3 = +10 degrees
joint4 = 0 degrees
joint5 = -5 degrees
joint6 = 0 degrees
```

The search summary and its input hashes are included in the artifact.

## Test-only model

The spawned model is `turtlebot3_lime_arm_motion_test`. It retains the physical base, six-axis arm, gripper links, Gazebo manipulator dynamics, and ROS 2 Control interfaces, but intentionally omits:

- camera, depth, lidar, and IMU plugins;
- diff-drive and all base controllers;
- RealSense;
- grasp-fix;
- IFRA LinkAttacher;
- gripper controller;
- cube, support, contact sensor, or any other fixture.

Only `joint_state_broadcaster` and `arm_controller` may become active. The world-model set must be exactly `ground_plane` and the test robot before and after the trial.

## Bounded motion sequence

The evidence node:

1. receives at least 30 six-joint samples;
2. verifies a stable initial state near zero;
3. sends one explicit six-joint `FollowJointTrajectory` goal to the selected candidate over 3 seconds;
4. records action acceptance/result, controller feedback, joint states, tracking error, overshoot, nonmoving-joint drift, final error, velocity, and a 2-second hold;
5. sends one explicit six-joint return-to-zero goal over 3 seconds;
6. records the same evidence and a final 2-second hold.

No partial-joint goal is permitted. The arm-controller joint order must be `joint1` through `joint6`.

## Acceptance thresholds

- initial maximum absolute position error: `<=0.01 rad`;
- candidate and recovery final maximum absolute position error: `<=0.01 rad`;
- hold maximum absolute velocity: `<=0.02 rad/s`;
- hold position range: `<=0.005 rad`;
- transition overshoot: `<=0.02 rad`;
- nonmoving-joint drift: `<=0.02 rad`;
- maximum controller-feedback position error: `<=0.08 rad`;
- at least five feedback samples per goal;
- both goals must be accepted and return `SUCCESSFUL`/`STATUS_SUCCEEDED`.

The thresholds are simulation-evidence gates, not physical-hardware tolerances.

## Fail-closed behavior

The phase rejects on:

- candidate regeneration mismatch;
- missing or malformed six-joint samples;
- nonzero or unstable initial arm state;
- unavailable or incorrectly ordered arm action;
- rejected, aborted, or timed-out goal;
- exceeded tracking, overshoot, drift, velocity, hold, or recovery limits;
- gripper/base controller or action availability;
- any world model other than the ground plane and test robot;
- cube, support, fixture, contact sensor, grasp-fix, or IFRA presence;
- missing exact-head summary or manifest.

## Safety boundary

Authorized:

- headless Gazebo only;
- `joint_state_broadcaster` and `arm_controller`;
- one candidate arm goal and one return-to-zero arm goal.

Prohibited:

- physical hardware;
- gripper controller or command;
- base controller or command;
- cube, support, fixture, contact sensor, grasp-fix, or IFRA;
- lift or grasp claim;
- environment-clearance claim;
- hardware-readiness claim;
- production URDF/controller/runtime modification.

## Next permissible gate

A passing exact-head result may authorize a new supported-fixture simulation phase. That phase must first settle at the measured candidate, record measured `joint1` through `joint6`, and only then spawn the accepted cube and support transforms relative to `link7`. It must not reuse a guessed joint state or move the previously accepted zero-state scene together with the arm.
