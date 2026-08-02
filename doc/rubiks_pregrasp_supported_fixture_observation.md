# Rubik pregrasp supported-fixture observation

## Authority

```text
lane_id: RUBIK-PREGRASP-SUPPORTED-FIXTURE-OBSERVATION
writer_lease: WL-RUBIK-PREGRASP-SUPPORTED-FIXTURE-OBSERVATION-20260803-01
parent_pr: #14
exact_base: 036e27628e7ebf23e42a7d47d31594aa9ba5a1b8
authority: simulation-only fixture reconstruction evidence
```

This phase reconstructs the accepted supported Rubik fixture only after the deterministic pregrasp candidate has been measured in Gazebo. It does not close the gripper, move the arm after fixture spawn, lift the cube, or claim environment-collision clearance.

## Setup sequence

The exact target is regenerated from the PR #13 structured search. In one headless Gazebo process the workflow:

1. starts the stripped test robot with no fixture object;
2. loads only `joint_state_broadcaster` and `arm_controller`;
3. repeats the accepted PR #14 one-goal arm alignment and verifies measured tracking/settling;
4. loads `gripper_controller` only after the arm candidate is established;
5. sends exactly one simulation-only open command to `0.019 m`;
6. verifies the open command reached its goal without stall;
7. records the measured six-joint arm state, gripper state, and `link7` pose;
8. spawns the canonical support and 57 mm/90 g dynamic cube relative to measured candidate `link7`;
9. sends no further arm or gripper command;
10. observes fixture settling for at least 2.0 seconds.

## Evidence-bound fixture transforms

The transforms are copied exactly from accepted PR #7 supported-hold evidence:

```text
reference frame: turtlebot3_lime_gripper_test::link7
support xyz: [-0.019, 0.0, 0.0857] m
cube xyz:    [-0.019, 0.0, 0.1192] m
```

The support model is `rubiks_cube_support_surface`; its collision box is `0.075 x 0.040 x 0.010 m`. The cube model is `rubiks_dynamic_cube_supported_057`; its collision box is exactly `0.057 m` on each axis and its mass is `0.09 kg`.

No replacement pose is inferred from the earlier singular all-zero state. The fixture is spawned only after the new candidate state has been measured.

## Observation and acceptance

The cube contact sensor provides support, finger, and unintended robot contact evidence. The observation requires:

- arm candidate error at most `0.01 rad` before and after fixture spawn;
- `link7` drift after fixture spawn at most `2 mm` and `0.01 rad`;
- exactly one setup arm goal and one setup gripper-open goal;
- zero arm and gripper commands after fixture spawn;
- support contact messages and support-contact sample ratio at least `0.70`;
- zero finger contact while the gripper remains open;
- zero non-finger robot contact;
- observation duration at least `2.0 s`;
- cube final linear speed at most `0.02 m/s`;
- cube final angular speed at most `0.2 rad/s`;
- cube horizontal drift at most `3 mm`;
- cube rotation at most `0.10 rad`;
- finite measured arm, gripper, `link7`, cube, support, contact, and model-name telemetry.

The gripper remains open. Contact with a finger is a failure, not a successful grasp.

## Command accounting

Two setup commands are allowed before fixture spawn:

1. one arm trajectory to the accepted candidate;
2. one gripper-open command.

After fixture spawn:

```text
post_fixture_arm_command_count = 0
post_fixture_gripper_command_count = 0
```

No close, hold, lift, support removal, IFRA attachment, or rollback trajectory is sent.

## Fail-closed behavior

The phase rejects on:

- candidate mismatch or tracking failure;
- failed or stalled gripper-open goal;
- any extra setup command;
- any post-fixture arm or gripper command;
- changed fixture transform or model;
- missing support contact;
- finger or unintended robot contact;
- excessive cube movement or speed;
- arm or `link7` drift;
- insufficient or non-finite telemetry;
- any environment-collision or grasp-success claim.

On failure the simulation process is terminated. No automatic motion rollback is claimed.

## Safety boundary

- simulation only;
- no physical hardware access/readiness claim;
- no gripper close command;
- no arm or gripper command after fixture spawn;
- no lift, support removal, IFRA attachment, or grasp-success claim;
- no planning-scene/environment-collision claim;
- no production runtime modification;
- `actuation_authorized=false` outside the two exact bounded setup commands.

## Next permissible gate

After exact-head acceptance, `RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE` may build a command-free MoveIt planning scene from the newly measured six-joint state and measured cube/support world poses. It must compare the scene geometry and transforms against this artifact, check the candidate and correction endpoints plus bounded interpolation for self/environment collision, and must not send any controller command.
