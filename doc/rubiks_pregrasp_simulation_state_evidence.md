# Rubik pregrasp simulation-state evidence

## Authority

```text
lane_id: RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE
writer_lease: WL-RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE-20260803-01
parent_pr: #13
exact_base: 80c8deb9b5416f31c2c9a6d223bf8aa3492390d3
authority: simulation-only arm-state evidence
```

This phase is the first bounded arm command in the current chain. It is restricted to Gazebo, an empty scene, and one exact pregrasp target regenerated from the accepted deterministic search.

## Test-only controller declaration

The stripped test model previously declared only the joint-state broadcaster and gripper controller. This phase adds an `arm_controller` declaration to the test-only controller YAML using the same six-joint ordering, position command interface, position/velocity state interfaces, and `FollowJointTrajectory` controller type as production.

The addition does not load the controller automatically. Existing gripper workflows continue to spawn only the controllers they request. This phase explicitly spawns:

- `joint_state_broadcaster`;
- `arm_controller`.

It explicitly does not spawn `gripper_controller`.

## Production/test arm equivalence

Before Gazebo starts, the workflow expands both:

- the exact production Lime Xacro;
- the stripped test-only Lime Xacro.

Both models are processed through the PR #10 kinematic extractor. The phase requires identical signatures for `joint1` through `joint6`:

- type;
- parent and child links;
- origin XYZ/RPY;
- axis;
- URDF limits;
- `ros2_control` position-command limits;
- effective limit intersections.

Both trajectory-controller schemas must resolve to the same six-joint ordering and `control_msgs/action/FollowJointTrajectory` action endpoint.

## Trial sequence

1. Start the stripped test-only robot in headless Gazebo with no cube/support model.
2. Start only the joint-state broadcaster and arm trajectory controller.
3. Regenerate the candidate through the exact structured search from PR #13.
4. Observe the configured initial six-joint state and `link7` pose.
5. Send exactly one two-point `FollowJointTrajectory` goal:
   - measured initial state at `0.5 s`;
   - accepted candidate at `3.0 s`.
6. Observe the action result and hold the final state for `1.5 s`.
7. Record joint states, controller desired/actual/error states, Gazebo `link7` poses, and model names.
8. Stop the trial. No second movement or rollback trajectory is sent.

## Acceptance criteria

- one and only one arm goal;
- accepted action goal and successful action result;
- initial measured joint magnitude at most `0.01 rad`;
- final per-joint target error at most `0.01 rad`;
- final stable interval at least `1.0 s`;
- final arm speed at most `0.01 rad/s`;
- finite and monotonic telemetry;
- sufficient joint, controller, and `link7` samples;
- observed `link7` relative translation within `5 mm` of command-free FK;
- observed `link7` relative orientation within `0.03 rad` of command-free FK;
- no cube/support model observed;
- no active gripper controller;
- no gripper command or grasp/lift claim.

## Failure and rollback boundary

On rejection, timeout, tracking error, non-finite telemetry, or pose mismatch, the trial fails and the Gazebo process is stopped. No automatic second trajectory is sent. Therefore this phase does not claim a motion rollback to zero; it claims only process termination and fail-closed evidence.

## Safety boundary

- simulation only;
- no physical hardware access or readiness claim;
- one bounded simulation arm command;
- no gripper controller or gripper command;
- no cube/support spawn;
- no environment-collision check;
- no lift, support removal, IFRA attachment, or grasp-success claim;
- no production runtime modification;
- `actuation_authorized=false` outside this exact simulation trial.

## Next permissible gate

After exact-head acceptance, `RUBIK-PREGRASP-SUPPORTED-FIXTURE-OBSERVATION` may repeat the accepted empty-scene alignment, verify the measured candidate state, then spawn the canonical support and 57 mm cube relative to measured `link7`. It must remain arm/gripper-command-free after fixture spawn and establish stable support contact, no unintended robot contact, and exact measured six-joint state before environment-collision planning-scene evidence resumes.
