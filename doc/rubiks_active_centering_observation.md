# Rubik active-centering observation dry-run

## Reason for this phase

Passive supported-hold matrices failed asymmetrically even at a `+0.025 mm` initial lateral offset, while exact center and `-0.025 mm` passed. Further shrinking would not establish a useful passive manipulation envelope.

This phase validates only the geometry and sign conventions required by a future active-centering system. It does not move the robot.

## Observation source

The trial reads:

- the cube model pose from Gazebo `/model_states`;
- the `turtlebot3_lime_gripper_test::link7` pose from Gazebo `/link_states`.

The cube world pose is transformed into the `link7` coordinate system with quaternion inversion and composition. The observed relative Y and relative yaw are then converted into a proposed `link7` lateral shift and yaw rotation.

Gazebo state is a **simulation oracle only**. This phase does not claim that a production camera, marker detector, depth estimator, or real-hardware calibration exists.

## Contact-free fixture

The support and dynamic 57 mm / 90 g cube are spawned approximately `0.25 m` away from the gripper contact region while still using `link7` as their reference frame. The cube settles on the same explicit support used by PR #7.

Any cube contact with the robot is a hard failure. Support contact is required.

## Matrix

Accepted correction cases:

- center: `Y=0`, `yaw=0`;
- `Y=-0.5 mm`;
- `Y=+0.5 mm`;
- `yaw=-1 degree`;
- `yaw=+1 degree`;
- mixed `Y=+0.5 mm`, `yaw=-1 degree`.

Expected rejection cases:

- `Y=+1.5 mm`, beyond the configured `1.0 mm` lateral limit;
- `yaw=+3 degrees`, beyond the configured `2 degree` yaw limit.

Every case runs in a fresh container, ROS domain, and Gazebo process.

## Proposal semantics

For an accepted observation:

```text
proposed_link7_lateral_shift_m = measured cube Y in link7 frame
proposed_link7_yaw_rotation_rad = measured cube yaw in link7 frame
```

These values are evidence that the sign and coordinate transformation are internally consistent. They are not published to a controller.

For a rejected observation, executable proposal fields remain `null`. Saturated values may be included only as a non-executable preview for reviewing limit behavior.

## Acceptance

- observed Y agrees with the requested fixture Y within `0.15 mm`;
- observed yaw agrees within `0.15 degree`;
- accepted and rejected cases match the matrix expectation;
- cube linear speed remains below `0.02 m/s` during observation;
- cube angular speed remains below `0.2 rad/s`;
- support contact exists;
- robot contact does not exist;
- no action client, publisher, controller spawner, or command path exists in the harness.

## Safety boundary

- `simulation_oracle_only=true`;
- `production_pose_estimator_claimed=false`;
- `arm_controller_loaded=false`;
- `arm_command_sent=false`;
- `gripper_controller_loaded=false`;
- `gripper_command_sent=false`;
- support remains installed;
- no lift;
- no IFRA attachment;
- no grasp-success claim;
- no production URDF change.

## Next gate

After this matrix passes, the next safe phase is an arm-controller and kinematic-feasibility preflight that consumes the same correction proposal but still sends no trajectory. It must resolve the real arm joint names, limits, controller interface, Jacobian or IK method, collision-free correction bound, and rollback behavior before any Gazebo arm movement is authorized.
