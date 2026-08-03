# Rubik pregrasp passive gripper-state ground clearance

Phase: `RUBIK-PREGRASP-GRIPPER-STATE-AND-GROUND-CLEARANCE-EVIDENCE`

Writer lease: `WL-RUBIK-PREGRASP-GRIPPER-STATE-GROUND-CLEARANCE-20260803-01`

## Purpose

This phase extends the accepted arm-chain ground-plane evidence by binding the actual passive gripper joint state emitted by `joint_state_broadcaster` during the unchanged selected-pose and return-to-zero trial.

It does not load a gripper controller or send a gripper goal. The measured left and right prismatic positions are replayed offline in the exact-head MoveIt model.

## Measured state contract

The observer starts before the existing Gazebo runner and subscribes only to `/joint_states`. A usable sample must contain, in the same message:

- `joint1` through `joint6`;
- `gripper_left_joint`;
- `gripper_right_joint`;
- position and velocity arrays covering all required names.

Every accepted arm telemetry sample is matched by exact ROS timestamp. The maximum arm-position match error is `1e-9 rad`.

The test and production URDFs must agree exactly on:

- both gripper joint definitions;
- URDF and `ros2_control` position limits;
- state interfaces;
- right-joint mimic source and multiplier;
- collision geometry for `gripper_left_link` and `gripper_right_link`.

Measured left/right position residual and velocity residual must satisfy the declared mimic relation within `1e-6 m` and `1e-5 m/s` respectively. Every measured position must remain inside the URDF/`ros2_control` effective limit intersection.

## MoveIt replay

For every bound sample, MoveIt receives:

- the observed `base_footprint` world transform from the preceding ground audit;
- measured `joint1..joint6` values;
- measured `gripper_left_joint` value.

MoveIt applies the model mimic relation to `gripper_right_joint`. The modeled right value must match the measured right value within `1e-9 m`.

The canonical SRDF allowed-collision matrix is used for self-collision. A separate offline floor matrix allows intentionally grounded non-audited links and checks only:

- `gripper_left_link`;
- `gripper_right_link`.

Every sample must remain in bounds, self-collision-free, and free of finger-link contacts with the exact ground plane.

## Claim boundary

A pass demonstrates only that the sampled measured passive finger states:

- were finite and inside effective limits;
- obeyed the declared mimic relation;
- matched the exact-head MoveIt mimic model;
- were self-collision-free;
- did not contact the exact ground plane.

It does not demonstrate continuous-between-sample clearance, active gripper control, object clearance, grasp force, contact quality, lifting, transport, placement, or physical-hardware readiness.

## Safety boundary

- same two Gazebo arm goals only;
- passive `/joint_states` subscription only;
- no gripper controller;
- no gripper actuation;
- no new actuation path;
- no cube, support, or fixture;
- no base or lift actuation;
- no attachment plugin;
- no physical hardware;
- no production runtime modification;
- Draft and unmerged.

## Next phase

A pass may authorize `RUBIK-PREGRASP-FIXTURE-PLACEMENT-OFFLINE-COLLISION-PREFLIGHT`.

That later phase may reconstruct the previously accepted cube/support fixture poses offline. It must add no Gazebo object or manipulation command until the complete static placement and collision contract passes.
