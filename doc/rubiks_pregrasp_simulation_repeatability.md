# Rubik pregrasp empty-scene repeatability evidence

## Purpose

This phase repeats the accepted empty-scene arm-state transition from PR #16 in three fresh containers. It tests whether the selected pregrasp joint state and return-to-zero transition repeatedly satisfy the same state, action, world-model, controller, and safety gates.

It does not add a Rubik's cube, support, fixture, gripper command, base command, lift, attachment, camera estimator, production runtime path, or hardware path.

## Exact base

The phase starts from PR #16 exact head:

`b0dcb07067af40099f14afafdcef224b59bf4259`

The selected arm goal remains:

- `joint1 = 0 deg`
- `joint2 = 0 deg`
- `joint3 = +10 deg`
- `joint4 = 0 deg`
- `joint5 = -5 deg`
- `joint6 = 0 deg`

Each trial sends exactly two arm goals:

1. selected pose;
2. return to zero.

## Trial isolation

The workflow builds one exact-head image, then runs three fresh containers with distinct ROS domain IDs. Each container starts a fresh Gazebo process and writes to a separate artifact directory.

Trial labels are fixed to:

- `trial_01`
- `trial_02`
- `trial_03`

A failed trial is preserved and does not prevent later trials from running. The aggregate gate fails closed unless all three trials pass.

## Per-trial gates

Every trial must retain the PR #16 acceptance thresholds and prove:

- accepted and successful candidate goal;
- accepted and successful return-to-zero goal;
- controller result status `SUCCEEDED` and error code `0`;
- sufficient joint-state and action-feedback samples;
- feedback error, overshoot, nonmoving-joint drift, final error, hold velocity, and hold range within the unchanged PR #16 limits;
- exact world-model set before and after: `ground_plane`, `turtlebot3_lime_arm_motion_test`;
- exact active controller set before and after: `joint_state_broadcaster`, `arm_controller`;
- one stable `gazebo_msgs/msg/ModelStates` topic;
- no gripper action or forbidden model/controller;
- exact source-ref binding.

## Aggregate evidence

The aggregator records all trial errors without hiding partial failures. It stores per-trial metrics and aggregate minimum, maximum, mean, and range for each measured motion quantity.

The manifest binds the aggregate summary and each required trial input by SHA-256 to the exact PR head.

## Safety boundary

The following remain false:

- physical hardware used;
- cube, support, or fixture spawned;
- gripper controller loaded or gripper command sent;
- base controller loaded or base command sent;
- IFRA attachment or grasp-fix plugin used;
- lift command sent;
- grasp success claimed;
- environment clearance claimed;
- hardware readiness claimed;
- production runtime modified.

Passing this phase demonstrates repeatable empty-scene controller-state execution only. It does not demonstrate collision-free motion in a populated scene, object perception, grasping, transport, placement, or hardware safety.
