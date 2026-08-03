# Rubik pregrasp dynamic path audit

## Scope

This phase reuses the already accepted empty-scene two-goal cycle:

1. zero to `[0, 0, +10°, 0, -5°, 0]`;
2. the same selected state back to zero.

It adds no command, endpoint, object, controller, planner, or hardware path. Its purpose is to bind every recorded actual joint-state sample to the accepted offline path and MoveIt self-collision model.

## Exact base

`e53cf5d1fd9613e0709f06e14f067aa44fea7ac8`

## Audit

The exact-head trial records all `/joint_states` samples. The audit then:

- requires finite six-joint position vectors and complete source binding;
- requires monotonic wall-clock and ROS timestamps;
- checks every position against the URDF/ros2_control effective limit intersection;
- projects every sample onto the zero-to-selected joint-space segment;
- retains the existing `0.02 rad` maximum per-joint nonmoving/path residual boundary;
- permits at most `1e-4` normalized segment-end excursion;
- sends every actual state through the existing MoveIt `arm` self-collision checker;
- requires all states in bounds, zero self-collisions, and zero contact pairs;
- binds the source trace, URDF, SRDF, controller configuration, extraction, and collision results by SHA-256.

## Claim boundary

A pass establishes only that the recorded empty-scene joint trace stayed inside the accepted joint-space tube and was self-collision-free under the exact-head MoveIt model.

It does not establish populated-scene clearance, ground-plane clearance, cube clearance, perception accuracy, grasping, transport, placement, or physical-hardware readiness.

The following remain prohibited and false:

- new arm goal or command path;
- gripper or base command;
- cube, support, or fixture;
- IFRA or grasp-fix;
- lift;
- production runtime modification;
- hardware use;
- grasp or environment-clearance claim.
