# Rubik pregrasp fixture placement offline collision preflight

Phase: `RUBIK-PREGRASP-FIXTURE-PLACEMENT-OFFLINE-COLLISION-PREFLIGHT`

Writer lease: `WL-RUBIK-PREGRASP-FIXTURE-OFFLINE-COLLISION-20260804-01`

## Purpose

This phase reconstructs the accepted PR #7 cube/support fixture entirely offline and places it relative to the selected-pose `link7` state accepted by PR #21. The resulting cube and support poses are then held fixed in world coordinates while every recorded PR #21 arm/finger state is replayed in MoveIt.

No Gazebo process, ROS node, controller, publisher, service client, action client, trajectory, object spawn, attachment, or physical hardware is used.

## Accepted inputs

The phase binds exact archive digests and critical member digests from:

- PR #7 supported close/hold/release evidence;
- PR #21 passive gripper and ground-clearance evidence.

The current exact-head expanded URDF and SRDF must match the accepted PR #21 model byte-for-byte.

## Fixture contract

The accepted fixture is:

- dynamic cube collision: `0.057 × 0.057 × 0.057 m`, mass `0.09 kg`;
- static support collision: `0.075 × 0.040 × 0.010 m`;
- reference link: `link7`;
- cube offset: `(-0.019, 0, 0.1192) m`;
- support offset: `(-0.019, 0, 0.0857) m`.

The center-Z separation is exactly the sum of the cube and support half-heights. The cube/support surface contact is the only intended fixture contact. No robot–cube or robot–support contact is allowed in this phase.

## Placement policy

The fixture is anchored at the final recorded selected-pose hold state. Both object transforms are converted to world coordinates at that anchor and remain fixed while the complete recorded path is replayed:

- approach from the initial state;
- selected-pose hold;
- return trajectory;
- zero hold.

This tests the accepted fixture against the actual recorded path rather than silently moving the fixture with the end effector.

## Decision

The evidence phase completes with one of two decisions:

- `FIXTURE_PLACEMENT_CLEAR`: every state remains in bounds, self-collision-free, cube-clear, and support-clear;
- `BLOCKED_ROBOT_FIXTURE_COLLISION`: at least one robot–cube or robot–support contact is recorded.

A blocked result is a valid completed preflight. Thresholds and contact rules must not be weakened to obtain a clear result.

## Claim boundary

The result is limited to exact sampled states and the fixed accepted fixture pose. It does not claim continuous clearance, positive distance margin, Gazebo fixture stability, grasp/contact quality, active gripper control, lift capability, or hardware readiness.

## Safety boundary

- offline analysis only;
- no Gazebo startup;
- no ROS node;
- no object spawn;
- no controller or command;
- no planning request or trajectory;
- no attachment;
- no physical hardware;
- no production runtime modification;
- Draft and unmerged.
