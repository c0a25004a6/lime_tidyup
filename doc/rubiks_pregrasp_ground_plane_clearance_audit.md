# Rubik pregrasp ground-plane clearance audit

Phase: `RUBIK-PREGRASP-GROUND-PLANE-CLEARANCE-AUDIT`

Writer lease: `WL-RUBIK-PREGRASP-GROUND-PLANE-CLEARANCE-AUDIT-20260803-01`

## Purpose

This phase extends the accepted empty-scene selected-pose and return-to-zero evidence with one narrowly scoped environment check: the recorded arm-chain states are replayed offline against the exact Gazebo ground plane.

The phase adds no new motion goal and does not place a collision object into the running Gazebo world. Gazebo is used only to observe the already-running model and link transforms while the unchanged two-goal trial executes. MoveIt performs the ground-plane checks after the simulation process has completed.

## Bound inputs

The evidence binds all of the following to the exact pull-request head:

- the existing selected-pose and return-to-zero simulation telemetry;
- every recorded `joint1..joint6` state;
- the requested Gazebo spawn pose: `(0, 0, 0.03)` with zero yaw;
- the observed Gazebo model pose;
- the observed `base_link` pose and, when exported by Gazebo, `base_footprint` pose;
- the fixed URDF transform from `base_footprint` to `base_link`;
- the test and production URDF collision geometry for `link1..link7`;
- the canonical SRDF allowed-collision matrix;
- the exact SDF `ground_plane` geometry.

A frame observation must be no more than `0.05 s` old at the matching joint-state sample. Joint-state matching is by exact ROS timestamp and a maximum position difference of `1e-9 rad`.

## Ground model

The accepted world must contain one static model named `ground_plane`. Its collision geometry must be an identity-pose SDF plane with:

- world normal `(0, 0, 1)`;
- declared size `6 m × 6 m`;
- world equation `z = 0`.

The MoveIt representation is an infinite `shapes::Plane` transformed into the observed `base_footprint` frame for every sample. The SDF size is preserved as evidence but is not used to claim finite-edge clearance; all audited arm geometry remains near the model origin and this phase makes no claim outside the declared plane footprint.

## Robot-frame binding

The simulation model and production MoveIt model have different robot names. The phase therefore requires:

- both models to have `base_footprint` as the unique root;
- an exactly equal fixed transform from `base_footprint` to observed `base_link`;
- exactly equal serialized collision geometry for `link1..link7`;
- a stable observed model-to-`base_link` transform throughout the trial.

The maximum accepted fixed-transform residual is `1e-5 m` and `1e-5 rad`.

## Collision policy

Two collision checks are kept separate:

1. self-collision uses the unmodified canonical SRDF allowed-collision matrix;
2. ground collision uses a temporary offline matrix that allows robot-to-robot pairs and allows the intentionally grounded base, wheel, caster, and other non-audited links against `ground_plane`.

Only these links are audited against the ground:

`link1`, `link2`, `link3`, `link4`, `link5`, `link6`, `link7`.

Every recorded state must:

- satisfy MoveIt arm bounds;
- remain self-collision-free;
- produce no contact pair between `ground_plane` and an audited arm link.

## Claim boundary

A passing result demonstrates only that the sampled recorded states of `link1..link7` were clear of the exact `ground_plane` under the observed root transforms and exact-head MoveIt model.

It does not demonstrate:

- continuous clearance between samples;
- gripper-link ground clearance;
- populated-scene or arbitrary-object clearance;
- cube or support clearance;
- perception, grasping, lifting, transport, or placement;
- physical-hardware readiness.

## Safety boundary

- simulation only;
- the same selected-pose and return-to-zero arm goals only;
- read-only Gazebo subscriptions;
- no new action, service, publisher, or command path;
- no cube, support, fixture, IFRA, or grasp-fix;
- no gripper, base, or lift command;
- no physical hardware;
- no production runtime modification;
- no merge requested.

## Next phase

A pass may authorize `RUBIK-PREGRASP-GRIPPER-STATE-AND-GROUND-CLEARANCE-EVIDENCE`.

That later phase must first bind the actual gripper joint state and collision geometry. This phase does not silently treat default gripper positions as measured evidence.
