# Rubik command-free self-collision preflight

## Authority

```text
lane_id: RUBIK-SELF-COLLISION-PREFLIGHT
writer_lease: WL-RUBIK-SELF-COLLISION-PREFLIGHT-20260803-01
parent_pr: #10
exact_base: 76866c54e19aa2fdf633558e64ecd5e26ed3b8cb
authority: documentation/test-design/evidence-only
```

This phase checks only self-collision for the accepted kinematic seed and six preview states. It does not check the cube, support, floor, robot surroundings, trajectory interpolation, dynamics, sensing, or hardware.

## Evidence design

The exact Lime Docker image is built from the PR head. Inside that image the workflow:

1. expands the exact production Lime Xacro with `use_sim:=true`, retaining robot name `turtlebot3_lime`;
2. copies the installed upstream SRDF for the same robot name;
3. regenerates the exact-head command-free kinematic summary;
4. extracts the deterministic seed and six accepted preview joint states;
5. builds a small C++ checker against the installed MoveIt Core and planning-scene libraries;
6. parses URDF and SRDF directly without creating a ROS node;
7. verifies the semantic `arm` group resolves to `joint1` through `joint6`;
8. requires collision geometry on `link1` through `link7` and both gripper links;
9. verifies an adjacent SRDF allowed-collision entry is present and applied;
10. checks each state against MoveIt bounds and the SRDF allowed-collision matrix;
11. records every colliding link pair and fails closed on any collision.

The earlier test-only Xacro has robot name `turtlebot3_lime_gripper_test`, while the canonical SRDF has robot name `turtlebot3_lime`. Rather than ignoring that semantic mismatch or rewriting the SRDF, this phase uses the production Xacro so the two canonical model names agree.

`move_group`, planning services, action clients, controllers, publishers, Gazebo, and trajectory messages are not started or created. Gazebo plugin elements in the rendered URDF are parsed as description data only; no plugin is loaded.

## Tested envelope

The matrix directly checks:

- the deterministic seed;
- center;
- lateral `-0.5 mm`;
- lateral `+0.5 mm`;
- yaw `-1 degree`;
- yaw `+1 degree`;
- mixed `+0.5 mm / -1 degree`.

A passing result may establish only the directly tested self-collision envelope:

```text
self_collision_verified_lateral_m = 0.0005
self_collision_verified_yaw_rad = 1 degree
environment_collision_verified_lateral_m = 0.0
environment_collision_verified_yaw_rad = 0.0
```

It must not be generalized to the full proposal threshold, an interpolated trajectory, or an environment-clearance claim.

## Fail-closed behavior

The phase rejects on:

- malformed URDF or SRDF;
- mismatched URDF/SRDF robot names;
- missing semantic `arm` group;
- active-joint order mismatch;
- missing collision geometry;
- an unapplied allowed-collision matrix;
- incomplete or out-of-bounds state;
- any self-collision or contact pair;
- parent kinematic evidence not bound to the exact head;
- unknown environment state.

Rollback remains `deterministic_no_op`.

## Safety boundary

- offline MoveIt Core library use only;
- no ROS node or service client;
- no publisher or action client;
- no controller or planning request;
- no trajectory message or execution;
- no Gazebo motion or plugin loading;
- no arm or gripper command;
- no support removal or lift;
- no IFRA attachment;
- no production URDF/SRDF/controller/runtime modification;
- environment collision remains unknown;
- `actuation_authorized=false`.

## Next permissible gate

After exact-head acceptance, the next safe phase may construct a command-free planning scene containing the canonical support and cube poses and check the same discrete states plus bounded interpolation. It must not execute a trajectory and must preserve zero actuation authorization until both environment clearance and dynamic rollback are established.
