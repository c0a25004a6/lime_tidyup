# Rubik pregrasp corrected-fixture geometric relevance preflight

## Purpose

PR #23 found the nearest declared-grid rigid fixture translation that removes every recorded robot–cube and robot–support collision: `(0, 0, +0.04575) m` in the selected-pose `link7` frame.

That result is collision-clear, but the translation is larger than the cube half extent. This phase therefore asks one narrower question before any gripper-motion work:

> Does the corrected cube still intersect the necessary two-finger reach envelope of the exact collision meshes?

## Inputs

The workflow digest-binds the accepted PR #23 artifact, including:

- exact selected correction and corrected cube/support centers;
- exact accepted URDF/SRDF;
- exact anchor arm and passive finger state;
- complete 2,802-state collision-clear path evidence.

It also records the exact `ROBOTIS-JAPAN-GIT/turtlebot3_lime` clone commit, origin, clean status, and SHA-256/size/resource path for both collision STL files.

## Geometry contract

The checker loads the accepted MoveIt robot model and requires each finger link to contain exactly one collision mesh. It evaluates the mesh AABB envelope in `link7` coordinates at:

- lower gripper limit;
- zero;
- accepted passive position;
- upper gripper limit.

The joint model must remain an unrotated linear mimic pair: the left AABB translates `+Y` by the requested joint position and the right AABB translates `-Y`, while X/Z bounds remain invariant to `1e-12 m`.

A corrected fixture is geometrically relevant only when both conditions hold:

1. both finger AABBs have positive overlap with the cube AABB in X and Z;
2. there is a common mimic-joint interval inside the exact limits where the positive and negative cube side planes are simultaneously contained by the corresponding finger AABBs.

No clearance tolerance is introduced to manufacture a positive result.

## Decisions

The completed evidence records one of:

- `CORRECTED_FIXTURE_GEOMETRICALLY_RELEVANT`; or
- `BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE`.

A positive result is only a necessary AABB-envelope condition. It does not prove exact mesh contact, contact normals, force closure, friction, support/ground clearance during gripper motion, grasp success, lifting, transport, placement, or hardware readiness.

## Next safe phase

When relevant, the next authorized phase is an offline gripper-joint sweep over the common interval, checking exact robot/cube/support/ground collision states without sending a command.

When blocked, the next authorized phase is a new offline fixture search that includes finger-reach relevance as a hard constraint. It must not weaken the accepted full-path collision rules.

## Safety boundary

- command-free offline analysis only;
- no Gazebo or ROS node;
- no object spawn;
- no controller, planning request, trajectory, or command;
- no attachment or physical hardware;
- no production runtime modification;
- no merge requested.
