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

It shallow-clones `ROBOTIS-JAPAN-GIT/turtlebot3_lime` only to resolve the exact collision STL resources named by the accepted URDF. Evidence records the clone commit, origin, clean status, and SHA-256/size/resource path for both STL files.

## Geometry contract

The checker uses only the Python standard library. It does not load ROS, Gazebo, MoveIt, FCL, a planner, or a controller.

For each finger link it requires:

- exactly one URDF collision element;
- a `package://turtlebot3_lime_description/...` STL resource;
- scale exactly `(0.001, 0.001, 0.001)`;
- unrotated collision and prismatic-joint origins;
- left axis `+Y` and right axis `-Y`;
- binary or ASCII STL with finite vertices and a consistent triangle count.

The checker reads every triangle vertex, applies the exact URDF scale and transforms, and calculates the collision-mesh AABB envelope in `link7` coordinates at:

- lower gripper limit;
- zero;
- accepted passive position;
- upper gripper limit.

The left AABB translates `+Y` by the requested joint position and the right AABB translates `-Y`; X/Z geometry is unchanged analytically by the declared prismatic model.

A corrected fixture is geometrically relevant only when both conditions hold:

1. both finger AABBs have positive overlap with the cube AABB in X and Z;
2. there is a common mimic-joint interval inside the exact limits where the positive and negative cube side planes are simultaneously contained by the corresponding finger AABBs.

No clearance tolerance is introduced to manufacture a positive result.

## Decisions

The completed evidence records one of:

- `CORRECTED_FIXTURE_GEOMETRICALLY_RELEVANT`; or
- `BLOCKED_CORRECTED_FIXTURE_OUTSIDE_FINGER_REACH_ENVELOPE`.

A positive result is only a necessary AABB-envelope condition. It does not prove exact mesh surface contact, contact normals, force closure, friction, support/ground clearance during gripper motion, grasp success, lifting, transport, placement, or hardware readiness.

## Next safe phase

When relevant, the next authorized phase is an offline gripper-joint sweep over the common interval, checking exact robot/cube/support/ground collision states without sending a command.

When blocked, the next authorized phase is a new offline fixture search that includes finger-reach relevance as a hard constraint. It must not weaken the accepted full-path collision rules.

## Safety boundary

- command-free standard-library offline analysis only;
- no ROS, Gazebo, MoveIt, FCL, or hardware interface;
- no object spawn;
- no controller, planning request, trajectory, or command;
- no attachment or physical hardware;
- no production runtime modification;
- no merge requested.
