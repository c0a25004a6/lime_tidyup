# Rubik grasp-relevance constrained fixture search

Phase: `RUBIK-PREGRASP-GRASP-RELEVANCE-CONSTRAINED-FIXTURE-SEARCH`

This phase combines the accepted PR #23 full-path collision evaluator with the exact-STL reach condition established by PR #24.

Implementation order is deliberate:

1. enumerate translations using the same deterministic PR #23 coarse grid;
2. reject candidates unless both finger AABBs have positive X/Z overlap with the cube and a common exact-limit mimic interval exists for the two cube side planes;
3. send only surviving candidates to one reused MoveIt batch evaluator;
4. stop at the first full 2,802-state collision-free candidate;
5. refine locally at `0.25 mm` resolution within `±2 mm` per axis and repeat the same reach and collision gates.

The cube/support rigid transform and all accepted arm/passive-finger states remain unchanged.

A positive result is still only an offline sampled-path preflight. It does not prove continuous-space optimality, continuous-between-sample clearance, exact mesh contact, force closure, grasp success, lifting, transport, placement, or hardware readiness.

No Gazebo, ROS node, controller, command, attachment, physical hardware, or production runtime mutation is permitted in this phase.
