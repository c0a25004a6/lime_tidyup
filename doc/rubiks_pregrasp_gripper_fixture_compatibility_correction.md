# Rubik gripper fixture compatibility correction

Phase: `RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-CORRECTION`

The accepted PR #27 sweep showed that both fingers reach the support before any cube contact at the PR #26 translation `(0.06325, 0, 0.01225) m`.

Implementation priority for this phase is a bounded local X correction first:

- center: PR #26 translation;
- X step: `0.25 mm`;
- X radius: `±2 mm`;
- 17 candidates, ordered by distance from the accepted center;
- cube/support rigid geometry is preserved by translating both together;
- each candidate reuses the accepted `10 µm` offline gripper closing sweep;
- only candidates reaching sampled dual finger–cube contact before any forbidden support/ground/non-finger/self/bounds condition are retained;
- retained candidates are then rechecked against the accepted 2,802-state passive full path.

A positive result authorizes only static Gazebo no-actuation evidence. If no X-only solution exists, the next implementation expands the same compatibility search into a bounded 3D neighborhood.

No Gazebo, ROS node, controller, command, attachment, physical hardware, or production runtime mutation is used in this phase.
