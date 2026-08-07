# Rubik corrected fixture offline gripper sweep

Phase: `RUBIK-PREGRASP-CORRECTED-FIXTURE-OFFLINE-GRIPPER-SWEEP-COLLISION-PREFLIGHT`

Input fixture translation is the PR #26 selected `link7` translation `(0.06325, 0, 0.01225) m`.

At the accepted anchor arm state, the MoveIt mimic gripper coordinate is swept from the exact upper limit `0.019 m` toward the exact lower limit `-0.01 m` in `0.00001 m` increments. No controller or trajectory is instantiated.

At every sampled gripper coordinate the audit separately checks:

- MoveIt bounds;
- canonical self-collision;
- robot–cube contacts;
- robot–support contacts;
- gripper-link–ground contacts.

Only `gripper_left_link|rubiks_cube` and `gripper_right_link|rubiks_cube` are classified as intended cube-contact evidence. Support contact, non-finger robot–cube contact, self-collision, bounds failure, or finger–ground contact is forbidden.

A positive result requires at least one collision-free open state followed, while closing, by a sampled dual-finger cube-contact state with no forbidden state before that dual contact.

The result does not prove continuous clearance, exact surface touch, contact normals, force closure, friction, grasp success, lifting, transport, placement, or hardware readiness.

This phase is offline only: no Gazebo, ROS node, object spawn, controller, planning request, trajectory, command, attachment, physical hardware, or production runtime mutation.
