# Rubik dynamic supported-hold lateral repeatability

## Scope

This gate repeats the accepted PR #7 dynamic supported close-hold-release trial in three fresh Gazebo processes. Only the Rubik cube's initial lateral Y position relative to `link7` changes:

- `-0.0005 m`;
- `0.0 m`;
- `+0.0005 m`.

The canonical `0.057 m`, `0.09 kg` cube, support surface, gripper effort, close step, hold duration, release duration, calibration, and all acceptance thresholds remain unchanged.

## Acceptance

Every trial must independently pass the PR #7 conditions. The matrix fails if any trial lacks support contact, bilateral side contact, calibrated width agreement, bounded movement, side-dominant normals, reopen clearance, or post-release settling.

Each trial starts a fresh container and Gazebo process and writes a separate evidence directory. The aggregate summary records the minimum contact coverage and worst movement, rotation, opening error, and final offset across the matrix.

## Safety boundary

- fixed arm;
- support never removed;
- no lift command;
- no IFRA attachment;
- no production URDF change;
- no grasp-success claim.

This gate measures repeatability only. It does not broaden the accepted manipulation envelope beyond the tested `±0.5 mm` lateral offsets.
