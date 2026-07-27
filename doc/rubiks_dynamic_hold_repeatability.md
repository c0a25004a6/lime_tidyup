# Rubik dynamic supported-hold lateral repeatability

## Current scope

This gate repeats the accepted PR #7 dynamic supported close-hold-release trial in three fresh Gazebo processes. The current candidate matrix changes only the Rubik cube's initial lateral Y position relative to `link7`:

- `-0.00025 m`;
- `0.0 m`;
- `+0.00025 m`.

The canonical `0.057 m`, `0.09 kg` cube, support surface, gripper effort, close step, hold duration, release duration, calibration, and all acceptance thresholds remain unchanged.

## Rejected ±0.5 mm matrix

The earlier `-0.0005 / 0.0 / +0.0005 m` candidate was rejected rather than accepted by threshold relaxation.

Both offset trials reached independent left/right contact within the bounded alignment travel, but did not form the accepted 57 mm side hold:

- `-0.5 mm`: bilateral contact required the full `1.5 mm` alignment allowance; the calibrated opening reached approximately `59.60 mm`, horizontal motion reached approximately `21.23 mm`, vertical rise reached approximately `3.34 mm`, and rotation reached approximately `0.220 rad`;
- `+0.5 mm`: bilateral contact required `0.5 mm` additional alignment; the calibrated opening reached approximately `61.75 mm`, horizontal motion reached approximately `18.06 mm`, vertical rise reached approximately `3.84 mm`, and rotation reached approximately `0.304 rad`.

The evidence indicates edge or tilted contact and support-climbing motion. Therefore `±0.5 mm` is outside the currently demonstrated passive-alignment envelope.

## Bounded unilateral alignment

An offset cube naturally contacts one finger before the other. The initial close sweep still cancels immediately on first contact. If that contact is unilateral, the test may continue closing at the same `0.25` maximum effort and `0.00025 m` step size for at most `0.0015 m` additional travel.

The alignment phase stops as soon as independent left/right contact is observed. A stall, an unreached alignment step, exhaustion of the `1.5 mm` travel bound, calibrated-width disagreement, excessive movement, support loss, or excessive rotation is a hard failure. The bound is test-only and does not alter the production controller or accepted physical thresholds.

## Acceptance

Every trial must independently pass the PR #7 conditions. The matrix fails if any trial lacks support contact, bilateral side contact, calibrated width agreement, bounded movement, side-dominant normals, reopen clearance, or post-release settling.

Each trial starts a fresh container and Gazebo process and writes a separate evidence directory. Initial gripper telemetry is awaited deterministically for up to five seconds. The aggregate summary records the minimum contact coverage and worst movement, rotation, opening error, and final offset across the matrix.

## Safety boundary

- fixed arm;
- support never removed;
- no lift command;
- no IFRA attachment;
- no production URDF change;
- no grasp-success claim.

This gate measures repeatability only. Passing the `±0.25 mm` matrix would not authorize `±0.5 mm`, support removal, or lifting.
