# Rubik dynamic supported-hold lateral repeatability

## Final boundary candidate

This gate repeats the accepted PR #7 dynamic supported close-hold-release trial in three fresh Gazebo processes. The final passive-alignment boundary candidate changes only the Rubik cube's initial lateral Y position relative to `link7`:

- `-0.00010 m`;
- `0.0 m`;
- `+0.00010 m`.

The canonical `0.057 m`, `0.09 kg` cube, support surface, gripper effort, close step, hold duration, release duration, calibration, and all acceptance thresholds remain unchanged.

## Rejected ±0.5 mm matrix

The earlier `-0.0005 / 0.0 / +0.0005 m` candidate was rejected rather than accepted by threshold relaxation.

Both offset trials reached independent left/right contact within the bounded alignment travel, but did not form the accepted 57 mm side hold:

- `-0.5 mm`: bilateral contact required the full `1.5 mm` alignment allowance; the calibrated opening reached approximately `59.60 mm`, horizontal motion reached approximately `21.23 mm`, vertical rise reached approximately `3.34 mm`, and rotation reached approximately `0.220 rad`;
- `+0.5 mm`: bilateral contact required `0.5 mm` additional alignment; the calibrated opening reached approximately `61.75 mm`, horizontal motion reached approximately `18.06 mm`, vertical rise reached approximately `3.84 mm`, and rotation reached approximately `0.304 rad`.

The evidence indicates edge or tilted contact and support-climbing motion. Therefore `±0.5 mm` is outside the currently demonstrated passive-alignment envelope.

## Rejected ±0.25 mm matrix

The subsequent `-0.00025 / 0.0 / +0.00025 m` candidate was also rejected without changing the PR #7 thresholds.

- `-0.25 mm`: one finger contacted at `close_step_38`; six additional `0.25 mm` alignment steps consumed the full `1.5 mm` allowance without independent contact from the opposite finger;
- `0.0 mm`: the central control trial passed the original PR #7 conditions, including calibrated opening, support coverage, bilateral hold, reopen clearance, and post-release settling;
- `+0.25 mm`: bilateral contact was reached after approximately `1.0 mm` additional closing, but the cube subsequently contacted `link7::link7_collision`, lost the accepted side-hold geometry, and became dynamically unstable.

Therefore `±0.25 mm` is also outside the currently demonstrated passive-alignment envelope.

## Bounded unilateral alignment and immediate abort

An offset cube naturally contacts one finger before the other. The initial close sweep still cancels immediately on first contact. If that contact is unilateral, the test may continue closing at the same `0.25` maximum effort and `0.00025 m` step size for at most `0.0015 m` additional travel.

The alignment phase stops as soon as independent left/right contact is observed. During alignment, the trial now aborts immediately if any of the following occurs:

- horizontal cube displacement exceeds `0.003 m`;
- upward cube displacement exceeds `0.0015 m`;
- cube rotation exceeds `0.10 rad`;
- the cube contacts any non-finger robot collision;
- a command stalls or fails to reach its target;
- the `1.5 mm` additional travel bound is exhausted.

Calibrated-width disagreement, insufficient support contact, excessive movement during hold, or failed post-release settling remain hard failures. These guards are test-only and do not alter the production controller or accepted physical thresholds.

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

The `±0.10 mm` matrix is the final passive-offset boundary test for this phase. If it fails, further passive narrowing is not considered a meaningful next phase; active pose sensing or centering must be designed before any broader manipulation or lift experiment.
