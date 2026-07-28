# Rubik dynamic supported-hold lateral repeatability

## Current boundary candidate

This gate repeats the accepted PR #7 dynamic supported close-hold-release trial in three fresh Gazebo processes. The current conservative passive-alignment candidate changes only the Rubik cube's initial lateral Y position relative to `link7`:

- `-0.00005 m`;
- `0.0 m`;
- `+0.00005 m`.

The canonical `0.057 m`, `0.09 kg` cube, support surface, gripper effort, close step, hold duration, release duration, calibration, and every physical acceptance threshold remain unchanged.

## Rejected larger matrices

### ±0.5 mm

The earlier `-0.0005 / 0.0 / +0.0005 m` candidate was rejected rather than accepted by threshold relaxation.

Both offset trials reached independent left/right contact within bounded alignment travel, but formed edge or tilted contact instead of the accepted 57 mm side hold:

- `-0.5 mm`: bilateral contact required the full `1.5 mm` alignment allowance; calibrated opening reached approximately `59.60 mm`, horizontal motion approximately `21.23 mm`, vertical rise approximately `3.34 mm`, and rotation approximately `0.220 rad`;
- `+0.5 mm`: bilateral contact required `0.5 mm` additional closing; calibrated opening reached approximately `61.75 mm`, horizontal motion approximately `18.06 mm`, vertical rise approximately `3.84 mm`, and rotation approximately `0.304 rad`.

### ±0.25 mm

The subsequent `-0.00025 / 0.0 / +0.00025 m` candidate was also rejected without changing PR #7 thresholds.

- `-0.25 mm`: one finger contacted at `close_step_38`; six additional `0.25 mm` alignment steps consumed the full `1.5 mm` allowance without independent contact from the opposite finger;
- `0.0 mm`: the central control trial passed the original PR #7 conditions;
- `+0.25 mm`: bilateral contact was reached after approximately `1.0 mm` additional closing, but the cube then contacted `link7::link7_collision` and became unstable.

### ±0.10 mm

The `-0.00010 / 0.0 / +0.00010 m` matrix was rejected by workflow run `30295741929`.

- `-0.10 mm`: both fingers were detected at `close_step_39`, but bilateral contact coverage during the two-second hold was only `0.05`; the contact immediately decayed instead of remaining a supported side hold;
- `0.0 mm`: the central control passed with `1.0` support coverage, `1.0` bilateral coverage, approximately `0.369 mm` calibrated opening error, less than `0.01 mm` horizontal movement, and negligible rotation;
- `+0.10 mm`: one-sided contact at `close_step_39` was followed by one `0.25 mm` alignment step. During that step the cube moved approximately `8.71 mm` horizontally and rose approximately `1.82 mm`, exceeding both safety limits before a hold was attempted.

Therefore `±0.10 mm` is outside the currently demonstrated passive-alignment envelope. The failure is asymmetric and cannot be repaired by averaging or by loosening thresholds.

## Bounded unilateral alignment and immediate abort

An offset cube can contact one finger before the other. The initial close sweep still cancels immediately on first contact. If that contact is unilateral, the test may continue closing at the same `0.25` maximum effort and `0.00025 m` step size for at most `0.0015 m` additional travel.

The alignment phase stops as soon as independent left/right contact is observed. During alignment, the trial aborts immediately if any of the following occurs:

- horizontal cube displacement exceeds `0.003 m`;
- upward cube displacement exceeds `0.0015 m`;
- cube rotation exceeds `0.10 rad`;
- the cube contacts any non-finger robot collision;
- a command stalls or fails to reach its target;
- the `1.5 mm` additional travel bound is exhausted.

Calibrated-width disagreement, insufficient support contact, excessive movement during hold, or failed post-release settling remain hard failures. These guards are test-only and do not alter the production controller or accepted physical thresholds.

## Evidence and aggregation

Every trial must independently pass the PR #7 conditions. The matrix fails if any trial lacks support contact, bilateral side contact, calibrated width agreement, bounded movement, side-dominant normals, reopen clearance, or post-release settling.

Each trial starts a fresh container, ROS domain, and Gazebo process and writes a separate evidence directory. Initial gripper joint and link telemetry is awaited for up to ten seconds. Partial failures are preserved as trial records; the aggregate reporter no longer raises a secondary exception when a failed trial has no hold or release result.

## Safety boundary

- fixed arm;
- support never removed;
- no lift command;
- no IFRA attachment;
- no production URDF change;
- no grasp-success claim.

If the `±0.05 mm` matrix fails, the next safe characterization point is `±0.025 mm`. If it passes, `±0.075 mm` may be tested as the upper boundary. No support-removal or lift phase is authorized by this document.
