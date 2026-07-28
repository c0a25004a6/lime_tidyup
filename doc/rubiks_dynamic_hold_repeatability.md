# Rubik dynamic supported-hold lateral repeatability

## Current boundary candidate

This gate repeats the accepted PR #7 dynamic supported close-hold-release trial in three fresh Gazebo processes. The current passive-alignment candidate changes only the Rubik cube's initial lateral Y position relative to `link7`:

- `-0.000025 m`;
- `0.0 m`;
- `+0.000025 m`.

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

- `-0.10 mm`: both fingers were detected at `close_step_39`, but bilateral contact coverage during the two-second hold was only `0.05`;
- `0.0 mm`: the central control passed with full support and bilateral coverage, approximately `0.369 mm` calibrated opening error, less than `0.01 mm` horizontal movement, and negligible rotation;
- `+0.10 mm`: one-sided contact followed by one `0.25 mm` alignment step moved the cube approximately `8.71 mm` horizontally and raised it approximately `1.82 mm`, exceeding both safety limits.

### ±0.05 mm

The `-0.00005 / 0.0 / +0.00005 m` matrix was rejected by workflow run `30327497255`.

Both offset trials reported immediate bilateral contact at `close_step_39`, but that contact was not a stable face hold:

- `-0.05 mm`: the transient maximum calibrated opening was approximately `64.42 mm`; during the final half-second the cube reached approximately `0.66 m/s`, `15.56 rad/s`, `36.8 mm` horizontal displacement, `3.69 mm` upward displacement, and `0.342 rad` rotation;
- `0.0 mm`: the central control again passed with `1.0` support and bilateral coverage, approximately `0.359 mm` opening error, about `0.012 mm` maximum horizontal movement, and negligible rotation;
- `+0.05 mm`: the transient maximum calibrated opening was approximately `63.61 mm`; during the final half-second the cube reached approximately `0.58 m/s`, `12.15 rad/s`, `42.3 mm` horizontal displacement, `4.00 mm` upward displacement, and `0.351 rad` rotation.

The offset failures therefore cannot be repaired by replacing the maximum opening with a median or by waiting longer. The cube was dynamically diverging and leaving the supported side-contact state.

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

If the `±0.025 mm` matrix also fails while the center continues to pass, passive lateral tolerance is not practically demonstrated beyond exact centering. The next phase must then design active pose sensing or centering rather than continue shrinking the passive offset indefinitely. No support-removal or lift phase is authorized by this document.
