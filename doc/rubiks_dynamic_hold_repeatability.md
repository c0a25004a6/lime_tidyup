# Rubik dynamic supported-hold lateral repeatability

## Final conclusion

Passive lateral tolerance is not demonstrated symmetrically beyond exact centering under the current support, collision, effort, timing, and controller conditions.

The final characterization matrix used initial cube Y positions relative to `link7` of:

- `-0.000025 m`;
- `0.0 m`;
- `+0.000025 m`.

Workflow run `30328428539` completed the full Docker build and all three physical trials. The aggregate gate failed because the positive offset trial exceeded the unchanged calibrated-width tolerance and became dynamically unstable during the two-second contact hold.

### `-0.025 mm`

This trial passed all PR #7 conditions:

- calibrated opening error: approximately `-0.367 mm`;
- support-contact coverage: `0.995`;
- bilateral-finger-contact coverage: `1.0`;
- maximum horizontal movement: approximately `0.024 mm`;
- maximum vertical rise: approximately `0.006 mm`;
- maximum rotation: approximately `0.000690 rad`;
- post-release settling and support contact: passed.

### `0.0 mm`

The central control again passed:

- calibrated opening error: approximately `-0.369 mm`;
- support-contact coverage: `1.0`;
- bilateral-finger-contact coverage: `1.0`;
- maximum horizontal movement: approximately `0.009 mm`;
- maximum rotation: approximately `0.000279 rad`;
- post-release settling and support contact: passed.

### `+0.025 mm`

The trial reached bilateral contact at `close_step_39`, but the transient maximum calibrated opening error was approximately `+1.255 mm`, exceeding the unchanged `1.0 mm` tolerance. The full recorded hold interval also showed actual instability:

- maximum horizontal displacement: approximately `13.26 mm`;
- maximum upward displacement: approximately `3.24 mm`;
- maximum rotation: approximately `0.248 rad`;
- maximum linear speed: approximately `0.419 m/s`;
- maximum angular speed: approximately `9.42 rad/s`.

The stable portion of the contact data remained near `56.65 mm`, but the cube subsequently diverged. Therefore replacing the maximum with a median would hide a real dynamic failure and is not accepted.

## Rejected larger matrices

### ±0.5 mm

Both offset trials reached independent left/right contact within bounded alignment travel, but formed edge or tilted contact instead of the accepted 57 mm side hold:

- `-0.5 mm`: bilateral contact required the full `1.5 mm` alignment allowance; calibrated opening reached approximately `59.60 mm`, horizontal motion approximately `21.23 mm`, vertical rise approximately `3.34 mm`, and rotation approximately `0.220 rad`;
- `+0.5 mm`: bilateral contact required `0.5 mm` additional closing; calibrated opening reached approximately `61.75 mm`, horizontal motion approximately `18.06 mm`, vertical rise approximately `3.84 mm`, and rotation approximately `0.304 rad`.

### ±0.25 mm

- `-0.25 mm`: one finger contacted at `close_step_38`; six additional `0.25 mm` alignment steps consumed the full `1.5 mm` allowance without independent contact from the opposite finger;
- `0.0 mm`: the central control passed the original PR #7 conditions;
- `+0.25 mm`: bilateral contact was reached after approximately `1.0 mm` additional closing, but the cube then contacted `link7::link7_collision` and became unstable.

### ±0.10 mm

Workflow run `30295741929` rejected the matrix:

- `-0.10 mm`: bilateral contact coverage during the hold was only `0.05`;
- `0.0 mm`: passed;
- `+0.10 mm`: approximately `8.71 mm` horizontal movement and `1.82 mm` rise during the first alignment step.

### ±0.05 mm

Workflow run `30327497255` rejected the matrix:

- `-0.05 mm`: approximately `0.66 m/s`, `15.56 rad/s`, `36.8 mm` horizontal movement, `3.69 mm` rise, and `0.342 rad` rotation;
- `0.0 mm`: passed;
- `+0.05 mm`: approximately `0.58 m/s`, `12.15 rad/s`, `42.3 mm` horizontal movement, `4.00 mm` rise, and `0.351 rad` rotation.

## Test safeguards

An offset cube can contact one finger before the other. The initial close sweep cancels immediately on first contact. If contact is unilateral, the test may continue at the same `0.25` maximum effort and `0.00025 m` step size for at most `0.0015 m` additional travel.

Alignment aborts immediately if:

- horizontal displacement exceeds `0.003 m`;
- upward displacement exceeds `0.0015 m`;
- rotation exceeds `0.10 rad`;
- a non-finger robot collision is contacted;
- a command stalls or fails;
- the `1.5 mm` travel bound is exhausted.

The aggregate reporter preserves partial trial evidence without raising a secondary exception. Each trial uses a fresh container, ROS domain, and Gazebo process. Initial joint/link telemetry is awaited for up to ten seconds.

## Safety boundary

- fixed arm;
- support never removed;
- no lift command;
- no IFRA attachment;
- no production URDF change;
- no grasp-success claim.

## Required next phase

Further passive offset shrinking is not meaningful. The next safe phase is an active-centering observation dry-run:

1. observe the cube and `link7` poses in Gazebo;
2. transform the cube pose into the `link7` coordinate system;
3. calculate bounded lateral and yaw correction proposals;
4. validate sign, magnitude, saturation, and rejection behavior across a pose matrix;
5. send no arm, gripper, attachment, or lift command.

Gazebo ground truth is permitted only as a simulation oracle for this design gate. It is not evidence that a real camera or production pose estimator is available.
