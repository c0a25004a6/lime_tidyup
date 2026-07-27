# Rubik dynamic supported close-hold-release evidence

## Scope

This gate uses a test-only dynamic copy of the canonical Rubik model with the same `0.057 m` collision, `0.09 kg` mass, and diagonal inertia `4.8735e-05 kg m^2`. The cube rests on an explicit static support surface while the Lime arm remains fixed.

The gripper opens, allows the cube to settle, closes in bounded `0.00025 m` steps, cancels on first finger contact, holds the resulting position for `2.0 s`, reopens, and observes post-release settling for `1.5 s`.

## Required evidence

- support contact before the close sweep;
- no finger contact while fully open;
- independent left and right side contact;
- calibrated inner-face opening within `1 mm` of `0.057 m`;
- support-contact coverage of at least `75%` during the hold;
- bilateral-finger-contact coverage of at least `65%` during the hold;
- horizontal cube motion no greater than `3 mm` during the hold;
- upward cube motion no greater than `1.5 mm` during the hold;
- cube rotation no greater than `0.10 rad` during the hold;
- cleared finger contact after reopening;
- support contact restored and final cube speed below `0.02 m/s`;
- bilateral contact normals remain side-dominant relative to the moving cube center.

## Safety boundary

This is not a lift test. The arm receives no motion command, the support is never removed, IFRA attachment is not loaded, and no grasp-success claim is made. The production URDF and canonical cube model remain unchanged.

## Evidence interpretation

The generated MP4 is a deterministic rendering of calibrated finger opening and contact phases. Dynamic cube displacement, rotation, support-contact coverage, and post-release settling remain authoritative in the JSON telemetry and summary. It is not raw Gazebo GUI footage.
