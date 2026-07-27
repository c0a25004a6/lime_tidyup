# Rubik static cube side-contact gate

## Purpose

This gate checks whether the simulation calibration accepted in PR #5 correctly predicts bilateral contact against the canonical `0.057 m` Rubik cube collision shape.

It deliberately stops before dynamic retention, arm motion, or lifting.

## Test fixture

The test model copies the canonical cube collision and visible face dimensions:

- collision: `0.057 x 0.057 x 0.057 m`;
- friction coefficients: `1.0 / 1.0`;
- restitution: `0.0`.

The test-only model is `static=true`. This is intentional: the gate isolates collision geometry and calibrated finger opening from cube mass, support friction, pushing, and lift dynamics.

The canonical dynamic cube remains unchanged and is not replaced.

## Calibration input

The gate consumes the accepted PR #5 values:

- combined inner-face offset: `0.003000000004097683 m`;
- estimated calibration uncertainty: `0.0003279557798182653 m`.

For every physical link-frame observation:

```text
inner_face_opening_m
  = link_frame_separation_m - combined_inner_face_offset_m
```

At bilateral cube contact, the calibrated inner-face opening must match `0.057 m` within:

```text
max(0.001 m, 3 * calibration_uncertainty_m)
```

The current acceptance tolerance is therefore `0.001 m`.

## Procedure

1. Start the isolated Lime Gazebo fixture used by the preceding gates.
2. Open the gripper to `0.019 m` joint position.
3. Spawn the stationary cube relative to `link7` at the previously calibrated contact pose.
4. Require no initial finger contact and no collision with non-finger robot links.
5. Close in `0.00025 m` joint-command increments at bounded effort `0.25`.
6. Cancel the active goal as soon as contact is detected.
7. Require independently reported left and right finger contact.
8. Compare the calibrated physical opening against the 57 mm cube width.
9. Reopen to `0.019 m` and require contact to clear.
10. Verify that the static cube pose did not change.

## Evidence sources

- `/rubiks_cube_contact/contact_states`;
- `/joint_states`;
- `/link_states`;
- `/model_states`;
- `GripperCommand` action results;
- deterministic calibrated telemetry MP4.

The MP4 is not raw Gazebo GUI footage. It renders the observed physical link frames, calibrated inner faces, the 57 mm cube width, command targets, and contact phases.

## Safety boundary

A successful result establishes only that the calibrated simulated finger geometry can contact both sides of the stationary canonical cube collision.

It does not establish:

- dynamic cube retention;
- the effect of the configured `0.09 kg` cube mass;
- support or table friction behavior;
- resistance to lateral disturbance;
- arm motion compatibility;
- lifting capability;
- IFRA attachment validity;
- grasp success.

The following must remain false:

- `cube_mass_dynamics_tested`;
- `arm_motion_performed`;
- `lift_command_sent`;
- `cube_lifted`;
- `ifra_attachment_used`;
- `grasp_success_claimed`.

## Next permissible gate

After this gate passes, the next safe phase is a dynamic `0.09 kg` cube resting on an explicit support surface. The gripper may close and hold for a fixed period, but the arm must remain fixed and the cube must not be lifted. That later gate must measure cube translation, rotation, support contact, and bilateral finger contact without attachment assistance.
