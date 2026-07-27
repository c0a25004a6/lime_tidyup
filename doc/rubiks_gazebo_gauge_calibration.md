# Rubik static-gauge gripper calibration

## Purpose

This gate calibrates the relationship between the Lime gripper link-frame separation and the physical opening between the two inner finger faces.

It runs only after the no-contact Gazebo controller gate has passed. It does not introduce a Rubik grasp trajectory.

## Fixture

The fixture is a static Gazebo box with a known width of `0.057 m` along the gripper closing axis.

Its other dimensions are deliberately smaller:

- depth: `0.012 m`
- width: `0.057 m`
- height: `0.020 m`

The model is spawned relative to `turtlebot3_lime_gripper_test::link7` at:

- `x = -0.019 m`
- `y = 0.000 m`
- `z = 0.1007 m`

The fully open gripper must remain contact-free after the fixture is spawned. Initial overlap is a hard failure and must not be interpreted as calibration.

## Contact evidence

The gauge collision owns a Gazebo Classic contact sensor using `libgazebo_ros_bumper.so`. The evidence recorder subscribes to:

- `/rubiks_gauge/contact_states`
- `/joint_states`
- `/link_states`
- `/model_states`

A contact message is accepted only when it contains the gauge collision and a physical gripper collision. Both `gripper_left_link` and `gripper_right_link` must be independently observed.

A one-sided contact does not calibrate the opening.

## Closing procedure

1. Open to `0.019 m` before spawning the gauge.
2. Confirm that the gauge exists and no open-state contact occurs.
3. Close in `0.00025 m` command steps beginning at `0.0185 m`.
4. Use a bounded command effort of `0.25`.
5. Cancel the active goal as soon as gauge contact is observed.
6. Collect a short dual-contact window.
7. Reopen to `0.019 m` and require contact to clear.

This stepped procedure is intended to reduce impact and solver penetration. It is not a force-control implementation.

## Calibration calculation

The accepted calibration separation is the maximum physical left/right link-frame separation observed in a message containing simultaneous left and right gauge contact.

```text
combined_inner_face_offset_m
  = calibration_link_frame_separation_m - gauge_width_m

inner_face_opening_m
  = link_frame_separation_m - combined_inner_face_offset_m
```

The symmetric per-finger offset is also reported as half of the combined offset. That value is valid only for this mirrored Lime gripper geometry; the combined offset is the primary recorded result.

The artifact records the dual-contact separation range, maximum contact depth, and an estimated uncertainty. CI rejects uncertainty above `0.003 m`.

## Evidence boundary

A successful result establishes only a simulation calibration for the current Lime collision geometry and Gazebo setup.

It does not establish:

- physical hardware calibration;
- Rubik cube contact;
- grasp stability;
- lifting capability;
- IFRA attachment validity;
- automatic grasp-success classification.

The following remain fixed in every accepted artifact:

- `cube_spawned=false`
- `cube_contact=false`
- `ifra_attachment_used=false`
- `grasp_success_claimed=false`

## Next gate

After this calibration is accepted, the next permissible gate is a stationary `0.057 m` cube-side contact trial at the already calibrated opening. It must stop before lifting and must separate pure Gazebo collision evidence from any later attachment-assisted experiment.
