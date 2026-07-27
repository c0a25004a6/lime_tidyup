# Rubik gripper command evidence

## Scope

This gate verifies only the ROS 2 command path from `GripperCommand` to the Lime gripper joint states.

The accepted sequence is:

1. open to `0.019 m`;
2. close to `-0.010 m`;
3. open again to `0.019 m`.

The test uses `fake_components/GenericSystem`. It does not start Gazebo, spawn the cube, contact an object, calibrate the finger inner faces, or claim grasp success.

## Why this gate is separate

The controller configuration commands only `gripper_left_joint`, while `gripper_right_joint` is configured as a mimic joint. The gate therefore requires both joint names to appear independently on `/joint_states` and verifies that their observed positions agree.

A missing right joint is not replaced with the left value for acceptance. This prevents a broken mimic path from being hidden by fallback logic.

The measured quantity is still only link-frame separation:

```text
link_frame_separation
  = abs((0.021 + left_q) - (-0.021 - right_q))
```

Expected values for symmetric joint positions are:

- open: approximately `0.080 m`;
- closed: approximately `0.022 m`.

These are not the finger inner-face opening. `inner_face_opening_m` remains `null` until a known-width gauge is contacted in the real Lime simulation environment.

## Evidence files

The CI artifact contains:

- `gripper_command_telemetry.json`: all `/joint_states` samples and action results;
- `gripper_command_summary.json`: numerical acceptance result;
- `gripper_command_evidence.manifest.json`: safety and provenance fields;
- `gripper_command_telemetry_evidence.mp4`: deterministic animation of the two link frames and separation history;
- before/after joint-state snapshots;
- launch and command logs;
- `ffprobe` metadata.

The video shows link-frame markers, not physical finger inner surfaces. The left panel animates the observed link-frame separation; the right panel plots the same value over time. Dashed target markers represent the command target.

## Acceptance gates

The workflow fails unless all of the following hold:

1. all three action goals are accepted and report `reached_goal=true`;
2. no action reports `stalled=true`;
3. both gripper joints are present in every accepted telemetry sample;
4. left and right positions match within `0.5 mm`;
5. each held position reaches its target within `1.5 mm`;
6. minimum link-frame separation is between `20 mm` and `24 mm`;
7. maximum link-frame separation is between `78 mm` and `82 mm`;
8. at least 20 joint-state samples are recorded;
9. the evidence video is at least four seconds long and non-empty;
10. cube contact, inner-face calibration, and grasp success remain false.

## Local execution

Inside the completed Lime image:

```bash
bash /root/bin/run_gripper_command_evidence /artifacts
```

Render the telemetry on a machine with `ffmpeg`:

```bash
python3 bin/render_gripper_command_video.py \
  --telemetry artifacts/gripper_command_telemetry.json \
  --output artifacts/gripper_command_telemetry_evidence.mp4
```

## Next gate

Passing this fake-hardware gate permits a simulation-only no-contact command test using the Gazebo gripper controller, provided the robot sensor/rendering plugins are isolated or disabled so that the previous headless rendering crash cannot affect the result.

It does not yet permit gauge contact or a Rubik-specific grasp trajectory. Gauge calibration must remain a separately reviewed manual test because the actual inner-face contact state cannot be established from joint positions alone.
