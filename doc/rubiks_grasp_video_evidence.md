# Rubik cube settle video evidence

## Scope

This phase records a deterministic Gazebo video of the parameterized Rubik cube being spawned above an isolated ground plane and allowed to settle.

It intentionally does not:

- move the arm or gripper,
- attach the cube with IFRA LinkAttacher,
- use the absent `libgazebo_grasp_fix.so`,
- write gripper calibration,
- declare grasp success.

The purpose is to verify the model, world, display, recording, spawn, gravity, collision, and settle-observation path before contact calibration is attempted.

## Local execution

Build the current image, start an X display, and run:

```bash
/root/bin/run_rubiks_cube_settle_video /artifacts
```

The runner waits for `/artifacts/start_capture`, spawns `rubiks_cube_settle` at `(0.45, 0.0, 0.35)`, records model-state snapshots, observes for ten seconds, and writes a manifest.

## Automated evidence

`.github/workflows/rubiks-grasp-video.yml` builds the complete Lime image, runs Gazebo under Xvfb with software rendering, records the Gazebo window to H.264 MP4, validates video duration, and uploads all logs and evidence as one artifact.

Expected artifact files include:

- `rubiks_cube_settle_visual_evidence.mp4`
- `rubiks_cube_settle.manifest.json`
- `rubiks_cube_model_validation.json`
- initial and final `/gazebo/model_states` snapshots
- Gazebo, container, Xvfb, and ffmpeg logs

## Review gate

Human review must confirm:

1. the cube is visible and not spawned inside the ground or robot;
2. gravity acts in the expected direction;
3. the cube reaches the support surface without explosive motion;
4. the cube does not continue drifting after settling;
5. the video shows the intended Gazebo scene rather than a blank or corrupted frame.

Passing this gate permits a separate static-gauge gripper close/open video. It still does not permit a grasp trajectory or automatic grasp-success claim.
