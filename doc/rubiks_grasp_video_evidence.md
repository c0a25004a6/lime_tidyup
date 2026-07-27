# Rubik cube settle video evidence

## Scope

This phase records the parameterized Rubik cube's pose and velocity from Gazebo Classic `/model_states`, then renders those measured values into a deterministic MP4 with synchronized side and top views.

It intentionally does not:

- move the arm or gripper,
- attach the cube with IFRA LinkAttacher,
- use the absent `libgazebo_grasp_fix.so`,
- write gripper calibration,
- declare grasp success,
- describe the rendered MP4 as raw Gazebo GUI footage.

The purpose is to verify the model, spawn, gravity, collision, settling, telemetry, and video-evidence path before contact calibration is attempted.

## Why telemetry rendering is used

The hosted GitHub runner successfully executed `gzserver`, spawned the cube, and produced valid `/model_states`, but Gazebo Classic's GUI produced splash/black frames under Xvfb. Those frames are rejected rather than accepted as evidence.

The accepted video is generated only from recorded Gazebo telemetry. The JSON trajectory remains the canonical evidence; the MP4 is its human-readable visualization.

## Local execution

Build the current image and run the headless trial:

```bash
bash /root/bin/run_rubiks_cube_settle_video /artifacts
```

Then render the recorded trajectory:

```bash
python3 bin/render_rubiks_trajectory_video.py \
  --trajectory artifacts/rubiks_cube_trajectory.json \
  --parameters project/resource/model_editor_models/rubiks_cube/parameters.json \
  --output artifacts/rubiks_cube_settle_telemetry_evidence.mp4
```

The runner starts the recorder before spawning `rubiks_cube_settle` at `(0.45, 0.0, 0.35)`, records for ten seconds, and writes a manifest that explicitly marks the artifact as telemetry-rendered and not a grasp result.

## Automated evidence

`.github/workflows/rubiks-grasp-video.yml` performs all of the following:

1. compiles and validates the evidence tools;
2. builds the complete Lime Docker image;
3. runs headless Gazebo physics;
4. records `/model_states` at up to 30 Hz;
5. verifies initial height, observed fall, final support height, final speed, and duration;
6. renders an H.264 MP4 from the accepted trajectory;
7. validates MP4 duration and size;
8. uploads the video, raw trajectory, numerical summary, manifest, and logs together.

Expected artifact files include:

- `rubiks_cube_settle_telemetry_evidence.mp4`
- `rubiks_cube_trajectory.json`
- `rubiks_cube_settle.physics_summary.json`
- `rubiks_cube_settle.manifest.json`
- `rubiks_cube_model_validation.json`
- final `/model_states` snapshot
- Gazebo, spawn, recorder, and container logs

## Accepted run

Workflow run `30236495098` passed every gate. Its recorded values were:

- samples: `251`
- duration: `10.0095 s`
- initial cube-center height: `0.3499706 m`
- final cube-center height: `0.0284002 m`
- observed fall: `0.3215704 m`
- expected center height from the 57 mm edge: `0.0285 m`
- final linear speed: `4.45e-10 m/s`

The final height error is approximately `0.10 mm`, and the final speed is effectively zero at the recorded precision.

## Review gate

Human review must confirm:

1. the side view begins with the cube above the support plane;
2. the cube moves downward and reaches the support plane;
3. the top view remains consistent with the recorded horizontal pose and yaw;
4. the cube remains stationary through the end of the progress bar;
5. the video is identified as telemetry visualization, not raw camera footage or proof of grasping.

Passing this gate permits a separate gripper open/close command-and-joint-state video. A physical gauge-contact calibration still requires the actual Lime simulation environment and must remain a later gate. This phase does not permit a grasp trajectory or automatic grasp-success claim.
