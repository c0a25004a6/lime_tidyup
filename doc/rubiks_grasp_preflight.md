# Rubik's Cube grasp preflight

## Decision after re-evaluation

The first implementation phase is intentionally limited to evidence and calibration. It does not change the existing behavior tree, arm trajectory, gripper command, or grasp-success criteria.

The phase contains three gates:

1. Confirm whether `libgazebo_grasp_fix.so` exists in the built container and whether Gazebo can load it.
2. Measure the gripper's real inner-face opening instead of treating joint-link frame separation as jaw opening.
3. Parameterize cube edge length and mass and generate a deterministic single-rigid-body Gazebo model.

Static repository inspection indicates that the robot Xacro requests `libgazebo_grasp_fix.so`, while the `gazebo-pkgs` dependency and build section are commented out. A clean image must therefore be treated as **plugin absent until the runtime probe proves otherwise**. `IFRA_LinkAttacher` is present for transport/integration tests, but an attachment made by that service must not be counted as a physics grasp success.

## Files

- `bin/rubiks_grasp_preflight.py`: dependency-free diagnostics, opening calibration, model generation, and validation.
- `project/resource/model_editor_models/rubiks_cube/parameters.json`: canonical model and gripper calibration parameters.
- `project/resource/model_editor_models/rubiks_cube/model.sdf`: generated single-rigid-body collision model with six colored face visuals.
- `project/resource/model_editor_models/rubiks_cube/model.config`: Gazebo model metadata.

The default cube values are a simulation baseline, not a claim about every commercial cube:

- edge length: `0.057 m`
- mass: `0.09 kg`
- ODE friction: `mu = 1.0`, `mu2 = 1.0`

## 1. Run dependency-free checks

From the repository root:

```bash
python3 bin/rubiks_grasp_preflight.py self-test
python3 bin/rubiks_grasp_preflight.py validate-model
```

Regenerate after changing `parameters.json`:

```bash
python3 bin/rubiks_grasp_preflight.py generate-model
python3 bin/rubiks_grasp_preflight.py validate-model
```

The generator computes solid-cube diagonal inertia as `m * s^2 / 6`; mass, collision dimensions, inertia, collision count, and face-visual count are validated.

## 2. Confirm Gazebo plugins inside the built container

Run after sourcing the ROS and workspace setup files:

```bash
python3 /root/bin/rubiks_grasp_preflight.py \
  probe-plugin \
  --runtime-ros \
  --report /tmp/rubiks_grasp_plugins.report.json
```

Interpretation:

- exit `0`: `libgazebo_grasp_fix.so` was found.
- exit `3`: the required contact-assist plugin was not found.
- `plugins.libgazebo_link_attacher.so.found=true`: transport-only attach support exists; this does not prove physical grasping.
- `runtime_ros.matching_services`: records attach/detach and entity-state services visible on the current ROS graph.

The probe searches `GAZEBO_PLUGIN_PATH`, workspace install/build directories, the ROS Humble prefix, common system library directories, `ldconfig`, and the dynamic linker.

Do not add or silently vendor the GPLv3 grasp plugin merely to turn this check green. Plugin adoption and license policy are a separate decision. Physics-only grasp evaluation remains valid when the plugin is absent.

## 3. Measure gripper opening

The current gripper geometry gives a **link-frame separation**, not a measured inner-face opening. With symmetric joint position `q`, the configured limits imply:

- `q = -0.010 m` -> frame separation `0.022 m`
- `q = 0.019 m` -> frame separation `0.080 m`

Check the kinematic value without ROS:

```bash
python3 bin/rubiks_grasp_preflight.py opening --joint-position-m 0.019
```

Until calibration, the output deliberately reports:

```json
{
  "inner_face_opening_m": null,
  "mesh_calibrated": false
}
```

### Gauge-based calibration

1. Insert a rigid gauge with a known width between the fingers. The cube model can serve as a `0.057 m` gauge only when it is axis-aligned and its collision size has not been changed.
2. Close the fingers slowly until both inner faces are in stable contact. Do not use IFRA LinkAttacher for this measurement.
3. Read the live joint state and calculate the inner-face offset:

```bash
python3 /root/bin/rubiks_grasp_preflight.py opening \
  --subscribe \
  --reference-width-m 0.057
```

4. Review the reported offset. Write it to the active `parameters.json` only after the contact state is verified:

```bash
python3 /root/bin/rubiks_grasp_preflight.py opening \
  --subscribe \
  --reference-width-m 0.057 \
  --write-calibration
```

The calculation is:

```text
inner_face_offset_per_finger
  = (link_frame_separation - reference_width) / 2

actual_inner_opening
  = link_frame_separation - 2 * inner_face_offset_per_finger
```

A calibration written under `/root/.gazebo/models` is local to that container. Copy the reviewed value back to the repository parameter file before rebuilding.

## Exit codes

- `0`: requested check passed.
- `2`: invalid configuration, missing file, unavailable ROS dependency, or invalid calibration input.
- `3`: `libgazebo_grasp_fix.so` was not found by `probe-plugin`.
- `4`: generated or checked-in SDF failed validation.

## Gate before implementing grasp motion

Do not add a Rubik-specific grasp trajectory until all of the following are recorded:

1. Plugin probe report from the actual Lime simulation container.
2. Calibrated maximum inner-face opening and at least `2 mm` clearance over the configured cube edge for an axis-aligned pre-grasp.
3. Passing model generation and validation.
4. A spawn-and-settle observation showing that the cube does not begin embedded in the support surface and does not move without contact.
5. Separate labels for physics-only, grasp-fix-assisted, and IFRA-attached trials.

The next phase may then add only a ground-truth-pose, axis-aligned pre-grasp/lift smoke test. Vision, learned policies, and general-object grasp planning remain out of scope until that baseline is stable.
