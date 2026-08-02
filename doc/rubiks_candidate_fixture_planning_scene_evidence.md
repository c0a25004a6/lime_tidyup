# Rubik candidate-fixture planning-scene evidence

## Authority

```text
lane_id: RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE
writer_lease: WL-RUBIK-CANDIDATE-FIXTURE-PLANNING-SCENE-EVIDENCE-20260803-01
parent_pr: #15
exact_base: f03af5493108efd436142c1b015e8cc94c049e29
authority: command-free MoveIt environment-collision evidence
```

This phase reconstructs the measured supported-fixture state inside MoveIt Core and checks the accepted correction envelope without starting ROS, Gazebo, controllers, planning services, or a trajectory executor.

## Canonical input artifact

The workflow downloads the exact accepted PR #15 artifact and verifies:

- workflow run `30761961570`;
- exact head `f03af5493108efd436142c1b015e8cc94c049e29`;
- artifact ID `8837821223`;
- archive SHA-256 `0f25766bd5e94a10f459538dcbeb293f42af25b780e35ba3a48e270e0b3bcf48`;
- summary, manifest, fixture telemetry, arm telemetry, and search-summary member hashes.

Measured arm state, open-gripper state, fixture transforms, and candidate evidence are read from the verified artifact. They are not copied into the workflow as replacement state values.

## Robot and gripper state

The production URDF and canonical SRDF are loaded directly into MoveIt Core. The robot state uses:

- measured `joint1` through `joint6` immediately before fixture spawn;
- measured `gripper_left_joint` from the successful open command;
- `gripper_right_joint` generated through the canonical URDF mimic relation.

The checker requires both arm and gripper group bounds for every sampled state. The gripper remains fixed and open throughout the correction matrix.

## World-object reconstruction

The planning scene contains exactly two fixed collision objects:

```text
rubiks_support: box 0.075 x 0.040 x 0.010 m
  relative to measured candidate link7: [-0.019, 0.0, 0.0857] m

rubiks_cube: box 0.057 x 0.057 x 0.057 m
  relative to measured candidate link7: [-0.019, 0.0, 0.1192] m
```

MoveIt computes the measured-candidate global `link7` transform. Each relative transform is composed with that frame once, and the resulting world objects remain fixed for all correction states.

The support-cube contact is an intentional world-world relationship. The robot collision query does not add an Allowed Collision Matrix entry for either object. Any robot-object contact remains a failure.

## Correction matrix

The six established correction cases are regenerated from the measured candidate using the PR #10 DLS geometric-Jacobian and nonlinear-FK checks:

- center;
- lateral `-0.5 mm`;
- lateral `+0.5 mm`;
- yaw `-1 degree`;
- yaw `+1 degree`;
- mixed `+0.5 mm / -1 degree`.

The checker evaluates:

- the measured candidate;
- 100 deterministic linear joint-space interpolation samples from the measured candidate to each correction endpoint, including each endpoint.

Total state count:

```text
1 + 6 x 100 = 601 states
```

Each state is checked for:

- arm bounds;
- gripper bounds;
- SRDF-filtered self-collision;
- robot-support collision;
- robot-cube collision;
- complete contact-pair evidence.

## Fail-closed behavior

The phase rejects on:

- PR #15 artifact or member hash mismatch;
- incomplete/non-finite measured arm or gripper state;
- changed fixture dimensions or relative transforms;
- malformed production URDF/SRDF;
- missing arm/gripper semantic group;
- missing required robot collision geometry;
- failed measured-state correction generation;
- arm or gripper bounds violation;
- self-collision;
- robot-support or robot-cube contact;
- any robot-object ACM exemption;
- any command or runtime-motion path.

Rollback is `deterministic_no_op`; no command is created or sent.

## Safety boundary

- offline MoveIt Core only;
- no ROS node, publisher, service client, or action client;
- no controller, planning request, or trajectory message;
- no Gazebo process or command;
- no robot-object ACM exemption;
- no physical-hardware access/readiness claim;
- no dynamics/contact-retention claim;
- no lift, support removal, IFRA attachment, or grasp-success claim;
- no production runtime modification;
- `actuation_authorized=false`.

## Next permissible gate

After exact-head acceptance, `RUBIK-SUPPORTED-CORRECTION-SIMULATION-TRIAL` may repeat the accepted pregrasp setup and supported fixture reconstruction, then send one bounded simulation-only correction trajectory selected from the directly verified envelope while the gripper remains open and the cube remains supported. It must record tracking, contacts, support retention, cube motion, rollback boundary, and must not lift or close the gripper.
