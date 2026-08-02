# Rubik command-free pregrasp state alignment preflight

## Authority

```text
lane_id: RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT
writer_lease: WL-RUBIK-PREGRASP-STATE-ALIGNMENT-PREFLIGHT-20260803-01
parent_pr: #12
exact_base: 43ab5ab12dea3758d9e49b69b02ec13d2e23f6c9
authority: documentation/test-design/evidence-only
```

This corrective phase resolves the state mismatch established by PR #12. It searches for a nearby nonsingular arm posture before any cube/support fixture is created. It does not move the robot, create a world scene, or authorize actuation.

## Evidence-bound starting state

The configured supported-hold state remains:

```text
joint1..joint6 = 0.0 rad
```

That state has a rank-deficient Jacobian and cannot generate the accepted lateral correction. The search must begin from this state and may not substitute the more distant PR #10 seed.

## Deterministic structured search

The exact production URDF and installed controller configuration are parsed using the same effective-limit policy as PR #10: the intersection of URDF and rendered `ros2_control` position-command limits, with a one-degree margin.

The search exhaustively evaluates this declared subspace:

- varied joints: `joint1`, `joint2`, `joint3`, `joint5`;
- fixed joints: `joint4=0`, `joint6=0`;
- range: `-15 degrees` through `+15 degrees`;
- step: `2.5 degrees`;
- total candidates: `13^4 = 28,561`.

A candidate is retained only when:

1. it satisfies effective position limits with margin;
2. its geometric Jacobian is finite and has condition number at most `250`;
3. all six accepted correction cases pass the PR #10 finite-output, effective-limit, five-degree maximum-step, nonlinear forward-kinematics residual, and unwanted-translation checks.

Selection is deterministic and ordered by:

1. minimum maximum absolute joint displacement from zero;
2. minimum joint-space norm;
3. minimum condition number;
4. minimum `link7` translation;
5. minimum `link7` orientation change;
6. lexicographic joint vector.

This is exhaustive only within the declared structured subspace. It does not claim a globally nearest posture over the full continuous joint space.

## Self-collision matrix

The selected candidate is checked with offline MoveIt Core using the exact production URDF, canonical SRDF, and SRDF allowed-collision matrix.

The state matrix contains:

- the configured all-zero state;
- 100 deterministic linear joint-space samples from zero to the selected candidate, including the candidate at sample 100;
- all six correction endpoints generated from the candidate.

Every state must:

- satisfy MoveIt arm-group bounds;
- report `self_collision=false`;
- report zero collision contact pairs.

The generic collision checker remains backward compatible with the seven-state PR #11 matrix while accepting this larger state matrix.

## Candidate meaning

A passing candidate is only a **pre-fixture posture**. It does not prove that the accepted cube/support scene can be moved with the arm. A later simulation phase must first move the empty robot to the candidate, record measured `joint1` through `joint6`, and only then spawn the cube and support relative to the resulting `link7` pose.

The cube and support are not added in this phase. Environment collision remains unknown and its verified lateral/yaw envelope remains zero.

## Fail-closed behavior

The phase rejects on:

- a configured state other than exact all-zero;
- missing effective limits or controller evidence;
- no finite-condition candidate in the declared grid;
- no candidate satisfying every correction case;
- malformed or duplicate state labels;
- any MoveIt bounds violation;
- any self-collision or contact pair;
- missing required collision geometry;
- unapplied SRDF allowed-collision evidence;
- any environment object or environment-clearance claim;
- any command or runtime path.

Rollback remains `deterministic_no_op`.

## Safety boundary

- offline Python search and MoveIt Core collision checking only;
- no ROS node, service client, publisher, or action client;
- no controller or planning request;
- no trajectory message or execution;
- no Gazebo process or motion;
- no arm or gripper command;
- no cube/support world object;
- no lift, support removal, or IFRA attachment;
- no production runtime modification;
- no global-nearest claim;
- `actuation_authorized=false`.

## Next permissible gate

After exact-head acceptance, the next safe phase is `RUBIK-PREGRASP-SIMULATION-STATE-EVIDENCE`.

That phase may use Gazebo and the arm trajectory controller only in simulation to move an empty scene from the configured zero state to the accepted candidate. It must use bounded timing, record commanded and measured six-joint trajectories, verify tracking and settling, preserve a deterministic rollback to zero or no-op on failure, and must not spawn the cube/support until the candidate state has been measured and accepted. It does not authorize physical hardware.
