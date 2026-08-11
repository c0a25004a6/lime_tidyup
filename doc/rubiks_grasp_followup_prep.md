# Rubik grasp follow-up preparation

Phase: `RUBIK-GRASP-FOLLOWUP-PREP`

This stacked preparation consumes the final summary from PR #28
`RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-CORRECTION` and routes the next
step without authorizing actuation.

## Positive PR #28 result

When PR #28 reports
`GRIPPER_FIXTURE_COMPATIBLE_FULL_PATH_CLEAR_TRANSLATION_FOUND`, the router
requires:

- exact-head-bound PR #28 evidence;
- the complete 17-candidate X scan;
- a selected translation that passed all 2,802 passive path states;
- an open-clear gripper prefix;
- sampled dual finger-cube contact;
- no forbidden collision before dual contact;
- all PR #28 safety flags to remain false.

The next phase is then fixed to:

`RUBIK-PREGRASP-CORRECTED-FIXTURE-STATIC-GAZEBO-NO-ACTUATION-EVIDENCE`

That next phase may start Gazebo and observe ROS state, but this preparation
explicitly does **not** authorize an arm command, gripper command, lift,
attachment, physical hardware access, or production runtime mutation.

## Negative PR #28 result

When PR #28 reports either X-only negative decision, the router emits the
remaining local 3-D translation grid around `(0.06325, 0, 0.01225) m`:

- step: `0.25 mm` per axis;
- radius: `±2 mm` per axis;
- full local cube: `17^3 = 4,913` points;
- the already-tested 17-point X line is excluded;
- remaining candidates: `4,896`;
- deterministic distance-first ordering;
- no Gazebo, ROS, controller, command, lift, attachment, hardware, or runtime
  modification is authorized.

This only prepares the input for
`RUBIK-PREGRASP-GRIPPER-FIXTURE-COMPATIBILITY-3D-EXPANSION`; it does not claim
that all 4,896 points should be brute-force evaluated. The execution phase may
add evidence-preserving geometric prefilters or shell-wise early stopping as
long as the declared search domain and rejection accounting remain complete.

## Static verification

Run:

```bash
python3 -m py_compile bin/prepare_rubiks_grasp_followup.py
python3 bin/prepare_rubiks_grasp_followup.py --self-test
```

The self-test covers positive routing, negative routing, exact remaining-grid
coverage, exclusion of the prior X line, and rejection of a summary that claims
a command was sent.
