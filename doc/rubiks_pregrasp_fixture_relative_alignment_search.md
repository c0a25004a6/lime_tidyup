# Rubik pregrasp fixture relative-alignment correction preflight

Phase: `RUBIK-PREGRASP-FIXTURE-RELATIVE-ALIGNMENT-CORRECTION-PREFLIGHT`

Writer lease: `WL-RUBIK-PREGRASP-FIXTURE-ALIGNMENT-SEARCH-20260804-01`

## Purpose

PR #22 proved that transferring the accepted supported-hold fixture pose directly into the passive pregrasp state causes finger contact with both the cube and support in every recorded state.

This phase searches for a rigid translation of the complete cube/support fixture relative to the selected-pose `link7` anchor. The passive measured arm and finger states are not changed.

## Accepted input

The complete PR #22 evidence archive is bound by its accepted archive digest. The following internal files are also verified by known SHA-256 values:

- exact-head URDF and SRDF;
- all `2,802` passive pregrasp states;
- fixture input contract;
- fixture collision matrix;
- fixture blocker summary and manifest.

The baseline translation `(0, 0, 0)` must reproduce the PR #22 blocker at state `0`, with both fingers contacting both fixture objects.

## Transformation contract

Only translation is searched. Rotation is fixed.

For a candidate translation `d` in the selected-pose `link7` frame:

- `cube_offset = accepted_cube_offset + d`;
- `support_offset = accepted_support_offset + d`.

The cube/support relative transform and intended surface contact remain exact. The translated fixture is anchored at selected-pose hold state `1541` and remains fixed in world coordinates while the complete recorded path is replayed.

## Coarse search

- step: `0.002 m`;
- closed Euclidean sphere radius: `0.08 m`;
- candidate count: `267,761`;
- order: squared Euclidean norm, then `abs(z)`, `abs(y)`, `abs(x)`, then signed `z`, `y`, `x`;
- each candidate must clear all `2,802` states;
- evaluation stops at the first fixture contact for a rejected candidate.

The first full-path-clear coarse-grid candidate is retained.

## Local fine search

When a coarse candidate exists:

- step: `0.00025 m`;
- local radius: `0.002 m` on every axis around the coarse candidate;
- candidate count: `4,913`;
- the same deterministic ordering and full-path requirement are used.

The coarse candidate itself is part of the fine grid. Therefore the fine phase must retain at least one full-path-clear candidate.

## Decisions

The completed preflight produces one of:

- `FULL_PATH_CLEAR_TRANSLATION_FOUND`;
- `NO_FULL_PATH_CLEAR_TRANSLATION_WITHIN_COARSE_RADIUS`.

A positive result proves only clearance for the selected translation on the declared coarse grid plus local fine grid and across the recorded sampled path.

It is not a proof of the continuous-space global optimum. It does not establish that the translated fixture remains a useful grasp pose.

## Safety boundary

- offline analysis only;
- accepted passive finger state is unchanged;
- cube/support rigid relationship is unchanged;
- no Gazebo or ROS node;
- no object spawn;
- no controller, planning request, trajectory, or command;
- no attachment or physical hardware;
- no production runtime modification;
- Draft and unmerged.

## Follow-up

When a full-path-clear candidate is found, the next phase is a command-free geometric-relevance audit. It must evaluate whether the translation remains compatible with a plausible grasp arrangement before any Gazebo fixture is created.

When no candidate exists inside the `0.08 m` coarse radius, the next safe phase may evaluate an explicitly modeled active-open gripper state offline. It must not load a controller or send a gripper command.
