# Rubik pregrasp grasp-relevance constrained fixture search

## Purpose

PR #23 found the nearest declared-grid full-path-clear rigid fixture translation, `(0, 0, +0.04575) m` in the selected-pose `link7` frame. PR #24 then proved that this translation moves the cube about `0.750015 mm` above both finger collision-mesh AABB envelopes.

This phase repeats the translation-only search with the PR #24 finger-reach necessary condition as a hard candidate filter. It does not weaken any accepted collision rule.

## Accepted evidence

The workflow digest-binds the exact PR #24 artifact at source `7d8b9b055cd331d11b89ba275a1ecce7312b4b84`, including:

- PR #24 decision, mesh provenance, exact STL-derived AABBs, and contract;
- PR #23 accepted URDF/SRDF;
- all 2,802 accepted recorded arm/passive-finger states;
- the original cube/support fixture contract;
- external mesh source commit `3363b27aae6d37598dc142cf4dddb342fef3047f`.

## Hard relevance filter

For each rigid `link7` translation, the cube must satisfy both conditions before any MoveIt collision query is performed:

1. positive AABB overlap with both finger envelopes in X and Z;
2. a non-empty common mimic-joint interval inside `[-0.01, 0.019] m` where both cube Y side planes lie inside their corresponding finger AABBs.

The coarse search remains the PR #23 sphere:

- step: `2 mm`;
- radius: `80 mm`;
- total candidates: `267,761`;
- grasp-relevant candidates: `61,511`;
- filtered before collision checking: `206,250`;
- relevant coarse Z-step range: `[-40, 22]`, or `[-80, +44] mm`.

Candidates retain the exact PR #23 deterministic distance/tie-break ordering. If a coarse clear relevant candidate exists, the same local fine grid is used:

- step: `0.25 mm`;
- radius around coarse center: `2 mm` per axis;
- total local candidates: `4,913`;
- irrelevant fine candidates are filtered before collision checking.

## Collision contract

The new executable includes the accepted PR #23 search source in the same translation unit and reuses its helpers directly:

- accepted-state bounds validation;
- accepted-state self-collision validation;
- exact robot/world transforms;
- cube and support checked separately against every robot link;
- robot–robot pairs allowed only for fixture queries, exactly as in PR #23;
- all robot–fixture pairs forbidden;
- first-collision early rejection;
- baseline reproduction of the four accepted PR #22 contact pairs;
- full 2,802-state re-audit for a selected candidate.

The cube/support relative transform is never changed.

## Decisions

The evidence records one of:

- `FULL_PATH_CLEAR_GRASP_RELEVANT_TRANSLATION_FOUND`; or
- `NO_FULL_PATH_CLEAR_GRASP_RELEVANT_TRANSLATION_WITHIN_COARSE_RADIUS`.

A positive result proves only that one declared-grid translation satisfies the AABB reach necessary condition and is bounds-valid, self-collision-free, cube-clear, and support-clear over the accepted recorded path.

It does not prove exact mesh surface contact, contact normals, force closure, friction, gripper-motion clearance, grasp success, lifting, transport, placement, or hardware readiness.

A negative result proves only that no candidate in the declared relevant coarse grid passed the accepted full-path collision rules. It does not prove that no continuous translation, rotation, different fixture geometry, or different arm pose can work.

## Next safe phase

- Positive: offline gripper-joint sweep collision preflight at the selected fixture translation.
- Negative: stop at a fixture-or-pose redesign blocker; do not expand into commands or hardware.

## Safety boundary

- offline analysis only;
- no Gazebo start or ROS node;
- no object spawn;
- no controller, planner request, trajectory, gripper command, or attachment;
- no physical hardware;
- no production runtime modification;
- no merge requested.
