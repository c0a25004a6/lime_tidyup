# Nav2 resilience notes

## Problem

On the Lime real-robot/dev setup, Nav2 can occasionally stop accepting or completing navigation work and manual bringup is then used to restore operation.

The TurtleBot3 Lime navigation launch forwards Nav2's `use_composition` and `use_respawn` options, but its defaults are composed bringup with respawn disabled. Nav2 Humble only applies per-node process respawn when composition is disabled.

## Changes in this branch

- `run_nav`, `run_navigation`, and `run_all` launch Nav2 with:
  - `use_composition:=False`
  - `use_respawn:=True`
- `bin/nav2_watchdog.py` checks:
  - `/navigate_to_pose` action availability
  - lifecycle state of localization nodes (`map_server`, `amcl`)
  - lifecycle state of navigation nodes (`controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`)
- Recovery is intentionally conservative:
  - three consecutive unhealthy probes are required
  - a 45-second startup grace avoids recovering during normal bringup
  - a 60-second cooldown prevents recovery loops
  - a zero `Twist` is published before lifecycle recovery
  - navigation-only failures reset/start only `lifecycle_manager_navigation`
  - localization failures reset/start localization and navigation in dependency order

## Manual diagnostics

Check once without changing lifecycle state:

```bash
python3 ~/bin/nav2_watchdog.py --once --check-only --startup-grace 0
```

Attempt one recovery only if the stack is unhealthy:

```bash
python3 ~/bin/nav2_watchdog.py \
  --once \
  --startup-grace 0 \
  --failure-threshold 1 \
  --recovery-grace 10
```

## Validation still required

The following should be tested in simulation before relying on it on hardware:

1. Start `run_all` and verify all monitored lifecycle nodes become `active`.
2. Confirm `/navigate_to_pose` accepts a normal goal.
3. In simulation only, terminate one Nav2 server process and verify launch respawns it.
4. Verify the watchdog restores the lifecycle stack to `active` without restarting the full bringup.
5. Confirm an in-progress robot command receives zero velocity before recovery.
6. Run repeated navigation goals long enough to cover the original intermittent failure window.

## Known limitation

This watchdog detects missing/inactive lifecycle nodes and loss of the NavigateToPose action server. It does not yet prove that an `active` Nav2 server is making progress. A server that remains alive and active but is logically hung may therefore require a separate progress/timeout witness.

Also, the current `Tb3NavigationSystem.goto()` implementation still reports `True` unconditionally after the Nav2 action returns. That should be corrected separately so Behavior Trees can distinguish navigation success from abort/cancel/failure.
