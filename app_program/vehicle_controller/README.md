# Shared Vehicle Controller

All five buddies may contribute; Buddy 5 leads integration review. This is NOT
Buddy 5's obstacle module and is NOT a sixth student's subsystem.

The portable `vehicle_controller.c/.h` and `safety.c/.h` were moved unchanged from
the old `firmware/app` location. They are partial, host-testable state logic, not
a complete controller or a working RTOS task. Existing limitations were not fixed
by this structural migration.

## Obstacle missions

An obstacle is handled as a mission, not a latched stop, so Buddy 5 can still
move the car to scan and drive around it:

| Call | Mission after | Action returned |
|---|---|---|
| `vehicle_controller_on_obstacle()` (AUTO, nothing latched) | `OBSTACLE_STOP` | `STOP_MOTION` |
| `vehicle_controller_obstacle_progress(SCAN)` | `SCAN` | `START_SCAN` |
| `vehicle_controller_obstacle_progress(PLAN_BYPASS)` | `PLAN_BYPASS` | none |
| `vehicle_controller_obstacle_progress(BYPASS)` | `BYPASS` | none |
| `vehicle_controller_obstacle_progress(RECOVERY)` | `RECOVERY` | `ENTER_RECOVERY` |
| `vehicle_controller_obstacle_cleared(true / false)` | `VERIFY_LINE` / `TRACK_LINE` | `START_LINE_TRACKING` |

Also allowed: `PLAN_BYPASS -> SCAN` (rescan or reverse), `BYPASS -> SCAN` and
`RECOVERY -> SCAN` (re-plan after an emergency stop), `SCAN -> RECOVERY`
(obstacle gone after leaving the line). Anything else is rejected.

- `vehicle_controller_motion_allowed()`: only `MOTION_SOURCE_OBSTACLE` may
  drive during an obstacle mission; it may not drive outside one.
- Every mission change starts a new authority generation, so late results of
  older goals are ignored. A finished obstacle goal does not resume line
  following; a timed-out or failed one latches `STOP_REASON_MOTION_FAULT`.
- Junction observations are ignored during an obstacle mission (the line
  sensors cross the line at an angle while going around).
- An obstacle cancels a reserved junction route.
- If avoidance fails, latch `STOP_REASON_RECOVERY_FAILED` (or
  `STOP_REASON_SENSOR_CRITICAL`) with `vehicle_controller_latch_stop()`; it
  needs `vehicle_controller_clear_stop()` and AUTO again to restart.

A dedicated controller task, kernel message transport, button/remote adapters,
goal dispatch, line-stability confirmation, recovery permits and full fault policy
are future work. Keep a single runtime owner of mutable controller state. Do not
call the portable state-mutating functions concurrently from both cores.

Future lifecycle integration must create controller AND subsystem objects before
starting any application task. This directory is directly under `app_program`
so its `.c` files are discovered by the inspected one-level course Make rule.
