# Buddy 5 - Obstacle Scanning and Avoidance

Primary responsibility: sonar trigger timing, servo scanning, obstacle
profiles, avoidance proposals, and bounded line recovery.

Servo scanning is the current primary assumption; calibrated whole-car
scanning is a conditional fallback. Buddy 5 also leads integration review but
this folder does not contain or own the separate shared Vehicle Controller.

The course folder is `subsystem5`; future `drv_sonar.c/.h` and optional
`drv_servo.c/.h` go directly here. Current lifecycle hooks return `E_NOSPT` and
target tests are SKIP placeholders. No trigger, servo or manoeuvre is started.

Hardware-independent logic ported from Buddy 5's standalone prototype (no GPIO,
PWM, interrupts or kernel calls; uses no libm, since the image is not linked
with it):

| File | Purpose |
|---|---|
| `range.c/.h` | Reading status (ok / no-echo / too-close / fault) and median-of-N filter |
| `obstacle.c/.h` | Scan geometry: direction+distance to x/y, obstacle width, side choice, bypass plan |
| `avoidance.c/.h` | Non-blocking detect, scan, decide, bypass, reacquire state machine (section 6.5) |

Scanning: the servo checks a narrow arc first (coarse sweep, then fine steps).
Edges outside the servo's reach are found by turning the whole car on the spot
in steps (`TURN_SCAN_RIGHT/LEFT`); once a turn shows open space, the servo looks
back to pin the edge down. The bypass is a mirrored three-leg detour (turn out,
drive, straighten, drive alongside, turn back the same angle, drive, straighten)
so the car ends near the line facing the line direction, then searches for it.
Motion is requested through `motion_turn` / `motion_forward` / `motion_reverse`.

`avoidance.h` defines `av_platform_t`, the callback table that the future
sonar/servo drivers and the controller/motion adapters fill in. Avoidance only
proposes motion through those callbacks; it never drives PWM itself. Host tests:
`../../testing/host/subsystem5/` (target `obstacle_avoidance_host_tests`).

Agreed pins (`../project_config/pinmap.h`): sonar TRIG GP16, ECHO GP17 (through
a voltage divider), scan servo GP12. The physical scan range and car-turn
accuracy remain unverified. Avoidance
proposes actions to the controller; it must not compete with Buddy 2 for PWM.
Keep evidence in `../../testing/subsystem5/`.

Controller missions for each Buddy 5 state (`../vehicle_controller/README.md`):

| Buddy 5 state | Controller mission | Call when entering it |
|---|---|---|
| obstacle confirmed in `MONITOR` | `OBSTACLE_STOP` | `vehicle_controller_on_obstacle()` |
| `COARSE_SCAN`, `FINE_SCAN_*`, `TURN_SCAN_*`, `WAIT_RESCAN`, `REVERSE` | `SCAN` | `obstacle_progress(MISSION_SCAN)` |
| `DECIDE` | `PLAN_BYPASS` | `obstacle_progress(MISSION_PLAN_BYPASS)` |
| `BYPASS` | `BYPASS` | `obstacle_progress(MISSION_BYPASS)` |
| `REACQUIRE` | `RECOVERY` | `obstacle_progress(MISSION_RECOVERY)` |
| back to `MONITOR` | `VERIFY_LINE` (left the line) or `TRACK_LINE` | `obstacle_cleared()` |
| `FAILED` | `SAFE_STOP` | `latch_stop(STOP_REASON_RECOVERY_FAILED)`, or `SENSOR_CRITICAL` for a sensor fault |

`b5_mission_bridge.c` makes these calls (as controller events) after every
`av_tick()`, filling in any step the controller requires that Buddy 5 passes
through within one tick.

## Kernel task (`B5_RUN=1`)

| File | Runs on | Purpose |
|---|---|---|
| `subsystem5.c/.h` | Pico | Task (core 0, 10 ms tick): `av_tick()`, bridge, posts events to the controller |
| `drv_sonar.c/.h` | Pico | HC-SR04: TRIG output, ECHO edges through the shared GPIO dispatcher |
| `drv_servo.c/.h` | Pico | Scan servo: 50 Hz PWM, 1 us per count, slice 6 |
| `b5_mission_bridge.c/.h` | both | Avoidance state -> controller mission events (host-tested) |
| `b5_run_target.c/.h` | Pico | `B5_RUN=1` bring-up: B2 foundation, controller task, Buddy 5 task |

Build from `build_make`:

```
make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 B5_RUN=1 PICO_SDK_PATH=<sdk path>
```

The default build (`B5_RUN=0`) starts nothing, as before.

What it does now: watches ahead, stops the motors on an obstacle (directly
through `subsystem2_urgent_stop()`, and the controller does too), scans with
the servo, decides a side and reports each step to the controller.

What it cannot do yet, and refuses instead of faking:
- car turns, forward/reverse moves and line searches: Buddy 2 has no goal
  executor (motors stay inhibited). A refused move fails the encounter at
  once, so anything wider than the servo arc ends in a latched safe stop;
- line detection: Buddy 3 is not wired to Buddy 5.

Not run on hardware: compiled against the kernel headers with the team flags
for every profile (B5_RUN, default SMP=0/1, B2_BENCH); not linked or flashed
here (no `make` on this machine).

Component bench tests (sensor and servo on their own, Pico SDK, wheels never
move): `../../testing/subsystem5/bench_firmware/README.md`.
