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

Scanning: the servo sweeps a coarse arc; the nearest reading in the car's path
is the obstacle (the centre echo alone can miss a round bottle), and the fine
steps fan out from it to find both edges. Only if the servo finds no open side
does the car turn on the spot in steps (`TURN_SCAN_RIGHT/LEFT`) to look further;
once a turn shows open space, the servo looks back to pin the edge down. The bypass is a mirrored three-leg detour (turn out,
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
| `b5_motion.c/.h` | both | `CAR_RUN=1`: moves as authority-stamped Buddy 2 goals, line search, line seen (host-tested) |

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

Not run on hardware.

### In the whole car (`CAR_RUN=1`)

`car_run_target.c` (vehicle_controller folder) gives Buddy 5 the optional
motion bindings, and then:
- every turn / forward / reverse is a Buddy 2 goal, submitted only once the
  controller grants obstacle authority (waits up to 300 ms, because the first
  turn is asked for in the same tick the obstacle is reported);
- the line search is Buddy 3's `subsystem3_reacquire()` plus a slow arc
  toward the line; "line detected" is Buddy 3's line status;
- stops are `subsystem2_motion_halt()` (zero at once, no latched inhibit);
- avoidance only runs while the controller is in AUTO, and is reset to
  MONITOR when AUTO ends, so a failed encounter does not disable it for the
  next run.

Moves and the line search run at 50 mm/s, the trial line cruise, so avoidance
never drives faster than line following; Buddy 2's measured executor sets
each move's time limit. Search arc ~33 cm radius (`b5_motion_default_config()`).

Turns are pivots (`arc_turns = false`) at the trial's 30% effort cap. A
lagging wheel used to finish the pivot alone and stall; Buddy 2 now stops both
together when it is within 20 mm. Forward arcs (both wheels forward, R ~9 cm,
timed) were tried but carried the car well off the line and their slow inner
wheel stalled. Set `arc_turns = true`
and `turn_radius_cm = 9` to use them again. Before turning, the nearest scan
hit must be at least 10 cm (`pivot_clearance_cm`); if not, the car reverses to
get that room and plans from the scan it already has. The turn out is at least
60 deg so the drive out stays short, and the pass beside the obstacle is
lengthened to reach past its front.

Scan (`av_config_default()`): the servo mount was freed on 2026-11-02, so the
scan covers 20..160 deg (+/-70) in 10 deg steps and the car stays still while
scanning; it does not turn to look past 70 deg (`turn_scan_max_deg` = 70), and
if it is too close to see a way past it reverses and scans again. Detection is at 20 cm. The servo sweeps at 300 deg/s rather than jumping.

Component bench tests (sensor and servo on their own, Pico SDK, wheels never
move): `../../testing/subsystem5/bench_firmware/README.md`.

### Line rejoin, result lines and telemetry

- Rejoin alignment (`line_align_cm`, `line_align_min_deg` in `av_config_t`):
  when a line sensor sees the line on a straight bypass leg crossing at
  20 deg or more, the car stops, drives `line_align_cm` so the turning
  centre is over the line, pivots to the line direction, then hands over
  (`AV_EVT_LINE_ALIGN`, still the BYPASS mission). If the line is no longer
  seen after the turn, the normal line search runs. A line seen mid-pivot or
  during the search is handed over at once, as before. Off (0) in
  `av_config_default()`; `CAR_RUN` sets it from the controller's
  `approach_mm` (line sensors to axle), so one measurement covers junctions
  and the rejoin.
- `[B5 RESULT]` / `[B5 TOTALS]` console lines at the end of every encounter
  (rejoined, cleared, failed, or aborted when AUTO ends);
  `../../testing/subsystem5/b5_metrics.py` turns a log into the page 8
  metrics table (`../../testing/subsystem5/TEST_LOG.md`, S5-M01..M05).
- `CAR_MQTT`: the section 5 profile, decision (free range past each edge)
  and plan fields are filled from the avoidance events, so the dashboard's
  Profile / Decision / Bypass plan rows and the added obstacle rows show
  real values. Groups are cleared at each new encounter and on reset.
