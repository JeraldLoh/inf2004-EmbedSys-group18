# Buddy 5 - Obstacle Scanning and Avoidance

Buddy 5 detects an obstacle on the line, measures it with the servo-mounted
sonar, drives round it and finds the line again. It also leads integration
review, but the shared Vehicle Controller is not in this folder.

This describes the firmware on `main` at `d1e2871` (2026-11-25; later commits
changed only documents and logs). Numbers are the
`CAR_RUN=1` settings in `subsystem5.c` unless stated otherwise.

## Files

| File | Runs on | Purpose |
|---|---|---|
| `range.c/.h` | both | Reading status, median filter, agreeing-pair filter for scans |
| `obstacle.c/.h` | both | Scan geometry, width, side choice, bypass plan |
| `avoidance.c/.h` | both | Detect, scan, decide, bypass and rejoin state machine |
| `b5_motion.c/.h` | both | Moves as Buddy 2 goals or continuous requests |
| `b5_mission_bridge.c/.h` | both | Avoidance state to controller events |
| `subsystem5.c/.h` | Pico | Task (core 0, 10 ms tick), car settings, console and telemetry |
| `drv_sonar.c/.h` | Pico | HC-SR04, ECHO edges timestamped in the GPIO interrupt |
| `drv_servo.c/.h` | Pico | Scan servo, 50 Hz PWM |
| `b5_run_target.c/.h` | Pico | `B5_RUN=1` bring-up without car motion |

"Both" means it is also host-tested. Pins (`../project_config/pinmap.h`): sonar
TRIG GP16, ECHO GP17 through a voltage divider, servo GP12.

## What happens in an encounter

1. **Detect.** Two readings in a row under 20 cm straight ahead. The car stops.
2. **Coarse scan.** The car stays still while the servo sweeps 30..150 deg
   (+/-60) at 30 deg/s, taking 13 readings 10 deg apart. The nearest reading in
   the car's path (10 cm each side) is the obstacle. The centre reading is used
   instead if it is within 4 cm of the nearest.
3. **Fine scan.** 10 deg steps outward on each side look for the edges. An edge
   needs measured hits on the obstacle and two measured open steps in a row. A
   missing echo is retried twice, then probed once per side at a half step. It
   never counts as open space or as an edge. Found edges are pulled in by 12 deg
   for the sonar beam.
4. **Decide.** A side needs a found edge and 25 cm free past it, with no reading
   in the strip the car will drive through. If both sides pass, the edge nearer
   the line wins; on a near tie, the side with more room.
5. **Check the exit.** Before driving out, a measured reading along the first leg
   must show at least 25 cm, or the leg length plus 10 cm if that is more. With
   both edges found and a turn of at most 60 deg the servo checks before the car
   turns; otherwise the car checks after turning. Blocked or unknown: try the
   other side, then reverse.
6. **Bypass.** Turn out 60..75 deg, drive out (at least 14 cm sideways, 10 cm
   past the edge), straighten, pass, turn back, drive to the line.
7. **Rejoin.** See "Finding the line again".

If neither side is usable the car reverses 10 cm and scans again, at most twice,
then fails. There is no rescan from the same spot. If the nearest reading is
under 10 cm it first reverses to about 15 cm. If the servo finds no usable side
and an edge is missing, the car turns to 70 deg to look for it (`TURN_SCAN_*`).

"Obstacle gone" needs measured far readings ahead: the scan and three more
pings. If the front gives no usable echo the encounter fails with
`obstacle status unknown` rather than driving on. After a reverse on the line,
the obstacle is assumed still there and the bypass uses its last position.

## Sonar readings

| Status | Meaning |
|---|---|
| `RANGE_OK` | Measured distance, 2..400 cm |
| `RANGE_NO_ECHO` | No return in range |
| `RANGE_TOO_CLOSE` | Under 2 cm, treated as blocked |
| `RANGE_FAULT` | No ECHO pulse, or ECHO stuck high |

Each reading is 3 pings. Watching ahead and the collision guard use the median.
Scans use the nearest two pings that agree within 3 cm (10% from 30 cm); if none
agree, the reading is unknown. Five faults in a row stop the car with
`SENSOR_CRITICAL`.

## Passing the obstacle

- The servo watches the obstacle at 65 deg. The pass ends once the side opens:
  the car covers the ray's lead plus 14 cm, so its back end clears the corner.
- The pass may run at most 25 cm past the plan. If the side is never seen, it
  ends at the planned length.
- A missing echo after the side has been seen does not count as the corner.
- The distance driven beside it gives the depth (a lower bound). The measured
  side distance resizes the return drive, within 0.5..1.5 times the out leg,
  before 8 cm is added to reach the line.
- One ping in seven looks ahead. Any reading under 6 cm ahead on a bypass drive
  stops the car; it backs off 12 cm and scans again.

## Finding the line again

- Line seen on a drive crossing at 20 deg or more: stop, drive 13 cm (line
  sensors to axle), pivot along the line, hand over.
- Not seen: pivot to 15, 30 and 45 deg each way, then out to 90 deg each way,
  passing through the line direction so no pivot is over 90 deg.
- Still not seen: Buddy 3's line search plus a 20 cm radius arc at 50 mm/s, two
  tries of 2 s, then fail with `line not reacquired`.
- Buddy 3's reading must be fresh to count as the line.

Line contact alone is not reported as a rejoin. `REJOINED` is printed and counted
only after 300 ms of fresh line following with both wheels moving and at least
20 mm driven, within 1.6 s of contact. Otherwise it is `REJOIN_UNVERIFIED`.

## Motion

Buddy 5 never writes PWM. `car_run_target.c` supplies these bindings:

- Moves go through `controller_obstacle_goal()` and
  `controller_obstacle_continuous()`. The controller checks the source and
  authority, passes the move to Buddy 2, and counts those accepted and rejected.
- A goal waits up to 300 ms for obstacle authority, then fails.
- A stop is `subsystem2_motion_halt()`: outputs to zero at once, no latched
  inhibit.
- Avoidance runs only in AUTO and returns to `MONITOR` when AUTO ends.

| Move | Speed |
|---|---|
| Pivots, and drives other than the two below | 120 mm/s |
| Drives that may cross the line | 70 mm/s |
| Watched pass | 60 mm/s |

Left turns round the obstacle are timed forward arcs about the left wheel (60
mm/s, 16 /m), because that wheel stalled in reverse. All other turns are pivots,
including the line align, the line search and turns made before the exit check.

## Controller missions

| Buddy 5 state | Controller mission |
|---|---|
| obstacle confirmed in `MONITOR` | `OBSTACLE_STOP` |
| `COARSE_SCAN`, `FINE_SCAN_*`, `TURN_SCAN_*`, `WAIT_RESCAN`, `REVERSE` | `SCAN` |
| `DECIDE` | `PLAN_BYPASS` |
| `BYPASS` | `BYPASS` |
| `REACQUIRE` | `RECOVERY` |
| back to `MONITOR` | `VERIFY_LINE` (left the line) or `TRACK_LINE` |
| `FAILED` | `SAFE_STOP`: `SENSOR_CRITICAL` or `RECOVERY_FAILED` |

`b5_mission_bridge.c` sends these as events after every `av_tick()`, filling in
any step passed through within one tick. A failure stays latched in the
controller; Buddy 5 does not clear it.

## Console and telemetry

| Line | Shows |
|---|---|
| `[B5 ECHO]` | The three raw pings behind a scan reading |
| `[B5 SCAN]` | Each scan or side point used |
| `[B5 PROFILE]` | Edges, width, clearances and side for each decision |
| `[B5 EXIT]` | Exit check: clear, blocked or unknown |
| `[B5 REJOIN SWEEP]` | Each line-search pivot target |
| `[B5 RESULT]`, `[B5 TOTALS]` | Outcome, measurements and running counts |

Width prints as `-` unless both edges were found, and is sent as 0 over MQTT.
`CAR_MQTT=1` fills the dashboard's profile, decision, plan and obstacle rows.
`../../testing/subsystem5/b5_metrics.py` turns a log into the metrics table.

## Build and test

From `build_make`, the car without Wi-Fi:

```
make SMP=1 CONSOLE=usb_cdc TEST=0 WIFI=none NET=none MQTT=0 CAR_RUN=1 PICO_SDK_PATH=<sdk path>
```

- `CAR_RUN=1 OBSTACLE_TABLE=1`: bypass on a table with line sensors ignored.
- `B5_RUN=1 CRASH_REPORT=0`, same network flags: scan and report only. Moves are
  refused, and stops use `subsystem2_urgent_stop()`.

Host tests: `../../testing/host/subsystem5/`. Sensor and servo bench tests:
`../../testing/subsystem5/bench_firmware/README.md`. Results:
`../../testing/subsystem5/TEST_LOG.md` and `../../testing/DEMO_EVIDENCE.md`.

## Known limits

- On 25 November (development builds), 20 logged encounters gave 3 confirmed
  rejoins, 2 unverified and 15 aborted. Two-edge widths ranged from 10 to 243 mm,
  so width was not reliable. Boxes were not measured, and recovery when boxed in
  is open. See `../../testing/subsystem5/OBSTACLE_RESULTS.md`.
- While watching ahead, no echo still means nothing is there. An angled or soft
  surface can be missed.
- The sonar is not read during turns or reverses, so obstacles and sensor faults
  are not noticed then.
- The sensor tilts down at the ends of its swing, so scans stop at +/-60 deg.
- The 12 deg beam correction is fixed. It can pull a real edge in too far.
- One reading under 6 cm is enough for the collision stop.
- The left arc is timed, with no angle feedback. It assumes the 200 mm/s^2 wheel
  acceleration of the default `CAR_B2_TRIAL=1` profile.
