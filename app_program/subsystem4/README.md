# Buddy 4: IMU, terrain and hump monitoring

Buddy 4 owns the GY-511 (LSM303DLHC accelerometer and magnetometer) and
produces the following:

- **Calibration:** level calibration that removes the board's mounting
  pitch/roll, and optional magnetometer hard-iron calibration.
- **Attitude:** pitch, roll, tilt-compensated magnetic heading, and turn rate.
- **Terrain:** the state LEVEL / ASCENDING / CREST / DESCENDING / SETTLING,
  plus a speed cap for the controller.
- **Humps:** detection with peak height, uncertainty, length and validity.
- **Motion:** classification as stationary, moving, rough or impact (a
  jolt); acceleration and braking.
- **End of run:** the car lifted at least 20 cm, at any tilt, or overturned.
  The motors stop and stay stopped until the car is back on its wheels and
  START is given again. This replaces IMU collision detection, which the
  lecturer removed from the requirements (2026-11-11).
- **Turns:** completed-turn events with direction and angle.

All shared data uses minimum-bit packed fields (Change Requirement 1). See
[DATA_REPRESENTATION.md](DATA_REPRESENTATION.md).

Other buddies may contribute, but Buddy 4 stays accountable for this
subsystem's contracts, calibration and evidence.

## Status

| Item | State |
|---|---|
| Portable math, estimator, packing, seqlock, MQTT payloads | Implemented; host tests pass (`testing/host/test_terrain_estimator.c`, built twice as `terrain_demo_baseline_tests` and `terrain_demo_jolt_pause_tests`; run with `ctest -R terrain`) |
| LSM303 driver and IMU task (micro T-Kernel) | Implemented and **running on the car** in the combined demo build (`CAR_RUN=1 CAR_MQTT=1`, see [docs/DEMO_BUILD.md](../../docs/DEMO_BUILD.md)) |
| Hardware evidence (24–25 Nov, [testing/DEMO_EVIDENCE.md](../../testing/DEMO_EVIDENCE.md) E2–E4) | 64 complete, valid, non-stale `[B4]` summaries with odometry present; median processing about 3.15 ms per sample; a valid **9 mm ±8 mm** hump event; three pickup stops latched. **Open:** fixture truth for the hump height, an instrumented pickup timing series, overturn test |
| On-target self-test S4-T01..T11 | Implemented (`make TEST=1`); not yet run |
| Lifecycle hookup (`subsystem4_init/start/test`) | Done: `car_run_target.c` inits Buddy 4 after the others and starts it after Buddy 5. Optional: a failure is printed and the car runs without it |
| I2C bus | The driver bit-bangs I2C on GP2/GP3 itself; the course `"iicb"` device is not used (it stalled in Wi-Fi builds). See Integration step 1 |
| Controller, Buddy 2 and Buddy 1 links | Done in `CAR_RUN=1` and used in the demo build (see "Whole-car wiring") |
| Stale-data handling (F3) | Done: if the IMU sequence stops changing for `B4_STALE_MS` (200 ms), the controller drops the terrain cap, the MQTT section stops being sent and the pickup/restart checks treat Buddy 4 as unavailable. No STOP |
| End of run (pickup / overturn) | Done and seen on the car: latched stop, both START paths refused until the car is back upright and still. Lift threshold and window are now build knobs (`LIFT_MM`, `LIFT_WINDOW_MS`, `LIFT_JOLT_PAUSE`) |
| Heading log / heading assist | Optional, off in the demo build: `HEADING_LOG=1` (10 Hz `[B4 HDG]` trace) and `HDG_ASSIST=1` (compass heading hold while both line sensors are white) |
| Buddy 3 hump pause (tribunal R2) | **Not wired**, by decision: barcode reading is not suspended because Buddy 4 sees a hump (see "Open items") |
| Fault reporting | Detail mask, events and `s4_sensor_failures()`, shown in the `[B4]` console line and MQTT section 4. **Not** latched into the shared hardware fault mask (see "Whole-car wiring") |
| Thresholds in `s4_config.h` | Hump thresholds sized for the 10 mm x 195 mm demo hump by host simulation; `S4_CASTER_ARM_MM`, `S4_TRACK_MM` and `S4_CLIMB_NOSE_DOWN` still **to be measured** on the car |

## Files

All `.c` files sit directly in this folder, because the course Make rule only
builds one level below `app_program/`.

| File | Kernel? | Purpose |
|---|---|---|
| [s4_imu.h](s4_imu.h) | yes | **Public interface for other buddies**: lifecycle, requests, object IDs |
| [s4_packed.h](s4_packed.h) | no | Every packed field: shift, width, enums, fault bits, compile-time checks |
| [s4_shared.h](s4_shared.h) / [.c](s4_shared.c) | no | Seqlock snapshot, odometry input, MQTT frame/command, `terrain_report_t` adapter |
| [s4_atomic.h](s4_atomic.h) | no | 32-bit load/store and the cross-core fence |
| [terrain_estimator.h](terrain_estimator.h) / [.c](terrain_estimator.c) | no | The algorithm: calibration, filters, state machine, hump height, end of run (lift/overturn), turn |
| [imu_math.h](imu_math.h) / [.c](imu_math.c) | no | Fixed-point decode, axis mapping, atan2, isqrt, sine, heading |
| [drv_lsm303.h](drv_lsm303.h) / [.c](drv_lsm303.c) | yes | LSM303DLHC over a software I2C master on GP2/GP3 (the I2C1 block stalled in Wi-Fi builds) |
| [task_imu.c](task_imu.c) | yes | The IMU task, cyclic tick, request flag, event queue, bus fault recovery, optional `[B4 HDG]` trace and `s4_heading_read()` for the heading assist |
| [s4_selftest.c](s4_selftest.c) | yes | On-target checks, built only with `TEST=1` |
| [s4_config.h](s4_config.h) | no | All tuning values, periods, priority, mounting |

"Kernel? = no" files have no RTOS or hardware dependency. That is what lets
them be tested on a PC.

## Data flow

```text
 cyclic handler (20 ms) --tick--> +------------------------------+
 any task: s4_request() --bits--> | IMU task (only GP2/GP3 user) |
 binding: read_odometry_mm() ---> |  drv_lsm303 -> s4_est_step   |
                                  +-----+--------------+---------+
                    s4_shared_publish() |              | 4-byte events
                                        v              v
        seqlock snapshot (4 packed words)      s4_mbf_events (message buffer)
           |               |            |                 |
   controller (IF-03)  Buddy 1 MQTT    any ISR      Vehicle Controller
   s4_terrain_report_  s4_telemetry_   s4_shared_   hump / end of run /
   from_words()        pack() 17 B     read()       turn / fault / cal
```

## Integration steps

1. **I2C on GP2/GP3 (done).** `drv_lsm303.c` drives GP2 (SDA) and GP3
   (SCL) directly as open-drain SIO pins with pull-ups, at about 150 kHz
   (about 1 ms of CPU per sample). The course I2C1 device (`"iicb"`) stopped
   transmitting in every Wi-Fi build, so it is not used for the IMU.
   The root config still has `DEVCNF_USE_IIC 1`, I2C1 pins 2/3
   (`i2c_cnf_sysdep.h`) and unit 1 started in `devinit.c`. The driver takes
   the pins back from it when it opens. Keep those settings: they stop
   unit 0 claiming GP8/GP9 (Buddy 2 motor) and stop GP6/GP7 being used
   (GP7 is Buddy 3's left line sensor).

   If the bus is missing, the task still runs and reports `S4_FAULT_I2C` and
   terrain `UNKNOWN`/`INVALID` with the degraded speed cap.
2. **Wiring.** Connect the GY-511 to Robo Pico **Grove port 2** (or the
   Maker/QWIIC port, which shares GP2/GP3): GP2 to SDA, GP3 to SCL, 3V3 to
   VIN, and GND. Mount it flat, near the centre of the car. The default axes
   assume X points right, Y forward and Z up (the briefing's IMU figure). If
   the board is rotated, change `S4_AXIS_*` in `s4_config.h`.
3. **Lifecycle (done).** `task_imu.c` defines `subsystem4_init()` (creates
   the flag, message buffer and cyclic handler, rolling back on failure),
   `subsystem4_start()` (creates the task, starts the tick) and, with
   `TEST=1`, `subsystem4_test()`. Optional inputs are bound with
   `s4_imu_bind()` between init and start. `car_run_target.c` calls them in
   its all-inits-then-all-starts sequence.
4. **Resources (recorded in `TEAM.md`).** Task priority 12 (provisional),
   2 KB stack, and 4 kernel objects: 1 event flag, 1 message buffer, 1 cyclic
   handler, 1 task. The task has no core affinity.

## Whole-car wiring (`CAR_RUN=1`, in `vehicle_controller/car_run_target.c`)

| Link | How | Notes |
|---|---|---|
| Buddy 4 → controller: speed cap | `controller_task_bind_terrain(b4_read_terrain)`: the controller task reads the snapshot every 10 ms tick with `s4_terrain_report_from_words()` | `vehicle_controller_update_terrain()`; line tracking takes the lowest cap. The hump cap (100 mm/s) only bites above it: the demo build cruises at 80 mm/s, so today it changes nothing |
| Stale IMU data | `b4_read_terrain()` tracks the snapshot sequence (`S4_AT_SEQ`). If it has not changed for `B4_STALE_MS` (default 200 ms; the IMU publishes every 20 ms) the report becomes `TERRAIN_UNKNOWN` with cap 0 | The old cap is dropped, never kept. No STOP. The supervisor prints `[B4] IMU data stale ...` / `... fresh again` once per change, and `stale=` is in the `[B4]` line. Stale data also stops MQTT section 4 and section 2's terrain field, and the end-of-run and remote-restart checks |
| Buddy 2 → Buddy 4: odometry | Binding `read_odometry_mm` = `b4_read_odometry()`, called by the IMU task each period | Integrates `subsystem2_motion_telemetry()` wheel speeds over its sample time, each wheel signed by its last non-zero target, so a pivot adds no distance. Up to 10 missed reads repeat the last value; a gap over 200 ms adds no distance. Buddy 2's code is unchanged |
| Buddy 4 → Buddy 1: MQTT | `b4_wifi_telemetry()` sends `s4_telemetry_pack()` as section 4 every 200 ms (`CAR_MQTT=1`), only while the data is fresh; the controller's terrain state also fills section 2 | The dashboard's `decodeBuddy4()` matches `s4_packed.h` and shows calibration, terrain, the last hump and end of run |
| Buddy 4 → supervisor: end of run | `b4_end_of_run()` every supervisor cycle (see "End of run → motors stop" below) | Both GP20 and MQTT START are refused while the latch is set. MQTT START after a physical stop also needs `imu_ready_for_remote_restart()`: fresh, valid, level-calibrated, lift reading 0 and no latch |
| Optical pickup (optional, `OFF_GROUND=1`) | `off_ground_end_of_run()` with `pickup_guard.h`: Buddy 3's line-sensor ADC reading "air" | Off in the demo build: it needs a measured ADC threshold first (see [DEMO_BUILD.md](../../docs/DEMO_BUILD.md) "Pickup measurement gate"). A recent Buddy 4 lift reading of at least 2 cm shortens its confirmation from 1000 ms to 300 ms. The IMU lift/overturn stop stays active either way |
| Buddy 4 → controller: heading (optional, `HDG_ASSIST=1`) | `s4_heading_read()` from `controller_service.c` | Heading reference captured at run start; a small capped counter-curvature holds it within about ±2° only while both line sensors are white. The tape always wins. `[HDG]` status lines. Off in the demo build: motor currents disturb the magnetometer |
| Buddy 4 → console | `[B4]` line every second next to `[CAR]`, plus event lines (terrain, hump, calibration, end of run). `HEADING_LOG=1` adds a 10 Hz `[B4 HDG]` trace for `tools/heading_check.py` | See "Reading the serial log" |
| Buddy 4 → controller: faults | **Not mapped.** | Any bit in the shared hardware fault mask blocks junction handling and MQTT START, so an unplugged IMU would stop a run. The provisional `HARDWARE_FAULT_IMU/MAG` bits stay unused until the team agrees a non-blocking class |

### Reading the serial log

```text
[B4] valid=1 cal=level terrain=LEVEL cap=0 pitch=0.5 roll=-0.5 deg vib=8 mg motion=1 odom=1 faults=0x2 fail=0x0 i2c_err=0 live_h=0 mm | last hump 0 mm +/-0 peak 0 deg 0 cm invalid/none #0 | end=no lift=0 cm | 3150 us/sample stale=0
[B4] terrain ASCENDING, cap 100 mm/s
[B4] terrain CREST, cap 100 mm/s
[B4] terrain DESCENDING, cap 100 mm/s
[B4] terrain SETTLING, cap 100 mm/s
[B4] hump 10 mm +/-5, peak 2 deg, 20 cm, valid
[B4] terrain LEVEL, cap 0 mm/s
```

The `hump` line is followed by an indented detail line showing how the height
was worked out (one wheel or full width).

- `cal=RUNNING-keep still` / `none`: level calibration is running or failed; it retries every 2 s once the car is still.
- `faults` is the 7-bit detail mask: 0x01 I2C, 0x02 magnetometer, 0x04 stale, 0x08 uncalibrated, 0x10 saturated, 0x20 tilt (0x40 is reserved, formerly collision). 0x2 alone is normal until the magnetometer is calibrated or if it is absent.
- `vib` is the vibration level while driving. Host simulation shows no false humps up to about 30 mg of noise; if it reads well above that and you see `INVALID` humps on flat floor, raise `S4_RAISE_ON_MM` to 5 or `S4_PITCH_SMOOTH_SHIFT` to 4. At 45 mg, simulation gave about one INVALID episode a minute and no valid ones.
- `odom=1` means the height came from Buddy 2's wheel speeds; 0 means the fallback speed was used.
- `i2c_err` counts bus errors since boot (the task recovers the bus itself).
- `end` is `no`, `lifted` or `overturned`; `lift` is the current lift estimate in cm.
- `us/sample` is the processing time of one sample, not the 20 ms sample period. On the car the median was about 3.15 ms.
- `stale=1` means no new IMU sample for `B4_STALE_MS`: the speed cap is ignored until it returns.

### Build knobs (Makefile, `build_make/pico_rp2040.mk`)

| Knob | Default (demo build) | Effect |
|---|---|---|
| `LIFT_MM` | 200 | `S4_LIFT_HEIGHT_MM`: estimated rise that ends the run |
| `LIFT_WINDOW_MS` | 1200 | `S4_LIFT_WINDOW_MS`: the rise must happen this fast |
| `LIFT_JOLT_PAUSE` | 0 | 1: a jolt during a lift only pauses the integration (the window keeps counting) instead of cancelling the lift. Used in the recovery test build only |
| `B4_STALE_MS` | 200 | IMU data older than this is stale (cap dropped, no STOP) |
| `HEADING_LOG` | 0 | 1: 10 Hz `[B4 HDG]` heading trace |
| `HDG_ASSIST` | 0 | 1: controller compass heading hold while both sensors are white |
| `OFF_GROUND`, `OFF_GROUND_ADC`, ... | 0 | Optional optical pickup; refuses to build without a measured ADC threshold |

A 150 mm / 2000 ms lift setting was tried in host simulation and rejected: the
120 mm below-threshold lift test then read 150 mm and tripped. The 200 mm /
1200 ms limits stay until real lifts are characterised.

### Open items

- **Buddy 3 hump pause (tribunal R2).** Status bit 23 is published, but nothing consumes it. The team decided barcode reading is not suspended just because Buddy 4 sees a hump; on a 10 mm hump the line sensors tilt by only about 4°.
- **Hump accuracy (S4-U11).** The car reported a valid 9 mm ±8 mm hump, but the fixture was not measured. Run three passes over each measured fixture and record them in [TEST_LOG.md](../../testing/subsystem4/TEST_LOG.md).
- **Pickup timing (S4-L01) and overturn (S4-L02).** Pickup stops were logged; an instrumented from-motion series and the overturn fixture test are still to do.
- **MQTT commands to Buddy 4** (`s4_command()`: recalibrate, magnetometer calibration, range report) are not routed by Buddy 1 yet.
- **Event queue.** Nothing reads `s4_mbf_events`; the oldest event is dropped when it is full (`s4_events_dropped`). Events still reach the console.

If level calibration fails at start-up (for example the car was handled
during Buddy 3's barcode sweep), the IMU task retries it every 2 s until it
succeeds. Until then terrain is `UNKNOWN` and the degraded 100 mm/s cap
applies.

## Interfaces for the other buddies

| From / to | Call | Notes |
|---|---|---|
| Odometry → B4 | `s4_bindings_t.read_odometry_mm` via `s4_imu_bind()` | Called by the IMU task every period: return the signed average path distance of both wheels in mm, or false if unavailable. (Low level: `s4_post_odometry()` from exactly one task.) |
| B4 → controller | `s4_shared_read(words)` + `s4_terrain_report_from_words()` | Gives the existing `terrain_report_t` for `vehicle_controller_update_terrain()`. The speed cap composes as "lowest cap wins". |
| B4 → controller | `tk_rcv_mbf(s4_mbf_events, &msg, tmo)` | 4-byte events, single reader. When the queue is full the oldest event is dropped (`s4_events_dropped`). |
| B4 → Buddy 1 | `s4_telemetry_pack(buf, 17)` | Binary frame; decode with the tables in DATA_REPRESENTATION.md. |
| Buddy 1 → B4 | `s4_command(byte)` | After Buddy 1 has validated authority: 1 recalibrate, 2 clear latched, 3/4 magnetometer calibration start/stop, 5 print the observed ranges and sample timing. |
| B4 → Buddy 3 | status word bit 23 (hump flag) | Published, but **not consumed**: barcode reading is not suspended on a hump (tribunal decision R2, see "Open items"). |
| B4 → controller | `s4_heading_read(&heading_ddeg, &turn)` | Optional heading assist (`HDG_ASSIST=1`); false until the heading is valid. |
| B4 → controller | `s4_sensor_failures()` (`S4_FAILURE_IMU`, `S4_FAILURE_MAG`) | Persistent sensor failures, latched until reset. Shown in the `[B4]` line (`fail=`); not mapped into the shared hardware fault mask (it would block junctions and START). The detail mask (status bits 19..13) and `S4_EV_FAULT` events keep the full evidence; UNCAL, SATURATED and TILT are status, not failures. |
| Any → B4 | `s4_request(S4_REQ_*)` | ISR-safe. |

Week 8 tribunal decisions applied here:

- **One DATA_VALID bit per message** (status bit 31) replaces per-field
  invalid codes, and CLAMPED (bit 30) flags an out-of-range value.
- **Explicit hump and end-of-run flags** (the collision flag was replaced).
- **A 3-bit motion class:** stationary, moving, accelerating, braking, turn
  left, turn right, rough, impact.
- **A 2-bit calibration status:** none, running, level, level plus
  magnetometer.
- **Turn rate:** ±250 °/s in 2 °/s steps, which fits 8 bits.
- **Shared fault mask:** the tribunal agreed IMU faults go in the shared
  bitmask. The team's [contracts/hardware_faults.h](../contracts/hardware_faults.h)
  makes the Vehicle Controller its single owner, so Buddy 4 never writes it.
  Buddy 4 reports evidence instead (detail mask, `s4_sensor_failures()` and
  `S4_EV_FAULT` events). The controller does not map it yet (see "Whole-car
  wiring").

**End of run → motors stop.** Buddy 4 publishes the end-of-run latch (status
bit 24, cause in bits 26..25, lift height in motion bits 23..16) and an
`S4_EV_END_RUN` event. The car supervisor (`car_run_target.c`) reads the
snapshot every 5 ms with `s4_end_of_run_from_words()`, only while the data is
fresh. While the latch is set it halts the motors and posts
`CONTROLLER_EVENT_LATCH_STOP` with `STOP_REASON_PHYSICAL` every 100 ms, and
refuses GP20 and MQTT START (`[CAR] START refused: car still
lifted/overturned` / `[CAR] MQTT START refused: pickup stop is still
active`). If the data goes stale while latched, the stop stays until a fresh,
valid snapshot shows the latch cleared. That is
the same stop reason as the GP21 STOP button: an operator's physical action,
so no new telemetry field is needed and a fresh START clears it once the car
is back down.

## How it works

**Sampling.** Every 20 ms the task performs one 7-byte burst read (accel
status and X/Y/Z) and one 6-byte magnetometer read. The accelerometer runs at
±4 g, high resolution, 100 Hz output, with block data update, which leaves
headroom for 3 g shocks. All maths is integer: the RP2040 has no FPU.

**Level calibration** runs at start-up and on request. It averages 64 samples
(1.28 s) and requires each axis to stay within a 40 mg spread. Any movement
restarts the window, and after 5 retries it fails. The resting gravity vector
gives the mounting pitch/roll offsets and the gravity magnitude. **Keep the
car still on a level floor until `[B4] level calibration done` prints, or the `[B4]` status line shows `cal=level`.**

**Hump height.** The briefing suggests h ≈ v²/2g from a vertical
acceleration impulse. That requires integrating noisy acceleration twice,
which drifts badly with a ±several-mg MEMS sensor. Buddy 4 instead integrates
the slope along the travelled path:

  **height = Σ sin(pitch) × Δdistance**

Δdistance comes from Buddy 2's odometry, which is exact along the surface.
Without odometry it falls back to `S4_FALLBACK_SPEED_MM_S` × dt, and the
uncertainty is widened. Integration starts as soon as the pitch leaves the
level band, so the climb before the state is confirmed is not lost.

The integral follows the drive axle centre. It misses half of a ramp under
one wheel, and it averages a short hump over the caster arm. The car has two
drive wheels and a caster, so while the lowest contact is on the floor the
height of the highest raised wheel (or of the caster) follows from the
attitude alone:

  **raised = caster_arm × sin|pitch| + (track / 2) × sin|roll|**

- Demo ramp, under the **right wheel only**: roll = 10 / 112 mm = 5.1° and
  pitch only about 2.2°, with the same sign all the way across. Each term
  gives 5 mm, so the total is 10 mm.
- Full-width hump: roll 0, so the pitch term alone gives h. When the caster
  crosses, the pitch term gives the caster's height.

Each hump is classified when it ends. If the roll term (track × sin roll)
is at least half the pitch term (caster_arm × sin pitch), **one wheel** was
raised. The height is then **track × sin(peak roll)** alone: exact with the
other wheel and the caster on the floor, and independent of the caster arm
and of speed changes on the ramp (which read as pitch). The first car run
showed why this matters: 4° of pitch on the right-wheel ramp made the
combined formula read 13 mm for 10. Otherwise (**full width**) Buddy 4
reports the larger of the peak raised height and the integral: on long
ramps the integral wins, on humps the attitude.

**Hump width.** The height is also recorded against distance travelled:
one sample per 5 mm of odometry, a 128-sample ring (640 mm, 256 bytes) that
also keeps the approach before the episode. The ring is smoothed over
3 samples. At the end of a hump:

- the height is the mean of the samples within 90% of the top;
- the width is extrapolated to the floor from the 25% and 75% crossings,
  **1.5 × w25 − 0.5 × w75**. This is exact for straight ramp faces (wedge,
  trapezoid, triangle). A rounded bump with long shallow feet reads about
  15% short.

The width replaces the episode path in the hump's length field (cm). If
the ramp edges fall outside the 640 mm, the path is reported instead. Host
simulation, 10 mm × 195 mm under one wheel at 60 mm/s: height 9–10 mm,
width 180–200 mm. Over 20 noise seeds the width rms error is 4–7 mm at
15 mg vibration and 8–12 mm at 30 mg. Width depends on Buddy 2's odometry
being right: check the `path` figure against a measured run. Pitch and roll come
from a second 140 ms smoothing, so vibration does not inflate the peak.
Measure `S4_CASTER_ARM_MM` (drive axle centre to the caster's floor contact)
and `S4_TRACK_MM` (tyre centre to tyre centre). On the one-sided ramp, a 10%
error in the caster arm moves the height by 0.5 mm.

Each result reports:

- the peak height;
- the uncertainty: base + residual height at the end + 1% of the path, plus
  half the height when distance was estimated;
- the length and the peak pitch;
- a valid bit. Valid requires a rise and a fall back to the floor, height
  ≥ 5 mm, length between 80 and 1500 mm, a small residual, and no fault
  during the episode.

**Terrain state machine.** It follows the raised height g in mm, so a
one-sided ramp (roll) and a full-width hump (pitch) take the same path. Each
transition needs `S4_CONFIRM_SAMPLES` consecutive samples.

| From | To | When |
|---|---|---|
| LEVEL | ASCENDING | g ≥ 4 mm (`S4_RAISE_ON_MM`), counting roll and only climb-direction pitch (nose-up with the caster behind, `S4_CLIMB_NOSE_DOWN 0`). For 0.5 s after moving off from rest, pitch is ignored, because acceleration reads as nose-up |
| LEVEL | DESCENDING | pitch beyond 5° the other way (a dip or step down; braking reads nose-down, hence the large value) |
| ASCENDING | CREST | g has fallen 1 mm or 10% below its peak |
| CREST | ASCENDING / DESCENDING | new peak / g 2 mm or 25% below the peak |
| DESCENDING | SETTLING | g < 2 mm (`S4_RAISE_OFF_MM`) and the height integral is back below 4 mm. If the integral is still high, all three contacts are on top of a wide hump: wait for the caster, at most 2 s |
| DESCENDING | ASCENDING | g back near the peak (the caster crossing a full-width hump) |
| SETTLING | ASCENDING | g ≥ 4 mm again: the same hump continues |
| SETTLING | LEVEL (publish result) | 300 ms on the floor |
| any | INVALID | I2C error or stale data; returns to LEVEL only once level again |
| any | UNKNOWN | not calibrated, or recalibrating |

**Speed caps.** LEVEL has no cap. The hump states use 100 mm/s. UNKNOWN,
INVALID, tilt over 25°, and the impact hold all use 100 mm/s. The demo build
cruises at 80 mm/s, so the caps only take effect on faster profiles. Stale
IMU data removes the cap rather than applying it.

**Motion and jolts.** The vibration level is the filtered deviation of |a|
from 1 g. A horizontal jolt above 1000 mg relative to the slow filter (a knock
or bump) is kept out of the attitude filters and pauses hump integration for
0.5 s, so it cannot start a false hump. It only shows as the `impact`
motion class; no collision event is raised, because IMU collision
detection was removed from the requirements.

**End of run: lifted or overturned.** Two independent triggers; either one
latches the stop.

- *Lifted ≥ 20 cm.* Lifted straight up with acceleration a, the accelerometer
  reads |f| = g + a whatever its orientation. So the rise is the double
  integral of |f| − g, and it needs no attitude: it works at whatever angle
  the car is held. |f| is low-pass filtered (about 140 ms) to remove driving
  vibration. The filter keeps the area under the curve, so the integral is
  only delayed. g is the filtered |f| tracked while at rest, which absorbs
  the sensor's scale error. To stop drift, integration only runs in an
  episode that starts with 40 mg of upward acceleration. The episode ends
  after 1.2 s (`LIFT_WINDOW_MS`), or once the car has dropped 5 cm below its
  start. A jolt cancels it; with `LIFT_JOLT_PAUSE=1` it only skips that
  sample, and the window keeps running.
- *Overturned.* The filtered gravity vector is more than 60° from the
  calibrated UP axis for 0.3 s. Ramps are far below that.

The latch clears once the car rests upright (within 30°) and still for 1 s.
The controller keeps its own stop until START. Host tests (`ctest -R
terrain`): lifts of 23 to 30 cm trigger in 0.46 to 0.60 s, upright,
tilted 45° or 90°, or turned upside down. Overturning 120° in place triggers
as "overturned". A 12 cm lift, a 25° tilt on the floor and a 1.4 g sideways
knock do not trigger. Five minutes of driving at 30 mg vibration produced a
false rise of 0 mm.

**Heading and turns.** The heading is tilt-compensated (east = m × a,
north = a × east) and comes from the magnetometer, since the LSM303DLHC has
no gyroscope. The turn rate is the filtered heading derivative. A turn starts
above 20 °/s, ends below 8 °/s, and produces a "turned right 89°"-style
event. Motor currents disturb the magnetometer, so treat heading as
indicative until it has been checked on the car (`HEADING_LOG=1` and
`tools/heading_check.py`). Buddy 2's encoder differential is the more
reliable turn source.

## Calibration and demo procedure

Flash the combined demo image built with the command in
[docs/DEMO_BUILD.md](../../docs/DEMO_BUILD.md) (it includes
`LIFT_MM=200 LIFT_WINDOW_MS=1200 LIFT_JOLT_PAUSE=0`).

1. Power on with the car still and level. Wait for `cal=level` in the `[B4]` line.
2. Drive over the hump. The console prints the terrain transitions and
   `[B4] hump 10 mm +/-5, peak 2 deg, 20 cm, valid`.
3. Measure the real hump with a ruler. Append the comparison to
   [TEST_LOG.md](../../testing/subsystem4/TEST_LOG.md) (test S4-U11) and
   tune `s4_config.h` from it.
4. **End the run by lifting the car** at least 20 cm in one deliberate
   movement (under about 1 s), at any angle, or by turning it over. Expect
   `[B4] END OF RUN: car lifted 2x cm (limit 20 cm)` and `[CAR] END OF RUN:
   car lifted ... cm (Buddy 4 IMU); motors stopped.` The dashboard shows "End of run · car lifted .. cm". Put it down
   upright and keep it still for 1 s (`[CAR] Buddy 4: car back on its
   wheels`), then GP20 or MQTT START runs again.

## Known limitations

- Calibrate on the floor the car will drive on. Pitch and roll are measured
  against the calibration surface, and 4 mm of raised height is only about
  2° of roll or 1.8° of pitch (130 mm caster arm). A floor tilted 1.5°
  relative to it eats most of that margin, and a hump on it reads high. The
  height integral does not drift on such a floor (host test).
- The raised-height formula assumes a rigid car whose lowest contact is on
  the floor. If the whole car is on top of a wide raised area, the attitude
  reads level; the height integral holds the episode open for up to 2 s.
- With the caster in front (`S4_CLIMB_NOSE_DOWN 1`), climbing reads
  nose-down like braking, so a hard stop can open a false episode. A ramp
  under one wheel is still detected through roll alone.
- Pitch includes linear acceleration: 0.1 g of acceleration reads as about
  5.7° of pitch. Hard acceleration while on a hump inflates the height. The
  hump speed cap limits this.
- End of run: a lift spread over more than about 1.2 s (gentler than 40 mg)
  is not counted, by design, so vibration cannot add up to a false lift. Lift
  the car deliberately. Horizontal acceleration adds a little to |f| (300 mg
  sideways reads as +44 mg), so a hard sideways knock that is not caught as a
  jolt could add some rise. A large tilt changes the accelerometer's offset
  error and so the rise estimate; the overturn trigger covers that case.
  Pickup stops have been seen on the car, but the reported lift height has
  not been compared with a ruler (S4-L01).
- The magnetometer cannot be calibrated in the current build: `s4_command()`
  is not routed from MQTT, so `faults=0x2` stays set and the heading is
  uncalibrated. Even when routed, flat-spin calibration cannot separate hard
  iron from the vertical field leaking in through a tilted mounting, and
  soft-iron distortion is not corrected.
- IMU-only pickup detection cannot catch an arbitrarily slow lift or tell
  that the car is being held still in the air. The optional optical pickup
  is meant to cover that once its threshold is measured.
- Hump thresholds and the caster arm / track values are not yet tuned
  against measured fixtures. The 20 ms period, priority and stack ran
  without problems on the car (about 3 ms processing per sample) but are
  not formally scheduled.
- The car has run Buddy 4 (calibration, terrain, hump events, pickup stops),
  but hump accuracy, pickup timing and overturn are not yet measured against
  ground truth. See [TEST_LOG.md](../../testing/subsystem4/TEST_LOG.md).
