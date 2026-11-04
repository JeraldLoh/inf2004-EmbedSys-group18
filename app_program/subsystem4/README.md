# Buddy 4: IMU, terrain and hump monitoring

Buddy 4 owns the GY-511 (LSM303DLHC accelerometer and magnetometer) and
produces the following:

- **Calibration:** level calibration that removes the board's mounting
  pitch/roll, and optional magnetometer hard-iron calibration.
- **Attitude:** pitch, roll, tilt-compensated magnetic heading, and turn rate.
- **Terrain:** the state LEVEL / ASCENDING / CREST / DESCENDING / SETTLING,
  plus a speed cap for the controller.
- **Humps:** detection with peak height, uncertainty, length and validity.
- **Motion:** classification as stationary, moving, rough or impact;
  acceleration and braking; collision detection with the side that was hit.
- **Turns:** completed-turn events with direction and angle.

All shared data uses minimum-bit packed fields (Change Requirement 1). See
[DATA_REPRESENTATION.md](DATA_REPRESENTATION.md).

Other buddies may contribute, but Buddy 4 stays accountable for this
subsystem's contracts, calibration and evidence.

## Status

| Item | State |
|---|---|
| Portable math, estimator, packing, seqlock, MQTT payloads | Implemented; host tests pass (`testing/host/test_terrain_estimator.c`, `ctest -L subsystem4`) |
| LSM303 driver and IMU task (micro T-Kernel) | Implemented; `CAR_RUN=1` and `CAR_MQTT=1` images build with no `-Wall -Wextra` warnings in Buddy 4 files; **not yet run on hardware** |
| On-target self-test S4-T01..T11 | Implemented (`make TEST=1`); not yet run |
| Lifecycle hookup (`subsystem4_init/start/test`) | Done: `car_run_target.c` inits Buddy 4 after the others and starts it after Buddy 5. Optional: a failure is printed and the car runs without it |
| Root I2C configuration | Done: I2C on, I2C1 on GP2/GP3, boot starts unit 1 `"iicb"` only (Integration step 1) |
| Controller, Buddy 2 and Buddy 1 links | Done in `CAR_RUN=1` (see "Whole-car wiring"); **not yet run on the car** |
| Buddy 3 hump pause (tribunal R2) | **Not wired**: Buddy 3 has no hump input yet (see "Open items") |
| Fault reporting | Detail mask, events and `s4_sensor_failures()`, shown in the `[B4]` console line and MQTT section 4. **Not** latched into the shared hardware fault mask (see "Whole-car wiring") |
| Thresholds in `s4_config.h` | Hump thresholds sized for the 10 mm x 195 mm demo hump by host simulation; `S4_CASTER_ARM_MM`, `S4_TRACK_MM` and `S4_CLIMB_NOSE_DOWN` are **placeholders to measure** |

## Files

All `.c` files sit directly in this folder, because the course Make rule only
builds one level below `app_program/`.

| File | Kernel? | Purpose |
|---|---|---|
| [s4_imu.h](s4_imu.h) | yes | **Public interface for other buddies**: lifecycle, requests, object IDs |
| [s4_packed.h](s4_packed.h) | no | Every packed field: shift, width, enums, fault bits, compile-time checks |
| [s4_shared.h](s4_shared.h) / [.c](s4_shared.c) | no | Seqlock snapshot, odometry input, MQTT frame/command, `terrain_report_t` adapter |
| [s4_atomic.h](s4_atomic.h) | no | 32-bit load/store and the cross-core fence |
| [terrain_estimator.h](terrain_estimator.h) / [.c](terrain_estimator.c) | no | The algorithm: calibration, filters, state machine, hump height, collision, turn |
| [imu_math.h](imu_math.h) / [.c](imu_math.c) | no | Fixed-point decode, axis mapping, atan2, isqrt, sine, heading |
| [drv_lsm303.h](drv_lsm303.h) / [.c](drv_lsm303.c) | yes | LSM303DLHC over the course `"iicb"` (I2C1) device |
| [task_imu.c](task_imu.c) | yes | The IMU task, cyclic tick, request flag, event queue, bus fault recovery |
| [s4_selftest.c](s4_selftest.c) | yes | On-target checks, built only with `TEST=1` |
| [s4_config.h](s4_config.h) | no | All tuning values, periods, priority, mounting |

"Kernel? = no" files have no RTOS or hardware dependency. That is what lets
them be tested on a PC.

## Data flow

```text
 cyclic handler (20 ms) --tick--> +------------------------------+
 any task: s4_request() --bits--> |  IMU task (only I2C1 owner)  |
 binding: read_odometry_mm() ---> |  drv_lsm303 -> s4_est_step   |
                                  +-----+--------------+---------+
                    s4_shared_publish() |              | 4-byte events
                                        v              v
        seqlock snapshot (4 packed words)      s4_mbf_events (message buffer)
           |               |            |                 |
   controller (IF-03)  Buddy 1 MQTT    any ISR      Vehicle Controller
   s4_terrain_report_  s4_telemetry_   s4_shared_   hump / collision /
   from_words()        pack() 17 B     read()       turn / fault / cal
```

## Integration steps

1. **I2C1 on GP2/GP3 (done; shared files, so get team review).**
   - `config/config_device.h`: `DEVCNF_USE_IIC` is `1`.
   - `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`: `DEVCNF_I2C1_SDA_PIN 2`,
     `DEVCNF_I2C1_SCL_PIN 3`. The old I2C1 default GP6/GP7 must not return:
     GP7 is Buddy 3's left line sensor.
   - `kernel/sysdepend/pico_rp2040/devinit.c`: starts unit 1 (`"iicb"`)
     only, so unit 0's default GP8/GP9 (Buddy 2 motor) pins are never claimed.

   Every build now claims GP2/GP3 for I2C at boot, with internal pull-ups. If
   the bus is missing, the task still runs and reports `S4_FAULT_I2C` and
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
| Buddy 4 → controller: speed cap | `controller_task_bind_terrain(b4_read_terrain)`: the controller task reads the snapshot every 10 ms tick with `s4_terrain_report_from_words()` | `vehicle_controller_update_terrain()`; line tracking takes the lowest cap. The hump cap (100 mm/s) only bites above it: the official car cruises at 60 mm/s, so today it changes nothing |
| Buddy 2 → Buddy 4: odometry | Binding `read_odometry_mm` = `b4_read_odometry()`, called by the IMU task each period | Integrates `subsystem2_motion_telemetry()` wheel speeds over its sample time, each wheel signed by its last non-zero target, so a pivot adds no distance. Buddy 2's code is unchanged |
| Buddy 4 → Buddy 1: MQTT | `b4_wifi_telemetry()` sends `s4_telemetry_pack()` as section 4 every 200 ms (`CAR_MQTT=1`); the controller's terrain state also fills section 2 | The dashboard's `decodeBuddy4()` already matches `s4_packed.h` |
| Buddy 4 → console | `[B4]` line every second next to `[CAR]`, plus event lines (terrain, hump, calibration) | See "Reading the serial log" |
| Buddy 4 → controller: faults | **Not mapped.** | Any bit in the shared hardware fault mask blocks junction handling and MQTT START, so an unplugged IMU would stop a run. The provisional `HARDWARE_FAULT_IMU/MAG` bits stay unused until the team agrees a non-blocking class |

### Reading the serial log

```text
[B4] valid=1 cal=level terrain=LEVEL cap=0 pitch=0.5 roll=-0.5 deg vib=8 mg motion=1 odom=1 faults=0x2 fail=0x0 live_h=0 mm | last hump 0 mm +/-0 peak 0 deg 0 cm invalid/none #0 | 1450 us/sample
[B4] terrain ASCENDING, cap 100 mm/s
[B4] terrain CREST, cap 100 mm/s
[B4] terrain DESCENDING, cap 100 mm/s
[B4] terrain SETTLING, cap 100 mm/s
[B4] hump 10 mm +/-5, peak 2 deg, 27 cm, valid
[B4] terrain LEVEL, cap 0 mm/s
```

- `cal=RUNNING-keep still` / `none`: level calibration is running or failed; it retries every 2 s once the car is still.
- `faults` is the 7-bit detail mask: 0x01 I2C, 0x02 magnetometer, 0x04 stale, 0x08 uncalibrated, 0x10 saturated, 0x20 tilt, 0x40 collision. 0x2 alone is normal until the magnetometer is calibrated or if it is absent.
- `vib` is the vibration level while driving. Host simulation shows no false humps up to about 30 mg of noise; if it reads well above that and you see `INVALID` humps on flat floor, raise `S4_RAISE_ON_MM` to 5 or `S4_PITCH_SMOOTH_SHIFT` to 4. At 45 mg, simulation gave about one INVALID episode a minute and no valid ones.
- `odom=1` means the height came from Buddy 2's wheel speeds; 0 means the fallback speed was used.

### Open items

- **Buddy 3 hump pause (tribunal R2).** Status bit 23 is published, but Buddy 3 has no input for it, and the controller does not hold steering on a hump. On a 10 mm hump the line sensors tilt by only about 4°, so this may not be needed. Buddy 3 should decide after a hump run.
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
| B4 → Buddy 3 | status word bit 23 (hump flag) | Published, but **not consumed yet**: Buddy 3 has no hump input (tribunal decision R2, see "Open items"). |
| B4 → controller | `s4_sensor_failures()` (`S4_FAILURE_IMU`, `S4_FAILURE_MAG`) | Persistent sensor failures, latched until reset. Shown in the `[B4]` line (`fail=`); not mapped into the shared hardware fault mask (it would block junctions and START). The detail mask (status bits 19..13) and `S4_EV_FAULT` events keep the full evidence; UNCAL, SATURATED, TILT and COLLISION are status, not failures. |
| Any → B4 | `s4_request(S4_REQ_*)` | ISR-safe. |

Week 8 tribunal decisions applied here:

- **One DATA_VALID bit per message** (status bit 31) replaces per-field
  invalid codes, and CLAMPED (bit 30) flags an out-of-range value.
- **Explicit hump and collision flags.**
- **A 3-bit motion class:** stationary, moving, accelerating, braking, turn
  left, turn right, rough, impact.
- **A 2-bit calibration status:** none, running, level, level plus
  magnetometer.
- **Turn rate:** ±250 °/s in 2 °/s steps, which fits 8 bits.
- **Shared fault mask:** the tribunal agreed IMU faults go in the shared
  bitmask. The team's [contracts/hardware_faults.h](../contracts/hardware_faults.h)
  makes the Vehicle Controller its single owner, so Buddy 4 never writes it.
  Buddy 4 reports evidence instead (detail mask and `S4_EV_FAULT` events),
  and the controller task still needs an event to receive it.

Collision is reported as an event, a flag and a latched bit in Buddy 4's
detail mask. Whether it triggers an urgent stop (IF-07) is the controller's
policy decision.

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

Buddy 4 reports the larger of the peak raised height and the integral. On
long ramps the integral wins, on humps the attitude. Pitch and roll come
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
INVALID, tilt over 25°, and the impact hold all use 100 mm/s. The official
car (`CAR_RUN=1`) cruises at 60 mm/s, so the caps only take effect on faster
profiles.

**Motion and collision.** The vibration level is the filtered deviation of
|a| from 1 g. A horizontal jolt above 1000 mg relative to the slow filter is
a collision. The side that was hit comes from the jolt direction, because the
car is pushed away from the impact. Collision samples are kept out of the
attitude filters, so a hit cannot start a false hump.

**Heading and turns.** The heading is tilt-compensated (east = m × a,
north = a × east) and comes from the magnetometer, since the LSM303DLHC has
no gyroscope. The turn rate is the filtered heading derivative. A turn starts
above 20 °/s, ends below 8 °/s, and produces a "turned right 89°"-style
event. Motor currents disturb the magnetometer, so treat heading as
indicative until it has been checked on the car. Buddy 2's encoder
differential is the more reliable turn source.

## Calibration and demo procedure

1. Power on with the car still and level. Wait for `cal=level` in the `[B4]` line.
2. Optional magnetometer calibration (needs `s4_command()` routed from MQTT, not done yet): send command 3, spin the car slowly
   through at least 360° on the floor, then send command 4. Look for
   `[B4] magnetometer calibration done`.
3. Drive over the hump. The console prints the terrain transitions and
   `[B4] hump 10 mm +/-5, peak 2 deg, 27 cm, valid`.
4. Measure the real hump with a ruler. Append the comparison to
   `testing/subsystem4/TEST_LOG.md` (test S4-H01) and tune `s4_config.h`
   from it.

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
- Flat-spin magnetometer calibration cannot separate hard iron from the
  vertical field leaking in through a tilted mounting. Mount the board flat.
  Soft-iron distortion is not corrected.
- Thresholds are uncalibrated, and the 20 ms period, priority and stack are
  provisional until the team schedule (TEAM.md section 4) is agreed and
  measured.
- Nothing here has run on hardware yet. Host tests prove the logic, not the
  sensor, the wiring or the timing.
