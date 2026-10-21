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
| Portable math, estimator, packing, seqlock, MQTT payloads | Implemented; host tests pass (`testing/host`) |
| LSM303 driver and IMU task (micro T-Kernel) | Implemented; compiles warning-free with the course ARM flags; **not yet run on hardware** |
| On-target self-test S4-T01..T10 | Implemented (`make TEST=1`); not yet run |
| Lifecycle hookup (`subsystem4_init/start`) | **Not done**: the placeholder files were removed; see Integration step 3 |
| Root I2C configuration | Done: I2C1 on GP2/GP3, unit 1 `"iicb"` started at boot (Integration step 1) |
| Thresholds in `s4_config.h` | Uncalibrated starting values |

## Files

All `.c` files sit directly in this folder, because the course Make rule only
builds one level below `app_program/`. For a plain-language tour of every
file, see [FILES.pdf](FILES.pdf).

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
 Buddy 2: s4_post_odometry() ---> |  drv_lsm303 -> s4_est_step   |
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
     `DEVCNF_I2C1_SCL_PIN 3`.
   - `kernel/sysdepend/pico_rp2040/devinit.c`: starts unit 1 (`"iicb"`)
     instead of unit 0. Unit 0 is never started, so its default GP8/GP9
     (Buddy 2 motor) pins are never claimed.

   Booting now claims GP2/GP3 for I2C, with internal pull-ups enabled. If the
   bus is missing, the task still runs and reports `S4_FAULT_I2C` and terrain
   `UNKNOWN`/`INVALID` with the degraded speed cap.
2. **Wiring.** Connect the GY-511 to Robo Pico **Grove port 2** (or the
   Maker/QWIIC port, which shares GP2/GP3): GP2 to SDA, GP3 to SCL, 3V3 to
   VIN, and GND. Mount it flat, near the centre of the car. The default axes
   assume X points right, Y forward and Z up (the briefing's IMU figure). If
   the board is rotated, change `S4_AXIS_*` in `s4_config.h`.
3. **Lifecycle.** `app.h` still declares `subsystem4_init/start/test`, but
   their placeholder file was removed. When the team turns on the lifecycle,
   define them (for example in `task_imu.c`) as:

   ```c
   EXPORT ER subsystem4_init(void)  { return s4_imu_init(); }   /* creates flag, mbf, cyclic */
   EXPORT ER subsystem4_start(void) { return s4_imu_start(); }  /* creates task, starts tick */
   /* subsystem4_test(): call s4_imu_selftest(); */
   ```

   `s4_imu_init()` rolls back its own objects if a creation fails.
   `usermain()` must still run every subsystem's init before any start.
4. **Record the pins and resources in `TEAM.md`.** Task priority 12
   (provisional), 2 KB stack, and 4 kernel objects: 1 event flag, 1 message
   buffer, 1 cyclic handler, 1 task. The task has no core affinity.

## Interfaces for the other buddies

| From / to | Call | Notes |
|---|---|---|
| Buddy 2 → B4 | `s4_post_odometry(cumulative_mm)` | From **one** task, every control cycle, **even when stopped**. Pass the signed average path distance of both wheels. Call `s4_post_odometry_invalid()` on an encoder fault. |
| B4 → controller | `s4_shared_read(words)` + `s4_terrain_report_from_words()` | Gives the existing `terrain_report_t` for `vehicle_controller_update_terrain()`. The speed cap composes as "lowest cap wins". |
| B4 → controller | `tk_rcv_mbf(s4_mbf_events, &msg, tmo)` | 4-byte events, single reader. When the queue is full the oldest event is dropped (`s4_events_dropped`). |
| B4 → Buddy 1 | `s4_telemetry_pack(buf, 17)` | Binary frame; decode with the tables in DATA_REPRESENTATION.md. |
| Buddy 1 → B4 | `s4_command(byte)` | After Buddy 1 has validated authority: 1 recalibrate, 2 clear latched, 3/4 magnetometer calibration start/stop, 5 print the observed ranges and sample timing. |
| B4 → Buddy 3 | status word bit 23 (hump flag) | While it is set, pause the line-lost timeout and barcode decoding and hold the last steering (tribunal decision R2). |
| B4 → everyone | `fault_mask_read()` & `FAULT_MASK_B4` | Buddy 4's 4 bits of the team fault word: IMU, not calibrated, magnetometer, collision. |
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
- **Shared fault word:** IMU faults go in the team-wide word in
  [contracts/fault_mask.h](../contracts/fault_mask.h). Each buddy owns a
  4-bit field, and every write is atomic, so no task can lose another task's
  bits.

Collision is reported as an event, a flag and a latched fault bit. Whether
it triggers an urgent stop (IF-07) is the controller's policy decision.

## How it works

**Sampling.** Every 20 ms the task performs one 7-byte burst read (accel
status and X/Y/Z) and one 6-byte magnetometer read. The accelerometer runs at
±4 g, high resolution, 100 Hz output, with block data update, which leaves
headroom for 3 g shocks. All maths is integer: the RP2040 has no FPU.

**Level calibration** runs at start-up and on request. It averages 64 samples
(1.28 s) and requires each axis to stay within a 40 mg spread. Any movement
restarts the window, and after 5 retries it fails. The resting gravity vector
gives the mounting pitch/roll offsets and the gravity magnitude. **Keep the
car still on a level floor until "S4 level calibration done" prints.**

**Hump height.** The briefing suggests h ≈ v²/2g from a vertical
acceleration impulse. That requires integrating noisy acceleration twice,
which drifts badly with a ±several-mg MEMS sensor. Buddy 4 instead integrates
the slope along the travelled path:

  **height = Σ sin(pitch) × Δdistance**

Δdistance comes from Buddy 2's odometry, which is exact along the surface.
Without odometry it falls back to `S4_FALLBACK_SPEED_MM_S` × dt, and the
uncertainty is widened. Integration starts as soon as the pitch leaves the
level band, so the climb before the state is confirmed is not lost.

Each result reports:

- the peak height;
- the uncertainty: base + residual height at the end + 1% of the path, plus
  half the height when distance was estimated;
- the length and the peak pitch;
- a valid bit. Valid requires an ascent and a descent, height ≥ 5 mm, length
  between 80 and 1500 mm, a small residual, and no fault during the episode.

**Terrain state machine.** Pitch is in tenths of a degree. Each transition
needs `S4_CONFIRM_SAMPLES` consecutive samples.

| From | To | When |
|---|---|---|
| LEVEL | ASCENDING / DESCENDING | pitch ≥ +5° / ≤ −5° (a dip or step down goes straight to DESCENDING) |
| ASCENDING | CREST | pitch < +2.5° |
| CREST | DESCENDING / ASCENDING | pitch ≤ −5° / ≥ +5°; flat top for over 2 s ends as a "step" (invalid hump) |
| DESCENDING | SETTLING | pitch > −2.5° |
| SETTLING | LEVEL (publish result) | 300 ms settled; a new climb publishes and starts the next hump |
| any | INVALID | I2C error or stale data; returns to LEVEL only once level again |
| any | UNKNOWN | not calibrated, or recalibrating |

**Speed caps.** LEVEL has no cap. The hump states use 150 mm/s. UNKNOWN,
INVALID, tilt over 25°, and the impact hold all use 100 mm/s.

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

1. Power on with the car still and level. Wait for `S4 level calibration done`.
2. Optional magnetometer calibration: send command 3, spin the car slowly
   through at least 360° on the floor, then send command 4. Look for
   `S4 magnetometer calibration done`.
3. Drive over the hump. The console prints the terrain transitions and
   `S4 hump 39 mm +/-7, peak 17 deg, 44 cm, valid`.
4. Measure the real hump with a ruler. Append the comparison to
   `testing/subsystem4/TEST_LOG.md` (test S4-H01) and tune `s4_config.h`
   from it.

## Known limitations

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
