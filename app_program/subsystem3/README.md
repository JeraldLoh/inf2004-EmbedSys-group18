# Buddy 3 - Line, Barcode, and Junction Perception

Buddy 3 reads the two line sensors and the barcode sensor. It publishes a
steering proposal, the line status, decoded barcode routes and junction
observations. It never writes motor PWM: the Vehicle Controller turns its
evidence into wheel targets for Buddy 2. Save evidence in
`../../testing/subsystem3/`.

This describes the firmware on `main` at `ee0e70d` (2026-11-25). Numbers are
the `CAR_RUN=1` settings unless stated otherwise.

## Status

- Runs in the combined demo firmware ([`docs/DEMO_BUILD.md`](../../docs/DEMO_BUILD.md)):
  80 mm/s cruise, digital line steering (`LINE_PID=0`), damping 40, fixed
  barcode thresholds, live barcode map from the dashboard.
- On-car evidence ([`testing/DEMO_EVIDENCE.md`](../../testing/DEMO_EVIDENCE.md)):
  - E1: operator observed good tracking on straights and bends in both
    directions (80 mm/s baseline, `da4e9d9`).
  - E2 to E4: complete 29-element Code 39 decodes on the track, mapped turns
    completed (including U-turn, command 4), and some failed captures (for
    example a 14-element capture rejected as `weak-start-pattern`).
- Still open: decode rate per letter (S3-T03), junction count on a fixture
  (S3-T05), decode-to-junction timing (S3-T07), the physical barcode re-read,
  and line PID on the car. See [`TEST_LOG.md`](../../testing/subsystem3/TEST_LOG.md).

## Pins (from `project_config/pinmap.h`)

| Pin | Use |
|---|---|
| GP0 | Right line sensor, digital D0 (also UART0 TX: use USB console, `CRASH_REPORT=0`) |
| GP7 | Left line sensor, digital D0 |
| GP26 / ADC0 | Barcode sensor, analogue A0 |
| GP28 / ADC2 | Left line sensor, analogue A0, `LINE_PID=1` only (isolate the VBAT-sense jumper) |

## Files

| File | Host-tested | Purpose |
|---|---|---|
| `drv_ir.c/.h` | no | GPIO, ADC and microsecond timer access |
| `line_follow.c/.h` | no | Filter, steering, junction detection, probe, reacquire |
| `line_pid.c/.h` | yes | Opt-in PID steering from the left line sensor's analogue output |
| `barcode_sensor.c/.h` | no | ADC to black/white: thresholds, hysteresis, adaptation |
| `barcode_decoder.c/.h` | yes | Code 39 (A-Z, 0-9), both directions, capture trace, re-read signature |
| `barcode_map.c/.h` | yes | Four direction slots, set at run time from the dashboard |
| `subsystem3.c/.h` | no | Task, published interface, console and live status |

`line_follow`, `barcode_sensor` and `barcode_map` have no `tk_*` calls, so they
still build on a PC. Their old host suite (`test_subsystem3_sensors.c`,
S3-H01..S3-H16) was removed; the current host tests are listed under
[Build and test](#build-and-test).

## Task

- `s3_task`: priority 10, 4096 B stack, one sample every 1 ms. This needs the
  1 ms kernel tick (`CNF_TIMER_PERIOD=1`), which every `CAR_RUN` build sets.
- Started by `car_run_target.c` (`subsystem3_init()`, then `subsystem3_start()`).
- The millisecond clock is wrap-safe: it no longer jumps back to 0 after
  71.6 min of uptime (R3).
- In `CAR_RUN` builds Buddy 3 owns the ADC and polls each conversion directly,
  with a 250 us timeout. A failed read skips that sample and prints
  `[S3 ADC] read failed ...`; recovery prints `[S3 ADC] acquisition restored`.

## Line following

- Each side uses a 2-of-3 majority filter, so a single-sample glitch is ignored.
- Steering runs -100..+100 and ramps up the longer the car is off centre
  (`LINE_STEER_RAMP_MS=6` in `CAR_RUN`).
- While centred, it holds a steer of 20 toward the side the line was last seen
  (`LINE_HOLD_STEER=20`).
- Damping: after each correction, a short counter-steer lasting half as long
  as the correction, capped at `LINE_DAMP_MS` (40 ms). If that half is under
  `LINE_DAMP_MIN_MS` (15 ms), there is no counter-steer. Strength is
  `LINE_DAMP_STEER`: 15 by default, 40 in the demo command.
- Both sensors on white is normal: they straddle a straight line. So
  both-white on its own does not prove the line is lost.
- `subsystem3_reacquire()` searches for the line. The controller calls it at
  the end of Buddy 5's obstacle bypass, with the side to search.
- Sensor polarity: `S3_LINE_BLACK_LEVEL` in `drv_ir.h` (1 = reads high on black).

## Junctions

- Both sensors black for 30 ms (`LINE_JUNCTION_CONFIRM_MS`) gives one junction
  observation.
- A decoded barcode route stays valid until the next junction (no timer). The
  controller applies it once.
- Demo profile (`CAR_B2_TRIAL=1`): at each junction the car stops for
  `JUNCTION_PAUSE_MS` (1000 ms). It then runs the live barcode route, or drives
  straight on if there is none.
- The older no-barcode probe is still in the code ("+" gives STRAIGHT, T gives
  RIGHT, inferred route valid 10 s). The demo profile does not use it.

## Barcode reading

- Fixed thresholds 350/450 apply from power-on (`S3_BARCODE_FIXED=1`), with no
  calibration sweep. The console prints
  `[S3] Barcode ready: fixed thresholds 350/450 (no calibration)`.
  Mount the sensor about 2-3 mm above the floor.
- One Code 39 character framed by `*` start/stop characters: 29 elements. It
  decodes in either direction (for example after a U-turn). Reads with the
  wrong number of wide elements or a wide gap between characters are rejected.
- A capture may last up to 10 s (`BARCODE_MAX_SCAN_US`), enough for the largest
  printed copy (about 270 mm, 5.4 s at 50 mm/s).
- `line_status.barcode_active` is 1 while a barcode is being read. The
  controller holds a steady speed until it clears
  (`BARCODE_STEER_PERCENT` can also scale steering during the read).
- A decoded letter that is not mapped gives no route and prints
  `[S3] Barcode 'X' read, but it is not mapped`.

### Barcode re-read (`BARCODE_RETRY=1`)

- A failed capture publishes a "read failed" event
  (`subsystem3_take_barcode_failure()`) only if it looks like a real barcode:
  a start character, a narrow gap and part of the next character
  (`barcode_decoder_retry_signature()`, at least 13 elements). It must also not
  be at a junction, have no missed ADC samples, come before any route was read
  on this approach, and use an unchanged map. `[S3 RETRY]` prints the reason
  when it does not qualify (for example `weak-start-pattern`).
  `BARCODE_RETRY_MIN_ELEMENTS` is still a Make knob but the code no longer
  uses it.
- The controller then reverses to just before the capture started
  (`BARCODE_RETRACE_MARGIN_MM`, 25 in the demo command), calls
  `subsystem3_barcode_reset()` with the car stopped, and passes the barcode
  again at `BARCODE_REREAD_MM_S` (80 in the demo command).
- A route decoded backwards during the retrace is discarded. One re-read per
  junction approach.
- On in the combined demo command, off (`BARCODE_RETRY=0`) in the packaged
  baseline. Not yet verified on the car.

## Demo day: setting the barcode routes over MQTT

The letters are announced on the day, so nothing is hardcoded and the
power-on map is **empty**. Send the map after every reset.

In the dashboard's **05 Barcode routes** panel, type one letter or digit for
each direction (Left, Right, Straight, U-turn), then **Send to rover**. It works
before `START` or mid-run; the next barcode read uses the new map.
**Read from rover** shows what the car holds, and the panel says whether your
draft is in sync.

Wire format (binary, on `/command`, see `subsystem1/buddy1_protocol.h`):

| Message | Bytes | Layout |
|---|---|---|
| MAP_SET | 22 | `B1 02 03 03` + command ID (u32 LE) + 14 map bytes |
| MAP_SHOW | 8 | `B1 02 03 04` + command ID (u32 LE) |
| Reply on `/command_result` (applied) | 26 | `B1 01 06` + flags + command ID (u32 LE) + map revision (u32 LE) + 14 map bytes |
| Reply on `/command_result` (rejected) | 8 | `B1 01 06` + flags + command ID (u32 LE), no map |

- The 14 map bytes are 36 entries (A-Z then 0-9) of 3 bits each, LSB-first:
  0 none, 1 left, 2 right, 3 straight, 4 U-turn (`route_command_t`).
- MAP_SET replaces the whole map. The car rejects the whole command, and
  changes nothing, if two characters share a direction, an entry is above 4,
  spare bits are nonzero, the ID is zero or the length is wrong.
- Inside the car the map is one 24-bit atomic word: four 6-bit slots, one per
  direction (0 = unassigned, 1-36 = A-Z then 0-9). A revision counter (odd
  while publishing) lets readers detect a change mid-read, so the S3 task
  sees the old map or the new one, never a mix.
- Buddy 1's comms task is the only writer. The map is in RAM and resets on
  power-up.

## Console and telemetry

| Line | When |
|---|---|
| `[S3] Route 'X' -> command N` | A barcode decoded to a mapped route |
| `[S3] (barcode read in reverse)` | That barcode was read backwards |
| `[S3] Barcode NOT decoded: N elements (need 29), ms: ...` | A capture failed |
| `[S3 CAP] id= ok= n= pass= fail= us=...` | Every finished capture, pass or fail. Replay with `tools/barcode_replay.py` |
| `[S3 RETRY] id= decision=` | Re-read decision for a failed capture |
| `[GROUND] L= R= adc= valid= seq=` | 10 Hz, with `S3_GROUND_LOG=1` (on in the demo command) |
| `[S3 TASK] t_ms= loops= stage=` | Once a second, printed by Buddy 2's task |
| `[S3 RAW] ...`, `[S3 WAVE] ...` | Only with `S3_DEBUG_RAW=1` |

Buddy 1's live status carries the Buddy 3 fields to the dashboard: line
paused, barcode state, last character and route, scan and failure counts,
and the map revision.

## Line PID (opt-in, `LINE_PID=1`)

Not run on the car. The demo uses digital steering (`LINE_PID=0`). Build with
`make ... CAR_RUN=1 LINE_PID=1`.

- Wiring: add one wire from the LEFT line module's **A0** pin to GP28; its D0
  stays on GP7. The module must run from 3.3 V, or A0 can exceed the ADC's
  3.3 V limit. Using the right module instead: `-DLINE_PID_LEFT_EDGE=0`.
  Module reads lower on black: `-DLINE_PID_BLACK_IS_HIGH=0`.
- The PID holds the left sensor half on the tape's left edge. The right sensor
  stays on white; junction crossbars still read both black.
- `LINE_PID=1` builds run a 3 s sweep at startup: slide the car sideways across
  the tape. Prints `[S3] Line PID OK: white W black B` or
  `FAILED ... using digital steering`. Barcodes are ignored until it succeeds.
- The PID steers only in CENTRED / STEER_LEFT / STEER_RIGHT. Junctions, probe,
  reacquire and line loss still come from the digital sensors. If calibration
  or an ADC read fails, digital steering is used.
- Gains (x100, defaults in `line_pid.h`): `LINE_PID_KP_X100` 35,
  `LINE_PID_KI_X100` 0, `LINE_PID_KD_X100` 0. Kp 80 with Kd 300 weaved on a
  straight (2026-11-04): the edge spans only about 5 mm, so small gains
  already steer hard. Tune Kp first, then Kd until the weave stops, then a
  little Ki only if it sits off the edge on curves.
- `S3_DEBUG_RAW=1` adds `[S3 PID] adc= dark= err= cal=` every 100 ms.

## Build and test

- Car: use the combined demo command in
  [`docs/DEMO_BUILD.md`](../../docs/DEMO_BUILD.md), from `build_make/`.
- Host tests (CMake, `testing/host`): `perception_math_host_tests` (barcode
  decoder and re-read signature), `buddy1_map_tests` (MAP_SET / MAP_SHOW
  through Buddy 1 into the barcode map), `line_pid_host_tests` and
  `line_pid_integral_host_tests`.
- Without CMake, from the repository root:

```
gcc -std=c11 -Wall -Wextra -Werror -Wpedantic -UNDEBUG \
  testing/host/test_perception_math.c \
  app_program/subsystem3/barcode_decoder.c app_program/subsystem3/barcode_map.c \
  app_program/subsystem4/imu_math.c -lm -o perception && ./perception
```

All four passed on `ee0e70d` (2026-11-25, gcc, host only). The full commands
are in [`TEST_LOG.md`](../../testing/subsystem3/TEST_LOG.md#host-tests-no-hardware).

## Data representation (Change Requirement 1)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| `line_status.left_black` / `right_black` | 0-1 | 1 each | bit-field |
| `line_status.state` | 5 states | 3 | bit-field (`line_state_t`) |
| `line_status.last_side` | 3 values | 2 | bit-field (`line_side_t`) |
| `line_status.barcode_active` | 0-1 | 1 | bit-field |
| **`line_status_t` total** | | **8** | **1 byte, atomic single-byte store** |
| Steering proposal | -100..+100 | 8 | `int8_t` |
| Line task private state | history 0-7 per side, previous state 0-4, 4 flags | 16 | bit-fields in one `uint16_t` unit |
| Barcode edge (ISR to task) | time + level | 32 | `uint32_t`: bits 31..1 time in us, bit 0 level |
| Barcode ADC sample | 0-4095 | 12 | `uint16_t`, masked `& 0x0FFF` |
| Barcode route map (in the car) | 4 slots, 0-36 each | 24 | one 32-bit atomic word, 6 bits per slot |
| Barcode route map (MQTT) | 36 entries, 0-4 each | 108 | 14 bytes; MAP_SET 22 B, reply 26 B binary (was up to 131 B JSON) |

Before and after RAM and message sizes are in
[`docs/protocol/STATIC_RAM_COMPARISON.md`](../../docs/protocol/STATIC_RAM_COMPARISON.md);
all field ranges are in
[`docs/protocol/DATA_REPRESENTATION.md`](../../docs/protocol/DATA_REPRESENTATION.md).

## Known limits

- Both-white cannot tell "straddling the line" from "lost the line", so line
  loss recovery is off in the baseline (`LINE_LOST_MS=0`).
- Fixed barcode thresholds assume the mounting height and lighting used in
  testing. Low contrast or a raised sensor gives failed captures.
- The barcode map is lost on reset and starts empty.
- Barcode re-read and line PID compile and pass host tests but are not
  verified on the car.
