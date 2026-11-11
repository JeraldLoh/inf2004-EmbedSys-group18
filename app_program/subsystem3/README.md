# Buddy 3 - Line, Barcode, and Junction Perception

Primary responsibility: IR acquisition, line evidence, steering proposals,
barcode decoding, and junction observations.

This subsystem publishes evidence and never writes motor PWM. Other buddies
may contribute, but Buddy 3 remains accountable for its contracts, calibration,
and evidence. Save evidence in `../../testing/subsystem3/`.

Status: DRAFT. The host tests and the `S3_BENCH` images were removed on
2026-10-29. Nothing has been run on the car yet.

## Pins (from `project_config/pinmap.h`)

| Pin | Use |
|---|---|
| GP0 | Right line sensor, digital D0 (also UART0 TX: use USB console, `CRASH_REPORT=0`) |
| GP7 | Left line sensor, digital D0 |
| GP26 / ADC0 | Barcode sensor, analogue A0 |
| GP28 / ADC2 | Left line sensor, analogue A0, `LINE_PID=1` only (isolate the VBAT-sense jumper) |

## Files

| File | Runs on | Purpose |
|---|---|---|
| `drv_ir.c/.h` | Pico | GPIO, ADC and microsecond timer access |
| `line_follow.c/.h` | Pico + host | Filter, steering, junction detection, probe, reacquire |
| `line_pid.c/.h` | Pico + host | Opt-in PID steering from the left line sensor's analogue output |
| `barcode_sensor.c/.h` | Pico + host | ADC to black/white: calibration, hysteresis, adaptation |
| `barcode_decoder.c/.h` | Pico + host | Full Code 39 (A-Z, 0-9), both directions, validated |
| `barcode_map.c/.h` | Pico + host | Letter-to-command map, set at run time from the dashboard |
| `subsystem3.c/.h` | Pico | Task, cyclic handler, published interface |

## Behaviour

- Steering: -100..+100, ramps up the longer the car is off centre, short
  counter-steer after each correction (damping).
- Line readings use a 2-of-3 majority filter.
- Junction: both sensors black for 30 ms. A barcode read before it decides the
  direction. With no barcode, the controller runs the probe (see
  `subsystem3.c`): "+" gives STRAIGHT, T gives RIGHT.
- Barcodes decode in either direction (e.g. after a U-turn). Reads with the
  wrong number of wide elements or a wide inter-character gap are rejected.
- `line_status.barcode_active` is 1 while a barcode is read: the controller
  should hold a steady speed until it clears.
- `subsystem3_reacquire()` searches for the line after a bypass.
- Sensor polarity: `S3_LINE_BLACK_LEVEL` in `drv_ir.h`.
- No barcode calibration at startup: fixed thresholds (350/450,
  `S3_BARCODE_FIXED`) apply from power-on and the console prints
  `[S3] Barcode ready: fixed thresholds 350/450 (no calibration)`. Lower the
  sensor to about 2-3 mm above the floor. `LINE_PID=1` builds still run a 3 s
  sweep at startup, for the line PID.

## Line PID (opt-in, `LINE_PID=1`)

Not run on the car yet. Build with `make ... CAR_RUN=1 LINE_PID=1`
(image `..._carrun_linepid.uf2`; may be combined with the default CAR_RUN
trial flags).

- Wiring: no extra sensor. Add one wire from the LEFT line module's **A0**
  pin to GP28; its D0 stays on GP7 (the module drives both at once). The
  module must run from 3.3 V, or A0 can exceed the ADC's 3.3 V limit.
  Using the right module instead: `-DLINE_PID_LEFT_EDGE=0`.
  Module reads lower on black: `-DLINE_PID_BLACK_IS_HIGH=0`.
- The PID holds the left sensor half on the tape's left edge, so the car runs
  a few mm right of where the digital steering holds it. The right sensor
  stays on white; junction crossbars still read both black.
- Calibration: in the same 3 s "sweep now" window as the barcode, also slide
  the car sideways across the tape. Prints `[S3] Line PID OK: white W black B`
  or `FAILED ... using digital steering`.
- The PID steers only in CENTRED / STEER_LEFT / STEER_RIGHT. Junctions,
  probe, reacquire and line loss still come from the digital sensors.
  If calibration fails or an ADC read fails, the digital steering is used.
- Gains (x100, edit the defaults in `line_pid.h`): `LINE_PID_KP_X100` 25,
  `LINE_PID_KI_X100` 0, `LINE_PID_KD_X100` 0, `LINE_PID_I_LIMIT` 30.
  Kp 80 with Kd 300 weaved on a straight (2026-11-04): the edge spans only
  ~5 mm, so small gains already steer hard. D uses the smoothed error over
  `LINE_PID_D_WINDOW_MS` (10 ms), so ADC noise does not make it twitch.
  Tune Kp first (Ki = Kd = 0) until it follows but weaves, then Kd until the
  weave stops, then a little Ki only if it sits off the edge on curves.
- `S3_DEBUG_RAW=1` adds a `[S3 PID] adc= dark= err= cal=` line every 100 ms.
- Host tests: `line_pid_host_tests` and `line_pid_integral_host_tests` in
  `testing/host`.

## Demo day: setting the barcode letters over MQTT

The letters are announced on the day, so nothing is hardcoded. Use the
dashboard's **05 Barcode routes** panel: type a letter, pick Left, Right,
Straight or U-turn, add the rest, then **Send to rover**. It works before
`START` or mid-run; the next barcode read uses the new map. **Read from rover**
shows what the car actually holds, and the panel says whether your draft is in
sync. The draft is kept in the browser between page loads.

Wire format (binary, on `/command`, see `subsystem1/buddy1_protocol.h`):

| Command | Bytes | Layout |
|---|---|---|
| MAP_SET | 22 | `B1 02 03 03` + command ID (u32 LE) + 14 map bytes |
| MAP_SHOW | 8 | `B1 02 03 04` + command ID (u32 LE) |

The 14 map bytes are 36 entries (A-Z then 0-9) of 3 bits each, LSB-first:
0 none, 1 left, 2 right, 3 straight, 4 U-turn (`route_command_t`). MAP_SET
replaces the whole table. Any entry above 4, nonzero spare bits, a zero ID or a
wrong length rejects the whole command and nothing changes. The reply on
`/command_result` is JSON with `"status":"applied"` and `"map"`: the 28 hex
characters of the table the car now holds (power-on map A=L B=R C=S D=U is
`d108000000000000000000000000`).

Applying runs on the comms task. Each entry is one byte in `barcode_map.c`, so
the S3 task sees either the old or the new command for a letter, never a torn
value. A letter that is read but not mapped is treated like no barcode (the
junction probe runs).

The map is in RAM, so it resets to the defaults on power-up: send it again
after any reset.

## Integration checklist (shared items, need team agreement)

- [ ] `config/config.h`: `CNF_TIMER_PERIOD` 10 -> 1.
      `CAR_RUN=1` already builds with 1 ms (D11)
- [x] `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h`: `DEVCONF_ADC_PIN_INIT_0` TRUE
      in `CAR_RUN` builds only (GP26 analogue mode)
- [x] `pinmap.h` updated to the agreed map
- [ ] `app_main.c` calls `subsystem3_init()` / `subsystem3_start()`; priority agreed
- [x] Controller: route and junction handling, probe handshake with Buddy 2's
      pivot, slow speed on `barcode_active` (`CAR_RUN`, decision D14; not yet
      run on the car)
- [ ] Buddy 5: call `subsystem3_reacquire()` at the end of a bypass
- [x] Buddy 1: binary MAP_SET / MAP_SHOW on `/command` applied to the barcode map (dashboard panel 05)

## Data representation (Change Requirement 1)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| `line_status.left_black` | 0-1 | 1 | bit-field |
| `line_status.right_black` | 0-1 | 1 | bit-field |
| `line_status.state` | 5 states | 3 | bit-field (`line_state_t`) |
| `line_status.last_side` | 3 values | 2 | bit-field (`line_side_t`) |
| `line_status.barcode_active` | 0-1 | 1 | bit-field |
| **`line_status_t` total** | | **8** | **1 byte, atomic single-byte store** |
| Steering proposal | -100..+100 | 8 | `int8_t` (201 values need 8 bits) |
| Line filter history (each side) | last 3 samples | 3 | low 3 bits of `uint8_t` |
| Junction type | 3 values | 2 | `junction_type_t` |
| Barcode edge (ISR to task) | time + level | 32 | `uint32_t`: bits 31..1 time in us, bit 0 level |
| Barcode ADC sample | 0-4095 | 12 | `uint16_t`, masked `& 0x0FFF` |
| Barcode route map entry (MQTT) | 0-4 (none, L, R, S, U) | 3 | 36 entries x 3 bits = 14 bytes; MAP_SET is 22 bytes vs about 30 for `MAP K=L Q=R Z=S M=U` text |

Still to do for Week 13: RAM and message-size before/after measurements, and
packing `route_instruction_t` / `junction_observation_t` (shared contracts,
needs team agreement).
