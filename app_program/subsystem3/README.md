# Buddy 3 - Line, Barcode, and Junction Perception

Primary responsibility: IR acquisition, line evidence, steering proposals,
barcode decoding, and junction observations.

This subsystem publishes evidence and never writes motor PWM. Other buddies
may contribute, but Buddy 3 remains accountable for its contracts, calibration,
and evidence. Save evidence in `../../testing/subsystem3/`.

Status: DRAFT. Host tests pass; nothing has been run on the car yet.

## Pins (from `project_config/pinmap.h`)

| Pin | Use |
|---|---|
| GP0 | Right line sensor, digital D0 (also UART0 TX: use USB console, `CRASH_REPORT=0`) |
| GP7 | Left line sensor, digital D0 |
| GP26 / ADC0 | Barcode sensor, analogue A0 |
| TBD | Optional barcode D0 for edge-interrupt timing (`S3_BARCODE_USE_EDGES`) |

## Files

| File | Runs on | Purpose |
|---|---|---|
| `drv_ir.c/.h` | Pico | GPIO, ADC and microsecond timer access |
| `line_follow.c/.h` | Pico + host | Filter, steering, junction detection, probe, reacquire |
| `barcode_sensor.c/.h` | Pico + host | ADC to black/white: calibration, hysteresis, adaptation |
| `barcode_decoder.c/.h` | Pico + host | Full Code 39 (A-Z, 0-9), both directions, validated |
| `barcode_map.c/.h` | Pico + host | Letter-to-command map, set at run time with `MAP` over MQTT |
| `line_drive.c/.h` | Pico + host | BENCH ONLY: steering to wheel effort, junction stop / straight rule |
| `s3_bench.c/.h` | Pico | BENCH ONLY: `S3_BENCH=1` sensors, `S3_BENCH=2` line follow |
| `barcode_edges.c/.h` | Pico + host | Optional ISR-to-task edge buffer (lock-free) |
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

## Bench builds (physical testing)

Both use Buddy 2's bench platform: GP21 = STOP, GP20 = start gesture, motor
pins held low by his driver from boot. Same profile as `B2_BENCH`:

```
make -C build_make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none \
  MQTT=0 TEST=0 S3_BENCH=1 PICO_SDK_PATH=/path/to/pico-sdk
```

| Build | Image | What it does |
|---|---|---|
| `S3_BENCH=1` | `mtk3pico_smp1_usb_cdc_s3bench1.uf2` | Sensors only, motors never armed. Prints `[S3 RAW] L= R= steer= adc=` every 100 ms, plus decoded barcodes and junctions. |
| `S3_BENCH=2` | `mtk3pico_smp1_usb_cdc_s3bench2.uf2` | Line follow. Hold GP20 >= 1 s, release to start. At a junction: stop 1.5 s, then the barcode turn, or STRAIGHT if no barcode. |

Bench-only settings (not shared defaults): 1 ms kernel tick and GP26 analogue
mode. Normal builds are unchanged (10 ms tick, ADC pin off).

Line-follow safety: GP21 STOP interrupt stops the motors directly; TIMER
alarm0 is re-armed every 5 ms cycle as a 100 ms hardware watchdog; 20 s run
limit; effort 0..35% and never negative (no reversal); any driver error ends
the run. **First runs with the wheels raised.** U-turn barcodes end the run.

Tuning: edit the `#define` defaults at the top of `line_drive.c` and
`s3_bench.c` (the makefile has no hook for extra `-D` flags):
`S3_BENCH_BASE_EFFORT` (200), `S3_BENCH_GAIN` (2), `S3_BENCH_MAX_EFFORT` (350),
`S3_BENCH_JUNCTION_WAIT_MS` (1500), `S3_BENCH_CROSS_MS` (400),
`S3_BENCH_TURN_MS` (700), `S3_BENCH_RUN_MS` (20000). Sensor polarity:
`S3_LINE_BLACK_LEVEL` in `drv_ir.h`.

Note: the bench's no-barcode rule is "straight". The normal subsystem 3 rule
is still the probe ("+" straight, T right); pick one as a team.

`s3_bench.c` drives Buddy 2's motor driver through his bench seams, so it
needs Buddy 2's review before floor use.

## Demo day: setting the barcode letters over MQTT

The letters are announced on the day, so nothing is hardcoded. Send a `MAP`
message on the command topic before `START`:

```
MAP CLEAR K LEFT Q RIGHT Z STRAIGHT M UTURN
```

The reply is the whole map, e.g. `OK K=L M=U Q=R Z=S`. Also accepted:
`MAP K=L, Q=R` (short names), `MAP SHOW`, `MAP DEFAULT` (A=L B=R C=S D=U,
the power-on map), `MAP CLEAR`, `MAP K NONE`. Commands: LEFT/L, RIGHT/R,
STRAIGHT/S, UTURN/U, NONE/X. A bad message changes nothing and replies
`ERR <reason>: <word>`. A letter that is read but not mapped prints a warning
and is treated like no barcode (the junction probe runs).

The map is in RAM, so it resets to the defaults on power-up: send it again
after any reset.

Buddy 1 integration (in `b1_telemetry_task`, before `b1_parse`):

```c
if (barcode_map_is_command(m.data, m.length)) {
    char reply[BARCODE_MAP_REPLY_MAX];
    (void)barcode_map_command(m.data, m.length, reply, sizeof(reply));
    put(B1_RESULT, reply, now);   /* or wrap it in the JSON result */
    continue;
}
```

## Integration checklist (shared items, need team agreement)

- [ ] `config/config.h`: `CNF_TIMER_PERIOD` 10 -> 1 (or use `S3_BARCODE_USE_EDGES`)
- [ ] `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h`: `DEVCONF_ADC_PIN_INIT_0` TRUE
- [x] `pinmap.h` updated to the agreed map
- [ ] `app_main.c` calls `subsystem3_init()` / `subsystem3_start()`; priority agreed
- [ ] Controller: take route before junction each cycle; implement the probe
      handshake with Buddy 2's pivot; hold speed on `barcode_active`
- [ ] Buddy 5: call `subsystem3_reacquire()` at the end of a bypass
- [ ] Buddy 1: forward `MAP ...` messages to `barcode_map_command()` (snippet above)

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

Still to do for Week 13: RAM and message-size before/after measurements, and
packing `route_instruction_t` / `junction_observation_t` (shared contracts,
needs team agreement).
