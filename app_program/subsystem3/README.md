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
| `barcode_decoder.c/.h` | Pico + host | Code 39 `*A*`-`*D*`, both directions, validated |
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

## Integration checklist (shared items, need team agreement)

- [ ] `config/config.h`: `CNF_TIMER_PERIOD` 10 -> 1 (or use `S3_BARCODE_USE_EDGES`)
- [ ] `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h`: `DEVCONF_ADC_PIN_INIT_0` TRUE
- [x] `pinmap.h` updated to the agreed map
- [ ] `app_main.c` calls `subsystem3_init()` / `subsystem3_start()`; priority agreed
- [ ] Controller: take route before junction each cycle; implement the probe
      handshake with Buddy 2's pivot; hold speed on `barcode_active`
- [ ] Buddy 5: call `subsystem3_reacquire()` at the end of a bypass

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
