# Buddy 4: minimum-bit data representation (Change Requirement 1)

This is the Buddy 4 (IMU, terrain and hump) evidence for the Week 13 change
requirement. It covers the four deliverables: the data representation table,
the RAM comparison, the message size comparison, and correctness and
performance.

The authoritative layout is in [s4_packed.h](s4_packed.h). The
`_Static_assert`s there fail the build if a field overlaps another or an enum
outgrows its field.

## How the data is represented

| Where the data lives | Technique | Why |
|---|---|---|
| Shared between tasks / ISRs, queue messages, MQTT | Explicit shifts and masks on `uint32_t` words (`s4_put` / `s4_get` / `s4_get_signed`) | Bit positions are fixed on every compiler, so the controller, Buddy 1 and the dashboard all decode the same layout |
| Task-private state and configuration | C bit-fields (`unsigned int x : n`) | Only one task touches them, so compiler-chosen bit order is harmless and the code stays readable |
| Faults | A 7-bit Buddy 4 detail mask, carried in the status word and in `FAULT` events. The shared hardware fault mask ([contracts/hardware_faults.h](../contracts/hardware_faults.h)) belongs to the Vehicle Controller | One bit per condition instead of separate integer variables, as the requirement's example asks |
| Validity | One DATA_VALID bit per message | Agreed at the Week 8 tribunal instead of reserving an "invalid" value in every field |
| Arithmetic working values | Natural width (`int32_t` accumulators, Q15 sine, deci-degrees) | The range genuinely needs it, for example height in µm over a 1.5 m hump |

## 1. Data representation table

### Word 0: status (`S4_W_STATUS`)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Terrain state | 0..6 (`terrain_state_t`: UNKNOWN, LEVEL, ASCENDING, CREST, DESCENDING, SETTLING, INVALID) | 3 | bits 2..0 |
| Speed cap | 0 = none, 10..1270 mm/s | 7 | bits 9..3, units of 10 mm/s, rounded down (stricter) |
| Motion class | STATIONARY, MOVING, ACCELERATING, BRAKING, TURN_LEFT, TURN_RIGHT, ROUGH, IMPACT | 3 | bits 12..10; the highest-priority condition wins (tribunal: 3-bit motion class) |
| Detail fault mask | I2C, MAG, STALE, UNCAL, SATURATED, TILT, COLLISION | 7 | bits 19..13, one bit each |
| Calibration status | NONE, RUNNING, LEVEL, LEVEL_MAG | 2 | bits 21..20 (tribunal: 2-bit calibration status) |
| Distance from odometry | 0 = fallback estimate, 1 = Buddy 2 odometry | 1 | bit 22 |
| Hump flag | 0/1: on a hump now (ASCENDING..SETTLING) | 1 | bit 23; Buddy 3 pauses its line-lost timeout and barcode decoding while it is set |
| Collision flag | 0/1: impact in progress | 1 | bit 24 |
| Collision counter | 0..7, wraps | 3 | bits 27..25; a change means a new hit |
| Last collision side | FRONT, REAR, LEFT, RIGHT | 2 | bits 29..28 |
| CLAMPED | 0/1: some field in this message was saturated to fit | 1 | bit 30 (explicit out-of-range status) |
| DATA_VALID | 0/1: level-calibrated and no bus/stale fault | 1 | bit 31; **the single validity bit for the whole message** |

Heading is meaningful when DATA_VALID is set and the MAG detail bit is clear.
No field uses a reserved "invalid" code.

### Word 1: attitude (`S4_W_ATTITUDE`)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Pitch | −90.0..+90.0°, nose up positive | 9 | signed, 0.5° units (−180..180), bits 8..0 |
| Roll | −90.0..+90.0°, right side down positive | 9 | signed, 0.5° units, bits 17..9 |
| Magnetic heading | 0..359° | 9 | bits 26..18 |
| Sample counter | 0..31, wraps | 5 | bits 31..27; unchanged means stale data |

### Word 2: motion (`S4_W_MOTION`)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Turn rate | −256..+254 °/s, clockwise positive | 8 | signed, **2 °/s units**, bits 7..0 (tribunal: ±250 °/s at 2 °/s = 8 bits); beyond that it saturates and sets CLAMPED |
| Live hump height | 0..255 mm | 8 | bits 15..8, current episode only |
| Last shock magnitude | 0..4080 mg | 8 | bits 23..16, units of 16 mg (±4 g full scale = 4000 mg) |
| Vibration level | 0..504 mg | 6 | bits 29..24, units of 8 mg |
| Reserved | 0 | 2 | bits 31..30 |

### Word 3: last completed hump (`S4_W_HUMP`)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Peak height | 0..255 mm | 8 | bits 7..0 |
| Uncertainty | 0..63 mm | 6 | bits 13..8, saturating |
| Peak \|pitch\| | 0..63° | 6 | bits 19..14 |
| Path length | 0..255 cm | 8 | bits 27..20 |
| Valid | 0/1 | 1 | bit 28 |
| Result counter | 0..7, wraps | 3 | bits 31..29; change means a new result |

### Event message (`s4_event_msg_t`, one `uint32_t`)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Type | TERRAIN, HUMP, COLLISION, TURN, FAULT, CAL | 3 | bits 2..0 |
| TERRAIN payload | state 0..6, cap 0..127 | 3 + 7 | bits 5..3, 12..6 |
| HUMP payload | word-3 bits 28..0 | 29 | bits 31..3 |
| COLLISION payload | side 0..3, magnitude 0..255 (×16 mg) | 2 + 8 | bits 4..3, 12..5 |
| TURN payload | right 0/1, angle 0..511° | 1 + 9 | bit 3, bits 12..4 |
| FAULT payload | fault mask | 7 | bits 9..3 |
| CAL payload | kind (0 level, 1 mag), ok | 1 + 1 | bits 3, 4 |

### Odometry input word (Buddy 2 → Buddy 4)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| Cumulative distance | mod 65536 mm | 16 | bits 15..0; deltas use modular subtraction, unambiguous below 32 m between reads |
| Post counter | 0..255, wraps | 8 | bits 23..16; unchanged for 10 reads (200 ms) means stale |
| Valid | 0/1 | 1 | bit 24 |

### MQTT payloads

| Payload | Content | Size |
|---|---|---|
| Telemetry frame | byte 0: version (3 bits) and frame type 4 (5 bits); bytes 1..16: words 0..3, little-endian | 17 B |
| Command byte | bits 2..0: NONE, RECALIBRATE, CLEAR_LATCHED, MAG_CAL_START, MAG_CAL_STOP, REPORT_RANGES; bits 7..3 must be 0 | 1 B |

### Shared hardware fault mask ([contracts/hardware_faults.h](../contracts/hardware_faults.h))

The team's shared mask is one 20-bit word owned by the Vehicle Controller.
Subsystems report fault evidence and never write the word, so there is only
one writer and no cross-task read-modify-write. Individual bit meanings are
not allocated yet.

Buddy 4 offers its evidence through the detail mask (status bits 19..13) and
`FAULT` events. Of the seven detail bits, only these describe a sensor
failure and are candidates for the shared mask:

| Detail bit | Condition |
|---|---|
| I2C | Accelerometer bus transaction failed |
| STALE | No fresh accelerometer sample for 200 ms |
| MAG | Magnetometer missing, failing or overflowing |

UNCAL, SATURATED, TILT and COLLISION are calibration, measurement or vehicle
status, not hardware failures. When I2C/STALE or MAG evidence lasts 1 s, the
IMU task latches it in a single-writer word read with `s4_sensor_failures()`.
In the whole-car build the controller polls that word every tick and latches
the **provisional** `HARDWARE_FAULT_IMU` / `HARDWARE_FAULT_MAG` bits (12 and 13).

### Task-private state ([terrain_estimator.h](terrain_estimator.h))

| Data | Valid range | Bits |
|---|---|---|
| Terrain state, pending state | 0..6 | 3 each |
| Confirm counter, turn-quiet counter | 0..15 | 4 each |
| Motion, longitudinal, turn state, collision side | 0..3 | 2 each |
| Fault mask | 7 flags | 7 |
| Calibration retries | 0..7 | 3 |
| Collision counter, hump counter | 0..7 | 3 each |
| Sample counter | 0..31 | 5 |
| Calibration sample count | 0..255 | 8 |
| 11 boolean flags (calibrated, cal active, mag cal active, ...) | 0/1 | 1 each |
| **Total** | | **64 bits = 8 bytes** |

| Configuration | Valid range | Bits |
|---|---|---|
| Axis mounting codes (×3) | sensor axis 0..2 plus a negate bit | 3 each |
| Accelerometer full scale | ±2/4/8/16 g | 2 |
| Filter shifts (×2) | 1..6 | 3 each |
| Confirm samples | 1..15 | 4 |
| Calibration retries | 1..7 | 3 |
| Calibration samples | 8..255 | 8 |
| Angles, mm and mg thresholds below 256 (×8) | 0..255 | 8 each |
| Times, speeds and larger thresholds (×16) | 0..65535 | 16 each |

### Choosing the widths from logged ranges

The tribunal asked for real turn-rate and hump ranges to be logged before
the widths are fixed. The estimator keeps a range log: the maximum
|turn rate|, |pitch|, |roll|, hump height and length, shock and vibration,
measured **before** packing, plus a count of clamped messages. Send command 5
(`S4_CMD_REPORT_RANGES`) after a test run and the IMU task prints the log,
then resets it:

```text
S4 ranges: turn <max> dps (field +/-254), pitch <max> deg, roll <max> deg
S4 ranges: hump <max> mm / <max> mm long, shock <max> mg, vibration <max> mg, clamped <n>
S4 sample time: last <us> us, worst <us> us (period 20 ms)
```

Record the real values in `testing/subsystem4/TEST_LOG.md` (test S4-H06). If
a logged maximum gets close to its field limit, or `clamped` is non-zero in
normal driving, widen that field in `s4_packed.h`; the static asserts
re-check the layout. In the host simulation, a 383 °/s spin saturated the
turn-rate field, set CLAMPED, and the log still recorded 383.

## 2. RAM usage comparison

Sizes are for the real target ABI (arm-none-eabi-gcc 10.3.1, Cortex-M0+).
They come from `arm-none-eabi-size` and `arm-none-eabi-nm -S` on the compiled
objects, and are reproducible with
[testing/subsystem4/s4_size_report.c](../../testing/subsystem4/s4_size_report.c).

Buddy 4 had no previous integrated implementation. "Before" therefore means
the same data held in a conventional representation: one natural C type per
field, `float` angles, one `bool` or enum per flag, and 32-bit configuration
words.

| Item | Before (B) | After (B) | Saving |
|---|---:|---:|---:|
| Shared snapshot | 96 | 16 | 83% |
| Estimator state flags | 60 | 8 | 87% |
| Estimator configuration | 132 | 44 | 67% |
| Whole estimator object (`s4_est_t`, incl. 16 B range log) | 352 | 212 | 40% |
| Odometry input | 12 | 4 | 67% |
| Last hump result (vs `terrain_report_t`) | 24 | 4 | 83% |
| **Buddy 4 static RAM (.bss)** | **≈ 540** | **312 (measured)** | **42%** |
| Event queue in the kernel pool (8 slots + 4 B header each) | 416 | 64 | 85% |
| **Total Buddy 4 RAM** | **≈ 956** | **376** | **61%** |

The measured "after" static RAM (2026-10-28, `CAR_RUN=1` flags, after the
whole-car integration) is `task_imu.o` 284 B plus `s4_shared.o` 28 B. The
"before" figure applies the conventional layouts above to the same items and
keeps the other 72 B of task state (driver handle, odometry reader, bindings,
failure word and timers) unchanged. The task stack (2048 B) and the kernel control blocks are the
same in both versions and are not counted.

`s4_size_report.c` is on the `buddy4` and `4` branches but not yet on this
one, so the link above only works once it is brought over.

For the whole-team figure, run `arm-none-eabi-size` on the linked `.elf`
before and after integration and record both numbers here. That needs the
Make/WSL target build, which this change did not run.

## 3. Message size comparison

| Message | Path | Before | After | Saving |
|---|---|---:|---:|---:|
| Event (hump, collision, turn, ...) | IMU task → controller queue | 48 B struct | 4 B | 92% |
| Snapshot copy | IMU task → any reader | 96 B | 16 B | 83% |
| Odometry | Buddy 2 → Buddy 4 | 12 B | 4 B | 67% |
| Terrain report (IF-03) | Buddy 4 → controller | 24 B (`terrain_report_t`) | 4 B status word + 4 B hump word | 67% |
| MQTT telemetry | Buddy 1 → broker | 347 B JSON text | 17 B binary | 95% |
| MQTT command | broker → Buddy 1 → Buddy 4 | 21 B JSON text | 1 B | 95% |

The JSON baseline carries exactly the same 26 fields as the binary frame; the
text is in `s4_size_report.c`. The existing `terrain_report_t` contract is
still supported through `s4_terrain_report_from_words()`, so the controller
can adopt the packed words without a flag day.

## 4. Correctness and performance

### Correctness: host tests

[testing/host/test_terrain_estimator.c](../../testing/host/test_terrain_estimator.c)
runs the packed code against a simulated car: a 2° mounting tilt, ±6 mg
sensor noise, and real odometry. Every number below comes out of the packed
fields, so the reduced widths are exercised end to end.

| Check | Result (re-run 2026-10-28, host gcc 14.2, uncommitted working tree on `testing`) |
|---|---|
| atan2 vs libm, full circle | worst error 0.104° |
| Tilt-compensated heading, pitch −15..+15°, 36 headings | worst error 0.20° |
| 20 mm hump, 300 mm long | measured 20 mm, ±6, valid |
| 40 mm hump, 400 mm long | measured 39 mm, ±7, valid; the configured hump cap (100 mm/s) applies while on it |
| 70 mm hump, 500 mm long | measured 69 mm, ±8, valid |
| 40 mm hump without odometry | 39 mm, uncertainty widened to ±26, valid |
| 90° right pivot at 90 °/s | rate 88 °/s, event "right 89°" |
| Head-on and side collisions | correct side, latched, no false hump afterwards |
| Bus fault, stale data, dropout mid-hump | INVALID state, degraded cap, aborted hump marked invalid, recovery |
| Pack/unpack boundaries | −180 and +179 in 9-bit signed fields; no spill into neighbouring fields |
| Tribunal fields | Calibration status RUNNING → LEVEL → LEVEL_MAG, and NONE after a failure; DATA_VALID drops on one bus error; hump flag set only on the hump; collision flag only during the impact; a 383 °/s spin saturates the 8-bit turn rate at +127 (254 °/s), sets CLAMPED, and the range log keeps 383 |

The run used the team host build (`app_program/CMakeLists.txt`) and `ctest`.
All 4 suites passed together: subsystems 3, 4 and 5, plus Buddy 5's bridge
test. Build warnings: none under `-Wall -Wextra -Werror -Wpedantic -std=c11`.

### Concurrency safety

| Shared item | Writer | Readers | Mechanism |
|---|---|---|---|
| Snapshot words 0..3 | IMU task only | any task or handler, either core | Sequence lock: odd/even counter plus DMB fences. No reader ever writes, so there is no read-modify-write. Readers retry a bounded number of times and get a coherent 4-word copy or `false`. |
| Odometry word | Buddy 2's one task only | IMU task | A single aligned 32-bit store or load, atomic on RP2040 across both cores |
| Request bits | any task or handler | IMU task | Kernel event flag (`tk_set_flg` is atomic and ISR-safe) |
| Tick | cyclic handler | IMU task | Same event flag, private bit 15 |
| Events | IMU task | controller (single reader) | Kernel message buffer, copied by value; when full, the oldest event is dropped and counted |
| Estimator bit-fields | IMU task only | none | Task-private, never shared, so read-modify-write is safe |

ARMv6-M has no exclusive load/store instructions. That is why the design
avoids shared read-modify-write entirely instead of relying on atomic
fetch-or. Compiling for Cortex-M0+ confirmed that the seqlock emits `dmb` and
pulls in no libatomic calls.

### Performance

| Measure | Value | Status |
|---|---|---|
| Buddy 4 flash (text) | 12,828 B: estimator 7,020, task 2,693, math 2,010, driver 629, shared 476 | measured, ARM objects (2026-10-28) |
| I2C traffic per 20 ms period | 7-byte accel burst + 6-byte mag burst, about 0.45 ms at 400 kHz (≈2% bus duty) | calculated |
| Estimator step time | worst case must stay under 1 ms (5% of the period) | measured on the car by self-test **S4-T09**; not yet run |
| Sampling time (I2C + estimate + publish) | `s4_sample_time_us` / `_worst_us`, 1 MHz hardware timer, every period | measured continuously on the car; printed by command 5 and the self-test; not yet run |
| Pack/unpack overhead | a few shifts and masks per field, no division | by inspection |

Remaining work: run S4-T09 on hardware and record the mean and worst time,
and record whole-ELF before/after RAM once the target build is qualified.
