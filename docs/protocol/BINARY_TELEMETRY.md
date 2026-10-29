# Buddy 1 binary telemetry v2 — first integration stage

**Historical layout:** the producer now emits version 3 and commands are binary
START/STOP only. See `BUDDY2_START_STOP.md` for the current interface. Version 2
remains supported by dashboard decoders for compatibility.

The later diagnostics topic, rejection reporting and measured static RAM evidence
are documented in `DIAGNOSTICS.md` and `STATIC_RAM_COMPARISON.md`. The telemetry
layout described below remains unchanged.

Implemented 22 October 2026. This is a proposed, versioned application format,
not the team's final measured representation. No hardware was flashed by this change.

## Scope

`/telemetry` now carries binary bytes, nominally every 200 ms. `/heartbeat`
(5 seconds), `/command` and `/command_result` retain the existing text protocol.
Both dashboards accept old JSON telemetry as well as binary v2. Commands still
request simulated actions in the explicit Buddy 1 demo; they do not execute motors.
The dashboard design, WiFi owner, command queues and heartbeat retry policy remain.

No Buddy 2 representation is invented. No other buddy's source is modified.
The updated `main` pin map is authoritative and is unchanged by this feature.

## Wire envelope

All multibyte integers are little-endian. Never transmit a C struct with memcpy.

| Byte offset | Width | Meaning |
|---|---|---|
| 0 | 1 | Magic `0xB1` |
| 1 | 1 | Format version `2` |
| 2 | 1 | Message type `1` (telemetry) |
| 3 | 1 | bit 0 simulated; bit 1 legacy base readings valid; bits 2/3/4 present S3/S4/S5; remaining bits zero |
| 4 | 4 | Packet sequence, wraps uint32; shared producer sequence also advances for heartbeat |
| 8 | 4 | Producer kernel uptime milliseconds, wraps uint32 |
| 12 | 2 | Shared hardware/sensor fault mask, **positions not assigned yet**; use zero until team agreement |
| 14 | 1 | Existing Buddy 1 speed percent, 0..100 |
| 15 | 3 | Existing Buddy 1 distance cm, 0..100000; retained for baseline compatibility, not a Buddy 2 definition |
| 18 onward | variable | Sections, only when their presence bit is set |

Base readings must not be displayed when bit 1 is zero. Their unused bytes are
still range-validated. A telemetry frame with no sections is valid: absence means
unavailable, not zero. Unknown versions, unknown/duplicate sections, missing
sections, invalid lengths, out-of-range values and nonzero bit padding are rejected.

Each section: ID (1 byte: 3, 4 or 5), payload length (1 byte), snapshot time
(4 bytes, same uptime clock), then payload. S3 and S5 fields are packed LSB-first;
signed fields use two's complement. The authoritative field order, widths,
accepted ranges and units are in `telemetry-fields.json`.

| Section | Payload | With section header |
|---|---|---|
| Buddy 3 | 17 bytes | 23 bytes |
| Buddy 4 | 17 bytes | 23 bytes |
| Buddy 5 | 138 bytes | 144 bytes |
| Combined including envelope | | **208 bytes** |

At 5 Hz the full draft is 1040 payload bytes/second, excluding MQTT/TCP overhead.
Most Buddy 5 space is running 32-bit counters. Physical ranges and counter widths
can be tightened after team review and measurements; changing this layout needs
a new version. This does not prove minimum-bit storage throughout the firmware.

## Buddy 3

Uses continuous signed steering -100..100, not three discrete levels. Two ADC
thresholds are retained. `adc_valid`, `scan_valid` and `pattern_valid` gate their
respective readings. Bar duration is an integer millisecond proposal, 0..6000,
because the old 15-bit microsecond proposal was insufficient. Pattern is the
retained nine-element complete pattern; the first element is bit 8. Route is
0 none, 1 left, 2 right, 3 straight, 4 U-turn. Exact firmware-provided state values
are retained. Counters are proposed uint16 modulo counters; these are wider than
the current proposed four-bit route counter and require adapter agreement.

The adapter must retain a per-scan summary. It must not take the controller's
single-reader route/junction events or read private fields from another task.
No hardware fault detector is fabricated from constant black/white readings.

## Buddy 4

The original 17-byte `0x21` frame is carried intact, including status/detail flags.
Those detail flags are **not** copied into the shared hardware-only fault mask.
The web decoder gates attitude on DATA_VALID; heading and derived turn rate also
require calibration LEVEL+MAG and no MAG detail fault. CLAMPED is displayed.

The section time is when an adapter obtains the snapshot, not a measured sensor
timestamp. Reception time is not proof of fresh measurements. Raw data has no
timestamp, and a modulo-32 sample counter cannot prove freshness across arbitrary
packet loss. Hump/collision counters wrap modulo 8; differences are only
unambiguous when fewer than eight events occurred between received frames.
Only the latest hump detail persists. Never consume the controller's event queue.

## Buddy 5

Distance proposal: integer mm, 0..4000 (0.1 cm granularity, not measured accuracy).
Status: 0 OK, 1 no echo, 2 too close, 3 sensor fault, 4 no reading yet. Distance
is meaningful only for OK. No echo is not presented as a valid maximum distance.
`front_at_ms` and `last_at_ms` refer to measurements; the section timestamp
refers to the snapshot. All must use the same kernel uptime epoch.

Front means at servo centre **and the scan-start car heading**, not necessarily
the car's present forward direction. Direction is in half-degrees, positive left;
heading is commanded integer degrees, positive left. Servo angle is commanded,
not feedback. Profile coordinates remain positive x right/y forward, per source.

Geometry widths are conservative **proposals**, not measured limits. Producers
must quantize deliberately and reject/mark invalid any value outside the table;
the encoder never silently clamps. Profile/decision/plan valid flags control
whether their data may be used. Old profiles must be cleared/marked invalid by
the producer when their encounter is no longer applicable. Failure codes 1..6
are reserved pending an agreed mapping of the source strings; use 0 for now.

Nineteen uint32 metrics are carried. The source report's uint64 total time and
the full variable-length scan are not included in this snapshot. Detections,
cleared etc. are cumulative modulo counters; dropped-event count does not
reconstruct individual event details. The event ring/API and separate event
packets remain a later integration stage; this change does not silently consume
or claim to preserve every event. The 200 ms producer may transmit an older
Buddy 5 snapshot; measurement age must remain visible. Its estimated 240 ms
blocking measurement time is not a measured guarantee.

## Producer interface and lifecycle

`b1_submit_binary_snapshot(const b1_binary_snapshot *)` validates and copies a
complete combined snapshot under Buddy 1's nonblocking mutex. A return of 0 means
invalid input or busy: the adapter must retry later. The old
`b1_submit_snapshot` continues to update the baseline speed/distance fields.

The combined adapter must collect each module using that module's agreed safe
interface, retain independent per-module timestamps, and submit a complete copy.
Repeated submissions must not restamp unchanged sensor measurements. It must not
submit a partial struct that clears another producer's fields inadvertently.
The actual controller/owner-task adapters are not implemented here. A latest
combined snapshot is published only for 1500 ms after its last submission.

Default/demo startup remains unchanged. Firmware demo telemetry has only baseline
sample fields; it does not pretend to read real S3/S4/S5 modules. Browser preview
has generated S3/S4/S5 sections, explicitly labelled simulated, and uses the same
binary decoder as real traffic. Buddy 2 remains unavailable.

Outgoing packets carry explicit lengths; the MQTT service publishes that length,
never strlen. The 512-byte buffer is capacity; the current maximum packet is 208
bytes. No credentials are changed or committed. No physical START/STOP guarantee
is introduced by a telemetry format change.

## Generate and verify

`python tools/generate_buddy1_binary.py` regenerates C/JS field codecs from the
JSON field table. `python tools/verify_buddy1_binary.py` runs the portable C suites,
mocked bridge timing tests, web tests and Python tests, and compares independently
populated C bytes to the checked-in golden vector. GCC, Node and Python required.
CTest also includes the binary codec test. Browser and hardware tests are separate.

The stage remains incomplete for hardware integration, event transport, final
fault positions, measured ranges, Buddy 2 fields, and binary commands/heartbeat.

### Verification performed on 22 October 2026

- 12 portable C test executions passed in Ubuntu: binary boundaries/golden vector,
  existing protocol/retry/controller/motion/perception suites, inactive and demo
  lifecycle, three heartbeat-contention cases, and real-snapshot expiry. Windows
  application control blocked newly built executables, so C execution used WSL.
- 16 Node tests and 7 Python tests passed. C, JavaScript and Python use the same
  independently populated golden packet; signed and zero-byte fields survive.
- Headless Edge preview passed at desktop and 390px mobile widths, including
  simulated STOP, unavailable reset, binary subsystem cards and no page errors.
- Isolated Mosquitto port 19001 passed CONNECT/SUBACK and the full binary
  telemetry WebSocket roundtrip. It did not contact the Pico.
- Pico SDK 2.3.0 demo profile compiled and linked with placeholder credentials:
  `NET=lwip MQTT=1 TEST=1 B1_DEMO_RUN=1 CONSOLE=usb_cdc CRASH_REPORT=0`.
  No firmware was flashed and no physical integration was tested.
