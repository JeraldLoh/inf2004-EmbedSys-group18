# Signal diagnostics

The browser panel shows the last **accepted telemetry payload's byte length**,
excluding MQTT/TCP/WebSocket overhead. For binary telemetry it also computes a
JSON encoding of the same represented values. Buddy 4 is represented by its
original raw frame, without adding duplicate derived display fields. This is a
computed encoding comparison, not a historical captured message.

Device diagnostics are a separate non-retained QoS 0 `/diagnostics` topic,
nominally once per second. Old firmware without this topic leaves device counts
unavailable; it does not show a fabricated zero. Browser preview sends explicitly
simulated diagnostic samples. No sensor faults are added to the hardware mask.

The 52-byte payload uses little-endian integers:

| Offset | Bytes | Meaning |
|---|---|---|
| 0 | 4 | `B1 02 02`, followed by demo flag 0/1 |
| 4 | 4 | Current kernel uptime ms |
| 8 | 4 | Snapshots rejected by validation |
| 12 | 4 | Subset rejected for an out-of-range field |
| 16 | 4 | Submission attempts that could not acquire the mutex |
| 20 | 4 | Last validation rejection time, kernel ms |
| 24 | 1 | Section: 1 base, 3 Buddy 3, 4 Buddy 4, 5 Buddy 5 |
| 25 | 1 | Field ID, one-based in `telemetry-fields.json`; base fields are base_valid, speed_percent, distance_cm, fault_mask, present |
| 26 | 1 | Reason: 0 none, 1 missing snapshot, 2 range, 3 format |
| 27 | 1 | Reserved zero |
| 28 | 8 | Offending value, signed int64 |
| 36 | 8 | Allowed minimum, signed int64 |
| 44 | 8 | Allowed maximum, signed int64 |

Reason 0 has zero metadata. Missing input is section 1/field 0 with zero values.
Buddy 4 format failure is section 4/field 1, required header 33 (`0x21`). Range
errors identify the first invalid field; they do not enumerate every invalid
field in the same snapshot. Inconsistent metadata is rejected by the dashboard.

Counters are uint32 modulo counters of **submission attempts**, not distinct
physical incidents. A caller retry can increase them again. They reset when the
device restarts. Busy submissions are counted separately; a busy attempt is not
classified as out of range until an attempt acquires the guard and validates.
The previous valid snapshot stays intact on failure and still expires after its
normal 1500 ms producer window. No raw out-of-range value is inserted into a
valid sensor field. Diagnostics are retained only as the latest report, so they
do not promise an event log of every rejected sample.

The browser counts invalid inbound messages separately for its own session.
Retained packets are ignored without changing counts. Diagnostic packets cannot
refresh the heartbeat or replace sensor telemetry. Last report age is displayed;
disconnection or age >=3 seconds marks it stale. Old values remain historical.

The RAM panel uses `build-memory.json` and the actual linked ELF evidence in
`STATIC_RAM_COMPARISON.md`. It is historical `.data + .bss`, not live RAM or a
stack/heap high-water measurement. The measured increase includes new features
and buffers, so it is not a claim of binary-only savings. Regenerate evidence
after firmware changes before using it as a current comparison.

Verified with portable C rejection/bridge tests, browser unit tests, a real
isolated local MQTT/WebSocket diagnostics roundtrip, headless desktop/mobile
preview, and a Pico SDK demo build. No firmware was flashed.
