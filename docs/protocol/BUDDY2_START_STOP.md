# Buddy 2 telemetry and START/STOP integration stage

28 October 2026. User-approved scope: add Buddy 2 from the supplied 27 October
handover, like Buddies 3–5; START and STOP only; no remote speed selection.
No Buddy 2 branch was merged or motor implementation copied.

## What the dashboard now shows

- Buddy 2 card: target, left/right wheel speeds (mm/s), mean wheel speed,
  signed travel distance, operating/movement state and detailed contract fields.
- Mean speed needs both wheel-speed validity flags. Missing/invalid data is
  unavailable, never a fabricated zero. Snapshot age >=1500 ms is stale; receiving
  fresh packets does not refresh an old motor sample or add it to the chart.
- The 60-second speed chart reports mm/s, including signed values. PWM percentage
  is not presented as physical speed. Legacy speed_percent stays only in the
  envelope/API for old producers, not as the new speed measurement.
- START/STOP controls replace the slider. Preview starts stopped and simulates
  driving at configured 30 mm/s when START is pressed. This is a demo setting
  from the handover, not a calibrated car speed. STOP returns simulated speed to
  zero. All preview/firmware-demo values are explicitly marked simulated.
- A default last_motion_result=0 is not an acknowledgement. The dashboard shows
  a result only when result_pending is set. No result producer exists yet.
- No wheel calibration result is invented from encoder activity or speed validity.

## Telemetry version 3

Buddy 2's handover describes a 38-byte compact frame, but its exact implementation
was not present in the published branch inspected. We use the handover's field
definitions to propose a Buddy 1 section; we do **not** claim byte compatibility
with the unseen compact_telemetry_encode codec. Agree a runtime adapter later.
Some fields are system/controller metadata, not measurements owned by Buddy 2.

The authoritative order, widths, units and ranges are in telemetry-fields.json,
section 2. Its 292 meaningful bits occupy 37 payload bytes; the optional section
adds its ID, payload length and four-byte snapshot timestamp (43 bytes total).
32-bit speed/distance ranges are representation ranges, not measured limits.
They remain full-width until physical ranges and precision can be justified.

The version 3 envelope is 19 bytes:

| Offset | Meaning |
|---|---|
| 0 | 0xB1 magic |
| 1 | Version 3 |
| 2 | Type 1, telemetry |
| 3 | bit0 demo, bit1 legacy-base-valid, bit2 S3, bit3 S4, bit4 S5, bit5 S2 present; bits6–7 zero |
| 4–7 | uint32 sequence |
| 8–11 | uint32 kernel uptime milliseconds |
| 12–14 | Shared 20-bit hardware fault mask, high four bits zero; positions still unassigned |
| 15 | Legacy percent field, not measured speed |
| 16–18 | Legacy distance cm field |
| 19 onward | Optional section records; each retains its independent snapshot timestamp |

All sections together: **252 bytes**, excluding MQTT overhead. S3/S4/S5 payload
layouts are unchanged. Web/Python readers retain v2 support; the C producer now
emits v3. Old dashboard versions will not decode v3; deploy the matching client.
The MQTT output ring is explicitly bounded at 1024 bytes, enough for the packet
plus topic and MQTT framing. The shared hardware mask and Buddy 2's copy must be
assembled consistently by the future controller/aggregator; no bits are assigned
or detected by this format change.

## START/STOP wire format

The /command topic carries exactly 8 bytes: `B1 02 03 opcode id0 id1 id2 id3`.
Opcode 1 = START, 2 = STOP. ID is nonzero uint32 little-endian. ASCII commands,
speed commands, unknown versions/opcodes, zero IDs and wrong lengths are rejected.
IDs correlate requests/results; they are not an authority token or replay-proof
security mechanism. Existing connection epochs and one-second command freshness
checks remain. JSON command results now include command_id. A `queued` result
means queued by Buddy 1, not approved/applied by the controller or motors.

The optional Python dashboard also sends these binary START/STOP messages and
has no speed selector. Heartbeats/results remain text in this stage.

## Runtime work still pending

The real path remains Buddy 1 → shared Vehicle Controller → Buddy 2. No diagnostic
motor functions are called from MQTT. The production Buddy 2 task, submission
queue, coherent live aggregator and command-result producer are still pending.
The future controller must cancel active/pending movement and latch remote STOP,
and START must require new authorisation without clearing other stop reasons.
This stage does not implement a physical emergency stop or guaranteed STOP latency.

Public adapter input is b1_binary_snapshot.s2, s2_at_ms and B1_SECTION_2, passed
inside a complete combined snapshot to b1_submit_binary_snapshot. The adapter
must preserve other sections, use the owning task's safe readings, maintain
validity/freshness and retain one-off results independently. No raw encoder counts
or PWM are added because those are diagnostic-only fields in the handover.

The current main pin map is unchanged. No firmware is flashed by this work.

## Checks

Portable C/JS/Python codec and command tests, independent v2/v3 golden packets,
invalid/truncated input cases, source freshness tests, isolated MQTT/WebSocket
roundtrip and desktop/mobile preview are used for verification. Hardware motor
execution remains untested. Rebuild RAM evidence after this firmware change;
old values must not be labelled as the current version until regenerated.
