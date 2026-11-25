# Combined rover data representation

Audit of the next-junction Wi-Fi integration working tree based on `da4e9d9`, measured 25 November 2026. The storage patch changes three places: Buddy 1's MQTT transmit staging, Buddy 3's map and the controller mailbox. The typed control APIs, wire schemas and internal units remain the reference for field ranges. Reserved capacity is retained where it is needed to move a record atomically, preserve rollover, store an unvalidated value or keep an existing wire format compatible.

## Fields changed in the first storage checkpoint

| Owner and field | Valid range or sentinel | Bits and storage | Access and validation |
|---|---|---|---|
| Buddy 1 outbox payload | Nonempty bytes per topic, up to 65 heartbeat, 252 telemetry, 131 result, 52 diagnostics, 100 live status | 8 per payload byte, one exact-capacity array per topic | Actual length checked against topic capacity before copying. Largest v3 payload holds all four sensor sections. |
| Buddy 1 outbox length | 0 empty, 1–252 occupied, subject to the topic limit | 8, replaces a full length and a separate occupancy flag per slot | The mutex still covers overwrite, take and restore. Restore keeps a newer copy already in the slot. |
| Buddy 1 outbox creation time | 0–4,294,967,295 kernel ms, unsigned rollover | 32 per slot | Original age limits remain computed with unsigned subtraction. |
| Buddy 1 outbox topic | 0 heartbeat, 1 telemetry, 2 command result, 3 diagnostics, 4 live status | 3 logical bits, implicit in array index | Public topic argument checked before indexing. The radio-facing struct retains its typed kind. |
| Buddy 3 barcode assignment, four direction slots | 0 unassigned, 1–26 A–Z, 27–36 0–9 | 6 per slot, 24 in one 32-bit atomic word | The complete map is validated before publication. Duplicate directions and out-of-range commands are rejected. Lowercase aliases retain the original meaning. |
| Buddy 3 map revision | Even 2 through 4,294,967,294, wraps to 2. Odd publishes in progress and reads back as 0 | 32, independent atomic word | Single writer preserves in-progress and completion stores. Readers compare revisions before and after loading the map. |
| Controller mailbox kind | 0–5, event, motion, result, safety, wake, remote start | 3 at header bit 0 | Producer selects tag, kernel copies fixed-size records. |
| Controller event type, mode, mission, rejoined | Type 0–6, mode 0–6, mission 0–10, Boolean 0–1 | 3 at bit 3, 3 at bit 6, 4 at bit 9, 1 at bit 13 | Validate full input enums before shifts. Shared public structs retain their semantic types. |
| Mailbox motion kind and source | Kind 0–3, source 0–5. Continuous motion stays in its synchronized overwrite slot. | 2 at bit 3, 3 at bit 5 | Invalid enum values fail before enqueue. |
| Mailbox motion goal type or profile | Goal 0–4, profile 0–2. Meaning selected by motion kind. | 3 at bit 8 | Validated before shifting. |
| Mailbox motion scalar fields | Session, command ID, authority generation, creation and expiry ms, speed cap and timeout use the full uint32 transport domain. Goal distance mm and turn mdeg use int32. | 32 each | IDs and timestamps retain wrap behavior. Signed values survive transport to the existing controller and actuator checks. |
| Mailbox goal result code and odometry validity | Code 0–7, valid 0–1 | 3 at bit 3, 1 at bit 6 | Reject a code above the defined range before packing. |
| Mailbox result, restart and safety metadata | Goal ID, generation, restart ticket, evidence ID are uint32. Travel mm and turned mdeg are int32. Restart ticket `UINT32_MAX` remains a sentinel. | 32 per value | Restart checks, stop mask and urgent-stop ordering retain the original synchronization. |
| Stop reason mask | Bits 0–8 assigned by the controller. Full uint32 storage at STOP and evidence ingress. | 32 | Preserve safety evidence and keep the urgent stop lane independent of the ordinary FIFO. |

The compact mailbox stays word-aligned. The encoded header uses only bits needed by its selected record kind. Its 40-byte fixed stride holds the largest supported message and preserves the 16-message FIFO limit. The RTOS adds its own 4-byte length per slot. Neither C bitfield layout nor a global enum-size compiler flag sets this format.

## Audit of retained fields

| Area and fields | Range and representation | Reason for retaining |
|---|---|---|
| Buddy 1 packet header and diagnostics | Presence 4 bits, base-valid and demo 1 each, speed 0–100 in 7 meaningful bits of one byte, distance 0–100,000 cm in 17 meaningful bits of three bytes, fault mask 20 bits, sequence and uptime uint32. Diagnostic offending value and limits signed int64. | Keep the v3 frame offsets and preserve the original invalid values for diagnostics. |
| Buddy 1 inbound MQTT and vehicle queue | Valid command sizes 8 and 22 B. Raw buffer accepts up to 128 B for parsing and invalid-frame handling. Connection epoch and receipt timestamp use uint32. Validated command carries full uint32 ID and typed opcode 1–2. | Keep invalid-input handling, reconnect isolation and one-second freshness. Queue header overhead and word alignment accompany the typed records. |
| Buddy 2 wheel measurements and motion | Edge counts and acquired time are uint32 rolling. Accumulated wheel travel is uint64 in micrometres. Retained retrace samples are already uint16 per wheel, 0–65,535 micrometres per stored entry. | Keep encoder precision, modular edge/tick arithmetic, PID and safe actuator handoff. Existing motor/encoder synchronization and ISR owners remain in charge. |
| Buddy 2 wire motion | Speed and distance retain int32 resolution and separate 1-bit validity fields. Sequence is 8-bit modulo, uptime/command ID/generation are 32-bit. Mode 0–6 uses 3 bits, mission 0–10 uses 4, terrain 0–6 uses 3, route 0–4 uses 3, lifecycle 0–5 uses 3, motion source 0–5 uses 3, result 0–7 uses 3, connection state 0–3 uses 2, faults 20 and stops 9. | Existing exact field encoding and reserved values remain compatible. |
| Buddy 3 line and barcode | Line snapshot already uses one byte for two black bits, 3-bit line state 0–4, 2-bit last side 0–2, and a barcode-active bit. Wire steering −100…100 uses signed 8 bits, ADC 0–4,095 uses 12, bar duration 0–6,000 ms uses 13, 9-element pattern uses 9, accepted event counters wrap at 16 bits. Native edge timing and diagnostic traces retain uint32 microseconds. | Preserve the full timer subtraction and recorded diagnostic interval range. Sensor validity remains separate from measured zero or white. |
| Buddy 3 route and junction messages | Route, lifetime validity and junction evidence retain typed flags and full uint32 clock/ID fields. Route enum 0–4 needs 3 bits logically. Timed probe and next-junction lifetime are separate values. | Keep the owner mutex, map revision check and bounded scope of a pending turn. |
| Buddy 4 packed status and attitude | Four existing uint32 words, with a 17 B MQTT subframe including header. Terrain 3 bits, cap 7 in 10 mm/s units, motion 3, faults 7, calibration 2, single-bit flags, end cause 2 and counter 3. Pitch and roll signed 9 each in half-degrees, heading 9 in degrees, sample sequence 5. | Existing validity flags, unit resolution and modulo counters retained. Snapshot publication uses the existing stable-read protocol. |
| Buddy 4 packed motion and hump | Signed rate 8 in 2°/s units, live hump and lift 8 each in mm and cm respectively, vibration 6 in 8 mg units. Peak height 8, uncertainty 6, peak pitch 6, path/width 8, validity 1, sequence 3. | Preserve saturation reporting, lift and rollover behavior. |
| Buddy 4 event and odometry links | Event tag 0–5 in 3 bits with up to 29 payload bits. Separate odometry word has 16-bit distance mm modulo 65,536, 8-bit sequence and 1-bit validity. | Already compact and published with the existing single-word or snapshot synchronization. |
| Buddy 5 sonar and geometry | Telemetry range status 0–4 in 3 bits, 0–4,000 mm in 12 bits, signed direction 11 half-degree bits, heading 10 signed degrees, servo 0–180 in 8 bits, side 0–2 in 2. Profile widths and distance use 16-bit mm. Coordinates use signed 17-bit mm. Movement targets and filtering remain native. | No echo, too close and unavailable remain distinct from measured zero. Keep sonar estimation and bypass math independent of telemetry. |
| Buddy 5 state, search and metrics | State 0–11 uses 4 bits. Flags 1 each, phase/attempt bins use 2–7 bits as specified by the codec. Encounter IDs, times and cumulative metrics use uint32 on the wire. Local motion geometry uses native floating point and time is uint32 ms. | Keep overlap with the existing binary decoder and all timing, fallback and motion authority behavior. |
| Shared controller state, fault and authority | Native enums, typed shared snapshots and counters remain in their existing mutex and urgent-stop lanes. Underlying clocks and generation/command/goal IDs use uint32. Wire fault mask uses low 20 bits and stop mask low 9 bits. | Keep atomic ownership and entire rejected or safety input visible until an owning task validates it. |

Every configurable Buddy 2, 3 and 5 telemetry field has an individual row in the [codec field/range/bit-width schema](telemetry-fields.json). Buddy 4's authoritative offsets and field widths are in [s4_packed.h](../../app_program/subsystem4/s4_packed.h), with preserved units and ranges in [Buddy 4 representation](../../app_program/subsystem4/DATA_REPRESENTATION.md). See [matched measurements and test results](STATIC_RAM_COMPARISON.md) for record sizes and evidence. The results establish particular representation savings within this combined build. A global minimum for all firmware state remains unproven.

## Files changed for the first storage checkpoint

| Files | Change |
|---|---|
| `app_program/subsystem1/buddy1_binary.h`, `tools/generate_buddy1_binary.py`, `app_program/subsystem1/buddy1_bridge.c` | Actual telemetry maximum and bounded per-topic outbox. |
| `app_program/subsystem3/barcode_map.c` | Atomic 24-bit direction map and preserved version handshake. |
| `app_program/vehicle_controller/controller_task.c` | Packed mailbox records and validation before packing. |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | Firmware ID for run-log provenance. |
| `dashboard/web/app.mjs`, `dashboard/web/index.html`, `dashboard/web/diagnostics.mjs`, `dashboard/web/build-memory.json`, `dashboard/web/diagnostics.test.mjs` | Matched ELF data, labels, cache revision and display test. |
| `testing/host/test_buddy1_map.c`, `testing/host/bench_barcode_map.c`, `testing/subsystem2/b2_01/test_controller_transport.c`, `testing/subsystem2/b2_01/CMakeLists.txt` | Boundaries, packed transport regression and paired map timing. |
| `docs/protocol/DATA_REPRESENTATION.md`, `docs/protocol/STATIC_RAM_COMPARISON.md`, `docs/protocol/DIAGNOSTICS.md`, `Project/ROBOT_DEMO_HANDOFF_2026-11-25.md` | Audit, measured evidence, documentation and handoff. |
| `output/firmware/demo-combined-minbits-2026-11-25/group18_demo_combined_80_mqtt_minbits_2026-11-25.uf2`, `manifest.json`, `measurements.json`, `PICO_TEST.md` in the same directory | Named combined UF2, manifest, measurement JSON and Pico test instructions. |

Motor control settings and all hardware assignments are inherited from the same next-junction profile. Early rejection of unrepresentable mailbox inputs, validation of outbox length, and the firmware label are the externally visible changes beyond storage. Existing valid commands and telemetry keep their interpretation.

## Compact-data follow-up

The storage below is in the combined compact-data build. The linked comparison uses the same profile as the original next-junction build and the first storage checkpoint.

| Producer → consumer, field | Domain, units and rollover | Logical bits and physical representation | Validation and ownership |
|---|---|---|---|
| Buddy 3 task → Buddy 1, line paused | 0 or 1 | 1 at bit 0 of the live flags word | Canonicalize Boolean before packing. |
| Buddy 3 task → Buddy 1, calibration state and reason | State 0–4, reason 0–1 | 3 at bit 1, 1 at bit 4 | Invalid values retain the previous wire fallback of 0. |
| Buddy 3 task → Buddy 1, barcode state | 0–4 | 3 at bit 5 | Invalid state retains the previous fallback of 3. |
| Buddy 3 task → Buddy 1, barcode character and route | 0–127 ASCII, 0–4 route | 7 at bit 8, 3 at bit 15 | Invalid values retain the previous fallback of 0. |
| Buddy 3 task → Buddy 1, digital mode and publication marker | 0 or 1 each | 1 each at bits 18 and 19 | Digital mode is canonicalized. Publication marker is set by the single writer. |
| Buddy 3 task → Buddy 1, reading timestamp and barcode-event timestamp | Unsigned milliseconds modulo 2³² | 32 each, aligned native word | Full source precision. The reader rejects mixed generations. |
| Buddy 3 task → Buddy 1, scan sequence, failure count, map revision | uint32 rollover for counters and full map revision | 32 each, aligned native word | No narrowing before freshness and map checks. |
| Buddy 3 task → Buddy 1, snapshot generation | Odd while publishing, even when stable, wraps modulo 2³² | 32, aligned native word | Atomic reader retries at most four times. The publication marker distinguishes the reset state from sequence wrap. |
| Buddy 2 status producer → Buddy 1, target-valid, qualified, searching | 0 or 1 each | 3 meaningful bits in one aligned 32-bit word | Validate full input before packing under the Buddy 1 mutex. |
| Buddy 2 status producer → Buddy 1, signed wheel targets | int32 | 32 each, aligned | Full targets and validity survive independently. |
| Buddy 2 status producer → Buddy 1, timestamp, searches, failures, accepted/rejected counts, driver errors, both encoder counts, wheel diameter | uint32, native ms, counts or mm | 32 each, aligned | Preserve the original wire domains and modulo arithmetic. |
| Buddy 5 task → Buddy 1, fast, active, backing off, current/total segment, side | Booleans 0–1, segments 0–7 with current ≤ total, side 0–2 | 11 bits in one aligned word | Validate before the same mutex-owned update. |
| Buddy 5 task → Buddy 1, timestamp, backoff count, elapsed time | uint32 ms or count | 32 each, aligned | Keep duration and counter rollover. |
| Buddy 2/5 status producers → Buddy 1, presence | Each section present or absent | 1 bit per section, positions follow the live schema | OR under mutex after validation. |
| Buddy 1, telemetry base and section cache | Uses existing v3 field domains, precision and presence | 252-byte fixed cache with the existing section encodings | Validate the complete write before touching the accepted cache. Both access paths use the same mutex. |
| Buddy 3 line task, history left and right | Each 0–7, last three raw samples with newest in bit 0 | 3 each | Mask before assignment. The task owns the record. |
| Buddy 3 line task, previous state | 0–4 | 3 | Derived from the existing line state. |
| Buddy 3 line task, both-black debounce, junction reported, probe active, probe saw line | 0 or 1 each | 1 each, together with history/state in a 16-bit unit | Private task state. The published line-status byte is unchanged. |
| Buddy 3 line task, previous duration, both-black origin, state origin | uint32 milliseconds | 32 each | Unchanged elapsed-time subtraction across wrap. |
| Buddy 1 → dashboards, heartbeat, result, map ID and revision | See compact-control schema | 12 B heartbeat, 8 B result, 26 B result with map | Version and bounds checked at both ends. |

The source of truth for unchanged wire fields remains [telemetry-fields.json](telemetry-fields.json), [the live-status codec](../../app_program/subsystem1/buddy1_live_status.c) and [Buddy 4's packed format](../../app_program/subsystem4/s4_packed.h). Heartbeat and result layouts are documented in [Compact MQTT control messages](COMPACT_CONTROL.md).

### Retained control and ISR boundaries

| Boundary | Representation and owner | Reason |
|---|---|---|
| Buddy 1 ingress → controller | Raw length, reconnect epoch and received-at time are uint32. Private RX stages 128 bytes. Validated command uses uint32 opcode and ID. Two RTOS queues, each depth 8. | Full frame validated before admission. The connection epoch and receipt age isolate stale commands. |
| Buddy 2 encoder ISR → measurement and motion task | Encoder edge and timer fields retain uint32 modulo arithmetic. Wheel movement over several samples retains uint64 micrometres and signed original-unit estimates. Wheel sample validity and rejection evidence remain separate. | Preserve precision, sample validation and existing actuator watchdog/STOP authority. |
| Buddy 3 barcode edge capture → decoder | The edge ring uses the existing packed time/level words with elapsed timers and capture diagnostics retained at their original widths. | Preserves capture timing and rejected-run evidence. No new trace range is assumed from observed samples. |
| Buddy 4 IMU → supervisor and telemetry | Four packed words, sequence handshake and one 4-byte event word. Shared odometry uses 16-bit modular distance, 8-bit sequence and validity bit. | Preserve existing resolution, quality and lift-stop evidence. |
| Buddy 5 sonar driver → avoidance | Echo duration and age retain existing timer widths. Range result uses the typed status, with no echo distinct from driver faults. Width, fine-scan angles and bypass geometry retain native arithmetic. | Keeps the nearest-obstacle, width and rejoin decisions on the same inputs. |
| Producers → vehicle controller and STOP lane | Hardware fault word reserves 20 assigned bits. STOP reasons reserve 9. Diagnostic/raw evidence and controller clocks, IDs and generations retain original uint32 ingress. FIFO holds sixteen 40-byte records. | STOP remains independent of ordinary traffic. Preserve unseen high input bits through validation and retain generation checks. |

This follow-up covers the measured storage and wire changes above. The audit has field families for retained motion executor, calibration and obstacle geometry. An exhaustive, per-member proof across those control structures and matched RP2040 timing and contention evidence are open.

### Files changed in the compact-data follow-up

| Area | Files |
|---|---|
| Buddy 1 firmware | `app_program/subsystem1/buddy1_binary.c`, `buddy1_binary.h`, `buddy1_bridge.c`, `buddy1_protocol.c`, `buddy1_protocol.h` |
| Buddy 3 firmware | `app_program/subsystem3/line_follow.c`, `line_follow.h` |
| Transport and codec generation | `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, `tools/generate_buddy1_binary.py` |
| Dashboard | `dashboard/model.py`, `dashboard/web/app.mjs`, `build-memory.json`, `control-wire.mjs`, `index.html`, `state.mjs` |
| Regression checks | `dashboard/test_model.py`, `dashboard/web/test.mjs`, `host_tests/binary_test.c`, `host_tests/protocol_test.c`, `testing/host/test_buddy1_car.c`, `test_buddy1_heartbeat.c`, `test_buddy1_map.c`, `test_live_status.c` |
| Evidence | `docs/protocol/COMPACT_CONTROL.md`, `DATA_REPRESENTATION.md`, `STATIC_RAM_COMPARISON.md`, `Project/ROBOT_DEMO_HANDOFF_2026-11-25.md` |
| Packaged files | `output/firmware/demo-combined-compact-data-2026-11-25/group18_demo_combined_80_mqtt_compact_data_2026-11-25.uf2`, `manifest.json`, `measurements.json`, `PICO_TEST.md` |
