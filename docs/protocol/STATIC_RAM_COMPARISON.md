# Recorded static RAM

## Combined compact-data candidate

All three images were measured from their ELFs using the same effective profile, Pico SDK 2.3.1 and Arm GNU Toolchain 15.2.Rel1. The last two working trees are on the same source base as the original next-junction image, `da4e9d91e78a53f9e77ba030b086b44b38279a13`. The dashboard now shows original and final values. The checkpoint is in the measurements JSON with the profile and artifact hashes. [Measured profile and artifact hashes](../../dashboard/web/build-memory.json)

[Build command and Pico test steps](COMPACT_DATA_PICO_TEST.md) · [Field ranges and bit widths](DATA_REPRESENTATION.md)

| Measurement | Next-junction reference | First storage checkpoint | Compact-data candidate |
|---|---:|---:|---:|
| ELF `.data` | 404 B | 404 B | 404 B |
| ELF `.bss` | 112,996 B | 110,948 B | 109,780 B |
| **Recorded static RAM** | **113,400 B** | **111,352 B** | **110,184 B** |
| ELF `.text` | 465,008 B | 465,228 B | 466,796 B |
| Flash load including initialized data | 466,132 B | 466,352 B | 467,920 B |
| Separate RAM vector section | 192 B | 192 B | 192 B |

The candidate saves **3,216 B of static RAM (2.84%)** and adds **1,788 B of flash load** against the original. Against the first checkpoint it saves **1,168 B of static RAM** and adds **1,568 B of flash load**. The sections include reserves allocated at link time. Runtime stack, heap and timing have yet to be measured on the Pico.

| Record or MQTT payload | First checkpoint | Candidate |
|---|---:|---:|
| Buddy 1 outbox symbol | 628 B | 468 B |
| Buddy 1 retained telemetry snapshot | 504 B | 252 B |
| Buddy 2/5 live staging, including previous unused Buddy 3 member | 148 B | 68 B |
| Buddy 3 live snapshot and publication state | 56 B | 28 B |
| Buddy 1 inbound and validated command records | 140 B and 16 B | 140 B and 16 B |
| Controller FIFO stride and allocation | 40 B, 704 B incl. kernel lengths | 40 B, 704 B incl. kernel lengths |
| MQTT heartbeat | ≤65 B JSON | 12 B binary |
| MQTT command result with map | ≤131 B JSON | 26 B binary |
| MQTT result without map | Variable JSON | 8 B binary |
| MQTT telemetry, live status, diagnostics and inbound command | 252 B max, 96/100 B, 52 B, 8/22 B | 252 B max, 96/100 B, 52 B, 8/22 B |

Payload sizes exclude MQTT framing and topic names. The readable log format and 256-byte chunk limit are preserved. The firmware ID changes the maximum message size by five bytes.

- **Correctness.** 27 CTest cases, 70 browser cases and 10 Python dashboard cases passed. The C telemetry encoder matched the existing 252-byte golden file. All sixteen presence masks matched the cached and uncached encoders. Invalid writes left the cache unchanged. The same control vectors were checked in C, JavaScript, Python and an isolated loopback Mosquitto round trip.
- **Line behavior replay.** The old and packed line-follow builds emitted identical state, left/right flags and remembered side across 20,000 deterministic samples covering timer wrap. The private line record measured 20 → 16 B on the host.
- **Prior failures.** The existing Buddy 1 car and live-status fixtures had expectations for an older acceptance path and frame size. Both were updated to check current semantics, including sanitized values, full-width counters, and v4/v5 layouts. The historical runner gaps below remain separate.
- **PC timing.** Three paired runs of 500,000 full v3 encodes using GCC 15.2.0 and `-O2` yielded median **848 → 528 ns** per encode with equal checksums. This benchmark measures the isolated synchronous encoder on this PC. RP2040 code inspection found native atomic word loads and stores with memory barriers and no atomic runtime helper. Per-task service time, cross-core contention, stack/heap high-water marks and physical regression are open.

## First storage checkpoint

Measured on **25 November 2026 (SGT)** using the combined Wi-Fi profile, Pico SDK 2.3.1, and Arm GNU Toolchain 15.2.Rel1. Both ELFs use source base `da4e9d91e78a53f9e77ba030b086b44b38279a13`. The baseline is the integration working tree that produced the packaged next-junction UF2. Its UF2 hash matched before any representation edits. The revised build adds the representation patch to that same working tree. [Measured profile and artifact hashes](../../dashboard/web/build-memory.json)

| Linked measurement | Combined baseline | Packed storage | Change |
|---|---:|---:|---:|
| ELF `.data` | 404 B | 404 B | 0 B |
| ELF `.bss` | 112,996 B | 110,948 B | −2,048 B |
| **Static RAM (`.data + .bss`)** | **113,400 B** | **111,352 B** | **−2,048 B (−1.81%)** |
| ELF `.text` | 465,008 B | 465,228 B | +220 B |
| Flash load bytes including initialized data | 466,132 B | 466,352 B | +220 B |
| RAM vector section | 192 B | 192 B | 0 B |

Individual section values come from `arm-none-eabi-size -A`. The GNU summary `size -B` counts the separate 192-byte RAM vector section in its BSS-like total. Flash load bytes sum `p_filesz` for program segments with flash physical addresses from `arm-none-eabi-readelf -lW`. ELF and UF2 hashes are in the measurement JSON. Reserved memory pools within `.data` or `.bss` are included. Live stack and heap use, peak usage and Pico timing require separate runtime measurements.

## Storage and message sizes

| Record or allocation | Baseline | Revised | Evidence |
|---|---:|---:|---|
| Buddy 1 queued transmit slots and occupancy | 2,640 B | 628 B | ELF symbol sizes, 5 latest-packet slots in both builds |
| Buddy 1 radio-facing packet copy | 524 B | 264 B | Arm ELF type info |
| Buddy 1 inbound command record | 140 B | 140 B | Arm ELF type info, 8 records plus 4 B kernel length per record |
| Buddy 1 validated vehicle-command record | 16 B | 16 B | Arm ELF type info, 8 records plus 4 B kernel length per record |
| Buddy 3 barcode map and revision | 40 B | 8 B | ELF symbol sizes |
| Buddy 4 event | 4 B | 4 B | Existing uint32 format, 8 records plus 4 B kernel length per record |
| Controller mailbox record | 56 B | 40 B | Arm ELF type info |
| Controller mailbox data allocation | 960 B | 704 B | 16 × (record + 4 B kernel length), requested at startup from the existing kernel pool |
| Controller task local frame allocation | 156 B | 172 B | Arm disassembly prologue, excluding saved registers and callees |

The 256 B mailbox allocation reduction is separate from the measured static RAM saving. The reserved pool remains in the build, so the allocator can use the recovered space for other objects. Outbox and map symbols account for 2,044 B of the static reduction. The remaining 4 B come from linked layout alignment.

| MQTT topic | Baseline payload | Revised payload |
|---|---:|---:|
| `/telemetry` v3, all Buddy 2–5 sections | 252 B | 252 B |
| `/live_status` v4/v5 | 96/100 B | 96/100 B |
| `/diagnostics` v2 | 52 B | 52 B |
| `/command` START, STOP, MAP_SHOW or MAP_SET | 8 B or 22 B | 8 B or 22 B |
| `/heartbeat` maximum JSON | 65 B | 65 B |
| `/command_result` maximum JSON with successful route-map echo | 131 B | 131 B |
| `/log` maximum including ASCII firmware ID and 256 B chunk | 362 B | 356 B |

MQTT framing, topic names and TCP/WebSocket overhead are excluded. The 252-byte C telemetry fixture matched the repository's v3 golden bytes exactly. The log maximum differs because the firmware ID is shorter.

## Correctness and performance evidence

- Passed **18 focused native firmware tests**, **4 controller mailbox tests**, **61 browser tests**, and **9 Python dashboard tests**. The mailbox tests exercised full FIFO depth, urgent STOP on a full queue, latest-only continuous control, signed result extremes, uint32 goal ID and timeout, and invalid enum rejection before packing. The map and outbox fixtures covered replacement, unassignment, invalid map input, full payload size, and preserving a newer packet when an older packet is restored.
- Reproduced two failures in the unchanged Buddy 1 source and in the revised source. `buddy1_car_tests` expects Buddy 3's fast live-status submission to reject an invalid record immediately, while the current implementation accepts its snapshot and sanitizes at encoding. `buddy1_live_status_tests` expects the older 80-byte v3 live frame and the current code produces a 96-byte v4 frame in that fixture. The older `verify_buddy1_car.py` runner also omits `barcode_map.c` at link time on the baseline. The current CMake targets were used for the test comparison. The broader Buddy 2 startup test build was already blocked by missing integration stubs.
- The PC map benchmark used GCC 15.2.0 with `-O2`, 8 million mixed-character lookups and 400,000 map replacements per run. Median of seven paired runs was **2.25 → 3.12 ns** per lookup and **270 → 35 ns** per table replacement. Both executables produced the same checksum and revision. Lookup now examines up to four fields. Replacement publishes one atomic word in place of 36 atomic byte stores. [Benchmark source](../../testing/host/bench_barcode_map.c)
- The controller still uses its existing 2,048-byte task stack. Disassembly provides the local-frame comparison above. Hardware latency, task stack high-water marks under Wi-Fi load, and car behavior remain pending.
