# Change Requirement 1 implementation plan

**Target.** Meet the course requirement for minimum-bit application data across Buddy 1–5, the vehicle controller, task and ISR boundaries, inter-task queues, and MQTT. Deliver an auditable field table, matched RAM and message measurements, correctness and timing evidence, and one combined Wi-Fi UF2.

**Starting point.** Use the local combined integration working tree based on `da4e9d9`. The existing compact-storage candidate contains the Buddy 1 outbox, Buddy 3 atomic map, and controller mailbox changes. Its matched next-junction baseline used 113,400 B of static RAM. The compact-storage candidate uses 111,352 B. The controller mailbox is 40 B per record, down from 56 B. Full v3 telemetry is 252 B in both. [Current field audit](DATA_REPRESENTATION.md) · [Matched build measurements](STATIC_RAM_COMPARISON.md)

## 1. Pin the baseline and complete the field inventory

1. Capture the exact source base and integration changes, compiler and Pico SDK versions, effective build profile, and SHA-256 for the inspected ELF and UF2. Save the original next-junction baseline and the existing compact-storage candidate as separate references. Compare the final candidate to both so the report distinguishes total savings from the additional work.
2. Record the baseline queue depths, record sizes, kernel slot overhead, and actual accepted MQTT payload sizes. Include optional sections, map echo, and diagnostics. Record exact ELF `.data` and `.bss`, the separately reported vector section, and flash load bytes.
3. Expand the current audit into one row per relevant field, with array or record counts where applicable. A row must identify the producer, reader or ISR, source path, unit, valid input range, storage range, signedness, sentinel and rollover rules, required bits, physical width and alignment, validation point, and publication method. Use the existing generated schema as the source of truth for the wire fields.
4. Derive integer widths from the representable domain before choosing storage. Include sentinels and negative values in the domain. Preserve full-width clocks, counters and IDs wherever a smaller modulo would alter age, ordering, or event accounting. Derive sensor ranges from configuration and producer arithmetic. Treat observed floor readings as additional evidence. Record when a wide temporary is needed to validate input, accumulate a result or perform a calculation with the original precision.
5. Reproduce the two existing Buddy 1 fixture failures before changing the relevant transport. For each inventory row, record whether the existing representation is justified, which patch will change it, or what evidence is still needed.

**Gate.** Every relevant field or repeated field family has a traced owner and range. Safety-related fields have explicit validation and synchronization decisions. Baseline artifacts and failures can be reproduced.

## 2. Make the shared representations safe to narrow

| Area | Implementation | Required checks |
|---|---|---|
| Buddy 3 live status shared with Buddy 1 | Replace the plain-struct concurrent copy with a packed snapshot published using native atomic words and the existing generation handshake. Publish one word where the fields fit. For multiword data, use atomic accesses for each word and a sequence check that rejects a mixed generation. Keep bounded reads and the existing reader behavior on contention. | Compare every field after encode/decode, including map revision. Force interleaved reads and writes and sequence wrap. Check the Arm build emits appropriate native accesses and introduces no blocking atomic helper. |
| Shared faults and validity | Inventory the controller fault mask, subsystem detail masks, actuator stop reasons and communication state. Assign one bit per confirmed fault to its owning evidence path. Define set, clear and retention rules. Keep an ordinary sonar no-echo result and a disconnected or faulted sensor distinguishable so the controller sees the same evidence as the baseline. | Simultaneous set and clear operations, unrelated bits surviving an update, fault recovery, STOP with queued traffic, and telemetry displaying unknown versus a confirmed fault. |
| Other task and ISR boundaries | Audit producer ownership, word alignment, flags and counters. Use an owner-local packed value and a whole-word publication under the existing mutex, native atomic or IRQ/spinlock appropriate to the path. Keep irreversible STOP and watchdog actions in their existing priority lanes. | Check the RP2040 build and exercise queue-full STOP, STOP/restart races, rollover, and mixed-snapshot rejection. Record lock hold and critical-path timing against the reference. |

**Gate.** Each shared field has a documented publication rule. The host race fixtures pass, and the Arm inspection confirms native access and safe synchronization for the packed storage.

## 3. Finish the native representation in small patches

| Order | Scope | Specific work and regression gate |
|---|---|---|
| 1 | Buddy 1 transport staging | Pack bounded status and validity flags in retained per-Buddy snapshots. Reuse validated representations where they can reduce duplicate storage without lengthening a sensor task's critical section. Keep full-width raw rejection details until they are reported. Compare exact payload bytes and check snapshot age, restore and contention. |
| 2 | Buddy 2 motion and encoders | Narrow bounded mode, source, result and validity fields in retained records. Audit signed speed, PWM effort, encoder deltas, odometry and retrace separately. Keep the precision, accumulator widths, lease arithmetic and watchdog behavior used by the controller. Replay normal drive, a measured pivot, a rejected encoder sample, retrace, and urgent STOP. |
| 3 | Buddy 3 line and barcode | Keep the existing compact line byte and route map. Pack additional proven flags, decoder states and counts. Derive the actual stored microsecond interval range before touching capture or trace arrays. Verify both scan directions, malformed and partial reads, map replacement, and the one-use next-junction instruction. |
| 4 | Buddy 4 terrain and pickup | Audit the existing four-word status, event word and odometry word, then narrow any remaining bounded retained fields that have a proven range. Exercise calibration, angle limits, signed pitch, hump validity, stale/I²C fault, pickup, and modulo rollover. |
| 5 | Buddy 5 obstacle and rejoin | Pack bounded scan state, servo command, validity, side/attempt fields and retained telemetry staging. Keep full timing and the arithmetic precision used for edge finding and bypass geometry. Replay close, no-echo, unknown width, both bypass sides, rejoin search and fallback. |
| 6 | Vehicle controller and queues | Complete the inventory for shared states and queue messages. Keep the 16-message controller depth and urgent lane. Where another mailbox can safely use the smallest supported fixed record, measure the new stride and preserve ordering. Test admission boundaries, wrap, queue-full behavior, typed round trips and independent STOP. |

For each patch, validate the original-width value before narrowing, add a range assertion for generated constants or enum counts where applicable, and compare valid outputs to baseline fixtures. Use smaller typed views or explicit masks and shifts where the field requires them. Record any byte or word padding required for alignment and atomic access. Keep PWM limits, PID gains, pins, task priorities, watchdog settings, control periods and driving choices pinned to the measured build profile across all six patches.

**Gate.** The inventory contains a field-level justification for all remaining widths. Each patch has bounded invalid-input tests and a passing regression for its feature. Retained wider storage has an explicit precision, range, compatibility or concurrency justification.

## 4. Finish compact MQTT application messages

1. Keep the existing binary telemetry and START, STOP and route-map commands compatible. Audit every field in the live-status and diagnostics schemas for padding or unused values. Review alternate profiles and existing dashboard clients before publishing a new layout.
2. Define versioned binary heartbeat and command-result schemas. Derive widths for status enums, flags, map presence and action assignments. Keep full uint32 command IDs and revisions. Encode the route-map echo as packed values and preserve command-result meaning, including queued, applied and rejected.
3. Update the Pico encoder, web dashboard and Python decoder together. Keep legacy decoding during rollout so the same dashboard can compare baseline and candidate UF2s. Maintain text for the separate human-readable serial-log export and document that topic as diagnostic text.
4. Generate or check golden vectors across C, JavaScript and Python. Include max IDs, timestamp wrap, stale/retained packets, invalid/reserved fields, short buffers, duplicate map assignments and optional map echo. Confirm that a heartbeat alone controls live connection state and that decoding errors leave previous valid state intact.
5. Measure actual topic payload lengths and compare whole frames under the same data, with any framing overhead itemized separately. Test a local MQTT round trip and repeat on the real Pico.

**Gate.** Firmware and both dashboard decoders agree on each binary format and its error handling. The Pico accepts the same normal commands and exports the same diagnostic information. All structured application payloads covered by the requirement have explicit binary schemas.

## 5. Measure performance and run the Pico regression

1. Build the references and final candidate using the same combined Wi-Fi flags, toolchain, SDK and linker setup. Record ELF sections, flash load, queue record and pool bytes, and on-wire payloads. Update **Recorded static RAM** using the final comparison, including date, exact profile and source identity. Keep runtime stack and heap evidence separate.
2. Collect matched timing traces under idle and active MQTT load. Observe Buddy 2's 20 ms executor and 100 ms watchdog, controller dispatch, Buddy 4's 20 ms sampling, Buddy 5's 10 ms task service plus sonar transaction time, and barcode capture. Record call duration, missed deadlines, input and queue drops, and stack high-water marks where instrumented. Use the same instrumentation in both images. A PC benchmark covers isolated operations. The Pico traces determine whether timing still meets the existing deadlines and the watchdog has margin.
3. Use the same course, placement and start conditions for paired baseline and candidate runs. Record straight tracking and bends, mapped barcode with a delayed junction, hump and pickup, right and left obstacle bypass and rejoin, dashboard and physical STOP, and broker disconnect/reconnect. Record trial counts, failures, firmware IDs and the wireless logs. Repeat any regression against the baseline to locate whether it predates the storage changes.
4. Compare known functional outcomes, fault handling and timing rather than marking a run successful just because the UF2 linked. Record any measurable trade-off, such as extra CPU work or flash to save RAM. Keep the existing driving settings during the comparison.

**Gate.** The final UF2 passes linked-image checks and paired physical regression. The evidence shows required behavior and acceptable timing under load, with pre-existing failures and unresolved physical checks clearly identified.

## 6. Assemble the Week 13 evidence

| Required evidence | Deliverable and completion condition |
|---|---|
| Data representation table | One reviewed inventory, covering each relevant field, with ranges, widths, owners, source links, and reasons for alignment or retained precision. |
| RAM comparison | Original baseline, existing compact-storage checkpoint and final build in a single matched table. Include exact ELF section sizes, flash delta, hashes, toolchain, profile and date. |
| Message sizes | Matched record sizes, queue depth and overhead, and before/after MQTT payloads for all structured topics. Include binary golden fixtures. |
| Correctness and performance | Host results, recorded baseline failures, Pico runs with logs, and a comparison of control/service timing against the original budgets. |
| Repository and demo | Reviewable patches, evidence and a concise Pico test guide in the agreed GitHub branch, plus one named combined Wi-Fi UF2 with manifest. Publish after repository review and the paired hardware test. |

**First implementation patch.** Complete the field inventory for Buddy 3's live-status snapshot, then replace its cross-task publication with a validated packed snapshot. Verify it preserves the reader's timestamps, route map revision and displayed result. Use this patch as the template for the remaining shared records.
