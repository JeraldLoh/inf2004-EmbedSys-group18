# Group 18 demo verification and traceability

Use the case tables in the seven test logs and the [rehearsal scorecard](system/REHEARSAL_SCORECARD.md). The November procedures govern a normal combined build where a migration-era or bench plan differs. Preserve older procedures and results with their original profile. The case IDs in the left column of each November table are the IDs to use when recording a run.

## Sources and test profile

- Course handouts `Project Briefing.pdf` (pp. 2–10) and `Project Write-up.pdf` (pp. 1–11) cover buddy deliverables, platform, autonomous mission, highest hump, profiling and reliability. Both are in the team's course materials under `Project/Others/`.
- Team handoff `ROBOT_DEMO_HANDOFF_2026-11-25.md` supplies the exact ten-point rubric and completed-work ledger. Team runbook `BUDDY2_DEMO_REVIEW_AND_RUNBOOK_2026-11-26.md` covers setup and interpretation of the later physical exports. Both are in the team's project workspace.
- `Tribunal_2_NewRequirement.txt` in the team's Tribunal 2 materials defines the technical justification exercise. The [course test template](../docs/template-reference/testing/README.md) and [video template](../docs/template-reference/videos/README.md) define recordkeeping.
- [TEAM §5](../TEAM.md#5-interfaces), interface IDs. Use the runbook and profile's source for pins and behavior because parts of TEAM and the original test guides describe older allocations.

Local check on 25 November 2026 SGT found active integration branch `demo-integration-20261124` at `da4e9d91e78a53f9e77ba030b086b44b38279a13` with existing source, test and dashboard changes. The published Buddy-2 checkout was clean and detached at `ac005fc943454e31a9d583bdb09eab003c109d2b` (combined firmware parent `24a0807`). Local dashboard changes include the compact barcode reader and three-direction map editor. Record the served dashboard checkout and local revision/diff separately from the flashed UF2. See [evidence profiles](DEMO_EVIDENCE.md) for what each source supports.

Default test intent is the exact working normal combined CAR_RUN image selected by the team, with MQTT on and physical STOP available. Check firmware ID and manifest before every session. The newer retry80 variants enable guarded barcode retry at 80 mm/s and disable automatic line-loss reversal. The original 80 mm/s baseline had both reversals off, fixed 350/450 ADC thresholds, and operator-confirmed straight/bend behavior. The combined profile enables barcode calibration, uses 80 mm/s cruise and 40-for-40-ms damping, keeps optical off-ground and line PID off, and enables watchdog-only local retry. The +5% candidate adds the U-turn goal override. Treat trials on each as separate evidence. Use a bench image only for a case explicitly marked bench.

## Requirement-to-test matrix

| Source | Requirement | Subsystem cases | Integration and system cases |
|---|---|---|---|
| Rubric 1 / Mission 2 | Detect a barcode. | S3-T01, S3-T02, S3-T03 | INT-28a, SYS-01 |
| Rubric 2 / Mission 2 | Decode the barcode and obtain the code. | S3-H07, S3-H08, S3-T03 | INT-28a, INT-30, SYS-01, SYS-02 |
| Rubric 3 / Missions 4–5 | Measure hump height. Detect humps and report the highest valid peak for the run. | S4-U10, S4-U11 | INT-13, SYS-01, SYS-03 |
| Rubric 4 / Mission 1 | Follow the line. Handle straights, bends and modest track/lighting variation. | S2-U12, S2-U13, S3-T04 | INT-11, INT-25d, INT-27a, SYS-01 |
| Rubric 5 / Mission 3 | Follow the barcode instruction. Exercise mapped left, right, U-turn and automatic straight. | S2-U13, S3-T05, S3-T07, S1-U10 | INT-28a–c, INT-29a–b, INT-31, SYS-01, SYS-02 |
| Rubric 6 / Mission 6 | Detect an obstacle on or near the line. | S5-U10, S5-M01 | INT-10, INT-26a, SYS-01 |
| Rubric 7 / Mission 7 | Measure obstacle width. Profile shape/position, nearest return and side clearance where observable. | S5-U11, S5-M02, S5-M03 | INT-26a, INT-27b, SYS-01 |
| Rubric 8 / Missions 8–9 | Bypass without contact and reacquire the original line. | S5-M03, S5-M04, S5-M05, S5-M06 | INT-26b, INT-27b–c, SYS-01 |
| Rubric 9 / Mission 10 | Send telemetry via MQTT. Confirm wireless diagnostic export. | S1-U10, S1-U11 | INT-12, INT-14, INT-31, SYS-01, SYS-05 |
| Rubric 10 | Stop motors when the car is picked up, ending the demo. | S4-L01, S4-L02 | INT-32, SYS-01, SYS-04 |
| Brief B1 / IF-01, IF-08, IF-09 | Command validation, connection, heartbeat, freshness and bounded service. | S1-U10, S1-U11 | INT-12, INT-14, INT-31, SYS-05 |
| Brief B2 / IF-05, IF-06, IF-07 | Drivers, calibrated encoder speed/distance, PID tuning, physical accuracy, safe starts/stops, typed motion outcomes. | S2-U10–U14 | INT-21, INT-25a–e, INT-29a, INT-32, SYS-04 |
| Brief B3 / IF-02 | Sensor calibration, junction evidence, single-use navigation and guarded retry. | S3-T01–T07, S3-H07–H08, S3-H13 | INT-11, INT-28a–c, INT-29a–b, INT-30, INT-31 |
| Brief B4 / IF-03, plus B2 odometry to B4 | IMU calibration, attitude/terrain and motion events, validity and expiry. Assess turn-rate/impact reporting against the broader brief. | S4-U10–U12 | INT-13, INT-32, SYS-03, SYS-06 |
| Brief B5 / IF-04 | Safe range/servo, coarse and fine profile, bounded planning and recovery. | S5-U10–U11, S5-M01–M06 | INT-10, INT-26a–b, INT-27b–c, SYS-06 |
| Write-up pp. 1, 11 | Pico/Robo Pico, μT-Kernel, C/Pico SDK, repo template, Barr C and resource efficiency. Autonomy after START. | S2-U12, S4-U10, S1-U11 | INT-12, INT-14, SYS-01, SYS-07 |
| Tribunal 2 | Explain how unavailable or invalid inputs affect dependencies and justify each safe response. | — | SYS-06 analysis review |
| Team normal-firmware requirement | Capability loss at power-on and during use, graceful degradation, honest availability and safe recovery on the same UF2. | S1-U11, S2-U14, S4-U12, S5-U11 | INT-12, INT-13, INT-14, INT-32, SYS-05, SYS-06 |

## Execution and evidence conventions

1. **P0 demo gate:** verify pins, STOP, boot/START/latches and loaded drive first. Qualify barcode route, known hump, both-edge obstacle width and bypass before rehearsal. Use the active USB-free dashboard with preview off and confirm a real heartbeat. Local START is GP20 hold at least 1 s and release. Physical STOP is GP21. Have a spotter and an independent motor-power cut. Use a secured stand first for actuator, start/stop and fault cases. Use marked floor fixtures, exclusion space and a spotter for travel. Fit or isolate wiring with power off. Simulate live loss only through a team-reviewed isolated fixture or non-electrical method. Record **Blocked** when there is no safe injection method.
2. **P1 engineering:** collect PID/load and travel/angle statistics, retries and fault/reconnect data in dedicated runs. Keep same UF2 throughout each fault cycle. Restore the baseline fixture between cases and rerun STOP/ready checks after any fault injection. Avoid network or fault injection during scored rehearsal.
3. **Team proposals (`P` in case tables):** confirm numerical criteria before runs. Motor STOP ≤50 ms to electrically disabled output is the earlier Buddy 2 target, awaiting target verification. Default floor checks use 3/3 nominal repetitions unless a case specifies more. Course supplies mission/rubric behaviors without these numerical tolerances. Report each trial and worst result, and record any agreed deviation prior to running.
4. For each attempt record date/time SGT, operator, ID, commit and local source provenance, complete profile/manifest, UF2 hash and runtime ID, dashboard revision, setup, supply/load, fixture and instrument references, actual measurements, result and evidence. A boot tag and sequence/uptime identify a physical session. Append one row per result to the owning log. A redacted extract, timing trace, photo or measurement CSV can be saved under its `logs/` folder once reviewed. Keep credentials, UF2s and full raw exports private. Record lost batches, sequence gaps and resets where applicable. Historical aliases and limitations are in [DEMO_EVIDENCE.md](DEMO_EVIDENCE.md).
5. Use **PASS**, **FAIL**, **Inconclusive**, **Blocked** or **Not run** for the stated procedure. Use **Partial** for earlier evidence that meets only a part of the procedure. Host/build evidence can pass its own scope. Record hardware criteria separately. Mark uncertain fixture truth, timing, or missing evidence as open. Leave video links empty until there are recordings.
