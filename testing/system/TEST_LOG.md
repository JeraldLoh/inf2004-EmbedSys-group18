# Whole-system test plan

## November normal-firmware cases

Use the [shared setup and recording rules](../DEMO_TRACEABILITY.md), [evidence ledger](../DEMO_EVIDENCE.md) and [rehearsal scorecard](REHEARSAL_SCORECARD.md). Earlier planning text is preserved below.

| ID | Priority, owner and requirement | Setup and steps | Pass criteria and evidence | Status |
|---|---|---|---|---|
| SYS-01 | P0 · All · Ten-point autonomous mission | Complete the rehearsal scorecard on the demo course with known fixtures, confirmed map, live broker, preview off and USB unplugged. Issue one fresh START, leave steering to the rover, and end with deliberate pickup from motion. Run three rehearsals on the same named profile. | Score each of the ten checks independently per run. P: 10/10 on each of three rehearsals for repeatability, zero contact and zero intervention after START except pickup. Record completion time, scorecard, actual video and redacted event evidence. | E1–E4 cover individual behaviors across builds. Whole run Not run. |
| SYS-02 | P0 · B1/B2/B3 + team · Autonomous route coverage | Lay out a course with mapped Left, Right, U-turn and a straight fallback, then use a second route with a map change between starts. Confirm barcode layout and allow acquisition distance. Use INT-29a isolated first. | Every fixture takes the intended branch once and exits under line control. No remote steering, stale map route or uncommanded reverse. Record branch and handoff for each, including U-turn measurement and quality-stop failures. | Partial E2–E4. Full route and +5% not demonstrated. |
| SYS-03 | P0 · B4/B2/B1 + team · Highest hump reported per run | Use two or more ruler-measured humps, with tallest in the middle and a smaller one after it. Start new run, leave settling distance between, end with pickup. Repeat in reverse fixture order or fresh run. | Compare each valid estimate with S4-U11 tolerances. Record highest valid height with fixture/event ID and uncertainty. Verify the rover's reporting retains or explicitly identifies the same maximum after smaller humps and resets at next run. Current dashboard shows per-run history and latest reading; if a reviewer can only calculate max from history, mark explicit highest report as Open/Fail for that part and record manual max separately. | Individual events in E2–E4. Highest behavior and calibration Not run. |
| SYS-04 | P0 · All, B2 leads · End of demo and independent stop | Execute INT-25a–e and INT-32 on the assembled normal image, then repeat nominal pickup from active tracking. Confirm STOP and lift independently; try stale, held or no START on a secured fixture. | Every safety path stops motors and remains latched until eligible fresh intent. GP21 uses ≤50 ms P electrical cutoff. Log prior motion, trigger, PWM cutoff, coast and restart outcome. Save video with wheels, dashboard and trigger visible. | E1/E2–E4 latch evidence. Complete safety check Not run. |
| SYS-05 | P1 · B1 leads, B2/B3 + team · Freshness and reconnect | With live telemetry and active local tracking or stand, interrupt Pico Wi-Fi only while browser stays broker-connected. Separately drop broker, then restore each connection with no new commands. Start the stopped variant with old command present in browser state. Run online→offline→online three times. | Heartbeat nominal 5 s, dashboard stale after 15 s without fresh heartbeat while broker reachable, controls locked while offline and on reconnect until fresh heartbeat. Local control remains safe; no reset, old START, or stale data presented as fresh. New telemetry/logs resume, loss visible. P: ≤30 s from restored usable link to new heartbeat; record measured recovery and any browser reconnect action. | E4 healthy hardware link; standalone infrastructure evidence E6. Normal fault/recovery timing Not run. |
| SYS-06 | P1 · All · Normal-image degradation and Tribunal review | Use the dependency scenarios below, exact same UF2, all-good control and one changed component at a time. Run cold boot, runtime loss/invalid data, restore and fresh authorization where applicable. | Verify every tested scenario against its row. Record detected vs unknown, safe function state, surviving functions, recovery and any output pulse. Mark separately the Tribunal document review and normal image hardware acceptance. | Physical matrix Not run. Source supports only parts of it. |
| SYS-07 | P1 · All · Platform and efficiency evidence | Compare final source/manifest/build and rover BOM against write-up p. 1. Inspect C/SDK/μT-Kernel target, template structure, agreed Barr C audit sample, linker map and task priorities, stacks, queues and measurements in INT-12/14. | Platform/target/build identified and reproducible. Record audit scope/deviations, image and RAM sizes, available stack headroom or evidence gap, worst measured task timing and drop counts. No live demo point assigned. | Existing compile/structure evidence. Final profile audit Not run. |

### SYS-06 dependency scenarios

In each scenario record boot/application liveness, firmware ID, surviving outputs, lost/invalid data and its reported age, motor authority, restoration, and whether a reset or fresh authorization was required. On restoration observe at least 15 s without new START. Watch for a hidden reset via uptime. A lack of observable fault detection is a finding. Use protections available on the stand where pickup/obstacle sensing is absent.

| Fault injected independently | Expected safe response and code limitation to check |
|---|---|
| B4 IMU unavailable or invalid, then restored | App, line/route and telemetry remain available where safe. No fresh height/pickup claim from invalid data. Old cap expires around 200 ms at source. B4 retries I2C; verify actual recovery separately for boot and runtime, because successful retry does not establish task bring-up after every boot failure. |
| B3 barcode ADC unavailable or invalid | Digital line can remain available. No decode/route from bad data. Record current pause/straight policy for a mission without a live route and mark the route unavailable if straight cannot complete the course. With an electrically open analogue input, valid-looking numbers can be ambiguous. |
| B3 line input unavailable or held | If driver/line capability is explicitly unavailable, inhibit AUTO or stop affected motion while other functions report. Stuck or unplugged passive input can be indistinguishable from normal black/white. Record actual observability and any unsafe continuation as a gap. |
| B5 sonar loss/no echo, and servo failure separately | Check timeout/status and verify no false width, clear path or completed bypass from missing data. Scan angle command is not measured shaft position. Servo presence may remain unknown without feedback. Verify losing a dependency during avoidance cancels or safely stops affected motion, or document the gap. |
| B2 encoder loss, wheel stall or motor supply unavailable | No uncontrolled fallback or false completion. Inhibit/stop affected movement where fault observable while sensing/logging survives. Confirm ordinary motion fault cannot be cleared by reconnect or START alone. Distinguish supply or feedback loss from successful zero-speed command. |
| B1 network/broker unavailable at boot and during use | Physical local control and onboard sensing survive where safe; see SYS-05 for outage/reconnect. Wi-Fi credentials and broker settings remain private. |

**Tribunal 2 analysis review.** Owners review each dependency row for trigger evidence, observability limits, affected capability, safe response, reasoning, surviving functions and re-entry conditions. Evidence is the team's technically justified analysis with recorded review disposition. The Tribunal explicitly did not require implementation. The normal-UF2 trials above are the team's separate engineering verification.

No system tests have been run. Define mission-specific acceptance against the
brief and course constraints after subsystem/runtime integration.

Proposed coverage: autonomous line/junction route, barcode lifecycle, valid hump
height and terrain speed limits, obstacle profile/bypass/original-line recovery,
AUTO behavior on communication loss, manual lease expiry, physical stop, bounded
recovery and sensor/motion faults. Keep unverified claims out of readiness status.

## Actual results

| Date | Test ID | Requirement | Commit/profile | Result | Evidence | Notes |
|---|---|---|---|---|---|---|

## November evidence review

| Date | Test ID | Requirement | Commit/profile | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | SYS-01 | Rubric points 1–10 | Multiple separately identified profiles | Not run | [Evidence E1–E4](../DEMO_EVIDENCE.md) | Historical feature observations across sessions. Use one scorecard per complete physical attempt. |
