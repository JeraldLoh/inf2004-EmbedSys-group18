# SYS-01 rehearsal scorecard

Copy one scorecard per run. Score the ten rubric rows from the same physical run. The judged demo allows up to three runs and the latest score replaces earlier scores. The team can stop after any run. Record all rehearsal and judged attempts individually.

| Session field | Entry |
|---|---|
| Date/time SGT, operators, rehearsal or judged attempt number | |
| Firmware ID, commit or source snapshot, UF2 SHA-256, full profile | |
| Dashboard revision, boot/session, battery/load, course and fixture IDs | |
| Confirmed map, actual hump heights, actual obstacle width | |
| START/end times, elapsed time, intervention or fault | |
| Evidence references, log loss counters, tester/reviewer | |

| Point | Exact rubric | Observable in this run | Score 0/1 and evidence |
|---|---|---|---|
| 1 | Detect a barcode. | Capture at marked barcode with fixture/time reference. | |
| 2 | Decode the barcode and obtain the code. | Report correct known printed character. | |
| 3 | Measure hump height. | Show valid height, uncertainty and actual ruler height. | |
| 4 | Follow the line. | Stay on track on straights and bends without assistance, except planned obstacle bypass. | |
| 5 | Follow the instruction from the barcode. | Show confirmed map, capture and physical branch taken. | |
| 6 | Detect an obstacle. | Forward detection and stop/scan response before contact. | |
| 7 | Measure obstacle width. | Show valid both-edge width, ruler width and error. | |
| 8 | Navigate around the obstacle and rejoin the line. | No contact; visible original-line recovery and continued tracking. | |
| 9 | Send telemetry data via MQTT. | Fresh real heartbeat and telemetry with USB unplugged and preview off. | |
| 10 | Stop motors when the car is picked up; this signals the end of the demo. | Show pickup from powered motion, motor stop/latch and end state. | |

**Total /10** ______   **Highest valid hump this run, event and reporting source** ______

### Physical review checklist

- [ ] Pin checks, cold boot with no motion, GP21 held denial, physical cutoff timing, local and remote START and restart interlocks (S2-U10, INT-25a–e, INT-32).
- [ ] Wheel counts with independent reference, loaded PID plots and report, straight drift, physical distance/angle and fault-free U-turn exit for chosen profile (S2-U11–U14, INT-29a).
- [ ] Known code contrast, positive/negative controls, three actual route turns and straight, route expiry and guarded reverse/reread in a separate trial (S3-T02–T07, INT-28a–c, INT-29a–b, INT-30–31).
- [ ] Known-height repeated fixtures, largest hump retained across the run, pickup from active motion and tilt controls (S4-U10–U12, S4-L01–L02, SYS-03–04).
- [ ] Known-width both-edge scan, both bypass sides, stop distance, stable exit, boxed/removed fixture behavior (S5-U10–U11, S5-M01–M06, INT-10, INT-26a–b, INT-27b–c).
- [ ] Assembled live telemetry and export, stale display, reconnect with no stale movement, loop timing with load, one-at-a-time fault matrix including restore (INT-12–14, SYS-05–06).
- [ ] Append each actual result with provenance and scope. Save redacted evidence, and link only actual recordings. Apply scorecard to a complete run and retain each attempt.
