# Buddy 5 Obstacle scanning and avoidance test record

## November demo cases and bench follow-up

Use the [shared setup, tolerances and recording rules](../DEMO_TRACEABILITY.md) and [evidence ledger](../DEMO_EVIDENCE.md). These procedures govern the normal combined image. Earlier plans and results below apply to their recorded profiles.

| ID | Priority, owner and requirement | Setup and steps | Pass criteria and evidence | Status |
|---|---|---|---|---|
| S5-U10 | P0 · B5 · Sonar and safe electrical timing | With wheel power isolated, use a scope and flat targets at 10/18/20/22/50/100 cm. Check Echo electrical levels and timing, turn servo to verified limits, then try no target and too-close target. Use an approved isolation for ECHO-open only if available. | P: valid range within ±1 cm on flat reference targets, no hardware overvoltage, scan angles match left/right axes. No echo/timeout remains typed and is not a measured clear return. Save scope and truth/measurement CSV. | Old S5-B01–B03 are bench procedures. Current measurement series Not run. |
| S5-U11 | P0 · B5 · Coarse/fine profile and bounded proposals | With fixed known-width obstacle and clear background, collect per-angle coarse/fine range, freshness, edges, nearest return, position and side clearances. Repeat with one edge occluded, one side blocked and both sides blocked. | Full width requires two measured edges. One-edge/invalid scan may yield width unavailable and limited planning. No falsely valid clearance or unauthorized motion. Save per-angle trace and fixture geometry. Use INT-27b/c for physical verdict. | E3 one-edge detection/rejoin and another aborted encounter. Both-edge accuracy and boxed recovery open. |
| S5-M01–M06 | P1 · B5 · Obstacle metrics | Use the six existing metric IDs individually and their existing CSV procedure. Record 20 fixture encounters across centre/offset and blocked-left/right placements, with 10/30 cm depth trials. Match result by boot/session, sequence and encounter number. Measure face width, depth, clearance, minimum distance and time. | P: ≥19/20 detected, no contact, ≥90% valid both-edge widths and each valid width within ±30%; choose only available side. Require video-confirmed original-line rejoin and 0.5 m stable exit for each successful bypass. Report rate, all aborts, clearance, depth lower bound and worst time. | E3 partial. Existing script output is engineering evidence; scores in the handoff remain the live rubric. |

This is a proposed plan, not test evidence. No hardware test was run during
folder migration. Define exact preconditions, thresholds, procedures and evidence
before execution. Electrical work requires G1; active motion needs explicit
bounded-test approval and a physical power cut.

## Proposed cases

| ID | Purpose | Method and acceptance | Status |
|---|---|---|---|
| S5-U10 | Validate sonar range and Echo timing safely | Procedure and quantitative acceptance to be agreed | Not run |
| S5-U11 | Validate scan/profile and bounded recovery proposals | Procedure and quantitative acceptance to be agreed | Not run |
| S5-B01 | Sensor distance accuracy | Bench firmware; flat object at 10/20/50/100 cm; pass if within +/-1 cm | Not run |
| S5-B02 | Sensor edge cases | Open space, 1 cm, ECHO unplugged; pass if no-echo / too-close / sensor-fault | Not run |
| S5-B03 | Servo movement and safe range | Boot menu `c`; smooth, no jitter; saved range survives reset; record both limits | Not run |
| S5-B04 | Obstacle detection | Object brought under 25 cm; pass if detect + STOP + scan starts | Not run |
| S5-B05 | Scan, width and side choice | Servo + hand-turned car scan; width within +/-30% of measured; walled side avoided | Not run |
| S5-B06 | Boxed-in handling | Rescan x2, reverse, repeat, then FAILED (no clear path) | Not run |
| S5-B07 | Obstacle removed mid-scan | Resumes line following without bypass | Not run |
| S5-T01 | Kernel task bring-up (`B5_RUN=1`) | Flash with motors raised/disconnected; console shows `[B5 RUN] ... running`; hand under 25 cm prints `[B5] obstacle ahead`; motors never move; GP21 STOP works | Not run |
| S5-T02 | Avoidance in the whole car (`CAR_RUN=1`) | INT-25..27 in `../integration/CAR_RUN_TEST_PLAN.md` | Not run |
| S5-B08 | Bypass path and return | Follow motion prompts by hand; legs mirrored, no contact, ends facing line direction | Not run |

Bench procedures: `bench_firmware/README.md`.

The code's subsystem5_test() is only a SKIP placeholder and is not invoked by
the inactive app_main. Do not report it as a subsystem PASS.

## Week 10 metrics (briefing page 8)

The `CAR_RUN` firmware prints one `[B5 RESULT]` line per encounter (outcome,
detection-to-line time, scan time, profile width and edges, side, clearances,
whether the rejoin turned along the line) and a `[B5 TOTALS]` line.
`b5_metrics.py` turns a console log into the page 8 table and grades.

| ID | Metric | Procedure | Status |
|---|---|---|---|
| S5-M01 | Obstacle detection rate | Place the obstacle on the line ahead of the car, N >= 20 runs. Record each run in `trials.csv`; leave `n` empty when no RESULT line appears (missed or hit) | Not run |
| S5-M02 | Width estimation error | Same runs; measure the obstacle face with a ruler (`true_width_mm`). Only `edges=both` results count | Not run |
| S5-M03 | Direction selection success | Block one side with a box inside the bypass clearance (`blocked_side`); vary left/right between runs | Not run |
| S5-M04 | Line reacquisition | `outcome=REJOINED` over every encounter that chose a side; `aligned=1` shows the car turned along the line first | Not run |
| S5-M06 | Pass-leg side watch | Boxes 10 cm and 30 cm deep on the line. `pass=clear` expected; compare `depth_mm` (a lower bound) with a ruler; the car should turn back just past the back corner and return close to the line | Not run |
| S5-M05 | Time to bypass | `ms=` of the REJOINED lines (detection to line follow); graded on the worst run | Not run |

```
python testing/subsystem5/b5_metrics.py logs/<date>_car_run.log logs/<date>_trials.csv
```

`trials.csv`: `trial,true_width_mm,blocked_side,n` (one row per obstacle
placed). Save the raw console log and the CSV under `logs/` and add a row to
Results with the commit and the printed table.

Rejoin alignment (`line_align_cm`) uses the controller's `approach_mm`
(line sensors to wheel axle), still the 50 mm placeholder: measure it with a
ruler before S5-M04 (`CAR_RUN_TEST_PLAN.md`, junction approach).

## Results

Append actual runs only. Keep failures and link the exact tested commit, profile,
configuration and raw log/video. Upstream example rows are archived separately
under docs/template-reference and are not results for this team.

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S5-H01 host unit tests (range, obstacle, avoidance) | branch buddy5, testing/host CTest, Release, zig cc | Buddy 5 | PASS, 248 checks, 0 failures; all 4 host suites pass | ctest output in branch commit notes | Host only: simulated sonar/servo/motion/line. No target build, flash or hardware run. |
| 2026-10-21 | S5-H02 host unit tests after turn scan + mirrored bypass | branch buddy5, testing/host CTest, zig cc | Buddy 5 | PASS, 383 checks, 0 failures; all 4 host suites pass | ctest output | Host only: simulated world turns with the car heading. No hardware run. |
| 2026-10-28 | S5-H03 controller task core + mission bridge + refused-motion path | branch buddy5, testing/host CTest, zig cc | Buddy 5 | PASS, 7/7 host suites (Buddy 5 core 432 checks, bridge 76 checks, controller service, controller) | ctest output | Host only. Kernel files compiled (not linked) with the team flags for B5_RUN, default SMP=0/1 and B2_BENCH. No flash or hardware run. |
| 2026-10-28 | S5-H04 b5_motion (authority wait, goals, search) + console wait / status line | branch integration-drive, testing/host CTest, zig cc | Buddy 5 | PASS, 9/9 host suites incl. b5_motion_host_tests | ctest output | Host only. B5_RUN and CAR_RUN images build and link. No hardware run. |

## November evidence review

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S5-M01, S5-M02, S5-M04 | Closing-bars, local changes on da4e9d9 | Operator (name unrecorded) | Partial | [Evidence E3](../DEMO_EVIDENCE.md) | Confirmed detection/rejoin handoff and wheel progress. One edge and width unavailable. Full metric series open. |
