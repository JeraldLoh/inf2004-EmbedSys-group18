# Buddy 5 Obstacle scanning and avoidance test record

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
| S5-B08 | Bypass path and return | Follow motion prompts by hand; legs mirrored, no contact, ends facing line direction | Not run |

Bench procedures: `bench_firmware/README.md`.

The code's subsystem5_test() is only a SKIP placeholder and is not invoked by
the inactive app_main. Do not report it as a subsystem PASS.

## Results

Append actual runs only. Keep failures and link the exact tested commit, profile,
configuration and raw log/video. Upstream example rows are archived separately
under docs/template-reference and are not results for this team.

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S5-H01 host unit tests (range, obstacle, avoidance) | branch buddy5, testing/host CTest, Release, zig cc | Buddy 5 | PASS, 248 checks, 0 failures; all 4 host suites pass | ctest output in branch commit notes | Host only: simulated sonar/servo/motion/line. No target build, flash or hardware run. |
| 2026-10-21 | S5-H02 host unit tests after turn scan + mirrored bypass | branch buddy5, testing/host CTest, zig cc | Buddy 5 | PASS, 383 checks, 0 failures; all 4 host suites pass | ctest output | Host only: simulated world turns with the car heading. No hardware run. |
