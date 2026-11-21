# Buddy 4 Terrain and hump monitoring test record

This is a proposed plan, not test evidence. No hardware test was run during
folder migration. Define exact preconditions, thresholds, procedures and evidence
before execution. Electrical work requires G1; active motion needs explicit
bounded-test approval and a physical power cut.

## Proposed cases

| ID | Purpose | Method and acceptance | Status |
|---|---|---|---|
| S4-U10 | Validate stationary IMU calibration and terrain categories | Procedure and quantitative acceptance to be agreed | Not run |
| S4-U11 | Compare hump estimates against known measured heights | Procedure and quantitative acceptance to be agreed | Not run |
| S4-L01 | End of run: lifting the car stops the motors (requirement change, 2026-11-11) | Car running the line (MQTT build). Lift it 25-30 cm in one movement (under 1 s): upright, tilted about 45 deg, and turned over. Pass: `[B4] END OF RUN` and `[CAR] END OF RUN ... motors stopped` within 1 s of the lift, wheels stop, the dashboard shows "End of run". A 10-15 cm lift and normal driving over the ramp must not trigger. Put it down upright for 1 s, then START must run again. Record the reported lift height against a ruler. | Not run |
| S4-L02 | End of run: overturned | Turn the car past 60 deg (on its side or upside down) and hold 0.3 s. Pass: `[B4] END OF RUN: car overturned` and the motors stop. | Not run |

The code's subsystem4_test() is only a SKIP placeholder and is not invoked by
the inactive app_main. Do not report it as a subsystem PASS.

## Results

Append actual runs only. Keep failures and link the exact tested commit, profile,
configuration and raw log/video. Upstream example rows are archived separately
under docs/template-reference and are not results for this team.

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
