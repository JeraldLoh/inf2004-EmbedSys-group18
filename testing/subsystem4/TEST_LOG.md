# Buddy 4 Terrain and hump monitoring test record

## November demo cases and bench follow-up

Use the [shared setup, tolerances and recording rules](../DEMO_TRACEABILITY.md) and [evidence ledger](../DEMO_EVIDENCE.md). These procedures govern the normal combined image. Earlier plans and results below apply to their recorded profiles.

| ID | Priority, owner and requirement | Setup and steps | Pass criteria and evidence | Status |
|---|---|---|---|---|
| S4-U10 | P0 · B4 · IMU calibration, attitude and terrain | Boot level and still. Record level readiness, valid sequence, 64-sample calibration, nominal 20 ms acquisition and processing time. Use protractor for static pitch/roll and a ramp for ascent/crest/descent/level with valid B2 odometry. | P: valid level within 3 s, static pitch/roll within ±5°, no false hump on flat and ordered phase changes on ramp. Save calibration, estimate timing and instrument truth. For the broader brief, compare turn/motion labels with video and inspect availability of quantitative turn-rate and explicit collision events. Record absent outputs as gaps. Current jolt filter does not report collision, and this IMU has no gyro. | E4 periodic valid telemetry and processing history. Calibration/classification accuracy open. |
| S4-U11 | P0 · B4 · Hump measurement accuracy | Measure two or more distinct fixtures with ruler/caliper; use a flat control and allow level settling between humps. Run three passes per fixture at cruise, including course approach orientation. Record valid event, estimate, uncertainty and path; exclude pickup artifacts by chronology. | P: 3/3 valid per nominal fixture well above the 5 mm event validity floor, height error ≤max(5 mm, 20%), no valid event on flat. Report spread and whether actual truth falls within advertised uncertainty. Save fixture photo and paired results. | E2–E4 valid estimates with no matched truth. |
| S4-L01 | P0 · B4/B2 · Pickup discrimination | On a straight under active motion with spotter and safe grip/fixture, perform a deliberate upright pickup, then test ordinary lifts of agreed heights/rates and flat/ramp controls. Run INT-32 for controller latch. Log actual trajectory, IMU estimate, stop reason and prior wheel drive. | Code uses estimated ≥200 mm within 1.2 s. P: 3/3 deliberate moving pickups cut drive and no false pickup on normal floor/humps. Record lift-onset-to-disable separately, including misses; subthreshold trials characterize limits. | E2–E4 latched event evidence. Instrumented from-motion timing and repeatability open. |
| S4-L02 | P1 · B4/B2 · Overturn path | Use a secured tilt fixture with wheel clearance, first below threshold then over about 60° for ≥300 ms. Log attitude, trigger, wheel output and upright/settle. | Over-threshold sustained tilt yields overturn event and latched stop. Below-threshold control is quiet. P: 3/3 qualified triggers; save angle/time and log. | Not run for this procedure. |
| S4-U12 | P1 · B4 · Invalid sensor/odometry | With safe fixture, run S4 valid/invalid/valid and B2 odometry loss separately on the normal image. Capture sequence, I2C/read validity and terrain/hump outputs. | Invalid input has no current valid result; stale B4 cap cleared in INT-13; record restored validity or explicit unavailability, with no spontaneous restart. | Source/host support. Normal hardware fault matrix open. |

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

## November evidence review

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S4-U11 | Controller-trace, local changes on da4e9d9 | Operator (name unrecorded) | Partial | [Evidence E4](../DEMO_EVIDENCE.md) | Valid 9 mm Â±8 mm estimate observed. Actual fixture height unknown. |
| 2026-11-25 | S4-L01 | Partial-marker through controller-trace, local changes on da4e9d9 | Operator (name unrecorded) | Partial | [Evidence E2–E4](../DEMO_EVIDENCE.md) | Logged pickup and latched stop. Controlled from-motion series and timing open. |
