# Buddy 4 Terrain and hump monitoring test record

This is a proposed plan, not test evidence. No hardware test was run during
folder migration. Define exact preconditions, thresholds, procedures and evidence
before execution. Electrical work requires G1; active motion needs explicit
bounded-test approval and a physical power cut.

## Proposed cases

| ID | Purpose | Method and acceptance | Status |
|---|---|---|---|
| S4-U10 | Validate stationary IMU calibration and terrain categories | Car still and level: calibration completes in under 3 s; \|pitch\| and \|roll\| ≤ 2°; terrain LEVEL; no cap. Tilt the car 10° nose up by hand: pitch reads +10° ± 1.5° | Not run |
| S4-U11 | Compare hump estimates against known measured heights | Ruler-measure each test hump. Drive over it 5 times at the hump cap speed. Mean error ≤ 5 mm, every result marked valid, and the true height within the reported uncertainty in ≥ 4 of 5 runs | Not run |
| S4-HOST | Portable logic (math, packing, estimator, seqlock, payloads) | `ctest` in `testing/host`: `terrain_estimator_host_tests` passes with no warnings | Not run on a commit |
| S4-T01..T12 | On-target self-test (`s4_selftest.c`, `make TEST=1`) | Every check prints PASS. Includes T09 (estimator step worst case under 1 ms; record the printed mean and worst µs), T11 (DATA_VALID set) and T12 (Buddy 4 bits of the shared fault word clear); also record the printed sample time | Not run |
| S4-H01 | Hump height during the demo procedure | As S4-U11, logging the console `S4 hump ...` lines | Not run |
| S4-H02 | Collision detection | Push the stopped car into a padded wall at low speed, 5 times from the front and 5 from the side. ≥ 9/10 detected with the correct side, and 0 false collisions over a 2-minute normal drive | Not run |
| S4-H03 | Turn reporting | Commanded 90° pivots, 5 left and 5 right, motors running. Event angle within ±15°. Repeat after magnetometer calibration | Not run |
| S4-H04 | Bus fault and recovery | Unplug SDA while running: I2C fault and INVALID within 100 ms; replug: LEVEL again within 2 s without a reset | Not run |
| S4-H06 | Range log to confirm the bit widths (Week 8 tribunal) | Run the whole course (hump, turns, obstacle bypass), then send command 5. Record the printed maximum turn rate, pitch, roll, hump height and length, shock, vibration and clamp count. Pass if each maximum is below about 80% of its field limit (turn rate below 200 °/s) and clamped is 0. Otherwise widen the field and repeat | Not run |
| S4-H07 | Shared fault word with concurrent writers | Run with Buddy 2/3/5 setting and clearing their own fault bits while Buddy 4 is unplugged and replugged. No bit is ever lost or wrongly cleared (cross-subsystem test "Communications + fault producers") | Not run |
| S4-H05 | Timing | Sample counter (word 1) advances every 20 ms ± 2 ms over 60 s with Wi-Fi active | Not run |

The original subsystem4_test() is still a SKIP placeholder and is not invoked
by the inactive app_main. Do not report it as a subsystem PASS. The real
checks are in s4_imu_selftest() and run once the lifecycle hookup in
app_program/subsystem4/README.md (step 3) is done.

To regenerate the Change Requirement 1 size evidence, compile
`s4_size_report.c` as described in its header comment.

## Results

Append actual runs only. Keep failures and link the exact tested commit, profile,
configuration and raw log/video. Upstream example rows are archived separately
under docs/template-reference and are not results for this team.

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
