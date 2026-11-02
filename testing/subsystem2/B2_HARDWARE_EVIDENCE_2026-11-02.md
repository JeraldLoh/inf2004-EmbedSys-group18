# Buddy 2 operator hardware evidence — 2 November 2026

## Tested image and settings

Standalone image:group18_b2_movement_test_v6_2026-11-02.uf2.
SHA256:ccb6aedbe8539d1391e5ba180c920fec837e293776cd89326f36885f61f28bb5.
Exact source snapshot/ELF/map/logs are in the local delivery folder
output/firmware/b2-movement-test-v6-2026-11-02. Generated firmware is ignored by
Git; the source and build recipe are versioned.

Wheel diameter65mm; left1271/right1264 B-channel both-edge counts per wheel
revolution; ruler track112mm. P/I/D milli1500/1500/0 both wheels, FF2163/2689,
30% effort cap, launch30%<=500ms, measurement100ms/loop20ms/watchdog100ms,
linear and pivot rolling floors60mm/s capped by request,5mm wheel tolerance,
10mm overshoot,700ms no-progress,1500ms startup grace,200000um/s2 acceleration.
Test-only batching<=2 observations/100ms AND<=5% observed edges per wheel.
These values remain test candidates; production qualification is false.

## Evidence received from the operator

- Independent powered left/right raw trials completed with thousands of edges
  on the intended channel and zero on the other channel.
- Physical GP21 interrupted the both-wheel trial; end3/reason0x1 was logged.
- Timed straight command reached its normal time/rotation limit.
- V5 trace:forward300mm completed at296mm encoder estimate;1m completed at996mm;
  reverse300mm completed at-296mm, all code3/odom_valid1. Operator reports actual
  30cm forward/backward travel was correct, and later repeated travel correctly.
- V5 pivot faulted short at67/74mm wheel travel, target87mm (estimated72.6deg,
  visible75-80deg). V6 introduced a separate pivot rolling floor.
- V6 operator reports both left and right pivots reached approximately90deg,
  then reports repeating travel and pivot trials correctly a few times. Exact
  repeated angles/distances, load/supply, numeric run count and v6 pivot terminal
  RESULT lines were not supplied. Do not invent these as measured records.
- V6 watchdog log:cutoff_observed1 after task withheld150ms; RAW end2 at156ms,
  driver subsequently zero. This is timer/driver functional evidence, not a
  measurement of physical motor-pin cutoff latency.156ms is observation time.

No flashing or car operation was performed by the coding agent. These are
operator reports and supplied serial traces, not an independently instrumented
hardware qualification campaign.

## Integration status and limits

The next image is an explicit bounded CAR_B2_TRIAL profile using these same
settings through the measured executor, preserving production qualifiedfalse.
Line/junction/obstacle behavior, combined task load, network operation, physical
STOP<=50ms, floor drift/error statistics, voltage/load range and repeatability
of integrated maneuvers still require testing/review. Normal production START
must not be marked qualified merely because standalone movements look correct.

Testing commit641ad74 reverted earlier measured integration solely to restore
START with the draft executor. Publishing the hardware-tested corrections
intentionally supersedes that revert and restores the measured path. Buddy1's
telemetry/command integration froma112ed4 is preserved. Main is not a target.
