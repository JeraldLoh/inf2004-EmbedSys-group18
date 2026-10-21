# Buddy 4 - Terrain and Hump Monitoring

Primary responsibility: LSM303 transactions, terrain classification,
speed-cap requests, hump detection, and height estimates with validity and
uncertainty.

Other buddies may contribute, but Buddy 4 remains accountable for this
subsystem's contracts, calibration, and evidence.

The course folder is `subsystem4`. `lsm303_decode.c/.h` ports only the reusable
signed raw-sample conversion; it performs no I2C transaction or classification.
Future `drv_lsm303.c/.h` files go directly here. GP16/GP17 correspond to I2C0 in
this pin assignment, despite the Week 6
table's I2C1 wording. No sensor or kernel I2C driver is enabled by this migration.

Lifecycle hooks return `E_NOSPT`; target tests are SKIP placeholders. A tilt
category does not by itself establish hump height. Validate height and speed-cap
transitions against measured terrain and retain estimate validity. Evidence
belongs in `../../testing/subsystem4/`.
