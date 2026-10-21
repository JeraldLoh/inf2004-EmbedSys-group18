# Group 18 testing

This follows the course `testing/subsystem1`-`subsystem5`, `integration` and
`system` evidence format. Tests are not passed until actually executed and
recorded with their commit/profile/configuration and saved evidence. Do not turn
an upstream example result or placeholder into a reported measurement.

## Current computer-only checks

`host/test_vehicle_controller.c` contains the portable controller checks,
`host/test_motion_math.c` checks Buddy 2's hardware-independent conversion logic,
and `host/test_perception_math.c` checks the ported barcode/raw-IMU decoding.
These CMake targets compile only portable code; they do not compile the kernel,
build a UF2 or access hardware. Assertions remain enabled in Release builds.
CMake configuration/build is a HOST operation:

```sh
cmake -S testing/host -B testing/host/build
cmake --build testing/host/build
ctest --test-dir testing/host/build --output-on-failure
```

Choose a working native host compiler/generator for your machine. A fresh SDK or
ARM compiler is not needed for these portable tests. Results from the migration
are recorded separately in `host/MIGRATION_CHECK.md` with their exact scope.

`tools/verify_group18_layout.ps1` is a static path/discovery check and does not
invoke Make, a target compiler, a network service or device I/O.

## Future target tests

The course supports `TEST=1`, but this scaffold's target hooks only print SKIP
if eventually called; current usermain starts no subsystem and runs no target
test. Do not flash it or run the archived LED/producer demo on the connected car.

Before any future target run: complete relevant G0/G1 checks, define the test,
review its GPIO/power/motion effects, agree quantitative pass criteria, and
explicitly authorise the build/flash/test step. Record the full Make profile.
Do not assume `WIFI=cyw43` includes MQTT or `TEST=1` is electrically safe.

## Evidence rules

- One buddy's unit/manual evidence goes in `subsystemN/TEST_LOG.md` and `logs/`.
- Cross-buddy interfaces go in `integration/TEST_LOG.md`.
- Whole-mission acceptance goes in `system/TEST_LOG.md`.
- Append a result for every run, including failures; never rewrite the history.
- Store raw logs/photos/scope results with setup, units and firmware version.
- Share videos as evidence links in `videos/README.md`, not committed binaries.
- Host success is not proof of scheduling, sensor quality, braking, physical
  safety or complete controller behavior. Planned tests remain Not run.
