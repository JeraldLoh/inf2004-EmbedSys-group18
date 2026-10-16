# Group 18 Integrated Vehicle Framework

This directory contains the architecture framework for the integrated vehicle.
It deliberately does not contain board startup code, a Pico firmware build, or
hardware-enabling code yet.

The current framework provides:

- stable shared contracts with explicit units, timestamps, validity, and IDs;
- a testable Vehicle Controller state-machine skeleton;
- independent stop-reason latching and authority-generation cancellation;
- the barcode command lifecycle used at confirmed junctions;
- proposed module and documentation boundaries;
- host tests that do not operate vehicle hardware.

The buddy-owned subsystem folders are named `buddy1_comms`, `buddy2_motion`,
`buddy3_perception`, `buddy4_terrain`, and `buddy5_obstacle` so team members can
find their primary areas quickly. Shared code remains in `app`, `contracts`,
`config`, `platform`, `diagnostics`, and `tests`.

## G0 limitation

The professor's exact micro T-Kernel template has not been verified. Therefore,
the framework does not invent `main()`, task creation, interrupt-safe calls,
network integration, or a Pico firmware `CMakeLists.txt`. Those integration
files must be added only after the real template, startup hook, kernel APIs, and
CYW43/lwIP backend are confirmed.

## Electrical limitation

Signals entering the Pico are required to be 3.3 V-domain safe, but that has
not been physically verified. G1 remains open. Do not use this framework as
evidence that the HC-SR04 echo, GP28 jumper, IR outputs, common ground, or power
arrangement are safe.

## Review workflow

Buddy 5 leads integration review. All five buddies may contribute to the
separate Vehicle Controller. Changes to shared contracts, pin allocation,
mode/stop policy, or controller transitions require review by Buddy 5 and every
affected producer or consumer.

See `docs/architecture/architecture.md` and `docs/verification/gates.md` before
extending the framework.
