# Standalone buddy-code port inventory

This inventory is the boundary between the earlier standalone Pico SDK bench
programs and the integrated micro T-Kernel application. It prevents working
bench experiments from being lost while also preventing unsafe copy-and-paste
into the dual-core target.

Source folders inspected on 2026-10-21:

- `Project/buddy2/`
- `Project/buddy2_bench/`
- `Project/buddy3/`
- `Project/buddy4/`
- `Project/group18-main/`

The sibling folders remain unchanged. They are reference/bench work, not the
submitted integrated application. New integrated work belongs in this repository.

## Port decisions

| Source | Decision | Integrated destination / reason |
|---|---|---|
| Buddy 2 wheel/count geometry | Ported | `app_program/subsystem2/motion_math.c`; pure integer conversions with explicit calibration and millidegrees |
| Buddy 2 motor PWM | Rework required | Future `subsystem2/drv_motor.c`; standalone Pico SDK PWM ownership/build assumptions do not match this port |
| Buddy 2 encoder IRQ | Rework required | Future `subsystem2/drv_encoder.c`; current volatile counters and local interrupt assumptions are not a complete SMP synchronization policy |
| Buddy 2 blocking motion loop | Rejected for runtime | Replace with a periodic motion task and finite-goal state machine; it must never block controller STOP handling |
| Buddy 2 bench `main()` programs | Bench only | Do not copy into `app_program`; the course entry point is `usermain()` |
| Buddy 3 barcode decoder | Ported, hardware-independent core | `app_program/subsystem3/barcode_decoder.c`; maps only complete `*A*`-`*D*` symbols to shared route commands |
| Buddy 3 three-channel ADC line code | Reconcile first | It claims GP26-GP28, conflicting with the current plan: digital line sensors on GP7/GP0, barcode on GP26 only, and GP27 as Buddy 2's left encoder |
| Buddy 3 motor shim | Rejected | Buddy 2 is the sole motor-output owner; Buddy 3 publishes steering evidence/requests |
| Buddy 3 bench `main()` | Bench only | Contains standalone startup and direct motor control |
| Buddy 4 LSM303 raw sample decoding | Ported | `app_program/subsystem4/lsm303_decode.c`; pure signed 12-bit conversion with host tests |
| Buddy 4 LSM303 I2C transaction code | Rework required | Future `subsystem4/drv_lsm303.c`; use one I2C-owning task and a reviewed course-port adapter |
| Buddy 4 floating-point classification demo | Reference only | Thresholds and timing are uncalibrated; publish `terrain_report_t` after measurement |
| Buddy 4 bench `main()` | Bench only | Standalone startup, infinite demo loop and blocking delays are not integrated lifecycle code |
| Old `group18-main` controller/contracts | Ported | Now under `app_program/vehicle_controller` and `app_program/contracts` |

## Required order for remaining ports

1. Reconcile the physical pin map and complete G1 electrical checks.
2. Record measured wheel, encoder, sensor and timing calibration values.
3. Allocate each peripheral to exactly one owning task in `TEAM.md`.
4. Select task periods, priorities, stacks, core affinities and kernel objects.
5. Port drivers without standalone `main()`, `stdio_init_all()` or blocking
   movement routines.
6. Connect drivers to the shared contracts and Vehicle Controller.
7. Define tests before enabling hardware, then record every result and log.

Nothing in this inventory is evidence that a target driver or the vehicle has
passed. The new motion-math host test checks arithmetic only.
