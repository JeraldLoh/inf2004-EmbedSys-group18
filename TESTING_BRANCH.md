# `testing` branch: first whole-car build

**Base:** `main` at `ceab57b` (2026-10-28).
**Author:** Buddy 5, as integration lead.
**Status:**
- Builds and passes the PC tests.
- **Not yet run on the car.**
- **Not merged into `main`.** It needs the reviews listed below first.

This branch adds a whole-car build, `CAR_RUN=1`. In that build:
- Buddy 3's sensors steer the car along the line.
- Buddy 2's motors drive it, through a new **draft** executor.
- Buddy 5 stops the car for obstacles and drives around them.
- The shared controller decides who may move the car at any moment.

Every existing build (default, `B2_BENCH`, `S3_BENCH`, `B5_RUN`) still works as before.

## Buddy 2 measured executor addition (2026-10-28)

The branch also contains Buddy 2's independently tested wheel PID and typed
START/profile/goal interface: `subsystem2/measured_motion_executor.*`,
`motion_task.*`, and `vehicle_controller/controller_motion.*`. See
[B2_EXECUTOR.md](testing/subsystem2/B2_EXECUTOR.md) for setup, units and limits.

The existing `CAR_RUN` path below still uses the integration draft
`motion_executor.*` and `subsystem2_motion.*`. It has not been silently switched
to the measured executor. The new adapter refuses binding in `CAR_RUN` and raw
bench builds, and the controller refuses mixing its typed adapter with legacy
line-drive hooks, so the two executors cannot compete for PWM through these
startup APIs. Migrating the whole-car producers and qualifying gains remain
separate integration work. No flash or physical qualification was performed.

Merged software verification: 55 B2 tests and 9 shared host suites pass. This
includes the existing draft executor, line-tracking and Buddy 5 motion tests.

## 1. The code base at a glance

| Folder | Owner | What is in it now |
|---|---|---|
| `app_program/subsystem1/` | Buddy 1 | Scaffold only, no runtime code |
| `app_program/subsystem2/` | Buddy 2 | Motor/encoder drivers, GP21 STOP, hardware cutoff, wheel bench, and the **new draft motion executor** |
| `app_program/subsystem3/` | Buddy 3 | IR line follow, Code 39 barcode, junction detection, sensor and line-follow benches |
| `app_program/subsystem4/` | Buddy 4 | LSM303 decode helpers; the task is still a placeholder |
| `app_program/subsystem5/` | Buddy 5 | Sonar, scan servo, obstacle avoidance and recovery, and the link to the controller |
| `app_program/vehicle_controller/` | Shared | Controller rules, controller task, and the **new** line tracking and whole-car startup |
| `app_program/contracts/`, `project_config/` | Shared | Message types, fault bits, pin map, GPIO interrupt dispatcher |
| `testing/host/` | All | PC unit tests (CTest), now 9 suites |
| `build_make/` | Shared | `make` build; one setting chooses the image |

### Images (`make ... <setting>`)

| Setting | What it does |
|---|---|
| (none) | Starts nothing; prints the scaffold message |
| `B2_BENCH=1` | Buddy 2 wheel bench: 15% for 250 ms, started by hand |
| `S3_BENCH=1` / `2` | Buddy 3: sensors only / line follow on the bench drivers |
| `B5_RUN=1` | Buddy 5 obstacle task plus the controller; motors stay off |
| `CAR_RUN=1` | **New:** whole car |

## 2. What changed since `main` (6 commits, 32 files)

| Commit | Change | Files |
|---|---|---|
| `99dc319` | Buddy 5 waits up to 5 s for the serial monitor, then prints a status line every second | `subsystem5/` |
| `e6f6147` | **Draft Buddy 2 motion executor** (see 2.1) | `subsystem2/motion_executor.*`, `subsystem2_motion.*`, 2 accessors in `subsystem2.c`/`subsystem2_internal.h` |
| `9a68d5c` | Controller line tracking, plus the check that decides who may move the car | `vehicle_controller/controller_service.*`, `controller_task.*` |
| `f2a376e` | Buddy 5 moves become Buddy 2 goals; line detection and line search come from Buddy 3 | `subsystem5/b5_motion.*`, `subsystem5.*` |
| `162f5c9` | `CAR_RUN=1` startup, buttons, run limit, 1 ms tick in this build only; decisions D11–D14; test plan | `vehicle_controller/car_run_target.*`, `app.h`, `app_main.c`, `pico_rp2040.mk`, `docs/`, `testing/integration/` |
| `5dd0c39` | Makefile: changing `S3_BENCH` now forces a full rebuild | `pico_rp2040.mk` |
| (this commit) | Measured track width (112 mm) and this file | `motion_executor.h`, `controller_service.c`, docs |

### 2.1 How the whole car works

```
Buddy 3 steering --> controller task (every 10 ms) --> move command (lasts 150 ms) --> Buddy 2 executor --> motors
Buddy 5 sonar --> obstacle event --> controller: obstacle mission
Buddy 5 avoidance --> "may I move?" (controller) --> Buddy 2 goal (turn N deg / drive N mm) --> result back to Buddy 5
GP20 hold + release = start    GP21 = STOP (Buddy 2 interrupt, at once)    60 s run limit
```

- **Only the executor writes to the motors.** This matches the TEAM.md rule.
- **Move commands run out by themselves.** If the controller stops sending them, the car stops.
- **A stale command can't restart the car.** Each command carries an authority number from the controller. After a STOP or a driver error, the car only moves again once a fresh START gives a new number.
- **A frozen executor stops the motors anyway.** While the wheels are driven, Buddy 2's hardware cutoff alarm stays armed.

### 2.2 Numbers to check on the car

| Value | Now | Where |
|---|---|---|
| Track width | **112 mm**, measured with a ruler | `MOTION_TRACK_WIDTH_MM` (`subsystem2/motion_executor.h`) |
| Power per 100 mm/s | 133 (150 mm/s gives 20% power), a guess | `motion_executor_default_config()` |
| Lowest power for goals / power cap | 15% / 35% | same |
| Line cruise / barcode speed | 150 / 100 mm/s | `controller_line_default_config()` |
| Avoidance move speed / timeout | 100 mm/s / 4 s | `b5_motion_default_config()` |

## 3. What each buddy needs to do

**Buddy 2 (motors).** This is the most urgent item.
- Review the draft executor (`e6f6147`, decision D13). Keep it, change it or replace it; you own it from now on.
- Before floor runs, check:
  - the arming order and the cutoff use;
  - the open-loop power numbers. There is no speed control (PID) yet, so the speed changes with battery level.
- With the wheels raised, measure power against speed: the `[CAR]` status line prints both wheel speeds in mm/s.

**Buddy 3 (line and barcode).**
- Review the 1 ms kernel tick, which is used in the whole-car build only (D11), and the makefile fix (`5dd0c39`).
- Check the steering scale: steer 100 should give an inner wheel of about 0, the same as your bench.
- Junction routes are not driven yet (D14). We need to agree how the controller runs your junction probe and the barcode turns. Until then, test on a loop with no junctions.

**Buddy 4 (IMU).**
- `testing/host/test_lsm303_decode.c` is still missing on `main`, so the per-subsystem CMake test build fails without it.
- Turns are measured by the encoders only. A compass heading would make turn angles more accurate when you are ready.

**Buddy 1 (communications).**
- Nothing blocks driving.
- The status data for telemetry is already available: the `[CAR]` and `[B5]` lines, plus `controller_get_status()` and `subsystem2_motion_telemetry()`.

**Everyone (shared files).**
- Review the controller changes (`9a68d5c`) and the whole-car startup (`162f5c9`).
- TEAM.md requires the affected owner's review before this goes into `main`.

## 4. What can be run now

| Image | Ready? | Notes |
|---|---|---|
| default, `B2_BENCH`, `S3_BENCH=1/2`, `B5_RUN` | Yes | Same as `main`; `S3_BENCH=2` is now really the line-follow image |
| `CAR_RUN`, **wheels raised** | Yes | Start/STOP, steering response, obstacle stop/scan/turn, run limit: INT-25 and INT-26 |
| `CAR_RUN`, **floor** | After Buddy 2's review | INT-27: a slow loop with no junctions, someone on STOP |

Build, flash and every test step with what should happen: [`testing/integration/CAR_RUN_TEST_PLAN.md`](testing/integration/CAR_RUN_TEST_PLAN.md).
Record results in `testing/integration/TEST_LOG.md`.

## 5. Checks done on this branch

- **PC tests:** 9/9 suites pass, including the new executor, controller line-tracking and Buddy 5 move tests.
- **Bug-catching check:** the safety lines were broken on purpose one at a time, and the tests catch each one.
- **Buddy 2's tests:** 38/38 pass. The layout check passes.
- **Firmware:** all six images compile and link with 0 warnings in app code.
- **On the car:** nothing has been run yet.
