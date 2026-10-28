# B2-01 software verification and platform contract

28 October 2026. Production-intended driver foundation; **not ready for powered hardware testing**. No board was flashed and no motor/servo/wiring was operated or changed. The normal course entry remains inactive. Target runtime binding and a bounded test entry are integration work still required before a powered trial.

## What the harness tests

The native library compiles actual `motion_hal.c`, `drv_motor.c`, `drv_encoder.c` and `subsystem2.c`. The adapter test compiles actual `motion_hal_rp2040.c` against BSP call mocks. Only kernel header types/return codes are stubbed; the tests do not simulate a working scheduler or prove target ISR safety. Assertions stay enabled even for release test configurations.

| CTest group | Evidence |
|---|---|
| `b2_driver_foundation` | Disabled boot, independent wheel signs/efforts and zero, stale generations/STOP tickets, clear-versus-arm separation, repeated init, retained tokens across deinit/reinit, invalid active reinitialization, generation wrap, STOP/direction sequence exhaustion. Injects each of 16 initialization operations, each of 10 powered-commit operations and each of 14 active STOP operations; also tests persistent HAL failure reporting. |
| `b2_stop_interleavings` | STOP before commit and waiting at every one of 10 HAL commit boundaries; modeled same-core masked IRQ or cross-core next lock acquisition. No stale replay restarts after STOP, even when ordinary work is backlogged. Clear/arm raced before and after acknowledgement; SIO-low-before-slice-disable ordering; repeated STOP. |
| `b2_encoder_snapshots` | Independent rising/falling edges, simultaneous wheel/sonar masks, fresh stationary capture versus edge age, U32 count/time wrap, explicit reset epoch, capture/reset boundary events, command-only direction basis, invalid clock, future/reordered edge timestamps, coalescing indicator and epoch exhaustion. |
| `b2_lifecycle` and 20 failure cases | Real course wrapper startup without binding, disabled successful init/start, every init GPIO/PWM/input/attach/clock failure, rollback and successful retry. No object allocation/task creation is claimed: storage is static. |
| `b2_lifecycle_detach_failure`, `start_clock_failure`, `start_output_failure` | Quiescing/detach failure blocks reinit; start rechecks acquisition/output readiness and shuts down on failure. |
| `b2_rp2040_adapter` | No GPIO access from invalid or successful HAL creation; real adapter maps the reviewed BSP interface, correct wheel/pin routing and error propagation. BSP calls are mocks in this test. |

There are 28 CTest entries; several entries contain loops over many interleavings/failure locations. This count is not a physical qualification score or a count of completed hardware tests.

## Run from the workspace root

```powershell
cmake -S Project/All/testing/subsystem2/b2_01 -B output/planning/2026-10-28-integration-review/buddy2-b2-01/host-build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe -DB2_COVERAGE=ON
cmake --build output/planning/2026-10-28-integration-review/buddy2-b2-01/host-build
ctest --test-dir output/planning/2026-10-28-integration-review/buddy2-b2-01/host-build --output-on-failure
```

Existing regression suite uses `Project/All/testing/host` and a separate `regression-build` directory under the same evidence folder. New tests are isolated within Buddy 2 ownership; shared `testing/host/CMakeLists.txt` was not changed.

## Required platform binding

`motion_hal_rp2040_create()` wraps BSP GPIO/PWM calls and requires a persistent `motion_rp2040_port_t` from the integration lead. Port creation does not touch hardware. The port must supply:

1. One bounded, nonblocking lock covering both cores and every motor commit/STOP/encoder access. Prevent local ISR preemption before taking the cross-core lock. Review interrupt priorities, recursive entry, other locks and maximum hold time; a task must not be preempted while holding a lock that its STOP ISR will spin on. A host lock model or local masking by itself is insufficient.
2. A real U32 microsecond clock. Modulo comparisons require intervals below `2^31` us (about 35.8 minutes). Counter deltas must fit one U32 domain between acquisitions; establish an edge-rate/maximum-gap bound before using them for speed. Do not derive a misleading millisecond domain by dividing a low-32-bit microsecond clock.
3. Shared dispatcher registration for both B edges on GP27/5, preserving sonar/other handlers. Attach failure leaves no subscription; detach quiesces all in-flight callbacks. Dispatcher owns IRQ acknowledgement and exactly-once mask delivery. Latched edge bits can coalesce at excessive rates; timestamped snapshots describe serviced counts, not simultaneous physical acquisition or a guarantee of no lost edges. Qualify latency/filter/count convention on target.

Bind with `subsystem2_bind_hal()` during serialized startup, then call init with all other subsystem object creation, and start only after all required objects/readiness gates pass. Init/start do not arm. Without binding the hooks return `E_NOSPT`, so an unintegrated target cannot silently enable motion.

`subsystem2_urgent_stop()` provides no queue dependency. It must be reached through the reviewed physical/remote/safety adapter; source reason latches, generation advance and controller reconciliation remain integration-lead work. Its status reports whether HAL shutdown calls succeeded, not measured low voltages or physical rest. A failed safe command leaves inhibit/fault asserted and state unconfirmed; it cannot be cleared by a normal START.

The future Buddy 2 executor alone uses `drv_motor_apply()`. `clear_inhibit()` requires the latest observed STOP sequence; `arm()` requires cleared applicable reasons and fresh controller authority. Neither produces nonzero output. Generations advance modularly by less than half range, zero is reserved, and the integration lead rejects prior boot/session requests. Sequence/epoch exhaustion inhibits/requires reboot-level session renewal rather than reusing an old ticket. There is no profile store or mission arbitration in this driver.

Effort is signed -1000..1000 per-mille, with independent wheel channels. It is not `SET_SPEED`, mm/s or tuned PID. Direct nonzero sign reversal stops/inhibits; reversal deadtime must be qualified by later control. PWM wrap 6249 is only the initial 20 kHz candidate at the reviewed 125 MHz/divider-1 setup; confirm clock/slice ownership before activation. No other component may use motor slices 4/5 or run diagnostics concurrently.

## Target compilation and full course build

All five foundation files compile for Cortex-M0+ with GCC 15.2.1, `-std=c11 -Os -Wall -Wextra -Werror`, real course headers, SMP=1, APP_TEST=0 and UART0-free configuration. A header conflict between libc `size_t` and the course kernel definition was fixed by keeping the target adapter's kernel/BSP translation unit free of unnecessary libc headers. No shared kernel header was changed.

A separate source snapshot at `tmp/b2-01-target-20261028` was used to compile/link the full course image with:

```text
make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 PICO_SDK_PATH=C:/Users/rbola/.pico-sdk/sdk/2.3.1
```

This uses the existing make source glob; each foundation object is linked exactly once. Wi-Fi/network are disabled for this build check. It is the inactive normal course application, not a powered test harness. The makefile also emits a UF2; its presence does not authorize or justify flashing it. The production checkout's existing build products were not cleaned or overwritten. No RAM-saving comparison is claimed from the image sizes.

Exact test/build logs, source manifests, coverage, target map/hashes and preservation checks are under `output/planning/2026-10-28-integration-review/buddy2-b2-01/`. Historical review reports and logs remain unchanged.

## First hardware test, after prerequisites

Do not start this procedure from the current build alone. First provide and review the actual STOP/locking/dispatcher binding and a bounded, manually triggered test entry using these same drivers. Confirm G1 wiring/electrical checks and an accessible stop/power cut. Record the exact image hash, effective configuration, calibration version, battery/load and operator. No automatic boot movement or calibration routine is permitted.

| Order | Later user-operated procedure | Pass/evidence |
|---|---|---|
| H1: passive startup | In the approved non-driving measurement setup, observe all four motor input pins through power/reset/init, repeated start and failure; do not begin with a floor run. | All four pins low/SIO and no unsolicited pulse. HAL return codes alone cannot prove this. |
| H2: raised-wheel bounded trial | With car secured and wheels clear, manually request one wheel/direction at a time, then both, at an explicitly capped effort and duration. Verify direction and separate encoder paths. STOP and reset between cases; an opposite powered sign request must not reverse directly. | Correct physical wheel/pins, no cross-coupled encoder count, shutdown stays latched until a new permitted request. This does not qualify loaded speed/distance. |
| H3: STOP timing | Capture confirmed STOP and both motor disabled states, including worst relevant interrupt/logging/queue load. Keep detection/filter time separate. | Both disabled within <=50 ms from confirmed STOP. Record physical coast/stopping distance separately. No numeric physical stop-distance threshold is invented here. |

Loaded floor testing, PID/profile tuning, five 1 m runs within 5% distance error/50 mm drift, five 90-degree pivots within +/-10 degrees and right-wheel-ramp/left-barcode qualification are later gates. Raw encoder motion is wheel rotation, not surveyed ground displacement or measured direction.
