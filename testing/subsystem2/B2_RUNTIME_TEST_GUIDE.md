# Buddy 2 runtime bench — software-built, hardware not yet tested

For a clean clone, start with [publication/build instructions](PUBLISH_VERIFICATION.md).
The workspace artifact paths below describe the original development session;
generated firmware is ignored by Git and should be rebuilt from this source.

28 October 2026. This is the new B2 driver test image, separate from the integration lead's older component tester. It uses the actual B2-01 drivers and course lifecycle. No flashing, motor/servo operation or wiring change was performed while preparing it.

## Image and limits

Workspace artifact: output/firmware/buddy2-runtime-2026-10-28/group18_b2_driver_bench_smp1_usb_cdc.uf2. Exact ELF/UF2/map hashes and build profile are in manifest.json beside it. Do not confuse it with the inactive normal-build UF2 or the older multi-sensor component tester.

Profile: SMP=1, USB CDC, CRASH_REPORT=0, WIFI=none, NET=none, MQTT=0, TEST=0, **B2_BENCH=1**. A dedicated task is pinned to core 0 (TP_PRC1), with a static aligned 4096-byte stack. It starts disabled; no boot movement or calibration. Normal B2_BENCH=0 startup remains inactive.

This image tests motors GP8/9 and GP10/11, encoder B left GP27/right GP5, GP20 manual input and GP21 physical STOP. It does not enable sonar, servo, IMU, line/barcode navigation, Wi-Fi/dashboard, PID/speed profiles or autonomous goals. The dispatcher supports another client such as sonar; that sharing is host-tested, not a live sonar test in this image. Encoder A remains insulated and GP26 remains reserved for barcode.

The fixed menu uses **15% actuator effort for 250 ms**. This is not 15% speed, 30 mm/s or calibrated PID. Software rejects effort above 15% or duration above 500 ms; each trial has one hardware alarm cutoff and a task-side backup deadline. No automatic retries or queued motions exist. A time-ended result is not evidence that either wheel moved, travelled a distance or met accuracy targets.

## Before a later hardware session

The current authorization was software implementation/build only; these are instructions for a later user-operated session. Keep all wheels secured clear of the surface and GP21 plus an appropriate physical power cut accessible. Confirm the current G1 wiring/voltage/power arrangement; USB can also supply motor power on the Robo board, so do not assume a battery switch removes every motor supply. Do not change wiring based on this document.

Record operator/date, image SHA-256, board/pin map, battery/supply/load and current calibration version. Start with passive disabled-state measurements. Do not begin with a floor run. Do not halt the CPU/debugger during a powered trial: the cutoff is interrupt-driven and does not protect against a halted CPU, disabled interrupts, a failed timer clock or a failed motor driver.

## Expected console and manual controls

After a later authorized flash/reset, open the board's USB CDC serial console. Its banner begins **[B2 BENCH] production drivers, raised wheels only. NO boot motion.** No motion occurs until a fresh hold/release gesture. If INIT FAILED or task creation failed appears, do not request a trial; record the log and inspect the failure. Initialization checks clock progress and refuses existing GPIO IRQ enables on either core or alarm0 ownership.

| Case | Requested outputs |
|---|---|
| 0 (initial selection) | Left positive; right zero |
| 1 | Left negative; right zero |
| 2 | Right positive; left zero |
| 3 | Right negative; left zero |
| 4 | Both positive |
| 5 | Both negative |

Positive/negative are driver command signs; the first raised-wheel checks establish their actual physical directions. Single-channel encoders do not independently measure direction.

- A GP20 press held for 30 ms to less than 1 s, followed by a stable release, cycles to the next case. Read the printed selected case.
- To run the selected case once, press GP20 freshly, hold until the confirmation appears (at least 1 s), then release. Release must remain stable for 30 ms. Confirmation expires after 5 s. Holding GP20 during boot cannot start a trial.
- GP21 asserts STOP on the first falling edge, without waiting for debounce or a command FIFO. Holding STOP blocks all trial admission and cancels a confirmed gesture. Release alone does not resume; another fresh GP20 hold/release is required. Unrelated controller/failure reasons are never cleared by that gesture.
- After each trial, outputs stop and a result is printed. A result is retained until the task consumes it; no following trial overwrites an unconsumed result. There is no automatic continuation to the next case.

## User-operated order

1. **Passive startup/reset:** verify both motor input pairs stay low/SIO with no unsolicited pulse. Observe repeated reset, STOP held at boot and GP20 held at boot. A software message alone does not establish electrical state.
2. **Left only, both signs:** run cases 0 then 1, one brief trial at a time. Record physical wheel direction and left/right raw count changes. Right should remain uncommanded; external rotation/coast may still cause counts.
3. **Right only, both signs:** cases 2 and 3. Record the same evidence independently.
4. **Both wheels:** cases 4 and 5 only after isolated checks pass. Press GP21 during a trial and verify both outputs remain disabled. Test fresh-start requirements; releasing STOP and holding a previous start must not resume.
5. **Timing instrumentation:** measure confirmed STOP to both physical motor inputs reaching the disabled state, targeting <=50 ms. Measure input detection latency, software handler timing, PWM/pin behaviour and physical rest separately. Retain failed runs and raw traces.

At 15% and 250 ms a wheel may not start, especially under load. Zero pulses are not a unique diagnosis of an encoder failure. Do not increase limits, repeat on the floor or declare PID/distance performance from a no-motion or raised-wheel result; save the observations first.

The console prints raw counts, reset epoch, trial ID/generation and end state: 2=time ended, 3=STOP, 4=failure. Counts are unsigned serviced edges, not signed travel. It also prints the **last ISR shutdown-call** elapsed time and maximum lock hold/wait. These software timestamps are not a measurement of physical output voltage, detection latency or a <=50 ms pass. They may refer to an earlier ISR if the task backup ended this trial first. Hardware measurements remain mandatory.

## Software evidence and reproducible build

38/38 B2 native tests pass, including real port code with MMIO/IRQ/BSP/kernel-lock mocks; 4/4 existing host regressions pass. New target sources compile against real course headers with ARM GCC 15.2.1 and warnings-as-errors; the dedicated bench and normal profiles compile/link. Mocks verify ordering and error handling, not Cortex-M0+ execution timing, simultaneous two-core stress, electrical shutdown or physical accuracy.

Build from Project/All/build_make in an isolated source copy:

```text
make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 B2_BENCH=1 PICO_SDK_PATH=C:/Users/rbola/.pico-sdk/sdk/2.3.1
```

The build option has a distinct _b2bench filename/profile and rejects TEST=1, UART console/mirroring, network/radio or SMP=0. It does not run or flash the image. Evidence, manifests, source hashes and preservation results live in output/planning/2026-10-28-integration-review/buddy2-runtime/.

## Shared runtime contract

The only GPIO owner is project_config/gpio_dispatcher: four non-overlapping client slots, core-0 task registration/detach, acknowledgement of registered edges only and urgent callbacks before ordinary clients. Callbacks run outside the registry lock. Core-0-only dispatch and task registration provide detach quiescence; another-core or ISR registration is rejected.

The common motion lock uses the existing SMP kernel ISpinLock/ISpinUnlock. It masks local interrupts before lock acquisition, excludes both cores and restores the prior interrupt state. All lock-held callbacks are bounded register/state operations; no console, task service or blocking kernel object API is called while held. GPIO/Timer HLL handlers only capture/latch/disable; controller changes occur in the bench task. The kernel's HLL wrapper updates task-independent nesting without taking the big kernel lock.

Alarm0/IRQ0 and IO_BANK0/IRQ13 are reserved by this bench. The raw U32 microsecond clock is not reconfigured. A missed alarm deadline during setup rejects admission before PWM. Normal service has a backup elapsed-time cutoff; the timer ISR disables PWM even if the bench task is delayed. Worst-case target lock wait/interrupt masking and physical STOP timing are still unmeasured; a stalled core holding the lock or hardware fault cannot be dismissed by passing host tests.

This is a bench/controller adapter, not full mission integration. Profiles, manual remote leases, route/barcode flow, terrain/ramp qualification, obstacle avoidance and mission completion remain with the integration lead and later B2 tasks.
