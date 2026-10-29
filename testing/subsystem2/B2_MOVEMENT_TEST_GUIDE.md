# Buddy 2 movement-test UF2 — 29 October 2026

This is a dedicated **B2-only movement and PID-tuning image**. It tests real
motors, independent encoders, the measured executor, distance/reverse/pivot
goals, STOP and watchdog. It does not start line following, obstacle avoidance,
servo, barcode or IMU tasks. There is no automatic test sequence or boot motion.

This is separate from the circuit-test image. Your previous operator report
confirmed motor power ON and left encoder feedback during manual rotation;
15% pulses had not established powered movement. Therefore this image begins
with short 30% raised-wheel checks before permitting PID/distance commands.

## Flash and controls

1. Switch motor/battery power off and disconnect USB. Secure both wheels clear
   of the surface. Hold BOOTSEL while reconnecting USB and copy the UF2 to
   RPI-RP2. Reopen the USB serial monitor (COM3 if reassigned; 115200 is fine).
2. Confirm `[B2 MOVEMENT TEST]` or repeating `[B2 MOVE]` lines. Wheels must stay
   still at boot. Turn motor supply on with wheels still raised.
3. GP20 short press/release selects the next case. Selection alone does nothing.
4. Hold GP20 >=1s until `RELEASE ... ONCE`, then release to start that case.
   Holding a button at boot or through a test cannot queue another movement.
5. GP21 STOP interrupts movement. Releasing STOP alone never restarts it.
   Keep the physical power disconnect accessible as well.

## Menu and sequence

| Case | Requested test | Where / limit |
|---|---|---|
| 0 | Left positive, right off | RAISED; 30% for 5s |
| 1 | Right positive, left off | RAISED; 30% for 5s |
| 2 | Both positive; exercise GP21 during pulse | RAISED; 30% for 5s |
| 3 | Straight PID target 100mm/s, curvature zero | First RAISED; ends at5s or500mm rotation on either wheel |
| 4 | Forward 300mm | First RAISED for logic, then a clear level floor for accuracy |
| 5 | Forward 1000mm | Clear level floor, only after case4 completed and was measured |
| 6 | Reverse 300mm | Clear level floor, only after case4 |
| 7 | Left pivot 90 degrees | Clear level floor, only after case4 |
| 8 | Right pivot 90 degrees | Clear level floor, only after case4 |
| 9 | Watchdog cutoff exercise | RAISED; both30%, alarm100ms, task withheld150ms |

Case0 must show at least3 left edges and at most2 right edges; case1 the
opposite. This checks powered feedback and independence, **not actual forward
direction**. Visually establish that both positive commands turn the wheels
in the forward-driving direction. If reversed, power off before correcting
motor-terminal order; channel-B encoder signs cannot establish this for you.

Case2 must be interrupted with GP21 to set `STOP=1`. Position one hand over
GP21 before releasing GP20 with the other. An ordinary time-ended pulse does
not set that flag. The serial readiness line shows `L=1 R=1 STOP=1` when these
checks have been exercised. Cases3..9 remain LOCKED until then. If a wheel
does not turn at30%, stop this sequence and diagnose motor supply, terminals
and driver output. The image cannot solve a missing motor-power path by PID.

Case3 first runs with wheels raised. Check live target/measured speeds and
efforts. Oscillation, a stopped wheel, persistent saturation or faults require
reviewing the log and tuning before floor tests. The candidate gains are not
a promise that either wheel will track100mm/s accurately under your load.

For floor tests, use a level clear area and keep people/hands clear of wheels.
Start with case4 (300mm), with GP21 accessible. Measure actual travel and side
drift. Case4's encoder completion sets `SHORT=1`, unlocking cases5..8; it does
not prove actual floor distance or orientation. Do not attempt1m or pivots
until you have inspected the short physical result.

The five-second raw checks refresh a100ms hardware cutoff on each service;
a missed service stops the wheels early and is a failure, not a passed check.
The original bench and circuit pulse limits stay unchanged.

Every action ends stopped and needs a new manual gesture. There is no chained
forward/reverse/turn or automatic retry. Case5 has a15s outer test limit; other
goals10s, and the timed speed test6.5s (normally stops at5s). The executor also
derives finite goal deadlines, detects wheel stall/overshoot and requires both
wheels to reach tolerance and settle. A motion fault requires reset after
diagnosis; a physical STOP can be cleared only by a fresh manual start.

## What the readings mean

Live lines show signed wheel targets, measured speed **magnitudes**, motor
effort in per-mille (300=30%), raw independent counts and the production
qualification flag. Data are logged approximately every100ms during a PID run.

Raw results: `end=2` means time ended, `end=3` means STOP, `end=4` means failure.
Zero edges do not establish motor motion. Manual rotation during a powered
test must not be used to satisfy the feedback check.

Goal `RESULT code`: 3=encoder criterion completed, 4=cancelled, 5=timeout,
6=aborted acquisition, 7=fault. A result's `mm` and `mdeg` are inferred from
wheel rotation and command direction. They are not independent measurements
of floor position, wheel direction or body angle. Ground slip and load need
external measurement; a printed completed result is not a final physical pass.

Case9 deliberately sleeps the owner task for150ms without servicing the raw
trial. The100ms hardware alarm should stop PWM independently. It then checks
cutoff evidence and disabled driver state, explicitly stops again as backup,
and reports `WATCHDOG cutoff_observed=1` or0. If GP21 interrupted that probe,
repeat only after inspecting the log; it is not evidence of timer-only cutoff.
This is a functional software/IRQ observation, **not physical pin timing** and
not a debugger/CPU-halt test. Physical <=50ms STOP still needs instrumentation.

## Calibration and tuning scope

The image retains diameter65mm, left1271/right1264 channel-B both edges per
wheel revolution, and ruler track width112mm. No gearbox multiplier is added.
Effective pivot width remains to be checked against actual body angle.

`b2_move_profile.c` contains **untuned starting gains**. P/I/D gains
x1000 are1500/1500/0 on each wheel; feedforward x1000 is2163 left /2689 right.
The FF starting points come from prior branch notes (effort199 at roughly
92/74mm/s), not a new qualification measurement. Effort is capped30%, launch
assistance at30% for at most500ms, with separate wheel PID and anti-windup.
The config's `qualified` flag stays FALSE throughout the test.

To break the calibration/testing dependency, the measured executor has a
separate calibration-authority function compiled **only in B2_MOVE_TEST**.
It rejects profiles above120mm/s,30% effort,1m maximum goal,100ms watchdog,
20ms control period or20s total authority. It never sets production-qualified
true and never changes the CAR_RUN gains/configuration. Each test's shorter
time limits apply as well. Normal production authorization still rejects
unqualified gains; this image is not an autonomous production-drive approval.

## Buddy 2 final evidence to record

- Image hash, supply/battery, load, wheel/track calibration and gain revision.
- Powered independent wheels and actual direction, raw counts and STOP result.
- Requested/measured wheel speeds and effort under the intended load.
- Five1m runs, externally measured distance error and side drift; compare
  the planned <=5% distance error /50mm drift target.
- Five90-degree pivots in each required direction, externally measured angle;
  compare the planned +/-10-degree target and tune effective track width.
- Physical STOP latency and watchdog behavior, retaining failed runs too.
- Stop, stall, cancellation and timeout results. Do not physically obstruct a
  powered wheel with your hand to manufacture a stall; native fault tests
  already cover that software path, and any hardware fault exercise needs a
  controlled fixture and power limitation.

Other-buddy/controller integration is separately software-tested. This file
isolates B2 and cannot certify a complete CAR_RUN course or network stack.

## Build/software evidence

Profile: `B2_MOVE_TEST=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none
MQTT=0 TEST=0 B2_BENCH=0 B5_RUN=0 CAR_RUN=0 CIRCUIT_TEST=0`. Modes are mutually
exclusive. The core-0 test task uses a6144-byte static stack and nominal5ms
service; the real executor uses20ms updates and100ms watchdog. Motor hardware
is still the production driver/IRQ path. No sensor navigation tasks are started.

111 B2/circuit/movement native CTests pass. New coverage includes bounded test
authority, production refusal unchanged, deadline and raw-clock wrap, STOP
races, simulated calibrated1m/reverse/pivot completion, actual task boot/locks,
missing feedback, STOP/stall, long-test lock and watchdog exercise. These use
fake HAL/peripherals and are not physical performance evidence. No flash or
motor operation was performed while creating this UF2.
