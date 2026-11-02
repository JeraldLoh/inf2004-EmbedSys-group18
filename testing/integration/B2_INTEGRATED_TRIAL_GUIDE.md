# Buddy 2 integrated trial — 2 November 2026

## Source and scope

Fresh clone:tmp/b2-testing-integration-20261102, local branchtesting. Started
from origin/testing641ad74, then fast-forwarded to published Buddy-2be5b250,
which intentionally restores the v6 measured path over that single revert.
Buddy1's earlier code is preserved. New integrated-trial changes are local;
remote testing and main have not been pushed by this task.

This first offline integration runs Buddy2 measured motor control with Buddy3
line/junction/barcode and Buddy5 sonar/obstacle/servo via the shared controller.
It does not run Buddy1 Wi-Fi/MQTT or start Buddy4 IMU/terrain tasks. Network and
IMU integration remain separate checks. This is not a full-course approval.

The image reuses b2_move_candidate_profile(), the exact v6 parameter builder:
65mm wheel diameter,1271/1264 counts per wheel rev,112mm track, same PID/FF,
linear/pivot60mm/s floors,5mm wheel tolerance,10mm overshoot,700ms progress,
30% effort cap,100ms watchdog and test batching limits. Production-qualified
remains FALSE. Explicit CAR_B2_TRIAL permits bounded trial authority instead;
normal CAR_RUN still refuses START with unqualified gains.

V4 line candidate: cruise50mm/s (5cm/s), steering starts12, grows by1
every6ms and caps90 after468ms; curvature per steering unit120000 micro-1/m.
When both sensors return white, the previous correction fades to zero over200ms.
Same-side black during that interval resumes the remaining correction and ramps;
opposite-side black immediately starts correction in the newly observed direction.
No fixed circle radius input is required; this adapts to the duration/side of
digital sensor evidence, within the bounded steering authority available.
Initial correction requests approximately47/53mm/s; sustained full correction
requests approximately19/81mm/s after adapter rounding (nominal91mm radius,
versus125mm for v3 and333mm for v2). Neither wheel is commanded to reverse during line tracking;
50mm/s is the chassis cruise target, not a per-wheel cap during turning.
Barcode/approach slow speed is also capped at50mm/s, so it cannot speed up.
The slower wheel is below the previously used60mm/s rolling candidate: floor stalling
and curve tracking must be checked physically. Tight curves may need retuning.
Wheel PID/calibration and finite distance/pivot settings are unchanged.

Maximum wheel target120mm/s, maximum goal1m. Each
manual run is limited to15s by the supervisor; each authority also expires
within15s independently in the measured executor. Commands keep their short
leases, faults latch, GP21 stops the motors and release alone never restarts.
No startup motor command or automatic retry. Servo/sensor tasks initialize.

## First test: wheels raised

1. Switch motor power OFF. Secure both wheels clear of the surface and leave
   the scan servo free to move. Flash the integrated UF2 using BOOTSEL.
2. Open the Pico's current COM port. Confirm `[CAR B2 TRIAL]` and no startup
   motor motion. This image has NO0..9 menu; GP20 tap does not select tests.
3. At the barcode calibration prompt, sweep the barcode sensor over black and
   white for the indicated3s. Check the subsequent CAR/S3/B5 status output.
4. Turn motor power on with wheels raised. Hold GP20>=1s, wait for RELEASE,
   then release. This requests the shared controller's AUTO mode.
5. Present a black/white sample beneath the line sensors without touching the
   wheels. Check that targets/encoder speeds respond and motor efforts stay
   bounded. If no line is detected, line-lost stop after3s is expected.
6. Press GP21 during wheel movement. Motors should stop promptly and remain
   stopped when GP21 is released. Retain the log. Do not proceed if this fails.
7. With sensors seeing a line, make a fresh START and allow the15s limit to
   stop the trial. Confirm `STOPPED: run time limit`. A fault needs diagnosis
   and reset; a released physical STOP can be cleared by a fresh gesture.

## First floor test

Use a clear, level short straight black line with no junction/barcode/obstacle
initially. Hold/release GP20 to START, keep GP21 accessible, and observe a short
run. It should follow the line using independent wheel PID, then stop on GP21,
line loss, a controller stop or the15s limit. Record drift, sensor states,
requested/measured speeds, per-wheel efforts and all stop/fault messages.

Next, test a gentle curve. Only after line following and STOP look right,
exercise one obstacle with room for the servo scan and avoidance maneuver.
Avoiding an obstacle and turning at a junction have not been validated on the
combined firmware; do not infer them from standalone90deg pivots. Junction
approach geometry is still a placeholder and routes/probes need team review.

Send the log from boot through the first stop. Report line following, drift,
STOP response and whether the servo/sonar readings looked plausible. Do not
claim physical STOP latency, exact floor position or complete integration from
software timestamps. Measured physical STOP<=50ms is still outstanding.

## Software verification (V4 history; not rerun for V5)

157/157 native tests and19/19 shared suites pass. The added real line/controller/
motion-adapter test checks noise filtering, gradual correction, both turn signs,
correction limits, bounded fading curve memory, slow-mode speed and STOP suppression.
V4 tests exercise intermediate/max corrections in both directions, memory decay/
expiry without renewal on white, same-side resume, opposite-side override and reset.
Includes real profile values,
normal unqualified refusal, explicit trial bounds, line requests, source/ID
junction result mapping, deadline, STOP, no boot START, supervisor15s cutoff
and fault-latched START refusal. Existing B2 encoder/PID/goal tests and incoming
Buddy1 portable tests remain. Full integrated/offline-normal/standalone builds
and normal-profile symbol exclusion are checked in the delivery logs.

Build from build_make:

```sh
make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 B2_BENCH=0 B5_RUN=0 CAR_RUN=1 CAR_B2_TRIAL=1 CIRCUIT_TEST=0 B2_MOVE_TEST=0
```

Trial mode requires offline USB, no crash UART, no other test image. Normal
CAR_RUN without CAR_B2_TRIAL keeps production qualification gating. Nothing
was flashed or physically operated while preparing this image.

## Latest operator evidence and limits

The supplied run showed nonzero feedback on both wheels and repeated steering
sign changes while the car followed the tape but swerved. It ended with
MOTION_FAULT (0x8), drv_err=1 near a junction observation; the current log does
not establish which executor condition caused that fault. The softer line
candidate addresses steering behavior, not a demonstrated fix for that fault.
Barcode calibration was unsuccessful, so routes still use the probe fallback.
Buddy5 detection remains25cm confirmed twice, with6cm emergency guard during
avoidance/recovery. Its scanning/goal speeds are not reduced by line cruise.

The subsequent v2 hardware log showed both encoder speeds, drv_err=0 throughout,
and a final physical STOP (0x1). Steering stayed at the25 cap for several status
samples; targets were58/41mm/s, then50/50 when steering returned0. The operator
reported that it did not follow the curve. This supports increasing sustained
turn authority while retaining the50mm/s cruise; v3 has no floor result yet.

## Line diagnostics

Each periodic CAR status includes `line_black L=... R=... state=... last_side=...`.
These are Buddy3's filtered black/white inputs (1=black), not raw GPIO voltages.
State0=centred/both-white interpretation,1=steer-left,2=steer-right,3=junction,
4=reacquire. Last-side0=none,1=left,2=right. Both-white can also mean the tape
has been lost; two digital sensors cannot distinguish those geometries alone.
V4's200ms fading memory can bridge brief gaps; it cannot establish whether the
line is centred or entirely lost, and is not a guaranteed reacquisition routine.
The existing3s line-loss stop remains. These are periodic snapshots, copied
separately from controller/motor status, not synchronized high-rate traces.

For the next run, check raised-wheel START/STOP, then the same clear straight
and gentle curve without intersections or an obstacle. Check for initial gentle
correction that grows when one sensor stays black, inner-wheel stalling and any
wrong-way steering. Keep GP21 ready. Retain the complete log through the stop.

## V3 operator run and V4 candidate

The v3 log repeatedly showed right-black, steer70 and wheel targets72/27mm/s.
Measured wheel speeds included73/26 and78/24, with drv_err=0: neither wheel
was persistently stalled in those samples. Steering later returned0 on both-
white readings; the first run ended with the3s line-loss stop. A later run
showed Buddy5 detecting an obstacle at23cm, followed by physical STOP evidence.
These observations do not establish physical STOP timing or an avoidance pass.

The operator requested adaptation from sensor readings without a fixed circle
radius. V4 retains50mm/s and strengthens sustained turns, ramps sooner, and
bridges brief same-side detection gaps with bounded fading memory. It does not
raise base speed or remove STOP/watchdog/TTL/run-time/line-loss protections.
No V4 floor result yet: check both turn directions, oscillation, curve tracking,
and inner-wheel stall. In particular the new19mm/s inner target is unqualified
on the floor; increasing speed alone does not change a requested curvature.

## V5 junction-pause fix and current validation status

V5 retains V4 steering and50mm/s base speed. The full operator log showed
repeated MOTION_FAULT immediately alongside the first junction observation,
then physical STOP adding0x1 to form0x9. Before that the wheel speeds were
nonzero, including80/20 and83/17mm/s on80/19 targets; those samples do not
establish an inner-wheel stall. The exact executor failure subtype was not
printed. Barcode calibration failed, so junctions request the probe fallback.

Code inspection identified a definite handoff issue: entering PROBE_SETTLE
withholds line refresh for250ms but leaves a150ms continuous lease live.
The measured executor interprets lease expiry as a timeout fault. V5 explicitly
calls the controller's normal zero-output motion pause when entering settle.
The existing output-failure latch, physical STOP, watchdog, goal limits and
production qualification gate remain in place. This fixes that code path;
physical confirmation that it resolves the reported stop is still needed.

The user explicitly requested skipping internal testing. No automated test
suites were run for V5; the157/19 results above belong to the previous V4
revision. V5 compiled successfully into a UF2. No flashing or car operation
was performed. Next hardware check: raised-wheel START/STOP, then a short
curve run; preserve the log across junction detection, probe and final stop.
Junction detection on a curve may itself need tuning if both sensors see black
without a real intersection; this change does not remove junction detection.
