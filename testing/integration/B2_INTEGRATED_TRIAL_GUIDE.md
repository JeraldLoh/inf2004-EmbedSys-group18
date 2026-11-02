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

V3 line candidate: cruise50mm/s (5cm/s), steering starts12, grows by1
every10ms and caps70 after580ms; curvature per steering unit120000 micro-1/m. Centred
steering returns to zero (no persistent last-side hold or counter-steer).
Initial correction requests approximately47/53mm/s; sustained full correction
requests approximately28/72mm/s after adapter rounding (nominal125mm radius,
versus333mm for v2). Neither wheel is commanded to reverse during line tracking;
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

## Software verification

157/157 native tests and19/19 shared suites pass. The added real line/controller/
motion-adapter test checks noise filtering, gradual correction, both turn signs,
correction limits, zero centred hold, slow-mode speed and STOP suppression.
V3 tests exercise intermediate corrections and sustained maximum in both directions.
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

## V3 line diagnostics

Each periodic CAR status includes `line_black L=... R=... state=... last_side=...`.
These are Buddy3's filtered black/white inputs (1=black), not raw GPIO voltages.
State0=centred/both-white interpretation,1=steer-left,2=steer-right,3=junction,
4=reacquire. Last-side0=none,1=left,2=right. Both-white can also mean the tape
has been lost; two digital sensors cannot distinguish those geometries alone.
The existing3s line-loss stop remains. These are periodic snapshots, copied
separately from controller/motor status, not synchronized high-rate traces.

For the next run, check raised-wheel START/STOP, then the same clear straight
and gentle curve without intersections or an obstacle. Check for initial gentle
correction that grows when one sensor stays black, inner-wheel stalling and any
wrong-way steering. Keep GP21 ready. Retain the complete log through the stop.
