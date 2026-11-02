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

Trial line cruise100mm/s, maximum wheel target120mm/s, maximum goal1m. Each
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

156/156 native tests and19/19 shared suites pass. Includes real profile values,
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
