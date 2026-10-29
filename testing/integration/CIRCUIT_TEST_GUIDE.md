# Manual full-circuit test v2 — 29 October 2026

This flashable image tests the connected motor/encoder, line/barcode, IMU,
sonar and scan-servo circuits. It starts sensing automatically, with wheel and
servo outputs disabled. Actuators only run after a fresh local GP20 gesture.
There is no automatic floor drive, obstacle manoeuvre, PID speed trial or
distance/pivot goal in this diagnostic. B1 networking is disabled.

## Flash and first report

1. Secure the car so **both wheels are clear of the surface**. Keep the servo
   linkage clear and GP21 plus a physical power disconnect accessible. Keep
   the existing protected sonar ECHO connection; GP17 must not receive 5 V.
2. Hold the Pico's BOOTSEL button while connecting USB. Copy the circuit-test
   UF2 to the RPI-RP2 drive. It reboots automatically.
3. Reopen the USB serial monitor (COM3 if Windows assigns it again). 115200
   baud is a convenient monitor setting; this is native USB CDC output.
4. Keep the car still and level during the IMU's initial calibration. There
   should be **no wheel or servo movement at boot**.
5. Save the startup and repeating `[CIRCUIT]` lines before powered tests.
   If CRITICAL appears or motion starts unexpectedly, disconnect power and
   report the log. An optional sensor init/start failure is reported separately
   and does not pretend the missing component passed.

USB receive commands are not implemented by this console. Use the board's
buttons; typing into the serial monitor does not start a test.

## Buttons and test menu

### Select wheel power while STOP is held

The default remains 15%. To select a higher starting pulse, **hold GP21 STOP**,
release GP20 if held, then tap and release GP20 while continuing to hold GP21.
Each debounced tap cycles **15 -> 20 -> 25 -> 30 -> 15%**. Wait for
`power selected=20%; STOP held, NO motion`. This changes only power selection,
not the test case, and does not start any output.

Release both buttons. A new >=1 s GP20 hold followed by release is still needed
to run the selected case once. Power cannot exceed 30%, and every wheel pulse
is still limited to 250 ms. Servo and sensor cases do not use this power value.
The older dedicated B2_BENCH image retains its original 15% maximum.

- A short GP20 press/release cycles to the next case. Watch the selected case
  printed on the monitor. Selection alone never activates an output.
- Hold GP20 for at least 1 s until `RELEASE ... ONCE`, then release. A stable
  release confirms exactly one test. Confirmation expires after 5 s.
- GP21 stops the wheels and servo. Releasing it does not resume a test; a fresh
  GP20 hold/release is required. Holding GP20 through boot or through a running
  test cannot queue a new actuator command.

| Case | Test |
|---|---|
| 0 | Left motor positive, right off |
| 1 | Left motor negative, right off |
| 2 | Right motor positive, left off |
| 3 | Right motor negative, left off |
| 4 | Both motors positive |
| 5 | Both motors negative |
| 6 | Barcode calibration: sweep black and white under GP26 for 3 s |
| 7 | One sonar measurement; point it at a flat object about 10–30 cm away |
| 8 | Restart IMU level calibration; keep still and level |
| 9 | Servo command 78 degrees for 250 ms, then PWM disabled |
| 10 | Servo command 90 degrees for 250 ms, then PWM disabled |
| 11 | Servo command 102 degrees for 250 ms, then PWM disabled |

Motor cases use **selected actuator effort, 15–30%, for 250 ms**. This is not
a calibrated speed command. A wheel may not start at 15%. Record a no-motion
result; do not interpret it as proof of encoder failure. Requested positive
and negative signs need to be checked against actual wheel direction.

Servo commands have no position feedback. Visually check which way the servo
moves; a printed command does not establish the angle reached. The firmware
uses the same physical STOP and timer cutoff to disable auxiliary servo PWM.

## Suggested order and observations

1. **Encoders, motors disabled:** manually rotate one wheel at a time. Only
   its count should increase. Check the wiring repair remains reliable.
2. **Line sensors:** move black/white material under each sensor separately.
   `GPIO7/GPIO0` are raw levels; `black L/R` shows B3's interpretation. Record
   reversed polarity rather than changing the harness while powered.
3. **Barcode:** verify `ADC26` changes over black/white with `valid=1`. Run
   case 6 and sweep for the full 3 s. `barcode_cal=1` means its contrast
   calibration succeeded. Sweep a known mapped Code 39 barcode and capture
   the printed character/route. Routes and junctions are reported only; no
   automatic turn or probe is executed.
4. **IMU:** look for changing sample sequence, calibration state, DATA_VALID
   and fault bits. After level calibration, gently tilt the chassis while
   wheels remain clear; pitch/roll should change. Their printed integer unit
   is **0.5 degree**. Heading is unreliable if the magnetometer fault is set.
   This alone does not validate hump-height accuracy or mounting orientation.
5. **Sonar:** case 7 prints status and distance in mm. Only `status=0` is a
   valid distance. NO_ECHO/FAULT must not be treated as a successful distance
   test; check the sensor's supply, protected ECHO path and target surface.
6. **Servo:** try case 10 (centre) first, then 9 and 11 if mechanically clear.
   Each action stops supplying PWM after the bounded pulse.
7. **Wheel power:** motor supply must be on and each encoder must respond to
   manual rotation. If the previous 15% pulse did not turn the wheel, select
   **20%** using the STOP-held gesture above, release both buttons, and run
   case 0 once. Then test case 2. Record actual direction plus
   the `trial ... end ... L ... R` line. Only proceed to both-wheel cases after
   the separate wheel checks. No calibrated straightness claim follows from
   raised-wheel rotation.
   If 20% gives no movement, repeat at 25%, then at most 30%, one explicit pulse
   at each step. If 30% still gives no movement, stop the motor tests and inspect
   M1/M2 output wiring, motor supply and driver outputs. Do not keep raising
   effort or proceed to floor driving. The image never ramps power by itself.
8. **STOP:** with a hand positioned over GP21, interrupt a bounded trial.
   `end=3` records a STOP-ended wheel trial; `end=2` means the time ended.
   Watching it stop does not establish the physical <=50 ms requirement;
   that requires measuring the actual motor input/output signals.

`IMU fresh=1` means its sample sequence has advanced within the last 250 ms.
`fresh=0` means the stored sample is not confirmed current, even if its raw
`valid=1` flag remains set. A frozen IMU sequence must be investigated before
any autonomous driving qualification. Sensor diagnostics remain available.

## Your calibration values are retained

The banner identifies calibration version 1:

- Wheel diameter: **65 mm**.
- Left: **1271**, right: **1264**, B-channel rising+falling edges per wheel
  revolution. These already refer to wheel revolutions; no additional gearbox
  multiplier is applied.
- Ruler-measured track width: **112 mm**. It is reported for provenance, not
  used to claim a qualified pivot in this image.

Each valid wheel trial prints the observed counts and estimated wheel rotation
distance, using circumference = pi × diameter and the separate left/right
calibration. Distances are magnitudes, not measured direction or floor position.
Optional convention check: ten manual wheel turns should be approximately
12710 left / 12640 right edges; record actual before/after counts before changing
existing calibration. Single-channel encoders cannot independently prove the
direction of rotation, and serviced edges can be lost at excessive rates.

PID gains, starting effort under load, effective pivot width and physical STOP
timing remain separate qualification work. This diagnostic does not change
the unqualified CAR_RUN PID configuration or declare the whole car qualified.

## Software/build record

The course profile is `CIRCUIT_TEST=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0
WIFI=none NET=none MQTT=0 TEST=0 B2_BENCH=0 B5_RUN=0 CAR_RUN=0`.
It uses a 1 ms kernel tick for B3 acquisition, core-0 diagnostic task with a
6144-byte static stack, and the production B2 driver/STOP/timer path.

Source: `app_program/diagnostics/circuit_test.c`. Normal/bench/CAR_RUN startup
remain separate. The raw ADC snapshot comes from B3's existing acquisition
owner; no second task reads its ADC device. IMU uses B4's task, sonar B5's
driver, and servo B5's bounded PWM setup with auxiliary STOP/cutoff.

92 B2/circuit native CTests pass, including all six motor cases, physical STOP,
servo cutoff, late STOP before servo activation, no boot actuation, calibration
requests, peripheral failure and task creation/start failure. Native HAL and
kernel/peripheral mocks are not evidence of electrical or real-time behavior.
V2 adds real-loop checks for 20/25/30% selection, 30% STOP, power cycling back
to 15%, no motion from power selection alone and stale IMU indication. Trial
policy checks retain the old bench cap and reject excessive effort/duration.
The target Cortex-M0+ build is checked with warnings as errors. UF2 family,
block layout, flash address range and SHA-256 are verified in the artifact
manifest. Nothing was flashed or operated while creating the file.
