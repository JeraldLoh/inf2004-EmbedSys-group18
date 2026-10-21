# Buddy 5 bench-test firmware

Tests Buddy 5's two components on their own: the **HC-SR04 ultrasonic sensor**
and the **scan servo**. This is a Pico SDK program for bench testing only. It is
not the micro T-Kernel image in `build_make/` and does not start any team task.

The obstacle logic is compiled straight from `app_program/subsystem5/`, so a
bench test always exercises the team's copy of the code.

**The wheels never move.** Motion (Buddy 2) and line sensing (Buddy 3) are
simulated: `[motion] ...` lines are print-outs only.

## Two builds

| File | What it does | When to use it |
|---|---|---|
| `buddy5_bench.uf2` | Reads the sensor, runs the full detect, scan, decide sequence and prints every step | Normal testing |
| `buddy5_servo_cal.uf2` | Only lets you step the servo by hand and take readings | Once, to find the servo's safe range (or after the mount changes) |

The normal build has no calibration prompt and starts testing straight away.

## Wiring (from `app_program/project_config/pinmap.h`)

| Part | Pin | Notes |
|---|---|---|
| HC-SR04 TRIG | GP6 | Output |
| HC-SR04 ECHO | GP7 | **5 V signal: use a 1k/2k voltage divider or an HC-SR04P**, or GP7 can be damaged |
| HC-SR04 VCC / GND | 5 V / GND | |
| Servo signal | GP12 | 50 Hz PWM |
| Servo power | 5 V / GND | Shared GND with the Pico |

## Build

Either ask Buddy 5 for the two `.uf2` files, or build them:

1. VS Code with the Raspberry Pi Pico extension: **Import Project**, select this
   folder, then **Compile**.
2. Or from a terminal with the Pico SDK installed:
   `cmake -S . -B build -G Ninja` then `cmake --build build`

## Flash and view the log

1. Hold **BOOTSEL**, plug in USB, release. A drive called `RPI-RP2` appears.
2. Copy the `.uf2` onto it. The Pico restarts by itself.
3. Open a serial terminal (PuTTY: Serial, the Pico's COM port, speed 115200)
   within 5 s. Find the COM port in Device Manager, under Ports (COM & LPT).

## Test procedure

Use a flat, hard object (book or box), not a hand. Ultrasound reflects badly
off fingers and soft or angled surfaces.

### S5-B01 Sensor distance accuracy (`buddy5_bench.uf2`)
Every second it prints `[monitor] ahead: XX.X cm`.
1. Place the object straight ahead at 10, 20, 50 and 100 cm (measure from the
   sensor face with a ruler).
2. Pass: each reading is within **±1 cm** of the ruler.

### S5-B02 Sensor edge cases (`buddy5_bench.uf2`)
1. Point at open space (nothing within about 4 m): prints `no-echo` or a large value.
2. Hold the object about 1 cm from the sensor: prints `too-close`.
3. Unplug the ECHO wire: within a few seconds it prints
   `[FAILED] ultrasonic sensor fault -- robot stopped`.
4. Pass: all three behave as described. Plug ECHO back in and reset.

### S5-B03 Servo movement and range (`buddy5_servo_cal.uf2`)
Keys: `a`/`d` step 5 deg, `A`/`D` step 1 deg, `c` centre, `m` measure,
`l`/`r` mark limits, `p` print settings, `q` park and stop.
1. At start it centres at 90 deg. Check the sensor points straight ahead.
2. Press `d` slowly. Stop as soon as it touches anything or buzzes; press `a`
   until it moves freely. Press `m`: if it reads only a few cm it is seeing the
   car itself, so step back more. Press `l` (or `r`, whichever way it is facing).
3. Repeat in the other direction and mark the other limit.
4. Press `p` and copy the printed `cfg->...` lines into `av_config_default()` in
   `app_program/subsystem5/avoidance.c`, then rebuild.
5. Pass: servo moves smoothly, holds each angle without jitter, and `p` prints
   a valid config. Record both limits in `../TEST_LOG.md`.

### S5-B04 Detection (`buddy5_bench.uf2`)
1. Slowly bring the object to under 15 cm straight ahead.
2. Pass: prints `[detect] obstacle ahead` and `[motion] STOP`, then the servo
   sweeps for the coarse scan.

### S5-B05 Scan and decision (`buddy5_bench.uf2`)
1. Object about 10 cm ahead, sides open: the servo sweeps 5 angles, then steps
   outward. Prints `[profile] ... width=...` and `[decide] ... -> LEFT/RIGHT`.
2. Measure the object's real width. Pass: printed width within **±30%**
   (briefing grade C; ±10% is grade A).
3. Put a wall or box on one side only: pass if it chooses the **other** side.

### S5-B06 Boxed in (`buddy5_bench.uf2`)
1. Surround the sensor with objects about 10 cm away on all sides.
2. Pass: `rescan #1`, `rescan #2`, `reversing (#1)`, repeated, then
   `[FAILED] no clear path after reversing`.

### S5-B07 Obstacle removed mid-scan (`buddy5_bench.uf2`)
1. Trigger a detection, then take the object away as soon as the servo starts sweeping.
2. Pass: prints `obstacle no longer ahead` and `[motion] RESUME line following`
   without a bypass.

After every encounter it prints a `[metrics]` line (detections, left/right
choices, timings). Copy these into the test log as evidence.

## Recording results

Add one row per run to `../TEST_LOG.md` with the date, test ID, commit, tester,
result and evidence (log text, photo or video link).
