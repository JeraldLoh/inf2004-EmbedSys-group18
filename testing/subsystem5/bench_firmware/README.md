# Buddy 5 bench-test firmware

Tests Buddy 5's two components on their own: the **HC-SR04 ultrasonic sensor**
and the **scan servo**. This is a Pico SDK program for bench testing only. It is
not the micro T-Kernel image in `build_make/` and does not start any team task.

The obstacle logic is compiled straight from `app_program/subsystem5/`, so a
bench test always exercises the team's copy of the code.

**The wheels never move.** Motion (Buddy 2) and line sensing (Buddy 3) are
simulated: `[motion] ...` lines are print-outs only.

## What happens when it boots

One file, `buddy5_bench.uf2`, does everything:

1. **Calibration menu.** Shows the saved servo range and counts down 5 s:
   - `Enter` or `u`: use the saved calibration and start.
   - `c`: calibrate the servo now (see S5-B03). `s` saves it and starts the car.
   - `x`: erase the saved calibration and use the built-in values.
   - No key (or no PC plugged in): uses the saved calibration automatically.

   The calibration is stored in the Pico's flash, so it survives power-off and
   reflashing. Calibrate once; after that just press Enter (or wait).
   Built-in values (measured on our car): LEFT 102, CENTER 90, RIGHT 78,
   fine scan +/-10 deg.
2. **Configuration.** Prints the servo angles, scan angles and thresholds in use.
3. **Self-check.** Sweeps the servo to each scan angle and prints a reading at
   each one, then says `sensor OK` or a wiring warning.
   If any angle reads `sensor-fault`, it runs a pin diagnostic that says whether
   ECHO is stuck high, no ECHO pulse arrives (power / pins / divider), or the
   pulse is fine.
4. **Run.** Prints the distance ahead every second, every scan reading, the
   obstacle profile, the decision, every `[state] A -> B` change, and
   `[metrics]` after each encounter. Keys: `m` metrics, `d` sensor pin
   diagnostic, `r` restart after
   FAILED, `h` help.

## Wiring (from `app_program/project_config/pinmap.h`)

| Part | Pin | Notes |
|---|---|---|
| HC-SR04 TRIG | GP6 | Output |
| HC-SR04 ECHO | GP7 | **5 V signal: use a 1k/2k voltage divider or an HC-SR04P**, or GP7 can be damaged |
| HC-SR04 VCC / GND | 5 V / GND | |
| Servo signal | GP12 | 50 Hz PWM |
| Servo power | 5 V / GND | Shared GND with the Pico |

## Build

Either ask Buddy 5 for `buddy5_bench.uf2`, or build it:

1. VS Code with the Raspberry Pi Pico extension: **Import Project**, select this
   folder, then **Compile**.
2. Or from a terminal with the Pico SDK installed:
   `cmake -S . -B build -G Ninja` then `cmake --build build`

## Flash and view the log

1. Hold **BOOTSEL**, plug in USB, release. A drive called `RPI-RP2` appears.
2. Copy the `.uf2` onto it. The Pico restarts by itself.
3. Open a serial terminal (PuTTY: Serial, the Pico's COM port, speed 115200)
   within 5 s, so you see the calibration menu. Find the COM port in Device Manager, under Ports (COM & LPT).

## Test procedure

Use a flat, hard object (book or box), not a hand. Ultrasound reflects badly
off fingers and soft or angled surfaces. At the menu press Enter (or wait 5 s)
unless the test says to calibrate.

### S5-B01 Sensor distance accuracy
Every second it prints `[monitor] ahead: XX.X cm`.
1. Place the object straight ahead at 10, 20, 50 and 100 cm (measure from the
   sensor face with a ruler).
2. Pass: each reading is within **±1 cm** of the ruler.

### S5-B02 Sensor edge cases
1. Point at open space (nothing within about 4 m): prints `no-echo` or a large value.
2. Hold the object about 1 cm from the sensor: prints `too-close`.
3. Unplug the ECHO wire: within a few seconds it prints
   `[FAILED] ultrasonic sensor fault -- robot stopped`.
4. Pass: all three behave as described. Plug ECHO back in and press `r`.

### S5-B03 Servo movement and range (press `c` in the boot menu)
Keys: `a`/`d` step 5 deg, `A`/`D` step 1 deg, `c` centre, `m` measure,
`l`/`r` mark limits, `p` preview, `s` save and start, `q` quit without saving.
1. At start it centres at 90 deg. Check the sensor points straight ahead.
2. Press `d` slowly. Stop as soon as it touches anything or buzzes; press `a`
   until it moves freely. Press `m`: if it reads only a few cm it is seeing the
   car itself, so step back more. Press `l` (or `r`, whichever way it is facing).
3. Repeat in the other direction and mark the other limit.
4. Press `p` to preview, then `s` to save. Reset the Pico: the menu should now
   show the new values under `Saved in flash`.
5. Pass: servo moves smoothly, holds each angle without jitter, the saved
   values survive a reset, and the self-check sweep stays inside the limits.
   Record both limits in `../TEST_LOG.md`.

### S5-B04 Detection
1. Slowly bring the object to under 15 cm straight ahead.
2. Pass: prints `[detect] obstacle ahead` and `[motion] STOP`, then the servo
   sweeps for the coarse scan.

### S5-B05 Scan and decision
1. Object about 10 cm ahead, sides open: the servo sweeps the 5 coarse angles
   printed under `=== Configuration ===`, then steps outward. Prints `[profile] ... width=...` and `[decide] ... -> LEFT/RIGHT`.
2. Measure the object's real width. Pass: printed width within **±30%**
   (briefing grade C; ±10% is grade A).
3. Put a wall or box on one side only: pass if it chooses the **other** side.

### S5-B06 Boxed in
1. Surround the sensor with objects about 10 cm away on all sides.
2. Pass: `rescan #1`, `rescan #2`, `reversing (#1)`, repeated, then
   `[FAILED] no clear path after reversing`.

### S5-B07 Obstacle removed mid-scan
1. Trigger a detection, then take the object away as soon as the servo starts sweeping.
2. Pass: prints `obstacle no longer ahead` and `[motion] RESUME line following`
   without a bypass.

After every encounter it prints a `[metrics]` line (detections, left/right
choices, timings). Copy these into the test log as evidence.

## Recording results

Add one row per run to `../TEST_LOG.md` with the date, test ID, commit, tester,
result and evidence (log text, photo or video link).
