# Whole-car test plan (current main builds + `CAR_RUN=1`)

Status: nothing below has been run on hardware. Every image builds and links
(2026-10-28, branch `integration-drive`). Record each run in `TEST_LOG.md`
(integration) or the owner's `TEST_LOG.md`, including failures.

## Safety rules for every step

- Wheels raised (car on a block, wheels free) until a step says "floor".
- GP21 STOP within reach. Know where the motor power cut is: USB can also
  power the motors on the Robo board.
- Serial monitor open on the Pico's COM port before you need the output. The
  B5_RUN and CAR_RUN images wait up to 5 s for it at boot.
- First floor runs: slow, open area, someone ready on STOP, a short loop.
- If anything moves that should not, press STOP, unplug, and log it.

## 1. Build

Git Bash, from `build_make/`. Tools: Arm GNU toolchain (Pico SDK
extension's `toolchain/.../bin` on PATH), GNU make (Windows:
`winget install ezwinports.make`), a host C++ compiler for the `elf2uf2`
tool (any `c++`; zig works: `zig c++ -O2 -std=c++14 -I../tools/elf2uf2 -o
tools/elf2uf2.exe ../tools/elf2uf2/main.cpp` once, before `make`).

```
COMMON="SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 PICO_SDK_PATH=<path to pico-sdk 2.3.1>"
make -j4 $COMMON              # default: starts nothing
make -j4 $COMMON B2_BENCH=1   # Buddy 2 wheel bench
make -j4 $COMMON B5_RUN=1     # Buddy 5 obstacle task, motors off
make -j4 $COMMON CAR_RUN=1    # whole car
```

Each image is named after its profile, e.g. `mtk3pico_smp1_usb_cdc_carrun.uf2`.
Switching profile rebuilds everything automatically.

Flash: hold BOOTSEL, plug in USB, drag the `.uf2` onto the RPI-RP2 drive.

## 2. Current main builds (before the whole car)

| ID | Image | Steps | Pass |
|---|---|---|---|
| INT-20 | default | Flash, open console | Prints `inactive architecture scaffold`; nothing moves |
| INT-21 | B2_BENCH | Follow `../subsystem2/B2_RUNTIME_TEST_GUIDE.md` cases 0-5, raised | Each wheel turns the commanded way; encoder counts rise on that wheel; STOP during a trial stops it |
| INT-24 | B5_RUN | Test S5-T01 (`../subsystem5/TEST_LOG.md`): hand under 25 cm | `[B5] obstacle ahead`, servo sweeps, `refused` lines; motors never move; status line every 1 s |

## 3. Whole car (`CAR_RUN=1`)

Boot banner: `[CAR] whole-car build ...`. Two status lines every second:
- `[CAR] run=.. mode=.. mission=.. stops=0x.. steer=.. | effort L R speed L R mm/s | drive rej exec_rej drv_err`
- `[B5] state=.. servo=.. dist=..cm | ctrl mode=.. mission=.. | move=.. ...`

Modes: 2 = AUTO, 5 = STOPPED. Missions: 1 TRACK_LINE, 2 JUNCTION_APPROACH,
3 JUNCTION_EXECUTE, 4 VERIFY_LINE, 5 OBSTACLE_STOP, 6 SCAN, 7 PLAN_BYPASS,
8 BYPASS, 9 RECOVERY, 10 SAFE_STOP. `junc phase`: 0 idle, 1 probe settle,
2/3 probe pivot out/back, 4 probe result, 5 approach, 6 turn. `route`:
1 LEFT, 2 RIGHT, 3 STRAIGHT, 4 UTURN.

| ID | Where | Steps | Pass | Record |
|---|---|---|---|---|
| INT-25a | Raised | Boot, do nothing for 30 s | No wheel movement; `run=0`, effort 0/0 | Console log |
| INT-25b | Raised | Hold GP21, then try GP20 hold/release | Start refused while STOP held; no movement | Log |
| INT-25c | Raised | GP20 hold 1 s, release, tape centred under both sensors | `run=1 mode=2 mission=1`; both wheels forward, effort ~200; `drive` count rising | Log, efforts |
| INT-25d | Raised | Move tape left/right under the sensors | `steer` changes sign; inner wheel slows, outer speeds up; never reverses | Log, video |
| INT-25e | Raised | Press GP21 while running | Wheels stop at once; `[CAR] STOPPED: STOP button`; `stops` shows 0x1; restarts only with a new GP20 gesture | Log, time to stop |
| INT-25f | Raised | Let it run 60 s | Stops by itself: `run time limit` | Log |
| INT-26a | Raised | Running, hand at 20 cm in front of the sonar | Wheels stop at once; mission 5 -> 6; servo sweeps; then pivot goal (wheels turn opposite ways briefly) | Log, video |
| INT-26b | Raised | Running, hand removed during the scan | Back to mission 1 (line) or 9 then 1 | Log |
| INT-27a | Floor | Slow loop of tape, no junctions; GP20 start | Follows the loop for 60 s or until STOP; stays on the line | Video, laps, off-line count |
| INT-27b | Floor | Box on the loop | Stops before it (>= 6 cm), scans, goes round, finds the line again, carries on | Video, log, encounter time |
| INT-27c | Floor | Wide wall across the loop | Gives up safely: latched safe stop (`stops` shows 0x80), no collision | Video, log |
| INT-28a | Raised | Running; pass a mapped barcode (e.g. `A` = LEFT) under the barcode sensor, then black tape across both line sensors | `route=1`; mission 3; both wheels forward briefly (phase 5), then a left pivot (phase 6); then mission 4 and line following | Log, video |
| INT-28b | Raised | Running; tape across both line sensors, no barcode | Mission 2; wheels stop (phase 1); small pivot left and back (phases 2, 3); then a route and the approach/turn as in INT-28a | Log, video |
| INT-28c | Raised | Pass a mapped barcode, wait more than 10 s (route expires), then tape across both line sensors | Latched stop `stops` 0x100 (ROUTE_REQUIRED); a new GP20 start clears it | Log |
| INT-29a | Floor | `+` junction on the loop, barcode before it | Turns the barcode's way, finds the new line, carries on | Video, log |
| INT-29b | Floor | `+` and T junctions with no barcode | `+`: straight on; T: right | Video, log |

## 4. Calibration to do during these tests

| What | How | Where to put the result |
|---|---|---|
| Track width | 112 mm measured with a ruler; check with a 360 deg pivot (turn ends square to the start) | `MOTION_TRACK_WIDTH_MM` (subsystem2/motion_executor.h) |
| Effort vs speed | INT-25c: read `speed L/R` at effort 200 | `effort_per_100_mm_s` in `motion_executor_default_config()` |
| Start effort | Lowest effort that moves both wheels on the floor | `goal_min_effort` |
| Cruise speed / steering | INT-27a: off-line count at 150 mm/s; adjust if it weaves or cuts corners | `controller_line_default_config()` |
| Junction approach | Ruler: line sensors to wheel axle (placeholder 50 mm). INT-29a: the car should pivot over the junction centre | `approach_mm` in `controller_junction_default_config()` |
| Probe angle | INT-29b: a `+` must read CROSS and a T must read T (`[S3] No barcode: junction is ...`) | `probe_mdeg` (10 deg) in `controller_junction_default_config()` |

Record the old and new value, the date and the evidence (log/video) in
`../../docs/calibration/`.

## 5. Known limits of this build

- Buddy 2's executor is a DRAFT (decision D13): open-loop speed, no PID,
  placeholders above. Buddy 2 must review it before floor runs.
- Junction geometry (approach distance, probe angle, 90 deg turns) is untested
  placeholders (D14): test INT-28 raised before INT-29 on the floor.
- Barcode calibration: sweep the sensor over black and white for 3 s right
  after `[CAR] Sweep the barcode sensor ... now`; check for `[S3] Barcode OK`.
  If it says FAILED, barcodes are ignored (every junction uses the probe)
  until the board is reset.
- Turn angles use the ruler-measured 112 mm track; not yet pivot-checked (D12).
- Not run on hardware at all yet.
