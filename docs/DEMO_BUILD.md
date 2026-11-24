# Demo integration on latest main — 24 November 2026

Base: GitHub `origin/main` at `b0dee3ee3fce810bc39c496031016342b8ba91da` (fetched and checked on 24 November). Local branch `demo-integration-20261124` imports all 12 commits from `buddy2-demo-fixes`, then addresses the independent review findings. The original main checkout and old fixes branch are preserved. Nothing has been pushed or flashed.

## Build and firmware selection

Use `Project/robot-demo-integration`, not the preserved `Project/robot-testing` checkout. Its ignored `config/wifi_secrets.h` contains the locally supplied settings. Do not commit this file or share a UF2 containing the credentials. Recheck the laptop Wi-Fi IPv4 after changing networks; the current MQTT build uses `172.20.10.2:1883`, topic `group18/robot1`.

Set `PICO_SDK_PATH` and put Arm GCC and Make on PATH. From `build_make/`, the frozen baseline command is:

```sh
make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=cyw43 NET=lwip MQTT=1 TEST=0 B1_DEMO_RUN=0 B2_BENCH=0 B5_RUN=0 CAR_RUN=1 CAR_MQTT=1 CAR_B2_TRIAL=1 CIRCUIT_TEST=0 B2_MOVE_TEST=0 OBSTACLE_TABLE=0 LINE_PID=0 NO_BARCODE_CAL=0 CRUISE_MM_S=80 LINE_DAMP_STEER=40 LINE_DAMP_MS=40 LINE_LOST_MS=0 LINE_BACKTRACK_MM=0 BARCODE_RETRY=0 WATCHDOG_RETRY=1 WATCHDOG_TEST=0 OFF_GROUND=0 S3_GROUND_LOG=1 LIFT_MM=200 LIFT_WINDOW_MS=1200 LIFT_JOLT_PAUSE=0
```

Output: `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_carrun_b1live.uf2`. Different profiles share that filename, so use the separately named packaged files and their manifest. The serial `[CAR DEMO]` banner identifies the enabled features. Every listed behavior setting is included in the Make profile identity.

The packaged **baseline** is the first floor-test candidate: 80 mm/s, digital line steering, damping 40, fresh-IMU handling, controller-approved obstacle moves, wrap-safe Buddy 3 time, physical watchdog restart, MQTT and lightweight ground logging. Its normal straight trim remains 95% on the right wheel. It has no automatic reverse/search or barcode re-read. Both-white is normal when the two sensors straddle a straight line, so no-black timing alone cannot reliably establish line loss.

The packaged **recovery test** build uses the same command with `LINE_LOST_MS=1500 LINE_BACKTRACK_MM=150 BARCODE_RETRY=1 LIFT_JOLT_PAUSE=1`. It exercises the optional code; it is not a qualified scoring image. First test reverse on the floor, then test search and barcode retry independently by changing one knob at a time. The previously observed reverse-wheel stall can still trigger Buddy 2's protective MOTION_FAULT. An exhausted search still stops with RECOVERY_FAILED. A successful compile does not validate these maneuvers on the car.

## Implemented changes

| Area | Result | Remaining check |
| --- | --- | --- |
| F4 build | NO_BARCODE_CAL defaults to 0; explicit frozen settings; damping 40 in supplied images | Straight and both bend directions with MQTT active |
| F3 freshness | Unchanging IMU sequence ages out after 200 ms; its old speed cap is removed; stale B4 sections stop being republished with new timestamps | Actual IMU loss/recovery, including last-height dashboard freshness |
| F2 line recovery | Optional pivot search; bounded reverse uses actual encoder distance since last line contact; reverse stops early on reacquisition, then resumes with new authority | Both-white ambiguity and left reverse stall; disabled in baseline |
| F5 ownership | Each B5 goal/arc passes controller source/generation checks; stopping stays direct; rejection accounting now includes guard/binding failures after initialization | Full bypass/rejoin and all refusal/timeout logs |
| R3 clock | Buddy 3 milliseconds advance across raw microsecond wrap | Long powered-on run |
| R5 restart | A fresh GP20 gesture may retry the integrated car's watchdog cutoff; MQTT START cannot clear it; normal motion faults remain latched | Bench-only WATCHDOG_TEST=1, then rebuild with 0 |
| R7 retrace | Successive reverse encoder waypoints preserve bend order; full requested history is required; ring/recording are omitted when BARCODE_RETRY=0 | Physical slip, reverse rolling speed, barcode alignment; disabled in baseline |
| R7 episode | Reverse-decoded routes are discarded and decoder/events reset before the forward pass; no repeated retry on the same junction approach | One retry is re-armed by a completed junction or a new run, not merely a timer; this can conservatively skip a second barcode between junctions |
| F1 pickup | Optical/IMU confirmation, stale-sample rejection, periodic motor-stop reassertion and floor-confirmed restart gate are implemented | ADC/polarity characterization is required before optical activation |
| Lift settings | LIFT_MM, LIFT_WINDOW_MS and bounded LIFT_JOLT_PAUSE are real build knobs | Original 200 mm/1200 ms limits retained in supplied images |
| LINE_PID | Knob now actually defines APP_S3_LINE_PID and participates in profile identity | Remains 0; enabling needs GP28 analogue wiring and calibration |

F3, R3, F5 and the interface fixes are unconditional corrections. Selected recovery behaviors are optional; the entire branch does not preserve main's resource use/behavior in every respect. Encoder waypoints do not guarantee exact physical path replay because wheel slip and quantization are unmeasured. No new wire format or invented height/width measurement is introduced.

## Pickup measurement gate

`OFF_GROUND=1` without `OFF_GROUND_ADC` now fails the build instead of silently doing nothing. The baseline emits `[GROUND] L=... R=... adc=... valid=... seq=...` at 10 Hz without enabling the large waveform diagnostics.

With motors disabled, collect those lines on white, the line, a crossbar, each hump, and at 2/5/20 cm above the floor, upright and tilted. Confirm whether each normalized digital black bit is 0 or 1 when raised, and whether the air ADC readings can be separated from ALL track readings. Only then choose the threshold and polarities:

```text
OFF_GROUND=1 OFF_GROUND_ADC=<measured integer>
OFF_GROUND_LEFT_BLACK=<0 or 1> OFF_GROUND_RIGHT_BLACK=<0 or 1>
OFF_GROUND_CONFIRM_MS=300 OFF_GROUND_ALONE_MS=1000
```

These are settings to add to a full build command, not literal placeholder arguments to Make. ADC range is 101..4095. A candidate requires both digital values and ADC >= the measured threshold. A fresh recent IMU rise permits 300 ms confirmation; otherwise 1000 ms is required. Junction maneuvers/searches do not accumulate confirmation time. An optical stop stays latched through sensor loss. Fresh ADC below threshold minus 100 for one second unlocks a fresh operator/remote START; it never automatically resumes. Existing IMU lift/overturn stops remain active.

The proposed 150 mm/2000 ms retune was tried in simulation. The existing below-threshold 120 mm lift fixture then reached a reported 150 mm and tripped the threshold. This is evidence of height-estimate error near that candidate threshold, not an observed floor/hump false stop or a validated physical pickup result. The original limits are retained until characterization. IMU-only detection still cannot guarantee an arbitrarily slow pickup or distinguish being held still in the air.

## Rubric and demonstration

The changes improve correctness, observability and recovery options. No rubric point has been physically demonstrated by these builds. Hump height and obstacle width still need comparison against real measured fixtures; MQTT needs fresh real telemetry, and obstacle navigation needs a complete rejoin.

For the ten points: observe barcode capture, decoded character, measured hump height, line following, the mapped barcode turn, obstacle detection, width, complete bypass/rejoin, real MQTT telemetry, and a pickup that stops both motors. Pickup ends the judged run. Use the assigned slot. At most three judged runs; each new run replaces the previous score. A GP20 watchdog retry is a technical recovery mechanism, not a promise that an examiner treats it as the same run.

Suggested car checks: GP21 and deliberate pickup on a stand; three long straights and bends in both directions; known barcode plus mapped junction; measured hump; measured-width obstacle and five complete rejoins; a complete run with MQTT; then slow/low/tilted pickup after characterization. Keep baseline/recovery observations separate. Do not treat the recovery test UF2 as validated just because its software tests pass.

Tribunal wording: Buddy 5 proposes motion, the controller checks authority, and Buddy 2 owns motor outputs. Halt may bypass the controller to stop. Configured line-loss recovery searches, with a protective stop on exhaustion/fault. Barcode reading is not suspended merely because Buddy 4 sees a hump.
