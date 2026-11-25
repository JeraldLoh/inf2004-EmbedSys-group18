# Buddy 3 Line barcode and junction perception test record

## November demo cases and bench follow-up

Use the [shared setup, tolerances and recording rules](../DEMO_TRACEABILITY.md) and [evidence ledger](../DEMO_EVIDENCE.md). These procedures govern the normal combined image. Earlier plans and results below apply to their recorded profiles.

| ID | Priority, owner and requirement | Setup and steps | Pass criteria and evidence | Status |
|---|---|---|---|---|
| S3-T01 | P0 · B3 · Active IR pins and polarity | With wheels secured, place each line head over tape and background separately, then centred on the straddled line. Capture raw GP7/GP0 values and decoded black flags, verify GP26 is the separate barcode input. | Each head resolves black vs white on the intended channel. Centred both-white is ordinary tracking evidence in this geometry. Save raw/mapped states and mount photo. | Current profile calibration Not run. |
| S3-T02 | P0 · B3 · Barcode contrast / ADC validity | On normal profile at actual height, sweep a known Code 39 strip and quiet zone through the GP26 sensor under representative lighting. Log black/white ranges, thresholds, gaps, rejected readings and recovery. | No overlap across operating thresholds in a usable sweep, no invalid sample counted as a bar. Record margin; 300 counts belongs to the historical bench criterion. Save thresholds and sample CSV. | E2–E4 ADC/capture evidence, controlled contrast pending. |
| S3-T03 | P0 · B3 · Known code detection and decode | Use measured Code 39 prints with start/stop and quiet zones. At normal speed run ten passes per installed character and intended scan direction plus ten blank/crossing controls. Log capture, runs/windows, decode and expected truth independently. | P: ≥9/10 correct per character/direction, zero wrong characters and zero accepted controls. Record capture and correct decode rates independently. Save fixture photo and per-pass export extract. | E2/E4 real complete decodes and partial captures; rates Not run. |
| S3-T04 | P0 · B3 with B2 · Straight and bend tracking | Run practice loop with straights and both bends three times, then repeat with one modest documented lighting or surface variation. Record sensor flags, steer, wheel targets and overhead path. | P: zero manual corrections/track departures over each loop, steering sign matches each turn and no ordinary line-loss reverse. | E1 historical operator observation; controlled repetition open. |
| S3-T05 | P1 · B3 · Junction debounce | Cross a marked + ten times with normal mounting. Insert short single-sensor transients and ordinary centred white in marked portions; use route case for T/+ decisions. | P: 10/10 crossings each generate one junction, zero extra events in controls. Save physical crossing and event sequence. | E3/E4 junction events, fixture-based count Not run. |
| S3-T06 | P1 · B3 · Legacy no-barcode probe applicability | Inspect normal firmware phase and run INT-29b with + and T fixtures. Preserve the legacy probe-angle case for matching bench images. | In normal combined, record actual pause/straight fallback; capture any unexpected automatic right turn or probe. Legacy procedure is applicable only with its matching profile. | Normal-profile regression Not run. |
| S3-T07 | P0 · B3/controller · Route lifetime: barcode route applies at the next junction only | Decode a mapped barcode, then cross two junctions. Repeat with a short and a long barcode-to-junction distance. Record decode and junction timestamps from `[S3] Route` and controller lines. | P: the route runs at the first junction after the decode, and the second junction gets the no-barcode behaviour (pause, then straight). Barcode routes have no timer (valid until the next junction); record decode-to-junction time as information, not a limit. Save timestamps. | E3 route completion observed. Two-junction check Not run. |

Host tests check the logic only. Hardware cases still need G1 electrical
clearance; active motion needs explicit bounded-test approval and a physical
power cut.

## Host tests (no hardware)

Run from the repository root (or build `testing/host` with CMake and run the
same four targets):

```
gcc -std=c11 -Wall -Wextra -Werror -Wpedantic -UNDEBUG \
  testing/host/test_perception_math.c \
  app_program/subsystem3/barcode_decoder.c app_program/subsystem3/barcode_map.c \
  app_program/subsystem4/imu_math.c -lm -o perception && ./perception
gcc -std=c11 -Wall -Wextra -Werror -Wpedantic -UNDEBUG \
  -DLINE_PID_KP_X100=80 -DLINE_PID_KI_X100=0 -DLINE_PID_KD_X100=300 \
  testing/host/test_line_pid.c app_program/subsystem3/line_pid.c -o line_pid && ./line_pid
gcc -std=c11 -Wall -Wextra -Werror -Wpedantic -UNDEBUG \
  -DLINE_PID_KP_X100=80 -DLINE_PID_KI_X100=100 -DLINE_PID_KD_X100=0 \
  testing/host/test_line_pid.c app_program/subsystem3/line_pid.c -o line_pid_i && ./line_pid_i
cd testing/host && gcc -std=c11 -Wall -Wextra -Werror -Wpedantic -UNDEBUG \
  -DB1_DEMO=0 -Ibuddy1_stubs test_buddy1_map.c \
  ../../app_program/subsystem1/buddy1_bridge.c ../../app_program/subsystem3/barcode_map.c \
  ../../app_program/subsystem1/buddy1_live_status.c ../../app_program/subsystem1/buddy1_binary.c \
  ../../app_program/subsystem1/buddy1_diagnostics.c ../../app_program/subsystem1/buddy1_protocol.c \
  -o map && ./map
```

| Target | Covers |
|---|---|
| `perception_math_host_tests` | Code 39 decoding of 'A': partial read then real, measured car widths, after long black, blurred reverse read, A maps to Left; re-read signature accepts and rejects |
| `buddy1_map_tests` | MAP_SET / MAP_SHOW through Buddy 1 into the barcode map: empty power-on map, whole-map replace, bad entry and second letter for one direction rejected, map revision |
| `line_pid_host_tests` | Line PID with P+D gains |
| `line_pid_integral_host_tests` | Line PID with P+I gains |

### Retired suite S3-H01..S3-H16

These cases ran from `testing/host/test_subsystem3_sensors.c`, which was
removed together with `barcode_edges.c` and `line_drive.c`. The table is kept
because the October results below refer to it. Reverse decoding (part of
S3-H07) is covered by `perception_math_host_tests`, and the run-time map
(S3-H13, S3-H14) by `buddy1_map_tests`. These have no current host test:
full A-Z, 0-9 decoding, malformed reads (S3-H08), and the line-follow logic
(filter, steering ramp and damping, junction debounce, reacquire).

| ID | Purpose | Acceptance |
|---|---|---|
| S3-H01 | Packed status is 1 byte | `sizeof(line_status_t) == 1` |
| S3-H02 | Majority filter ignores single-sample glitches | 1-sample blip does not change state |
| S3-H03 | Steering ramps and damps | magnitude grows with time off centre; counter-steer after correction |
| S3-H04 | Junction debounce, single report | 5 ms both-black ignored; 100 ms reported once |
| S3-H05 | Barcode calibration and hysteresis | thresholds from sweep; band holds state |
| S3-H06 | Adaptation follows lighting, never loses contrast | levels track drift; low-contrast drift refused |
| S3-H07 | Code 39 A-D forward and reverse at 50-300 mm/s | all decode correctly |
| S3-H08 | Malformed reads rejected | 4-wide character and wide gap give no read |
| S3-H09 | Barcode then junction executes route (A-D) | controller EXECUTE_ROUTE with that command |
| S3-H10 | No barcode: "+" straight, T right; barcode overrides | as stated |
| S3-H11 | Reacquire steers to last side, ends on black | as stated |
| S3-H12 | Edge buffer order, packing, overflow | FIFO order; time/level round-trip; full flagged |
| S3-H13 | Full Code 39 A-Z, 0-9, both directions, run-time map | every character decodes to itself both ways; unmapped letter gives command NONE |
| S3-H14 | `MAP` command parsing | set/clear/default/show work; bad message changes nothing |
| S3-H15 | Bench steering mixer | right steer speeds left wheel; efforts clamped 0..max, never negative |
| S3-H16 | Bench junction rule | stop for wait; barcode turn, else straight; route used once; U-turn ends run |

## Retired bench cases (`S3_BENCH` images)

Superseded by the November demo cases at the top of this file. The
`S3_BENCH` images these cases need were removed on 2026-10-29, so they can no
longer be run as written. Kept for reference only.

| ID | Purpose | Method | Acceptance |
|---|---|---|---|
| S3-T01 | Line sensor polarity | `S3_BENCH=1` image, hold each sensor over tape and foam | L/R read 1 on black, 0 on white |
| S3-T02 | Barcode contrast at mounting height | Calibration sweep, record white/black levels | black - white >= 300 counts |
| S3-T03 | Barcode decode on track | 10 passes per letter, each direction, at operating speed | >= 9/10 correct, 0 wrong letters |
| S3-T04 | Line following on curves | Full loop of the practice track, 3 runs | stays on line every run |
| S3-T05 | Junction detection | 10 crossings of a "+" | 10/10 reported once |
| S3-T06 | Probe angle | No-barcode "+" and T, 5 each | correct type 10/10 |
| S3-T08 | Bench line follow, wheels raised | `S3_BENCH=2`, move tape under sensors by hand | wheel speeds follow steer; GP21 stops at once; run ends at 20 s |
| S3-T09 | Bench line follow on floor | `S3_BENCH=2` on the practice loop, 3 runs | stays on line; stops at each junction, then straight with no barcode |
| S3-T07 | Barcode to junction time | Time from decode to junction at operating speed | sets `S3_ROUTE_VALID_MS` with margin |

## Results

Append actual runs only. Keep failures and link the exact tested commit,
profile, configuration and raw log/video.

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S3-H01..S3-H12 | buddy3 working tree, before first commit; host gcc 11, `-Wall -Wextra -Werror -Wpedantic` | Zane (host run) | PASS 12/12 | console: "subsystem3 sensor host tests passed (S3-H01..S3-H12)" | Logic only; no hardware. Re-run and replace commit field after committing. |
| 2026-10-28 | S3-H01..S3-H14 | buddy3 working tree (uncommitted, adds barcode_map); host gcc, `-Wall -Wextra -Werror -Wpedantic` | Zane (host run) | PASS 14/14 | console: "subsystem3 sensor host tests passed (S3-H01..S3-H14)" | Logic only; no hardware. |
| 2026-10-28 | S3-H01..S3-H16 | buddy3 working tree (adds line_drive, s3_bench); host gcc `-Wall -Wextra -Werror -Wpedantic` | Zane (host run) | PASS 16/16 | console: "subsystem3 sensor host tests passed (S3-H01..S3-H16)"; B2 suite 38/38; normal, B2_BENCH, S3_BENCH=1, S3_BENCH=2 images compile and link (arm-none-eabi-gcc 13.2, pico-sdk 2.2.0) | Logic and build only; not flashed. |
| 2026-11-25 | perception_math, buddy1_map, line_pid, line_pid_integral host tests | `ee0e70d` (main), host gcc `-Wall -Wextra -Werror -Wpedantic` | Claude for Zane (host run) | PASS 4/4 | console: "perception math host tests passed", "PASS binary MAP set/show through the bridge, whole-table replace, bad entry rejected", "line_pid host tests passed" (x2) | Logic only; no hardware. Old S3-H suite no longer exists. |

## November evidence review

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S3-T03 | Partial-marker, closing-bars and controller-trace, local changes on da4e9d9 | Operator (name unrecorded) | Partial | [Evidence E2–E4](../DEMO_EVIDENCE.md) | Successful 29-run captures and failed captures on track. Fixture-by-fixture accuracy and trial denominator open. |
| 2026-11-24–25 | S3-T04 | Original 80 mm/s baseline, da4e9d9 | Operator (name unrecorded) | Partial | [Evidence E1](../DEMO_EVIDENCE.md) | Operator observed good tracking on straights and bends. Confirm controlled repetitions on chosen normal profile. |
