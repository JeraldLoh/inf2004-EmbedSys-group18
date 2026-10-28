# Buddy 3 Line barcode and junction perception test record

Host tests check the logic only. Hardware cases still need G1 electrical
clearance; active motion needs explicit bounded-test approval and a physical
power cut.

## Host tests (no hardware)

Run from the repository root:

```
gcc -std=c11 -Wall -Wextra -Werror -Wpedantic -UNDEBUG -Iapp_program \
  testing/host/test_subsystem3_sensors.c \
  app_program/subsystem3/line_follow.c app_program/subsystem3/barcode_sensor.c \
  app_program/subsystem3/barcode_decoder.c app_program/subsystem3/barcode_edges.c \
  app_program/subsystem3/barcode_map.c \
  app_program/vehicle_controller/vehicle_controller.c \
  app_program/vehicle_controller/safety.c -o test_s3 && ./test_s3
```

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

## Hardware cases (to run on the car)

| ID | Purpose | Method | Acceptance |
|---|---|---|---|
| S3-T01 | Line sensor polarity | `S3_DEBUG_RAW=1`, hold each sensor over tape and foam | L/R read 1 on black, 0 on white |
| S3-T02 | Barcode contrast at mounting height | Calibration sweep, record white/black levels | black - white >= 300 counts |
| S3-T03 | Barcode decode on track | 10 passes per letter, each direction, at operating speed | >= 9/10 correct, 0 wrong letters |
| S3-T04 | Line following on curves | Full loop of the practice track, 3 runs | stays on line every run |
| S3-T05 | Junction detection | 10 crossings of a "+" | 10/10 reported once |
| S3-T06 | Probe angle | No-barcode "+" and T, 5 each | correct type 10/10 |
| S3-T07 | Barcode to junction time | Time from decode to junction at operating speed | sets `S3_ROUTE_VALID_MS` with margin |

## Results

Append actual runs only. Keep failures and link the exact tested commit,
profile, configuration and raw log/video.

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S3-H01..S3-H12 | buddy3 working tree, before first commit; host gcc 11, `-Wall -Wextra -Werror -Wpedantic` | Zane (host run) | PASS 12/12 | console: "subsystem3 sensor host tests passed (S3-H01..S3-H12)" | Logic only; no hardware. Re-run and replace commit field after committing. |
| 2026-10-28 | S3-H01..S3-H14 | buddy3 working tree (uncommitted, adds barcode_map); host gcc, `-Wall -Wextra -Werror -Wpedantic` | Zane (host run) | PASS 14/14 | console: "subsystem3 sensor host tests passed (S3-H01..S3-H14)" | Logic only; no hardware. |
