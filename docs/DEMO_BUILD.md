# Demo build (Group 18 line-following car)

This file freezes the one build command for the demo image. Run it from
`build_make/` with `PICO_SDK_PATH` set (see `BUDDY1_README.md`).

```bash
make -j6 CAR_RUN=1 CAR_MQTT=1 WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 TEST=0 LINE_DAMP_STEER=40 LINE_DAMP_MS=40 LINE_LOST_MS=1500 LINE_BACKTRACK_MM=150 OFF_GROUND=1
```

The team default for `LINE_DAMP_STEER` stays 15. The demo image passes 40 on
the command line only. Every knob below keeps today's behaviour when it is not
given. Each knob is part of `PROFILE_ID`, so changing one forces a clean
rebuild of the objects.

`LINE_BACKTRACK_MM=150` and `OFF_GROUND=1` in the demo command are
provisional until the F2 reverse test and the F1 tests pass on the car.

A knob that does not exist yet in the makefile is ignored by make. The "Added
by" column says which fix introduces it. Until that fix is merged the demo
command builds, but the knob has no effect.

| Knob | Default | Demo | Added by | What it does | Tested on the car |
|------|---------|------|----------|--------------|-------------------|
| `LINE_DAMP_STEER` | 15 | 40 | existing | Counter-steer percent applied after a line correction (Buddy 3 weave damping). 0 turns it off. | (fill in) |
| `LINE_DAMP_MS` | 40 | 40 | existing | How long the counter-steer lasts, in ms. | (fill in) |
| `NO_BARCODE_CAL` | 0 | default | F4 | Had no makefile default, so the old README command failed with "#if with no expression". Now defaults to 0 (the `app.h` value). 1 skips the start-up barcode sweep. | (fill in) |
| `B4_STALE_MS` | 200 | default | F3 | If the IMU sequence number does not change for this long, the speed cap is ignored. No stop. | (fill in) |
| `LINE_LOST_MS` | 0 (off) | 1500 | F2 | Controller starts a line search after this many ms without the line. `CAR_LINE_LOST_MS` (supervisor stop) stays 0. | (fill in) |
| `LINE_BACKTRACK_MM` | 0 (off) | 150 | F2 | Before the pivot search in track-line mode, reverse the distance driven since the line was last seen, capped at this many mm. | (fill in) |
| `OFF_GROUND` | 0 (off) | 1 | F1 | Enables the off-ground stop from Buddy 3 sensors plus a timer and IMU agreement. | (fill in) |

Further F1 knobs (`OFF_GROUND_ADC`, `OFF_GROUND_CONFIRM_MS`,
`OFF_GROUND_ALONE_MS`, `LIFT_MM`, `LIFT_WINDOW_MS`) are added with F1 after the
M1 measurement. They will be listed here then.

## Check that a build still matches the default

`make` with no extra knobs must produce the same CFLAGS as before for every
existing knob.

## Notes for the report

- Hump suspension of barcode decoding is rejected, since barcodes are on the
  left and humps on the right. Confirm on the course that no barcode sits at the
  same point along the track as a hump.
