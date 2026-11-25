# Buddy 2 motion accuracy evaluation

Written 25 November 2026 from results already recorded. No new car test was run
for it.

**Main limitation:** travel was not measured with a ruler and angles were not
measured with a protractor, with trial counts. The distances below come from
the car's own encoders, so they show how well the car reaches its own target
and not its true error on the floor.

## Distance

Standalone movement test V5, 2 November 2026:

| Command | Encoder result | Error | Percent | Result code |
|---|---|---|---|---|
| Forward 300 mm | 296 mm | -4 mm | -1.3% | 3, odometry valid |
| Forward 1000 mm | 996 mm | -4 mm | -0.4% | 3, odometry valid |
| Reverse 300 mm | -296 mm | 4 mm short | -1.3% | 3, odometry valid |

Every run stopped about 4 mm short, inside the profile's 5 mm wheel tolerance.
One result per command is recorded, so repeatability cannot be calculated. The
operator reported that 30 cm forward and backward travel was correct and later
repeated correctly. No readings were kept.

## Turning

| Build | Command | Recorded result |
|---|---|---|
| V5 standalone | 90° pivot | Stopped with a fault at 67/74 mm of 87 mm wheel travel: about 73° estimated, 75 to 80° by eye |
| V6 standalone | 90° pivots, left and right | Operator saw about 90° both ways, repeated "a few times". No angles or counts. |
| Combined, November | Barcode turns | Completed turns in evidence E3 and E4, no angles |
| Combined, November | U-turn | One completed and resumed, then a `MEASUREMENT_QUALITY` fault (E3) |
| U-turn +5% | 197,316 instead of 187,920 millidegrees | Built and host-tested. No hardware result. |

Buddy 4's `turned ...` lines in the
[15:01 log](../integration/logs/2026-11-19_car_run_1501.txt) range from 0 to
64°. They are not used here: that heading comes from an uncalibrated
magnetometer, and the commanded angle of each turn is not in the log.

## Speed

Steady cruise at 80/76 mm/s on 19 November: mean error -6.2 mm/s left and +0.7
mm/s right. See the [PID tuning report](PID_TUNING_REPORT.md).

## Against the write-up's objectives

| Objective | What the records show |
|---|---|
| Consistent speed | Steady cruise within about 8% on the encoder measurement; left wheel slow |
| Accurate distance | Within 4 mm by encoder; true floor error not measured |
| Repeatable turns | Operator observation only; not measured |
| Stable motion | Line following observed on straights and bends (E1). Wheel stalls and one encoder-quality stop are recorded. |

## Calibration used

Wheel diameter 65 mm, track 112 mm by ruler, 1271 counts per revolution left and
1264 right. The encoder records are in [TEST_LOG.md](TEST_LOG.md) and
[logs](logs/2026-10-22_encoder-ten-revolution-raw.txt).

## Not measured

- Ruler distance, with repeats, for forward and reverse moves.
- Protractor angle, with repeats, for 90° left, 90° right and 180°.
- Straight-line drift over 1 m.
- Any effect of battery charge or load.

Sources: [hardware evidence](B2_HARDWARE_EVIDENCE_2026-11-02.md) and
[evidence ledger](../DEMO_EVIDENCE.md).
