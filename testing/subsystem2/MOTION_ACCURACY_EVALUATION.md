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
| U-turn +5% | 197,316 instead of 187,920 millidegrees | One completed on `retry80-uturn-plus5`, 25 November 09:30 SGT. Angle not measured. |

Barcode turns in the [25 November wireless logs](../integration/logs/2026-11-25_wireless_events.txt), 03:16 to 11:40 SGT:

| Turn | Barcode routes decoded | Turns completed |
|---|---|---|
| Left | 7 | 7 |
| Right | 3 | 3 |
| U-turn | 6 | 6 |

"Completed" is the controller's junction count. It shows the car's own move
finished, and not the angle or that the car left on the right branch. Missing
wireless batches may hide failures, so 16 of 16 is not a success rate. 82 further
junctions were crossed straight, which is also the default with no barcode.
They come from twelve builds whose firmware IDs run from `pivot90` to
`encoder-grace`. The ID on `main`, `group18-combined-compact-data-2026-11-25`,
is not among them, so none is the published image.

Buddy 4's `turned ...` lines in the
[15:01 log](../integration/logs/2026-11-19_car_run_1501.txt) range from 0 to
64°. They are not used here: that heading comes from an uncalibrated
magnetometer, and the commanded angle of each turn is not in the log. The wireless logs agree that it is
not usable: a left turn that the controller completed read 61°, and the +5% U-turn
147°.

## Speed

Steady cruise at 80/76 mm/s: mean error -6.2 mm/s left and +0.7 mm/s right on 19
November (90 samples), -1.6 and +4.3 mm/s on 25 November (3,906 samples). See the [PID tuning report](PID_TUNING_REPORT.md).

## Against the write-up's objectives

| Objective | What the records show |
|---|---|
| Consistent speed | On 25 November, 89% of left and 80% of right cruise samples within 8 mm/s, by encoder |
| Accurate distance | Within 4 mm by encoder; true floor error not measured |
| Repeatable turns | 16 barcode turns completed on 25 November; angles not measured |
| Stable motion | Line following observed on straights and bends (E1). 14 stops are in the 25 November logs: 10 encoder-quality, 4 stalls. |

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
