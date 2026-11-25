# Buddy 4 terrain analysis report

Written 25 November 2026 from source on `main` at `d1e2871`, a fresh run of the
host tests, and logs already recorded. No new car test was run for it.

## Method

The LSM303DLHC is read every 20 ms. After a level calibration, pitch and roll
give the height of a raised wheel:

raised height = caster arm x sin(pitch) + (track / 2) x sin(roll)

Height is also the sum of sin(pitch) x distance, using Buddy 2's odometry. Each
hump is classed as one-wheel (height from roll) or full-width (the larger of the
attitude and integrated heights). A result is valid only if the car rose and
came back to the floor, the height is at least 5 mm, the length is 80 to 1500 mm,
and no fault occurred. The state machine runs LEVEL, ASCENDING, CREST,
DESCENDING, SETTLING. Details: [Buddy 4 README](../../app_program/subsystem4/README.md).

## Host simulation

`terrain_demo_baseline_tests` and `terrain_demo_jolt_pause_tests` both pass on a
clean copy of `main`, run on 25 November 2026:

| Test | Result |
|---|---|
| 10 mm x 195 mm ramp under one wheel, 21 cases | 8 to 12 mm; width 195 ±20 mm |
| 10 mm full-width hump | One valid result, 8 to 12 mm |
| Flat floor, 5 min at 30 mg vibration | No hump episode |
| Floor tilted 1.5° for 60 s | No valid false hump |
| Lifts of 23 to 30 cm at different tilts | End of run in 0.46 to 0.60 s |
| 12 cm lift, 25° tilt, 1.4 g knock | No trigger |
| Turned over 120° | End of run, overturned |

The simulation has a 10 mm ramp only. It has no 5 mm case, and 5 mm is exactly
the smallest height that counts as valid.

## Humps used

The team reports two nominal humps, **5 mm and 10 mm**, used alternately with
the 10 mm one first. These are nominal heights. No ruler or caliper reading was
recorded, and no saved log marks which hump the car was crossing.

## Logged results, 19 November 2026

All 11 hump results in the [15:01 log](../integration/logs/2026-11-19_car_run_1501.txt):

| Host time | Reported | Length | Valid | Car was |
|---|---|---|---|---|
| 15:01:35 | 3 ±4 mm | 15 cm | No | in an obstacle bypass |
| 15:01:40 | 5 ±6 mm | 10 cm | Yes | in an obstacle bypass |
| 15:01:46 | 6 ±3 mm | 5 cm | No | just after rejoining the line |
| 15:03:09 | 6 ±7 mm | 40 cm | Yes | in an obstacle bypass |
| 15:03:13 | 5 ±3 mm | 5 cm | No | in an obstacle bypass |
| 15:03:16 | 7 ±7 mm | 40 cm | Yes | in an obstacle bypass |
| 15:03:19 | 4 ±3 mm | 2 cm | No | in an obstacle bypass |
| 15:03:21 | 4 ±4 mm | 10 cm | No | in an obstacle bypass |
| 15:03:22 | 6 ±3 mm | 5 cm | No | in an obstacle bypass |
| 15:03:27 | 5 ±3 mm | 5 cm | No | just after rejoining the line |
| 15:03:28 | 4 ±3 mm | 2 cm | No | at the start of a second scan |

- 3 of 11 are valid, and all three were reported during a bypass. Starting,
  stopping and turning read as pitch, so these may be false detections.
- The uncertainty of those three (±6 to ±7 mm) is about as large as the height.
- They do not match a 10 mm hump followed by a 5 mm one, so **no reading is
  paired with a hump and no accuracy figure is given.**
- Evidence [E4](../DEMO_EVIDENCE.md) records one valid 9 ±8 mm result on a later
  build. Its hump height is also not recorded. The raw file was not available
  for this report.

In all, the validity checks rejected 8 of the 11 results, but 3 still passed
while the car was manoeuvring.

## Other observations

- Processing takes 2.2 to 5.3 ms per 20 ms sample, with no I2C errors in the
  status lines reviewed.
- At start-up, level calibration rejected windows with more than 40 mg of
  movement and retried, as designed.
- Pickup stops were logged (E2 to E4). Lift height was not compared with a ruler.
- Faults show 0x40 in the 15:01 log's first boot. That bit was the collision
  latch on that build and is reserved on `main`.

## Highest hump in a run

The firmware reports each hump. The dashboard lists them from START to the end
of the run. The tallest is read from that list; it is not calculated.

## Limits and what would close them

- Measured hump heights with at least three passes each, marked in the log.
- A flat control run and a bypass with no hump, to count false detections.
- The caster arm and track were not confirmed against measured humps.
- The magnetometer cannot be calibrated in this build, so heading and turn rate
  are indicative only. The sensor has no gyroscope.
- Collision detection was dropped. The Buddy 4 README records this as the
  lecturer's decision of 11 November 2026.
