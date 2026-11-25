# Buddy 5 recorded obstacle results

Written 25 November 2026 from logs already recorded, 10 to 19 November. These
builds are older than `main` at `d1e2871`: the scan, exit check and rejoin check
have changed since, so this is not the final firmware's performance. Exact
commits were not recorded.

Logs: [10 and 11 November](logs/2026-11-10_11_b5_results.txt) (result lines only),
[19 November 13:54](../integration/logs/2026-11-19_car_run_1354.txt) and
[15:01](../integration/logs/2026-11-19_car_run_1501.txt).

## Outcomes

19 encounters, after removing one repeated line:

| Outcome | Count | Meaning |
|---|---|---|
| `REJOINED` | 5 | Bypassed and found the line again |
| `CLEARED` | 4 | Nothing found ahead after the stop |
| `ABORTED` | 10 | The run left AUTO first: a stop, fault or end of test |

These were tuning sessions, so this is not a success rate. `REJOINED` on these
builds meant line contact; `main` also requires wheel progress. The 10 and 11
November entries may include table tests, where line sensing is ignored; the
build type is not in the extract.

## Rejoined runs

| Date | Width | Edges | Side | Pass ended by | Side distance | Depth | Scan | Total |
|---|---|---|---|---|---|---|---|---|
| 10 Nov | 160 mm | both | left | not logged | - | - | 3.4 s | 15.1 s |
| 10 Nov | 164 mm | both | right | not logged | - | - | 3.4 s | 22.1 s |
| 11 Nov | 87 mm | both | left | side sonar | 109 mm | 6 mm | 3.0 s | 24.7 s |
| 19 Nov | - | left only | left | side not seen | - | - | 7.7 s | 21.1 s |
| 19 Nov | 163 mm | both | left | side sonar | 80 mm | 126 mm | 3.6 s | 24.2 s |

- Four of the five had a two-edge width. The boxes were not measured with a
  ruler, so width error is unknown.
- Depth is a lower bound. 6 mm is too small to be a real box depth.
- The bypass takes 15 to 25 s, against a team goal of 5 s. `b5_metrics.py`
  grades the 24.2 s run F for time.

## All encounters with two edges

Widths were 32, 69, 86, 87, 99, 124, 160, 163, 164, 182, 196 and 224 mm. The box
used in each run is not recorded, so the spread cannot be separated from
different boxes. The 32 mm one had an edge placed at 0°, where the fixed 12°
beam correction shrinks the box too far.

## Against the write-up

| Item | Evidence |
|---|---|
| Location, nearest point, width, clearances | In the `[B5 PROFILE]` lines |
| Turn left, turn right, reverse, continue | Both sides chosen; two encounters reversed; four found the way clear |
| Return to the line | Five rejoined runs, plus E3 in the [ledger](../DEMO_EVIDENCE.md) |

## Not measured

Detection rate, width error, minimum clearance, boxed-in recovery, and direction
choice with one side deliberately blocked (S5-M01 to S5-M03).
