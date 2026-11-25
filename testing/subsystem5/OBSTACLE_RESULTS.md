# Buddy 5 recorded obstacle results

Written 25 November 2026 from logs already recorded, 10 to 25 November. None is
from the published image on `main` (`compact-data`), so this is not the final
firmware's performance. Exact commits were not recorded.

Logs: [10 and 11 November](logs/2026-11-10_11_b5_results.txt) (result lines only),
[19 November 13:54](../integration/logs/2026-11-19_car_run_1354.txt) and
[15:01](../integration/logs/2026-11-19_car_run_1501.txt), and
[25 November wireless events](../integration/logs/2026-11-25_wireless_events.txt).

## 25 November 2026

20 encounters with a result line, 03:16 to 11:40 SGT. Encounters whose line was
lost over Wi-Fi are not counted.

| Outcome | Count |
|---|---|
| `REJOINED`: tracking handoff and wheel progress confirmed | 3 |
| `REJOIN_UNVERIFIED`: back on the line, progress not confirmed | 2 |
| `ABORTED`: the run left AUTO first (pickup, STOP or fault) | 15 |

| SGT, build | Outcome | Width | Edges | Side | Pass | Scan | Total |
|---|---|---|---|---|---|---|---|
| 03:52 `adcpoll-retry80` | Unverified | - | right only | right | side sonar | 4.7 s | 15.7 s |
| 04:27 `retry80-closing-bars` (E3) | Rejoined | - | right only | right | side sonar | 5.5 s | 13.2 s |
| 10:47 `b5-sweep` | Unverified | - | left only | left | side sonar | 15.3 s | 28.4 s |
| 10:50 `b5-sweep` | Rejoined | 54 mm | both | right | side sonar | 6.2 s | 17.6 s |
| 11:38 `encoder-grace` | Rejoined | - | left only | left | side not seen | 9.2 s | 29.5 s |

- Both sides were chosen and rejoined. Four of these five had only one edge.
- Ten encounters had a two-edge width: 10, 18, 22, 45, 45, 54, 84, 140, 167 and
  243 mm. The boxes were not measured, but a spread this wide, and 10 and 18 mm
  boxes, mean the width is not reliable on these builds.
- The scan alone took 3.3 to 15.7 s, longest after a reverse and rescan.
- Two aborted encounters ended within 1 s of detection, before a scan.

## 10 to 19 November 2026

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

### Rejoined runs, 10 to 19 November

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

### All encounters with two edges, 10 to 19 November

Widths were 32, 69, 86, 87, 99, 124, 160, 163, 164, 182, 196 and 224 mm. The box
used in each run is not recorded, so the spread cannot be separated from
different boxes. The 32 mm one had an edge placed at 0°, where the fixed 12°
beam correction shrinks the box too far.

## Against the write-up

| Item | Evidence |
|---|---|
| Location, nearest point, width, clearances | In the `[B5 PROFILE]` lines |
| Turn left, turn right, reverse, continue | Both sides chosen; two encounters reversed; four found the way clear |
| Return to the line | Eight rejoined runs, three of them confirmed by tracking and wheel progress |

## Not measured

Detection rate, width error, minimum clearance, boxed-in recovery, and direction
choice with one side deliberately blocked (S5-M01 to S5-M03).
