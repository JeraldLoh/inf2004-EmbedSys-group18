"""Wheel-speed summary from the 25 November wireless [B2 TRACK] samples.

Usage, from anywhere:
    python testing/subsystem2/wireless_track_analysis.py

Reads testing/integration/logs/2026-11-25_wireless_track.csv (moving samples
only). Samples are about 50 ms apart on a 100 ms speed measurement, and missing
wireless batches leave gaps.
"""
import collections
import csv
import os
import statistics

CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                   "integration", "logs", "2026-11-25_wireless_track.csv")
HOLD_MS = 200
GAP_MS = 150


def load():
    rows, prev = [], None
    with open(CSV, encoding="utf-8", newline="") as handle:
        for raw in csv.DictReader(handle):
            row = {"session": raw["session"], "fw": raw["firmware"],
                   "t": int(raw["t_ms"]),
                   "target": (int(raw["left_target"]),
                              int(raw["right_target"])),
                   "speed": (int(raw["left_measured"]),
                             int(raw["right_measured"])),
                   "valid": raw["speed_valid"] == "1",
                   "effort": (int(raw["left_effort"]),
                              int(raw["right_effort"]))}
            row["since"] = row["t"]
            if prev and prev["session"] == row["session"] \
                    and 0 <= row["t"] - prev["t"] <= GAP_MS \
                    and prev["target"] == row["target"]:
                row["since"] = prev["since"]
            rows.append(row)
            prev = row
    return rows


def report(name, rows):
    for wheel, label in enumerate(("left", "right")):
        err = [r["speed"][wheel] - abs(r["target"][wheel]) for r in rows]
        if err:
            print(f"{name} | {label} | n={len(err)} "
                  f"bias={statistics.mean(err):+.1f} "
                  f"mean_abs={statistics.mean(abs(e) for e in err):.1f} "
                  f"rms={(sum(e * e for e in err) / len(err)) ** 0.5:.1f} "
                  f"max={max(abs(e) for e in err)}")


def main():
    rows = load()
    valid = [r for r in rows if r["valid"]]
    print("moving samples", len(rows), "speed valid", len(valid))
    held = [r for r in valid if r["target"] == (80, 76)
            and r["t"] - r["since"] >= HOLD_MS]
    report("80/76 held >= 200 ms", held)
    for wheel, label in enumerate(("left", "right")):
        print("  effort", label,
              round(statistics.mean(r["effort"][wheel] for r in held)))
    within = [sum(abs(r["speed"][w] - r["target"][w]) <= 8 for r in held)
              / len(held) for w in (0, 1)]
    print("  share within 8 mm/s", [f"{v:.0%}" for v in within])
    by_fw = collections.defaultdict(list)
    for r in held:
        by_fw[r["fw"]].append(r)
    for fw, group in by_fw.items():
        report("  " + fw, group)
    report("forward, unequal targets",
           [r for r in valid if min(r["target"]) >= 10
            and abs(r["target"][0] - r["target"][1]) > 5])
    report("pivot (opposite targets)",
           [r for r in valid if r["target"][0] * r["target"][1] < 0
            and min(abs(v) for v in r["target"]) >= 10])


if __name__ == "__main__":
    main()
