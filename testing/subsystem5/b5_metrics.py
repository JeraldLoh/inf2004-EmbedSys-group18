"""Buddy 5 Week 10 metrics from a CAR_RUN console log.

The firmware prints one line per obstacle encounter (subsystem5.c; wrapped
here):

  [B5 RESULT] n=3 outcome=REJOINED ms=9120 scan_ms=2410 ahead_mm=181 width_mm=96
      edges=both centre_x_mm=-4 side=LEFT lclear_mm=4000 rclear_mm=612
      aligned=1 backoffs=0 reverses=0 reacq=0 pass=clear side_mm=152 depth_mm=98
      reason=""

Usage:
  python b5_metrics.py run.log                 # summary from the log alone
  python b5_metrics.py run.log trials.csv      # + detection, width, side vs truth

trials.csv has one row per obstacle placed, filled in during the run:
  trial,true_width_mm,blocked_side,n
  1,95,right,1
  2,95,none,          <- car did not detect it (no RESULT line): leave n empty
blocked_side is the side you blocked (left/right) or none. n is the n= of the
RESULT line for that trial.

Grades follow the page 8 "Week 10 demonstration metrics" table.
"""

import csv
import re
import sys

LINE = re.compile(r"\[B5 RESULT\]\s+(.*)")
FIELD = re.compile(r'(\w+)=("[^"]*"|\S+)')

# (A, B, C, D) thresholds; anything worse is F.
RATE_BANDS = (100.0, 95.0, 90.0, 80.0)        # higher is better, percent
WIDTH_BANDS = (10.0, 20.0, 30.0, 40.0)        # lower is better, percent error
TIME_BANDS = (3.0, 5.0, 7.0, 10.0)            # lower is better, seconds


def grade_high(value, bands):
    for letter, limit in zip("ABCD", bands):
        if value >= limit:
            return letter
    return "F"


def grade_low(value, bands, strict=False):
    for letter, limit in zip("ABCD", bands):
        if value < limit or (not strict and value <= limit):
            return letter
    return "F"


def parse_log(path):
    results = {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            m = LINE.search(raw)
            if not m:
                continue
            r = {k: v.strip('"') for k, v in FIELD.findall(m.group(1))}
            results[int(r["n"])] = r
    return results


def num(r, key):
    v = r.get(key, "-")
    return None if v in ("-", "") else int(v)


def pct(part, whole):
    return 100.0 * part / whole if whole else 0.0


def main(argv):
    if len(argv) not in (2, 3):
        print(__doc__)
        return 2
    results = parse_log(argv[1])
    if not results:
        print("no [B5 RESULT] lines in", argv[1])
        return 1

    rows = []
    print(f"{len(results)} encounters in {argv[1]}")
    by_outcome = {}
    for r in results.values():
        by_outcome[r["outcome"]] = by_outcome.get(r["outcome"], 0) + 1
    print("outcomes:", ", ".join(f"{k} {v}" for k, v in sorted(by_outcome.items())))

    # Line reacquisition: every encounter that left the line (chose a side).
    bypassed = [r for r in results.values() if r.get("side") in ("LEFT", "RIGHT")]
    rejoined = [r for r in bypassed if r["outcome"] == "REJOINED"]
    if bypassed:
        rate = pct(len(rejoined), len(bypassed))
        rows.append(("Successful line reacquisition", f"{len(rejoined)}/{len(bypassed)} = {rate:.0f}%",
                      grade_high(rate, RATE_BANDS)))
        aligned = sum(1 for r in rejoined if r.get("aligned") == "1")
        print(f"rejoins that ran the line-align step: {aligned}/{len(rejoined)}")
        passes = [r.get("pass", "off") for r in bypassed]
        if any(p != "off" for p in passes):
            print(f"pass legs ended by the side sonar: {passes.count('clear')}/{len(passes)}"
                  f" (capped {passes.count('cap')})")
        depths = [num(r, "depth_mm") for r in bypassed if num(r, "depth_mm") is not None]
        if depths:
            print(f"measured depth (lower bound): mean {sum(depths) / len(depths):.0f} mm,"
                  f" range {min(depths)}..{max(depths)} mm (n={len(depths)})")

    # Time to bypass: detection -> back on the line, for successful bypasses.
    if rejoined:
        secs = sorted(int(r["ms"]) / 1000.0 for r in rejoined)
        mean = sum(secs) / len(secs)
        rows.append(("Time to bypass obstacle",
                     f"mean {mean:.1f} s, worst {secs[-1]:.1f} s (n={len(secs)})",
                     grade_low(secs[-1], TIME_BANDS, strict=True)))

    if len(argv) == 3:
        with open(argv[2], newline="", encoding="utf-8") as f:
            trials = list(csv.DictReader(f))
        detected = [t for t in trials if (t.get("n") or "").strip()]
        rate = pct(len(detected), len(trials))
        rows.append(("Obstacle detection rate", f"{len(detected)}/{len(trials)} = {rate:.0f}%",
                     grade_high(rate, RATE_BANDS)))

        errors, sides_ok, sides = [], 0, 0
        for t in detected:
            r = results.get(int(t["n"]))
            if r is None:
                print(f"trial {t['trial']}: n={t['n']} not in the log")
                continue
            true_w = float(t["true_width_mm"]) if (t.get("true_width_mm") or "").strip() else 0.0
            width = num(r, "width_mm")
            if true_w > 0 and width is not None and r.get("edges") == "both":
                errors.append(abs(width - true_w) / true_w * 100.0)
            blocked = (t.get("blocked_side") or "none").strip().upper()
            if r.get("side") not in (None, "-") and r.get("outcome") != "CLEARED":
                sides += 1
                if r["side"] in ("LEFT", "RIGHT") and r["side"] != blocked:
                    sides_ok += 1
        if errors:
            worst = max(errors)
            rows.append(("Width estimation error",
                         f"mean {sum(errors) / len(errors):.0f}%, worst {worst:.0f}% (n={len(errors)})",
                         grade_low(worst, WIDTH_BANDS)))
        if sides:
            rate = pct(sides_ok, sides)
            rows.append(("Direction selection success", f"{sides_ok}/{sides} = {rate:.0f}%",
                         grade_high(rate, RATE_BANDS)))

    print()
    print(f"{'Test':32} {'Result':44} Grade")
    for name, value, grade in rows:
        print(f"{name:32} {value:44} {grade}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
