#!/usr/bin/env python3
"""Count left/right line corrections in a car serial log.

Usage: python tools/weave_stats.py run.log [more.log]
Reads "[B2 TRACK] ... curve=N ... t_ms=T" lines. A correction starts when curve
goes from 0 to non-zero (negative = steer left, positive = steer right). A car
that drifts one way shows almost only one sign; a balanced car shows both.
Also reports wheel-speed swing while tracking.
"""
import re, sys
TR = re.compile(r"\[B2 TRACK\].*?t_ms=(\d+) steering=(\w+) curve=(-?\d+) target=(\d+)/(\d+) measured=(\d+)/(\d+)")
for path in sys.argv[1:]:
    left = right = 0
    starts = []
    prev = 0
    tmin = tmax = None
    swing = []
    for line in open(path, errors="replace"):
        m = TR.search(line)
        if not m or m.group(2) != "digital":
            continue
        t, curve = int(m.group(1)), int(m.group(3))
        ml, mr = int(m.group(6)), int(m.group(7))
        swing.append(abs(ml - mr))
        tmin = t if tmin is None else tmin
        tmax = t
        if curve != 0 and prev == 0:
            starts.append(t)
            if curve < 0: left += 1
            else: right += 1
        prev = curve
    secs = ((tmax - tmin) / 1000.0) if tmin is not None else 0
    n = left + right
    print(f"== {path}: tracking {secs:.1f} s")
    print(f"  corrections: left={left} right={right}  ({n / secs:.2f}/s)" if secs else "  no tracking lines")
    if n:
        print(f"  one-sidedness: {100 * max(left, right) / n:.0f}% toward the {'left' if left >= right else 'right'}"
              " (50% = balanced; ~100% = steady drift to the other side)")
    if swing:
        s = sorted(swing)
        print(f"  |L-R| wheel speed: median {s[len(s)//2]} mm/s, 90th pct {s[int(len(s)*0.9)]} mm/s")
