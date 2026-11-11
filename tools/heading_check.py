#!/usr/bin/env python3
"""Compare the IMU compass heading with the heading the wheels imply.

Usage: python tools/heading_check.py run.log [--track-mm 112]

Needs a build with HEADING_LOG=1 so the log has both
    [B4 HDG] t_ms=.. valid=.. hdg_ddeg=.. ...
    [B2 TRACK] ... t_ms=.. ... measured=L/R ...
Wheel heading is the integral of (right - left) / track_width. If the compass is
trustworthy while the car drives, its heading CHANGES should follow the wheel
heading changes. If they disagree, the magnetometer is being disturbed (motors,
wires) and must not be used to steer.
"""
import argparse
import math
import re

HDG = re.compile(r"\[B4 HDG\]\s+t_ms=(-?\d+)\s+valid=(\d)\s+hdg_ddeg=(-?\d+)")
TRK = re.compile(r"\[B2 TRACK\].*?t_ms=(\d+) steering=(\w+).*?measured=(\d+)/(\d+)")


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--track-mm", type=float, default=112.0)
    args = ap.parse_args()

    hdg, trk = [], []
    for line in open(args.log, errors="replace"):
        m = HDG.search(line)
        if m:
            hdg.append((int(m.group(1)), int(m.group(2)), int(m.group(3)) / 10.0))
            continue
        m = TRK.search(line)
        if m and m.group(2) == "digital":
            trk.append((int(m.group(1)), int(m.group(3)), int(m.group(4))))
    if len(hdg) < 5 or len(trk) < 5:
        print(f"Not enough data: {len(hdg)} heading lines, {len(trk)} tracking lines. "
              "Is this a HEADING_LOG=1 build and did the car drive?")
        return

    # Wheel heading (deg, + = left turn): integrate (R - L)/track over time.
    wheel = []
    yaw = 0.0
    for i, (t, l, r) in enumerate(trk):
        if i:
            dt = (t - trk[i - 1][0]) / 1000.0
            if 0 < dt < 0.5:
                yaw += math.degrees((r - l) / args.track_mm * dt)
        wheel.append((t, yaw))

    def wheel_at(t):
        best = min(wheel, key=lambda p: abs(p[0] - t))
        return best[1] if abs(best[0] - t) < 300 else None

    pairs = []
    prev = None
    comp = 0.0
    for t, valid, h in hdg:
        w = wheel_at(t)
        if not valid or w is None:
            prev = None
            continue
        if prev is not None:
            comp += wrap(h - prev)
        prev = h
        pairs.append((t, comp, w))
    if len(pairs) < 5:
        print("Compass heading was not valid while the car was tracking.")
        return

    c0, w0 = pairs[0][1], pairs[0][2]
    errs = [(c - c0) - (w - w0) for _, c, w in pairs]
    dc = [b[1] - a[1] for a, b in zip(pairs, pairs[1:])]
    dw = [b[2] - a[2] for a, b in zip(pairs, pairs[1:])]
    n = len(dc)
    mc, mw = sum(dc) / n, sum(dw) / n
    cov = sum((a - mc) * (b - mw) for a, b in zip(dc, dw))
    vc = sum((a - mc) ** 2 for a in dc) ** 0.5
    vw = sum((b - mw) ** 2 for b in dw) ** 0.5
    corr = cov / (vc * vw) if vc and vw else float("nan")
    print(f"samples={len(pairs)}  span={(pairs[-1][0] - pairs[0][0]) / 1000:.1f} s")
    print(f"compass swing: {min(c - c0 for _, c, _ in pairs):.1f} .. {max(c - c0 for _, c, _ in pairs):.1f} deg")
    print(f"wheel   swing: {min(w - w0 for _, _, w in pairs):.1f} .. {max(w - w0 for _, _, w in pairs):.1f} deg")
    print(f"correlation of heading changes (1 = they move together): {corr:.2f}")
    print(f"max |compass - wheel| disagreement: {max(abs(e) for e in errs):.1f} deg;  "
          f"end drift: {errs[-1]:.1f} deg")
    if corr == corr and corr > 0.7 and max(abs(e) for e in errs) < 15:
        print("Verdict: compass tracks the wheels well enough to consider as a gentle assist.")
    else:
        print("Verdict: compass and wheels disagree. Do not steer from the magnetometer yet.")


if __name__ == "__main__":
    main()
