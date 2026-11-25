"""Wheel-speed summary and plot from saved [B2 TRACK] console lines.

Usage, from anywhere:
    python testing/subsystem2/pid_log_analysis.py

Reads the saved 2026-11-19 logs, prints the PID_TUNING_REPORT tables and
rewrites testing/subsystem2/pid_tracking_2026-11-19.svg.

[B2 TRACK] lines are about 50 ms apart and the speed estimate uses a 100 ms
window, so consecutive samples are correlated. Wheel speeds are magnitudes.
"""
import os
import re
import statistics

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
LOGS = ("testing/integration/logs/2026-11-19_car_run_1354.txt",
        "testing/integration/logs/2026-11-19_car_run_1501.txt")
PLOT = os.path.join(ROOT, "testing/subsystem2/pid_tracking_2026-11-19.svg")
TRACK = re.compile(
    r"\[B2 TRACK\] fw=(\S+) t_ms=(\d+) steering=(\S+) curve=(-?\d+) "
    r"target=(-?\d+)/(-?\d+) measured=(\d+)/(\d+) mm/s speed_valid=(\d) "
    r"effort=(-?\d+)/(-?\d+) output_valid=(\d) floor_mask=0x\w+ "
    r"state=(\d+)")
HOLD_MS = 200
GAP_MS = 150


def load(path):
    """Samples in order; 'since' is when the current target was first seen."""
    rows, prev = [], None
    with open(os.path.join(ROOT, path), encoding="utf-8") as handle:
        for line in handle:
            found = TRACK.search(line)
            if not found:
                continue
            g = found.groups()
            row = {"t": int(g[1]), "target": (int(g[4]), int(g[5])),
                   "speed": (int(g[6]), int(g[7])), "valid": g[8] == "1",
                   "effort": (int(g[9]), int(g[10])), "since": int(g[1])}
            if prev and 0 <= row["t"] - prev["t"] <= GAP_MS \
                    and row["target"] == prev["target"]:
                row["since"] = prev["since"]
            rows.append(row)
            prev = row
    return rows


def summary(rows, wheel):
    err = [r["speed"][wheel] - abs(r["target"][wheel]) for r in rows]
    if not err:
        return None
    return (len(err), statistics.mean(err),
            statistics.mean(abs(e) for e in err),
            (sum(e * e for e in err) / len(err)) ** 0.5,
            max(abs(e) for e in err))


def plot(rows):
    """Plot the longest continuous stretch of forward driving."""
    best, cur = [], []
    for row in rows:
        if cur and not 0 < row["t"] - cur[-1]["t"] <= GAP_MS:
            cur = []
        if min(row["target"]) <= 0 or not row["valid"]:
            cur = []
            continue
        cur = cur + [row]
        if len(cur) > len(best):
            best = cur
    t0, t1 = best[0]["t"], best[-1]["t"]
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="560" '
           'font-family="sans-serif" font-size="12">'
           '<rect width="900" height="560" fill="#fff"/>']
    for wheel, name in enumerate(("Left", "Right")):
        top = 40 + wheel * 260

        def px(t):
            return 60 + 810 * (t - t0) / (t1 - t0)

        def py(v, base=top):
            return base + 200 - 200 * v / 160

        out.append(f'<text x="60" y="{top - 8}">{name} wheel (mm/s): '
                   'target black, measured blue</text>')
        for v in range(0, 161, 40):
            out.append(f'<line x1="60" x2="870" y1="{py(v):.1f}" '
                       f'y2="{py(v):.1f}" stroke="#ddd"/>'
                       f'<text x="28" y="{py(v) + 4:.1f}">{v}</text>')
        for key, colour in (("target", "#000"), ("speed", "#1667c0")):
            pts = " ".join(f'{px(r["t"]):.1f},{py(abs(r[key][wheel])):.1f}'
                           for r in best)
            out.append(f'<polyline fill="none" stroke="{colour}" '
                       f'stroke-width="1.5" points="{pts}"/>')
    out.append(f'<text x="60" y="540">Uptime {t0 / 1000:.1f} s to '
               f'{t1 / 1000:.1f} s, {len(best)} samples, 2026-11-19 15:01 '
               'log</text></svg>')
    with open(PLOT, "w", encoding="utf-8", newline="\n") as handle:
        handle.write("\n".join(out) + "\n")
    print("plot", t0, t1, len(best))


def main():
    logs = {path: load(path) for path in LOGS}
    everything = [r for rows in logs.values() for r in rows]
    valid = [r for r in everything if r["valid"]]
    print("samples", {p: len(r) for p, r in logs.items()},
          "speed valid", len(valid))
    cruise = [r for r in valid if r["target"] == (80, 76)]
    groups = (
        ("80/76 held >= 200 ms",
         [r for r in cruise if r["t"] - r["since"] >= HOLD_MS]),
        ("80/76 all", cruise),
        ("forward, unequal targets",
         [r for r in valid if min(r["target"]) >= 10
          and abs(r["target"][0] - r["target"][1]) > 5]),
        ("pivot (opposite targets)",
         [r for r in valid if r["target"][0] * r["target"][1] < 0
          and min(abs(v) for v in r["target"]) >= 10]))
    for name, rows in groups:
        for wheel, label in enumerate(("left", "right")):
            got = summary(rows, wheel)
            if got:
                print(f"{name} | {label} | n={got[0]} bias={got[1]:+.1f} "
                      f"mean_abs={got[2]:.1f} rms={got[3]:.1f} max={got[4]}")
    for wheel, label in enumerate(("left", "right")):
        rows = groups[0][1]
        if rows:
            print("held cruise effort", label,
                  round(statistics.mean(r["effort"][wheel] for r in rows)))
    print("largest effort (per mille)",
          max(max(abs(v) for v in r["effort"]) for r in valid))
    plot(logs[LOGS[1]])


if __name__ == "__main__":
    main()
