#!/usr/bin/env python3
"""Replay barcode captures logged by the car and report why each did/didn't decode.

Input: a serial log containing lines such as
    [S3 CAP] id=3 ok=0 n=18 pass=0 fail=3 us=413000,328000,...
Usage:
    python tools/barcode_replay.py run.log [more.log ...]
    python tools/barcode_replay.py run.log --expect F      # also score vs. expected letter

The decoder below mirrors app_program/subsystem3/barcode_decoder.c (single
Code 39 character framed by '*', 29 elements, both directions). Use it to see
how close a failed capture was (ratios, missing elements) without driving the car.
"""
import argparse
import re

CODE39 = {
    0x109: 'A', 0x049: 'B', 0x148: 'C', 0x019: 'D', 0x118: 'E', 0x058: 'F',
    0x00D: 'G', 0x10C: 'H', 0x04C: 'I', 0x01C: 'J', 0x103: 'K', 0x043: 'L',
    0x142: 'M', 0x013: 'N', 0x112: 'O', 0x052: 'P', 0x007: 'Q', 0x106: 'R',
    0x046: 'S', 0x016: 'T', 0x181: 'U', 0x0C1: 'V', 0x1C0: 'W', 0x091: 'X',
    0x190: 'Y', 0x0D0: 'Z', 0x034: '0', 0x121: '1', 0x061: '2', 0x160: '3',
    0x031: '4', 0x130: '5', 0x070: '6', 0x025: '7', 0x124: '8', 0x064: '9',
    0x085: '-', 0x184: '.', 0x0C4: ' ', 0x094: '*',
}
SYMBOL_RUNS = 29
WIDE_NUM, WIDE_DEN = 17, 10   # real wide element >= 1.7x narrow (per colour)

LINE = re.compile(r"\[S3 CAP\]\s+id=(\d+)\s+ok=(\d)\s+n=(\d+).*?us=([\d,]*)")


def limits(runs):
    low, high = [0, 0], [0, 0]
    for i in range(SYMBOL_RUNS):
        c, w = i & 1, runs[i]
        if low[c] == 0 or w < low[c]:
            low[c] = w
        high[c] = max(high[c], w)
    if (low[0] == 0 or low[1] == 0 or high[0] * WIDE_DEN < low[0] * WIDE_NUM
            or high[1] * WIDE_DEN < low[1] * WIDE_NUM):
        return None, low, high
    return ((low[0] + high[0]) // 2, (low[1] + high[1]) // 2), low, high


def pattern(runs, start, bl, sl, rev):
    bits = 0
    for k in range(9):
        r = start + k
        if rev:
            r = SYMBOL_RUNS - 1 - r
        bits = (bits << 1) | (1 if runs[r] > (bl if r % 2 == 0 else sl) else 0)
    return bits


def decode(runs):
    """Return (letter, reversed, reason). letter is None on failure."""
    if len(runs) < SYMBOL_RUNS:
        return None, False, f"only {len(runs)} of {SYMBOL_RUNS} elements"
    runs = runs[:SYMBOL_RUNS]
    lim, low, high = limits(runs)
    if lim is None:
        return None, False, f"no wide/narrow contrast (bars {low[0]}-{high[0]}us, spaces {low[1]}-{high[1]}us)"
    bl, sl = lim
    if runs[9] > sl or runs[19] > sl:
        return None, False, "inter-character gap not narrow"
    for rev in (False, True):
        s, d, e = (pattern(runs, o, bl, sl, rev) for o in (0, 10, 20))
        if any(bin(x).count("1") != 3 for x in (s, d, e)):
            continue
        if CODE39.get(s) == '*' and CODE39.get(e) == '*':
            ch = CODE39.get(d)
            if ch and ch != '*':
                return ch, rev, "ok"
    return None, False, "start/stop pattern invalid in both directions"


def parse(path):
    caps = []
    with open(path, errors="replace") as fh:
        for line in fh:
            m = LINE.search(line)
            if m:
                runs = [int(x) for x in m.group(4).split(",") if x]
                caps.append((int(m.group(1)), m.group(2) == "1", runs))
    return caps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--expect", help="expected letter, e.g. F")
    args = ap.parse_args()

    total = good = firm_ok = 0
    for path in args.logs:
        print(f"== {path}")
        for cid, fw_ok, runs in parse(path):
            letter, rev, why = decode(runs)
            total += 1
            right = letter is not None and (args.expect is None or letter == args.expect.upper())
            good += 1 if right else 0
            firm_ok += 1 if fw_ok else 0
            mark = "PASS" if right else "FAIL"
            detail = f"'{letter}'{' (rev)' if rev else ''}" if letter else why
            note = "" if fw_ok == (letter is not None) else "  [replay differs from firmware]"
            print(f"  #{cid:<3} {mark} n={len(runs):<2} fw_ok={int(fw_ok)} {detail}{note}")
            if not right and len(runs) < SYMBOL_RUNS:
                print("       widths(ms): " + " ".join(str(round(r / 1000)) for r in runs))
    if total:
        print(f"\ncaptures={total} replay_pass={good} ({100 * good / total:.0f}%) firmware_ok={firm_ok}")
        print("Note: short captures can be line-tape crossings, not barcode attempts.")
    else:
        print("No '[S3 CAP]' lines found. Is this a build with the capture trace?")


if __name__ == "__main__":
    main()
