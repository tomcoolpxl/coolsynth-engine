"""Measures a rendered TB-303 VCO (render.py's CSV, D275): frequency, the saw's top and bottom, the square's levels and
duty, the reset's length, over the time after --skip seconds.

Usage: python -I tools/tb303/inspect_vco.py out.csv [--skip 0.1]
"""
import argparse
import csv


def crossings(times, values, level, rising):
    """Times where values cross level, linearly interpolated."""
    out = []
    for i in range(1, len(values)):
        a, b = values[i - 1], values[i]
        if (a < level <= b) if rising else (a > level >= b):
            out.append(times[i - 1] + (level - a) / (b - a) * (times[i] - times[i - 1]))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("--skip", type=float, default=0.1)
    args = parser.parse_args()
    with open(args.csv, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    rows = [r for r in rows if float(r["time"]) >= args.skip]
    t = [float(r["time"]) for r in rows]
    saw = [float(r["nsaw"]) for r in rows]
    sq = [float(r["nsq"]) for r in rows]
    c33 = [float(r["nc33"]) for r in rows]

    top, bottom = max(c33), min(c33)
    middle = 0.5 * (top + bottom)
    # The reset is the fast rise; a period from one reset to the next.
    rises = crossings(t, c33, middle, True)
    periods = [b - a for a, b in zip(rises, rises[1:])]
    if not periods:
        print("no full period after", args.skip)
        return
    period = sum(periods) / len(periods)
    # The reset's length: the time C33 spends rising from 10 % to 90 % of its swing.
    low10, high90 = bottom + 0.1 * (top - bottom), bottom + 0.9 * (top - bottom)
    r10 = crossings(t, c33, low10, True)
    r90 = crossings(t, c33, high90, True)
    resets = [b - a for a in r10 for b in r90 if 0 < b - a < 0.5 * period][:20]

    sq_high, sq_low = max(sq), min(sq)
    sq_mid = 0.5 * (sq_high + sq_low)
    up = crossings(t, sq, sq_mid, True)
    down = crossings(t, sq, sq_mid, False)
    highs = []
    for u in up:
        later = [d for d in down if d > u]
        if later and later[0] - u < period:
            highs.append(later[0] - u)
    duty = (sum(highs) / len(highs) / period) if highs else float("nan")

    print("frequency {:.4f} Hz (period {:.4f} ms, {} periods)".format(1.0 / period, 1e3 * period, len(periods)))
    print("C33 {:.3f} V to {:.3f} V; the saw (Q28's source) {:.3f} V to {:.3f} V".format(bottom, top, min(saw), max(saw)))
    if resets:
        print("reset (10 to 90 %) {:.2f} us".format(1e6 * sum(resets) / len(resets)))
    print("square {:.3f} V to {:.3f} V, high {:.1f} % of the period".format(sq_low, sq_high, 100.0 * duty))


if __name__ == "__main__":
    main()
