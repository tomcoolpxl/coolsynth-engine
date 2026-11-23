"""Measures a rendered TB-303 VCF (render.py's CSV of vcf.cir with ntp6 and nin, D275; TP6 is Q19's emitter): the ring after each of the
saw's resets, as page 7's filter adjustment reads it at TP6 (the period between the ring's peaks, 2 ms +- 0.5 ms
there), its decay from peak to peak, and TP6's swing.

Usage: python -I tools/tb303/inspect_vcf.py out.csv [--skip 0.05]
"""
import argparse
import csv
import math


def ring(t, v, start, end):
    """The ring's peaks between start and end: their times and heights above the window's mean."""
    window = [(a, b) for a, b in zip(t, v) if start <= a <= end]
    if len(window) < 5:
        return []
    mean = sum(b for _, b in window) / len(window)
    peaks = []
    # A peak is the highest point within 0.25 ms either side (the solver's own wiggles are far shorter).
    reach = 0.25e-3
    j0 = 0
    for i in range(len(window)):
        ti, vi = window[i]
        while window[j0][0] < ti - reach:
            j0 += 1
        j = j0
        best = True
        while j < len(window) and window[j][0] <= ti + reach:
            if window[j][1] > vi:
                best = False
                break
            j += 1
        if best and vi > mean and (not peaks or ti - peaks[-1][0] > reach):
            peaks.append((ti, vi - mean))
    # Keep only peaks that stand out: above a fifth of the largest.
    if not peaks:
        return []
    top = max(h for _, h in peaks)
    return [p for p in peaks if p[1] > 0.2 * top]


def measure(path, skip):
    with open(path, encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if float(r["time"]) >= skip]
    t = [float(r["time"]) for r in rows]
    tp6 = [float(r["ntp6"]) for r in rows]
    vin = [float(r["nin"]) for r in rows]
    edges = [t[i] for i in range(1, len(vin)) if vin[i] - vin[i - 1] > 2.0]
    periods, ratios = [], []
    for e in edges:
        peaks = ring(t, tp6, e + 0.1e-3, e + 10e-3)
        if len(peaks) >= 3:
            gaps = [b[0] - a[0] for a, b in zip(peaks[:4], peaks[1:5])]
            periods.append(sum(gaps) / len(gaps))
            ratios.append(peaks[1][1] / peaks[0][1])
    return {
        "edges": len(edges),
        "ring_ms": 1e3 * sum(periods) / len(periods) if periods else float("nan"),
        "decay": sum(ratios) / len(ratios) if ratios else float("nan"),
        "tp6": (min(tp6), max(tp6)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("--skip", type=float, default=0.05)
    args = parser.parse_args()
    m = measure(args.csv, args.skip)
    print("{} resets; ring period {:.3f} ms ({:.0f} Hz), each peak {:.2f} of the last; TP6 {:.3f} to {:.3f} V".format(
        m["edges"], m["ring_ms"], 1e3 / m["ring_ms"] if m["ring_ms"] == m["ring_ms"] else math.nan, m["decay"],
        m["tp6"][0], m["tp6"][1]))


if __name__ == "__main__":
    main()
