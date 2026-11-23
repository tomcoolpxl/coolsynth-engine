"""Compares a rendered TB-303 voice note with one of Din Sync's reference recordings of a real TB-303 (D275): the
fundamental, and harmonics 1 to 12 in dB against the fundamental, over the same window after the note's start.

The recordings (dinsync.info, 2010; no licence stated) are kept outside the repository; pass their folder.
Each SET-xx.wav holds 16 low C notes, 3 s apart from 1.14 s: per knob position, an unaccented saw, an accented saw,
an unaccented square, an accented square.

Usage: python -I tools/tb303/compare_dinsync.py <folder> SET-A1 13 sim.csv [--t0 0.02] [--periods 8] [--ton 0.01]
Start the note late (ton 0.4 s): C11 needs some 0.2 s to settle, and a real 303's VCO never stops.
  sim.csv: render.py's CSV of voice.cir with nout, the note's gate rising at ton (default 10 ms).
"""
import argparse
import csv
import math
import os
import struct
import wave


def goertzel(x, rate, f):
    """The amplitude of frequency f in x, Hann-windowed."""
    n = len(x)
    w = [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]
    c = 2 * math.cos(2 * math.pi * f / rate)
    s1 = s2 = 0.0
    for i in range(n):
        s0 = x[i] * w[i] + c * s1 - s2
        s2, s1 = s1, s0
    re = s1 - s2 * math.cos(2 * math.pi * f / rate)
    im = s2 * math.sin(2 * math.pi * f / rate)
    return 2 * math.hypot(re, im) / sum(w)


def fundamental(x, rate, guess):
    """The fundamental near guess, by the peak of the Goertzel amplitude over +-6 %, refined twice."""
    best, span = guess, 0.06
    for _ in range(3):
        cands = [best * (1 + span * (k / 20.0 - 1)) for k in range(41)]
        best = max(cands, key=lambda f: goertzel(x, rate, f))
        span /= 10
    return best


def spectrum(x, rate, f0, count=12):
    a1 = goertzel(x, rate, f0)
    return [20 * math.log10(max(goertzel(x, rate, k * f0), 1e-12) / a1) for k in range(1, count + 1)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("folder")
    p.add_argument("set")
    p.add_argument("note", type=int)
    p.add_argument("sim")
    p.add_argument("--t0", type=float, default=0.02, help="the window's start after the note's onset, s")
    p.add_argument("--periods", type=int, default=8)
    p.add_argument("--ton", type=float, default=0.01)
    a = p.parse_args()

    w = wave.open(os.path.join(a.folder, a.set + ".wav"))
    rate, n = w.getframerate(), w.getnframes()
    data = struct.unpack("<%dh" % (n * w.getnchannels()), w.readframes(n))
    left = data[0::w.getnchannels()]
    onset_guess = 1.14 + 3.0 * (a.note - 1)
    # The onset: the first sample after onset_guess - 0.1 s whose level passes a tenth of the note's peak.
    lo, hi = int((onset_guess - 0.1) * rate), int((onset_guess + 0.4) * rate)
    seg = left[lo:hi]
    peak = max(abs(v) for v in seg)
    onset = lo + next(i for i, v in enumerate(seg) if abs(v) > 0.1 * peak)
    f0_guess = 65.406
    start = onset + int(a.t0 * rate)
    length = int(a.periods / f0_guess * rate)
    real = [float(v) for v in left[start:start + length]]
    f_real = fundamental(real, rate, f0_guess)

    with open(a.sim, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    t = [float(r["time"]) for r in rows]
    v = [float(r["nout"]) for r in rows]
    # Resample the simulation evenly at the recording's rate.
    sim_rate = rate
    t_on = a.ton + a.t0
    sim = []
    j = 0
    for i in range(length):
        ti = t_on + i / sim_rate
        while j < len(t) - 2 and t[j + 1] < ti:
            j += 1
        frac = (ti - t[j]) / (t[j + 1] - t[j])
        sim.append(v[j] + frac * (v[j + 1] - v[j]))
    f_sim = fundamental(sim, sim_rate, f0_guess)

    sr, ss = spectrum(real, rate, f_real), spectrum(sim, sim_rate, f_sim)
    print("{} note {}: real 303 f0 {:.2f} Hz, the circuit {:.2f} Hz ({:+.1f} cents)".format(
        a.set, a.note, f_real, f_sim, 1200 * math.log2(f_sim / f_real)))
    print("harmonic   real dB   circuit dB   difference")
    for k, (x, y) in enumerate(zip(sr, ss), 1):
        print("{:8d}   {:7.1f}   {:10.1f}   {:+9.1f}".format(k, x, y, y - x))
    diffs = [y - x for x, y in zip(sr[1:8], ss[1:8])]
    print("harmonics 2 to 8: mean difference {:+.1f} dB, largest {:.1f} dB".format(
        sum(diffs) / len(diffs), max(abs(d) for d in diffs)))


if __name__ == "__main__":
    main()
