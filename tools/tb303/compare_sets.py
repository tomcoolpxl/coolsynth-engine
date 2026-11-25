"""Compares whole sets of Din Sync's real-TB-303 recordings with the circuit (D275): for each of a set's four knob
positions, the plain saw and the accented saw (notes 1 and 2 of each position), rendered by voice.cir with the VCO
settled, the trimmers as fitted (tm3, c11), and compared by compare_dinsync.py; plus the accent's loudness over the
plain note against the recording's.

Din Sync's chart: SET-A1 to A5 turn CUTOFF, RESONANCE, ENV MOD, DECAY, ACCENT from 25 to 100 % with the others at
0 %; sets B, C, D and E the same with the others at 25, 50, 75 and 100 %.

voice.cir is taken at its OUTPUT jack (njack, VOLUME full, the recorder's load as fitted; CoolSynth's D362), RESONANCE
through the engine's track (Tb303Voice::resonanceTrack, its loop's share as --loop), CUTOFF and ENV MOD through
theirs (Tb303Voice::audioTrack, D364). The accent's loudness is the
RMS over 30 to 80 ms weighted by time: ngspice's steps crowd on fast edges, and a plain mean of its samples put
voice.cir up to 1.7 dB under the engine on set E (D362).

Usage: python -I tools/tb303/compare_sets.py <folder> A2 [A3 ...] [--tm3 0.35] [--c11 0.82u] [--jobs 8] [--model exe]
--model compares the engine's voice instead (D295), rendered by a program as render() describes.
"""
import argparse
import concurrent.futures
import math
import os
import re
import struct
import subprocess
import sys
import tempfile
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
KNOBS = ["cutoff", "res", "envmod", "decay", "accent"]
BASE = {"A": 0.0, "B": 0.25, "C": 0.5, "D": 0.75, "E": 1.0}
TON = 0.4


def knobs_for(name, position):
    values = {k: BASE[name[0]] for k in KNOBS}
    values[KNOBS[int(name[1]) - 1]] = 0.25 * position
    return values


def resonance_track(turn, loop):
    """Tb303Voice::resonanceTrack: the pot's share at a turn of RESONANCE (CoolSynth's D324)."""
    t = min(max(turn, 0.0), 1.0)
    u = 1.0 - t
    return loop * t * t / (t * t + u * u)


def audio_track(turn):
    """Tb303Voice::audioTrack: CUTOFF's and ENV MOD's pot share at a turn of the knob (CoolSynth's D364)."""
    t = min(max(turn, 0.0), 1.0)
    if t <= 0.25:
        return 0.1 * t
    if t <= 0.5:
        return 0.025 + 0.5 * (t - 0.25)
    return 0.15 + 1.7 * (t - 0.5)


def render(values, accented, tm3, c11, out, model=None, loop=0.96):
    if model:
        # The engine's voice (D295): a program taking "csv cutoff res envmod decay accent accented out.csv" that writes
        # time and the OUTPUT jack's volts for low C, the gate at 0.4 s for 0.25 s, the trimmers as fitted.
        command = [model, "csv"] + ["{}".format(values[k]) for k in KNOBS] + [str(int(accented)), out]
        subprocess.run(command, check=True, capture_output=True)
        return
    command = [sys.executable, "-I", os.path.join(HERE, "render.py"), "voice.cir", "--nodes", "njack", "--tend", "0.6",
               "--step", "4e-6", "--out", out]
    sets = dict(values, cv=2.0, sq=0, ton=TON, toff=TON + 0.25, taoff=TON + 0.25, acc=12 if accented else 0, tm3=tm3,
                c11=c11, q1area=22)  # the BA662's input pair as fitted (CoolSynth's D324)
    sets["res"] = resonance_track(values["res"], loop)
    sets["cutoff"] = audio_track(values["cutoff"])
    sets["envmod"] = audio_track(values["envmod"])
    for k, v in sets.items():
        command += ["--set", "{}={}".format(k, v)]
    subprocess.run(command, check=True, capture_output=True)


def compare(folder, set_name, note, csv_path):
    result = subprocess.run([sys.executable, "-I", os.path.join(HERE, "compare_dinsync.py"), folder, "SET-" + set_name,
                             str(note), csv_path, "--t0", "0.03", "--ton", str(TON)],
                            check=True, capture_output=True, text=True).stdout
    cents = float(re.search(r"\(([-+0-9.]+) cents\)", result).group(1))
    mean, largest = map(float, re.search(r"mean difference ([-+0-9.]+) dB, largest ([0-9.]+) dB", result).groups())
    return cents, mean, largest


def rms_real(folder, set_name, note, a, b):
    w = wave.open(os.path.join(folder, "SET-" + set_name + ".wav"))
    rate, n = w.getframerate(), w.getnframes()
    left = struct.unpack("<%dh" % (n * 2), w.readframes(n))[0::2]
    guess = 1.14 + 3.0 * (note - 1)
    lo = int((guess - 0.1) * rate)
    seg = left[lo:lo + int(0.5 * rate)]
    peak = max(abs(v) for v in seg)
    onset = lo + next(i for i, v in enumerate(seg) if abs(v) > 0.1 * peak)
    x = left[onset + int(a * rate):onset + int(b * rate)]
    m = sum(x) / len(x)
    return math.sqrt(sum((v - m) ** 2 for v in x) / len(x))


def rms_sim(path, a, b):
    """The RMS over [TON + a, TON + b), each sample weighted by the time it stands for (trapezoids): ngspice's own
    steps are uneven, an engine's even (D362)."""
    t, x = [], []
    with open(path, encoding="utf-8") as f:
        next(f)
        for line in f:
            p = line.split(",")
            if TON + a <= float(p[0]) < TON + b:
                t.append(float(p[0]))
                x.append(float(p[1]))
    span = t[-1] - t[0]
    m = sum(0.5 * (x[i] + x[i + 1]) * (t[i + 1] - t[i]) for i in range(len(t) - 1)) / span
    s = sum(0.5 * ((x[i] - m) ** 2 + (x[i + 1] - m) ** 2) * (t[i + 1] - t[i]) for i in range(len(t) - 1)) / span
    return math.sqrt(s)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("folder")
    p.add_argument("sets", nargs="+")
    p.add_argument("--tm3", default="0.35")
    p.add_argument("--c11", default="0.82u")
    p.add_argument("--jobs", type=int, default=8)
    p.add_argument("--model", help="the engine's voice as a program (see render()); ngspice's voice.cir if left out")
    p.add_argument("--loop", type=float, default=0.96, help="Tb303Voice::resonanceLoop, for voice.cir's RESONANCE")
    a = p.parse_args()
    tmp = tempfile.mkdtemp()
    jobs = []
    for s in a.sets:
        for pos in range(1, 5):
            for accented in (False, True):
                jobs.append((s, pos, accented, os.path.join(tmp, "{}_{}_{}.csv".format(s, pos, int(accented)))))
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as pool:
        list(pool.map(lambda j: render(knobs_for(j[0], j[1]), j[2], a.tm3, a.c11, j[3], a.model, a.loop), jobs))
    for s in a.sets:
        print("SET-{} ({} turned, the others at {:.0f} %):".format(s, KNOBS[int(s[1]) - 1], 100 * BASE[s[0]]))
        for pos in range(1, 5):
            plain, acc = [os.path.join(tmp, "{}_{}_{}.csv".format(s, pos, k)) for k in (0, 1)]
            n_plain, n_acc = 4 * (pos - 1) + 1, 4 * (pos - 1) + 2
            cp, mp, lp = compare(a.folder, s, n_plain, plain)
            ca, ma, la = compare(a.folder, s, n_acc, acc)
            real = 20 * math.log10(rms_real(a.folder, s, n_acc, 0.03, 0.08) / rms_real(a.folder, s, n_plain, 0.03, 0.08))
            sim = 20 * math.log10(rms_sim(acc, 0.03, 0.08) / rms_sim(plain, 0.03, 0.08))
            print("  {:3d} %: plain {:+.1f} dB (largest {:.1f}), accented {:+.1f} dB (largest {:.1f}); accent's loudness "
                  "real {:+.1f} dB, circuit {:+.1f} dB; pitch {:+.0f} cents".format(
                      25 * pos, mp, lp, ma, la, real, sim, cp))


if __name__ == "__main__":
    main()
