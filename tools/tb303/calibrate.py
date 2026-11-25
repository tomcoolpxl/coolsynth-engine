"""Calibrates the netlists' trimmers as the service notes' page 7 does (D275).

The VCO: TM5 (WIDTH) for 2:1 between the low C key (2.0 V) and the high C key (3.0 V), then TM4 (TUNE) for 110 Hz on
the A key (2.75 V), TUNING centred; then confirms 4:1 between low C and high C transposed up (4.0 V).
The VCF: TM3 for a ring of 2 ms at TP6 with the saw on the low C key, CUTOFF centred, RESONANCE full, ENV MOD, DECAY
and ACCENT at nothing.

Usage: python -I tools/tb303/calibrate.py [vco|vcf]
Results (2026-11-23, the rail at 11.85 V): tm4 0.5687, tm5 0.7389 (A 110.0 Hz, 4.002:1); tm3 0.1642 (a 2.000 ms ring).
The netlists hold them.
"""
import concurrent.futures
import csv
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from inspect_vco import crossings  # noqa: E402
import inspect_vcf  # noqa: E402


def render(netlist, values, nodes, tend, step=None):
    tmp = tempfile.mkdtemp()
    out = os.path.join(tmp, "r.csv")
    command = [sys.executable, "-I", os.path.join(HERE, "render.py"), netlist, "--nodes", nodes, "--tend", str(tend),
               "--out", out]
    for k, v in values.items():
        command += ["--set", "{}={}".format(k, v)]
    if step:
        command += ["--step", str(step)]
    subprocess.run(command, check=True, capture_output=True)
    return out


def frequency(cv, tm4, tm5, tend=0.07, skip=0.03):
    out = render("vco.cir", {"cv": cv, "tm4": tm4, "tm5": tm5}, "nc33", tend)
    with open(out, encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if float(r["time"]) >= skip]
    t = [float(r["time"]) for r in rows]
    v = [float(r["nc33"]) for r in rows]
    rises = crossings(t, v, 0.5 * (max(v) + min(v)), True)
    return (len(rises) - 1) / (rises[-1] - rises[0])


def ratio(cv_low, cv_high, tm4, tm5, pool):
    low, high = pool.map(lambda cv: frequency(cv, tm4, tm5), [cv_low, cv_high])
    return high / low


def vco():
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        tm4 = 0.5
        lo, hi = 0.0, 1.0
        for _ in range(12):
            tm5 = 0.5 * (lo + hi)
            r = ratio(2.0, 3.0, tm4, tm5, pool)
            print("TM5 {:.4f}: {:.5f} : 1".format(tm5, r), flush=True)
            # More resistance in TM5 means less of the CV at Q26's base, so a smaller ratio.
            if r > 2.0:
                lo = tm5
            else:
                hi = tm5
        tm5 = 0.5 * (lo + hi)
        lo, hi = 0.0, 1.0
        for _ in range(12):
            tm4 = 0.5 * (lo + hi)
            f = frequency(2.75, tm4, tm5)
            print("TM4 {:.4f}: A at {:.4f} Hz".format(tm4, f), flush=True)
            # TM4 toward 12 V raises the reference current, so the pitch.
            if f < 110.0:
                lo = tm4
            else:
                hi = tm4
        tm4 = 0.5 * (lo + hi)
        r = ratio(2.0, 4.0, tm4, tm5, pool)
        print("calibrated: tm4={:.4f} tm5={:.4f}; low C to high C up {:.5f} : 1 (the notes: 4 : 1 within 0.5 %)".format(
            tm4, tm5, r))


def ring_ms(tm3):
    # CUTOFF at its centre: its pot's share there (Tb303Voice::audioTrack, CoolSynth's D364)
    out = render("vcf.cir", {"tm3": tm3, "cutoff": 0.15, "res": 1, "envmod": 0, "acc": 0}, "ntp6,nin", 0.25, 2e-6)
    return inspect_vcf.measure(out, 0.15)["ring_ms"]


def vcf():
    # More resistance in TM3 lowers the cutoff, so lengthens the ring. Bisect within the range where it rings.
    lo, hi = 0.0, 0.3
    for _ in range(10):
        tm3 = 0.5 * (lo + hi)
        ms = ring_ms(tm3)
        print("TM3 {:.4f}: ring {:.4f} ms".format(tm3, ms), flush=True)
        if ms != ms or ms > 2.0:
            hi = tm3
        else:
            lo = tm3
    print("calibrated: tm3={:.4f} (the notes: 2 ms +- 0.5 ms)".format(0.5 * (lo + hi)))


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "vco"
    {"vco": vco, "vcf": vcf}[which]()
