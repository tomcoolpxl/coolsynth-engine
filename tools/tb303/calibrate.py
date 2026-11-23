"""Calibrates the VCO netlist's trimmers as the service notes' page 7 does (D275): TM5 (WIDTH) for 2:1 between the low
C key (2.0 V) and the high C key (3.0 V), then TM4 (TUNE) for 110 Hz on the A key (2.75 V), TUNING centred; then
confirms 4:1 between low C and high C transposed up (4.0 V).

Usage: python -I tools/tb303/calibrate.py
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


def frequency(cv, tm4, tm5, tend=0.07, skip=0.03):
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "f.csv")
        subprocess.run([sys.executable, "-I", os.path.join(HERE, "render.py"), "vco.cir", "--nodes", "nc33",
                        "--set", "cv={}".format(cv), "--set", "tm4={}".format(tm4), "--set", "tm5={}".format(tm5),
                        "--tend", str(tend), "--out", out], check=True, capture_output=True)
        with open(out, encoding="utf-8") as f:
            rows = [r for r in csv.DictReader(f) if float(r["time"]) >= skip]
    t = [float(r["time"]) for r in rows]
    v = [float(r["nc33"]) for r in rows]
    rises = crossings(t, v, 0.5 * (max(v) + min(v)), True)
    return (len(rises) - 1) / (rises[-1] - rises[0])


def ratio(cv_low, cv_high, tm4, tm5, pool):
    low, high = pool.map(lambda cv: frequency(cv, tm4, tm5), [cv_low, cv_high])
    return high / low


def main():
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


if __name__ == "__main__":
    main()
