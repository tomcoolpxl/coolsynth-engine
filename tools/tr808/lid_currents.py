"""Records every part's current and every node's voltage of a TR-808 drum netlist with ngspice, for CoolSynth 808's
Under the Lid, which plays them back on the drum's complete schematic (CoolSynth's D263, D278). Run from the
repository root, for example:

    python tools/tr808/lid_currents.py tools/tr808/snare.cir snare_currents.json --set amp=8 snappy=0.6 wn=0.25

It writes one JSON object: the setting, the sample times (ms), the series' names and scales, the series as 16-bit
integers in base64 (series by series), and each drawn part's nodes as the netlist has them, so the app can check its
drawing against the netlist. Only resistors, capacitors, diodes and transistors are recorded and checked: the
op-amps' behavioural insides (G, B, their RI, CI and clamp diodes), the sources and the solver's aids are drawn as
symbols and pins, or not at all. ngspice is installed, never committed: COOLSYNTH_NGSPICE names it, or it is in a
tools/ngspice folder of this repository or of the app holding it as a submodule.
"""
import argparse
import base64
import json
import os
import re
import struct
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_NGSPICE_PLACES = [os.environ.get("COOLSYNTH_NGSPICE", "")] + [
    os.path.join(base, "tools", "ngspice", "Spice64", "bin", "ngspice_con.exe")
    for base in (ROOT, os.path.join(ROOT, os.pardir, os.pardir))]
NGSPICE = next((p for p in _NGSPICE_PLACES if p and os.path.isfile(p)), _NGSPICE_PLACES[1])
STEP = 20.0e-6
# The op-amps' behavioural insides and the clamps on them: never drawn.
SOLVER_AIDS = re.compile(r"^[rc]i(\d+[a-z]?|[a-z]\d*)$|^dc[pn]\w*$")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("netlist")
    parser.add_argument("out")
    parser.add_argument("--set", nargs="*", default=[], help="param=value pairs, e.g. amp=8 tone=0.5")
    parser.add_argument("--leave", nargs="*", default=[], help="parts not drawn, besides the op-amps' insides")
    parser.add_argument("--tend", type=float, default=0.3)
    parser.add_argument("--early", type=float, default=0.008, help="seconds kept at every sample, from the start")
    args = parser.parse_args()

    settings = dict(s.split("=", 1) for s in args.set)
    settings["tend"] = repr(args.tend)
    leave = {p.lower() for p in args.leave}
    with open(args.netlist, "r", encoding="ascii") as f:
        lines = f.read().splitlines()

    parts, body = {}, []
    for line in lines:
        fields = line.split()
        if line.startswith(".param"):
            for key, value in settings.items():
                line = re.sub(r"\b{}=\S+".format(re.escape(key)), "{}={}".format(key, value), line)
        elif line.startswith(".tran"):
            line = ".tran {:.9g} {} 0 {:.9g}".format(STEP, args.tend, STEP / 4)
        elif line.strip() == ".end":
            continue
        if fields and re.match(r"^[RCDQ]\w*$", fields[0]):
            name = fields[0].lower()
            if name not in leave and not SOLVER_AIDS.match(name):
                count = 3 if name[0] == "q" else 2
                parts[name] = [n.lower() for n in fields[1:1 + count]]
        body.append(line)
    missing = [k for k in settings if k != "tend" and not any(l.startswith(".param") and re.search(r"\b{}=".format(re.escape(k)), l) for l in body)]
    if missing:
        sys.exit("no such parameter: " + ", ".join(missing))

    nodes = sorted({n for ns in parts.values() for n in ns if n != "0"})
    vectors = ["v({})".format(n) for n in nodes]
    for name in parts:
        if name[0] == "q":
            vectors += ["@{}[ic]".format(name), "@{}[ib]".format(name), "@{}[ie]".format(name)]
        else:
            vectors.append("@{}[{}]".format(name, "id" if name[0] == "d" else "i"))

    work = tempfile.mkdtemp(prefix="tr808-lid-")
    data = os.path.join(work, "data.txt").replace("\\", "/")
    body += [".options savecurrents", ".control", "set noaskquit", "set wr_singlescale", "set wr_vecnames", "run",
             "linearize " + " ".join(vectors), "wrdata {} {}".format(data, " ".join(vectors)), "quit", ".endc", ".end"]
    netlist = os.path.join(work, "run.cir")
    with open(netlist, "w", encoding="ascii") as f:
        f.write("\n".join(body) + "\n")
    result = subprocess.run([NGSPICE, "-b", netlist], capture_output=True, text=True)
    if not os.path.exists(data):
        sys.stderr.write(result.stdout[-3000:] + "\n" + result.stderr[-3000:] + "\n")
        sys.exit("ngspice wrote no data")

    rows = []
    with open(data, "r", encoding="ascii") as f:
        for line in f:
            fields = line.split()
            if fields and (fields[0][0].isdigit() or fields[0][0] in "-."):
                rows.append([float(x) for x in fields])
    if len(rows[0]) != len(vectors) + 1:
        sys.exit("expected {} columns, ngspice wrote {}".format(len(vectors) + 1, len(rows[0])))

    # Every sample of the start (the trigger and the attack), then one every 0.52 ms (the ring and the tail).
    early = int(round(args.early / STEP))
    stride = max(1, int(round(0.00052 / STEP)))
    picked = list(range(0, min(early + 1, len(rows)))) + list(range(early + stride, len(rows), stride))
    times = [round(rows[i][0] * 1000.0, 4) for i in picked]

    names, scales, packed = [], [], bytearray()
    for column, vector in enumerate(vectors, start=1):
        series = [rows[i][column] for i in picked]
        peak = max(abs(x) for x in series) or 1.0
        names.append(vector[2:-1] if vector.startswith("v(") else vector[1:].replace("[", ".").rstrip("]"))
        scales.append(peak / 32767.0)
        packed += struct.pack("<{}h".format(len(series)), *[int(round(x / peak * 32767.0)) for x in series])

    payload = {"setting": {k: v for k, v in settings.items() if k != "tend"}, "t": times, "names": names,
               "scales": scales, "parts": parts, "data": base64.b64encode(bytes(packed)).decode("ascii")}
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(payload, f, separators=(",", ":"))
    print("{}: {} parts, {} series of {} samples ({} KB)".format(os.path.basename(args.netlist), len(parts), len(names),
                                                              len(times), os.path.getsize(args.out) // 1024))


if __name__ == "__main__":
    main()
