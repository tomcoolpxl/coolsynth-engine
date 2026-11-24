"""Records every part's current and every node's voltage of the whole TB-303 voice (voice.cir) with ngspice, for the
CoolSynth 303 mockup's Under the Lid sheet: docs/design/mockups/coolsynth-303-lid.html (in the CoolSynth repository)
plays them back on its schematic. Run from the engine's root:

    python -I tools/tb303/mockup_currents.py --html <CoolSynth>/docs/design/mockups/coolsynth-303-lid.html

The phrase: a plain note, an accented note that slides into a third note, then a rest (about 0.5 s). voice.cir plays
one note from its parameters; here its five sources (the CPU's DAC, the slide line and its inverse, the gate, the
accent line) become PWL sources that play the phrase. The circuit itself is voice.cir's, untouched; the knobs are the
app's defaults.

It writes one JSON object into the page's <script id="lidData"> element: the sample times (ms), the series' names and
scales, the series as 16-bit integers (series by series, each one delta-coded, the whole gzipped and in base64), each
drawn part's nodes as the netlist has them (subcircuits flattened, xvco.nc33 for VCO's nc33), its value and kind, and
the phrase's events, so the page can check its drawing against the netlist and caption the time bar.

With --csv (and no --html) it leaves the page alone and writes the engine's lid tap reference instead (the engine's
Tb303Lid, tests/V3Tb303LidTests.cpp): one row a series, in the page's order, its peak over the run and its exact
values every --csv-every ms (10 by default) from 0, the first column the operating point:

    python -I tools/tb303/mockup_currents.py --csv tests/data/tb303/lid.csv

Left out, as the drawing leaves them out: the op-amps' macro-model insides (GA, RIA, CIA, the clamps DCPA, VCPA, DCNA,
VCNA, and IC11b's GB set): each op-amp is one part whose output current is its B source's (BA, BB); the rails' sources
VCC and VR (the sheet's rails and pins).
"""
import argparse
import base64
import gzip
import json
import os
import re
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
_NGSPICE_PLACES = [os.environ.get("COOLSYNTH_NGSPICE", "")] + [
    os.path.join(base, "tools", "ngspice", "Spice64", "bin", "ngspice_con.exe")
    for base in (ROOT, os.path.join(ROOT, os.pardir, os.pardir), os.path.join(ROOT, os.pardir, os.pardir, os.pardir, "CoolSynth"))]
NGSPICE = next((p for p in _NGSPICE_PLACES if p and os.path.isfile(p)), _NGSPICE_PLACES[1])
NETLIST = os.path.join(HERE, "voice.cir")

# The app's default knobs (saw), and the BA662's input pair as the engine has it fitted (CoolSynth's D324). res is
# the pot's share at the knob's 0.72 (Tb303Voice::resonanceTrack: the fitted S-shaped track, times the loop's 0.96).
KNOBS = {"cutoff": 0.42, "res": 0.833888, "envmod": 0.55, "decay": 0.48, "accent": 0.62, "sq": 0, "tune": 0.5,
         "q1area": 22}
# The op-amps' solver aids, left out of the drawing.
AIDS = {"ga", "ria", "cia", "dcpa", "vcpa", "dcna", "vcna", "gb", "rib", "cib", "dcpb", "vcpb", "dcnb", "vcnb"}
STEP = 20e-6

# The phrase, 120 BPM sixteenths (125 ms), the gate half a step as the 303's: step 1 a plain A (2.75 V), step 2 an
# accented E (+7 semitones) with its slide flag, so its gate holds into step 3, an A an octave up (3.75 V) slid to.
T1, T2, T3, TOFF3 = 0.010, 0.135, 0.260, 0.3225
PHRASE = {
    "cv": [(0, 2.75), (T2 - 0.004, 2.75), (T2 - 0.004 + 1e-6, 2.75 + 7 / 12.0), (T3, 2.75 + 7 / 12.0), (T3 + 1e-6, 3.75)],
    "gate": [(0, 0), (T1, 0), (T1 + 1e-6, 5), (T1 + 0.0625, 5), (T1 + 0.0625 + 1e-6, 0),
             (T2, 0), (T2 + 1e-6, 5), (TOFF3, 5), (TOFF3 + 1e-6, 0)],
    "acc": [(0, 0), (T2, 0), (T2 + 1e-6, 12), (T3, 12), (T3 + 1e-6, 0)],
    "slide": [(0, 0), (T3, 0), (T3 + 1e-6, 1), (T3 + 0.125, 1), (T3 + 0.125 + 1e-6, 0)],
}
EVENTS = [
    {"t": 0.0, "text": "at rest: the oscillator runs, the gate is off, only the standing currents flow"},
    {"t": T1 * 1e3, "text": "note 1, A (2.75 V): the gate opens, the one-shot charges C62, the attack"},
    {"t": T1 * 1e3 + 4, "text": "note 1: the decay, C62 runs down through R138 and DECAY"},
    {"t": (T1 + 0.0625) * 1e3, "text": "note 1: the gate closes, the VCA's control current falls"},
    {"t": (T2 - 0.004) * 1e3, "text": "rest: the CPU sets the next note, E (3.33 V)"},
    {"t": T2 * 1e3, "text": "note 2, E, accented: the gate opens, IC12 shorts DECAY and passes the envelope to ACCENT"},
    {"t": T2 * 1e3 + 4, "text": "note 2: the accent sweep, C13 charges through D24, the filter opens further"},
    {"t": T3 * 1e3, "text": "note 3, A an octave up, slid to: the gate holds, C35 glides the pitch through RDAC"},
    {"t": TOFF3 * 1e3, "text": "the gate closes: the release, the VCA's current runs out"},
    {"t": (TOFF3 + 0.03) * 1e3, "text": "rest: only the standing currents and the oscillator"},
]


def pwl(points):
    return "PWL(" + " ".join("{:.9g} {:.9g}".format(t, v) for t, v in points) + ")"


def parse(lines):
    """The netlist's subcircuits and top-level elements, comment and continuation lines resolved."""
    joined = []
    for line in lines:
        if line.startswith("+") and joined:
            joined[-1] += " " + line[1:]
        else:
            joined.append(line)
    subckts, top, current = {}, [], None
    for line in joined:
        low = line.strip().lower()
        if not low or low.startswith("*"):
            continue
        if low.startswith(".subckt"):
            f = line.split()
            if "params:" in (w.lower() for w in f):
                f = f[:[w.lower() for w in f].index("params:")]
            current = {"ports": [p.lower() for p in f[2:]], "elements": []}
            subckts[f[1].lower()] = current
        elif low.startswith(".ends"):
            current = None
        elif low.startswith("."):
            continue
        elif current is not None:
            current["elements"].append(line)
        else:
            top.append(line)
    return subckts, top


NODE_COUNT = {"r": 2, "c": 2, "d": 2, "v": 2, "b": 2, "q": 3, "j": 3, "s": 4, "g": 4}


def flatten(subckts, lines, prefix, portmap, out):
    """Every element as (ngspice name, plain name, kind, nodes, value), nodes with their full hierarchical names."""
    def node(n):
        n = n.lower()
        if n == "0":
            return "0"
        if n in portmap:
            return portmap[n]
        return prefix + n if prefix else n
    for line in lines:
        f = line.split()
        name = f[0].lower()
        kind = name[0]
        if kind == "x":
            if "params:" in (w.lower() for w in f):  # a subcircuit's parameters follow its name
                f = f[:[w.lower() for w in f].index("params:")]
            sub = subckts[f[-1].lower()]
            args = [node(n) for n in f[1:-1]]
            flatten(subckts, sub["elements"], (prefix + name + "."), dict(zip(sub["ports"], args)), out)
            continue
        count = NODE_COUNT.get(kind)
        if count is None:
            continue
        nodes = [node(n) for n in f[1:1 + count]]
        value = " ".join(f[1 + count:])
        path = prefix.rstrip(".")
        spice = (kind + "." + path + "." + name) if path else name
        out.append({"spice": spice, "key": (path + "." + name) if path else name, "kind": kind, "nodes": nodes,
                    "value": value})
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--html")
    parser.add_argument("--csv", help="write the engine's lid tap reference here instead of the page")
    parser.add_argument("--csv-every", type=float, default=10.0, help="the reference's moments, ms apart")
    parser.add_argument("--tend", type=float, default=0.5)
    parser.add_argument("--step", type=float, default=STEP)
    parser.add_argument("--keep", help="a folder to keep the run's netlist and data in")
    args = parser.parse_args()
    if not args.html and not args.csv:
        parser.error("--html or --csv")

    with open(NETLIST, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()

    # The variant: the knobs, the phrase's sources, the run's length and step.
    body = []
    for line in lines:
        low = line.strip().lower()
        m = re.match(r"^\.param (\w+)=", low)
        if m and m.group(1) in KNOBS:
            line = ".param {}={}".format(m.group(1), KNOBS[m.group(1)])
        elif m and m.group(1) == "tend":
            line = ".param tend={}".format(args.tend)
        elif low.startswith("vdac "):
            line = "VDAC ndac0 0 " + pwl(PHRASE["cv"])
        elif low.startswith("vsl "):
            line = "VSL nsl 0 " + pwl(PHRASE["slide"])
        elif low.startswith("vns "):
            line = "VNS nns 0 " + pwl([(t, 1 - v) for t, v in PHRASE["slide"]])
        elif low.startswith("vg "):
            line = "VG ng 0 " + pwl(PHRASE["gate"])
        elif low.startswith("vacc "):
            line = "VACC nacc 0 " + pwl(PHRASE["acc"])
        elif low.startswith(".tran"):
            line = ".tran {:.9g} {{tend}} 0 2u".format(args.step)
        elif low == ".end":
            continue
        body.append(line)

    subckts, top = parse(body)
    elements = flatten(subckts, top, "", {}, [])

    parts, vectors, series = {}, [], []   # series: (name, [(vector index, sign), ...])

    def want(vec):
        if vec not in vectors:
            vectors.append(vec)
        return vectors.index(vec)

    for e in elements:
        plain = e["key"].split(".")[-1]
        if plain in AIDS or e["key"] in ("vcc", "vr"):
            continue
        kind, key = e["kind"], e["key"]
        if kind == "b" and plain in ("ba", "bb"):
            # The op-amp: + and - inputs, the output; its current is its B source's.
            path = key.rsplit(".", 1)[0] + "."
            g = next(x for x in elements if x["key"] == path + ("ga" if plain == "ba" else "gb"))
            okey = path + ("ic11a" if plain == "ba" else "ic11b")
            parts[okey] = {"kind": "op", "nodes": [g["nodes"][2], g["nodes"][3], e["nodes"][0]], "value": "AN6562"}
            series.append((okey + ".i", [(want("@{}[i]".format(e["spice"])), 1)]))
            continue
        if kind == "b" and plain == "bwave":
            # The WAVEFORM switch: the saw's throw, the square's throw, the common (the netlist's buffer stands for it).
            parts["wave"] = {"kind": "wave", "nodes": ["nsaw", "nsq", "nwave"], "value": "SAW / SQUARE"}
            series.append(("wave.i", [(want("@bwave[i]"), 1)]))
            continue
        parts[key] = {"kind": kind, "nodes": e["nodes"], "value": e["value"]}
        if kind in "rcb":
            series.append((key + ".i", [(want("@{}[i]".format(e["spice"])), 1)]))
        elif kind == "d":
            series.append((key + ".id", [(want("@{}[id]".format(e["spice"])), 1)]))
        elif kind == "s":
            series.append((key + ".i", [(want("@{}[i]".format(e["spice"])), 1)]))
        elif kind == "v":
            series.append((key + ".i", [(want("{}#branch".format(e["spice"])), 1)]))
        elif kind == "q":
            series.append((key + ".ic", [(want("@{}[ic]".format(e["spice"])), 1)]))
            series.append((key + ".ib", [(want("@{}[ib]".format(e["spice"])), 1)]))
        elif kind == "j":
            series.append((key + ".id", [(want("@{}[id]".format(e["spice"])), 1)]))
            series.append((key + ".ig", [(want("@{}[ig]".format(e["spice"])), 1)]))

    nodes = sorted({n for p in parts.values() for n in p["nodes"] if n != "0"})
    for n in nodes:
        series.append((n, [(want("v({})".format(n)), 1)]))

    work = args.keep or tempfile.mkdtemp(prefix="tb303-currents-")
    os.makedirs(work, exist_ok=True)
    data = os.path.join(work, "data.txt").replace("\\", "/")
    if os.path.exists(data):
        os.remove(data)
    run = body + [
        ".options savecurrents interp",
        ".control",
        "set noaskquit",
        "set wr_singlescale",
        "set wr_vecnames",
        "run",
        "wrdata {} {}".format(data, " ".join(vectors)),
        "quit",
        ".endc",
        ".end",
    ]
    netlist = os.path.join(work, "run.cir")
    with open(netlist, "w", encoding="ascii") as f:
        f.write("\n".join(run) + "\n")
    print("running ngspice on {} vectors ...".format(len(vectors)), flush=True)
    result = subprocess.run([NGSPICE, "-b", netlist], capture_output=True, text=True)
    if not os.path.exists(data):
        sys.stderr.write(result.stdout[-3000:] + "\n" + result.stderr[-3000:] + "\n")
        sys.exit("ngspice wrote no data")

    columns = [[] for _ in range(len(vectors) + 1)]
    with open(data, "r", encoding="ascii") as f:
        for line in f:
            fields = line.split()
            if not fields or not (fields[0][0].isdigit() or fields[0][0] in "-."):
                continue
            if len(fields) != len(vectors) + 1:
                sys.exit("expected {} columns, ngspice wrote {}".format(len(vectors) + 1, len(fields)))
            for c, x in enumerate(fields):
                columns[c].append(float(x))
    # interp writes the step's grid; keep one sample per step from 0.
    times = columns[0]
    picked, want_t = [], 0.0
    for i, t in enumerate(times):
        if t >= want_t - args.step * 0.01:
            picked.append(i)
            want_t += args.step
    times = [round(times[i] * 1000.0, 4) for i in picked]

    if args.csv:
        # The moments on the step's grid nearest each multiple of csv_every; the exact values, not the page's 16 bits.
        moments, k = [], 0
        while k * args.csv_every <= times[-1] + 1e-9:
            want_ms = k * args.csv_every
            moments.append(min(range(len(times)), key=lambda j: abs(times[j] - want_ms)))
            k += 1
        with open(args.csv, "w", encoding="ascii", newline="\n") as f:
            f.write("name,peak," + ",".join("{:.4f}".format(times[j]) for j in moments) + "\n")
            for name, terms in series:
                values = [sum(sign * columns[c + 1][i] for c, sign in terms) for i in picked]
                peak = max(abs(x) for x in values)
                f.write(name + ",{:.9g},".format(peak) + ",".join("{:.9g}".format(values[j]) for j in moments) + "\n")
        print("wrote {} series at {} moments into {}".format(len(series), len(moments), args.csv))
        return

    names, scales, packed = [], [], bytearray()
    for name, terms in series:
        values = [sum(sign * columns[c + 1][i] for c, sign in terms) for i in picked]
        peak = max(abs(x) for x in values) or 1.0
        q = [int(round(x / peak * 32767.0)) for x in values]
        delta = [q[0]] + [((q[i] - q[i - 1] + 32768) % 65536) - 32768 for i in range(1, len(q))]
        names.append(name)
        scales.append(peak / 32767.0)
        packed += struct.pack("<{}h".format(len(delta)), *delta)
    # Low bytes then high bytes: gzip finds the high bytes' runs.
    raw = bytes(packed)
    shuffled = raw[0::2] + raw[1::2]
    blob = base64.b64encode(gzip.compress(shuffled, 9)).decode("ascii")

    payload = {
        "setting": dict(KNOBS),
        "step_us": args.step * 1e6,
        "t": times,
        "names": names,
        "scales": scales,
        "parts": parts,
        "events": EVENTS,
        "phrase": {k: [[round(t * 1000, 4), v] for t, v in pts] for k, pts in PHRASE.items()},
        "data": blob,
    }
    with open(args.html, "r", encoding="utf-8") as f:
        page = f.read()
    marker = re.compile(r'(<script id="lidData" type="application/json">)(.*?)(</script>)', re.S)
    if not marker.search(page):
        sys.exit("the page has no <script id=\"lidData\"> element")
    page = marker.sub(lambda m: m.group(1) + json.dumps(payload, separators=(",", ":")) + m.group(3), page)
    with open(args.html, "w", encoding="utf-8", newline="\n") as f:
        f.write(page)
    print("wrote {} series of {} samples ({} KB gzipped, {} parts) into {}".format(
        len(names), len(times), len(blob) // 1024, len(parts), args.html))


if __name__ == "__main__":
    main()
