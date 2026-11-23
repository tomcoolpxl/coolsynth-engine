"""Renders a TB-303 stage's netlist (vco.cir, ...) with ngspice and writes the chosen voltages as CSV (D275).

Usage:
  python -I tools/tb303/render.py vco.cir --set cv=2.75 --set tm4=0.5 --tend 0.05 --out out.csv
  python -I tools/tb303/render.py vco.cir --nodes nsaw,nsq --rate 192000 --out out.csv

--set name=value overrides any .param; --nodes names the voltages written (default: each netlist's own list);
--rate resamples the output evenly (0 writes ngspice's own steps); --step sets the solver's largest step. ngspice is
looked for as tools/tr808/render.py looks for it (COOLSYNTH_NGSPICE, then tools/ngspice here or in the app).
"""
import argparse
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
_NGSPICE_PLACES = [os.environ.get("COOLSYNTH_NGSPICE", "")] + [
    os.path.join(base, "tools", "ngspice", "Spice64", "bin", "ngspice_con.exe")
    for base in (ROOT, os.path.join(ROOT, os.pardir, os.pardir))]
NGSPICE = next((p for p in _NGSPICE_PLACES if p and os.path.isfile(p)), _NGSPICE_PLACES[1])

NODES = {"vco.cir": ["nsaw", "nsq", "nc33", "ncv", "nb", "nbase", "nem", "ne8"],
         "vcf.cir": ["ntp6", "nvcf", "nin", "nb10", "nb11", "nvq9", "n13"],
         "env.cir": ["nenv", "nc62", "ngate", "nc38", "naccw", "ndec"],
         "vca.cir": ["nout", "nvout", "niabc", "nb31", "ngate", "nenv"]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("netlist")
    parser.add_argument("--set", action="append", default=[])
    parser.add_argument("--nodes")
    parser.add_argument("--tend", type=float)
    parser.add_argument("--rate", type=float, default=0.0)
    parser.add_argument("--step", type=float, default=0.2e-6, help="the solver's largest step, seconds")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    path = os.path.join(HERE, args.netlist)
    values = {}
    for item in args.set:
        name, _, value = item.partition("=")
        values[name.strip()] = value.strip()
    if args.tend is not None:
        values["tend"] = repr(args.tend)

    nodes = args.nodes.split(",") if args.nodes else NODES.get(os.path.basename(path), [])
    if not nodes:
        sys.exit("name the nodes to write with --nodes")
    vectors = " ".join("v({})".format(n) for n in nodes)

    with open(path, encoding="utf-8") as f:
        lines = f.read().splitlines()
    body = []
    for line in lines:
        low = line.strip().lower()
        if low == ".end":
            continue
        if low.startswith(".tran"):
            line = ".tran {0:.12g} {{tend}} 0 {0:.12g}".format(args.step)
        body.append(line)
    # Overrides go after the netlist's own .param lines, so the later definition wins.
    insert = next((i for i, l in enumerate(body) if l.strip().lower().startswith(".model")), len(body))
    body[insert:insert] = [".param {}={}".format(k, v) for k, v in values.items()]

    with tempfile.TemporaryDirectory() as tmp:
        data = os.path.join(tmp, "out.txt")
        control = [".control", "set noaskquit", "set wr_singlescale", "set wr_vecnames", "run"]
        if args.rate > 0:
            control.append("linearize " + vectors)
        control += ["wrdata {} {}".format(data.replace("\\", "/"), vectors), "quit", ".endc", ".end"]
        netlist = os.path.join(tmp, "in.cir")
        with open(netlist, "w", encoding="utf-8") as f:
            f.write("\n".join(body + control) + "\n")
        if args.rate > 0:
            with open(netlist, encoding="utf-8") as f:
                text = f.read()
            text = text.replace(".tran {0:.12g}".format(args.step), ".tran {0:.12g}".format(1.0 / args.rate), 1)
            with open(netlist, "w", encoding="utf-8") as f:
                f.write(text)
        result = subprocess.run([NGSPICE, "-b", netlist], capture_output=True, text=True)
        if not os.path.isfile(data):
            sys.stderr.write(result.stdout[-4000:] + result.stderr[-4000:])
            sys.exit("ngspice wrote no data")
        with open(data, encoding="utf-8") as f, open(args.out, "w", encoding="utf-8") as out:
            header = f.readline().split()
            out.write(",".join(["time"] + nodes) + "\n")
            for line in f:
                parts = line.split()
                if parts:
                    out.write(",".join(parts) + "\n")
    print("wrote", args.out, "from", os.path.basename(path), "with", ", ".join("{}={}".format(k, v) for k, v in values.items()))


if __name__ == "__main__":
    main()
