"""Builds voice.cir, the whole TB-303 voice, from the stage netlists (D275): vco.cir into vcf.cir through the WAVEFORM
switch, vca.cir (with the envelope) after it, the envelope into ENV MOD and the ACCENT knob's wiper into D24. Each
stage becomes a subcircuit; the stand-ins each stage file keeps for its neighbours (the filter's input saw, its
envelope and accent sources, its VCA load; the VCA's audio sources; the envelope's ENV MOD load) are left out.

Usage: python -I tools/tb303/voice.py   (writes tools/tb303/voice.cir; render it with render.py voice.cir ...)
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))


def body(name):
    """A stage's element lines and its .param lines, without its models, rails, .options, .tran or .end."""
    lines = open(os.path.join(HERE, name), encoding="utf-8").read().splitlines()
    params, elements, in_model = [], [], False
    for line in lines:
        low = line.strip().lower()
        if low.startswith(".model"):
            in_model = True
            continue
        if in_model and low.startswith("+"):
            continue
        in_model = False
        if low.startswith(".param"):
            params.append(line)
        elif low.startswith((".options", ".tran", ".ic")) or low == ".end" or low.startswith("*") or not low:
            continue
        elif re.match(r"^(vcc|vr) ", low):
            continue
        else:
            elements.append(line)
    return params, elements


def models(name):
    text = open(os.path.join(HERE, name), encoding="utf-8").read()
    out, keep = [], False
    for line in text.splitlines():
        if line.strip().lower().startswith(".model"):
            keep = True
        elif not line.startswith("+"):
            keep = False
        if keep:
            out.append(line)
    return out


def drop(elements, names):
    return [e for e in elements if e.split()[0].upper() not in names]


def main():
    vco_p, vco = body("vco.cir")
    vcf_p, vcf = body("vcf.cir")
    vca_p, vca = body("vca.cir")
    # The filter's stand-ins for its neighbours, and the VCA's.
    vcf = drop(vcf, {"BIN", "VENV", "VACC", "R121", "C21"})
    vca = drop(vca, {"VAUD", "BRES", "RLOAD", "VQ9"})
    # The VCA file holds a subcircuit (the BA662) whose lines body() kept: split it out.
    sub_start = next(i for i, e in enumerate(vca) if e.lower().startswith(".subckt"))
    sub_end = next(i for i, e in enumerate(vca) if e.lower().startswith(".ends"))
    ba662 = vca[sub_start:sub_end + 1]
    vca = vca[:sub_start] + vca[sub_end + 1:]
    ba662_models = [m for m in models("vca.cir") if m.split()[1] in ("NI", "PL")]
    ba662 = [l for l in ba662 if not l.lower().startswith((".model", "+"))]
    ba662 = ba662[:-1] + ba662_models + ba662[-1:]
    ba662_models = []

    out = ["* The whole Roland TB-303 voice (D275), built by voice.py from vco.cir, vcf.cir and vca.cir: do not edit, edit",
           "* the stages and run voice.py again. The VCO's saw or square (sq) through the WAVEFORM switch into the filter;",
           "* the envelope into ENV MOD; the ACCENT knob's wiper into D24; the filter's output and the RESONANCE wiper into",
           "* the VCA. The note: cv (volts, 1 V an octave, the A key 2.75 V), the gate from ton to toff, acc (12 on an accent),",
           "* and every knob as the stages name them (cutoff, res, envmod, decay, accent, tune).",
           ".param sq=0"]
    seen = set()
    for plist in (vco_p, vcf_p, vca_p):
        for p in plist:
            for name, value in re.findall(r"(\w+)=(\{[^}]*\}|\S+)", p[len(".param"):]):
                if name in seen or name in ("freq", "amp", "env", "sq"):
                    continue
                seen.add(name)
                out.append(".param {}={}".format(name, value))
    # Every stage's models once; the VCO's and the envelope's switches differ (SWI), so the envelope's is SWV here.
    names = set()
    for stage in ("vco.cir", "vcf.cir", "env.cir"):
        block, current = [], None
        for line in models(stage):
            if line.lower().startswith(".model"):
                current = line.split()[1]
                if stage == "env.cir" and current == "SWI":
                    line = line.replace("SWI", "SWV", 1)
                    current = "SWV"
            if current not in names or line.startswith("+") and block and block[-1] == current:
                out.append(line)
                block.append(current)
        names.update(block)
    out += ["", "VCC vcc 0 {vcc}", "VR vr 0 5.333", ""]

    out += [".subckt VCO vcc vr nsaw nsq"] + vco + [".ends", ""]
    out += [".subckt VCF vcc vr nin nenv nacc ntp6 nvcf nresw"] + vcf + [".ends", ""]
    vca = [l.replace(" SWI", " SWV") for l in vca]
    out += [".subckt VCA vcc vr nvcf nresw nenv naccw nout"] + vca + [".ends", ""]
    out += ba662 + ba662_models + [""]

    out += ["XVCO vcc vr nsaw nsq VCO",
            "* The WAVEFORM switch.",
            "BWAVE nwave 0 V = {sq} > 0.5 ? v(nsq) : v(nsaw)",
            "XVCF vcc vr nwave nenv naccw ntp6 nvcf nresw VCF",
            "XVCA vcc vr nvcf nresw nenv naccw nout VCA",
            "",
            ".ic v(xvco.nc33)=8 v(xvco.nb)=5.9",
            ".options method=trap reltol=1e-5 abstol=1e-12 vntol=1e-7",
            ".tran 2u {tend} 0 2u",
            ".end"]
    path = os.path.join(HERE, "voice.cir")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    print("wrote", path)


if __name__ == "__main__":
    main()
