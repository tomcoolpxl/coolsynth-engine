#pragma once

#include <algorithm>
#include <cmath>

#include "Tb303Voice.h"

/*
    The TB-303's lid tap (CoolSynth 303's Under the Lid): every drawn part's current and every drawn node's voltage of
    the voice at this moment, under the names of the lid drawing (CoolSynth's docs/design/mockups/coolsynth-303-lid.html,
    recorded by tools/tb303/mockup_currents.py from tools/tb303/voice.cir), in its order:

        <part>.i      resistors, capacitors, switches, sources; xvco.ic11a.i and xvco.ic11b.i the op-amps' outputs;
                      wave.i the WAVEFORM switch
        <part>.id     diodes, JFETs' drains; <part>.ig their gates
        <part>.ic, <part>.ib   transistors' collectors and bases
        <node>        the netlist's nodes, subcircuits flattened (xvcf.nb12l, xvca.xic15.t)

    Amps and volts as ngspice signs them: a current into the part's first node in the netlist (a transistor's into its
    collector and its base; a source's into its + terminal, so a source giving current reads negative).

    Instantaneous and exact from the circuits' state (CircuitWdf's tap): a capacitor's current from its waves, a
    junction's from the root's solve, a resistor's from its nodes. On the audio thread: no heap, no locks, some 7 us a
    call in the optimised Debug build (V3Tb303LidTests measures it), most of it every node's voltage (some 4500
    multiply-adds over the seven circuits). Either build of the voice; a Devil Fish's own parts (the current into Q10's
    base, R123's sustain input, the shunts its switches put across stock parts) are not drawn: its stock parts read as
    they are wired in it (R47 with its 4.7k across, say).

    Where the engine's circuit is not the netlist's, part for part (each case also beside its entry below):

        The stages' hand-offs   each stage's circuit as its last sample left it: the shaper at its last step of eight
                                (the raw saw and square, before the polyBLEP and the decimation: the filter's input nwave
                                is the decimated saw, two shaper steps later); the envelope at its last solve near 48 kHz
                                (its outputs reach the filter and the VCA interpolated, up to a voice sample behind it);
                                the filter's converter with Q12's emitters (ne12) and the VCA's buffer with pin 6 a sample
                                late, as their circuits take them. Settled (Tb303Voice::settle), the filter's input is
                                the saw's mean, 8.35 V, until the first sample
        Split parts             the VCA's input couplings C21 and R121, C22 and R122 load the filter's loop (into a fixed
                                5.333 V) and drive the chip's pin 3; read from the chip, where pin 3 moves (the loop's
                                C21 sits after its R121: its node is not the netlist's n21)
        Q1A (BA662)             in the envelope's circuit, a diode-wired transistor that also carries Q1B's base current
                                (IS (1 + 2 / hFE)): its collector and base the diode's current in that ratio. Q1B in the
                                chip, its base on pin 1's voltage as the envelope hands it
        Diode-wired transistors the ladder's rungs (Q13 to Q23) are diodes of IS (1 + 1 / hFE) with the 2N3904's base
                                recombination: the collector IS (e^(V / VT) - 1) from the rung's voltage, the base the
                                rest. The chip's Q3B, Q4B, Q6B, Q9B are diodes of IS (1 + 1 / hFE): collector and base
                                the diode's current split hFE to one (a PNP's negative)
        Base recombination      the filter's 2N3904s (Q11, Q18, Q19, the rungs, Q20) carry Gummel-Poon's ISE in their
                                own junctions (CoolSynth's D360); Q41, Q40, Q32 with the diode that stands for it; the
                                envelope's and the chip's other transistors have none (ngspice's QEM does, so their base
                                currents read low at uA)
        Q20                     a fixed current into the loop's tail (the source behind 1 GOhm, its current as drawn),
                                solved at rest with its base current; R112 and R109 one Thevenin source in the loop:
                                Q20's base node from their divider less its base current through their 5k, its emitter
                                from R108 (2.2k) and the tail with its base current
        The op-amps             each a 1 MHz single-pole macro (ideal follower out); its output current is what its
                                output node's parts draw (KCL), with D30 and D31 on IC11b's computed from their nodes
        D30, D31                not in the engine (off at every note): Shockley from their nodes, as vco.cir's 1N4148
        The saw core            C33's ramp exact (its current 10 nF of the 10.0038 nF the ramp discharges, nothing while
                                the thyristor holds it at the top); the thyristor (Q24, Q25, Q27, D25, R101) an event: its
                                currents read 0 (off: ngspice's leak some 100 nA between resets), its nodes the
                                reference's standing voltages (nb 5.715, nb27 11.476, nk 5.883, ne24 5.334 V). Q28 (J28)
                                a follower solved per step: its drain the shaper's draw plus R105's; its gate 0 (ngspice's
                                junction charge some 10 nA)
        Q39 (J39)               its gate on its source: 0
        Switches, controls      IC12's switches as 200 Ohm and 1 GOhm resistors (SWI, SWV's RON and ROFF); the CPU's
                                control lines (VSL, VNS, VACC) draw nothing; nacc, nsl, nns their logic levels
        Leaks                   the engine's 1 GOhm leaks (as SPICE's GMIN) are not drawn and not added to any part
*/
namespace coolsynth::synth::v3
{
    namespace lid
    {
        /** What a series reads: a node, a port's current (scaled), two ports' summed, an ideal source's current into
            its + terminal (minus what its node's ports draw), nothing, a constant, or a case of its own. */
        enum class Tap : unsigned char { node, port, ports, source, zero, constant, special, rung };
        enum class On : unsigned char { pitch, shaper, converter, loop, envelope, chip, buffer, none };
        enum Special : short { opAmpB, d31, d30, c33, j28, r105, r112, r109, nb20, ne20, nacc, nc33, nns, nsl, q20b, r108 };

        /** A series: how it is read, and for a port the nodes and kind it must have in its circuit (Tb303Lid::check;
            n1 -1 for either, value 0 for any). */
        struct Series
        {
            const char* name;
            Tap tap;
            On on;
            short index, aux;
            double scale;
            short n1, n2;
            char kind;
            double value;
        };

        constexpr Series node(const char* n, On on, int k) { return { n, Tap::node, on, static_cast<short>(k), 0, 1.0, 0, 0, 0, 0.0 }; }
        constexpr Series amps(const char* n, On on, int port, int n1, int n2, char kind, double scale = 1.0, double value = 0.0)
        {
            return { n, Tap::port, on, static_cast<short>(port), 0, scale, static_cast<short>(n1), static_cast<short>(n2), kind, value };
        }
        constexpr Series both(const char* n, On on, int port, int aux, int n1, int n2)
        {
            return { n, Tap::ports, on, static_cast<short>(port), static_cast<short>(aux), 1.0, static_cast<short>(n1), static_cast<short>(n2), 'n', 0.0 };
        }
        /** A ladder rung (a diode-wired 2N3904 in the loop): its collector (collector true) or its base current, the
            diode's current split as the transistor splits it, IS (e^(V / VT) - 1) into the collector, the rest (IS / hFE
            and the recombination) into the base. */
        constexpr Series rung(const char* n, int port, int n1, int n2, bool collector)
        {
            return { n, Tap::rung, On::loop, static_cast<short>(port), static_cast<short>(collector ? 1 : 0), 1.0,
                     static_cast<short>(n1), static_cast<short>(n2), 'n', 0.0 };
        }
        constexpr Series source(const char* n, On on, int k) { return { n, Tap::source, on, static_cast<short>(k), 0, 1.0, 0, 0, 0, 0.0 }; }
        constexpr Series zero(const char* n) { return { n, Tap::zero, On::none, 0, 0, 0.0, 0, 0, 0, 0.0 }; }
        constexpr Series fixed(const char* n, double volts) { return { n, Tap::constant, On::none, 0, 0, volts, 0, 0, 0, 0.0 }; }
        constexpr Series special(const char* n, Special s) { return { n, Tap::special, On::none, static_cast<short>(s), 0, 1.0, 0, 0, 0, 0.0 }; }

        // The split of a diode-wired transistor's current: collector hFE / (hFE + 1), base 1 / (hFE + 1).
        constexpr double npnC = 150.0 / 151.0, npnB = 1.0 / 151.0;         // the BA662's NPNs
        constexpr double pnpC = -50.0 / 51.0, pnpB = -1.0 / 51.0;          // its lateral PNPs, from emitter to base
        constexpr double q1aC = 150.0 / 152.0, q1aB = 1.0 / 152.0;         // Q1A, with Q1B's base current

        constexpr On P = On::pitch, H = On::shaper, C = On::converter, L = On::loop, E = On::envelope, X = On::chip, B = On::buffer;

        // Every series in the drawing's order (its lidData names). Port numbers count each circuit's build: a
        // transistor two ports (base-emitter, collector-emitter), a diode one, an op-amp's macro two.
        inline constexpr Series table[] = {
            // ---- The VCO: pitch (1 12 V, 2 5.333 V, 3 the ladder, 4 Q26's 8 V; 5 ndac ... 18), shaper (3 nsaw ... 7 nsq)
            source("xvco.vdac.i", P, 3),
            amps("xvco.rdac.i", P, 0, 3, 5, 'r'),
            zero("xvco.vsl.i"),                                  // a control line
            zero("xvco.vns.i"),
            amps("xvco.s8.i", P, 1, 5, 6, 'r'),
            amps("xvco.s10.i", P, 2, 6, 7, 'r'),
            amps("xvco.c35.i", P, 3, 6, 0, 'c'),
            special("xvco.ic11b.i", opAmpB),                     // what ncv's parts draw, D30 and D31's too
            amps("xvco.r107.i", P, 6, 7, 8, 'r'),
            special("xvco.d31.id", d31),
            special("xvco.d30.id", d30),
            amps("xvco.r103.i", P, 7, 1, 10, 'r'),
            amps("xvco.rtm4a.i", P, 8, 2, 11, 'r'),
            amps("xvco.rtm4b.i", P, 9, 11, 1, 'r'),
            amps("xvco.r102.i", P, 10, 11, 10, 'r'),
            source("xvco.ic11a.i", P, 12),                       // what n11a's parts draw
            amps("xvco.c34.i", P, 13, 12, 10, 'c'),
            amps("xvco.r104.i", P, 14, 12, 14, 'r'),
            amps("xvco.q26a.ic", P, 16, 10, 14, 'n'),
            amps("xvco.q26a.ib", P, 15, 2, 14, 'n'),
            amps("xvco.q26b.ic", P, 18, 18, 14, 'n'),            // its collector held at 8 V, read across 1k
            amps("xvco.q26b.ib", P, 17, 16, 14, 'n'),
            amps("xvco.rtm5.i", P, 20, 7, 15, 'r'),
            amps("xvco.r106.i", P, 21, 15, 16, 'r'),
            amps("xvco.r100.i", P, 22, 16, 2, 'r'),
            amps("xvco.r118.i", P, 23, 16, 17, 'r'),
            amps("xvco.rv2a.i", P, 24, 2, 17, 'r'),
            amps("xvco.rv2b.i", P, 25, 17, 0, 'r'),
            special("xvco.c33.i", c33),
            zero("xvco.q25.ic"),                                 // the thyristor: an event, off between resets
            zero("xvco.q25.ib"),
            zero("xvco.q27.ic"),
            zero("xvco.q27.ib"),
            zero("xvco.d25.id"),
            zero("xvco.q24.ic"),
            zero("xvco.q24.ib"),
            zero("xvco.r101.i"),
            special("xvco.j28.id", j28),
            zero("xvco.j28.ig"),
            special("xvco.r105.i", r105),
            amps("xvco.r35.i", H, 2, 3, 4, 'r'),
            amps("xvco.c10.i", H, 3, 3, 5, 'c'),
            amps("xvco.r34.i", H, 4, 5, 4, 'r'),
            amps("xvco.r45.i", H, 5, 1, 6, 'r'),
            amps("xvco.c11.i", H, 6, 6, 0, 'c'),
            amps("xvco.q8.ic", H, 8, 7, 6, 'n'),
            amps("xvco.q8.ib", H, 7, 4, 6, 'n'),
            amps("xvco.r36.i", H, 9, 7, 2, 'r'),
            source("wave.i", L, 3),                              // the loop's input: what R62 draws
            // ---- The VCF: loop (1 12 V, 2 5.333 V, 3 nwave; 4 vf ... 34), converter (1 ... 5 inputs; 6 nb9 ... 19 n13)
            amps("xvcf.r95.i", L, 0, 1, 4, 'r'),
            amps("xvcf.c28.i", L, 1, 4, 0, 'c'),
            amps("xvcf.r62.i", L, 2, 3, 5, 'r'),
            amps("xvcf.c17.i", L, 3, 5, 6, 'c'),
            amps("xvcf.r68.i", L, 4, 2, 7, 'r'),
            amps("xvcf.c16.i", L, 5, 7, 0, 'c'),
            amps("xvcf.r70.i", L, 6, 6, 7, 'r'),
            amps("xvcf.r98.i", L, 7, 8, 7, 'r'),
            amps("xvcf.q12l.ic", L, 9, 9, 18, 'n'),
            amps("xvcf.q12l.ib", L, 8, 6, 18, 'n'),
            amps("xvcf.q12r.ic", L, 11, 13, 18, 'n'),
            amps("xvcf.q12r.ib", L, 10, 8, 18, 'n'),
            rung("xvcf.q13.ic", 12, 10, 9, true),                // the rungs: diodes, split as the transistor splits
            rung("xvcf.q13.ib", 12, 10, 9, false),
            rung("xvcf.q14.ic", 13, 11, 10, true),
            rung("xvcf.q14.ib", 13, 11, 10, false),
            rung("xvcf.q16.ic", 14, 12, 11, true),
            rung("xvcf.q16.ib", 14, 12, 11, false),
            rung("xvcf.q15.ic", 15, 14, 13, true),
            rung("xvcf.q15.ib", 15, 14, 13, false),
            rung("xvcf.q17.ic", 16, 15, 14, true),
            rung("xvcf.q17.ib", 16, 15, 14, false),
            rung("xvcf.q23.ic", 17, 16, 15, true),
            rung("xvcf.q23.ib", 17, 16, 15, false),
            amps("xvcf.c18.i", L, 24, 9, 13, 'c'),
            amps("xvcf.c19.i", L, 25, 10, 14, 'c'),
            amps("xvcf.c24.i", L, 26, 11, 15, 'c'),
            amps("xvcf.c26.i", L, 27, 12, 16, 'c'),
            amps("xvcf.q22l.ic", L, 29, 4, 12, 'n'),
            amps("xvcf.q22l.ib", L, 28, 17, 12, 'n'),
            amps("xvcf.q22r.ic", L, 31, 4, 16, 'n'),
            amps("xvcf.q22r.ib", L, 30, 17, 16, 'n'),
            amps("xvcf.r115.i", L, 32, 4, 17, 'r'),
            amps("xvcf.c30.i", L, 33, 17, 0, 'c'),
            amps("xvcf.r116.i", L, 34, 2, 19, 'r'),
            amps("xvcf.c29.i", L, 35, 19, 0, 'c'),
            amps("xvcf.c27.i", L, 37, 16, 20, 'c'),
            amps("xvcf.c25.i", L, 38, 12, 21, 'c'),
            amps("xvcf.r113.i", L, 39, 20, 19, 'r'),
            amps("xvcf.r114.i", L, 40, 21, 19, 'r'),
            amps("xvcf.q21l.ic", L, 42, 22, 24, 'n'),
            amps("xvcf.q21l.ib", L, 41, 20, 24, 'n'),
            amps("xvcf.q21r.ic", L, 44, 23, 24, 'n'),
            amps("xvcf.q21r.ib", L, 43, 21, 24, 'n'),
            amps("xvcf.r111.i", L, 45, 4, 22, 'r'),
            amps("xvcf.r110.i", L, 46, 4, 23, 'r'),
            special("xvcf.r112.i", r112),                        // R112 and R109: one 20k in the loop
            special("xvcf.r109.i", r109),
            amps("xvcf.q20.ic", L, 47, 24, 0, 'v'),              // Q20: the tail's fixed current
            special("xvcf.q20.ib", q20b),                        // its base current as the filter solved it
            special("xvcf.r108.i", r108),                        // the tail and its base current
            amps("xvcf.q19.ic", L, 49, 4, 25, 'n'),
            amps("xvcf.q19.ib", L, 48, 23, 25, 'n'),
            amps("xvcf.r96.i", L, 50, 25, 0, 'r'),
            amps("xvcf.c14.i", L, 51, 25, 26, 'c'),
            amps("xvcf.rres3.i", L, 52, 26, 27, 'r'),
            amps("xvcf.rres1.i", L, 53, 27, 0, 'r'),
            amps("xvcf.c15.i", L, 56, 27, 29, 'c'),
            amps("xvcf.r67.i", L, 57, 29, 30, 'r'),
            amps("xvcf.r66.i", L, 58, 2, 31, 'r'),
            amps("xvcf.c22.i", L, 59, 31, 0, 'c'),
            amps("xvcf.r99.i", L, 60, 31, 30, 'r'),
            amps("xvcf.q18.ic", L, 62, 1, 32, 'n'),
            amps("xvcf.q18.ib", L, 61, 30, 32, 'n'),
            amps("xvcf.r94.i", L, 63, 32, 0, 'r'),
            amps("xvcf.c23.i", L, 64, 32, 33, 'c'),
            amps("xvcf.r97.i", L, 65, 33, 8, 'r'),
            amps("xvcf.r69.i", C, 8, 10, 5, 'r', -1.0),          // built from nc11 to ne12
            amps("xvcf.q11.ic", C, 7, 10, 7, 'n'),
            amps("xvcf.q11.ib", C, 6, 9, 7, 'n'),
            amps("xvcf.r65.i", C, 0, 2, 6, 'r'),
            amps("xvcf.r64.i", C, 1, 6, 0, 'r'),
            amps("xvcf.q9.ic", C, 3, 0, 7, 'n'),
            amps("xvcf.q9.ib", C, 2, 6, 7, 'n'),
            amps("xvcf.q10.ic", C, 5, 0, 9, 'n'),
            amps("xvcf.q10.ib", C, 4, 8, 9, 'n'),
            amps("xvcf.rcut3.i", C, 9, 1, 11, 'r'),
            amps("xvcf.rcut1.i", C, 10, 11, 12, 'r'),
            amps("xvcf.r47.i", C, 11, 12, 7, 'r'),
            amps("xvcf.r73.i", C, 12, 11, 13, 'r'),
            amps("xvcf.rtm3.i", C, 13, 13, 9, 'r'),
            amps("xvcf.r71.i", C, 14, 8, 7, 'r'),
            amps("xvcf.renv3.i", C, 15, 3, 14, 'r'),
            amps("xvcf.renv1.i", C, 16, 14, 15, 'r'),
            amps("xvcf.r61.i", C, 17, 15, 7, 'r'),
            amps("xvcf.r63.i", C, 18, 14, 8, 'r'),
            amps("xvcf.d24.id", C, 19, 4, 16, 'n'),
            amps("xvcf.r46.i", C, 20, 16, 17, 'r'),
            amps("xvcf.rresb1.i", C, 21, 17, 18, 'r'),
            amps("xvcf.rresb3.i", C, 22, 18, 19, 'r'),
            amps("xvcf.c13.i", C, 23, 19, 0, 'c'),
            amps("xvcf.r72.i", C, 24, 18, 8, 'r'),
            // ---- The VCA: envelope (1 12 V, 2 5.333 V, 3 ng, 4 VQ9; 5 nb37 ... 35 ne32), chip (1 ... 5 inputs; 6 e1b
            // ... 21 nq), buffer (1 12 V, 2 pin 6; 3 Q7A's base, 4 e7, 5 b9, 6 nvout, 7 nout)
            source("xvca.vg.i", E, 3),
            amps("xvca.r146.i", E, 0, 3, 5, 'r'),
            amps("xvca.r144.i", E, 1, 5, 0, 'r'),
            amps("xvca.q37.ic", E, 3, 6, 0, 'n'),
            amps("xvca.q37.ib", E, 2, 5, 0, 'n'),
            amps("xvca.r147.i", E, 4, 1, 7, 'r'),
            amps("xvca.r142.i", E, 5, 1, 8, 'r'),
            amps("xvca.c72.i", E, 6, 1, 8, 'c'),
            amps("xvca.r143.i", E, 7, 8, 6, 'r'),
            amps("xvca.q36.ic", E, 9, 9, 7, 'n'),
            amps("xvca.q36.ib", E, 8, 8, 7, 'n'),
            amps("xvca.r145.i", E, 10, 9, 0, 'r'),
            amps("xvca.r150.i", E, 11, 1, 10, 'r'),
            amps("xvca.c55.i", E, 12, 10, 0, 'c'),
            amps("xvca.r149.i", E, 13, 1, 11, 'r'),
            amps("xvca.r148.i", E, 14, 11, 12, 'r'),
            amps("xvca.c54.i", E, 15, 12, 6, 'c'),
            amps("xvca.q38.ic", E, 17, 13, 10, 'n'),
            amps("xvca.q38.ib", E, 16, 11, 10, 'n'),
            amps("xvca.r151.i", E, 18, 13, 0, 'r'),
            amps("xvca.d37.id", E, 19, 13, 14, 'n'),
            amps("xvca.r152.i", E, 20, 14, 15, 'r'),
            amps("xvca.c62.i", E, 21, 15, 0, 'c'),
            amps("xvca.q41.ic", E, 23, 1, 20, 'n'),
            both("xvca.q41.ib", E, 22, 24, 15, 20),              // with its recombination diode
            amps("xvca.q40.ic", E, 26, 1, 21, 'n'),
            both("xvca.q40.ib", E, 25, 27, 20, 21),
            amps("xvca.j39.id", E, 28, 21, 0, 'n'),
            zero("xvca.j39.ig"),                                 // its gate on its source
            zero("xvca.vacc.i"),                                 // a control line
            amps("xvca.r138.i", E, 30, 15, 16, 'r'),
            amps("xvca.rdec.i", E, 31, 16, 17, 'r'),
            amps("xvca.s12b.i", E, 32, 16, 17, 'r'),
            amps("xvca.d28.id", E, 33, 17, 18, 'n'),
            amps("xvca.d29.id", E, 34, 18, 19, 'n'),
            amps("xvca.r117.i", E, 35, 2, 19, 'r'),
            amps("xvca.d26.id", E, 36, 19, 0, 'n'),
            amps("xvca.s12a.i", E, 37, 21, 22, 'r'),
            amps("xvca.racc3.i", E, 38, 22, 23, 'r'),
            amps("xvca.racc1.i", E, 39, 23, 0, 'r'),
            amps("xvca.r119.i", E, 40, 23, 24, 'r'),
            amps("xvca.c36.i", E, 42, 24, 0, 'c'),
            amps("xvca.r120.i", E, 43, 24, 25, 'r'),
            amps("xvca.r126.i", X, 22, 2, 15, 'r'),
            amps("xvca.c37.i", X, 23, 15, 0, 'c'),
            amps("xvca.r125.i", X, 24, 15, 16, 'r'),
            amps("xvca.r124.i", X, 25, 15, 17, 'r'),
            amps("xvca.c21.i", X, 26, 3, 18, 'c'),               // the chip's copies (the loop carries the others)
            amps("xvca.r121.i", X, 27, 18, 17, 'r'),
            amps("xvca.c22.i", X, 28, 4, 19, 'c'),
            amps("xvca.r122.i", X, 29, 19, 17, 'r'),
            amps("xvca.d35.id", E, 47, 9, 28, 'n'),
            amps("xvca.r133.i", E, 48, 28, 29, 'r'),
            amps("xvca.r131.i", E, 49, 29, 30, 'r'),
            amps("xvca.q31.ic", E, 51, 26, 30, 'n'),
            amps("xvca.q31.ib", E, 50, 31, 30, 'n'),
            amps("xvca.r123.i", E, 52, -1, 31, 'r'),             // from 12 V, or a Devil Fish's sustain input
            amps("xvca.c42.i", E, 53, 31, 0, 'c'),
            amps("xvca.r132.i", E, 54, 31, 32, 'r'),
            amps("xvca.d36.id", E, 55, 13, 33, 'n'),
            amps("xvca.r134.i", E, 56, 33, 34, 'r'),
            amps("xvca.c41.i", E, 57, 34, 0, 'c'),
            amps("xvca.q32.ic", E, 59, 32, 35, 'n'),
            both("xvca.q32.ib", E, 58, 60, 34, 35),
            amps("xvca.r130.i", E, 61, 35, 0, 'r'),
            amps("xvca.d27v.id", E, 44, 25, 26, 'n'),
            amps("xvca.xic15.q1a.ic", E, 45, 26, 27, 'n', q1aC), // in the envelope: a diode with Q1B's base current
            amps("xvca.xic15.q1a.ib", E, 45, 26, 27, 'n', q1aB),
            amps("xvca.xic15.r1a.i", E, 46, 27, 0, 'r'),
            amps("xvca.xic15.q1b.ic", X, 1, 7, 6, 'n'),
            amps("xvca.xic15.q1b.ib", X, 0, 5, 6, 'n'),
            amps("xvca.xic15.r1b.i", X, 2, 6, 0, 'r'),
            amps("xvca.xic15.q2a.ic", X, 4, 8, 7, 'n'),
            amps("xvca.xic15.q2a.ib", X, 3, 16, 7, 'n'),
            amps("xvca.xic15.q2b.ic", X, 6, 9, 7, 'n'),
            amps("xvca.xic15.q2b.ib", X, 5, 17, 7, 'n'),
            amps("xvca.xic15.q3a.ic", X, 8, 8, 1, 'n'),
            amps("xvca.xic15.q3a.ib", X, 7, 10, 1, 'n'),
            amps("xvca.xic15.q3b.ic", X, 9, 1, 10, 'n', pnpC),   // diode-wired
            amps("xvca.xic15.q3b.ib", X, 9, 1, 10, 'n', pnpB),
            amps("xvca.xic15.q5b.ic", X, 11, 12, 10, 'n'),
            amps("xvca.xic15.q5b.ib", X, 10, 8, 10, 'n'),
            amps("xvca.xic15.q4a.ic", X, 13, 9, 1, 'n'),
            amps("xvca.xic15.q4a.ib", X, 12, 11, 1, 'n'),
            amps("xvca.xic15.q4b.ic", X, 14, 1, 11, 'n', pnpC),
            amps("xvca.xic15.q4b.ib", X, 14, 1, 11, 'n', pnpB),
            amps("xvca.xic15.q5a.ic", X, 16, 14, 11, 'n'),
            amps("xvca.xic15.q5a.ib", X, 15, 9, 11, 'n'),
            amps("xvca.xic15.q6a.ic", X, 18, 12, 0, 'n'),
            amps("xvca.xic15.q6a.ib", X, 17, 13, 0, 'n'),
            amps("xvca.xic15.q6b.ic", X, 19, 13, 0, 'n', npnC),
            amps("xvca.xic15.q6b.ib", X, 19, 13, 0, 'n', npnB),
            amps("xvca.xic15.q7b.ic", X, 21, 14, 13, 'n'),
            amps("xvca.xic15.q7b.ib", X, 20, 12, 13, 'n'),
            amps("xvca.xic15.r20.i", B, 6, 1, 5, 'r'),
            amps("xvca.xic15.q9b.ic", B, 5, 5, 0, 'n', npnC),
            amps("xvca.xic15.q9b.ib", B, 5, 5, 0, 'n', npnB),
            amps("xvca.xic15.q9a.ic", B, 4, 4, 0, 'n'),
            amps("xvca.xic15.q9a.ib", B, 3, 5, 0, 'n'),
            amps("xvca.xic15.q7a.ic", B, 2, 1, 4, 'n'),
            amps("xvca.xic15.q7a.ib", B, 1, 3, 4, 'n'),
            amps("xvca.xic15.q10a.ic", B, 8, 0, 6, 'n'),
            amps("xvca.xic15.q10a.ib", B, 7, 4, 6, 'n'),
            amps("xvca.xic15.r21.i", B, 9, 1, 6, 'r'),
            amps("xvca.r128.i", X, 30, 14, 20, 'r', 1.0, 220.0e3),
            amps("xvca.r127.i", X, 31, 14, 20, 'r', 1.0, 47.0e3),
            amps("xvca.r129.i", X, 32, 20, 21, 'r'),
            amps("xvca.r162.i", X, 33, 2, 21, 'r'),
            amps("xvca.c40.i", X, 34, 21, 0, 'c'),
            amps("xvca.c38.i", B, 10, 6, 7, 'c'),
            amps("xvca.rmix.i", B, 11, 7, 0, 'r'),
            // ---- The nodes, as the drawing sorts them
            node("naccw", E, 23),
            node("nenv", E, 21),
            node("nout", B, 7),
            node("nresw", L, 27),
            node("nsaw", H, 3),                                  // the shaper's raw saw
            node("nsq", H, 7),
            node("ntp6", L, 25),
            node("nvcf", L, 26),
            node("nwave", L, 3),                                 // the filter's input: the decimated saw or square
            node("vcc", L, 1),
            node("vr", L, 2),
            node("xvca.n152", E, 14),
            node("xvca.n21", X, 18),
            node("xvca.n22", X, 19),
            node("xvca.n27", E, 25),
            node("xvca.n35", E, 28),
            node("xvca.n36", E, 24),
            node("xvca.n36d", E, 33),
            node("xvca.n54", E, 12),
            special("xvca.nacc", nacc),
            node("xvca.nacc3", E, 22),
            node("xvca.nb31", E, 31),
            node("xvca.nb32", E, 34),
            node("xvca.nb36", E, 8),
            node("xvca.nb37", E, 5),
            node("xvca.nb38", E, 11),
            node("xvca.nc32", E, 32),
            node("xvca.nc37", E, 6),
            node("xvca.nc38", E, 13),
            node("xvca.nc62", E, 15),
            node("xvca.ndec", E, 16),
            node("xvca.ne31", E, 30),
            node("xvca.ne32", E, 35),
            node("xvca.ne36", E, 7),
            node("xvca.ne38", E, 10),
            node("xvca.ne41", E, 20),
            node("xvca.ng", E, 3),
            node("xvca.ngate", E, 9),
            node("xvca.niabc", E, 26),
            node("xvca.nin2", X, 16),
            node("xvca.nin3", X, 17),
            node("xvca.nm", X, 15),
            node("xvca.nn", X, 20),
            node("xvca.nn2", E, 19),
            node("xvca.nn3", E, 18),
            node("xvca.nn4", E, 17),
            node("xvca.nota", X, 14),
            node("xvca.nq", X, 21),
            node("xvca.nvout", B, 6),
            node("xvca.nx31", E, 29),
            node("xvca.xic15.b3", X, 10),
            node("xvca.xic15.b4", X, 11),
            node("xvca.xic15.b6", X, 13),
            node("xvca.xic15.b9", B, 5),
            node("xvca.xic15.ca", X, 8),
            node("xvca.xic15.cb", X, 9),
            node("xvca.xic15.e1a", E, 27),
            node("xvca.xic15.e1b", X, 6),
            node("xvca.xic15.e7", B, 4),
            node("xvca.xic15.t", X, 7),
            node("xvca.xic15.x", X, 12),
            node("xvcf.n13", C, 19),
            node("xvcf.n15", L, 29),
            node("xvcf.n16", L, 7),
            node("xvcf.n17", L, 5),
            node("xvcf.n22", L, 31),
            node("xvcf.n23", L, 33),
            node("xvcf.n61", C, 15),
            node("xvcf.nb10", C, 8),
            node("xvcf.nb11", C, 9),
            node("xvcf.nb12l", L, 6),
            node("xvcf.nb12r", L, 8),
            node("xvcf.nb18", L, 30),
            special("xvcf.nb20", nb20),
            node("xvcf.nb21l", L, 20),
            node("xvcf.nb21r", L, 21),
            node("xvcf.nb22", L, 17),
            node("xvcf.nb9", C, 6),
            node("xvcf.nbias", L, 19),
            node("xvcf.nc11", C, 10),
            node("xvcf.nc21l", L, 22),
            node("xvcf.nc21r", L, 23),
            node("xvcf.ncut1", C, 12),
            node("xvcf.ncutw", C, 11),
            node("xvcf.nd24", C, 16),
            node("xvcf.ne12", L, 18),
            node("xvcf.ne18", L, 32),
            special("xvcf.ne20", ne20),
            node("xvcf.ne21", L, 24),
            node("xvcf.nenvw", C, 14),
            node("xvcf.nl1", L, 9),
            node("xvcf.nl2", L, 10),
            node("xvcf.nl3", L, 11),
            node("xvcf.nl4", L, 12),
            node("xvcf.nr1", L, 13),
            node("xvcf.nr2", L, 14),
            node("xvcf.nr3", L, 15),
            node("xvcf.nr4", L, 16),
            node("xvcf.nres1b", C, 17),
            node("xvcf.nresbw", C, 18),
            node("xvcf.ntm3", C, 13),
            node("xvcf.nvq9", C, 7),
            node("xvcf.vf", L, 4),
            node("xvco.n11a", P, 12),
            node("xvco.n35", P, 6),
            node("xvco.n5", P, 15),
            fixed("xvco.nb", 5.715),                             // the thyristor's standing voltages (ngspice's)
            fixed("xvco.nb27", 11.476),
            node("xvco.nb8", H, 4),
            node("xvco.nbase", P, 16),
            node("xvco.nc10", H, 5),
            special("xvco.nc33", nc33),
            node("xvco.ncv", P, 7),
            node("xvco.ndac", P, 5),
            node("xvco.ndac0", P, 3),
            fixed("xvco.ne24", 5.334),
            node("xvco.ne8", H, 6),
            node("xvco.nem", P, 14),
            fixed("xvco.nk", 5.883),
            node("xvco.nm11b", P, 8),
            special("xvco.nns", nns),
            node("xvco.nref", P, 10),
            special("xvco.nsl", nsl),
            node("xvco.ntm4", P, 11),
            node("xvco.ntune", P, 17),
        };
        inline constexpr int tableSize = static_cast<int>(sizeof(table) / sizeof(table[0]));
        static_assert(tableSize == 377, "the drawing's 377 series");
    }

    /** The voice's lid tap: Tb303Lid::read fills count() floats, amps and volts, in name(i)'s order. */
    struct Tb303Lid
    {
        [[nodiscard]] static constexpr int count() noexcept { return lid::tableSize; }
        [[nodiscard]] static constexpr const char* name(int i) noexcept
        {
            return i >= 0 && i < lid::tableSize ? lid::table[i].name : "";
        }

        /** Every series at this moment (call between samples, from the audio thread: no heap, no locks). */
        static void read(const Tb303Voice& voice, float* out) noexcept
        {
            const Tb303Oscillator& osc = voice.oscillator();
            const Tb303Filter& vcf = voice.vcf();
            const Tb303Envelope& env = voice.envelope();
            const Tb303Vca& vca = voice.amplifier();
            Volts v;
            osc.pitch.nodeVoltages(v.pitch);
            osc.shaper.nodeVoltages(v.shaper);
            vcf.converter.nodeVoltages(v.converter);
            vcf.loop.nodeVoltages(v.loop);
            env.circuit.nodeVoltages(v.envelope);
            vca.chip.nodeVoltages(v.chip);
            vca.buffer.nodeVoltages(v.buffer);
            for (int i = 0; i < lid::tableSize; ++i)
            {
                const lid::Series& s = lid::table[i];
                double x = 0.0;
                switch (s.tap)
                {
                    case lid::Tap::node: x = nodeOf(v, s.on, s.index); break;
                    case lid::Tap::port: x = s.scale * currentOf(voice, v, s.on, s.index); break;
                    case lid::Tap::ports: x = currentOf(voice, v, s.on, s.index) + currentOf(voice, v, s.on, s.aux); break;
                    case lid::Tap::source: x = -drawOf(voice, v, s.on, s.index); break;
                    case lid::Tap::zero: x = 0.0; break;
                    case lid::Tap::constant: x = s.scale; break;
                    case lid::Tap::special: x = special(voice, v, static_cast<lid::Special>(s.index)); break;
                    case lid::Tap::rung:
                    {
                        const double ic = Tb303Filter::npnSaturation * (std::exp(std::min((v.loop[s.n1] - v.loop[s.n2]) / Tb303Filter::thermal, 80.0)) - 1.0);
                        x = s.aux != 0 ? ic : currentOf(voice, v, s.on, s.index) - ic;
                        break;
                    }
                }
                out[i] = static_cast<float>(x);
            }
        }

        /** The map against the voice's circuits: the number of series whose port is not built as the map has it
            (its nodes, kind, value), 0 when all are; the first such name in firstWrong. For the tests. */
        static int check(const Tb303Voice& voice, const char** firstWrong = nullptr) noexcept
        {
            int wrong = 0;
            for (int i = 0; i < lid::tableSize; ++i)
            {
                const lid::Series& s = lid::table[i];
                if (s.tap != lid::Tap::port && s.tap != lid::Tap::ports && s.tap != lid::Tap::rung)
                    continue;
                bool ok = matches(portOf(voice, s.on, s.index), s.n1, s.n2, s.kind, s.value);
                if (s.tap == lid::Tap::ports)
                    ok = ok && matches(portOf(voice, s.on, s.aux), s.n1, s.n2, 'n', 0.0);
                if (! ok)
                {
                    if (wrong == 0 && firstWrong != nullptr)
                        *firstWrong = s.name;
                    ++wrong;
                }
            }
            return wrong;
        }

    private:
        struct PortInfo
        {
            int n1 = 0, n2 = 0;
            char kind = '?';
            double value = 0.0;
        };

        struct Volts // each circuit's nodes, [0] ground
        {
            double pitch[19], shaper[8], converter[20], loop[35], envelope[37], chip[22], buffer[8];
        };

        static double nodeOf(const Volts& v, lid::On on, int k) noexcept
        {
            switch (on)
            {
                case lid::On::pitch: return v.pitch[k];
                case lid::On::shaper: return v.shaper[k];
                case lid::On::converter: return v.converter[k];
                case lid::On::loop: return v.loop[k];
                case lid::On::envelope: return v.envelope[k];
                case lid::On::chip: return v.chip[k];
                case lid::On::buffer: return v.buffer[k];
                case lid::On::none: break;
            }
            return 0.0;
        }

        static double currentOf(const Tb303Voice& voice, const Volts& v, lid::On on, int port) noexcept
        {
            switch (on)
            {
                case lid::On::pitch: return voice.oscillator().pitch.portCurrent(port, v.pitch);
                case lid::On::shaper: return voice.oscillator().shaper.portCurrent(port, v.shaper);
                case lid::On::converter: return voice.vcf().converter.portCurrent(port, v.converter);
                case lid::On::loop: return voice.vcf().loop.portCurrent(port, v.loop);
                case lid::On::envelope: return voice.envelope().circuit.portCurrent(port, v.envelope);
                case lid::On::chip: return voice.amplifier().chip.portCurrent(port, v.chip);
                case lid::On::buffer: return voice.amplifier().buffer.portCurrent(port, v.buffer);
                case lid::On::none: break;
            }
            return 0.0;
        }

        static double drawOf(const Tb303Voice& voice, const Volts& v, lid::On on, int k) noexcept
        {
            switch (on)
            {
                case lid::On::pitch: return voice.oscillator().pitch.nodeCurrent(k, v.pitch);
                case lid::On::loop: return voice.vcf().loop.nodeCurrent(k, v.loop);
                case lid::On::envelope: return voice.envelope().circuit.nodeCurrent(k, v.envelope);
                default: break; // no source of the drawing's elsewhere
            }
            return 0.0;
        }

        template <class Circuit>
        static PortInfo infoOf(const Circuit& c, int port) noexcept
        {
            const auto p = c.portInfo(port);
            return { p.n1, p.n2, p.kind, p.value };
        }

        static PortInfo portOf(const Tb303Voice& voice, lid::On on, int port) noexcept
        {
            switch (on)
            {
                case lid::On::pitch: return infoOf(voice.oscillator().pitch, port);
                case lid::On::shaper: return infoOf(voice.oscillator().shaper, port);
                case lid::On::converter: return infoOf(voice.vcf().converter, port);
                case lid::On::loop: return infoOf(voice.vcf().loop, port);
                case lid::On::envelope: return infoOf(voice.envelope().circuit, port);
                case lid::On::chip: return infoOf(voice.amplifier().chip, port);
                case lid::On::buffer: return infoOf(voice.amplifier().buffer, port);
                case lid::On::none: break;
            }
            return {};
        }

        static bool matches(const PortInfo& p, int n1, int n2, char kind, double value) noexcept
        {
            return (n1 < 0 || p.n1 == n1) && p.n2 == n2 && p.kind == kind
                   && (value == 0.0 || std::abs(p.value - value) <= 1.0e-9 * value);
        }

        /** A 1N4148 (vco.cir's DEM: IS 2.52 nA, N 1.752) at this voltage. */
        static double dem(double volts) noexcept
        {
            return Tb303Filter::diodeSaturation * (std::exp(std::min(volts / (Tb303Filter::diodeEmission * Tb303Filter::thermal), 80.0)) - 1.0);
        }

        static double special(const Tb303Voice& voice, const Volts& v, lid::Special which) noexcept
        {
            const Tb303Oscillator& osc = voice.oscillator();
            const Tb303Filter& vcf = voice.vcf();
            switch (which)
            {
                case lid::opAmpB: // IC11b's output: what ncv's parts draw, D31 (to 5.333 V) and D30 (from ground) too
                    return -(osc.pitch.nodeCurrent(7, v.pitch) + dem(v.pitch[7] - v.pitch[2]) - dem(-v.pitch[7]));
                case lid::d31: return dem(v.pitch[7] - v.pitch[2]);
                case lid::d30: return dem(-v.pitch[7]);
                case lid::c33: // 10 nF of the ramp's 10.0038 nF; held at the top after a reset, still
                    return osc.holdLeft > 0.0 ? 0.0 : -(10.0e-9 / Tb303Oscillator::timing) * std::max(0.0, osc.current);
                case lid::j28: // Q28's drain: what the shaper draws from its source (behind the port), and R105's
                    return v.shaper[3] / Tb303Oscillator::r105 - osc.shaper.portCurrent(osc.sFollower, v.shaper);
                case lid::r105: return v.shaper[3] / Tb303Oscillator::r105;
                case lid::r112: return (v.loop[19] - q20Base(voice, v)) / 10.0e3;
                case lid::r109: return q20Base(voice, v) / 10.0e3;
                case lid::nb20: return q20Base(voice, v);
                case lid::ne20: return 2.2e3 * (vcf.loop.portCurrent(vcf.sTail, v.loop) + vcf.tailBaseAmps);
                case lid::q20b: return vcf.tailBaseAmps;
                case lid::r108: return vcf.loop.portCurrent(vcf.sTail, v.loop) + vcf.tailBaseAmps;
                case lid::nacc: return voice.envelope().accented ? 12.0 : 0.0;
                case lid::nc33: return osc.c33;
                case lid::nns: return osc.sliding ? 0.0 : 1.0;
                case lid::nsl: return osc.sliding ? 1.0 : 0.0;
            }
            return 0.0;
        }

        /** Q20's base: R112 and R109's divider from the loop's nbias, less Q20's base current through their 5k. */
        static double q20Base(const Tb303Voice& voice, const Volts& v) noexcept
        {
            return 0.5 * v.loop[19] - 5.0e3 * voice.vcf().tailBaseAmps;
        }
    };
}
