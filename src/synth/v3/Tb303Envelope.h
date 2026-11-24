#pragma once

#include <algorithm>
#include <cmath>

#include "CircuitWdf.h"
#include "Tb303Filter.h"

/*
    The Roland TB-303's envelope and the VCA's control current as one circuit (D275, D293), from the service notes
    (p. 5) and tools/tb303/vca.cir, its reference (env.cir's envelope with the VCA's control around it):

        Gate        the CPU's 5 V through R146 and R144 into Q37; Q36 makes GATE (R145)
        One-shot    C54 and R148 kick Q38 on at each gate; Q38 charges C62 (1 uF) through D37 and R152
        Envelope    Q41 and Q40 follow C62 over Q39, a JFET sink (gate on source); C62 falls through R138 (68k) and
                    DECAY (VR6, 1 M, audio taper) to the diode string D28, D29, D26 (R117); on an accent IC12B shorts
                    DECAY and IC12A puts the envelope on the ACCENT knob (VR7, 50k), whose wiper feeds the filter (D24,
                    the filter's own) and, through R119, C36, R120 and D27, the VCA's pin 1. ENV MOD's load (60k to
                    the filter's 3.31 V) as the reference has it
        Control     Q31 from GATE (D35, R133, R131) into the BA662's pin 1, its base on C42 (1 uF, refilled through
                    R123), which Q32 empties at each trigger (D36 from Q38's collector, R134, C41); pin 1 as the
                    chip's Q1A, a diode-wired transistor over 500 Ohm (with Q1B's base current: IS (1 + 2 / hFE))

    Slow, so it runs at a quarter of the voice's rate (its fastest part, C62's charge through R152, is 100 us), its
    outputs interpolated linearly to the voice's samples (some 60 us late at the attack). Its own Jacobian kept between
    steps (the chord method, D281). The NPNs that run at nanoamps (Q41, Q40, Q32) carry Gummel-Poon's base
    recombination (ISE 6.7 fA, NE 1.26) as a diode across their base and emitter: without it C41 held Q32 on after a
    trigger and C42 refilled 13 % slow. Leaks of 1 GOhm, as SPICE's GMIN, on the nodes only junctions leave.
    Outputs: the envelope (ENV MOD's top), the ACCENT knob's wiper, GATE, and pin 1's voltage (Q1B's base: the VCA).
*/
namespace coolsynth::synth::v3
{
    class Tb303Envelope final
    {
    public:
        static constexpr double thermal = Tb303Filter::thermal;
        static constexpr double supply = Tb303Filter::supply, reference = Tb303Filter::reference, vq9 = 3.31;
        static constexpr double npnSaturation = Tb303Filter::npnSaturation, npnBetaF = Tb303Filter::npnBetaF,
                                npnBetaR = Tb303Filter::npnBetaR, npnEarly = Tb303Filter::npnEarly;
        static constexpr double pnpSaturation = Tb303Filter::pnpSaturation, pnpBetaF = Tb303Filter::pnpBetaF,
                                pnpBetaR = Tb303Filter::pnpBetaR, pnpEarly = Tb303Filter::pnpEarly;
        static constexpr double diodeSaturation = Tb303Filter::diodeSaturation, diodeThermal = thermal * Tb303Filter::diodeEmission;
        static constexpr double npnRecombination = 6.734e-15, npnRecombinationEmission = 1.259; // QEM's ISE, NE
        static constexpr double jfetBeta = 0.89e-3, jfetPinch = -1.5, jfetLambda = 2.0e-3; // Q39, the 2SK30(Y)
        static constexpr double chipSaturation = 1.0e-15, chipBetaF = 150.0;               // the BA662's NPNs
        static constexpr int rateDivider = 4;

        Tb303Envelope()
        {
            circuit.setNewtonLimit(40);
            circuit.setJacobianReuse(true);
            build();
        }

        /** The chord method (D281), on by default; off solves each step exactly, for the tests. */
        void setJacobianReuse(bool on) noexcept { circuit.setJacobianReuse(on); }

        void prepare(double rate) noexcept
        {
            sampleRate = rate;
            circuit.setSampleRate(rate / rateDivider);
        }

        /** DECAY's and ACCENT's turns (0..1). DECAY changes re-analyse the circuit. */
        void setKnobs(double decay, double accent) noexcept
        {
            constexpr double k = 3.47; // the audio taper, as vcf.cir and env.cir have it
            const double d = (std::exp(k * std::clamp(decay, 0.0, 1.0)) - 1.0) / (std::exp(k) - 1.0);
            circuit.setResistance(rDecay, std::max(1.0e6 * d, 1.0));
            const double a = std::clamp(accent, 0.0, 1.0);
            circuit.setResistance(rAccentTop, std::max(50.0e3 * (1.0 - a), 1.0));
            circuit.setResistance(rAccentBottom, std::max(50.0e3 * a, 1.0));
        }

        /** The accent's switches (IC12A and IC12B): on for an accented step. */
        void setAccent(bool on) noexcept
        {
            if (on == accented)
                return;
            accented = on;
            circuit.setResistance(sDecayShort, on ? switchOn : switchOff);
            circuit.setResistance(sAccentPass, on ? switchOn : switchOff);
        }

        /** At rest with the gate low. */
        void settle() noexcept
        {
            gateVolts = 0.0;
            setInputs();
            circuit.settle();
            for (auto& o : now)
                o = 0.0;
            read(now);
            for (int k = 0; k < outputs; ++k)
                before[k] = value[k] = now[k];
            phase = rateDivider;
        }

        /** One sample of the voice: the CPU's gate (true: 5 V). Read the outputs after it. */
        void process(bool gate) noexcept
        {
            gateVolts = gate ? 5.0 : 0.0;
            if (phase >= rateDivider)
            {
                for (int k = 0; k < outputs; ++k)
                    before[k] = now[k];
                setInputs();
                circuit.process();
                read(now);
                phase = 0;
            }
            ++phase;
            // The outputs a quarter step late, between the last two solves.
            const double w = static_cast<double>(phase) / rateDivider;
            for (int k = 0; k < outputs; ++k)
                value[k] = before[k] + w * (now[k] - before[k]);
        }

        [[nodiscard]] double envelope() const noexcept { return value[0]; }
        [[nodiscard]] double accentWiper() const noexcept { return value[1]; }
        [[nodiscard]] double gateLine() const noexcept { return value[2]; }
        [[nodiscard]] double pinOne() const noexcept { return value[3]; }
        [[nodiscard]] double c62() const noexcept { return value[4]; }
        [[nodiscard]] double c42() const noexcept { return value[5]; }
        [[nodiscard]] double oneShot() const noexcept { return value[6]; }
        [[nodiscard]] bool lastSolveConverged() const noexcept { return circuit.lastSolveConverged(); }
        [[nodiscard]] int newtonSteps() const noexcept { return circuit.lastNewtonSteps(); }
        [[nodiscard]] bool fitsItsLimits() const noexcept { return circuit.fitsItsLimits(); }
        /** False if the linear part ties some junctions' currents (a group of nodes only junctions leave). */
        [[nodiscard]] bool isWellPosed() noexcept { return circuit.isWellPosed(); }

    private:
        static constexpr double switchOn = 200.0, switchOff = 1.0e9;
        static constexpr int outputs = 7;

        void setInputs() noexcept
        {
            circuit.setInput(inSupply, supply);
            circuit.setInput(inReference, reference);
            circuit.setInput(inGate, gateVolts);
            circuit.setInput(inVq9, vq9);
        }

        void read(double* out) const noexcept
        {
            out[0] = circuit.probe(pEnv);
            out[1] = circuit.probe(pAccent);
            out[2] = circuit.probe(pGate);
            out[3] = circuit.probe(pPin1);
            out[4] = circuit.probe(pC62);
            out[5] = circuit.probe(pC42);
            out[6] = circuit.probe(pOneShot);
        }

        // 1 the 12 V rail, 2 5.333 V, 3 the CPU's gate, 4 the filter's VQ9 (inputs); 5 Q37's base, 6 its collector,
        // 7 Q36's emitter, 8 its base, 9 GATE, 10 Q38's emitter, 11 its base, 12 C54's far side, 13 Q38's collector,
        // 14 after D37, 15 C62, 16 the DECAY line, 17 to 19 the diode string, 20 Q41's emitter, 21 the envelope, 22
        // IC12A's far side, 23 the ACCENT wiper, 24 C36, 25 D27's anode, 26 pin 1, 27 Q1A's emitter, 28 after D35,
        // 29 between R133 and R131, 30 Q31's emitter, 31 its base (C42), 32 Q32's collector, 33 after D36, 34 Q32's
        // base (C41), 35 its emitter.
        void build()
        {
            inSupply = circuit.addInput(1);
            inReference = circuit.addInput(2);
            inGate = circuit.addInput(3);
            inVq9 = circuit.addInput(4);
            auto npn = [this](int c, int b, int e) { circuit.addTransistor(c, b, e, npnSaturation, npnBetaF, npnBetaR, thermal, npnEarly); };
            auto pnp = [this](int c, int b, int e) { circuit.addPnpTransistor(c, b, e, pnpSaturation, pnpBetaF, pnpBetaR, thermal, pnpEarly); };
            auto diode = [this](int a, int k) { circuit.addDiode(a, k, diodeSaturation, diodeThermal); };
            // Gummel-Poon's low-current base recombination (ISE, NE) as a diode from base to emitter: at a few nA it is
            // most of an NPN's base current, and it is what lets C41 go after a trigger (Q32) and loads C62 (Q41, Q40).
            auto recombination = [this](int b, int e) { circuit.addDiode(b, e, npnRecombination, thermal * npnRecombinationEmission); };
            // The gate.
            circuit.addResistor(3, 5, 22.0e3);                                                   // R146
            circuit.addResistor(5, 0, 10.0e3);                                                   // R144
            npn(6, 5, 0);                                                                        // Q37
            circuit.addResistor(1, 7, 22.0);                                                     // R147
            circuit.addResistor(1, 8, 10.0e3);                                                   // R142
            circuit.addCapacitor(1, 8, 10.0e-6);                                                 // C72
            circuit.addResistor(8, 6, 10.0e3);                                                   // R143
            pnp(9, 8, 7);                                                                        // Q36
            circuit.addResistor(9, 0, 10.0e3);                                                   // R145
            // The one-shot into C62.
            circuit.addResistor(1, 10, 22.0);                                                    // R150
            circuit.addCapacitor(10, 0, 47.0e-6);                                                // C55
            circuit.addResistor(1, 11, 10.0e3);                                                  // R149
            circuit.addResistor(11, 12, 10.0e3);                                                 // R148
            circuit.addCapacitor(12, 6, 0.047e-6);                                               // C54
            pnp(13, 11, 10);                                                                     // Q38
            circuit.addResistor(13, 0, 22.0e3);                                                  // R151
            diode(13, 14);                                                                       // D37
            circuit.addResistor(14, 15, 100.0);                                                  // R152
            circuit.addCapacitor(15, 0, 1.0e-6);                                                 // C62
            // The buffer.
            npn(1, 15, 20);                                                                      // Q41
            recombination(15, 20);
            npn(1, 20, 21);                                                                      // Q40
            recombination(20, 21);
            circuit.addJfetChannel(21, 0, jfetBeta, jfetPinch, jfetLambda);                      // Q39
            circuit.addResistor(21, 4, 60.0e3);                                                  // ENV MOD and R61
            // The decay.
            circuit.addResistor(15, 16, 68.0e3);                                                 // R138
            rDecay = circuit.addResistor(16, 17, 1.0e6);                                         // DECAY
            sDecayShort = circuit.addResistor(16, 17, switchOff);                                // IC12B
            diode(17, 18);                                                                       // D28
            diode(18, 19);                                                                       // D29
            circuit.addResistor(2, 19, 22.0e3);                                                  // R117
            diode(19, 0);                                                                        // D26
            // The accent.
            sAccentPass = circuit.addResistor(21, 22, switchOff);                                // IC12A
            rAccentTop = circuit.addResistor(22, 23, 25.0e3);                                    // ACCENT above its wiper
            rAccentBottom = circuit.addResistor(23, 0, 25.0e3);                                  // and below
            circuit.addResistor(23, 24, 47.0e3);                                                 // R119
            circuit.addCapacitor(24, 0, 0.033e-6);                                               // C36
            circuit.addResistor(24, 25, 22.0e3);                                                 // R120
            diode(25, 26);                                                                       // D27
            // Pin 1: Q1A (with Q1B's base current) over its 500 Ohm.
            circuit.addDiode(26, 27, chipSaturation * (1.0 + 2.0 / chipBetaF), thermal);
            circuit.addResistor(27, 0, 500.0);
            // The VCA's control current.
            diode(9, 28);                                                                        // D35
            circuit.addResistor(28, 29, 2.2e3);                                                  // R133
            circuit.addResistor(29, 30, 220.0e3);                                                // R131
            pnp(26, 31, 30);                                                                     // Q31
            circuit.addResistor(1, 31, 1.5e6);                                                   // R123
            circuit.addCapacitor(31, 0, 1.0e-6);                                                 // C42
            circuit.addResistor(31, 32, 100.0);                                                  // R132
            diode(13, 33);                                                                       // D36
            circuit.addResistor(33, 34, 22.0e3);                                                 // R134
            circuit.addCapacitor(34, 0, 0.1e-6);                                                 // C41
            npn(32, 34, 35);                                                                     // Q32
            recombination(34, 35);
            circuit.addResistor(35, 0, 100.0);                                                   // R130
            // Leaks (as SPICE's GMIN) on the nodes and groups of nodes only junctions leave: Q41's emitter, the diode
            // string's middle, pin 1 (Q31's collector, D27, Q1A) and D35's chain to Q31's emitter (through R133 and
            // R131); without them the linear part ties those junctions' currents and the solve is meaningless.
            circuit.addResistor(20, 0, leakOhms);
            circuit.addResistor(18, 0, leakOhms);
            circuit.addResistor(26, 0, leakOhms);
            circuit.addResistor(30, 0, leakOhms);
            pEnv = circuit.addProbe(21);
            pAccent = circuit.addProbe(23);
            pGate = circuit.addProbe(9);
            pPin1 = circuit.addProbe(26);
            pC62 = circuit.addProbe(15);
            pC42 = circuit.addProbe(31);
            pOneShot = circuit.addProbe(13);
        }

        static constexpr double leakOhms = 1.0e9;
        // 65 ports: 39 parts (4 leaks) and 26 nonlinear (8 transistors, 12 one-ports: diodes, D27, Q1A, Q39, three
        // recombination diodes)
        wdf::Circuit<35, 65, 4, 0, 26, 7> circuit;

        double sampleRate = 192000.0, gateVolts = 0.0;
        bool accented = false;
        int phase = rateDivider;
        double now[outputs] {}, before[outputs] {}, value[outputs] {};
        int inSupply = 0, inReference = 0, inGate = 0, inVq9 = 0;
        int rDecay = 0, sDecayShort = 0, sAccentPass = 0, rAccentTop = 0, rAccentBottom = 0;
        int pEnv = 0, pAccent = 0, pGate = 0, pPin1 = 0, pC62 = 0, pC42 = 0, pOneShot = 0;
    };
}
