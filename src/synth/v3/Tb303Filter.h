#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include "CircuitWdf.h"

/*
    The Roland TB-303's filter as its circuit (D275, D279), from the service notes (p. 5, p. 8's Fig. 2) and
    tools/tb303/vcf.cir, its reference. Two circuits, each a wave digital filter (CircuitWdf.h):

        Converter   the current: Q9 holds the common node (VQ9) near 3.3 V; Q10, a follower, sets Q11's base above
                    its own; CUTOFF FREQ (VR3, 50k A) from 12 V to R47 feeds Q10's emitter through R73 and TM3; Q10's
                    base takes R71 to VQ9, ENV MOD's wiper through R63 and the RESONANCE pot's second gang through
                    R72, that gang lying between R46 (from D24, the ACCENT knob's wiper) and C13
        Loop        the resonance's whole loop in one solve (D279): the input through R62 and C17 into Q12's left
                    base; the ladder, four rungs of diode-wired transistors from Q22's emitters to Q12's collectors
                    (C18 at the bottom, C19, C24, C26 above); Q21 reading the top through C25 and C27 on a fixed tail;
                    Q19 to TP6, C14, the RESONANCE pot's first gang, C15, Q18 and C23, R97 into Q12's right base

    Q11's collector feeds the loop's Q12 a sample late, as a current (its collector is a current source's: nothing
    the loop does reaches back); Q20's tail likewise, a fixed current. Both enter the loop as a source behind 1 GOhm,
    its volts set each sample from the node's last voltage so the current is exact. Every junction conducts all the
    time, so every sample takes a Newton step: both circuits keep their Jacobian's factors and refactor only when
    those stop bringing the step down (the chord method, D281), about one sample in a hundred; a sample then costs
    some 2.2 us at 192 kHz (45 % of a core, optimised Debug), within some 3 ppm of the exact solve.

    Transistors: the 2SC945 and 2SA733 as vcf.cir's stand-ins (Fairchild's 2N3904 and 2N3906 with IS from the parts'
    typical VBE), here as Ebers-Moll with their forward Early voltages (VAF 74 V, 18.7 V); the duals (2SC1583, 2SC2291) as matched NPNs of hFE 500, VAF 100 V. The rungs' diode-wired
    2SC945s as diodes of IS (1 + 1 / hFE). The audio-taper pots as 15 % at half turn. ngspice's 27 C thermal voltage.
    The 12 V rail at 11.85 V (IC16, D275). Volts in, volts out.
*/
namespace coolsynth::synth::v3
{
    class Tb303Filter final
    {
    public:
        static constexpr double thermal = 0.025865;  // ngspice's at 27 C
        static constexpr double npnSaturation = 39.0e-15, npnBetaF = 416.4, npnBetaR = 0.7371, npnEarly = 74.03;
        static constexpr double pnpSaturation = 39.0e-15, pnpBetaF = 300.0, pnpBetaR = 4.977, pnpEarly = 18.7;
        static constexpr double dualSaturation = 10.0e-15, dualBetaF = 500.0, dualBetaR = 4.0, dualEarly = 100.0;
        static constexpr double diodeSaturation = 2.52e-9, diodeEmission = 1.752; // D24, the 1S2473 as a 1N4148
        static constexpr double supply = 11.851, reference = 5.333;
        static constexpr double sourceOhms = 1.0e9; // the current sources' Norton resistance
        static constexpr double leakOhms = 1.0e9;   // across each rung, as SPICE's GMIN

        /** loadedByVca: the RESONANCE wiper also feeds the VCA (C22, R122 into its pin 3, at 5.333 V as AC ground), as in
            the whole voice (D295); off, the wiper as vcf.cir has it alone. */
        explicit Tb303Filter(bool loadedByVca = false)
        {
            converter.setNewtonLimit(40);
            loop.setNewtonLimit(40);
            setJacobianReuse(true);
            buildConverter();
            buildLoop();
            if (loadedByVca)
            {
                loop.addCapacitor(27, 34, 0.01e-6); // the VCA's C22
                loop.addResistor(34, 2, 100.0e3);   // and R122, into pin 3
            }
        }

        /** The RESONANCE wiper's voltage (the VCA's second input). */
        [[nodiscard]] double resonanceWiper() const noexcept { return loop.probe(pResWiper); }
        /** The current D24 draws from the ACCENT knob's wiper (into R46): the envelope's load. */
        [[nodiscard]] double accentCurrent() const noexcept
        {
            return (converter.probe(pD24) - converter.probe(pR46)) / 47.0e3;
        }

        /** The chord method on both circuits (D281), on by default; off solves each sample exactly, for the tests. */
        void setJacobianReuse(bool on) noexcept
        {
            converter.setJacobianReuse(on);
            loop.setJacobianReuse(on);
        }
        /** False if a circuit was built past its template's limits (CircuitWdf's fitsItsLimits). */
        [[nodiscard]] bool fitsItsLimits() const noexcept { return converter.fitsItsLimits() && loop.fitsItsLimits(); }

        /** The share of the loop's steps that factored afresh. */
        [[nodiscard]] double freshShare() const noexcept
        {
            const long all = loop.chordStepCount() + loop.freshStepCount();
            return all > 0 ? static_cast<double>(loop.freshStepCount()) / static_cast<double>(all) : 0.0;
        }

        void prepare(double rate) noexcept
        {
            converter.setSampleRate(rate);
            loop.setSampleRate(rate);
        }

        /** The knobs, each its turn 0..1, and TM3 (0..1 of 470 kOhm). Changing one re-analyses its circuit. */
        void setKnobs(double cutoff, double resonance, double envMod, double tm3) noexcept
        {
            const double tc = audioTaper(cutoff), te = audioTaper(envMod), r = std::clamp(resonance, 0.0, 1.0);
            converter.setResistance(rCut3, std::max(50.0e3 * (1.0 - tc), 1.0));
            converter.setResistance(rCut1, std::max(50.0e3 * tc, 1.0));
            converter.setResistance(rEnv3, std::max(50.0e3 * (1.0 - te), 1.0));
            converter.setResistance(rEnv1, std::max(50.0e3 * te, 1.0));
            converter.setResistance(rResB1, std::max(50.0e3 * r, 1.0));
            converter.setResistance(rResB3, std::max(50.0e3 * (1.0 - r), 1.0));
            converter.setResistance(rTm3, std::max(470.0e3 * std::clamp(tm3, 0.0, 1.0), 1.0));
            loop.setResistance(rResA3, std::max(50.0e3 * (1.0 - r), 1.0));
            loop.setResistance(rResA1, std::max(50.0e3 * r, 1.0));
        }

        /** The circuit at rest with these inputs (the VCO's mean, the envelope, the accent's wiper): the
            reference's operating point. */
        void settle(double input, double envelope, double accent) noexcept
        {
            setInputs(input, envelope, accent);
            tailAmps = tailCurrent();
            double ne12 = 4.8;
            for (int pass = 0; pass < 6; ++pass)
            {
                converter.setInput(convNe12, ne12);
                converter.settle();
                collectorAmps = (ne12 - converter.probe(pNc11)) / 2.2e3;
                loop.setSource(sTail, loop.probe(pNe21) - tailAmps * sourceOhms);
                loop.setSource(sCollector, ne12 - collectorAmps * sourceOhms);
                loop.settle();
                ne12 = loop.probe(pNe12);
            }
            lastNe12 = ne12;
            lastNe21 = loop.probe(pNe21);
        }

        /** One sample: the VCO's output, the envelope (ENV MOD's top), the ACCENT knob's wiper; returns TP6. */
        double process(double input, double envelope, double accent) noexcept
        {
            setInputs(input, envelope, accent);
            converter.setInput(convNe12, lastNe12);
            converter.process();
            collectorAmps = (lastNe12 - converter.probe(pNc11)) / 2.2e3;
            loop.setSource(sTail, lastNe21 - tailAmps * sourceOhms);
            loop.setSource(sCollector, lastNe12 - collectorAmps * sourceOhms);
            loop.process();
            lastNe12 = loop.probe(pNe12);
            lastNe21 = loop.probe(pNe21);
            return loop.probe(pTp6);
        }

        /** Where a sample's solve ends (CircuitWdf's tolerance, 100 nV by default). */
        void setNewtonTolerance(double volts) noexcept
        {
            converter.setNewtonTolerance(volts);
            loop.setNewtonTolerance(volts);
        }

        [[nodiscard]] double tp6() const noexcept { return loop.probe(pTp6); }
        [[nodiscard]] double output() const noexcept { return loop.probe(pVcf); } // C14's side, toward the VCA
        [[nodiscard]] double ladderTopLeft() const noexcept { return loop.probe(pL4); }
        [[nodiscard]] double ladderTopRight() const noexcept { return loop.probe(pR4); }
        [[nodiscard]] double q10Base() const noexcept { return converter.probe(pNb10); }
        [[nodiscard]] double q11Base() const noexcept { return converter.probe(pNb11); }
        [[nodiscard]] double c13() const noexcept { return converter.probe(pN13); }
        [[nodiscard]] double ladderCurrent() const noexcept { return collectorAmps; }
        [[nodiscard]] int newtonSteps() const noexcept { return std::max(converter.lastNewtonSteps(), loop.lastNewtonSteps()); }
        [[nodiscard]] bool lastSolveConverged() const noexcept { return converter.lastSolveConverged() && loop.lastSolveConverged(); }

        /** An audio-taper pot's share from pin 1 at a turn: 15 % at half turn (vcf.cir's). */
        static double audioTaper(double turn) noexcept
        {
            constexpr double k = 3.47;
            return (std::exp(k * std::clamp(turn, 0.0, 1.0)) - 1.0) / (std::exp(k) - 1.0);
        }

    private:
        void setInputs(double input, double envelope, double accent) noexcept
        {
            converter.setInput(convSupply, supply);
            converter.setInput(convReference, reference);
            converter.setInput(convEnvelope, envelope);
            converter.setInput(convAccent, accent);
            loop.setInput(loopSupply, supply);
            loop.setInput(loopReference, reference);
            loop.setInput(loopIn, input);
        }

        /** Q20's collector current: its base from R116, R112, R109 (5.333 V into 10k, 10k, 10k), R108 2.2k. */
        static double tailCurrent() noexcept
        {
            const double thevenin = reference * (10.0e3 / 30.0e3), theveninOhms = 20.0e3 * 10.0e3 / 30.0e3;
            double ic = 0.5e-3;
            for (int i = 0; i < 50; ++i)
            {
                const double vbe = thermal * std::log(ic / npnSaturation);
                const double ie = ic * (1.0 + 1.0 / npnBetaF);
                const double vb = thevenin - (ie / (npnBetaF + 1.0)) * theveninOhms;
                ic = std::max(1.0e-9, (vb - vbe) / 2.2e3 / (1.0 + 1.0 / npnBetaF));
            }
            return ic;
        }

        // Converter: 1 the 12 V rail, 2 5.333 V, 3 the envelope, 4 the ACCENT wiper, 5 Q12's emitters (all inputs);
        // 6 Q9's base, 7 VQ9, 8 Q10's base, 9 Q11's base (Q10's emitter), 10 Q11's collector, 11 CUTOFF's wiper,
        // 12 its pin 1, 13 between R73 and TM3, 14 ENV MOD's wiper, 15 its pin 1, 16 D24's cathode, 17 the
        // RESONANCE second gang's pin 1, 18 its wiper, 19 C13.
        void buildConverter()
        {
            convSupply = converter.addInput(1);
            convReference = converter.addInput(2);
            convEnvelope = converter.addInput(3);
            convAccent = converter.addInput(4);
            convNe12 = converter.addInput(5);
            converter.addResistor(2, 6, 10.0e3);                                                   // R65
            converter.addResistor(6, 0, 10.0e3);                                                   // R64
            converter.addPnpTransistor(0, 6, 7, pnpSaturation, pnpBetaF, pnpBetaR, thermal, pnpEarly);      // Q9
            converter.addPnpTransistor(0, 8, 9, pnpSaturation, pnpBetaF, pnpBetaR, thermal, pnpEarly);      // Q10
            converter.addTransistor(10, 9, 7, npnSaturation, npnBetaF, npnBetaR, thermal, npnEarly);        // Q11
            converter.addResistor(10, 5, 2.2e3);                                                   // R69
            rCut3 = converter.addResistor(1, 11, 25.0e3);                                          // CUTOFF above its wiper
            rCut1 = converter.addResistor(11, 12, 25.0e3);                                         // and below
            converter.addResistor(12, 7, 10.0e3);                                                  // R47
            converter.addResistor(11, 13, 100.0e3);                                                // R73
            rTm3 = converter.addResistor(13, 9, 80.0e3);                                           // TM3
            converter.addResistor(8, 7, 2.2e3);                                                    // R71
            rEnv3 = converter.addResistor(3, 14, 25.0e3);                                          // ENV MOD above its wiper
            rEnv1 = converter.addResistor(14, 15, 25.0e3);                                         // and below
            converter.addResistor(15, 7, 10.0e3);                                                  // R61
            converter.addResistor(14, 8, 220.0e3);                                                 // R63
            converter.addDiode(4, 16, diodeSaturation, thermal * diodeEmission);                   // D24
            converter.addResistor(16, 17, 47.0e3);                                                 // R46
            rResB1 = converter.addResistor(17, 18, 25.0e3);                                        // RESONANCE's second gang
            rResB3 = converter.addResistor(18, 19, 25.0e3);
            converter.addCapacitor(19, 0, 1.0e-6);                                                 // C13
            converter.addResistor(18, 8, 100.0e3);                                                 // R72
            pNb10 = converter.addProbe(8);
            pNb11 = converter.addProbe(9);
            pNc11 = converter.addProbe(10);
            pN13 = converter.addProbe(19);
            pVq9 = converter.addProbe(7);
            pD24 = converter.addProbe(16);
            pR46 = converter.addProbe(17);
        }

        // Loop: 1 the 12 V rail, 2 5.333 V, 3 the VCO's output (inputs); 4 the filter's own rail (R95), 5 after R62,
        // 6 Q12's left base, 7 C16, 8 Q12's right base, 9 to 12 the ladder's left side bottom to top, 13 to 16 its
        // right side, 17 Q22's bases, 18 Q12's emitters, 19 the read-out's bias, 20 and 21 Q21's bases, 22 and 23
        // its collectors, 24 its emitters, 25 TP6 (Q19's emitter), 26 C14's side, 27 RESONANCE's wiper, 28 C21,
        // 29 C15's side, 30 Q18's base, 31 C22, 32 Q18's emitter, 33 C23's side.
        void buildLoop()
        {
            loopSupply = loop.addInput(1);
            loopReference = loop.addInput(2);
            loopIn = loop.addInput(3);
            loop.addResistor(1, 4, 100.0);                                                         // R95
            loop.addCapacitor(4, 0, 47.0e-6);                                                      // C28
            loop.addResistor(3, 5, 220.0e3);                                                       // R62
            loop.addCapacitor(5, 6, 1.0e-6);                                                       // C17
            loop.addResistor(2, 7, 2.2e3);                                                         // R68
            loop.addCapacitor(7, 0, 10.0e-6);                                                      // C16
            loop.addResistor(6, 7, 2.2e3);                                                         // R70
            loop.addResistor(8, 7, 2.2e3);                                                         // R98
            loop.addTransistor(9, 6, 18, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);          // Q12, left
            loop.addTransistor(13, 8, 18, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);         // Q12, right
            const double rung = npnSaturation * (1.0 + 1.0 / npnBetaF);
            loop.addDiode(10, 9, rung, thermal);                                                   // Q13
            loop.addDiode(11, 10, rung, thermal);                                                  // Q14
            loop.addDiode(12, 11, rung, thermal);                                                  // Q16
            loop.addDiode(14, 13, rung, thermal);                                                  // Q15
            loop.addDiode(15, 14, rung, thermal);                                                  // Q17
            loop.addDiode(16, 15, rung, thermal);                                                  // Q23
            // A leak across each rung (as SPICE's GMIN): with the capacitors open at rest, a rung's middle node would
            // hang between two junctions alone and the solve could not place it.
            for (const auto& [a, b] : { std::pair { 10, 9 }, { 11, 10 }, { 12, 11 }, { 14, 13 }, { 15, 14 }, { 16, 15 } })
                loop.addResistor(a, b, leakOhms);
            loop.addCapacitor(9, 13, 0.018e-6);                                                    // C18
            loop.addCapacitor(10, 14, 0.033e-6);                                                   // C19
            loop.addCapacitor(11, 15, 0.033e-6);                                                   // C24
            loop.addCapacitor(12, 16, 0.033e-6);                                                   // C26
            loop.addTransistor(4, 17, 12, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);         // Q22, left
            loop.addTransistor(4, 17, 16, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);         // Q22, right
            loop.addResistor(4, 17, 10.0e3);                                                       // R115
            loop.addCapacitor(17, 0, 10.0e-6);                                                     // C30
            loop.addResistor(2, 19, 10.0e3);                                                       // R116
            loop.addCapacitor(19, 0, 1.0e-6);                                                      // C29
            loop.addResistor(19, 0, 20.0e3);                                                       // R112 and R109, Q20's bias
            loop.addCapacitor(16, 20, 0.1e-6);                                                     // C27
            loop.addCapacitor(12, 21, 0.1e-6);                                                     // C25
            loop.addResistor(20, 19, 100.0e3);                                                     // R113
            loop.addResistor(21, 19, 100.0e3);                                                     // R114
            loop.addTransistor(22, 20, 24, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);        // Q21, left
            loop.addTransistor(23, 21, 24, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);        // Q21, right
            loop.addResistor(4, 22, 22.0e3);                                                       // R111 (22k, D275)
            loop.addResistor(4, 23, 22.0e3);                                                       // R110 (22k, D275)
            sTail = loop.addSource(24, 0, sourceOhms);                                             // Q20, a fixed current
            loop.addTransistor(4, 23, 25, npnSaturation, npnBetaF, npnBetaR, thermal, npnEarly);            // Q19
            loop.addResistor(25, 0, 10.0e3);                                                       // R96
            loop.addCapacitor(25, 26, 1.0e-6);                                                     // C14
            rResA3 = loop.addResistor(26, 27, 25.0e3);                                             // RESONANCE above its wiper
            rResA1 = loop.addResistor(27, 0, 25.0e3);                                              // and below
            loop.addResistor(26, 28, 220.0e3);                                                     // R121, the VCA's input
            loop.addCapacitor(28, 2, 0.01e-6);                                                     // C21
            loop.addCapacitor(27, 29, 1.0e-6);                                                     // C15
            loop.addResistor(29, 30, 2.2e3);                                                       // R67
            loop.addResistor(2, 31, 100.0e3);                                                      // R66
            loop.addCapacitor(31, 0, 1.0e-6);                                                      // C22
            loop.addResistor(31, 30, 100.0e3);                                                     // R99
            loop.addTransistor(1, 30, 32, npnSaturation, npnBetaF, npnBetaR, thermal, npnEarly);            // Q18
            loop.addResistor(32, 0, 10.0e3);                                                       // R94
            loop.addCapacitor(32, 33, 1.0e-6);                                                     // C23
            loop.addResistor(33, 8, 10.0e3);                                                       // R97
            sCollector = loop.addSource(18, 0, sourceOhms);                                        // Q11's collector
            pTp6 = loop.addProbe(25);
            pVcf = loop.addProbe(26);
            pL4 = loop.addProbe(12);
            pR4 = loop.addProbe(16);
            pNe12 = loop.addProbe(18);
            pNe21 = loop.addProbe(24);
            pResWiper = loop.addProbe(27);
        }

        wdf::Circuit<19, 26, 5, 0, 7, 7> converter;
        wdf::Circuit<34, 69, 3, 0, 22, 7> loop; // 67 ports counted from buildLoop, 2 more loaded by the VCA

        double tailAmps = 0.5e-3, collectorAmps = 0.0, lastNe12 = 4.8, lastNe21 = 2.9;
        int convSupply = 0, convReference = 0, convEnvelope = 0, convAccent = 0, convNe12 = 0;
        int rCut3 = 0, rCut1 = 0, rTm3 = 0, rEnv3 = 0, rEnv1 = 0, rResB1 = 0, rResB3 = 0;
        int pNb10 = 0, pNb11 = 0, pNc11 = 0, pN13 = 0, pVq9 = 0, pD24 = 0, pR46 = 0, pResWiper = 0;
        int loopSupply = 0, loopReference = 0, loopIn = 0, sTail = 0, sCollector = 0, rResA3 = 0, rResA1 = 0;
        int pTp6 = 0, pVcf = 0, pL4 = 0, pR4 = 0, pNe12 = 0, pNe21 = 0;
    };
}
