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
    typical VBE), here as Ebers-Moll with their forward Early voltages (VAF 74 V, 18.7 V) and the 2N3904's base
    recombination (Gummel-Poon's ISE 6.734 fA, NE 1.259: at the uA they carry here it doubles a base current; the
    2N3906 has none); the duals (2SC1583, 2SC2291) as matched NPNs of hFE 500, VAF 100 V, Q21 with its collector
    junctions' capacitance (CJC 2.5 pF at its bias, some 1.54 pF, which the pair's gain of some 225 multiplies). The
    rungs' diode-wired 2SC945s as diodes of IS (1 + 1 / hFE) with the same recombination. Q20, the tail, solved at
    rest as the same 2N3904 from the bias divider the loop leaves (Q21's base currents on it) and its own base
    current; the loop's R112 and R109 as their Thevenin equivalent from nbias, Q20's base current drawn through them.
    Without the recombination and the capacitance the filter sat 1.7 to 4.8 % off vcf.cir on the test's cases, 10 %
    at cutoff 0.9 with full resonance (CoolSynth's D360). The pots as their shares (the knobs' tracks are the voice's,
    CoolSynth's D324 and D364). ngspice's 27 C
    thermal voltage. The 12 V rail at 11.85 V (IC16, D275). Volts in, volts out.
*/
namespace coolsynth::synth::v3
{
    class Tb303Filter final
    {
        friend struct Tb303Lid; // the lid tap reads its circuits and state (Tb303Lid.h)
    public:
        static constexpr double thermal = 0.025865;  // ngspice's at 27 C
        static constexpr double npnSaturation = 39.0e-15, npnBetaF = 416.4, npnBetaR = 0.7371, npnEarly = 74.03;
        static constexpr double npnRecombination = 6.734e-15, npnRecombinationEmission = 1.259; // QEM's ISE and NE
        /** Q21's collector junctions at rest (ngspice's cmu at vcf.cir's operating point: CJC 2.5 pF, VJC 0.75 V and
            MJC 0.33 as ngspice's defaults, Vbc -2.496 V left, -2.456 V right), as fixed capacitors from base to collector. */
        static constexpr double q21LeftCjc = 1.5415e-12, q21RightCjc = 1.5478e-12;
        static constexpr double pnpSaturation = 39.0e-15, pnpBetaF = 300.0, pnpBetaR = 4.977, pnpEarly = 18.7;
        static constexpr double dualSaturation = 10.0e-15, dualBetaF = 500.0, dualBetaR = 4.0, dualEarly = 100.0;
        static constexpr double diodeSaturation = 2.52e-9, diodeEmission = 1.752; // D24, the 1S2473 as a 1N4148
        static constexpr double supply = 11.851, reference = 5.333;
        static constexpr double sourceOhms = 1.0e9; // the current sources' Norton resistance
        static constexpr double leakOhms = 1.0e9;   // across each rung, as SPICE's GMIN

        /** loadedByVca: the RESONANCE wiper also feeds the VCA (C22, R122 into its pin 3, at 5.333 V as AC ground), as in
            the whole voice (D295); off, the wiper as vcf.cir has it alone. devilFish: a current into Q10's base, the
            control's summing node, for the Devil Fish's Filter Tracking and Filter FM (CoolSynth's D309); off, the
            circuit is the stock one, part for part. */
        /** Where every TB-303 circuit's solve ends (the user's choice, CoolSynth's D345): 10 nV, not the solver's 100
            nV. Against the voice solved to 10 pV: -128.5 dB rms, the worst sample -96 dB; at 100 nV the oscillator's
            pitch ran some 0.0005 cents off and drifted from it (-90.5 dB rms, the worst sample -55.6 dB). */
        static constexpr double newtonTolerance = 1.0e-8;

        explicit Tb303Filter(bool loadedByVca = false, bool devilFish = false)
        {
            converter.setNewtonLimit(40);
            loop.setNewtonLimit(40);
            setNewtonTolerance(newtonTolerance);
            setJacobianReuse(true);
            buildConverter();
            buildLoop();
            if (loadedByVca)
            {
                loop.addCapacitor(27, 34, 0.01e-6); // the VCA's C22
                loop.addResistor(34, 2, 100.0e3);   // and R122, into pin 3
            }
            // Q21's collector-base capacitance, last so that every port before keeps its number (Tb303Lid.h).
            loop.addCapacitor(20, 22, q21LeftCjc);
            loop.addCapacitor(21, 23, q21RightCjc);
            if (devilFish)
                sFish = converter.addSource(8, 0, sourceOhms);
            fish = devilFish;
        }

        /** The Devil Fish's changes to the stock parts (Whittle's manual, the x0xb0x mod guide; CoolSynth's D309), all
            back to the stock values with the defaults:
                inputGain   R62 as 220k over it (0 opens it: no oscillator into the filter; 66.6 at most)
                wideRanges  Env Mod to zero and three times (R61 shorted, R63 with 100k across), Cut Off lower and
                            higher (R47 with 4.7k across; R73 at 33k and TM3 at half its resistance: an octave up at
                            the top, some 2.4 down at the bottom, as measured)
                highResonance  R97 at 5.1k: the loop's gain doubled, the filter oscillating at mid and high cutoffs
                c13, r46    the accent sweep's capacitor and its charging resistor (the Sweep Speed switch)
                bass        C21, the VCA's input coupling here, at 0.1 uF (the loop carries the VCA's input as its load) */
        void setChanges(double inputGain, bool wideRanges, bool highResonance, double c13, double r46, bool bass) noexcept
        {
            loop.setResistance(rR62, inputGain > 1.0e-6 ? std::max(220.0e3 / inputGain, 1.0) : 1.0e9);
            converter.setResistance(rR61, wideRanges ? 1.0 : 10.0e3);
            converter.setResistance(rR63, wideRanges ? 220.0e3 * 100.0e3 / 320.0e3 : 220.0e3);
            converter.setResistance(rR47, wideRanges ? 10.0e3 * 4.7e3 / 14.7e3 : 10.0e3);
            converter.setResistance(rR73, wideRanges ? 33.0e3 : 100.0e3);
            tm3Share = wideRanges ? 0.5 : 1.0; // TM3 turned on: measured, the top an octave up (0.93), half turn +0.19
            converter.setResistance(rTm3, std::max(470.0e3 * tm3Turn * tm3Share, 1.0));
            loop.setResistance(rR97, highResonance ? 5.1e3 : 10.0e3);
            converter.setCapacitance(cC13, c13);
            converter.setResistance(rR46, r46);
            r46Ohms = r46;
            loop.setCapacitance(cC21, bass ? 0.1e-6 : 0.01e-6);
        }

        /** The Devil Fish's current into the summing node (amps; Filter Tracking and Filter FM), taken at the next
            sample. Nothing on a stock filter. */
        void setFishCurrent(double amps) noexcept { fishAmps = amps; }

        /** The RESONANCE wiper's voltage (the VCA's second input). */
        [[nodiscard]] double resonanceWiper() const noexcept { return loop.probe(pResWiper); }
        /** The current D24 draws from the ACCENT knob's wiper (into R46): the envelope's load. */
        [[nodiscard]] double accentCurrent() const noexcept
        {
            return (converter.probe(pD24) - converter.probe(pR46)) / r46Ohms;
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

        /** The pots, each its share from pin 1 0..1 (the voice maps the knobs' turns: Tb303Voice's audioTrack and
            resonanceTrack), and TM3 (0..1 of 470 kOhm). Changing one re-analyses its circuit. */
        void setKnobs(double cutoff, double resonance, double envMod, double tm3) noexcept
        {
            const double tc = std::clamp(cutoff, 0.0, 1.0), te = std::clamp(envMod, 0.0, 1.0),
                         r = std::clamp(resonance, 0.0, 1.0);
            converter.setResistance(rCut3, std::max(50.0e3 * (1.0 - tc), 1.0));
            converter.setResistance(rCut1, std::max(50.0e3 * tc, 1.0));
            converter.setResistance(rEnv3, std::max(50.0e3 * (1.0 - te), 1.0));
            converter.setResistance(rEnv1, std::max(50.0e3 * te, 1.0));
            converter.setResistance(rResB1, std::max(50.0e3 * r, 1.0));
            converter.setResistance(rResB3, std::max(50.0e3 * (1.0 - r), 1.0));
            tm3Turn = std::clamp(tm3, 0.0, 1.0);
            converter.setResistance(rTm3, std::max(470.0e3 * tm3Turn * tm3Share, 1.0));
            loop.setResistance(rResA3, std::max(50.0e3 * (1.0 - r), 1.0));
            loop.setResistance(rResA1, std::max(50.0e3 * r, 1.0));
        }

        /** The circuit at rest with these inputs (the VCO's mean, the envelope, the accent's wiper): the
            reference's operating point. */
        void settle(double input, double envelope, double accent) noexcept
        {
            setInputs(input, envelope, accent);
            // Q20 from the bias as the loop leaves it (nbias under Q21's base currents and Q20's own, its collector at
            // Q21's emitters), each pass from the one before: the tail moves the loop's nodes by mV, they move it by ppm.
            double ne12 = 4.8, nbias = reference * (20.0e3 / 30.0e3), ne21 = lastNe21;
            for (int pass = 0; pass < 6; ++pass)
            {
                solveTail(nbias, ne21);
                loop.setSource(sBias, -10.0e3 * tailBaseAmps);
                converter.setInput(convNe12, ne12);
                converter.settle();
                collectorAmps = (ne12 - converter.probe(pNc11)) / 2.2e3;
                loop.setSource(sTail, loop.probe(pNe21) - tailAmps * sourceOhms);
                loop.setSource(sCollector, ne12 - collectorAmps * sourceOhms);
                loop.settle();
                ne12 = loop.probe(pNe12);
                double volts[35];
                loop.nodeVoltages(volts);
                nbias = volts[19];
                ne21 = volts[24];
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

        /** Where a sample's solve ends (newtonTolerance from the constructor; the 303's accuracy, CoolSynth's D349). */
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

        /** An audio-taper pot's share from pin 1 at a turn: 15 % at half turn (out.cir's VOLUME; CUTOFF and ENV MOD
            had it before D364). */
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
            if (fish)
                converter.setSource(sFish, converter.probe(pNb10) - fishAmps * sourceOhms);
            loop.setInput(loopSupply, supply);
            loop.setInput(loopReference, reference);
            loop.setInput(loopIn, input);
        }

        /**
            Q20 at rest (tailAmps its collector current, tailBaseAmps its base's), as the loop's other 2N3904s: Ebers-Moll
            with the Early effect and the base recombination. Its base from nbias through R112 (10k), R109 (10k) to
            ground, less its base current; R108 (2.2k) under its emitter; its collector at Q21's emitters (ne21).
        */
        void solveTail(double nbias, double ne21) noexcept
        {
            double vbe = 0.66, vb = 0.5 * nbias;
            double ic = 0.0, ib = 0.0;
            for (int outer = 0; outer < 40; ++outer)
            {
                // The emitter's KCL in vbe at this base voltage: (vb - vbe) / R108 = ic + ib, falling in vbe.
                for (int it = 0; it < 60; ++it)
                {
                    const double ef = std::exp(vbe / thermal), er = std::exp((vb - ne21) / thermal);
                    const double el = std::exp(vbe / (thermal * npnRecombinationEmission));
                    const double iF = npnSaturation * (ef - 1.0), iR = npnSaturation * (er - 1.0);
                    const double k = 1.0 - (vb - ne21) / npnEarly;
                    ic = (iF - iR) * k - iR / npnBetaR;
                    ib = iF / npnBetaF + iR / npnBetaR + npnRecombination * (el - 1.0);
                    const double f = (vb - vbe) / 2.2e3 - ic - ib;
                    const double gF = npnSaturation * ef / thermal;
                    const double df = -1.0 / 2.2e3 - gF * k - gF / npnBetaF
                                      - npnRecombination * el / (thermal * npnRecombinationEmission);
                    const double step = std::clamp(f / df, -0.05, 0.05);
                    vbe -= step;
                    if (std::abs(step) < 1.0e-15)
                        break;
                }
                const double next = 0.5 * (nbias - 10.0e3 * ib); // (nbias - vb) / R112 = vb / R109 + ib
                const bool done = std::abs(next - vb) < 1.0e-14;
                vb = next;
                if (done)
                    break;
            }
            tailAmps = std::max(1.0e-9, ic);
            tailBaseAmps = ib;
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
            converter.addTransistor(10, 9, 7, npnSaturation, npnBetaF, npnBetaR, thermal, npnEarly, npnRecombination,
                                    thermal * npnRecombinationEmission);                                          // Q11
            converter.addResistor(10, 5, 2.2e3);                                                   // R69
            rCut3 = converter.addResistor(1, 11, 25.0e3);                                          // CUTOFF above its wiper
            rCut1 = converter.addResistor(11, 12, 25.0e3);                                         // and below
            rR47 = converter.addResistor(12, 7, 10.0e3);                                           // R47
            rR73 = converter.addResistor(11, 13, 100.0e3);                                         // R73
            rTm3 = converter.addResistor(13, 9, 80.0e3);                                           // TM3
            converter.addResistor(8, 7, 2.2e3);                                                    // R71
            rEnv3 = converter.addResistor(3, 14, 25.0e3);                                          // ENV MOD above its wiper
            rEnv1 = converter.addResistor(14, 15, 25.0e3);                                         // and below
            rR61 = converter.addResistor(15, 7, 10.0e3);                                           // R61
            rR63 = converter.addResistor(14, 8, 220.0e3);                                          // R63
            converter.addDiode(4, 16, diodeSaturation, thermal * diodeEmission);                   // D24
            rR46 = converter.addResistor(16, 17, 47.0e3);                                          // R46
            rResB1 = converter.addResistor(17, 18, 25.0e3);                                        // RESONANCE's second gang
            rResB3 = converter.addResistor(18, 19, 25.0e3);
            cC13 = converter.addCapacitor(19, 0, 1.0e-6);                                          // C13
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
            rR62 = loop.addResistor(3, 5, 220.0e3);                                                // R62
            loop.addCapacitor(5, 6, 1.0e-6);                                                       // C17
            loop.addResistor(2, 7, 2.2e3);                                                         // R68
            loop.addCapacitor(7, 0, 10.0e-6);                                                      // C16
            loop.addResistor(6, 7, 2.2e3);                                                         // R70
            loop.addResistor(8, 7, 2.2e3);                                                         // R98
            loop.addTransistor(9, 6, 18, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);          // Q12, left
            loop.addTransistor(13, 8, 18, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);         // Q12, right
            const double rung = npnSaturation * (1.0 + 1.0 / npnBetaF), recombinationThermal = thermal * npnRecombinationEmission;
            loop.addDiode(10, 9, rung, thermal, 1000.0, npnRecombination, recombinationThermal);   // Q13
            loop.addDiode(11, 10, rung, thermal, 1000.0, npnRecombination, recombinationThermal);  // Q14
            loop.addDiode(12, 11, rung, thermal, 1000.0, npnRecombination, recombinationThermal);  // Q16
            loop.addDiode(14, 13, rung, thermal, 1000.0, npnRecombination, recombinationThermal);  // Q15
            loop.addDiode(15, 14, rung, thermal, 1000.0, npnRecombination, recombinationThermal);  // Q17
            loop.addDiode(16, 15, rung, thermal, 1000.0, npnRecombination, recombinationThermal);  // Q23
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
            sBias = loop.addSource(19, 0, 20.0e3); // R112 and R109, Q20's bias: their Thevenin, -10k times its base current
            loop.addCapacitor(16, 20, 0.1e-6);                                                     // C27
            loop.addCapacitor(12, 21, 0.1e-6);                                                     // C25
            loop.addResistor(20, 19, 100.0e3);                                                     // R113
            loop.addResistor(21, 19, 100.0e3);                                                     // R114
            loop.addTransistor(22, 20, 24, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);        // Q21, left
            loop.addTransistor(23, 21, 24, dualSaturation, dualBetaF, dualBetaR, thermal, dualEarly);        // Q21, right
            loop.addResistor(4, 22, 22.0e3);                                                       // R111 (22k, D275)
            loop.addResistor(4, 23, 22.0e3);                                                       // R110 (22k, D275)
            sTail = loop.addSource(24, 0, sourceOhms);                                             // Q20, a fixed current
            loop.addTransistor(4, 23, 25, npnSaturation, npnBetaF, npnBetaR, thermal, npnEarly, npnRecombination,
                               recombinationThermal);                                                       // Q19
            loop.addResistor(25, 0, 10.0e3);                                                       // R96
            loop.addCapacitor(25, 26, 1.0e-6);                                                     // C14
            rResA3 = loop.addResistor(26, 27, 25.0e3);                                             // RESONANCE above its wiper
            rResA1 = loop.addResistor(27, 0, 25.0e3);                                              // and below
            loop.addResistor(26, 28, 220.0e3);                                                     // R121, the VCA's input
            cC21 = loop.addCapacitor(28, 2, 0.01e-6);                                              // C21
            loop.addCapacitor(27, 29, 1.0e-6);                                                     // C15
            loop.addResistor(29, 30, 2.2e3);                                                       // R67
            loop.addResistor(2, 31, 100.0e3);                                                      // R66
            loop.addCapacitor(31, 0, 1.0e-6);                                                      // C22
            loop.addResistor(31, 30, 100.0e3);                                                     // R99
            loop.addTransistor(1, 30, 32, npnSaturation, npnBetaF, npnBetaR, thermal, npnEarly, npnRecombination,
                               recombinationThermal);                                                       // Q18
            loop.addResistor(32, 0, 10.0e3);                                                       // R94
            loop.addCapacitor(32, 33, 1.0e-6);                                                     // C23
            rR97 = loop.addResistor(33, 8, 10.0e3);                                                // R97
            sCollector = loop.addSource(18, 0, sourceOhms);                                        // Q11's collector
            pTp6 = loop.addProbe(25);
            pVcf = loop.addProbe(26);
            pL4 = loop.addProbe(12);
            pR4 = loop.addProbe(16);
            pNe12 = loop.addProbe(18);
            pNe21 = loop.addProbe(24);
            pResWiper = loop.addProbe(27);
        }

        wdf::Circuit<19, 27, 5, 0, 7, 7, true> converter; // 26 ports, one more in a Devil Fish
        wdf::Circuit<34, 71, 3, 0, 22, 7, true> loop; // 67 ports counted from buildLoop, 2 more loaded by the VCA, Q21's 2

        double tailAmps = 0.5e-3, tailBaseAmps = 0.0, collectorAmps = 0.0, lastNe12 = 4.8, lastNe21 = 2.9;
        int sBias = 0;
        bool fish = false;
        double fishAmps = 0.0, r46Ohms = 47.0e3, tm3Turn = 0.35, tm3Share = 1.0;
        int sFish = 0, rR47 = 0, rR73 = 0, rR61 = 0, rR63 = 0, rR46 = 0, cC13 = 0, rR62 = 0, cC21 = 0, rR97 = 0;
        int convSupply = 0, convReference = 0, convEnvelope = 0, convAccent = 0, convNe12 = 0;
        int rCut3 = 0, rCut1 = 0, rTm3 = 0, rEnv3 = 0, rEnv1 = 0, rResB1 = 0, rResB3 = 0;
        int pNb10 = 0, pNb11 = 0, pNc11 = 0, pN13 = 0, pVq9 = 0, pD24 = 0, pR46 = 0, pResWiper = 0;
        int loopSupply = 0, loopReference = 0, loopIn = 0, sTail = 0, sCollector = 0, rResA3 = 0, rResA1 = 0;
        int pTp6 = 0, pVcf = 0, pL4 = 0, pR4 = 0, pNe12 = 0, pNe21 = 0;
    };
}
