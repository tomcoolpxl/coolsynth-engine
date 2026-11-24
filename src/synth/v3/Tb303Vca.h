#pragma once

#include <algorithm>
#include <cmath>

#include "CircuitWdf.h"
#include "Tb303Envelope.h"
#include "Tb303Filter.h"

/*
    The Roland TB-303's VCA (D275, D293): IC15, the BA662A, as its transistors as Open Music Labs reverse-engineered it
    (tools/tb303/vca.cir, its reference), with its inputs' and output's networks. Two circuits:

        Chip        Q1B mirrors pin 1's current (its base at pin 1's voltage, from Tb303Envelope; its area Q1A's,
                    Tb303Envelope::chipInputArea, fitted to Din Sync's unit) into the pair Q2A
                    (pin 2) and Q2B (pin 3); two PNP Wilson mirrors (Q3A, Q3B, Q5B; Q4A, Q4B, Q5A) and an NPN one
                    (Q6A, Q6B, Q7B) make pin 6 the difference of the pair's currents. Pins 2 and 3 at C37's 5.333 V
                    (R126, R125, R124); the audio into pin 3 through C21 and R121 (the filter's output) and C22 and
                    R122 (the RESONANCE pot's wiper); pin 6's load R127 and R128 to R129 and C40's 5.333 V (R162)
        Buffer      pins 7 to 8, a complementary follower (Q7A, Q10A, R21), Q9A sinking Q9B's current from R20; out
                    through C38 into the mixer (100 kOhm, as the reference takes it)

    The buffer's input current (Q7A's base) is handed back to pin 6 a sample late, as a current (it barely moves); read
    across 100 Ohm in front of Q7A's base. Both circuits keep their Jacobians between samples (the chord method,
    D281). The chip's transistors as the reference's generic integrated parts: NPN hFE 150, VAF 100 V; lateral PNP
    hFE 50, VAF 50 V; IS 1 fA; diode-wired ones as diodes of IS (1 + 1 / hFE).
*/
namespace coolsynth::synth::v3
{
    class Tb303Vca final
    {
        friend struct Tb303Lid; // the lid tap reads its circuits and state (Tb303Lid.h)
    public:
        static constexpr double thermal = Tb303Filter::thermal;
        static constexpr double supply = Tb303Filter::supply, reference = Tb303Filter::reference;
        static constexpr double saturation = 1.0e-15;
        static constexpr double npnBetaF = 150.0, npnBetaR = 1.0, npnEarly = 100.0;
        static constexpr double pnpBetaF = 50.0, pnpBetaR = 1.0, pnpEarly = 50.0;
        static constexpr double sourceOhms = 1.0e9, senseOhms = 100.0;

        Tb303Vca()
        {
            chip.setNewtonLimit(40);
            buffer.setNewtonLimit(40);
            chip.setNewtonTolerance(Tb303Filter::newtonTolerance);
            buffer.setNewtonTolerance(Tb303Filter::newtonTolerance);
            chip.setJacobianReuse(true);
            buffer.setJacobianReuse(true);
            buildChip();
            buildBuffer();
        }

        /** Where a sample's solve ends in the chip and the buffer (the 303's accuracy, CoolSynth's D349). */
        void setNewtonTolerance(double volts) noexcept
        {
            chip.setNewtonTolerance(volts);
            buffer.setNewtonTolerance(volts);
        }

        /** The chord method (D281), on by default; off solves each sample exactly, for the tests. */
        void setJacobianReuse(bool on) noexcept
        {
            chip.setJacobianReuse(on);
            buffer.setJacobianReuse(on);
        }

        void prepare(double rate) noexcept
        {
            chip.setSampleRate(rate);
            buffer.setSampleRate(rate);
        }

        /** The Devil Fish's fuller bass (CoolSynth's D309): C21, the filter's coupling into pin 3, at 0.1 uF, not
            0.01 uF (32 Hz down 1 dB, not 5 dB, as Whittle measures it). */
        void setFullBass(bool on) noexcept { chip.setCapacitance(cC21, on ? 0.1e-6 : 0.01e-6); }

        /** At rest with this pin 1 voltage and no audio. */
        void settle(double pinOneVolts) noexcept
        {
            setInputs(0.0, 0.0, pinOneVolts);
            baseAmps = 0.0;
            for (int pass = 0; pass < 4; ++pass)
            {
                chip.setSource(sBase, chip.probe(pOut) - baseAmps * sourceOhms);
                chip.settle();
                buffer.setInput(bufIn, chip.probe(pOut));
                buffer.settle();
                baseAmps = (chip.probe(pOut) - buffer.probe(pBase)) / senseOhms;
            }
        }

        /** One sample: the filter's output (C14's side), the RESONANCE wiper's, pin 1's voltage; returns the output. */
        double process(double filterVolts, double resonanceVolts, double pinOneVolts) noexcept
        {
            setInputs(filterVolts, resonanceVolts, pinOneVolts);
            chip.setSource(sBase, lastPin6 - baseAmps * sourceOhms);
            chip.process();
            lastPin6 = chip.probe(pOut);
            buffer.setInput(bufIn, lastPin6);
            buffer.process();
            baseAmps = (lastPin6 - buffer.probe(pBase)) / senseOhms;
            return buffer.probe(pOutput);
        }

        [[nodiscard]] double output() const noexcept { return buffer.probe(pOutput); }
        [[nodiscard]] double pinSix() const noexcept { return chip.probe(pOut); }
        [[nodiscard]] double pinEight() const noexcept { return buffer.probe(pPin8); }
        [[nodiscard]] bool lastSolveConverged() const noexcept { return chip.lastSolveConverged() && buffer.lastSolveConverged(); }
        [[nodiscard]] int newtonSteps() const noexcept { return std::max(chip.lastNewtonSteps(), buffer.lastNewtonSteps()); }
        [[nodiscard]] bool fitsItsLimits() const noexcept { return chip.fitsItsLimits() && buffer.fitsItsLimits(); }
        /** False if the linear part ties some junctions' currents (a group of nodes only junctions leave). */
        [[nodiscard]] bool isWellPosed() noexcept { return chip.isWellPosed() && buffer.isWellPosed(); }

    private:
        void setInputs(double filterVolts, double resonanceVolts, double pinOneVolts) noexcept
        {
            chip.setInput(inSupply, supply);
            chip.setInput(inReference, reference);
            chip.setInput(inFilter, filterVolts);
            chip.setInput(inResonance, resonanceVolts);
            chip.setInput(inPin1, pinOneVolts);
            buffer.setInput(bufSupply, supply);
        }

        // Chip: 1 the 12 V rail, 2 5.333 V, 3 the filter's output, 4 the RESONANCE wiper, 5 pin 1 (inputs); 6 Q1B's
        // emitter, 7 the pair's tail, 8 Q2A's collector, 9 Q2B's, 10 Q3B's base, 11 Q4B's, 12 Q5B's collector (X),
        // 13 Q6B's base, 14 pin 6, 15 C37, 16 pin 2, 17 pin 3, 18 C21's far side, 19 C22's, 20 R129's top, 21 C40.
        void buildChip()
        {
            inSupply = chip.addInput(1);
            inReference = chip.addInput(2);
            inFilter = chip.addInput(3);
            inResonance = chip.addInput(4);
            inPin1 = chip.addInput(5);
            auto npn = [this](int c, int b, int e) { chip.addTransistor(c, b, e, saturation, npnBetaF, npnBetaR, thermal, npnEarly); };
            auto pnp = [this](int c, int b, int e) { chip.addPnpTransistor(c, b, e, saturation, pnpBetaF, pnpBetaR, thermal, pnpEarly); };
            chip.addTransistor(7, 5, 6, Tb303Envelope::chipInputArea * saturation, npnBetaF, npnBetaR, thermal,
                               npnEarly);                                                        // Q1B, as Q1A
            chip.addResistor(6, 0, 500.0);                                                       // its emitter's 500 Ohm
            npn(8, 16, 7);                                                                       // Q2A
            npn(9, 17, 7);                                                                       // Q2B
            pnp(8, 10, 1);                                                                       // Q3A
            chip.addDiode(1, 10, saturation * (1.0 + 1.0 / pnpBetaF), thermal);                 // Q3B, diode-wired
            pnp(12, 8, 10);                                                                      // Q5B
            pnp(9, 11, 1);                                                                       // Q4A
            chip.addDiode(1, 11, saturation * (1.0 + 1.0 / pnpBetaF), thermal);                 // Q4B, diode-wired
            pnp(14, 9, 11);                                                                      // Q5A
            npn(12, 13, 0);                                                                      // Q6A
            chip.addDiode(13, 0, saturation * (1.0 + 1.0 / npnBetaF), thermal);                 // Q6B, diode-wired
            npn(14, 12, 13);                                                                     // Q7B
            chip.addResistor(2, 15, 2.2e3);                                                      // R126
            chip.addCapacitor(15, 0, 10.0e-6);                                                   // C37
            chip.addResistor(15, 16, 2.2e3);                                                     // R125
            chip.addResistor(15, 17, 2.2e3);                                                     // R124
            cC21 = chip.addCapacitor(3, 18, 0.01e-6);                                            // C21
            chip.addResistor(18, 17, 220.0e3);                                                   // R121
            chip.addCapacitor(4, 19, 0.01e-6);                                                   // C22
            chip.addResistor(19, 17, 100.0e3);                                                   // R122
            chip.addResistor(14, 20, 220.0e3);                                                   // R128
            chip.addResistor(14, 20, 47.0e3);                                                    // R127
            chip.addResistor(20, 21, 22.0e3);                                                    // R129
            chip.addResistor(2, 21, 2.2e3);                                                      // R162
            chip.addCapacitor(21, 0, 10.0e-6);                                                   // C40
            sBase = chip.addSource(14, 0, sourceOhms);                                           // the buffer's input current
            // Leaks (as SPICE's GMIN) on the nodes only junctions touch: the tail, the mirrors' inner nodes.
            for (int node = 7; node <= 13; ++node)
                chip.addResistor(node, 0, leakOhms);
            pOut = chip.addProbe(14);
        }

        // Buffer: 1 the 12 V rail, 2 pin 6 (inputs); 3 Q7A's base (100 Ohm on), 4 its emitter, 5 Q9B's base, 6 pin 8,
        // 7 after C38 (the output).
        void buildBuffer()
        {
            bufSupply = buffer.addInput(1);
            bufIn = buffer.addInput(2);
            buffer.addResistor(2, 3, senseOhms);                                                 // where the base current is read
            buffer.addTransistor(1, 3, 4, saturation, npnBetaF, npnBetaR, thermal, npnEarly);    // Q7A
            buffer.addTransistor(4, 5, 0, saturation, npnBetaF, npnBetaR, thermal, npnEarly);    // Q9A
            buffer.addDiode(5, 0, saturation * (1.0 + 1.0 / npnBetaF), thermal);                 // Q9B, diode-wired
            buffer.addResistor(1, 5, 50.0e3);                                                    // R20
            buffer.addPnpTransistor(0, 4, 6, saturation, pnpBetaF, pnpBetaR, thermal, pnpEarly); // Q10A
            buffer.addResistor(1, 6, 11.0e3);                                                    // R21
            buffer.addCapacitor(6, 7, 1.0e-6);                                                   // C38
            buffer.addResistor(7, 0, 100.0e3);                                                   // the mixer
            buffer.addResistor(4, 0, leakOhms);                                                  // a leak on Q7A's emitter
            pBase = buffer.addProbe(3);
            pPin8 = buffer.addProbe(6);
            pOutput = buffer.addProbe(7);
        }

        static constexpr double leakOhms = 1.0e9;
        wdf::Circuit<21, 43, 5, 0, 21, 1, true> chip; // 43 ports: 15 parts, 7 leaks, 9 transistors' 18, 3 diodes
        wdf::Circuit<7, 13, 2, 0, 7, 3, true> buffer; // 13 ports: 5 parts, a leak, 3 transistors' 6, 1 diode

        double baseAmps = 0.0, lastPin6 = 5.333;
        int inSupply = 0, inReference = 0, inFilter = 0, inResonance = 0, inPin1 = 0, sBase = 0, pOut = 0, cC21 = 0;
        int bufSupply = 0, bufIn = 0, pBase = 0, pPin8 = 0, pOutput = 0;
    };
}
