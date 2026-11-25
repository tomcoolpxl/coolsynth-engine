#pragma once

#include <algorithm>
#include <cmath>

#include "CircuitWdf.h"
#include "Tb303Filter.h"

/*
    The Roland TB-303's output stage (CoolSynth's D362), from the service notes' page 5 ("MIXER") as
    tools/tb303/out.cir draws it: the VCA's pin 8 through C38 into VOLUME (VR8, 50k A), its wiper through R155 and C59
    into the mixer, Q34 in common base (its base at 5.333 V through R157, held by C52; its emitter the summing node)
    driving Q33's base through R158 against R154, Q33's collector the output with R160 to ground and R153 back to the
    summing node; the local rail through R136 and C53. Out through R161 and C56 to the OUTPUT jack (R165) and a
    recorder's input; the headphone amp's input (R159, C51 into the LA4140) loads it too.

    The mixer inverts, with a gain of about 2.1, and can pull its output down only through R160 (10k) against its load:
    into a recorder it falls only some 2.1 V under its 4.1 V rest, while it rises some 3 V. Loud notes at a high
    VOLUME clip on that side, as Din Sync's recordings of a real unit do at one level in all 25 sets: the load is
    fitted to that floor (recorderOhms). The x0xb0x wiki notes the 303's distortion at high volume and accent, cured by
    turning VOLUME down.

    In: pin 8's voltage (the VCA's buffer, a follower, as an ideal source: its load barely moves it); out: the jack's
    volts. Q34 as the 2SC945's stand-in (2N3904, its base recombination as the filter's), Q33 as the 2SA733's (2N3906).
    The LA4140's input as 30k (its siblings' data sheets). The chord method (D281) as the other stages.
*/
namespace coolsynth::synth::v3
{
    class Tb303Output final
    {
    public:
        static constexpr double thermal = Tb303Filter::thermal;
        static constexpr double supply = Tb303Filter::supply, reference = Tb303Filter::reference;
        /** The recorder's input on the OUTPUT jack, fitted to the floor Din Sync's recordings clip at with VOLUME full
            (CoolSynth's D362): over the 40 notes that reach it, 10k puts ours 1.0 dB under it, 13k 0.4 dB, 16k
            within 0.01 dB, 22k 0.5 dB over. The accent's loudness then within 0.78 dB rms over all 25 sets (0.96
            without the stage). A line input's usual 10k to 20k. */
        static constexpr double recorderOhms = 16.0e3;
        static constexpr double headphoneOhms = 30.0e3; // the LA4140's input

        Tb303Output()
        {
            circuit.setNewtonLimit(40);
            circuit.setNewtonTolerance(Tb303Filter::newtonTolerance);
            circuit.setJacobianReuse(true);
            build();
            setVolume(1.0);
        }

        void setNewtonTolerance(double volts) noexcept { circuit.setNewtonTolerance(volts); }
        void setJacobianReuse(bool on) noexcept { circuit.setJacobianReuse(on); }
        void prepare(double rate) noexcept { circuit.setSampleRate(rate); }

        /** VOLUME's turn (0..1): an audio-taper pot, its share from pin 1 as the 303's other A pots. */
        void setVolume(double turn) noexcept
        {
            const double share = Tb303Filter::audioTaper(turn);
            circuit.setResistance(rVolTop, std::max(50.0e3 * (1.0 - share), 1.0));
            circuit.setResistance(rVolBottom, std::max(50.0e3 * share, 1.0));
        }

        /** The recorder's input (the jack's load), ohms. */
        void setLoad(double ohms) noexcept { circuit.setResistance(rLoad, std::max(ohms, 1.0)); }

        /** At rest with pin 8 at this voltage. */
        void settle(double pinEightVolts) noexcept
        {
            setInputs(pinEightVolts);
            circuit.settle();
        }

        /** One sample: pin 8's voltage in, the jack's out. */
        double process(double pinEightVolts) noexcept
        {
            setInputs(pinEightVolts);
            circuit.process();
            return circuit.probe(pJack);
        }

        [[nodiscard]] double output() const noexcept { return circuit.probe(pJack); }
        /** Q33's collector, the mixer's own output. */
        [[nodiscard]] double mixer() const noexcept { return circuit.probe(pMixer); }
        [[nodiscard]] bool lastSolveConverged() const noexcept { return circuit.lastSolveConverged(); }
        [[nodiscard]] bool fitsItsLimits() const noexcept { return circuit.fitsItsLimits(); }
        [[nodiscard]] bool isWellPosed() noexcept { return circuit.isWellPosed(); }

    private:
        void setInputs(double pinEightVolts) noexcept
        {
            circuit.setInput(inSupply, supply);
            circuit.setInput(inReference, reference);
            circuit.setInput(inPin8, pinEightVolts);
        }

        // 1 the 12 V rail, 2 5.333 V, 3 pin 8 (inputs); 4 VOLUME's top (after C38), 5 its wiper, 6 between R155 and
        // C59, 7 Q34's emitter (the summing node), 8 its base, 9 its collector, 10 the local rail (C53), 11 Q33's
        // base, 12 its collector (the mixer's output), 13 between R161 and C56, 14 the jack, 15 between R159 and C51,
        // 16 the LA4140's input.
        void build()
        {
            inSupply = circuit.addInput(1);
            inReference = circuit.addInput(2);
            inPin8 = circuit.addInput(3);
            circuit.addCapacitor(3, 4, 1.0e-6);                                                  // C38
            rVolTop = circuit.addResistor(4, 5, 25.0e3);                                         // VOLUME above its wiper
            rVolBottom = circuit.addResistor(5, 0, 25.0e3);                                      // and below
            circuit.addResistor(5, 6, 47.0e3);                                                   // R155
            circuit.addCapacitor(6, 7, 1.0e-6);                                                  // C59
            circuit.addResistor(2, 8, 10.0e3);                                                   // R157
            circuit.addCapacitor(8, 0, 10.0e-6);                                                 // C52
            circuit.addTransistor(9, 8, 7, Tb303Filter::npnSaturation, Tb303Filter::npnBetaF, Tb303Filter::npnBetaR,
                                  thermal, Tb303Filter::npnEarly, Tb303Filter::npnRecombination,
                                  thermal * Tb303Filter::npnRecombinationEmission);              // Q34
            circuit.addResistor(1, 10, 100.0);                                                   // R136
            circuit.addCapacitor(10, 0, 47.0e-6);                                                // C53
            circuit.addResistor(10, 9, 100.0e3);                                                 // R154
            circuit.addResistor(9, 11, 10.0e3);                                                  // R158
            circuit.addPnpTransistor(12, 11, 10, Tb303Filter::pnpSaturation, Tb303Filter::pnpBetaF,
                                     Tb303Filter::pnpBetaR, thermal, Tb303Filter::pnpEarly);     // Q33
            circuit.addResistor(12, 0, 10.0e3);                                                  // R160
            circuit.addResistor(12, 7, 100.0e3);                                                 // R153
            circuit.addResistor(12, 13, 1.0e3);                                                  // R161
            circuit.addCapacitor(13, 14, 1.0e-6);                                                // C56
            circuit.addResistor(14, 0, 100.0e3);                                                 // R165
            rLoad = circuit.addResistor(14, 0, recorderOhms);                                    // the recorder
            circuit.addResistor(12, 15, 33.0e3);                                                 // R159
            circuit.addCapacitor(15, 16, 2.2e-6);                                                // C51
            circuit.addResistor(16, 0, headphoneOhms);                                           // the LA4140
            pJack = circuit.addProbe(14);
            pMixer = circuit.addProbe(12);
        }

        // 24 ports: 20 parts, 2 transistors' 4
        wdf::Circuit<16, 24, 3, 0, 4, 2> circuit;
        int inSupply = 0, inReference = 0, inPin8 = 0, rVolTop = 0, rVolBottom = 0, rLoad = 0, pJack = 0, pMixer = 0;
    };
}
