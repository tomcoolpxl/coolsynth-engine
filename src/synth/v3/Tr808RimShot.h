#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include "CircuitWdf.h"
#include "Tr808BassDrum.h"
#include "Tr808Tom.h"

/*
    The Roland TR-808's rim shot or claves (D252), one circuit with its switch (SW11) as on the machine, from the
    service notes (pp. 6, 9) and tools/tr808/rimclave.cir, its reference. Five circuits, each a wave digital filter with
    an R-type root (CircuitWdf.h), each driving the next one way, as an ideal source:

        Front       the trigger as the bass drum's: Q32 onto the TRIG line, the PNP switch Q60 (R299, R298 through Q59),
                    R300 as its load; the two resonators' pulses (C111, R307, D62, R301 over R302; C114, R306, D64,
                    R310 over R311); the envelope, C24 charged through D61 and R107, given up through R108
        IC21        a bridged-T, C116 and C115 4.7 nF, R316 1 MOhm across, R315 5.6 kOhm to ground: about 455 Hz
        IC20        a bridged-T, C119 and C117 2.2 nF, R308 820 kOhm across, R312 to ground, IC20b inverting it
                    (R314, R309); for the rim shot C118 across C117 (about 1.7 kHz), for the claves R313 from the
                    middle node to IC20b's output (about 2.5 kHz)
        Swing VCA   IC21 through R318 and IC20b through R320 onto R317, through C120 into Q62's base (R321 2.2 MOhm
                    from the supply); its collector fed by the envelope through R108 and D63; then Q63's high-pass
                    (C121, C122, R325, R322 from its emitter, R323 to the negative supply), loaded by the rim shot's
                    R324, C125 and R326 into the mixer's virtual ground
        Mixer       IC19: the rim shot from Q63 through R324, or the claves from IC20b through R319; C125 and R326
                    into its minus input, R330 470 kOhm, C153 22 pF and the LEVEL pot (100 kOhm A) with C126 in its
                    feedback

    R108 joins the front and the VCA a sample apart, each circuit holding the other's end as a source: 10 kOhm against
    C24's 0.47 uF is 4.7 ms, hundreds of samples. R313, printed 390 kOhm (with which the claves ring on), is 500 kOhm,
    fitted to both real units' claves decay; R324, which the service notes leave unprinted, is 130 kOhm, fitted to the
    units' rim shot over their claves (D252); R312, printed 1 kOhm (with which IC20 rings some 3 % sharp in both
    positions), is 1.07 kOhm, fitted to both units' rim shot and claves pitch (D258). Transistors and diodes as the
    bass drum's; the resonators' op-amps of
    3 MHz gain-bandwidth and a +-14 V swing, as the toms', the mixer's ideal. Volts in, volts out.
*/
namespace coolsynth::synth::v3
{
    class Tr808RimShot final
    {
    public:
        static constexpr double thermal = Tr808BassDrum::thermal;
        static constexpr double transistorSaturation = Tr808BassDrum::transistorSaturation;
        static constexpr double betaF = Tr808BassDrum::betaF, betaR = Tr808BassDrum::betaR;
        static constexpr double diodeSaturation = Tr808BassDrum::diodeSaturation;
        static constexpr double diodeThermal = thermal * Tr808BassDrum::diodeEmission;
        static constexpr double supply = 15.0;
        static constexpr double followerDrop = Tr808BassDrum::followerDrop;
        static constexpr double instrumentHold = Tr808BassDrum::instrumentHold;
        static constexpr double switchOff = Tr808BassDrum::switchOff;
        static constexpr double absent = Tr808Tom::absent;
        static constexpr double r313Ohms = 500.0e3, r324Ohms = 130.0e3; // fitted (D252)
        static constexpr double r312Ohms = 1.07e3; // printed 1 kOhm: both units' rim shot and claves pitch (D258)

        Tr808RimShot()
        {
            front.setNewtonLimit(40); // the trigger's edges switch two transistors and three junctions at once
            vca.setNewtonLimit(40);
            solveTo808Tolerance(front, low, high, vca, mixer); // D356
            buildFront();
            buildLow();
            buildHigh();
            buildVca();
            buildMixer();
            setClaves(false);
        }

        /** At a sample rate, at rest (a copy of the solved rest state: cheap enough for the audio thread). */
        void prepare(double rate) noexcept
        {
            sampleRate = rate;
            front.setSampleRate(rate);
            low.setSampleRate(rate);
            high.setSampleRate(rate);
            vca.setSampleRate(rate);
            mixer.setSampleRate(rate);
            rest();
            restoreOnSwitch = true; // from here a switch at rest restores its rest (the solving of it does not)
        }

        /** Back to rest: a copy of the solved rest state, no solving. */
        void rest() noexcept
        {
            const RestState& r = restState(isClaves);
            front.setResistance(r298, switchOff);
            front.restore(r.front);
            low.restore(r.low);
            high.restore(r.high);
            vca.restore(r.vca);
            mixer.restore(r.mixer);
            triggerSample = -1;
            frontResting = vcaResting = true;
            lastEnvelope = r.envelopeVolts;
            lastFeed = r.feedVolts;
            lastFollower = r.followerVolts;
            stillSamples = vcaStill = 0;
            setRestInputs();
        }

        /** The rest state, solved on first use and kept for the process: call it off the audio thread. */
        static void warmUp() noexcept
        {
            (void) restState(false);
            (void) restState(true);
        }

        /** Back to the circuit at rest by solving it: capacitors charged as they are with no trigger. */
        void settle() noexcept
        {
            triggerSample = -1;
            front.setResistance(r298, switchOff);
            lastEnvelope = lastFeed = lastFollower = 0.0;
            setRestInputs();
            for (int pass = 0; pass < 3; ++pass)
            {
                front.setSource(r108Front, lastFeed);
                front.settle();
                lastEnvelope = front.probe(frontEnvelope);
                low.setInput(0, front.probe(frontLow));
                low.settle();
                high.setInput(0, front.probe(frontHigh));
                high.settle();
                vca.setInput(vcaLow, low.probe(lowOut));
                vca.setInput(vcaHigh, high.probe(highOut));
                vca.setInput(vcaEnvelope, lastEnvelope);
                vca.settle();
                lastFeed = vca.probe(vcaFeed);
                lastFollower = vca.probe(vcaFollower);
            }
            mixer.setInput(mixerRim, lastFollower);
            mixer.setInput(mixerClaves, high.probe(highOut));
            mixer.settle();
            frontResting = vcaResting = true;
            stillSamples = vcaStill = 0;
        }

        /** A hit: the TRIG line at this many volts (4 unaccented to 14), for this long. */
        void trigger(double volts, double widthSeconds = 0.001) noexcept
        {
            triggerVolts = volts;
            triggerWidth = widthSeconds;
            triggerSample = 0;
            front.setResistance(r298, 10.0e3); // the instrument data stands: Q59 on
        }

        /** SW11: the claves take C118 out, R313 in, and the mixer's input from IC20b instead of Q63. Thrown while the
            drum rests, it starts from that position's own rest; while it rings, from where the ring is. */
        void setClaves(bool claves) noexcept
        {
            isClaves = claves;
            high.setCapacitance(c118, claves ? absent : 2.2e-9);
            high.setResistance(r313, claves ? r313Ohms : switchOff);
            vca.setResistance(r324Vca, claves ? switchOff : r324Ohms);
            mixer.setResistance(r324Mixer, claves ? switchOff : r324Ohms);
            mixer.setResistance(r319, claves ? 220.0e3 : switchOff);
            if (restoreOnSwitch && triggerSample < 0 && frontResting && vcaResting)
                rest();
        }
        [[nodiscard]] bool claves() const noexcept { return isClaves; }

        /** For fitting the rim shot's rings (tests only, D258): IC20's R308 and R312, the mix's R318 and R320. Settle
            the drum after: R318 and R320 carry the rings' bias at rest. */
        void setRingParts(double r308, double r312, double r318, double r320) noexcept
        {
            high.setResistance(r308Port, r308);
            high.setResistance(r312Port, r312);
            vca.setResistance(r318Port, r318);
            vca.setResistance(r320Port, r320);
        }

        /** LEVEL 0..1: the 100 kOhm audio-taper pot in IC19's feedback. */
        void setLevel(double turn) noexcept { mixer.setResistance(vr16, Tr808Tom::levelOhms(turn)); }

        /** One sample; returns the output in volts. */
        double process() noexcept
        {
            // Q32's base, as the bass drum's (Tr808BassDrum::process): the step half a sample into the hit.
            double drive = 0.0;
            if (triggerSample >= 0)
            {
                const int width = std::max(1, static_cast<int>(std::lround(triggerWidth * sampleRate)));
                if (triggerSample >= 1 && triggerSample <= width)
                    drive = triggerVolts + followerDrop;
                if (triggerSample >= width + static_cast<int>(std::lround(instrumentHold * sampleRate)))
                {
                    front.setResistance(r298, switchOff);
                    triggerSample = -1;
                }
                else
                    ++triggerSample;
            }
            // The front rests once the hit has passed and the envelope has given up its charge; the VCA with it for
            // the claves (it only drains their envelope), after the tail for the rim shot.
            const bool active = triggerSample >= 0;
            if (active || ! frontResting)
            {
                front.setInput(frontDrive, drive);
                front.setSource(r108Front, lastFeed);
                front.process();
                const double e = front.probe(frontEnvelope);
                stillSamples = std::abs(e - lastEnvelope) < 1.0e-6 ? stillSamples + 1 : 0;
                lastEnvelope = e;
                frontResting = ! active && std::abs(front.probe(frontTrigger)) < 1.0e-6 && std::abs(front.probe(frontLow)) < 1.0e-9
                               && std::abs(front.probe(frontHigh)) < 1.0e-9 && stillSamples > static_cast<int>(0.005 * sampleRate);
            }
            low.setInput(0, frontResting ? 0.0 : front.probe(frontLow));
            low.process();
            high.setInput(0, frontResting ? 0.0 : front.probe(frontHigh));
            high.process();
            if (active || ! frontResting || (! isClaves && ! vcaResting))
            {
                vca.setInput(vcaLow, low.probe(lowOut));
                vca.setInput(vcaHigh, high.probe(highOut));
                vca.setInput(vcaEnvelope, lastEnvelope);
                vca.process();
                const double f = vca.probe(vcaFollower);
                vcaStill = std::abs(f - lastFollower) < 1.0e-7 ? vcaStill + 1 : 0;
                lastFeed = vca.probe(vcaFeed);
                lastFollower = f;
                vcaResting = frontResting && vcaStill > static_cast<int>(0.005 * sampleRate);
            }
            mixer.setInput(mixerRim, lastFollower);
            mixer.setInput(mixerClaves, high.probe(highOut));
            mixer.process();
            return mixer.probe(mixerOut);
        }

        // ---- Its circuit as it runs, for a panel that draws it (CoolSynth's D317) ---------------------------------
        // The node voltages and part currents its ngspice recording names (CoolSynth808's resources/lid, the rim
        // shot's sheet, which the claves share), each from the stage the engine models it in: where a part is in two
        // stages, the one it really sits in (R108 in the VCA, which sees both its ends at this sample where the front
        // holds the VCA's end a sample late; R324, C125 and R326 in the mixer, right in both of SW11's positions,
        // where the VCA's are only Q63's load and miss the claves' way through R319), and a node fed on as an ideal
        // input read in the stage it is built in. Signs as ngspice's: a part's current from its first node to its
        // second, a transistor's into its leads. Not here, as the engine leaves them out: the common trigger's Q31,
        // R117 and R120 and its nodes n6, nb31 and nx (the engine drives Q32's base, Q31 gone), and the select line's
        // R297, R303 and Q59 and its nodes nrs, nb59 and nc59 (R298 switches to ground in their place).

        static constexpr int liveCount = 95;
        static constexpr double liveMs = 150.0; // the recording's length
        static constexpr std::array<const char*, liveCount> liveNames { "nb32", "vcc", "trig", "nb60", "vt", "na21", "nb21", "np21", "na20", "nb20", "np20", "n61", "nenv", "ninv21", "nm21", "no21", "ninv20", "nm20", "no20a", "nin20b", "no20b", "ns", "nb62", "n108", "nf", "ng", "nb63", "ne63", "vee", "nsw", "nc125", "nsum", "vout", "nvr", "q32.ib", "q32.ic", "q32.ie", "r113.i", "r299.i", "r298.i", "q60.ib", "q60.ic", "q60.ie", "r300.i", "c111.i", "r307.i", "d62.id", "r301.i", "r302.i", "c114.i", "r306.i", "d64.id", "r310.i", "r311.i", "d61.id", "r107.i", "c24.i", "c116.i", "c115.i", "r316.i", "r315.i", "c119.i", "c117.i", "c118.i", "r308.i", "r312.i", "r313.i", "r314.i", "r309.i", "r318.i", "r320.i", "r317.i", "c120.i", "r321.i", "r108.i", "d63.id", "q62.ib", "q62.ic", "q62.ie", "c121.i", "c122.i", "r325.i", "r322.i", "q63.ib", "q63.ic", "q63.ie", "r323.i", "r324.i", "r319.i", "c125.i", "r326.i", "r330.i", "c153.i", "rvr16.i", "c126.i" };

        /** The named values after the last sample, in liveNames' order, from the stages' own state (their taps). */
        void tap(float* out) const noexcept
        {
            // stage: 0 front, 1 IC21, 2 IC20, 3 the swing VCA, 4 the mixer. kind: 0 a node's voltage, 1 a port's
            // current, 2 a transistor's emitter current from its base port (its collector port the next). sign:
            // against the netlist's node order (all as it).
            struct Entry { int stage, kind, index, sign; };
            static constexpr std::array<Entry, liveCount> entries { {
              { 0, 0, 2, 1 },   // nb32
              { 0, 0, 3, 1 },   // vcc
              { 0, 0, 4, 1 },   // trig
              { 0, 0, 5, 1 },   // nb60
              { 0, 0, 6, 1 },   // vt
              { 0, 0, 7, 1 },   // na21
              { 0, 0, 8, 1 },   // nb21
              { 0, 0, 9, 1 },   // np21
              { 0, 0, 10, 1 },  // na20
              { 0, 0, 11, 1 },  // nb20
              { 0, 0, 12, 1 },  // np20
              { 0, 0, 13, 1 },  // n61
              { 0, 0, 14, 1 },  // nenv
              { 1, 0, 2, 1 },   // ninv21
              { 1, 0, 3, 1 },   // nm21
              { 1, 0, 4, 1 },   // no21
              { 2, 0, 2, 1 },   // ninv20
              { 2, 0, 3, 1 },   // nm20
              { 2, 0, 4, 1 },   // no20a
              { 2, 0, 5, 1 },   // nin20b
              { 2, 0, 6, 1 },   // no20b
              { 3, 0, 3, 1 },   // ns
              { 3, 0, 4, 1 },   // nb62
              { 3, 0, 7, 1 },   // n108
              { 3, 0, 8, 1 },   // nf
              { 3, 0, 9, 1 },   // ng
              { 3, 0, 10, 1 },  // nb63
              { 3, 0, 11, 1 },  // ne63
              { 3, 0, 12, 1 },  // vee
              { 4, 0, 3, 1 },   // nsw
              { 4, 0, 4, 1 },   // nc125
              { 4, 0, 5, 1 },   // nsum
              { 4, 0, 6, 1 },   // vout
              { 4, 0, 7, 1 },   // nvr
              { 0, 1, 1, 1 },   // q32.ib
              { 0, 1, 2, 1 },   // q32.ic
              { 0, 2, 1, 1 },   // q32.ie
              { 0, 1, 3, 1 },   // r113.i
              { 0, 1, 4, 1 },   // r299.i
              { 0, 1, 5, 1 },   // r298.i
              { 0, 1, 6, 1 },   // q60.ib
              { 0, 1, 7, 1 },   // q60.ic
              { 0, 2, 6, 1 },   // q60.ie
              { 0, 1, 8, 1 },   // r300.i
              { 0, 1, 9, 1 },   // c111.i
              { 0, 1, 10, 1 },  // r307.i
              { 0, 1, 11, 1 },  // d62.id
              { 0, 1, 12, 1 },  // r301.i
              { 0, 1, 13, 1 },  // r302.i
              { 0, 1, 14, 1 },  // c114.i
              { 0, 1, 15, 1 },  // r306.i
              { 0, 1, 16, 1 },  // d64.id
              { 0, 1, 17, 1 },  // r310.i
              { 0, 1, 18, 1 },  // r311.i
              { 0, 1, 19, 1 },  // d61.id
              { 0, 1, 20, 1 },  // r107.i
              { 0, 1, 21, 1 },  // c24.i
              { 1, 1, 0, 1 },   // c116.i
              { 1, 1, 1, 1 },   // c115.i
              { 1, 1, 2, 1 },   // r316.i
              { 1, 1, 3, 1 },   // r315.i
              { 2, 1, 0, 1 },   // c119.i
              { 2, 1, 1, 1 },   // c117.i
              { 2, 1, 2, 1 },   // c118.i
              { 2, 1, 3, 1 },   // r308.i
              { 2, 1, 4, 1 },   // r312.i
              { 2, 1, 5, 1 },   // r313.i
              { 2, 1, 6, 1 },   // r314.i
              { 2, 1, 7, 1 },   // r309.i
              { 3, 1, 0, 1 },   // r318.i
              { 3, 1, 1, 1 },   // r320.i
              { 3, 1, 2, 1 },   // r317.i
              { 3, 1, 3, 1 },   // c120.i
              { 3, 1, 4, 1 },   // r321.i
              { 3, 1, 5, 1 },   // r108.i
              { 3, 1, 6, 1 },   // d63.id
              { 3, 1, 7, 1 },   // q62.ib
              { 3, 1, 8, 1 },   // q62.ic
              { 3, 2, 7, 1 },   // q62.ie
              { 3, 1, 9, 1 },   // c121.i
              { 3, 1, 10, 1 },  // c122.i
              { 3, 1, 11, 1 },  // r325.i
              { 3, 1, 12, 1 },  // r322.i
              { 3, 1, 13, 1 },  // q63.ib
              { 3, 1, 14, 1 },  // q63.ic
              { 3, 2, 13, 1 },  // q63.ie
              { 3, 1, 15, 1 },  // r323.i
              { 4, 1, 0, 1 },   // r324.i
              { 4, 1, 1, 1 },   // r319.i
              { 4, 1, 2, 1 },   // c125.i
              { 4, 1, 3, 1 },   // r326.i
              { 4, 1, 4, 1 },   // r330.i
              { 4, 1, 5, 1 },   // c153.i
              { 4, 1, 6, 1 },   // rvr16.i
              { 4, 1, 7, 1 } } }; // c126.i
            // each stage's node voltages, once (ground and every node: MaxNodes + 1)
            std::array<double, 15> frontV {};
            std::array<double, 6> lowV {};
            std::array<double, 9> highV {};
            std::array<double, 15> vcaV {};
            std::array<double, 8> mixerV {};
            front.nodeVoltages(frontV.data());
            low.nodeVoltages(lowV.data());
            high.nodeVoltages(highV.data());
            vca.nodeVoltages(vcaV.data());
            mixer.nodeVoltages(mixerV.data());
            for (int k = 0; k < liveCount; ++k)
            {
                const auto& e = entries[static_cast<std::size_t>(k)];
                const auto read = [&e](const auto& c, const double* v) -> double
                {
                    switch (e.kind)
                    {
                        case 0: return v[e.index];
                        case 1: return c.portCurrent(e.index, v);
                        default: return -(c.portCurrent(e.index, v) + c.portCurrent(e.index + 1, v));
                    }
                };
                double value = 0.0;
                switch (e.stage)
                {
                    case 0: value = read(front, frontV.data()); break;
                    case 1: value = read(low, lowV.data()); break;
                    case 2: value = read(high, highV.data()); break;
                    case 3: value = read(vca, vcaV.data()); break;
                    default: value = read(mixer, mixerV.data()); break;
                }
                out[k] = static_cast<float>(e.sign * value);
            }
        }

        // The stages' last outputs, for tests against the reference.
        [[nodiscard]] double triggerNodeVolts() const noexcept { return front.probe(frontTrigger); }
        [[nodiscard]] double envelopeVolts() const noexcept { return lastEnvelope; }
        [[nodiscard]] double lowVolts() const noexcept { return low.probe(lowOut); }
        [[nodiscard]] double highVolts() const noexcept { return high.probe(highOut); }
        [[nodiscard]] double swingVolts() const noexcept { return vca.probe(vcaSwing); }
        [[nodiscard]] double collectorVolts() const noexcept { return vca.probe(vcaCollector); }
        [[nodiscard]] double followerVolts() const noexcept { return lastFollower; }
        [[nodiscard]] int newtonSteps() const noexcept
        {
            return std::max(frontResting ? 0 : front.lastNewtonSteps(), vcaResting ? 0 : vca.lastNewtonSteps());
        }
        [[nodiscard]] bool frontIsResting() const noexcept { return frontResting && vcaResting; }
        [[nodiscard]] bool lastSolveConverged() const noexcept { return front.lastSolveConverged() && vca.lastSolveConverged(); }

    private:
        void setRestInputs() noexcept
        {
            front.setInput(frontDrive, 0.0);
            front.setInput(frontSupply, supply);
            front.setSource(r108Front, lastFeed);
            low.setInput(0, 0.0);
            high.setInput(0, 0.0);
            vca.setInput(vcaLow, 0.0);
            vca.setInput(vcaHigh, 0.0);
            vca.setInput(vcaSupply, supply);
            vca.setInput(vcaEnvelope, lastEnvelope);
            vca.setInput(vcaNegative, -supply);
            mixer.setInput(mixerRim, lastFollower);
            mixer.setInput(mixerClaves, 0.0);
        }

        // Front: 1 Q32's drive (input), 2 its base, 3 the supply (input), 4 the TRIG line, 5 Q60's base, 6 its
        // collector (the trigger), 7 after C111, 8 D62's cathode, 9 IC21's plus input, 10 after C114, 11 D64's
        // cathode, 12 IC20a's plus input, 13 D61's cathode, 14 C24 (the envelope).
        void buildFront()
        {
            frontDrive = front.addInput(1);
            frontSupply = front.addInput(3);
            front.addResistor(1, 2, 4.7e3);                                               // R120
            front.addTransistor(3, 2, 4, transistorSaturation, betaF, betaR, thermal);    // Q32
            front.addResistor(4, 0, 4.7e3);                                               // R113
            front.addResistor(4, 5, 4.7e3);                                               // R299
            r298 = front.addResistor(5, 0, switchOff);                                    // R298 through Q59: open but while the data stands
            front.addPnpTransistor(6, 5, 4, transistorSaturation, betaF, betaR, thermal); // Q60
            front.addResistor(6, 0, 22.0e3);                                              // R300
            front.addCapacitor(6, 7, 10.0e-9);                                            // C111
            front.addResistor(7, 0, 100.0e3);                                             // R307
            front.addDiode(7, 8, diodeSaturation, diodeThermal);                          // D62
            front.addResistor(8, 9, 47.0e3);                                              // R301
            front.addResistor(9, 0, 10.0e3);                                              // R302
            front.addCapacitor(6, 10, 1.0e-9);                                            // C114
            front.addResistor(10, 0, 22.0e3);                                             // R306
            front.addDiode(10, 11, diodeSaturation, diodeThermal);                        // D64
            front.addResistor(11, 12, 33.0e3);                                            // R310
            front.addResistor(12, 0, 10.0e3);                                             // R311
            front.addDiode(6, 13, diodeSaturation, diodeThermal);                         // D61
            front.addResistor(13, 14, 1.0e3);                                             // R107
            front.addCapacitor(14, 0, 0.47e-6);                                           // C24
            r108Front = front.addSource(14, 0, 10.0e3);                                   // R108, to D63's anode in the VCA
            frontTrigger = front.addProbe(6);
            frontLow = front.addProbe(9);
            frontHigh = front.addProbe(12);
            frontEnvelope = front.addProbe(14);
        }

        // IC21: 1 the plus input (input), 2 the minus, 3 the middle node, 4 the output, 5 the op-amp's inner node.
        void buildLow()
        {
            low.addInput(1);
            low.addCapacitor(2, 3, 4.7e-9);  // C116
            low.addCapacitor(3, 4, 4.7e-9);  // C115
            low.addResistor(2, 4, 1.0e6);    // R316
            low.addResistor(3, 0, 5.6e3);    // R315
            low.addOpAmp(1, 2, 4, 5, Tr808Tom::opAmpBandwidth, Tr808Tom::opAmpRail); // IC21
            lowOut = low.addProbe(4);
        }

        // IC20: 1 IC20a's plus input (input), 2 its minus, 3 the middle node, 4 its output, 5 IC20b's minus, 6 its
        // output, 7 and 8 the op-amps' inner nodes.
        void buildHigh()
        {
            high.addInput(1);
            high.addCapacitor(2, 3, 2.2e-9);               // C119
            high.addCapacitor(3, 4, 2.2e-9);               // C117
            c118 = high.addCapacitor(3, 4, 2.2e-9);        // C118, the rim shot's
            r308Port = high.addResistor(2, 4, 820.0e3);    // R308
            r312Port = high.addResistor(3, 0, r312Ohms);   // R312
            r313 = high.addResistor(3, 6, switchOff);      // R313, the claves'
            high.addResistor(4, 5, 33.0e3);                // R314
            high.addResistor(5, 6, 33.0e3);                // R309
            high.addOpAmp(1, 2, 4, 7, Tr808Tom::opAmpBandwidth, Tr808Tom::opAmpRail); // IC20a
            high.addOpAmp(0, 5, 6, 8, Tr808Tom::opAmpBandwidth, Tr808Tom::opAmpRail); // IC20b
            highOut = high.addProbe(6);
        }

        // Swing VCA: 1 IC21 (input), 2 IC20b (input), 3 the swing node, 4 Q62's base, 5 the supply (input), 6 C24
        // (input), 7 D63's anode, 8 Q62's collector, 9 the high-pass's middle, 10 Q63's base, 11 its emitter, 12 the
        // negative supply (input), 13 the rim shot's way to the mixer, 14 after C125.
        void buildVca()
        {
            vcaLow = vca.addInput(1);
            vcaHigh = vca.addInput(2);
            vcaSupply = vca.addInput(5);
            vcaEnvelope = vca.addInput(6);
            vcaNegative = vca.addInput(12);
            r318Port = vca.addResistor(1, 3, 220.0e3);                                  // R318
            r320Port = vca.addResistor(2, 3, 22.0e3);                                   // R320
            vca.addResistor(3, 0, 10.0e3);                                              // R317
            vca.addCapacitor(3, 4, 10.0e-6);                                            // C120
            vca.addResistor(5, 4, 2.2e6);                                               // R321
            vca.addResistor(6, 7, 10.0e3);                                              // R108
            vca.addDiode(7, 8, diodeSaturation, diodeThermal);                          // D63
            vca.addTransistor(8, 4, 0, transistorSaturation, betaF, betaR, thermal);    // Q62
            vca.addCapacitor(8, 9, 4.7e-9);                                             // C121
            vca.addCapacitor(9, 10, 4.7e-9);                                            // C122
            vca.addResistor(10, 0, 330.0e3);                                            // R325
            vca.addResistor(9, 11, 10.0e3);                                             // R322
            vca.addTransistor(5, 10, 11, transistorSaturation, betaF, betaR, thermal);  // Q63
            vca.addResistor(11, 12, 22.0e3);                                            // R323
            r324Vca = vca.addResistor(11, 13, r324Ohms);                                // R324
            vca.addCapacitor(13, 14, 10.0e-9);                                          // C125
            vca.addResistor(14, 0, 10.0e3);                                             // R326, to IC19's virtual ground
            vcaSwing = vca.addProbe(3);
            vcaFeed = vca.addProbe(7);
            vcaCollector = vca.addProbe(8);
            vcaFollower = vca.addProbe(11);
        }

        // Mixer: 1 Q63's emitter (input), 2 IC20b (input), 3 the switch, 4 after C125, 5 IC19's minus, 6 its output,
        // 7 the LEVEL pot to C126.
        void buildMixer()
        {
            mixerRim = mixer.addInput(1);
            mixerClaves = mixer.addInput(2);
            r324Mixer = mixer.addResistor(1, 3, r324Ohms);       // R324
            r319 = mixer.addResistor(2, 3, switchOff);           // R319
            mixer.addCapacitor(3, 4, 10.0e-9);                   // C125
            mixer.addResistor(4, 5, 10.0e3);                     // R326
            mixer.addResistor(5, 6, 470.0e3);                    // R330
            mixer.addCapacitor(5, 6, 22.0e-12);                  // C153
            vr16 = mixer.addResistor(5, 7, Tr808Tom::levelOhms(0.8)); // VR16, LEVEL
            mixer.addCapacitor(7, 6, 33.0e-6);                   // C126
            mixer.addNullor(0, 5, 6);                            // IC19
            mixerOut = mixer.addProbe(6);
        }

        wdf::Circuit<14, 23, 2, 0, 7, 4, true> front; // Tapped: a panel reads its nodes and currents (D317)
        wdf::Circuit<5, 7, 1, 1, 1, 1, true> low;
        wdf::Circuit<8, 14, 1, 2, 2, 1, true> high;
        wdf::Circuit<14, 19, 5, 0, 5, 4, true> vca;
        wdf::Circuit<7, 8, 2, 1, 0, 1, true> mixer;

        struct RestState
        {
            decltype(front)::Snapshot front {};
            decltype(low)::Snapshot low {};
            decltype(high)::Snapshot high {};
            decltype(vca)::Snapshot vca {};
            decltype(mixer)::Snapshot mixer {};
            double envelopeVolts = 0.0, feedVolts = 0.0, followerVolts = 0.0;
        };

        /** The rest in each SW11 position (D258): the claves take the mixer's input from IC20b, at another DC than Q63's,
            so a claves hit from the rim shot's rest would recharge the mixer's coupling over tens of milliseconds. */
        static const RestState& restState(bool clavesSide) noexcept
        {
            static const auto solve = [](bool claves)
            {
                auto drum = std::make_unique<Tr808RimShot>();
                drum->setClaves(claves);
                drum->settle();
                RestState s;
                s.front = drum->front.snapshot();
                s.low = drum->low.snapshot();
                s.high = drum->high.snapshot();
                s.vca = drum->vca.snapshot();
                s.mixer = drum->mixer.snapshot();
                s.envelopeVolts = drum->lastEnvelope;
                s.feedVolts = drum->lastFeed;
                s.followerVolts = drum->lastFollower;
                return s;
            };
            static const RestState rimSide = solve(false);
            static const RestState clavesSide2 = solve(true);
            return clavesSide ? clavesSide2 : rimSide;
        }

        double sampleRate = 96000.0;
        double triggerVolts = 4.0, triggerWidth = 0.001;
        int triggerSample = -1; // samples since the hit, -1 between hits
        bool frontResting = true, vcaResting = true, isClaves = false, restoreOnSwitch = false;
        int r308Port = 0, r312Port = 0, r318Port = 0, r320Port = 0;
        double lastEnvelope = 0.0, lastFeed = 0.0, lastFollower = 0.0;
        int stillSamples = 0, vcaStill = 0;

        int frontDrive = 0, frontSupply = 0, frontTrigger = 0, frontLow = 0, frontHigh = 0, frontEnvelope = 0, r298 = 0, r108Front = 0;
        int lowOut = 0, highOut = 0, c118 = 0, r313 = 0;
        int vcaLow = 0, vcaHigh = 0, vcaSupply = 0, vcaEnvelope = 0, vcaNegative = 0, vcaSwing = 0, vcaFeed = 0, vcaCollector = 0, vcaFollower = 0, r324Vca = 0;
        int mixerRim = 0, mixerClaves = 0, mixerOut = 0, r324Mixer = 0, r319 = 0, vr16 = 0;
    };
}
