#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include "CircuitWdf.h"
#include "Tr808BassDrum.h"
#include "Tr808Tom.h"

/*
    The Roland TR-808's snare drum (D256), in its changed version (the service notes' page 9, "portion changed": C58
    0.056 uF, C61 0.015 uF, R191 10 kOhm, R200 a link), from the service notes (pp. 6, 9) and tools/tr808/snare.cir,
    its reference. Six circuits, each a wave digital filter with an R-type root (CircuitWdf.h), each driving the next
    one way, as an ideal source, or joined a sample apart through a resistor:

        Front       the trigger as the bass drum's: Q32 onto the TRIG line, the PNP switch Q46 (R183, R182 through
                    Q45); its loads, the two bridged-Ts' pulses (R189 with C57 over R190; R191 over R192) and the
                    SNAPPY pot VR9 (10 kOhm B, R199 below it); the snappy's envelope, Q47 through R186 charging C51
        Tail        C51 alone once the hit has passed and Q47 has let go: the front's devices rest while C51 gives up
                    its charge through R184
        IC14a       a bridged-T, C58 0.056 uF and C59 0.027 uF, R197 820 kOhm across, R196 680 Ohm to ground: the
                    lower shell, 173 Hz
        IC14b       a bridged-T, C60 0.0068 uF and C61 0.015 uF, R198 1 MOhm across, R195 2.2 kOhm to ground: the upper
                    shell, 334 Hz
        Snappy      R184 from C51 onto C52, R185 onto C53, D54 into the swing VCA Q48 (the white noise through C56 on its
                    base, R194 2.2 MOhm from the supply, R188 in its emitter, C54 on its collector); Q49's high-pass
                    (C66, C67, R201 back from its emitter, R202, R203 to the negative supply), C69 and R205 into IC13
        Mixer       VR8 (TONE, 100 kOhm B) between the shells (C63 from IC14a, C65 from IC14b), R204 into IC13; the
                    snappy through C69 and R205; R210 470 kOhm, C70 220 pF and the LEVEL pot VR7 (100 kOhm A) with C68
                    in IC13's feedback

    R184 joins C51 and the snappy a sample apart, each circuit holding the other's end as a source: 15 kOhm against
    C51's 0.47 uF and C52's 0.1 uF is hundreds of samples. Transistors and diodes as the bass drum's; the shells'
    op-amps of 3 MHz gain-bandwidth and a +-14 V swing, as the toms', the mixer's ideal. The bends (D256): Tune scales
    the four shell capacitors together, Decay each shell's damping (R197 and R198 up, R196 and R195 down by the same
    factor, so the pitch stays), Snappy Decay C51 and C52 together. Volts in, volts out.
*/
namespace coolsynth::synth::v3
{
    class Tr808Snare final
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
        static constexpr double c58 = 0.056e-6, c59 = 0.027e-6, c60 = 0.0068e-6, c61 = 0.015e-6; // as changed
        static constexpr double r197 = 820.0e3, r196 = 680.0, r198 = 1.0e6, r195 = 2.2e3;
        static constexpr double c51 = 0.47e-6, c52 = 0.1e-6;

        Tr808Snare()
        {
            front.setNewtonLimit(40); // the trigger's edges switch three transistors at once
            snappy.setNewtonLimit(40);
            solveTo808Tolerance(front, tail, lower, upper, snappy, mixer); // D356
            buildFront();
            buildTail();
            buildShell(lower, c58, c59, r197, r196, lowerParts);
            buildShell(upper, c60, c61, r198, r195, upperParts);
            buildSnappy();
            buildMixer();
            setTone(0.5);
            setSnappy(0.5);
        }

        /** At a sample rate, at rest (a copy of the solved rest state: cheap enough for the audio thread). */
        void prepare(double rate) noexcept
        {
            sampleRate = rate;
            front.setSampleRate(rate);
            tail.setSampleRate(rate);
            lower.setSampleRate(rate);
            upper.setSampleRate(rate);
            snappy.setSampleRate(rate);
            mixer.setSampleRate(rate);
            rest();
        }

        /** Back to rest: a copy of the solved rest state, no solving. */
        void rest() noexcept
        {
            const RestState& r = restState();
            front.setResistance(r182, switchOff);
            front.restore(r.front);
            tail.restore(r.tail);
            lower.restore(r.lower);
            upper.restore(r.upper);
            snappy.restore(r.snappy);
            mixer.restore(r.mixer);
            triggerSample = -1;
            frontResting = snappyResting = true;
            lastC51 = r.c51Volts;
            lastC52 = r.c52Volts;
            lastFollower = r.followerVolts;
            quietSamples = 0;
            setRestInputs();
        }

        /** The rest state, solved on first use and kept for the process: call it off the audio thread. */
        static void warmUp() noexcept { (void) restState(); }

        /** Back to the circuit at rest by solving it: capacitors charged as they are with no trigger. */
        void settle() noexcept
        {
            triggerSample = -1;
            front.setResistance(r182, switchOff);
            lastC51 = lastC52 = lastFollower = 0.0;
            setRestInputs();
            for (int pass = 0; pass < 3; ++pass)
            {
                front.setSource(r184Front, lastC52);
                front.settle();
                lastC51 = front.probe(frontC51);
                snappy.setSource(r184Snappy, lastC51);
                snappy.settle();
                lastC52 = snappy.probe(snappyC52);
                lastFollower = snappy.probe(snappyFollower);
            }
            tail.setSource(r184Tail, lastC52);
            tail.settle();
            lower.setInput(0, front.probe(frontLower));
            lower.settle();
            upper.setInput(0, front.probe(frontUpper));
            upper.settle();
            mixer.setInput(mixerLower, lower.probe(shellOut));
            mixer.setInput(mixerUpper, upper.probe(shellOut));
            mixer.setInput(mixerSnappy, lastFollower);
            mixer.settle();
            frontResting = snappyResting = true;
            quietSamples = 0;
        }

        /** A hit: the TRIG line at this many volts (4 unaccented to 14), for this long. */
        void trigger(double volts, double widthSeconds = 0.001) noexcept
        {
            if (frontResting)
            {
                // The front wakes from rest with C51 as the tail left it.
                front.restore(restState().front);
                front.adoptSharedState(tail);
                frontResting = false;
            }
            snappyResting = false;
            quietSamples = 0;
            triggerVolts = volts;
            triggerWidth = widthSeconds;
            triggerSample = 0;
            front.setResistance(r182, 10.0e3); // the instrument data stands: Q45 on
        }

        /** TONE 0..1: VR8's wiper from IC14a's end (pin 1) to IC14b's (pin 3). */
        void setTone(double turn) noexcept
        {
            const double t = std::clamp(turn, 0.0, 1.0);
            mixer.setResistance(vr8Upper, std::max(1.0, 100.0e3 * (1.0 - t)));
            mixer.setResistance(vr8Lower, std::max(1.0, 100.0e3 * t));
        }

        /** SNAPPY 0..1: VR9's wiper from R199's end to the trigger's. */
        void setSnappy(double turn) noexcept
        {
            const double t = std::clamp(turn, 0.0, 1.0);
            front.setResistance(vr9Top, std::max(1.0, 10.0e3 * (1.0 - t)));
            front.setResistance(vr9Bottom, std::max(1.0, 10.0e3 * t));
        }

        /** LEVEL 0..1: the 100 kOhm audio-taper pot in IC13's feedback. */
        void setLevel(double turn) noexcept { mixer.setResistance(vr7, Tr808Tom::levelOhms(turn)); }

        /** Tune: the shells' pitch times this (their four capacitors over it). */
        void setTuneScale(double scale) noexcept
        {
            const double k = 1.0 / std::clamp(scale, 0.1, 10.0);
            lower.setCapacitance(lowerParts.c1, c58 * k);
            lower.setCapacitance(lowerParts.c2, c59 * k);
            upper.setCapacitance(upperParts.c1, c60 * k);
            upper.setCapacitance(upperParts.c2, c61 * k);
        }

        /** Decay: each shell's ring times this, at the same pitch (its bridge resistor up, its ground leg down). */
        void setDecayScale(double scale) noexcept
        {
            const double k = std::clamp(scale, 0.1, 10.0);
            lower.setResistance(lowerParts.across, r197 * k);
            lower.setResistance(lowerParts.ground, r196 / k);
            upper.setResistance(upperParts.across, r198 * k);
            upper.setResistance(upperParts.ground, r195 / k);
        }

        /** Snappy Decay: the snappy's envelope times this (C51 and C52). */
        void setSnappyDecayScale(double scale) noexcept
        {
            const double k = std::clamp(scale, 0.1, 10.0);
            front.setCapacitance(c51Front, c51 * k);
            tail.setCapacitance(c51Tail, c51 * k);
            snappy.setCapacitance(c52Snappy, c52 * k);
        }

        /** One sample, the white noise bus at its volts; returns the output in volts. */
        double process(double whiteNoise) noexcept
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
                    front.setResistance(r182, switchOff);
                    triggerSample = -1;
                }
                else
                    ++triggerSample;
            }
            const bool active = triggerSample >= 0;
            if (! frontResting)
            {
                front.setInput(frontDrive, drive);
                front.setSource(r184Front, lastC52);
                front.process();
                lastC51 = front.probe(frontC51);
                // The front rests once the hit has passed, its nodes are back at rest and Q47 has let go: the tail
                // takes C51 over.
                const RestState& r = restState();
                if (! active && std::abs(front.probe(frontTrigger) - r.triggerVolts) < 1.0e-4
                    && std::abs(front.probe(frontLower) - r.lowerVolts) < 1.0e-9 && std::abs(front.probe(frontUpper) - r.upperVolts) < 1.0e-9
                    && front.probe(frontQ47Base) - lastC51 < 0.3)
                {
                    tail.adoptSharedState(front);
                    frontResting = true;
                }
            }
            else if (! snappyResting)
            {
                tail.setSource(r184Tail, lastC52);
                tail.process();
                lastC51 = tail.probe(0);
            }
            lower.setInput(0, frontResting ? restState().lowerVolts : front.probe(frontLower));
            lower.process();
            upper.setInput(0, frontResting ? restState().upperVolts : front.probe(frontUpper));
            upper.process();
            if (! snappyResting)
            {
                snappy.setInput(snappyNoise, whiteNoise);
                snappy.setSource(r184Snappy, lastC51);
                snappy.process();
                lastC52 = snappy.probe(snappyC52);
                lastFollower = snappy.probe(snappyFollower);
                // The snappy is quiet once its output has stayed within 0.1 mV of rest for 5 ms (with the mixer's gain
                // some -80 dB, the generator's own silence): it then stops being solved, its output held, so nothing
                // steps at the mixer; what charge C51 and C52 still hold opens D54 no further. rest() takes it back to
                // rest before a hit once the generator has let the instrument ring down.
                const RestState& r = restState();
                quietSamples = frontResting && std::abs(lastFollower - r.followerVolts) < 1.0e-4 ? quietSamples + 1 : 0;
                if (quietSamples > static_cast<int>(0.005 * sampleRate))
                    snappyResting = true;
            }
            mixer.setInput(mixerLower, lower.probe(shellOut));
            mixer.setInput(mixerUpper, upper.probe(shellOut));
            mixer.setInput(mixerSnappy, lastFollower);
            mixer.process();
            return mixer.probe(mixerOut);
        }

        // ---- Its circuit as it runs, for a panel that draws it (CoolSynth's D317) ---------------------------------
        // The node voltages and part currents its ngspice recording names (CoolSynth808's resources/lid), each from
        // the stage the engine models it in, signed as ngspice's: a part's current from its first node to its second,
        // a transistor's into its leads. C51 and its node are the front's while a hit runs and the tail's once the
        // front rests. R184, joined a sample apart, is the snappy's: it runs after the front, so it holds both of
        // R184's ends of the same sample (its port runs from C52, so turned round). C69, R205 and their node are the
        // mixer's, where R205 meets IC13's minus input (the snappy's copy ends on ground). R200, a link as changed, is
        // C65's current, its node IC14b's output. R187 is built from Q47's collector, so turned round. Each copy an
        // ideal input carries (the shells' plus inputs, their outputs, Q49's emitter, the supply) is read where it is
        // made. Not here, as the engine leaves them out: the common trigger's Q31, R117, R120 and their nodes n6,
        // nb31, nx (the engine drives Q32's base, Q31 gone), and the select line's R180, R181, Q45 and their nodes
        // nsd, nb45, nc45 (R182 switched to ground in their place). The white noise is the generator's, where the
        // recording has two sines.

        static constexpr int liveCount = 102;
        static constexpr double liveMs = 300.0; // the recording's length
        static constexpr std::array<const char*, liveCount> liveNames { "n199", "n200", "n52", "n55", "n69", "nb32", "nb46", "nb47", "nb48", "nb49", "nc47", "nc48", "ne47", "ne48", "ne49", "nm1", "nm2", "nm49", "nmid1", "nmid2", "no1", "no2", "np1", "np2", "nsn", "nsum", "nv1", "nv3", "nvr", "nw8", "nw9", "nwn", "trig", "vcc", "vee", "vout", "vt", "q32.ic", "q32.ib", "q32.ie", "r113.i", "r183.i", "r182.i", "q46.ic", "q46.ib", "q46.ie", "r189.i", "c57.i", "r190.i", "r191.i", "r192.i", "c58.i", "c59.i", "r197.i", "r196.i", "c60.i", "c61.i", "r198.i", "r195.i", "c63.i", "r200.i", "c65.i", "rv8a.i", "rv8b.i", "r204.i", "rv9a.i", "rv9b.i", "r199.i", "r186.i", "r193.i", "c55.i", "r187.i", "q47.ic", "q47.ib", "q47.ie", "c51.i", "r184.i", "c52.i", "r185.i", "c53.i", "c56.i", "r194.i", "q48.ic", "q48.ib", "q48.ie", "r188.i", "d54.id", "c54.i", "c66.i", "c67.i", "r201.i", "r202.i", "q49.ic", "q49.ib", "q49.ie", "r203.i", "c69.i", "r205.i", "r210.i", "c70.i", "rvr7.i", "c68.i" };

        /** The named values after the last sample, in liveNames' order, from the stages' own state (their taps). */
        void tap(float* out) const noexcept
        {
            // Kinds: 0 a node's voltage, 1 a port's current, 2 a transistor's emitter current from its base port (its
            // collector port the next), 3 a node of the front or, while the front rests, the tail's node alt, 4 a port
            // of the front or the tail's port alt. Sign -1 where the engine's port runs against the netlist's part.
            struct Entry { int stage, kind, index, alt, sign; };
            static constexpr std::array<Entry, liveCount> entries { {
              { 0, 0, 10, 0, 1 }, // n199
              { 5, 0, 3, 0, 1 }, // n200
              { 4, 0, 1, 0, 1 }, // n52
              { 0, 0, 13, 0, 1 }, // n55
              { 5, 0, 10, 0, 1 }, // n69
              { 0, 0, 2, 0, 1 }, // nb32
              { 0, 0, 5, 0, 1 }, // nb46
              { 0, 0, 11, 0, 1 }, // nb47
              { 4, 0, 4, 0, 1 }, // nb48
              { 4, 0, 9, 0, 1 }, // nb49
              { 0, 0, 14, 0, 1 }, // nc47
              { 4, 0, 3, 0, 1 }, // nc48
              { 0, 3, 12, 1, 1 }, // ne47
              { 4, 0, 5, 0, 1 }, // ne48
              { 4, 0, 10, 0, 1 }, // ne49
              { 2, 0, 2, 0, 1 }, // nm1
              { 3, 0, 2, 0, 1 }, // nm2
              { 4, 0, 8, 0, 1 }, // nm49
              { 2, 0, 3, 0, 1 }, // nmid1
              { 3, 0, 3, 0, 1 }, // nmid2
              { 2, 0, 4, 0, 1 }, // no1
              { 3, 0, 4, 0, 1 }, // no2
              { 0, 0, 7, 0, 1 }, // np1
              { 0, 0, 8, 0, 1 }, // np2
              { 4, 0, 2, 0, 1 }, // nsn
              { 5, 0, 6, 0, 1 }, // nsum
              { 5, 0, 2, 0, 1 }, // nv1
              { 5, 0, 4, 0, 1 }, // nv3
              { 5, 0, 8, 0, 1 }, // nvr
              { 5, 0, 5, 0, 1 }, // nw8
              { 0, 0, 9, 0, 1 }, // nw9
              { 4, 0, 6, 0, 1 }, // nwn
              { 0, 0, 4, 0, 1 }, // trig
              { 0, 0, 3, 0, 1 }, // vcc
              { 4, 0, 11, 0, 1 }, // vee
              { 5, 0, 7, 0, 1 }, // vout
              { 0, 0, 6, 0, 1 }, // vt
              { 0, 1, 4, 0, 1 }, // q32.ic
              { 0, 1, 3, 0, 1 }, // q32.ib
              { 0, 2, 3, 0, 1 }, // q32.ie
              { 0, 1, 5, 0, 1 }, // r113.i
              { 0, 1, 6, 0, 1 }, // r183.i
              { 0, 1, 7, 0, 1 }, // r182.i
              { 0, 1, 9, 0, 1 }, // q46.ic
              { 0, 1, 8, 0, 1 }, // q46.ib
              { 0, 2, 8, 0, 1 }, // q46.ie
              { 0, 1, 10, 0, 1 }, // r189.i
              { 0, 1, 11, 0, 1 }, // c57.i
              { 0, 1, 12, 0, 1 }, // r190.i
              { 0, 1, 13, 0, 1 }, // r191.i
              { 0, 1, 14, 0, 1 }, // r192.i
              { 2, 1, 0, 0, 1 }, // c58.i
              { 2, 1, 1, 0, 1 }, // c59.i
              { 2, 1, 2, 0, 1 }, // r197.i
              { 2, 1, 3, 0, 1 }, // r196.i
              { 3, 1, 0, 0, 1 }, // c60.i
              { 3, 1, 1, 0, 1 }, // c61.i
              { 3, 1, 2, 0, 1 }, // r198.i
              { 3, 1, 3, 0, 1 }, // r195.i
              { 5, 1, 0, 0, 1 }, // c63.i
              { 5, 1, 1, 0, 1 }, // r200.i
              { 5, 1, 1, 0, 1 }, // c65.i
              { 5, 1, 2, 0, 1 }, // rv8a.i
              { 5, 1, 3, 0, 1 }, // rv8b.i
              { 5, 1, 4, 0, 1 }, // r204.i
              { 0, 1, 15, 0, 1 }, // rv9a.i
              { 0, 1, 16, 0, 1 }, // rv9b.i
              { 0, 1, 17, 0, 1 }, // r199.i
              { 0, 1, 18, 0, 1 }, // r186.i
              { 0, 1, 23, 0, 1 }, // r193.i
              { 0, 1, 22, 0, 1 }, // c55.i
              { 0, 1, 21, 0, -1 }, // r187.i
              { 0, 1, 20, 0, 1 }, // q47.ic
              { 0, 1, 19, 0, 1 }, // q47.ib
              { 0, 2, 19, 0, 1 }, // q47.ie
              { 0, 4, 0, 0, 1 }, // c51.i
              { 4, 1, 0, 0, -1 }, // r184.i
              { 4, 1, 1, 0, 1 }, // c52.i
              { 4, 1, 2, 0, 1 }, // r185.i
              { 4, 1, 3, 0, 1 }, // c53.i
              { 4, 1, 10, 0, 1 }, // c56.i
              { 4, 1, 9, 0, 1 }, // r194.i
              { 4, 1, 7, 0, 1 }, // q48.ic
              { 4, 1, 6, 0, 1 }, // q48.ib
              { 4, 2, 6, 0, 1 }, // q48.ie
              { 4, 1, 8, 0, 1 }, // r188.i
              { 4, 1, 4, 0, 1 }, // d54.id
              { 4, 1, 5, 0, 1 }, // c54.i
              { 4, 1, 11, 0, 1 }, // c66.i
              { 4, 1, 12, 0, 1 }, // c67.i
              { 4, 1, 13, 0, 1 }, // r201.i
              { 4, 1, 14, 0, 1 }, // r202.i
              { 4, 1, 16, 0, 1 }, // q49.ic
              { 4, 1, 15, 0, 1 }, // q49.ib
              { 4, 2, 15, 0, 1 }, // q49.ie
              { 4, 1, 17, 0, 1 }, // r203.i
              { 5, 1, 5, 0, 1 }, // c69.i
              { 5, 1, 6, 0, 1 }, // r205.i
              { 5, 1, 7, 0, 1 }, // r210.i
              { 5, 1, 8, 0, 1 }, // c70.i
              { 5, 1, 9, 0, 1 }, // rvr7.i
              { 5, 1, 10, 0, 1 } // c68.i
            } };
            // each stage's node voltages, once (MaxNodes + 1 each: ground first)
            std::array<double, 15> frontV {};
            std::array<double, 2> tailV {};
            std::array<double, 6> lowerV {}, upperV {};
            std::array<double, 13> snappyV {};
            std::array<double, 11> mixerV {};
            front.nodeVoltages(frontV.data());
            tail.nodeVoltages(tailV.data());
            lower.nodeVoltages(lowerV.data());
            upper.nodeVoltages(upperV.data());
            snappy.nodeVoltages(snappyV.data());
            mixer.nodeVoltages(mixerV.data());
            for (int k = 0; k < liveCount; ++k)
            {
                const auto& e = entries[static_cast<std::size_t>(k)];
                const auto read = [&e](const auto& c, const double* v) -> double
                {
                    switch (e.kind)
                    {
                        case 0: return v[e.index];
                        case 2: return -(c.portCurrent(e.index, v) + c.portCurrent(e.index + 1, v));
                        default: return c.portCurrent(e.index, v);
                    }
                };
                double value = 0.0;
                if (e.kind == 3)
                    value = frontResting ? tailV[static_cast<std::size_t>(e.alt)] : frontV[static_cast<std::size_t>(e.index)];
                else if (e.kind == 4)
                    value = frontResting ? tail.portCurrent(e.alt, tailV.data()) : front.portCurrent(e.index, frontV.data());
                else
                {
                    switch (e.stage)
                    {
                        case 0: value = read(front, frontV.data()); break;
                        case 1: value = read(tail, tailV.data()); break;
                        case 2: value = read(lower, lowerV.data()); break;
                        case 3: value = read(upper, upperV.data()); break;
                        case 4: value = read(snappy, snappyV.data()); break;
                        default: value = read(mixer, mixerV.data()); break;
                    }
                }
                out[k] = static_cast<float>(e.sign * value);
            }
        }

        // The stages' last outputs, for tests against the reference.
        [[nodiscard]] double triggerNodeVolts() const noexcept { return frontResting ? restState().triggerVolts : front.probe(frontTrigger); }
        [[nodiscard]] double lowerPulseVolts() const noexcept { return frontResting ? restState().lowerVolts : front.probe(frontLower); }
        [[nodiscard]] double upperPulseVolts() const noexcept { return frontResting ? restState().upperVolts : front.probe(frontUpper); }
        [[nodiscard]] double lowerShellVolts() const noexcept { return lower.probe(shellOut); }
        [[nodiscard]] double upperShellVolts() const noexcept { return upper.probe(shellOut); }
        [[nodiscard]] double c51Volts() const noexcept { return lastC51; }
        [[nodiscard]] double c52Volts() const noexcept { return lastC52; }
        [[nodiscard]] double envelopeVolts() const noexcept { return snappy.probe(snappyEnvelope); }
        [[nodiscard]] double collectorVolts() const noexcept { return snappy.probe(snappyCollector); }
        [[nodiscard]] double followerVolts() const noexcept { return lastFollower; }
        [[nodiscard]] int newtonSteps() const noexcept
        {
            return std::max({ frontResting ? 0 : front.lastNewtonSteps(), snappyResting ? 0 : snappy.lastNewtonSteps(),
                              lower.lastNewtonSteps(), upper.lastNewtonSteps() });
        }
        [[nodiscard]] bool frontIsResting() const noexcept { return frontResting && snappyResting; }
        [[nodiscard]] bool lastSolveConverged() const noexcept
        {
            return (frontResting || front.lastSolveConverged()) && (snappyResting || snappy.lastSolveConverged());
        }
        [[nodiscard]] bool isWellPosed() noexcept
        {
            return front.isWellPosed() && tail.isWellPosed() && lower.isWellPosed() && upper.isWellPosed() && snappy.isWellPosed()
                   && mixer.isWellPosed();
        }

    private:
        struct ShellParts
        {
            int c1 = 0, c2 = 0, across = 0, ground = 0;
        };

        void setRestInputs() noexcept
        {
            front.setInput(frontDrive, 0.0);
            front.setInput(frontSupply, supply);
            front.setSource(r184Front, lastC52);
            tail.setSource(r184Tail, lastC52);
            lower.setInput(0, 0.0);
            upper.setInput(0, 0.0);
            snappy.setInput(snappyNoise, 0.0);
            snappy.setInput(snappySupply, supply);
            snappy.setInput(snappyNegative, -supply);
            snappy.setSource(r184Snappy, lastC51);
            mixer.setInput(mixerLower, 0.0);
            mixer.setInput(mixerUpper, 0.0);
            mixer.setInput(mixerSnappy, lastFollower);
        }

        // Front: 1 Q32's drive (input), 2 its base, 3 the supply (input), 4 the TRIG line, 5 Q46's base, 6 its
        // collector (the trigger), 7 IC14a's plus input, 8 IC14b's, 9 VR9's wiper, 10 VR9's foot over R199, 11 Q47's
        // base, 12 its emitter (C51), 13 C55, 14 Q47's collector. C51 and R184 come first, as the tail's.
        void buildFront()
        {
            c51Front = front.addCapacitor(12, 0, c51);                                  // C51
            r184Front = front.addSource(12, 0, 15.0e3);                                 // R184, to C52 in the snappy
            frontDrive = front.addInput(1);
            frontSupply = front.addInput(3);
            front.addResistor(1, 2, 4.7e3);                                             // R120
            front.addTransistor(3, 2, 4, transistorSaturation, betaF, betaR, thermal);  // Q32
            front.addResistor(4, 0, 4.7e3);                                             // R113
            front.addResistor(4, 5, 4.7e3);                                             // R183
            r182 = front.addResistor(5, 0, switchOff);                                  // R182 through Q45: open but while the data stands
            front.addPnpTransistor(6, 5, 4, transistorSaturation, betaF, betaR, thermal); // Q46
            front.addResistor(6, 7, 100.0e3);                                           // R189
            front.addCapacitor(6, 7, 6.8e-9);                                           // C57
            front.addResistor(7, 0, 680.0);                                             // R190
            front.addResistor(6, 8, 10.0e3);                                            // R191, as changed
            front.addResistor(8, 0, 220.0);                                             // R192
            vr9Top = front.addResistor(6, 9, 5.0e3);                                    // VR9, SNAPPY
            vr9Bottom = front.addResistor(9, 10, 5.0e3);
            front.addResistor(10, 0, 1.0e3);                                            // R199
            front.addResistor(9, 11, 33.0e3);                                           // R186
            front.addTransistor(14, 11, 12, transistorSaturation, betaF, betaR, thermal); // Q47
            front.addResistor(14, 13, 330.0);                                           // R187
            front.addCapacitor(13, 0, 47.0e-6);                                         // C55
            front.addResistor(3, 13, 100.0);                                            // R193
            frontTrigger = front.addProbe(6);
            frontLower = front.addProbe(7);
            frontUpper = front.addProbe(8);
            frontC51 = front.addProbe(12);
            frontQ47Base = front.addProbe(11);
        }

        // Tail: 1 C51.
        void buildTail()
        {
            c51Tail = tail.addCapacitor(1, 0, c51);       // C51
            r184Tail = tail.addSource(1, 0, 15.0e3);      // R184
            tail.addProbe(1);
        }

        // A shell: 1 the plus input (input), 2 the minus, 3 the middle node, 4 the output, 5 the op-amp's inner node.
        template <class Shell>
        static void buildShell(Shell& shell, double cIn, double cOut, double across, double ground, ShellParts& parts)
        {
            shell.addInput(1);
            parts.c1 = shell.addCapacitor(2, 3, cIn);
            parts.c2 = shell.addCapacitor(3, 4, cOut);
            parts.across = shell.addResistor(2, 4, across);
            parts.ground = shell.addResistor(3, 0, ground);
            shell.addOpAmp(1, 2, 4, 5, Tr808Tom::opAmpBandwidth, Tr808Tom::opAmpRail); // IC14a, IC14b
            shell.addProbe(4);
        }

        // Snappy: 1 C52, 2 C53 (D54's anode), 3 Q48's collector, 4 its base, 5 its emitter, 6 the white noise
        // (input), 7 the supply (input), 8 the high-pass's middle, 9 Q49's base, 10 its emitter, 11 the negative
        // supply (input), 12 C69 to R205.
        void buildSnappy()
        {
            snappyNoise = snappy.addInput(6);
            snappySupply = snappy.addInput(7);
            snappyNegative = snappy.addInput(11);
            r184Snappy = snappy.addSource(1, 0, 15.0e3);                                  // R184, from C51
            c52Snappy = snappy.addCapacitor(1, 0, c52);                                   // C52
            snappy.addResistor(1, 2, 15.0e3);                                             // R185
            snappy.addCapacitor(2, 0, 1.0e-9);                                            // C53
            snappy.addDiode(2, 3, diodeSaturation, diodeThermal);                         // D54
            snappy.addCapacitor(3, 0, 2.7e-9);                                            // C54
            snappy.addTransistor(3, 4, 5, transistorSaturation, betaF, betaR, thermal);   // Q48
            snappy.addResistor(5, 0, 100.0);                                              // R188
            snappy.addResistor(7, 4, 2.2e6);                                              // R194
            snappy.addCapacitor(6, 4, 22.0e-9);                                           // C56
            snappy.addCapacitor(2, 8, 1.8e-9);                                            // C66
            snappy.addCapacitor(8, 9, 1.8e-9);                                            // C67
            snappy.addResistor(8, 10, 22.0e3);                                            // R201
            snappy.addResistor(9, 0, 47.0e3);                                             // R202
            snappy.addTransistor(7, 9, 10, transistorSaturation, betaF, betaR, thermal);  // Q49
            snappy.addResistor(10, 11, 22.0e3);                                           // R203
            snappy.addCapacitor(10, 12, 10.0e-9);                                         // C69
            snappy.addResistor(12, 0, 27.0e3);                                            // R205, to IC13's virtual ground
            snappyC52 = snappy.addProbe(1);
            snappyEnvelope = snappy.addProbe(2);
            snappyCollector = snappy.addProbe(3);
            snappyFollower = snappy.addProbe(10);
        }

        // Mixer: 1 IC14a (input), 2 VR8's pin 1, 3 IC14b (input), 4 VR8's pin 3, 5 VR8's wiper, 6 IC13's minus, 7 its
        // output, 8 the LEVEL pot to C68, 9 Q49's emitter (input), 10 C69 to R205.
        void buildMixer()
        {
            mixerLower = mixer.addInput(1);
            mixerUpper = mixer.addInput(3);
            mixerSnappy = mixer.addInput(9);
            mixer.addCapacitor(1, 2, 0.047e-6);                    // C63
            mixer.addCapacitor(3, 4, 0.047e-6);                    // C65 (R200 a link, as changed)
            vr8Upper = mixer.addResistor(4, 5, 50.0e3);            // VR8, TONE
            vr8Lower = mixer.addResistor(5, 2, 50.0e3);
            mixer.addResistor(5, 6, 100.0e3);                      // R204
            mixer.addCapacitor(9, 10, 10.0e-9);                    // C69
            mixer.addResistor(10, 6, 27.0e3);                      // R205
            mixer.addResistor(6, 7, 470.0e3);                      // R210
            mixer.addCapacitor(6, 7, 220.0e-12);                   // C70
            vr7 = mixer.addResistor(6, 8, Tr808Tom::levelOhms(0.8)); // VR7, LEVEL
            mixer.addCapacitor(8, 7, 33.0e-6);                     // C68
            mixer.addNullor(0, 6, 7);                              // IC13
            mixerOut = mixer.addProbe(7);
        }

        wdf::Circuit<14, 24, 2, 0, 6, 5, true> front; // Tapped: a panel reads its nodes and currents (D317)
        wdf::Circuit<1, 2, 1, 0, 0, 1, true> tail;    // room for an input it does not use: MSVC finds code unreachable with none
        wdf::Circuit<5, 7, 1, 1, 1, 1, true> lower, upper;
        wdf::Circuit<12, 20, 3, 0, 5, 4, true> snappy;
        wdf::Circuit<10, 11, 3, 1, 0, 1, true> mixer;
        ShellParts lowerParts, upperParts;
        static constexpr int shellOut = 0;

        struct RestState
        {
            decltype(front)::Snapshot front {};
            decltype(tail)::Snapshot tail {};
            decltype(lower)::Snapshot lower {}, upper {};
            decltype(snappy)::Snapshot snappy {};
            decltype(mixer)::Snapshot mixer {};
            double c51Volts = 0.0, c52Volts = 0.0, followerVolts = 0.0, envelopeVolts = 0.0;
            double triggerVolts = 0.0, lowerVolts = 0.0, upperVolts = 0.0;
        };

        static const RestState& restState() noexcept
        {
            static const RestState state = []
            {
                auto drum = std::make_unique<Tr808Snare>();
                drum->settle();
                RestState s;
                s.front = drum->front.snapshot();
                s.tail = drum->tail.snapshot();
                s.lower = drum->lower.snapshot();
                s.upper = drum->upper.snapshot();
                s.snappy = drum->snappy.snapshot();
                s.mixer = drum->mixer.snapshot();
                s.c51Volts = drum->lastC51;
                s.c52Volts = drum->lastC52;
                s.followerVolts = drum->lastFollower;
                s.envelopeVolts = drum->snappy.probe(drum->snappyEnvelope);
                s.triggerVolts = drum->front.probe(drum->frontTrigger);
                s.lowerVolts = drum->front.probe(drum->frontLower);
                s.upperVolts = drum->front.probe(drum->frontUpper);
                return s;
            }();
            return state;
        }

        double sampleRate = 96000.0;
        double triggerVolts = 4.0, triggerWidth = 0.001;
        int triggerSample = -1; // samples since the hit, -1 between hits
        bool frontResting = true, snappyResting = true;
        double lastC51 = 0.0, lastC52 = 0.0, lastFollower = 0.0;
        int quietSamples = 0;

        int frontDrive = 0, frontSupply = 0, frontTrigger = 0, frontLower = 0, frontUpper = 0, frontC51 = 0, frontQ47Base = 0;
        int c51Front = 0, r184Front = 0, r182 = 0, vr9Top = 0, vr9Bottom = 0;
        int c51Tail = 0, r184Tail = 0;
        int snappyNoise = 0, snappySupply = 0, snappyNegative = 0, r184Snappy = 0, c52Snappy = 0;
        int snappyC52 = 0, snappyEnvelope = 0, snappyCollector = 0, snappyFollower = 0;
        int mixerLower = 0, mixerUpper = 0, mixerSnappy = 0, mixerOut = 0, vr8Upper = 0, vr8Lower = 0, vr7 = 0;
    };
}
