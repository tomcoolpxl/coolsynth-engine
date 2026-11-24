#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include "CircuitWdf.h"
#include "Tr808BassDrum.h"
#include "Tr808MetalBank.h"

/*
    The Roland TR-808's cymbal (D254), from the service notes (pp. 6, 13), Werner, Abel and Smith's analysis
    (ICMC-SMC 2014) and tools/tr808/metal.cir, its reference. Six circuits, each a wave digital filter (CircuitWdf.h):

        Envelopes   IC2a's narrowed trigger through the switch Q6 and R84 into Q19's base (R83 its load; its collector
                    fed through R85 and R86 with C35); its emitter charges three envelopes through D6, D7 and D8: C38
                    (with R87 and C37) for the top band through R94; C40, R88 and C39 for the middle band through R90;
                    C41, which drains through R93 and CY DECAY (VR2, 2 MOhm B), followed by Q20 into the low band
                    (R105, C45, R104) and into the middle band again (R92, R89, R91)
        Top band    IC3's high band through C42 onto Q16's base (2 MOhm its bias), a swing VCA pulling R94's end down
                    through D5 (C36, C47); IC5a's high-pass (C49, R106, then a Sallen-Key with a gain of 2.77)
        Middle      the high band again, Q17 (D11, C43, C50) under R90 and R91; IC5b's Sallen-Key high-pass (gain 2)
        Low band    IC3's low band, Q18 (D12) under R104; Q25's Sallen-Key around an emitter follower
        Tone, level C55, R112 and R119 (the top band), C56, R120 and C58, R123, C57, R121 (the others) to R129; CY TONE
                    (VR4, 20 kOhm B, its wiper grounded) between R112's end and R125; C90 into IC6b, CY LEVEL (VR6,
                    100 kOhm A) with C78 in its feedback

    The resistors between the envelopes and the VCAs (R94, R90, R91, R104) join them a sample apart, each circuit
    holding the other's end as a source; the envelopes' capacitors are microfarads behind tens of kilohms. Once a hit
    has passed (Q6, Q19 and the three diodes off), the envelopes run without the trigger's parts, the state passed
    across, so the tail costs only Q20. The VCAs run while the bank does: their coupling capacitors follow the bands.
    The high-passes' capacitors are prewarped at their peaks (10.4 and 12.5 kHz), which the bilinear transform would
    otherwise move 4 and 6 % down at 96 kHz. Transistors and diodes as the bass drum's; IC5's op-amps of 3 MHz, their
    +-14 V swing left out (at full accent they reach 6.5 V); IC6b and Q25 ideal. Volts in, volts out.
*/
namespace coolsynth::synth::v3
{
    class Tr808Cymbal final
    {
    public:
        static constexpr double thermal = Tr808BassDrum::thermal;
        static constexpr double transistorSaturation = Tr808BassDrum::transistorSaturation;
        static constexpr double betaF = Tr808BassDrum::betaF, betaR = Tr808BassDrum::betaR;
        static constexpr double diodeSaturation = Tr808BassDrum::diodeSaturation;
        static constexpr double diodeThermal = thermal * Tr808BassDrum::diodeEmission;
        static constexpr double supply = 15.0;
        static constexpr double instrumentHold = Tr808BassDrum::instrumentHold;
        static constexpr double switchOff = Tr808BassDrum::switchOff;
        /** Junction capacitances as the reference has them (CJE 8 pF, CJC 4 pF), where devices stand in series with
            nothing else between them (Q19's emitter feeds only the three diodes; Q18's collector only D12): without
            them their currents are tied and the root cannot be solved. */
        static constexpr double junctionEmitter = 8.0e-12, junctionCollector = 4.0e-12;
        static constexpr double topPeak = 10427.0, middlePeak = 12508.0;
        /** The VCAs' coupling capacitors (C42, C44 on the high band, C46 on the low) with the bank running and the
            cymbal at rest, the band's side less the base's: their mean (measured on the model, V3Tr808MetalTests with
            COOLSYNTH_808_COUPLING; ngspice's are -0.524 and -0.515 V). */
        static constexpr double highCouplingRest = -0.526, lowCouplingRest = -0.514;
        /** After a hit's pulse and switch, this long before the envelopes drop the trigger's parts: 0.25 ms matches
            the full circuit run throughout as 2 ms did, within -94 dB of it (D262). */
        static constexpr double triggerSettle = 0.00025;

        Tr808Cymbal()
        {
            full.setNewtonLimit(40);
            tail.setNewtonLimit(40);
            buildEnvelope(tail, false);
            buildEnvelope(full, true);
            buildTop();
            buildMiddle();
            buildLow();
            buildMix();
        }

        void prepare(double rate) noexcept
        {
            sampleRate = rate;
            full.setSampleRate(rate);
            tail.setSampleRate(rate);
            top.setSampleRate(rate);
            middle.setSampleRate(rate);
            low.setSampleRate(rate);
            mix.setSampleRate(rate);
            const double t = Tr808MetalBank::prewarp(topPeak, rate), m = Tr808MetalBank::prewarp(middlePeak, rate);
            top.setCapacitance(c49, 3.3e-9 * t);
            top.setCapacitance(c53, 1.0e-9 * t);
            top.setCapacitance(c54, 1.0e-9 * t);
            middle.setCapacitance(c51, 1.0e-9 * m);
            middle.setCapacitance(c52, 1.0e-9 * m);
            rest();
        }

        /** Back to rest with the bank running: a copy of the solved rest state. */
        void rest() noexcept
        {
            restEnvelope();
            restVcas();
        }

        /** The envelopes and the mix at rest (the VCAs, which run with the bank, as they are). */
        void restEnvelope() noexcept
        {
            const RestState& r = restState();
            full.setResistance(r10, switchOff);
            full.restore(r.full);
            tail.restore(r.tail);
            mix.restore(r.mix);
            usingFull = false;
            triggerSample = -1;
            sinceHit = 1 << 30;
            ranFull = false;
            tailSamples = 0;
            c35AtHandover = full.portVoltage(c35);
            setRestInputs();
            readEnvelope(tail);
        }

        /** The VCAs at rest, their coupling capacitors where the running bank holds them. */
        void restVcas() noexcept
        {
            const RestState& r = restState();
            top.restore(r.top);
            middle.restore(r.middle);
            low.restore(r.low);
            h1 = h2 = u = 0.0;
        }

        static void warmUp() noexcept { (void) restState(); }

        /** Back to the circuit at rest by solving it, the bands at these volts (as the reference's time 0). */
        void settle(double highBand, double lowBand) noexcept
        {
            full.setResistance(r10, switchOff);
            usingFull = false;
            triggerSample = -1;
            sinceHit = 1 << 30;
            setRestInputs();
            top.setInput(topBand, highBand);
            middle.setInput(middleBand, highBand);
            low.setInput(lowBandIn, lowBand);
            h1 = h2 = u = 0.0;
            for (int pass = 0; pass < 3; ++pass)
            {
                setEnvelopeSources(tail);
                tail.settle();
                readEnvelope(tail);
                setVcaSources();
                top.settle();
                middle.settle();
                low.settle();
                readVcas();
            }
            setEnvelopeSources(full);
            full.settle();
            ranFull = false;
            tailSamples = 0;
            c35AtHandover = full.portVoltage(c35);
            mix.setInput(mixTop, top.probe(topOut));
            mix.setInput(mixMiddle, middle.probe(middleOut));
            mix.setInput(mixLow, low.probe(lowOut));
            mix.settle();
        }

        /** A hit: IC2a's output at this many volts for this long, through the switch. */
        void trigger(double bufferVolts, double widthSeconds = 0.001) noexcept
        {
            if (! usingFull)
            {
                // The trigger's parts at rest, the rest as the tail holds it.
                full.restore(restState().full);
                full.adoptSharedState(tail);
                usingFull = true;
            }
            pulseVolts = bufferVolts;
            pulseWidth = widthSeconds;
            triggerSample = 0;
            sinceHit = 0;
            full.setResistance(r10, 22.0e3); // the data stands: Q2 on, R10 to ground
        }

        /** CY DECAY 0..1: VR2, 2 MOhm linear, as a rheostat. */
        void setDecay(double turn) noexcept
        {
            const double ohms = std::max(0.01, 2.0e6 * std::clamp(turn, 0.0, 1.0));
            full.setResistance(vr2, ohms);
            tail.setResistance(vr2, ohms);
        }

        /** CY TONE 0..1: VR4, 20 kOhm linear, its wiper grounded; 1 the brightest. */
        void setTone(double turn) noexcept
        {
            const double t = std::clamp(turn, 0.0, 1.0);
            mix.setResistance(vr4Top, std::max(0.01, 20.0e3 * t));
            mix.setResistance(vr4Rest, std::max(0.01, 20.0e3 * (1.0 - t)));
        }

        /** CY LEVEL 0..1: the 100 kOhm audio-taper pot in IC6b's feedback. */
        void setLevel(double turn) noexcept
        {
            const double t = std::clamp(turn, 0.0, 1.0);
            mix.setResistance(vr6, std::max(1.0, 100.0e3 * (std::pow(10.0, 2.0 * t) - 1.0) / 99.0));
        }

        /** The VCAs, one sample, with IC3's bands at these volts (while the bank runs). */
        void processVcas(double highBand, double lowBand) noexcept
        {
            top.setInput(topBand, highBand);
            middle.setInput(middleBand, highBand);
            low.setInput(lowBandIn, lowBand);
            setVcaSources();
            top.process();
            middle.process();
            low.process();
            readVcas();
        }

        /** How long the full circuit runs after the pulse and switch, for the settle check (tests only, D262). */
        void setTriggerSettleForTesting(double seconds) noexcept { settleSeconds = seconds; }

        /** The envelopes, one sample (before processVcas), while the cymbal sounds. */
        void processEnvelope() noexcept
        {
            double drive = 0.0;
            if (triggerSample >= 0)
            {
                const int width = std::max(1, static_cast<int>(std::lround(pulseWidth * sampleRate)));
                if (triggerSample >= 1 && triggerSample <= width)
                    drive = pulseVolts;
                if (triggerSample >= width + static_cast<int>(std::lround(instrumentHold * sampleRate)))
                {
                    full.setResistance(r10, switchOff);
                    triggerSample = -1;
                }
                else
                    ++triggerSample;
            }
            ++sinceHit;
            if (usingFull)
            {
                full.setInput(fullDrive, drive);
                setEnvelopeSources(full);
                full.process();
                readEnvelope(full);
                ranFull = true; // the tap reads this sample from the full circuit, the handover's too
                const int pulse = static_cast<int>(std::lround((pulseWidth + instrumentHold + settleSeconds) * sampleRate));
                if (triggerSample < 0 && sinceHit > pulse)
                {
                    tail.adoptSharedState(full); // the hit has passed: the trigger's parts are off
                    usingFull = false;
                    c35AtHandover = full.portVoltage(c35); // for the tap: C35 recovers from here (D317)
                    tailSamples = 0;
                }
            }
            else
            {
                setEnvelopeSources(tail);
                tail.process();
                readEnvelope(tail);
                ranFull = false;
                if (tailSamples < (1 << 30))
                    ++tailSamples;
            }
        }

        /** The tone and level stage, one sample (after processVcas); returns the output in volts. */
        double processMix() noexcept
        {
            mix.setInput(mixTop, top.probe(topOut));
            mix.setInput(mixMiddle, middle.probe(middleOut));
            mix.setInput(mixLow, low.probe(lowOut));
            mix.process();
            return mix.probe(mixOut);
        }

        // The stages' last outputs, for tests against the reference.
        [[nodiscard]] double envelopeVolts(int which) const noexcept
        {
            switch (which)
            {
                case 0: return envJ;   // C38
                case 1: return envM;   // C39
                case 2: return envV;   // C41
                case 3: return envT;   // C45
                case 4: return envN;   // R89
                default: return envK;  // Q19's emitter
            }
        }
        [[nodiscard]] double gateVolts(int which) const noexcept { return which == 0 ? h1 : which == 1 ? h2 : u; }
        [[nodiscard]] double bandOut(int which) const noexcept
        {
            return which == 0 ? top.probe(topOut) : which == 1 ? middle.probe(middleOut) : low.probe(lowOut);
        }
        [[nodiscard]] double couplingVolts(int which) const noexcept
        {
            return which == 0 ? top.portVoltage(c42) : which == 1 ? middle.portVoltage(c44) : low.portVoltage(c46);
        }
        [[nodiscard]] bool usingTriggerParts() const noexcept { return usingFull; }
        /** The VCAs' last Newton steps (0 the top band, 1 the middle, 2 the low), for the cost tests. */
        [[nodiscard]] int vcaNewtonSteps(int which) const noexcept
        {
            return which == 0 ? top.lastNewtonSteps() : which == 1 ? middle.lastNewtonSteps() : low.lastNewtonSteps();
        }
        [[nodiscard]] bool lastSolveConverged() const noexcept
        {
            return (usingFull ? full.lastSolveConverged() : tail.lastSolveConverged()) && top.lastSolveConverged()
                   && middle.lastSolveConverged() && low.lastSolveConverged();
        }

        /** Every circuit solvable (CircuitWdf::isWellPosed). */
        [[nodiscard]] bool wellPosed() noexcept
        {
            return full.isWellPosed() && tail.isWellPosed() && top.isWellPosed() && middle.isWellPosed() && low.isWellPosed()
                   && mix.isWellPosed();
        }

        // ---- Its circuit as it runs, for a panel that draws it (CoolSynth's D317) ---------------------------------
        // The node voltages and part currents its ngspice recording names (CoolSynth808's resources/lid), each from
        // the stage the engine models it in; where a resistor joins two circuits a sample apart (R94, R90, R91, R104),
        // from the VCA's side, whose source holds the envelope's node of this very sample. Signs as ngspice's: a part's
        // current from its first node to its second, a transistor's into its leads, Q19's and Q18's with the junction
        // capacitors the engine gives them (ngspice's take in their junctions). The envelopes from whichever circuit ran
        // last; once a hit has passed and the tail runs (D262), the trigger's parts as they then stand: Q6, Q19 and the
        // three diodes off, their nodes and currents 0, but for C35 (and Q19's collector with it), which recovers through
        // R86 from where the handover left it, and Q19's emitter, which D7 and D8 leak into and D6 passes on, so it sits
        // n VT ln 3 above C38. Q25 is an ideal follower and R126 (to the -15 V supply) is left out: its current is worked
        // out from the emitter's volts (which lack Q25's Vbe, so it reads some 6 % high), Q25's emitter current from all
        // its emitter feeds, its base current from betaF. Not here, as the engine leaves them out: the data line J6 and
        // the switch Q2 with R3, R6 and its collector (R10 through Q2 is one part).
        //
        // Kinds: 0 a node, 1 a port's current, 2 a transistor's emitter from its base port, 3 Q19's base with its
        // emitter junction, 4 its emitter, 5 Q18's base less its collector junction, 6 its collector with it, 7 the
        // middle of a 2 MOhm bias (two 1 MOhm in the circuit), 8 a constant (the index its volts), 9 R126, 10 Q25's
        // emitter, 11 its base, 12 its collector, 13 C35's node (or Q19's collector), 14 C35's current (or R86's), 15 Q19's
        // emitter. Stages: 0 the envelopes (full or tail), 1 the top band, 2 the middle, 3 the low, 4 the mix.

        static constexpr int liveCount = 164;
        static constexpr std::array<const char*, liveCount> liveNames { "n100", "n102", "n125", "n35", "n37", "n51", "n55", "n56", "n58", "n5am", "n5ap", "n5bm", "n5bp", "n6bm", "n6bp", "n78", "n93", "n96", "na1", "nb16", "nb17", "nb18", "nb19", "nb1n", "nb25", "nb6", "nc16", "nc17", "nc18", "nc19", "ncy6", "ne16", "ne17", "ne18", "nh1", "nh2", "nhb", "nhp3", "nhpa", "nhpb", "nj", "nk", "nl", "nlb", "nm", "nmx", "nn", "np5", "ns", "nt", "nu", "nv", "nv2", "nw", "nx1", "nx3", "vcc", "vcy", "vee", "r10.i", "r15.i", "q6.ic", "q6.ib", "q6.ie", "r83.i", "r84.i", "q19.ic", "q19.ib", "q19.ie", "r85.i", "c35.i", "r86.i", "d6.id", "d7.id", "d8.id", "c38.i", "r87.i", "c37.i", "r94.i", "c40.i", "r88.i", "c39.i", "r90.i", "c41.i", "r93.i", "rvr2.i", "q20.ic", "q20.ib", "q20.ie", "r105.i", "c45.i", "r104.i", "r92.i", "r89.i", "r91.i", "c42.i", "r96.i", "r97.i", "q16.ic", "q16.ib", "q16.ie", "r95.i", "c36.i", "d5.id", "c47.i", "c44.i", "r100.i", "r99.i", "q17.ic", "q17.ib", "q17.ie", "r98.i", "c43.i", "d11.id", "c50.i", "c46.i", "r102.i", "r103.i", "q18.ic", "q18.ib", "q18.ie", "r101.i", "d12.id", "c49.i", "r106.i", "c53.i", "c54.i", "r109.i", "r108.i", "r111.i", "r110.i", "c51.i", "c52.i", "r113.i", "r107.i", "r115.i", "r114.i", "c48.i", "r124.i", "c59.i", "r127.i", "q25.ic", "q25.ib", "q25.ie", "r126.i", "c58.i", "r123.i", "c57.i", "r121.i", "c55.i", "r112.i", "r119.i", "rvr4a.i", "rvr4b.i", "r125.i", "c56.i", "r120.i", "r129.i", "c90.i", "r118.i", "r128.i", "c77.i", "c78.i", "rvr6.i" };
        /** How long a hit's capture runs, as the recording: milliseconds. */
        static constexpr double liveMs = 600.0;

        /** The named values after the last sample, in liveNames' order, from the stages' own state (their taps). */
        void tap(float* out) const noexcept
        {
            struct Entry { int stage, kind, index, sign; };
            static constexpr std::array<Entry, liveCount> entries { {
              { 2, 7, 2, 1 }, { 3, 7, 2, 1 }, { 4, 0, 6, 1 }, { 0, 13, 16, 1 }, { 0, 0, 3, 1 }, { 2, 0, 7, 1 }, { 4, 0, 4, 1 }, { 4, 0, 8, 1 },
              { 4, 0, 9, 1 }, { 1, 0, 10, 1 }, { 1, 0, 9, 1 }, { 2, 0, 9, 1 }, { 2, 0, 8, 1 }, { 4, 0, 12, 1 }, { -1, 8, 0, 1 }, { 4, 0, 14, 1 },
              { 0, 0, 7, 1 }, { 1, 7, 2, 1 }, { 4, 0, 5, 1 }, { 1, 0, 3, 1 }, { 2, 0, 3, 1 }, { 3, 0, 3, 1 }, { 0, 0, 14, 1 }, { 4, 0, 7, 1 },
              { 3, 0, 8, 1 }, { 0, 0, 12, 1 }, { 1, 0, 4, 1 }, { 2, 0, 4, 1 }, { 3, 0, 4, 1 }, { 0, 13, 15, 1 }, { 0, 0, 13, 1 }, { 1, 0, 5, 1 },
              { 2, 0, 5, 1 }, { 3, 0, 5, 1 }, { 1, 0, 6, 1 }, { 2, 0, 6, 1 }, { 1, 0, 1, 1 }, { 3, 0, 9, 1 }, { 1, 0, 11, 1 }, { 2, 0, 10, 1 },
              { 0, 0, 2, 1 }, { 0, 15, 17, 1 }, { 0, 0, 4, 1 }, { 3, 0, 1, 1 }, { 0, 0, 5, 1 }, { 4, 0, 11, 1 }, { 0, 0, 10, 1 }, { 1, 0, 8, 1 },
              { 0, 0, 8, 1 }, { 0, 0, 9, 1 }, { 3, 0, 6, 1 }, { 0, 0, 6, 1 }, { 0, 0, 11, 1 }, { 3, 0, 7, 1 }, { 1, 0, 7, 1 }, { 4, 0, 10, 1 },
              { 1, 0, 2, 1 }, { 4, 0, 13, 1 }, { -1, 8, -15, 1 }, { 0, 1, 20, 1 }, { 0, 1, 19, 1 }, { 0, 1, 22, 1 }, { 0, 1, 21, 1 }, { 0, 2, 21, 1 },
              { 0, 1, 23, 1 }, { 0, 1, 24, 1 }, { 0, 1, 26, 1 }, { 0, 3, 25, 1 }, { 0, 4, 25, 1 }, { 0, 1, 28, 1 }, { 0, 14, 29, 1 }, { 0, 14, 30, 1 },
              { 0, 1, 31, 1 }, { 0, 1, 32, 1 }, { 0, 1, 33, 1 }, { 0, 1, 0, 1 }, { 0, 1, 1, 1 }, { 0, 1, 2, 1 }, { 1, 1, 8, -1 }, { 0, 1, 4, 1 },
              { 0, 1, 5, 1 }, { 0, 1, 6, 1 }, { 2, 1, 8, -1 }, { 0, 1, 8, 1 }, { 0, 1, 9, 1 }, { 0, 1, 10, 1 }, { 0, 1, 12, 1 }, { 0, 1, 11, 1 },
              { 0, 2, 11, 1 }, { 0, 1, 13, 1 }, { 0, 1, 14, 1 }, { 3, 1, 7, -1 }, { 0, 1, 16, 1 }, { 0, 1, 17, 1 }, { 2, 1, 9, 1 }, { 1, 1, 0, 1 },
              { 1, 1, 1, 1 }, { 1, 1, 1, 1 }, { 1, 1, 3, 1 }, { 1, 1, 2, 1 }, { 1, 2, 2, 1 }, { 1, 1, 4, 1 }, { 1, 1, 6, 1 }, { 1, 1, 5, 1 },
              { 1, 1, 7, 1 }, { 2, 1, 0, 1 }, { 2, 1, 1, 1 }, { 2, 1, 1, 1 }, { 2, 1, 3, 1 }, { 2, 1, 2, 1 }, { 2, 2, 2, 1 }, { 2, 1, 4, 1 },
              { 2, 1, 6, 1 }, { 2, 1, 5, 1 }, { 2, 1, 7, 1 }, { 3, 1, 0, 1 }, { 3, 1, 1, 1 }, { 3, 1, 1, 1 }, { 3, 6, 3, 1 }, { 3, 5, 2, 1 },
              { 3, 2, 2, 1 }, { 3, 1, 4, 1 }, { 3, 1, 5, 1 }, { 1, 1, 9, 1 }, { 1, 1, 10, 1 }, { 1, 1, 11, 1 }, { 1, 1, 12, 1 }, { 1, 1, 13, 1 },
              { 1, 1, 14, 1 }, { 1, 1, 15, 1 }, { 1, 1, 16, 1 }, { 2, 1, 10, 1 }, { 2, 1, 11, 1 }, { 2, 1, 12, 1 }, { 2, 1, 13, 1 }, { 2, 1, 14, 1 },
              { 2, 1, 15, 1 }, { 3, 1, 8, 1 }, { 3, 1, 9, 1 }, { 3, 1, 10, 1 }, { 3, 1, 11, 1 }, { 3, 12, 9, 1 }, { 3, 11, 9, 1 }, { 3, 10, 9, 1 },
              { 3, 9, 9, 1 }, { 4, 1, 8, 1 }, { 4, 1, 9, 1 }, { 4, 1, 10, 1 }, { 4, 1, 11, 1 }, { 4, 1, 0, 1 }, { 4, 1, 1, 1 }, { 4, 1, 2, 1 },
              { 4, 1, 3, 1 }, { 4, 1, 4, 1 }, { 4, 1, 5, 1 }, { 4, 1, 6, 1 }, { 4, 1, 7, 1 }, { 4, 1, 12, 1 }, { 4, 1, 13, 1 }, { -1, 8, 0, 1 },
              { 4, 1, 14, 1 }, { 4, 1, 15, 1 }, { 4, 1, 16, 1 }, { 4, 1, 17, 1 } } };
            static constexpr int tailNodes = 10, tailPorts = 19; // the tail's: the full circuit's first
            static constexpr double r86Ohms = 100.0, c35Seconds = 100.0 * 47.0e-6; // R86 and C35
            static constexpr double r126Ohms = 22.0e3;
            static constexpr double ln3 = 1.0986122886681098;
            // each stage's node voltages, once
            std::array<double, 18> envelopeV {};
            std::array<double, 13> topV {};
            std::array<double, 12> middleV {};
            std::array<double, 10> lowV {};
            std::array<double, 15> mixV {};
            if (ranFull)
                full.nodeVoltages(envelopeV.data());
            else
                tail.nodeVoltages(envelopeV.data());
            top.nodeVoltages(topV.data());
            middle.nodeVoltages(middleV.data());
            low.nodeVoltages(lowV.data());
            mix.nodeVoltages(mixV.data());
            // C35 once the hit has passed: Q19 off, it recovers through R86 from where the handover left it.
            const double c35Volts = supply - (supply - c35AtHandover) * std::exp(-static_cast<double>(tailSamples) / (sampleRate * c35Seconds));
            // Q25's emitter: R126 to -15 V, C58 on into the mix, less what R124 brings in.
            const double r126Amps = (lowV[9] + supply) / r126Ohms;
            const double q25Emitter = -(r126Amps + mix.portCurrent(8, mixV.data()) - low.portCurrent(9, lowV.data()));
            const double q25Base = -q25Emitter / (betaF + 1.0);

            const auto read = [](const auto& c, const double* v, const Entry& e) -> double
            {
                switch (e.kind)
                {
                    case 0: return v[e.index];
                    case 1: return c.portCurrent(e.index, v);
                    case 2: return -(c.portCurrent(e.index, v) + c.portCurrent(e.index + 1, v));
                    case 7: return 0.5 * (v[e.index] + v[e.index + 1]);
                    default: return 0.0;
                }
            };
            const auto envelope = [&](const Entry& e) -> double
            {
                const double* v = envelopeV.data();
                if (ranFull)
                {
                    switch (e.kind)
                    {
                        case 3: return full.portCurrent(e.index, v) + full.portCurrent(q19Junction, v);
                        case 4: return -(full.portCurrent(e.index, v) + full.portCurrent(e.index + 1, v) + full.portCurrent(q19Junction, v));
                        case 13: case 15: return v[e.index];
                        case 14: return full.portCurrent(e.index, v);
                        default: return read(full, v, e);
                    }
                }
                switch (e.kind)
                {
                    case 13: return c35Volts;
                    case 14: return (supply - c35Volts) / r86Ohms; // R86's, all into C35
                    case 15: return v[2] + diodeThermal * ln3;     // above C38
                    case 0: return e.index <= tailNodes ? v[e.index] : 0.0;
                    default: return e.index < tailPorts ? read(tail, v, e) : 0.0; // the trigger's parts (kinds 3, 4 too): off
                }
            };
            for (int k = 0; k < liveCount; ++k)
            {
                const auto& e = entries[static_cast<std::size_t>(k)];
                double value = 0.0;
                switch (e.stage)
                {
                    case 0: value = envelope(e); break;
                    case 1: value = read(top, topV.data(), e); break;
                    case 2: value = read(middle, middleV.data(), e); break;
                    case 3:
                        switch (e.kind)
                        {
                            case 5: value = low.portCurrent(e.index, lowV.data()) - low.portCurrent(q18Junction, lowV.data()); break;
                            case 6: value = low.portCurrent(e.index, lowV.data()) + low.portCurrent(q18Junction, lowV.data()); break;
                            case 9: value = r126Amps; break;
                            case 10: value = q25Emitter; break;
                            case 11: value = q25Base; break;
                            case 12: value = -q25Emitter - q25Base; break;
                            default: value = read(low, lowV.data(), e); break;
                        }
                        break;
                    case 4: value = read(mix, mixV.data(), e); break;
                    default: value = static_cast<double>(e.index); break; // the constants
                }
                out[k] = static_cast<float>(e.sign * value);
            }
        }

    private:
        void setRestInputs() noexcept
        {
            full.setInput(fullSupply, supply);
            full.setInput(fullDrive, 0.0);
            tail.setInput(tailSupply, supply);
            top.setInput(topSupply, supply);
            middle.setInput(middleSupply, supply);
            low.setInput(lowSupply, supply);
            mix.setInput(mixTop, 0.0);
            mix.setInput(mixMiddle, 0.0);
            mix.setInput(mixLow, 0.0);
        }

        template <class C>
        void setEnvelopeSources(C& c) noexcept
        {
            c.setSource(r94Env, h1);
            c.setSource(r90Env, h2);
            c.setSource(r104Env, u);
            c.setSource(r91Env, h2);
        }

        template <class C>
        void readEnvelope(const C& c) noexcept
        {
            envJ = c.probe(jProbe);
            envM = c.probe(mProbe);
            envN = c.probe(nProbe);
            envT = c.probe(tProbe);
            envV = c.probe(vProbe);
            envK = usingFull ? full.probe(kProbe) : 0.0;
        }

        void setVcaSources() noexcept
        {
            top.setSource(r94Vca, envJ);
            middle.setSource(r90Vca, envM);
            middle.setSource(r91Vca, envN);
            low.setSource(r104Vca, envT);
        }

        void readVcas() noexcept
        {
            h1 = top.probe(h1Probe);
            h2 = middle.probe(h2Probe);
            u = low.probe(uProbe);
        }

        // The envelopes. The tail: 1 the supply (input), 2 C38, 3 C37, 4 C40, 5 C39, 6 C41, 7 R93's end, 8 Q20's
        // emitter, 9 C45, 10 R89. The trigger's parts after them: 11 IC2a's output (input), 12 Q6's base, 13 its
        // collector, 14 Q19's base, 15 its collector, 16 C35, 17 its emitter.
        template <class C>
        void buildEnvelope(C& c, bool withTrigger)
        {
            const int s = c.addInput(1);
            c.addCapacitor(2, 0, 1.0e-6);                                                 // C38
            c.addResistor(2, 3, 22.0e3);                                                  // R87
            c.addCapacitor(3, 0, 2.2e-6);                                                 // C37
            r94Env = c.addSource(2, 0, 39.0e3);                                           // R94, to Q16's gate
            c.addCapacitor(4, 0, 1.0e-6);                                                 // C40
            c.addResistor(4, 5, 33.0e3);                                                  // R88
            c.addCapacitor(5, 0, 0.47e-6);                                                // C39
            r90Env = c.addSource(5, 0, 33.0e3);                                           // R90, to Q17's
            c.addCapacitor(6, 0, 1.0e-6);                                                 // C41
            c.addResistor(6, 7, 470.0e3);                                                 // R93
            vr2 = c.addResistor(7, 0, 1.0e6);                                             // CY DECAY
            c.addTransistor(1, 6, 8, transistorSaturation, betaF, betaR, thermal);        // Q20
            c.addResistor(8, 9, 33.0e3);                                                  // R105
            c.addCapacitor(9, 0, 2.2e-6);                                                 // C45
            r104Env = c.addSource(9, 0, 22.0e3);                                          // R104, to Q18's
            c.addResistor(8, 10, 33.0e3);                                                 // R92
            c.addResistor(10, 0, 10.0e3);                                                 // R89
            r91Env = c.addSource(10, 0, 33.0e3);                                          // R91, to Q17's
            jProbe = c.addProbe(2);
            mProbe = c.addProbe(5);
            nProbe = c.addProbe(10);
            tProbe = c.addProbe(9);
            vProbe = c.addProbe(6);
            if (! withTrigger)
            {
                tailSupply = s;
                return;
            }
            fullSupply = s;
            fullDrive = c.addInput(11);
            c.addResistor(11, 12, 4.7e3);                                                 // R15
            r10 = c.addResistor(12, 0, switchOff);                                        // R10 through Q2
            c.addPnpTransistor(13, 12, 11, transistorSaturation, betaF, betaR, thermal);  // Q6
            c.addResistor(13, 0, 22.0e3);                                                 // R83
            c.addResistor(13, 14, 10.0e3);                                                // R84
            c.addTransistor(15, 14, 17, transistorSaturation, betaF, betaR, thermal);     // Q19
            q19Junction = c.addCapacitor(14, 17, junctionEmitter);                        // its emitter junction (Q19 and the diodes)
            c.addResistor(15, 16, 100.0);                                                 // R85
            c35 = c.addCapacitor(16, 0, 47.0e-6);                                         // C35
            c.addResistor(1, 16, 100.0);                                                  // R86
            c.addDiode(17, 2, diodeSaturation, diodeThermal);                             // D6
            c.addDiode(17, 4, diodeSaturation, diodeThermal);                             // D7
            c.addDiode(17, 6, diodeSaturation, diodeThermal);                             // D8
            kProbe = c.addProbe(17);
        }

        // A swing VCA's front: 1 the band (input), 2 the supply (input), 3 the base, 4 the collector, 5 the emitter,
        // 6 the gate (the envelope's end, the diode's anode).
        template <class C>
        static int buildGate(C& c, int& band, int& supplyIn)
        {
            band = c.addInput(1);
            supplyIn = c.addInput(2);
            const int coupling = c.addCapacitor(1, 3, 22.0e-9);                            // C42 (C44, C46)
            c.addResistor(2, 3, 2.0e6);                                                    // R96 and R97 (R99, R100; R102, R103)
            c.addTransistor(4, 3, 5, transistorSaturation, betaF, betaR, thermal);         // Q16 (Q17, Q18)
            c.addResistor(5, 0, 100.0);                                                    // R95 (R98, R101)
            c.addDiode(6, 4, diodeSaturation, diodeThermal);                               // D5 (D11, D12)
            return coupling;
        }

        // Top band: the gate, then C36, C47, R94; 7 after C49, 8 C53/C54, 9 IC5a's plus, 10 its minus, 11 its
        // output, 12 its inner node.
        void buildTop()
        {
            c42 = buildGate(top, topBand, topSupply);
            top.addCapacitor(4, 0, 1.5e-9);                                                // C36
            top.addCapacitor(6, 0, 1.5e-9);                                                // C47
            r94Vca = top.addSource(6, 0, 39.0e3);                                          // R94, from C38
            c49 = top.addCapacitor(6, 7, 3.3e-9);                                          // C49
            top.addResistor(7, 0, 6.8e3);                                                  // R106
            c53 = top.addCapacitor(7, 8, 1.0e-9);                                          // C53
            c54 = top.addCapacitor(8, 9, 1.0e-9);                                          // C54
            top.addResistor(9, 0, 18.0e3);                                                 // R109
            top.addResistor(8, 11, 18.0e3);                                                // R108
            top.addResistor(10, 0, 22.0e3);                                                // R111
            top.addResistor(10, 11, 39.0e3);                                               // R110
            top.addOpAmp(9, 10, 11, 12, Tr808MetalBank::opAmpBandwidth);                   // IC5a (its swing never reached)
            h1Probe = top.addProbe(6);
            topOut = top.addProbe(11);
            top.addProbe(3);
        }

        // Middle band: the gate, then C43, C50, R90, R91; 7 C51/C52, 8 IC5b's plus, 9 its minus, 10 its output, 11 its
        // inner node.
        void buildMiddle()
        {
            c44 = buildGate(middle, middleBand, middleSupply);
            middle.addCapacitor(4, 0, 1.5e-9);                                             // C43
            middle.addCapacitor(6, 0, 1.5e-9);                                             // C50
            r90Vca = middle.addSource(6, 0, 33.0e3);                                       // R90, from C39
            r91Vca = middle.addSource(6, 0, 33.0e3);                                       // R91, from R89
            c51 = middle.addCapacitor(6, 7, 1.0e-9);                                       // C51
            c52 = middle.addCapacitor(7, 8, 1.0e-9);                                       // C52
            middle.addResistor(8, 0, 18.0e3);                                              // R113
            middle.addResistor(7, 10, 18.0e3);                                             // R107
            middle.addResistor(9, 0, 39.0e3);                                              // R115
            middle.addResistor(9, 10, 39.0e3);                                             // R114
            middle.addOpAmp(8, 9, 10, 11, Tr808MetalBank::opAmpBandwidth);                 // IC5b (likewise)
            h2Probe = middle.addProbe(6);
            middleOut = middle.addProbe(10);
            middle.addProbe(3);
        }

        // Low band: the gate, then R104; 7 C48/C59, 8 Q25's base, 9 its emitter.
        void buildLow()
        {
            c46 = buildGate(low, lowBandIn, lowSupply);
            q18Junction = low.addCapacitor(4, 3, junctionCollector);                       // Q18's collector junction (Q18 and D12)
            r104Vca = low.addSource(6, 0, 22.0e3);                                         // R104, from C45
            low.addCapacitor(6, 7, 1.5e-9);                                                // C48
            low.addResistor(7, 9, 22.0e3);                                                 // R124
            low.addCapacitor(7, 8, 1.5e-9);                                                // C59
            low.addResistor(8, 0, 82.0e3);                                                 // R127
            low.addNullor(8, 9, 9);                                                        // Q25, a follower
            uProbe = low.addProbe(6);
            lowOut = low.addProbe(9);
            low.addProbe(3);
        }

        // Tone and level: 1 to 3 the bands (inputs), 4 after C55, 5 R112's end, 6 R125's, 7 the lower bands' node, 8
        // after C56, 9 after C58, 10 C57, 11 R119 and R129's end, 12 IC6b's minus, 13 its output, 14 after C78.
        void buildMix()
        {
            mixTop = mix.addInput(1);
            mixMiddle = mix.addInput(2);
            mixLow = mix.addInput(3);
            mix.addCapacitor(1, 4, 10.0e-9);                     // C55
            mix.addResistor(4, 5, 22.0e3);                       // R112
            mix.addResistor(5, 11, 22.0e3);                      // R119
            vr4Top = mix.addResistor(5, 0, 10.0e3);              // CY TONE, R112's side of its wiper
            vr4Rest = mix.addResistor(0, 6, 10.0e3);             // and R125's
            mix.addResistor(6, 7, 2.2e3);                        // R125
            mix.addCapacitor(2, 8, 10.0e-9);                     // C56
            mix.addResistor(8, 7, 10.0e3);                       // R120
            mix.addCapacitor(3, 9, 10.0e-9);                     // C58
            mix.addResistor(9, 10, 100.0e3);                     // R123
            mix.addCapacitor(10, 0, 3.3e-9);                     // C57
            mix.addResistor(10, 7, 10.0e3);                      // R121
            mix.addResistor(7, 11, 15.0e3);                      // R129
            mix.addCapacitor(11, 12, 10.0e-9);                   // C90
            mix.addResistor(12, 13, 470.0e3);                    // R128
            mix.addCapacitor(12, 13, 220.0e-12);                 // C77
            mix.addCapacitor(12, 14, 33.0e-6);                   // C78
            vr6 = mix.addResistor(14, 13, 39.0e3);               // CY LEVEL
            mix.addNullor(0, 12, 13);                            // IC6b
            mixOut = mix.addProbe(13);
        }

        wdf::Circuit<17, 34, 2, 0, 9, 6, true> full; // Tapped: a panel reads its nodes and currents (D317)
        wdf::Circuit<10, 19, 1, 0, 2, 5, true> tail;
        wdf::Circuit<12, 19, 2, 1, 3, 3, true> top;
        wdf::Circuit<11, 18, 2, 1, 3, 3, true> middle;
        wdf::Circuit<9, 12, 2, 1, 3, 3, true> low;
        wdf::Circuit<14, 18, 3, 1, 0, 1, true> mix;

        struct RestState
        {
            decltype(full)::Snapshot full {};
            decltype(tail)::Snapshot tail {};
            decltype(top)::Snapshot top {};
            decltype(middle)::Snapshot middle {};
            decltype(low)::Snapshot low {};
            decltype(mix)::Snapshot mix {};
        };

        static const RestState& restState() noexcept
        {
            static const RestState state = []
            {
                RestState s {};
                auto cymbal = std::make_unique<Tr808Cymbal>();
                cymbal->sampleRate = 96000.0;
                cymbal->settle(0.0, 0.0);
                cymbal->top.setCapacitorVoltage(cymbal->c42, highCouplingRest);
                cymbal->middle.setCapacitorVoltage(cymbal->c44, highCouplingRest);
                cymbal->low.setCapacitorVoltage(cymbal->c46, lowCouplingRest);
                s.full = cymbal->full.snapshot();
                s.tail = cymbal->tail.snapshot();
                s.top = cymbal->top.snapshot();
                s.middle = cymbal->middle.snapshot();
                s.low = cymbal->low.snapshot();
                s.mix = cymbal->mix.snapshot();
                return s;
            }();
            return state;
        }

        double sampleRate = 96000.0;
        double pulseVolts = 7.0, pulseWidth = 0.001;
        double settleSeconds = triggerSettle; // setTriggerSettleForTesting
        int triggerSample = -1, sinceHit = 1 << 30;
        bool usingFull = false;
        double envJ = 0.0, envM = 0.0, envN = 0.0, envT = 0.0, envV = 0.0, envK = 0.0; // the envelopes' nodes this sample
        double h1 = 0.0, h2 = 0.0, u = 0.0;                           // the VCAs' gates this sample

        int fullSupply = 0, fullDrive = 0, tailSupply = 0, r10 = 0, vr2 = 0;
        int r94Env = 0, r90Env = 0, r104Env = 0, r91Env = 0;
        int jProbe = 0, mProbe = 0, nProbe = 0, tProbe = 0, vProbe = 0, kProbe = 0;
        int topBand = 0, topSupply = 0, c42 = 0, r94Vca = 0, c49 = 0, c53 = 0, c54 = 0, h1Probe = 0, topOut = 0;
        int middleBand = 0, middleSupply = 0, c44 = 0, r90Vca = 0, r91Vca = 0, c51 = 0, c52 = 0, h2Probe = 0, middleOut = 0;
        int lowBandIn = 0, lowSupply = 0, c46 = 0, r104Vca = 0, uProbe = 0, lowOut = 0;
        int mixTop = 0, mixMiddle = 0, mixLow = 0, vr4Top = 0, vr4Rest = 0, vr6 = 0, mixOut = 0;
        // For the tap (D317): C35's and the two junctions' ports, which envelope ran last, and C35 from the handover on.
        int c35 = 0, q19Junction = 0, q18Junction = 0;
        bool ranFull = false;
        int tailSamples = 0;          // the tail's samples since the handover
        double c35AtHandover = supply; // C35's volts as the full circuit left it
    };
}
