#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include "CircuitWdf.h"
#include "Tr808BassDrum.h"
#include "Tr808Cymbal.h"
#include "Tr808MetalBank.h"

/*
    The Roland TR-808's open and closed hi-hat (D254), from the service notes (pp. 6, 13) and tools/tr808/metal.cir,
    their reference. Both gate IC3's high band; the closed hat cuts the open one short. Eight circuits, each a wave
    digital filter (CircuitWdf.h):

        OH envelope IC2a's narrowed trigger through the switch Q7: through R135 into Q24, which charges C62 (its
                    collector fed through R137 and R136 with C61), and through R131 and R130 into Q21, which empties
                    C60. C62 drains slowly through R139 until C60, charging again through R132 and OH DECAY (VR3, 2
                    MOhm B), turns Q22 on through R133, which drains C62 through R134; Q23 does the same when the closed
                    hat's envelope reaches it through R173. Q29 follows C62 (R140, R143) into the VCA through R144
        CH envelope the switch Q8 through R141 into Q28, which charges C63 (its collector fed through R138 and R136 with
                    C61); C63 feeds the VCA through R142 and Q23's base through R173
        VCAs        the high band through C67 (C71) onto Q27's (Q30's) base, 2 MOhm its bias: a swing VCA pulling R144's
                    (R142's) end down through D13 (D14), C65 and C64 (C70 and C69) around it; then a Sallen-Key
                    high-pass around the emitter follower Q26 (Q31): 8.2 kHz (12.2 kHz), a Q of 2.5
        Outputs     C80 470 pF and R160 39 kOhm (C74, R162 33 kOhm) into IC7a (IC7b), its level pot (VR7, VR8, 100 kOhm A)
                    with C81 (C83) in its feedback, beside R159 (R161) and C79 (C82)

    R144, R142 and R173 join the circuits a sample apart, each holding the other's end as a source. Seen from the
    closed hat, Q23's base is a saturated transistor's (Is (1 / betaF + 1 / betaR)), as it is whenever the closed hat
    reaches it: the open hat's Q22 or Q23 then holds their collectors down. Once a hit has passed (its switch, Q21,
    Q24 or Q28 off), each envelope runs without the trigger's parts, the state passed across. The VCAs run while the
    bank does. Each high-pass's capacitors are prewarped at its peak. Transistors and diodes as the bass drum's; Q26,
    Q31 and IC7 ideal. Volts in, volts out.
*/
namespace coolsynth::synth::v3
{
    class Tr808Hats final
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
        static constexpr double openPeak = 8157.0, closedPeak = 12246.0;
        /** C67's and C71's voltage with the bank running and the hats at rest (as Tr808Cymbal's C42). */
        static constexpr double couplingRest = Tr808Cymbal::highCouplingRest;
        /** After a hit's pulse and switch, this long before an envelope drops the trigger's parts (D262, as the
            cymbal's). */
        static constexpr double triggerSettle = 0.00025;
        static constexpr double c63 = 0.47e-6;
        /** C60 (printed 10 uF) and R134 (printed 100 kOhm), fitted to both units' open hat across OH DECAY (D258). */
        static constexpr double c60Farads = 8.0e-6, r134Ohms = 82.0e3;

        enum Hat : int { open = 0, closed = 1 };

        Tr808Hats()
        {
            openFull.setNewtonLimit(40);
            openTail.setNewtonLimit(40);
            closedFull.setNewtonLimit(40);
            closedTail.setNewtonLimit(40);
            solveTo808Tolerance(openFull, openTail, closedFull, closedTail, openVca, closedVca, openOutput, closedOutput); // D356
            buildOpenEnvelope(openTail, false);
            buildOpenEnvelope(openFull, true);
            buildClosedEnvelope(closedTail, false);
            buildClosedEnvelope(closedFull, true);
            buildVca(openVca, openGate);
            buildVca(closedVca, closedGate);
            buildOutput(openOutput, openOut, 39.0e3);
            buildOutput(closedOutput, closedOut, 33.0e3);
        }

        void prepare(double rate) noexcept
        {
            sampleRate = rate;
            openFull.setSampleRate(rate);
            openTail.setSampleRate(rate);
            closedFull.setSampleRate(rate);
            closedTail.setSampleRate(rate);
            openVca.setSampleRate(rate);
            closedVca.setSampleRate(rate);
            openOutput.setSampleRate(rate);
            closedOutput.setSampleRate(rate);
            const double o = Tr808MetalBank::prewarp(openPeak, rate), c = Tr808MetalBank::prewarp(closedPeak, rate);
            openVca.setCapacitance(openGate.hp1, 1.5e-9 * o);
            openVca.setCapacitance(openGate.hp2, 1.5e-9 * o);
            closedVca.setCapacitance(closedGate.hp1, 1.0e-9 * c);
            closedVca.setCapacitance(closedGate.hp2, 1.0e-9 * c);
            rest();
        }

        void rest() noexcept
        {
            restEnvelope(open);
            restEnvelope(closed);
            restVcas();
        }

        /** A hat's envelope and output at rest (the VCAs, which run with the bank, as they are). */
        void restEnvelope(int hat) noexcept
        {
            const RestState& r = restState();
            if (hat == open)
            {
                openFull.setResistance(openSwitch, switchOff);
                openFull.restore(r.openFull);
                openTail.restore(r.openTail);
                openOutput.restore(r.openOutput);
            }
            else
            {
                closedFull.setResistance(closedSwitch, switchOff);
                closedFull.restore(r.closedFull);
                closedTail.restore(r.closedTail);
                closedOutput.restore(r.closedOutput);
            }
            auto& s = state[static_cast<std::size_t>(hat)];
            s.usingFull = false;
            s.triggerSample = -1;
            s.sinceHit = 1 << 30;
            s.ranFull = false;
            s.tailSamples = 0;
            s.c61AtHandover = hat == open ? openFull.portVoltage(c61Open) : closedFull.portVoltage(c61Closed);
            setRestInputs();
            if (hat == open)
                readOpen(openTail);
            else
                readClosed(closedTail);
        }

        void restVcas() noexcept
        {
            const RestState& r = restState();
            openVca.restore(r.openVca);
            closedVca.restore(r.closedVca);
            openGateVolts = closedGateVolts = 0.0;
        }

        static void warmUp() noexcept { (void) restState(); }

        /** Back to the circuit at rest by solving it, the high band at these volts (as the reference's time 0). */
        void settle(double highBand) noexcept
        {
            for (int hat = 0; hat < 2; ++hat)
            {
                auto& s = state[static_cast<std::size_t>(hat)];
                s.usingFull = false;
                s.triggerSample = -1;
                s.sinceHit = 1 << 30;
                s.ranFull = false;
                s.tailSamples = 0;
            }
            openFull.setResistance(openSwitch, switchOff);
            closedFull.setResistance(closedSwitch, switchOff);
            setRestInputs();
            openVca.setInput(openGate.band, highBand);
            closedVca.setInput(closedGate.band, highBand);
            openGateVolts = closedGateVolts = 0.0;
            openEnvelopeOut = closedEnvelope = 0.0;
            openChokeBase = closedChokeBase = 0.0;
            for (int pass = 0; pass < 4; ++pass)
            {
                setOpenSources(openTail);
                openTail.settle();
                readOpen(openTail);
                setClosedSources(closedTail);
                closedTail.settle();
                readClosed(closedTail);
                setVcaSources();
                openVca.settle();
                closedVca.settle();
                openGateVolts = openVca.probe(openGate.gateProbe);
                closedGateVolts = closedVca.probe(closedGate.gateProbe);
            }
            setOpenSources(openFull);
            openFull.settle();
            setClosedSources(closedFull);
            closedFull.settle();
            state[0].c61AtHandover = openFull.portVoltage(c61Open);
            state[1].c61AtHandover = closedFull.portVoltage(c61Closed);
            openOutput.setInput(openOut.in, openVca.probe(openGate.outProbe));
            openOutput.settle();
            closedOutput.setInput(closedOut.in, closedVca.probe(closedGate.outProbe));
            closedOutput.settle();
        }

        /** A hit: IC2a's output at this many volts for this long, through the hat's switch. */
        void trigger(int hat, double bufferVolts, double widthSeconds = 0.001) noexcept
        {
            auto& s = state[static_cast<std::size_t>(hat)];
            if (! s.usingFull)
            {
                if (hat == open)
                {
                    openFull.restore(restState().openFull);
                    openFull.adoptSharedState(openTail);
                }
                else
                {
                    closedFull.restore(restState().closedFull);
                    closedFull.adoptSharedState(closedTail);
                }
                s.usingFull = true;
            }
            s.pulseVolts = bufferVolts;
            s.pulseWidth = widthSeconds;
            s.triggerSample = 0;
            s.sinceHit = 0;
            if (hat == open)
                openFull.setResistance(openSwitch, 22.0e3); // the data stands: Q3 on, R12 to ground
            else
                closedFull.setResistance(closedSwitch, 22.0e3); // Q4 on, R11 to ground
        }

        /** OH DECAY 0..1: VR3, 2 MOhm linear, as a rheostat. */
        void setOpenDecay(double turn) noexcept
        {
            const double ohms = std::max(0.01, 2.0e6 * std::clamp(turn, 0.0, 1.0));
            openFull.setResistance(vr3, ohms);
            openTail.setResistance(vr3, ohms);
        }

        /** For fitting the open hat's decay (tests only, D258): C60, R134 and R139. The rest is not solved again: C60
            and R139 leave it as it is, R134 moves C62's by microvolts (Q22's leakage). */
        void setOpenEnvelopeParts(double c60, double r134, double r139) noexcept
        {
            const auto set = [&](auto& c)
            {
                c.setCapacitance(c60Port, c60);
                c.setResistance(r134Port, r134);
                c.setResistance(r139Port, r139);
            };
            set(openFull);
            set(openTail);
        }

        /** The CH Decay bend: C63 scaled (1 as drawn). */
        void setClosedDecayScale(double scale) noexcept
        {
            closedFull.setCapacitance(c63Port, c63 * scale);
            closedTail.setCapacitance(c63Port, c63 * scale);
        }

        /** OH LEVEL and CH LEVEL 0..1: the 100 kOhm audio-taper pots in IC7's feedback. */
        void setLevel(int hat, double turn) noexcept
        {
            const double t = std::clamp(turn, 0.0, 1.0);
            const double ohms = std::max(1.0, 100.0e3 * (std::pow(10.0, 2.0 * t) - 1.0) / 99.0);
            if (hat == open)
                openOutput.setResistance(openOut.level, ohms);
            else
                closedOutput.setResistance(closedOut.level, ohms);
        }

        /** How long the full circuits run after the pulse and switch, for the settle check (tests only, D262). */
        void setTriggerSettleForTesting(double seconds) noexcept { settleSeconds = seconds; }

        /** A hat's envelope, one sample (before processVcas), while it sounds. */
        void processEnvelope(int hat) noexcept
        {
            auto& s = state[static_cast<std::size_t>(hat)];
            double drive = 0.0;
            if (s.triggerSample >= 0)
            {
                const int width = std::max(1, static_cast<int>(std::lround(s.pulseWidth * sampleRate)));
                if (s.triggerSample >= 1 && s.triggerSample <= width)
                    drive = s.pulseVolts;
                if (s.triggerSample >= width + static_cast<int>(std::lround(instrumentHold * sampleRate)))
                {
                    if (hat == open)
                        openFull.setResistance(openSwitch, switchOff);
                    else
                        closedFull.setResistance(closedSwitch, switchOff);
                    s.triggerSample = -1;
                }
                else
                    ++s.triggerSample;
            }
            ++s.sinceHit;
            const int pulse = static_cast<int>(std::lround((s.pulseWidth + instrumentHold + settleSeconds) * sampleRate));
            const bool handOver = s.usingFull && s.triggerSample < 0 && s.sinceHit > pulse;
            if (hat == open)
            {
                if (s.usingFull)
                {
                    openFull.setInput(openDrive, drive);
                    setOpenSources(openFull);
                    openFull.process();
                    readOpen(openFull);
                    if (handOver)
                    {
                        openTail.adoptSharedState(openFull);
                        s.usingFull = false;
                    }
                }
                else
                {
                    setOpenSources(openTail);
                    openTail.process();
                    readOpen(openTail);
                }
            }
            else
            {
                if (s.usingFull)
                {
                    closedFull.setInput(closedDrive, drive);
                    setClosedSources(closedFull);
                    closedFull.process();
                    readClosed(closedFull);
                    if (handOver)
                    {
                        closedTail.adoptSharedState(closedFull);
                        s.usingFull = false;
                    }
                }
                else
                {
                    setClosedSources(closedTail);
                    closedTail.process();
                    readClosed(closedTail);
                }
            }
            // For the tap (D317): the envelope ran, and in which circuit; its C61 recovers from the handover on.
            envelopeRunning[static_cast<std::size_t>(hat)] = true;
            if (handOver)
            {
                s.ranFull = true; // this sample from the full circuit, the handover's too
                s.c61AtHandover = hat == open ? openFull.portVoltage(c61Open) : closedFull.portVoltage(c61Closed);
                s.tailSamples = 0;
            }
            else if (s.usingFull)
                s.ranFull = true;
            else
            {
                s.ranFull = false;
                if (s.tailSamples < (1 << 30))
                    ++s.tailSamples;
            }
        }

        /** Both VCAs, one sample, the high band at these volts (while the bank runs). */
        void processVcas(double highBand, bool runOpen = true, bool runClosed = true) noexcept
        {
            openVca.setInput(openGate.band, highBand);
            closedVca.setInput(closedGate.band, highBand);
            setVcaSources();
            // The 808's Lighter accuracy (CoolSynth's D339): a hat's VCA only while that hat sounds (D255's other way).
            if (runOpen)
                openVca.process();
            if (runClosed)
                closedVca.process();
            openGateVolts = openVca.probe(openGate.gateProbe);
            closedGateVolts = closedVca.probe(closedGate.gateProbe);
            // For the tap (D317): the envelopes this sample ran (they run before the VCAs), for after it.
            envelopeRan = envelopeRunning;
            envelopeRunning = { false, false };
        }

        /** A hat's output stage, one sample (after processVcas); returns the output in volts. */
        double processOutput(int hat) noexcept
        {
            if (hat == open)
            {
                openOutput.setInput(openOut.in, openVca.probe(openGate.outProbe));
                openOutput.process();
                return openOutput.probe(openOut.probe);
            }
            closedOutput.setInput(closedOut.in, closedVca.probe(closedGate.outProbe));
            closedOutput.process();
            return closedOutput.probe(closedOut.probe);
        }

        /** The closed hat's envelope reaching the open hat's Q23 through R173: while it does, the open hat is choked. */
        [[nodiscard]] double closedEnvelopeVolts() const noexcept { return closedEnvelope; }
        [[nodiscard]] double openEnvelopeVolts() const noexcept { return openEnvelopeC62; }
        [[nodiscard]] double openEnvelopeOutVolts() const noexcept { return openEnvelopeOut; }
        [[nodiscard]] double openTimerVolts() const noexcept { return openTimer; }
        [[nodiscard]] double gateVolts(int hat) const noexcept { return hat == open ? openGateVolts : closedGateVolts; }
        [[nodiscard]] double highPassVolts(int hat) const noexcept
        {
            return hat == open ? openVca.probe(openGate.outProbe) : closedVca.probe(closedGate.outProbe);
        }
        [[nodiscard]] double couplingVolts(int hat) const noexcept
        {
            return hat == open ? openVca.portVoltage(openGate.coupling) : closedVca.portVoltage(closedGate.coupling);
        }
        [[nodiscard]] bool usingTriggerParts(int hat) const noexcept { return state[static_cast<std::size_t>(hat)].usingFull; }
        /** Every circuit's Newton tolerance (D259's setting; the 808's Lighter accuracy, CoolSynth's D339). */
        void setNewtonTolerance(double volts) noexcept
        {
            openFull.setNewtonTolerance(volts);
            openTail.setNewtonTolerance(volts);
            closedFull.setNewtonTolerance(volts);
            closedTail.setNewtonTolerance(volts);
            openVca.setNewtonTolerance(volts);
            closedVca.setNewtonTolerance(volts);
            openOutput.setNewtonTolerance(volts);
            closedOutput.setNewtonTolerance(volts);
        }

        /** A hat's VCA's last Newton steps, for the cost tests. */
        [[nodiscard]] int vcaNewtonSteps(int hat) const noexcept { return hat == open ? openVca.lastNewtonSteps() : closedVca.lastNewtonSteps(); }
        /** A hat's envelope's last Newton steps (the full circuit's while the trigger stands), for the cost tests. */
        [[nodiscard]] int envelopeNewtonSteps(int hat) const noexcept
        {
            const bool full = state[static_cast<std::size_t>(hat)].usingFull;
            return hat == open ? (full ? openFull.lastNewtonSteps() : openTail.lastNewtonSteps())
                               : (full ? closedFull.lastNewtonSteps() : closedTail.lastNewtonSteps());
        }
        [[nodiscard]] bool lastSolveConverged() const noexcept
        {
            return (state[0].usingFull ? openFull.lastSolveConverged() : openTail.lastSolveConverged())
                   && (state[1].usingFull ? closedFull.lastSolveConverged() : closedTail.lastSolveConverged())
                   && openVca.lastSolveConverged() && closedVca.lastSolveConverged();
        }

        /** Every circuit solvable (CircuitWdf::isWellPosed). */
        [[nodiscard]] bool wellPosed() noexcept
        {
            return openFull.isWellPosed() && openTail.isWellPosed() && closedFull.isWellPosed() && closedTail.isWellPosed()
                   && openVca.isWellPosed() && closedVca.isWellPosed() && openOutput.isWellPosed() && closedOutput.isWellPosed();
        }

        // ---- Their circuit as it runs, for a panel that draws it (CoolSynth's D317) -------------------------------
        // Both hats on one sheet, as their ngspice recording has them (CoolSynth808's resources/lid: the open hat
        // struck, then the closed one 150 ms later, which chokes it): a capture starts at the open hat's hit. Each node
        // voltage and part current from the stage the engine models it in; where a resistor joins two circuits a
        // sample apart (R144, R142), from the VCA's side, whose source holds the envelope's node of this very sample.
        // Signs as ngspice's: a part's current from its first node to its second, a transistor's into its leads. Each
        // envelope from whichever circuit ran last; once a hit has passed and its tail runs (D262), its trigger's parts as
        // they then stand: the switch, Q21, Q24 and Q28 off, their nodes and currents 0, but for C61 and the collectors on
        // it, which recover through R136 from where the handover left them. The two envelopes each hold a copy of C61 and
        // R136, which are one in the circuit: being linear there, its volts are the two copies' droops together, its
        // currents their sum. The choke's Q23 and R173 from the open envelope while it runs, where Q23 is the whole
        // transistor; else from the closed one's, where Q23's base is a saturated junction and C62 is empty (no collector
        // current). Q26 and Q31 are ideal followers and R156 and R154 (to the -15 V supply) are left out: their currents
        // are worked out from the emitters' volts (which lack the Vbe, so they read some 6 % high), the emitter currents
        // from all each emitter feeds, the base currents from betaF. Not here, as the engine leaves them out: the data
        // lines J7 and J8 and the switches Q3 and Q4 with R1, R8, R2, R7 and their collectors (R12 through Q3 and R11
        // through Q4 are one part each).
        //
        // Kinds: 0 a node, 1 a port's current, 2 a transistor's emitter from its base port, 7 the middle of a 2 MOhm bias
        // (two 1 MOhm in the circuit), 8 a constant (the index its volts), 9 the follower's emitter resistor, 10 its
        // emitter, 11 its base, 12 its collector, 13 IC2a's output (nv2), 14 C61's node, 15 R136's current, 16 C61's, 17 a
        // collector on C61 (nc24, nc28), 18 Q23's base, 19 R173's current, 20 Q23's base current, 21 its collector's, 22
        // its emitter's. Stages: 0 the open envelope (full or tail), 1 the closed one, 2 the open VCA, 3 the closed VCA,
        // 4 the open output, 5 the closed output; -1 none.

        static constexpr int liveCount = 149;
        static constexpr std::array<const char*, liveCount> liveNames { "n132", "n148", "n151", "n61", "n66", "n72", "n74", "n7am", "n7ap", "n7bm", "n7bp", "n80", "n81", "n83", "nb22", "nb23", "nb24", "nb26", "nb27", "nb28", "nb29", "nb30", "nb31", "nb7", "nb8", "nc24", "nc27", "nc28", "nc30", "nce", "nch8", "ne1", "ne27", "ne30", "ng2", "nhb", "nhpc", "nhpo", "noenv", "noh7", "nqc", "nv1", "nv2", "nv2c", "ny1", "vcc", "vch", "vee", "voh", "r12.i", "r17.i", "q7.ic", "q7.ib", "q7.ie", "r11.i", "r16.i", "q8.ic", "q8.ib", "q8.ie", "r131.i", "r130.i", "q21.ic", "q21.ib", "q21.ie", "r135.i", "q24.ic", "q24.ib", "q24.ie", "r137.i", "c61.i", "r136.i", "c62.i", "r139.i", "r134.i", "q22.ic", "q22.ib", "q22.ie", "q23.ic", "q23.ib", "q23.ie", "r133.i", "c60.i", "r132.i", "rvr3.i", "r173.i", "r140.i", "q29.ic", "q29.ib", "q29.ie", "r143.i", "r144.i", "c67.i", "r148.i", "r149.i", "q27.ic", "q27.ib", "q27.ie", "r145.i", "c65.i", "d13.id", "c64.i", "c66.i", "r147.i", "c68.i", "r146.i", "q26.ic", "q26.ib", "q26.ie", "r156.i", "c80.i", "r160.i", "r158.i", "r159.i", "c79.i", "c81.i", "rvr7.i", "r13.i", "r141.i", "q28.ic", "q28.ib", "q28.ie", "r138.i", "c63.i", "r142.i", "c71.i", "r151.i", "r150.i", "q30.ic", "q30.ib", "q30.ie", "r152.i", "c70.i", "d14.id", "c69.i", "c72.i", "r153.i", "c73.i", "r155.i", "q31.ic", "q31.ib", "q31.ie", "r154.i", "c74.i", "r162.i", "r157.i", "r161.i", "c82.i", "c83.i", "rvr8.i" };
        /** How long a capture from the open hat's hit runs, as the recording: milliseconds. */
        static constexpr double liveMs = 400.0;

        /** The named values after the last sample, in liveNames' order, from the stages' own state (their taps). */
        void tap(float* out) const noexcept
        {
            struct Entry { int stage, kind, index, sign; };
            static constexpr std::array<Entry, liveCount> entries { {
              { 0, 0, 6, 1 }, { 2, 7, 2, 1 }, { 3, 7, 2, 1 }, { -1, 14, 0, 1 }, { 2, 0, 7, 1 }, { 3, 0, 7, 1 }, { 5, 0, 2, 1 }, { 4, 0, 3, 1 },
              { -1, 8, 0, 1 }, { 5, 0, 3, 1 }, { -1, 8, 0, 1 }, { 4, 0, 2, 1 }, { 4, 0, 5, 1 }, { 5, 0, 5, 1 }, { 0, 0, 4, 1 }, { -1, 18, 0, 1 },
              { 0, 0, 14, 1 }, { 2, 0, 8, 1 }, { 2, 0, 3, 1 }, { 1, 0, 7, 1 }, { 0, 0, 8, 1 }, { 3, 0, 3, 1 }, { 3, 0, 8, 1 }, { 0, 0, 11, 1 },
              { 1, 0, 5, 1 }, { 0, 17, 15, 1 }, { 2, 0, 4, 1 }, { 1, 17, 8, 1 }, { 3, 0, 4, 1 }, { 1, 0, 1, 1 }, { 1, 0, 6, 1 }, { 0, 0, 2, 1 },
              { 2, 0, 5, 1 }, { 3, 0, 5, 1 }, { 0, 0, 5, 1 }, { 2, 0, 1, 1 }, { 3, 0, 9, 1 }, { 2, 0, 9, 1 }, { 0, 0, 9, 1 }, { 0, 0, 12, 1 },
              { 0, 0, 3, 1 }, { 2, 0, 6, 1 }, { -1, 13, 0, 1 }, { 3, 0, 6, 1 }, { 0, 0, 13, 1 }, { 2, 0, 2, 1 }, { 5, 0, 4, 1 }, { -1, 8, -15, 1 },
              { 4, 0, 4, 1 }, { 0, 1, 18, 1 }, { 0, 1, 17, 1 }, { 0, 1, 20, 1 }, { 0, 1, 19, 1 }, { 0, 2, 19, 1 }, { 1, 1, 5, 1 }, { 1, 1, 4, 1 },
              { 1, 1, 7, 1 }, { 1, 1, 6, 1 }, { 1, 2, 6, 1 }, { 0, 1, 21, 1 }, { 0, 1, 22, 1 }, { 0, 1, 24, 1 }, { 0, 1, 23, 1 }, { 0, 2, 23, 1 },
              { 0, 1, 25, 1 }, { 0, 1, 27, 1 }, { 0, 1, 26, 1 }, { 0, 2, 26, 1 }, { 0, 1, 28, 1 }, { -1, 16, 0, 1 }, { -1, 15, 0, 1 }, { 0, 1, 0, 1 },
              { 0, 1, 1, 1 }, { 0, 1, 2, 1 }, { 0, 1, 4, 1 }, { 0, 1, 3, 1 }, { 0, 2, 3, 1 }, { -1, 21, 0, 1 }, { -1, 20, 0, 1 }, { -1, 22, 0, 1 },
              { 0, 1, 5, 1 }, { 0, 1, 6, 1 }, { 0, 1, 7, 1 }, { 0, 1, 8, 1 }, { -1, 19, 0, 1 }, { 0, 1, 12, 1 }, { 0, 1, 14, 1 }, { 0, 1, 13, 1 },
              { 0, 2, 13, 1 }, { 0, 1, 15, 1 }, { 2, 1, 8, -1 }, { 2, 1, 0, 1 }, { 2, 1, 1, 1 }, { 2, 1, 1, 1 }, { 2, 1, 3, 1 }, { 2, 1, 2, 1 },
              { 2, 2, 2, 1 }, { 2, 1, 4, 1 }, { 2, 1, 5, 1 }, { 2, 1, 6, 1 }, { 2, 1, 7, 1 }, { 2, 1, 9, 1 }, { 2, 1, 10, 1 }, { 2, 1, 11, 1 },
              { 2, 1, 12, 1 }, { 2, 12, 9, 1 }, { 2, 11, 9, 1 }, { 2, 10, 9, 1 }, { 2, 9, 9, 1 }, { 4, 1, 0, 1 }, { 4, 1, 1, 1 }, { -1, 8, 0, 1 },
              { 4, 1, 2, 1 }, { 4, 1, 3, 1 }, { 4, 1, 4, 1 }, { 4, 1, 5, 1 }, { 1, 1, 8, 1 }, { 1, 1, 9, 1 }, { 1, 1, 11, 1 }, { 1, 1, 10, 1 },
              { 1, 2, 10, 1 }, { 1, 1, 12, -1 }, { 1, 1, 0, 1 }, { 3, 1, 8, -1 }, { 3, 1, 0, 1 }, { 3, 1, 1, 1 }, { 3, 1, 1, 1 }, { 3, 1, 3, 1 },
              { 3, 1, 2, 1 }, { 3, 2, 2, 1 }, { 3, 1, 4, 1 }, { 3, 1, 5, 1 }, { 3, 1, 6, 1 }, { 3, 1, 7, 1 }, { 3, 1, 9, 1 }, { 3, 1, 10, 1 },
              { 3, 1, 11, 1 }, { 3, 1, 12, 1 }, { 3, 12, 9, 1 }, { 3, 11, 9, 1 }, { 3, 10, 9, 1 }, { 3, 9, 9, 1 }, { 5, 1, 0, 1 }, { 5, 1, 1, 1 },
              { -1, 8, 0, 1 }, { 5, 1, 2, 1 }, { 5, 1, 3, 1 }, { 5, 1, 4, 1 }, { 5, 1, 5, 1 } } };
            static constexpr int openTailNodes = 9, openTailPorts = 17, closedTailNodes = 3, closedTailPorts = 4; // the tails'
            static constexpr double r136Ohms = 100.0, c61Seconds = 100.0 * 47.0e-6; // R136 and C61
            static constexpr double followerOhms = 22.0e3;                          // R156 and R154
            static constexpr int q23Base = 9, q23Collector = 10;                    // the open envelope's ports
            static constexpr int r173Closed = 1, q23Junction = 2;                   // the closed envelope's
            const HatState& openState = state[0];
            const HatState& closedState = state[1];
            // each stage's node voltages, once
            std::array<double, 17> openV {};
            std::array<double, 10> closedV {}, openVcaV {}, closedVcaV {};
            std::array<double, 6> openOutV {}, closedOutV {};
            if (openState.ranFull)
                openFull.nodeVoltages(openV.data());
            else
                openTail.nodeVoltages(openV.data());
            if (closedState.ranFull)
                closedFull.nodeVoltages(closedV.data());
            else
                closedTail.nodeVoltages(closedV.data());
            openVca.nodeVoltages(openVcaV.data());
            closedVca.nodeVoltages(closedVcaV.data());
            openOutput.nodeVoltages(openOutV.data());
            closedOutput.nodeVoltages(closedOutV.data());
            const auto openCurrent = [&](int port) { return openState.ranFull ? openFull.portCurrent(port, openV.data()) : openTail.portCurrent(port, openV.data()); };
            const auto closedCurrent = [&](int port) { return closedState.ranFull ? closedFull.portCurrent(port, closedV.data()) : closedTail.portCurrent(port, closedV.data()); };

            // C61: each envelope's copy from its full circuit, or once the hit has passed recovering through R136 (Q24, Q28
            // off) from where the handover left it; the circuit's one C61 the two copies' droops together.
            const auto recovered = [this](const HatState& s)
            {
                return supply - (supply - s.c61AtHandover) * std::exp(-static_cast<double>(s.tailSamples) / (sampleRate * c61Seconds));
            };
            const double openC61 = openState.ranFull ? openV[16] : recovered(openState);
            const double closedC61 = closedState.ranFull ? closedV[9] : recovered(closedState);
            const double c61Volts = openC61 + closedC61 - supply;
            const double r136Amps = (openState.ranFull ? openFull.portCurrent(r136Open, openV.data()) : (supply - openC61) / r136Ohms)
                                    + (closedState.ranFull ? closedFull.portCurrent(r136Closed, closedV.data()) : (supply - closedC61) / r136Ohms);
            const double c61Amps = (openState.ranFull ? openFull.portCurrent(c61Open, openV.data()) : (supply - openC61) / r136Ohms)
                                   + (closedState.ranFull ? closedFull.portCurrent(c61Closed, closedV.data()) : (supply - closedC61) / r136Ohms);
            // IC2a's output, while either hat's pulse passes it
            const double nv2 = std::max(openState.ranFull ? openV[10] : 0.0, closedState.ranFull ? closedV[4] : 0.0);
            // The choke: from the open envelope while it runs, else from the closed one's
            const bool openRan = envelopeRan[0];
            const double chokeBase = openRan ? openV[7] : closedV[2];
            const double r173Amps = openRan ? -openCurrent(r173Open) : closedCurrent(r173Closed);
            const double q23BaseAmps = openRan ? openCurrent(q23Base) : closedCurrent(q23Junction);
            const double q23CollectorAmps = openRan ? openCurrent(q23Collector) : 0.0;
            // Q26's and Q31's emitters: R156 (R154) to -15 V, C80 (C74) on into the output, less what R147 (R153) brings in.
            const double openLoad = (openVcaV[9] + supply) / followerOhms;
            const double closedLoad = (closedVcaV[9] + supply) / followerOhms;
            const double openEmitter = -(openLoad + openOutput.portCurrent(0, openOutV.data()) - openVca.portCurrent(10, openVcaV.data()));
            const double closedEmitter = -(closedLoad + closedOutput.portCurrent(0, closedOutV.data()) - closedVca.portCurrent(10, closedVcaV.data()));

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
            const auto follower = [](const Entry& e, double load, double emitter) -> double
            {
                const double base = -emitter / (betaF + 1.0);
                switch (e.kind)
                {
                    case 9: return load;
                    case 10: return emitter;
                    case 11: return base;
                    default: return -emitter - base;
                }
            };
            // An envelope's entry: from the circuit that ran; the tail's trigger's parts off, a collector on C61 at its volts.
            const auto envelope = [&](const Entry& e, bool full, int tailNodes, int tailPorts, const auto& fullCircuit, const auto& tailCircuit,
                                      const double* v) -> double
            {
                if (e.kind == 17)
                    return full ? v[e.index] : c61Volts;
                if (full)
                    return read(fullCircuit, v, e);
                return (e.kind == 0 ? e.index <= tailNodes : e.index < tailPorts) ? read(tailCircuit, v, e) : 0.0;
            };
            for (int k = 0; k < liveCount; ++k)
            {
                const auto& e = entries[static_cast<std::size_t>(k)];
                double value = 0.0;
                switch (e.stage)
                {
                    case 0: value = envelope(e, openState.ranFull, openTailNodes, openTailPorts, openFull, openTail, openV.data()); break;
                    case 1: value = envelope(e, closedState.ranFull, closedTailNodes, closedTailPorts, closedFull, closedTail, closedV.data()); break;
                    case 2: value = e.kind >= 9 ? follower(e, openLoad, openEmitter) : read(openVca, openVcaV.data(), e); break;
                    case 3: value = e.kind >= 9 ? follower(e, closedLoad, closedEmitter) : read(closedVca, closedVcaV.data(), e); break;
                    case 4: value = read(openOutput, openOutV.data(), e); break;
                    case 5: value = read(closedOutput, closedOutV.data(), e); break;
                    default:
                        switch (e.kind)
                        {
                            case 13: value = nv2; break;
                            case 14: value = c61Volts; break;
                            case 15: value = r136Amps; break;
                            case 16: value = c61Amps; break;
                            case 18: value = chokeBase; break;
                            case 19: value = r173Amps; break;
                            case 20: value = q23BaseAmps; break;
                            case 21: value = q23CollectorAmps; break;
                            case 22: value = -(q23BaseAmps + q23CollectorAmps); break;
                            default: value = static_cast<double>(e.index); break; // the constants
                        }
                        break;
                }
                out[k] = static_cast<float>(e.sign * value);
            }
        }

    private:
        struct HatState
        {
            bool usingFull = false;
            int triggerSample = -1, sinceHit = 1 << 30;
            double pulseVolts = 7.0, pulseWidth = 0.001;
            // For the tap (D317): which circuit ran last, and the envelope's C61 from the handover on.
            bool ranFull = false;
            int tailSamples = 0;           // the tail's samples since the handover
            double c61AtHandover = supply; // its C61's volts as the full circuit left it
        };

        struct Gate
        {
            int band = 0, supplyIn = 0, coupling = 0, envelope = 0, hp1 = 0, hp2 = 0, gateProbe = 0, outProbe = 0;
        };

        struct Output
        {
            int in = 0, level = 0, probe = 0;
        };

        void setRestInputs() noexcept
        {
            openFull.setInput(openFullSupply, supply);
            openFull.setInput(openDrive, 0.0);
            openTail.setInput(openTailSupply, supply);
            closedFull.setInput(closedFullSupply, supply);
            closedFull.setInput(closedDrive, 0.0);
            openVca.setInput(openGate.supplyIn, supply);
            closedVca.setInput(closedGate.supplyIn, supply);
            openOutput.setInput(openOut.in, 0.0);
            closedOutput.setInput(closedOut.in, 0.0);
        }

        template <class C>
        void setOpenSources(C& c) noexcept
        {
            c.setSource(r173Open, closedEnvelope);
            c.setSource(r144Env, openGateVolts);
        }

        template <class C>
        void setClosedSources(C& c) noexcept
        {
            c.setSource(r142Env, closedGateVolts);
        }

        template <class C>
        void readOpen(const C& c) noexcept
        {
            openEnvelopeC62 = c.probe(e1Probe);
            openEnvelopeOut = c.probe(oenvProbe);
            openTimer = c.probe(g2Probe);
            openChokeBase = c.probe(b23Probe);
        }

        template <class C>
        void readClosed(const C& c) noexcept
        {
            closedEnvelope = c.probe(ceProbe);
            closedChokeBase = c.probe(b23cProbe);
        }

        void setVcaSources() noexcept
        {
            openVca.setSource(openGate.envelope, openEnvelopeOut);
            closedVca.setSource(closedGate.envelope, closedEnvelope);
        }

        // OH envelope. The tail: 1 the supply (input), 2 C62, 3 Q22's and Q23's collectors, 4 Q22's base, 5 C60, 6
        // VR3's end, 7 Q23's base, 8 Q29's base, 9 its emitter. The trigger's parts after them: 10 IC2a's output
        // (input), 11 Q7's base, 12 its collector, 13 Q21's base, 14 Q24's base, 15 its collector, 16 C61.
        template <class C>
        void buildOpenEnvelope(C& c, bool withTrigger)
        {
            const int s = c.addInput(1);
            c.addCapacitor(2, 0, 0.47e-6);                                                // C62
            r139Port = c.addResistor(2, 0, 1.0e6);                                        // R139
            r134Port = c.addResistor(2, 3, r134Ohms);                                     // R134
            c.addTransistor(3, 4, 0, transistorSaturation, betaF, betaR, thermal);        // Q22
            c.addResistor(5, 4, 10.0e3);                                                  // R133
            c60Port = c.addCapacitor(5, 0, c60Farads);                                    // C60
            c.addResistor(6, 5, 100.0e3);                                                 // R132
            vr3 = c.addResistor(1, 6, 1.0e6);                                             // OH DECAY
            c.addTransistor(3, 7, 0, transistorSaturation, betaF, betaR, thermal);        // Q23
            r173Open = c.addSource(7, 0, 330.0e3);                                        // R173, from C63
            c.addResistor(2, 8, 33.0e3);                                                  // R140
            c.addTransistor(1, 8, 9, transistorSaturation, betaF, betaR, thermal);        // Q29
            c.addResistor(9, 0, 33.0e3);                                                  // R143
            r144Env = c.addSource(9, 0, 33.0e3);                                          // R144, to Q27's gate
            e1Probe = c.addProbe(2);
            oenvProbe = c.addProbe(9);
            g2Probe = c.addProbe(5);
            b23Probe = c.addProbe(7);
            if (! withTrigger)
            {
                openTailSupply = s;
                return;
            }
            openFullSupply = s;
            openDrive = c.addInput(10);
            c.addResistor(10, 11, 4.7e3);                                                 // R17
            openSwitch = c.addResistor(11, 0, switchOff);                                 // R12 through Q3
            c.addPnpTransistor(12, 11, 10, transistorSaturation, betaF, betaR, thermal);  // Q7
            c.addResistor(12, 13, 22.0e3);                                                // R131
            c.addResistor(13, 0, 10.0e3);                                                 // R130
            c.addTransistor(5, 13, 0, transistorSaturation, betaF, betaR, thermal);       // Q21
            c.addResistor(12, 14, 10.0e3);                                                // R135
            c.addTransistor(15, 14, 2, transistorSaturation, betaF, betaR, thermal);      // Q24
            c.addResistor(15, 16, 470.0);                                                 // R137
            c61Open = c.addCapacitor(16, 0, 47.0e-6);                                     // C61
            r136Open = c.addResistor(1, 16, 100.0);                                       // R136
            c.addProbe(12);
        }

        // CH envelope. The tail: 1 C63, 2 Q23's base, 3 the supply (input). The trigger's parts after them: 4 IC2a's
        // output (input), 5 Q8's base, 6 its collector, 7 Q28's base, 8 its collector, 9 C61.
        template <class C>
        void buildClosedEnvelope(C& c, bool withTrigger)
        {
            const int s = c.addInput(3);
            c63Port = c.addCapacitor(1, 0, c63);                                          // C63
            c.addResistor(1, 2, 330.0e3);                                                 // R173
            c.addDiode(2, 0, transistorSaturation * (1.0 / betaF + 1.0 / betaR), thermal); // Q23's base, saturated
            r142Env = c.addSource(1, 0, 33.0e3);                                          // R142, to Q30's gate
            ceProbe = c.addProbe(1);
            b23cProbe = c.addProbe(2);
            if (! withTrigger)
                return;
            closedFullSupply = s;
            closedDrive = c.addInput(4);
            c.addResistor(4, 5, 4.7e3);                                                   // R16
            closedSwitch = c.addResistor(5, 0, switchOff);                                // R11 through Q4
            c.addPnpTransistor(6, 5, 4, transistorSaturation, betaF, betaR, thermal);     // Q8
            c.addResistor(6, 0, 22.0e3);                                                  // R13
            c.addResistor(6, 7, 33.0e3);                                                  // R141
            c.addTransistor(8, 7, 1, transistorSaturation, betaF, betaR, thermal);        // Q28
            c.addResistor(8, 9, 470.0);                                                   // R138
            c61Closed = c.addCapacitor(9, 0, 47.0e-6);                                    // C61
            r136Closed = c.addResistor(3, 9, 100.0);                                      // R136
            c.addProbe(6);
        }

        // A hat's VCA and high-pass: 1 the high band (input), 2 the supply (input), 3 the base, 4 the collector, 5 the
        // emitter, 6 the gate (R144's or R142's end), 7 between the high-pass's capacitors, 8 the follower's base, 9 its
        // emitter.
        template <class C>
        void buildVca(C& c, Gate& g)
        {
            const bool isOpen = &g == &openGate;
            const double hp = isOpen ? 1.5e-9 : 1.0e-9;
            g.band = c.addInput(1);
            g.supplyIn = c.addInput(2);
            g.coupling = c.addCapacitor(1, 3, 22.0e-9);                                   // C67 (C71)
            c.addResistor(2, 3, 2.0e6);                                                   // R148 and R149 (R150, R151)
            c.addTransistor(4, 3, 5, transistorSaturation, betaF, betaR, thermal);        // Q27 (Q30)
            c.addResistor(5, 0, 100.0);                                                   // R145 (R152)
            c.addCapacitor(4, 0, 1.5e-9);                                                 // C65 (C70)
            c.addDiode(6, 4, diodeSaturation, diodeThermal);                              // D13 (D14)
            c.addCapacitor(6, 0, 1.5e-9);                                                 // C64 (C69)
            g.envelope = c.addSource(6, 0, 33.0e3);                                       // R144 (R142)
            g.hp1 = c.addCapacitor(6, 7, hp);                                             // C66 (C72)
            c.addResistor(7, 9, 2.7e3);                                                   // R147 (R153)
            g.hp2 = c.addCapacitor(7, 8, hp);                                             // C68 (C73)
            c.addResistor(8, 0, 68.0e3);                                                  // R146 (R155)
            c.addNullor(8, 9, 9);                                                         // Q26 (Q31), a follower
            g.gateProbe = c.addProbe(6);
            g.outProbe = c.addProbe(9);
            c.addProbe(3);
        }

        // An output: 1 the follower's emitter (input), 2 after C80 (C74), 3 IC7's minus, 4 its output, 5 after C81 (C83).
        template <class C>
        static void buildOutput(C& c, Output& o, double inputOhms)
        {
            o.in = c.addInput(1);
            c.addCapacitor(1, 2, 470.0e-12);                   // C80 (C74)
            c.addResistor(2, 3, inputOhms);                    // R160 39k (R162 33k)
            c.addResistor(3, 4, 470.0e3);                      // R159 (R161)
            c.addCapacitor(3, 4, 220.0e-12);                   // C79 (C82)
            c.addCapacitor(3, 5, 33.0e-6);                     // C81 (C83)
            o.level = c.addResistor(5, 4, 39.0e3);             // VR7 (VR8)
            c.addNullor(0, 3, 4);                              // IC7a (IC7b)
            o.probe = c.addProbe(4);
        }

        wdf::Circuit<16, 31, 2, 0, 12, 5, true> openFull; // Tapped: a panel reads its nodes and currents (D317)
        wdf::Circuit<9, 17, 1, 0, 6, 4, true> openTail;
        wdf::Circuit<9, 15, 2, 0, 5, 3, true> closedFull;
        wdf::Circuit<3, 4, 1, 0, 1, 2, true> closedTail;
        wdf::Circuit<9, 13, 2, 1, 3, 3, true> openVca, closedVca;
        wdf::Circuit<5, 6, 1, 1, 0, 1, true> openOutput, closedOutput;

        struct RestState
        {
            decltype(openFull)::Snapshot openFull {};
            decltype(openTail)::Snapshot openTail {};
            decltype(closedFull)::Snapshot closedFull {};
            decltype(closedTail)::Snapshot closedTail {};
            decltype(openVca)::Snapshot openVca {}, closedVca {};
            decltype(openOutput)::Snapshot openOutput {}, closedOutput {};
        };

        static const RestState& restState() noexcept
        {
            static const RestState rest = []
            {
                RestState s {};
                auto hats = std::make_unique<Tr808Hats>();
                hats->sampleRate = 96000.0;
                hats->settle(0.0);
                hats->openVca.setCapacitorVoltage(hats->openGate.coupling, couplingRest);
                hats->closedVca.setCapacitorVoltage(hats->closedGate.coupling, couplingRest);
                s.openFull = hats->openFull.snapshot();
                s.openTail = hats->openTail.snapshot();
                s.closedFull = hats->closedFull.snapshot();
                s.closedTail = hats->closedTail.snapshot();
                s.openVca = hats->openVca.snapshot();
                s.closedVca = hats->closedVca.snapshot();
                s.openOutput = hats->openOutput.snapshot();
                s.closedOutput = hats->closedOutput.snapshot();
                return s;
            }();
            return rest;
        }

        double sampleRate = 96000.0;
        double settleSeconds = triggerSettle; // setTriggerSettleForTesting
        std::array<HatState, 2> state {};
        double openEnvelopeC62 = 0.0, openEnvelopeOut = 0.0, openTimer = 0.0, openChokeBase = 0.0;
        double closedEnvelope = 0.0, closedChokeBase = 0.0;
        double openGateVolts = 0.0, closedGateVolts = 0.0;

        int openFullSupply = 0, openTailSupply = 0, openDrive = 0, openSwitch = 0, vr3 = 0, r173Open = 0, r144Env = 0;
        int c60Port = 0, r134Port = 0, r139Port = 0;
        int e1Probe = 0, oenvProbe = 0, g2Probe = 0, b23Probe = 0;
        int closedFullSupply = 0, closedDrive = 0, closedSwitch = 0, c63Port = 0, r142Env = 0, ceProbe = 0, b23cProbe = 0;
        Gate openGate, closedGate;
        Output openOut, closedOut;
        // For the tap (D317): each full envelope's C61 and R136, and which envelopes ran this sample and the last.
        int c61Open = 0, r136Open = 0, c61Closed = 0, r136Closed = 0;
        std::array<bool, 2> envelopeRunning {}, envelopeRan {};
    };
}
