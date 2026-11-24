#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "CircuitWdf.h"
#include "Tb303Filter.h"

/*
    The Roland TB-303's VCO as its circuit (D275, D279, D290), from the service notes (p. 5) and tools/tb303/vco.cir,
    its reference:

        Pitch       the CPU's note as the 6-bit ladder's voltage behind 100 kOhm; the slide (IC12's switches, C35);
                    IC11b; the exponential converter (IC11a driving Q26's emitters through R104, R103 and TM4 its
                    reference current, TM5, R106, the posistor R100 and TUNING through R118 at Q26's right base). One
                    circuit; its output is Q26's right collector current
        Saw         C33 (10 nF, with 3.8 pF of junctions) discharged by that current, exactly (Q26's collector holds no
                    Early effect in the reference: the ramp is straight). The thyristor (Q24, Q25, Q27) is an event
                    (D279): at the threshold, found at its time inside the step, C33 jumps to the top and is held there
                    for the reset's dead time, which may run on into the next step. Threshold, top and dead time as
                    ngspice has the thyristor do it (5.35 to 5.37 V, 11.107 V, 1 us)
        Follower    Q28 (2SK30, O rank), a source follower over R105: the square law solved per step, its output
                    resistance (1 / gm) carried into the shaper as a correction to a fixed port
        Square      Q8 and its networks (R35, C10, R34; R45, C11; R36), one circuit, driven by the raw saw

    Band-limiting (D290): the shaper is driven by the raw saw, so both outputs jump on the step after the reset (the
    square's falling edge, 0.24 us in ngspice, is the reset's: Q8 lets go when the saw jumps). Each output's jump there,
    less its slope before it, is spread by a 4-point B-spline polyBLEP placed at the reset's time (Valimaki, Pekonen,
    Nam, JASA 2012); both outputs come two steps late. The shaper may run faster than the voice (setShaperOversampling),
    its outputs brought back by a Kaiser-windowed sinc: what is left of the square's aliasing is its rising edge's
    corners, Q8 starting to conduct and saturating, which halve with each doubling.

    Volts in (the ladder's voltage, 1/12 V a step: the A key unshifted 2.75 V), volts out (the saw at Q28's source,
    loaded by the shaper; the square at Q8's collector). Rates near 192 kHz (D279).
*/
namespace coolsynth::synth::v3
{
    class Tb303Oscillator final
    {
        friend struct Tb303Lid; // the lid tap reads its circuits and state (Tb303Lid.h)
    public:
        static constexpr double thermal = Tb303Filter::thermal;
        static constexpr double supply = Tb303Filter::supply, reference = Tb303Filter::reference;
        static constexpr double dualSaturation = 10.0e-15, dualBetaF = 500.0, dualBetaR = 4.0;
        // C33 and 3.8 pF of the junctions on its node as ngspice models them (Q26's collector, Q28's gate, Q25's
        // emitter), fitted from the pitch: without them the saw ran 0.57 to 0.71 cents sharp at every note.
        static constexpr double timing = 10.0e-9 + 3.8e-12;
        static constexpr double top = 11.107, deadTime = 1.0e-6;    // where the thyristor leaves C33, and after how long
        static constexpr double jfetBeta = 1.0e-3, jfetPinch = -1.0, jfetLambda = 2.0e-3, r105 = 10.0e3;
        static constexpr double opAmpBandwidth = 1.0e6;              // the AN6562 (an LM358 in effect)
        // The square's shaper near 768 kHz whatever the voice's rate (D290, D294: its aliases, 72 to 86 dB down, depend
        // on the shaper's own rate alone): 8 times a 96 kHz voice, 4 times a 192 kHz one.
        static constexpr double shaperRate = 768000.0;

        Tb303Oscillator()
        {
            pitch.setNewtonLimit(40);
            shaper.setNewtonLimit(40);
            pitch.setNewtonTolerance(Tb303Filter::newtonTolerance);
            shaper.setNewtonTolerance(Tb303Filter::newtonTolerance);
            buildPitch();
            buildShaper();
        }

        /** How many times faster than the voice the square's shaper runs (1 to 8), in place of shaperRate's; before
            prepare. */
        void setShaperOversampling(int factor) noexcept
        {
            shaperFactor = std::clamp(factor, 1, maxFactor);
            factorChosen = true;
        }

        void prepare(double rate) noexcept
        {
            sampleRate = rate;
            if (! factorChosen)
                shaperFactor = std::clamp(static_cast<int>(std::lround(shaperRate / rate)), 1, maxFactor);
            pitch.setSampleRate(rate);
            shaper.setSampleRate(rate * shaperFactor);
            designDecimator();
        }

        /** The trimmers as page 7 sets them (tools/tb303/calibrate.py), TUNING's turn, C11 (fitted per unit). */
        void setTrimmers(double tm4, double tm5, double tuning, double c11 = 1.0e-6) noexcept
        {
            pitch.setResistance(rTm4a, std::max(47.0e3 * std::clamp(tm4, 0.0, 1.0), 1.0));
            pitch.setResistance(rTm4b, std::max(47.0e3 * (1.0 - std::clamp(tm4, 0.0, 1.0)), 1.0));
            pitch.setResistance(rTm5, std::max(4.7e3 * std::clamp(tm5, 0.0, 1.0), 1.0));
            pitch.setResistance(rTuneA, std::max(50.0e3 * (1.0 - std::clamp(tuning, 0.0, 1.0)), 1.0));
            pitch.setResistance(rTuneB, std::max(50.0e3 * std::clamp(tuning, 0.0, 1.0), 1.0));
            shaper.setCapacitance(cC11, c11);
        }

        /** The Devil Fish's Slide Time (CoolSynth's D309): resistance in series with the ladder's 100 kOhm (its 3.3k
            protection and a 500k pot), so C35 charges slower in a slide. 0 is the stock ladder. */
        void setSlideOhms(double extra) noexcept { pitch.setResistance(rLadder, 100.0e3 + std::max(extra, 0.0)); }

        /** The slide: IC12 puts C35 on the ladder (true) or on IC11b's output (false). */
        void setSlide(bool on) noexcept
        {
            if (on == sliding)
                return;
            sliding = on;
            pitch.setResistance(rS8, on ? switchOn : switchOff);
            pitch.setResistance(rS10, on ? switchOff : switchOn);
        }

        /** At rest at this ladder voltage, C33 at `phase` of its ramp (0 the top), the square's networks settled. */
        void settle(double ladderVolts, double phase = 0.5) noexcept
        {
            setInputs(ladderVolts);
            pitch.settle();
            current = rampCurrent();
            c33 = top - std::clamp(phase, 0.0, 1.0) * (top - threshold());
            holdLeft = 0.0;
            follow(c33);
            followerAmps = 0.0;
            for (int pass = 0; pass < 4; ++pass) // the follower's current into the shaper, found with it
            {
                drive(followerOpen);
                shaper.settle();
                readFollowerCurrent();
            }
            lastSaw = shaper.probe(pSaw);
            lastSquare = shaper.probe(pSquare);
            rawSaw.fill(lastSaw);
            rawSquare.fill(lastSquare);
            fixSaw.fill(0.0);
            fixSquare.fill(0.0);
            cursor = 0;
            historySaw.fill(lastSaw);
            historySquare.fill(lastSquare);
            historyAt = 0;
        }

        /** One sample: the ladder's voltage; returns the saw (Q28's source). squareVolts() the square. */
        double process(double ladderVolts) noexcept
        {
            setInputs(ladderVolts);
            pitch.process();
            current = rampCurrent();
            const double slope = std::max(0.0, current) / timing; // volts a second, falling
            const double bottom = threshold();
            const double dt = 1.0 / (sampleRate * shaperFactor);
            for (int j = 0; j < shaperFactor; ++j)
            {
                double s = 0.0, q = 0.0;
                shape(dt, slope, bottom, s, q);
                if (shaperFactor == 1)
                {
                    lastSaw = s;
                    lastSquare = q;
                    continue;
                }
                historyAt = (historyAt + 1) % maxTaps;
                historySaw[static_cast<std::size_t>(historyAt)] = s;
                historySquare[static_cast<std::size_t>(historyAt)] = q;
            }
            if (shaperFactor > 1)
            {
                lastSaw = decimate(historySaw);
                lastSquare = decimate(historySquare);
            }
            return lastSaw;
        }

        /** False if a circuit was built past its template's limits (CircuitWdf's fitsItsLimits). */
        [[nodiscard]] bool fitsItsLimits() const noexcept { return pitch.fitsItsLimits() && shaper.fitsItsLimits(); }

        [[nodiscard]] double squareVolts() const noexcept { return lastSquare; }
        [[nodiscard]] double sawVolts() const noexcept { return lastSaw; }
        [[nodiscard]] double c33Volts() const noexcept { return c33; }
        [[nodiscard]] double cvVolts() const noexcept { return pitch.probe(pCv); }
        [[nodiscard]] double rampAmps() const noexcept { return current; }
        [[nodiscard]] int newtonSteps() const noexcept { return std::max(pitch.lastNewtonSteps(), shaper.lastNewtonSteps()); }
        [[nodiscard]] bool lastSolveConverged() const noexcept { return pitch.lastSolveConverged() && shaper.lastSolveConverged(); }

        /** The cubic B-spline step's residual (the kernel's integral less the unit step) at x samples from the step. */
        static double blepResidual(double x) noexcept
        {
            if (x <= -2.0 || x >= 2.0)
                return 0.0;
            if (x < -1.0)
                return std::pow(2.0 + x, 4) / 24.0;
            if (x < 0.0)
                return 1.0 / 24.0 + (4.0 * x - 2.0 * x * x * x - 0.75 * x * x * x * x) / 6.0 + 2.75 / 6.0;
            if (x < 1.0)
                return 0.5 + (4.0 * x - 2.0 * x * x * x + 0.75 * x * x * x * x) / 6.0 - 1.0;
            return -std::pow(2.0 - x, 4) / 24.0;
        }

    private:
        static constexpr double switchOn = 200.0, switchOff = 1.0e9;
        static constexpr double senseOhms = 1.0e3, collectorVolts = 8.0;
        static constexpr double portOhms = 600.0; // the follower's port in the shaper (1 / gm is 470 to 700 Ohm)
        static constexpr int maxFactor = 8, maxTaps = 128;

        /**
            One shaper step driven by the raw saw; on the step after a reset, the jump each output takes (less its
            slope before it) spread by the polyBLEP at the reset's time over the two steps before and the one after.
            Both outputs two steps late.
        */
        void shape(double dt, double slope, double bottom, double& sawOut, double& squareOut) noexcept
        {
            const double resetAt = advance(dt, slope, bottom);
            drive(followerOpen);
            shaper.process();
            readFollowerCurrent();
            const unsigned int now = cursor & 7u, before = (cursor + 7u) & 7u, earlier = (cursor + 6u) & 7u;
            rawSaw[now] = shaper.probe(pSaw);
            rawSquare[now] = shaper.probe(pSquare);
            if (resetAt >= 0.0)
            {
                const double jumpSaw = (rawSaw[now] - rawSaw[before]) - (rawSaw[before] - rawSaw[earlier]);
                const double jumpSquare = (rawSquare[now] - rawSquare[before]) - (rawSquare[before] - rawSquare[earlier]);
                for (int k = -2; k <= 1; ++k)
                {
                    // The reset at (this step - 1 + resetAt); the step k from this one is k - (resetAt - 1) from it.
                    const double r = blepResidual(static_cast<double>(k) - (resetAt - 1.0));
                    const unsigned int slot = (cursor + 8u + static_cast<unsigned int>(k)) & 7u;
                    fixSaw[slot] += jumpSaw * r;
                    fixSquare[slot] += jumpSquare * r;
                }
            }
            const unsigned int out = (cursor + 6u) & 7u; // two steps back
            sawOut = rawSaw[out] + fixSaw[out];
            squareOut = rawSquare[out] + fixSquare[out];
            fixSaw[out] = 0.0;
            fixSquare[out] = 0.0;
            ++cursor;
        }

        /**
            C33's ramp over one step (dt seconds), the reset an event inside it; after a reset C33 is held at the top
            for the dead time, which may run on into the next step. Returns where in the step the reset fell (0..1),
            or -1; the jump is placed at the reset's start, where C33 is held from (at its middle the polyBLEP and the
            samples disagreed for any step inside that half microsecond).
        */
        double advance(double dt, double slope, double bottom) noexcept
        {
            const double held = std::min(holdLeft, dt);
            holdLeft -= held;
            double next = c33 - slope * (dt - held);
            double resetAt = -1.0;
            if (next <= bottom && slope > 0.0)
            {
                const double crossing = held + (c33 - bottom) / slope; // seconds into the step
                const double after = dt - crossing;
                if (after >= deadTime)
                {
                    next = top - slope * (after - deadTime);
                    holdLeft = 0.0;
                }
                else
                {
                    next = top;
                    holdLeft = deadTime - after;
                }
                resetAt = crossing / dt;
            }
            c33 = next;
            follow(c33);
            return resetAt;
        }

        /**
            The shaper's outputs back to the voice's rate: a Kaiser-windowed sinc (beta 10, some 100 dB down) flat to
            40 kHz and closed from 152 kHz, so nothing folds into the audible band at 192 kHz; its length grows with
            the factor (23, 45, 89 taps at 2, 4, 8).
        */
        void designDecimator() noexcept
        {
            taps = 0;
            if (shaperFactor <= 1)
                return;
            constexpr double pi = 3.141592653589793;
            const double rate = sampleRate * shaperFactor;
            const double pass = 40.0e3, stop = std::max(pass + 1.0, sampleRate - 40.0e3);
            const double width = 2.0 * pi * (stop - pass) / rate;
            taps = std::min(maxTaps - 1, static_cast<int>(std::ceil(92.0 / (2.285 * width))) | 1);
            const double cutoff = 0.5 * (pass + stop) / rate, beta = 10.06, middle = 0.5 * (taps - 1);
            auto bessel = [](double x)
            {
                double sum = 1.0, term = 1.0;
                for (int k = 1; k < 40; ++k)
                {
                    term *= (x / (2.0 * k)) * (x / (2.0 * k));
                    sum += term;
                }
                return sum;
            };
            double total = 0.0;
            for (int i = 0; i < taps; ++i)
            {
                const double n = i - middle, r = n / std::max(1.0, middle);
                const double sinc = n == 0.0 ? 2.0 * cutoff : std::sin(2.0 * pi * cutoff * n) / (pi * n);
                const double w = bessel(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / bessel(beta);
                kernel[static_cast<std::size_t>(i)] = sinc * w;
                total += sinc * w;
            }
            for (int i = 0; i < taps; ++i)
                kernel[static_cast<std::size_t>(i)] /= total;
        }

        double decimate(const std::array<double, maxTaps>& history) const noexcept
        {
            double sum = 0.0;
            int at = historyAt;
            for (int i = 0; i < taps; ++i)
            {
                sum += kernel[static_cast<std::size_t>(i)] * history[static_cast<std::size_t>(at)];
                at = at == 0 ? maxTaps - 1 : at - 1;
            }
            return sum;
        }

        void setInputs(double ladderVolts) noexcept
        {
            pitch.setInput(inSupply, supply);
            pitch.setInput(inReference, reference);
            pitch.setInput(inLadder, ladderVolts);
            pitch.setInput(inCollector, collectorVolts);
        }

        /** Q26's right collector current, through the 1 kOhm it is read across (from the 8 V it is held at). */
        double rampCurrent() const noexcept { return (collectorVolts - pitch.probe(pCollector)) / senseOhms; }

        /**
            Where the thyristor fires, in effect: 5.349 to 5.366 V by the ramp's current, between nine notes by log
            current (D295). Each the threshold that gives vco.cir's period at its note (low C to high C by half
            octaves, 192 kHz renders): measured at three notes alone, the notes between ran 0.3 to 0.6 cents sharp.
        */
        double threshold() const noexcept
        {
            static constexpr double amps[] = { 1.8797e-6, 2.6597e-6, 3.7633e-6, 5.3248e-6, 7.5339e-6,
                                               10.6592e-6, 15.0800e-6, 21.3327e-6, 30.1748e-6 };
            static constexpr double volts[] = { 5.36543, 5.35995, 5.35580, 5.35282, 5.35085, 5.34974, 5.34940, 5.34979, 5.35098 };
            constexpr int count = 9;
            if (current <= amps[0])
                return volts[0];
            if (current >= amps[count - 1])
                return volts[count - 1];
            int k = 0;
            while (current > amps[k + 1])
                ++k;
            const double w = std::log(current / amps[k]) / std::log(amps[k + 1] / amps[k]);
            return volts[k] + w * (volts[k + 1] - volts[k]);
        }

        /** Q28's source with R105 alone: Vs / R105 = beta (Vg - Vs - Vp)^2 (1 + lambda (Vdd - Vs)), by Newton, from
            the last step's answer (the gate moves little in a step, so a step or two lands within 1e-12 V of where
            the fixed start's six or so did; the user's choice, CoolSynth's D341), or from the fixed start when that
            answer is off the channel. */
        static double followerAt(double gate, double from) noexcept
        {
            double vs = gate - from - jfetPinch > 0.0 && from >= 0.0 ? from : std::max(0.0, gate - jfetPinch - 0.03);
            for (int i = 0; i < 30; ++i)
            {
                const double vov = gate - vs - jfetPinch;
                if (vov <= 0.0)
                {
                    vs = gate - jfetPinch - 1.0e-6;
                    continue;
                }
                const double lam = 1.0 + jfetLambda * (supply - vs);
                const double id = jfetBeta * vov * vov * lam;
                const double f = vs / r105 - id;
                const double df = 1.0 / r105 + jfetBeta * (2.0 * vov * lam + vov * vov * jfetLambda);
                const double stepV = f / df;
                vs -= stepV;
                if (std::abs(stepV) < 1.0e-12)
                    break;
            }
            return vs;
        }

        /** The follower at this gate voltage: its open output and its output resistance (1 / gm against R105). */
        void follow(double gate) noexcept
        {
            followerOpen = followerAt(gate, followerOpen);
            const double vov = std::max(1.0e-6, gate - followerOpen - jfetPinch);
            const double gm = 2.0 * jfetBeta * vov * (1.0 + jfetLambda * (supply - followerOpen));
            followerOhms = 1.0 / (gm + 1.0 / r105);
        }

        /**
            The shaper's source: the follower's open voltage behind a fixed port resistance, corrected by the current it
            gave last step for the difference from its present output resistance (so the circuit is never
            re-analysed: a resistance changed each step would cost an analysis each step).
        */
        void drive(double open) noexcept
        {
            const double volts = open - (followerOhms - portOhms) * followerAmps;
            shaper.setSource(sFollower, volts);
            pendingSource = volts;
        }

        /** After a shaper solve: the current the follower gave it. */
        void readFollowerCurrent() noexcept { followerAmps = (pendingSource - shaper.probe(pSaw)) / portOhms; }

        // Pitch: 1 the 12 V rail, 2 5.333 V, 3 the ladder's open voltage, 4 Q26's right collector's 8 V (inputs);
        // 5 the ladder's output (behind its 100 kOhm), 6 C35, 7 IC11b's output (the CV), 8 its minus input, 9 its
        // inner node, 10 the reference node (Q26's left collector, IC11a's minus), 11 TM4's wiper, 12 IC11a's
        // output, 13 its inner node, 14 Q26's emitters, 15 after TM5, 16 Q26's right base, 17 TUNING's wiper, 18 Q26's
        // right collector (read across 1 kOhm).
        void buildPitch()
        {
            inSupply = pitch.addInput(1);
            inReference = pitch.addInput(2);
            inLadder = pitch.addInput(3);
            inCollector = pitch.addInput(4);
            rLadder = pitch.addResistor(3, 5, 100.0e3);                                        // the ladder's 100 kOhm
            rS8 = pitch.addResistor(5, 6, switchOff);                                          // IC12, pins 8 and 9
            rS10 = pitch.addResistor(6, 7, switchOn);                                          // IC12, pins 10 and 11
            pitch.addCapacitor(6, 0, 0.22e-6);                                                 // C35
            pitch.addOpAmp(5, 8, 7, 9, opAmpBandwidth);                                        // IC11b
            pitch.addResistor(7, 8, 2.2e3);                                                    // R107
            pitch.addResistor(1, 10, 220.0e3);                                                 // R103
            rTm4a = pitch.addResistor(2, 11, 23.5e3);                                          // TM4 from 5.333 V
            rTm4b = pitch.addResistor(11, 1, 23.5e3);                                          // and to 12 V
            pitch.addResistor(11, 10, 100.0e3);                                                // R102
            pitch.addOpAmp(2, 10, 12, 13, opAmpBandwidth);                                     // IC11a
            pitch.addCapacitor(12, 10, 1.0e-9);                                                // C34
            pitch.addResistor(12, 14, 2.2e3);                                                  // R104
            pitch.addTransistor(10, 2, 14, dualSaturation, dualBetaF, dualBetaR, thermal);    // Q26, left
            pitch.addTransistor(18, 16, 14, dualSaturation, dualBetaF, dualBetaR, thermal);   // Q26, right
            pitch.addResistor(18, 4, senseOhms);                                               // where its current is read
            rTm5 = pitch.addResistor(7, 15, 3.5e3);                                            // TM5
            pitch.addResistor(15, 16, 27.0e3);                                                 // R106
            pitch.addResistor(16, 2, 560.0);                                                   // R100, the posistor
            pitch.addResistor(16, 17, 100.0e3);                                                // R118
            rTuneA = pitch.addResistor(2, 17, 25.0e3);                                         // TUNING from 5.333 V
            rTuneB = pitch.addResistor(17, 0, 25.0e3);                                         // and to ground
            pCv = pitch.addProbe(7);
            pCollector = pitch.addProbe(18);
        }

        // Shaper: 1 the 12 V rail, 2 5.333 V (inputs); 3 the follower's output (behind its port), 4 Q8's base, 5 C10's
        // far side, 6 Q8's emitter, 7 its collector (the square).
        void buildShaper()
        {
            shaperSupply = shaper.addInput(1);
            shaperReference = shaper.addInput(2);
            sFollower = shaper.addSource(3, 0, portOhms);                                      // Q28 as its Thevenin
            shaper.addResistor(3, 0, 1.0e9);                                                   // (keeps node 3 defined)
            shaper.addResistor(3, 4, 100.0e3);                                                 // R35
            shaper.addCapacitor(3, 5, 0.01e-6);                                                // C10
            shaper.addResistor(5, 4, 10.0e3);                                                  // R34
            shaper.addResistor(1, 6, 22.0e3);                                                  // R45
            cC11 = shaper.addCapacitor(6, 0, 1.0e-6);                                          // C11
            shaper.addPnpTransistor(7, 4, 6, Tb303Filter::pnpSaturation, Tb303Filter::pnpBetaF, Tb303Filter::pnpBetaR, thermal,
                                    Tb303Filter::pnpEarly);                                    // Q8
            shaper.addResistor(7, 2, 10.0e3);                                                  // R36
            pSquare = shaper.addProbe(7);
            pSaw = shaper.addProbe(3);
            shaper.setInput(shaperSupply, supply);
            shaper.setInput(shaperReference, reference);
        }

        wdf::Circuit<18, 26, 4, 2, 4, 2, true> pitch; // 26 ports: 18 parts, Q26's 4, the two op-amps' 2 each
        wdf::Circuit<7, 10, 2, 0, 2, 2, true> shaper; // 10 ports: 8 parts, Q8's 2

        double sampleRate = 192000.0;
        double current = 5.0e-6, c33 = 8.0, followerOpen = 8.0, followerOhms = 700.0;
        double holdLeft = 0.0, followerAmps = 0.0, pendingSource = 8.0, lastSaw = 8.0, lastSquare = 5.333;
        bool sliding = false;
        std::array<double, 8> rawSaw {}, rawSquare {}, fixSaw {}, fixSquare {};
        unsigned int cursor = 0;
        int shaperFactor = 8, taps = 0, historyAt = 0;
        bool factorChosen = false;
        std::array<double, maxTaps> kernel {}, historySaw {}, historySquare {};
        int inSupply = 0, inReference = 0, inLadder = 0, inCollector = 0, rS8 = 0, rS10 = 0;
        int rTm4a = 0, rTm4b = 0, rTm5 = 0, rTuneA = 0, rTuneB = 0, pCv = 0, pCollector = 0, rLadder = 0;
        int shaperSupply = 0, shaperReference = 0, sFollower = 0, cC11 = 0, pSquare = 0, pSaw = 0;
    };
}
