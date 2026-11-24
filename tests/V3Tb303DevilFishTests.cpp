#include <juce_core/juce_core.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "synth/v3/Tb303Envelope.h"
#include "synth/v3/Tb303Voice.h"

/*
    The Devil Fish (CoolSynth's D309): Robin Whittle's modifications to the TB-303 as changes to the engine's circuit
    (part values, a current into the filter's control, a sustain input on the volume envelope) and, where he never
    published his circuit, simple stand-ins around it (Soft Attack, Muffler). Each control measured against the times
    and ranges of his manual (2019); the stock voice is left as it was, which the SPICE tests ("TB-303 circuit") hold.
*/
namespace
{
    using coolsynth::synth::v3::Tb303Envelope;
    using coolsynth::synth::v3::Tb303Voice;

    constexpr double rate = 96000.0;

    struct Step
    {
        double note = 2.0;
        bool gate = false, accent = false, slide = false;
    };

    struct Run
    {
        std::vector<double> out, cv, ladder;
        bool converged = true;
    };

    std::unique_ptr<Tb303Voice> makeVoice(bool fish, const Tb303Voice::Knobs& knobs, const Tb303Voice::DevilFish& f = {})
    {
        auto v = std::make_unique<Tb303Voice>(fish);
        v->prepare(rate);
        v->setKnobs(knobs);
        v->setDevilFish(f);
        v->settle(2.0);
        return v;
    }

    Run render(Tb303Voice& v, double seconds, const std::function<Step(double)>& at)
    {
        Run r;
        const auto n = static_cast<std::size_t>(seconds * rate);
        r.out.resize(n);
        r.cv.resize(n);
        r.ladder.resize(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            const auto s = at(static_cast<double>(i) / rate);
            r.out[i] = v.process(s.note, s.gate, s.accent, s.slide);
            r.cv[i] = v.oscillator().cvVolts();
            r.ladder[i] = v.vcf().ladderCurrent();
            r.converged = r.converged && v.lastSolveConverged();
        }
        return r;
    }

    double rms(const std::vector<double>& x, double from, double to)
    {
        const auto a = static_cast<std::size_t>(from * rate), b = std::min(x.size(), static_cast<std::size_t>(to * rate));
        double s = 0.0;
        for (std::size_t i = a; i < b; ++i)
            s += x[i] * x[i];
        return std::sqrt(s / static_cast<double>(std::max<std::size_t>(1, b - a)));
    }

    double db(double x) { return 20.0 * std::log10(std::max(x, 1.0e-12)); }

    /** The main envelope's decay: from its peak to 1/e of the way back to where it rests, seconds. */
    double envelopeDecay(double normalOhms, double accentOhms, bool accent, bool fish, double stockDecay = 0.0)
    {
        auto env = std::make_unique<Tb303Envelope>(fish);
        env->prepare(rate);
        env->setKnobs(stockDecay, 0.5);
        if (fish)
            env->setDevilFish(normalOhms, accentOhms, 1.5e6, Tb303Envelope::supply, true);
        env->setAccent(false);
        env->settle();
        const auto n = static_cast<std::size_t>(5.0 * rate);
        std::vector<double> e(n);
        const double rest = env->envelope();
        for (std::size_t i = 0; i < n; ++i)
        {
            const double t = static_cast<double>(i) / rate;
            env->setAccent(accent && t > 0.01);
            env->process(t > 0.01 && t < 0.2);
            e[i] = env->envelope();
        }
        const auto peakAt = static_cast<std::size_t>(std::max_element(e.begin(), e.end()) - e.begin());
        const double peak = e[peakAt], target = rest + (peak - rest) / std::exp(1.0);
        for (std::size_t i = peakAt; i < n; ++i)
            if (e[i] <= target)
                return static_cast<double>(i - peakAt) / rate;
        return 5.0;
    }
}

class V3Tb303DevilFishTests final : public juce::UnitTest
{
public:
    V3Tb303DevilFishTests() : juce::UnitTest("TB-303 Devil Fish", "V3") {}

    void runTest() override
    {
        beginTest("builds_and_plays");
        {
            auto v = makeVoice(true, {});
            expect(v->isDevilFish());
            expect(v->fitsItsLimits(), "a Devil Fish's circuits fit their templates");
            const auto r = render(*v, 0.5, [](double t) { return Step { 2.0, t > 0.05 && t < 0.3, false, false }; });
            expect(r.converged, "every sample converged");
            const double level = db(rms(r.out, 0.05, 0.3));
            const auto stock = makeVoice(false, {});
            const auto s = render(*stock, 0.5, [](double t) { return Step { 2.0, t > 0.05 && t < 0.3, false, false }; });
            const double stockLevel = db(rms(s.out, 0.05, 0.3));
            logMessage("at its stock-like settings " + juce::String(level, 1) + " dB, the stock voice "
                       + juce::String(stockLevel, 1) + " dB");
            expectLessThan(std::abs(level - stockLevel), 6.0, "near the stock voice's loudness");
        }

        beginTest("overdrive");
        {
            double levels[3] {};
            const double turns[] = { 0.0, 0.2, 1.0 };
            for (int k = 0; k < 3; ++k)
            {
                Tb303Voice::DevilFish f;
                f.overdrive = turns[k];
                auto v = makeVoice(true, {}, f);
                const auto r = render(*v, 0.3, [](double t) { return Step { 2.0, t > 0.02, false, false }; });
                expect(r.converged);
                levels[k] = db(rms(r.out, 0.05, 0.25));
            }
            logMessage("Overdrive 0, 0.2, 1: " + juce::String(levels[0], 1) + ", " + juce::String(levels[1], 1) + ", "
                       + juce::String(levels[2], 1) + " dB");
            expectLessThan(levels[0], levels[1] - 30.0, "none into the filter at 0");
            expectGreaterThan(levels[2], levels[1] + 3.0, "louder and driven at full");
        }

        beginTest("slide_time");
        {
            // The CV from 2 V to 3 V in a slide: the time to 95 % of the way.
            auto slideTime = [this](double turn)
            {
                Tb303Voice::DevilFish f;
                f.slideTime = turn;
                auto v = makeVoice(true, {}, f);
                const auto r = render(*v, 1.5, [](double t) { return Step { t < 0.1 ? 2.0 : 3.0, true, false, t >= 0.1 }; });
                expect(r.converged);
                for (std::size_t i = static_cast<std::size_t>(0.1 * rate); i < r.cv.size(); ++i)
                    if (r.cv[i] >= 2.95)
                        return static_cast<double>(i) / rate - 0.1;
                return 1.4;
            };
            const double shortest = slideTime(0.0), longest = slideTime(1.0);
            logMessage("Slide Time 0: " + juce::String(shortest * 1.0e3, 0) + " ms, 1: " + juce::String(longest * 1.0e3, 0)
                       + " ms to 95 %");
            expectGreaterThan(longest / shortest, 4.5, "some six times longer at full (603k over 103k)");
            expectLessThan(longest / shortest, 7.0);
        }

        beginTest("normal_and_accent_decay");
        {
            const double stockShort = envelopeDecay(0, 0, false, false, 0.0), stockLong = envelopeDecay(0, 0, false, false, 1.0);
            const double r0 = Tb303Voice::decayOhms(0.0) - Tb303Envelope::fishR138;
            const double r1 = Tb303Voice::decayOhms(1.0) - Tb303Envelope::fishR138;
            const double fishShort = envelopeDecay(r0, r1, false, true), fishLong = envelopeDecay(r1, r0, false, true);
            const double accentShort = envelopeDecay(r1, r0, true, true);
            logMessage("the stock main envelope: Decay 0 " + juce::String(stockShort * 1.0e3, 0) + " ms, 1 "
                       + juce::String(stockLong * 1.0e3, 0) + " ms; the Devil Fish's: 0 " + juce::String(fishShort * 1.0e3, 0)
                       + " ms, 1 " + juce::String(fishLong * 1.0e3, 0) + " ms; an accent at Accent Decay 0 "
                       + juce::String(accentShort * 1.0e3, 0) + " ms");
            // Whittle's ranges: the stock 200 ms to 2 s; the Devil Fish's 30 ms to 3 s (the same measure of each).
            expectGreaterThan(fishLong / fishShort, 40.0, "about a hundredfold");
            expectLessThan(fishLong / fishShort, 250.0);
            expectLessThan(fishShort, stockShort * 0.3, "shorter than the stock's shortest");
            expectGreaterThan(fishLong, stockLong * 1.2, "longer than the stock's longest");
            expectLessThan(std::abs(accentShort - fishShort) / fishShort, 0.2, "an accent takes Accent Decay");
        }

        beginTest("volume_envelope");
        {
            // DECAY on the volume envelope: short at 0, the stock's at half, held at full.
            double at1s[3] {}, early[3] {};
            const double turns[] = { 0.0, 0.5, 1.0 };
            for (int k = 0; k < 3; ++k)
            {
                Tb303Voice::Knobs knobs;
                knobs.decay = turns[k];
                Tb303Voice::DevilFish f;
                f.normalDecay = 1.0; // the filter open long, so the volume shows
                auto v = makeVoice(true, knobs, f);
                const auto r = render(*v, 1.3, [](double t) { return Step { 2.0, t > 0.02, false, false }; });
                expect(r.converged);
                early[k] = db(rms(r.out, 0.03, 0.06));
                at1s[k] = db(rms(r.out, 1.1, 1.2));
            }
            logMessage("DECAY 0, 0.5, 1: early " + juce::String(early[0], 1) + ", " + juce::String(early[1], 1) + ", "
                       + juce::String(early[2], 1) + " dB; at 1.1 s " + juce::String(at1s[0], 1) + ", "
                       + juce::String(at1s[1], 1) + ", " + juce::String(at1s[2], 1) + " dB");
            expectLessThan(at1s[0], at1s[1] - 30.0, "gone by a second at 0");
            expectGreaterThan(at1s[2], at1s[1] + 1.0, "held at full");
            expectGreaterThan(at1s[2], early[2] - 6.0, "held near its start");
        }

        beginTest("filter_tracking_and_ranges");
        {
            // The ladder's current (the cutoff) at rest: an octave doubles it.
            auto current = [this](bool fish, double note, double tracking, double cutoff)
            {
                Tb303Voice::Knobs knobs;
                knobs.cutoff = cutoff;
                Tb303Voice::DevilFish f;
                f.tracking = tracking;
                auto v = makeVoice(fish, knobs, f);
                v->settle(note);
                const auto r = render(*v, 0.1, [note](double) { return Step { note, false, false, false }; });
                expect(r.converged);
                return r.ladder.back();
            };
            const double octaves = std::log2(current(true, 3.0, 1.0, 0.5) / current(true, 2.0, 1.0, 0.5));
            const double still = std::log2(current(true, 3.0, 0.0, 0.5) / current(true, 2.0, 0.0, 0.5));
            logMessage("Filter Tracking at full: " + juce::String(octaves, 2) + " octaves a volt; at 0: "
                       + juce::String(still, 3));
            expectGreaterThan(octaves, 1.1, "some 1.5 octaves a volt at full");
            expectLessThan(octaves, 1.9);
            expectLessThan(std::abs(still), 0.02, "none at 0");
            const double top = std::log2(current(true, 2.0, 0.0, 1.0) / current(false, 2.0, 0.0, 1.0));
            const double bottom = std::log2(current(true, 2.0, 0.0, 0.0) / current(false, 2.0, 0.0, 0.0));
            logMessage("Cut Off against the stock: " + juce::String(top, 2) + " octaves at full, "
                       + juce::String(bottom, 2) + " at 0");
            expectGreaterThan(top, 0.75, "about an octave higher at the top (Whittle's 5 kHz against 2.5)");
            expectLessThan(top, 1.25);
            expectLessThan(bottom, -1.0, "much lower at the bottom");
        }

        beginTest("high_resonance_oscillates");
        {
            // No oscillator into the filter (Overdrive 0), RESONANCE full: only the high setting rings.
            double levels[2] {};
            for (int k = 0; k < 2; ++k)
            {
                Tb303Voice::Knobs knobs;
                knobs.resonance = 1.0;
                knobs.cutoff = 0.7;
                Tb303Voice::DevilFish f;
                f.overdrive = 0.0;
                f.accentSweep = k == 0 ? 2 : 1;
                auto v = makeVoice(true, knobs, f);
                const auto r = render(*v, 0.6, [](double t) { return Step { 2.0, t > 0.02, false, false }; });
                expect(r.converged);
                levels[k] = db(rms(r.out, 0.4, 0.6));
            }
            logMessage("RESONANCE full, nothing in: stock resonance " + juce::String(levels[0], 1) + " dB, high "
                       + juce::String(levels[1], 1) + " dB");
            expectGreaterThan(levels[1], levels[0] + 20.0, "the high setting oscillates on its own");
        }

        beginTest("accents_fm_and_muffler");
        {
            // A run of accents under each Sweep Speed and with the sweep off (judged by the filter's control, the
            // ladder's current, at its highest), Filter FM, and the Muffler: all converge and each changes the sound.
            auto accents = [](double t)
            {
                const double step = std::fmod(t, 0.125);
                return Step { 2.0, step < 0.06, true, false };
            };
            Tb303Voice::Knobs knobs; // as Whittle hears the modes: Env Mod down, the accent up
            knobs.resonance = 1.0;
            knobs.envMod = 0.0;
            knobs.accent = 1.0;
            std::vector<double> levels;
            juce::String line = "the cutoff's peak over four accents, octaves over the sweep off's, at Sweep Speed fast, normal, slow:";
            for (int mode = 0; mode < 4; ++mode)
            {
                Tb303Voice::DevilFish f;
                f.sweepSpeed = mode < 3 ? mode : 1;
                f.accentSweep = mode < 3 ? 2 : 0;
                auto v = makeVoice(true, knobs, f);
                const auto r = render(*v, 0.5, accents);
                expect(r.converged);
                levels.push_back(std::log2(*std::max_element(r.ladder.begin(), r.ladder.end())));
            }
            for (int mode = 0; mode < 3; ++mode)
                line << " " << juce::String(levels[static_cast<size_t>(mode)] - levels[3], 2);
            logMessage(line);
            expectGreaterThan(levels[1], levels[3] + 0.3, "the accent sweep opens the filter; off, it does not");
            expectGreaterThan(std::abs(levels[0] - levels[2]), 0.1, "the sweep speeds differ");

            Tb303Voice::DevilFish loud;
            loud.overdrive = 1.0;
            auto plain = makeVoice(true, {}, loud);
            const auto p = render(*plain, 0.3, [](double t) { return Step { 2.0, t > 0.02, true, false }; });
            loud.muffler = 2;
            auto muffled = makeVoice(true, {}, loud);
            const auto m = render(*muffled, 0.3, [](double t) { return Step { 2.0, t > 0.02, true, false }; });
            loud.muffler = 0;
            loud.filterFm = 0.8;
            auto fm = makeVoice(true, {}, loud);
            const auto q = render(*fm, 0.3, [](double t) { return Step { 2.0, t > 0.02, true, false }; });
            expect(m.converged && q.converged && p.converged);
            const double peakPlain = *std::max_element(p.out.begin(), p.out.end());
            const double peakMuffled = *std::max_element(m.out.begin(), m.out.end());
            double differs = 0.0;
            for (std::size_t i = 0; i < p.out.size(); ++i)
                differs = std::max(differs, std::abs(p.out[i] - q.out[i]));
            logMessage("overdriven accent: peak " + juce::String(peakPlain, 3) + " V, muffled " + juce::String(peakMuffled, 3)
                       + " V; Filter FM moves the output by up to " + juce::String(differs, 3) + " V");
            expectLessThan(peakMuffled, peakPlain * 0.8, "the Muffler softens the loudest");
            expectGreaterThan(differs, 0.01, "Filter FM changes the sound");
        }

        beginTest("soft_attack");
        {
            double first[2] {};
            for (int k = 0; k < 2; ++k)
            {
                Tb303Voice::DevilFish f;
                f.softAttack = k == 0 ? 0.0 : 1.0;
                auto v = makeVoice(true, {}, f);
                const auto r = render(*v, 0.2, [](double t) { return Step { 2.0, t > 0.02, false, false }; });
                expect(r.converged);
                first[k] = db(rms(r.out, 0.025, 0.035));
            }
            logMessage("the first 10 ms after the note's start: Soft Attack 0 " + juce::String(first[0], 1) + " dB, 1 "
                       + juce::String(first[1], 1) + " dB");
            expectLessThan(first[1], first[0] - 3.0, "a slow attack at full");
        }
    }
};

static V3Tb303DevilFishTests v3Tb303DevilFishTests;
