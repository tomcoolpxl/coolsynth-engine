#include <juce_core/juce_core.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <vector>

#include "synth/v3/Tb303Output.h"
#include "synth/v3/Tb303Voice.h"

/*
    The TB-303's output stage (CoolSynth's D362): VOLUME into the mixer (Q34, Q33), out at the OUTPUT jack into a
    recorder, against ngspice renders of tools/tb303/out.cir and voice.cir (tests/data/tb303): the stage alone on a
    sine that clips on the mixer's lower side, and the whole voice with every knob full on an accent, where Din Sync's
    recordings of a real unit clip. A class of its own: the circuit tests' runTest is long enough (MSVC Debug frames).
*/
namespace
{
    using coolsynth::synth::v3::Tb303Output;
    using coolsynth::synth::v3::Tb303Voice;

    struct Reference
    {
        std::map<juce::String, std::vector<double>> columns;
        double rate = 96000.0;
        std::size_t size() const { return columns.empty() ? 0 : columns.begin()->second.size(); }
    };

    Reference loadReference(const juce::File& file)
    {
        Reference r;
        juce::StringArray lines;
        file.readLines(lines);
        if (lines.size() < 3)
            return r;
        const auto names = juce::StringArray::fromTokens(lines[0], ",", "");
        double t0 = 0.0, t1 = 0.0;
        for (int i = 1; i < lines.size(); ++i)
        {
            const auto f = juce::StringArray::fromTokens(lines[i], ",", "");
            if (f.size() < names.size())
                continue;
            if (i == 1)
                t0 = f[0].getDoubleValue();
            if (i == 2)
                t1 = f[0].getDoubleValue();
            for (int c = 1; c < names.size(); ++c)
                r.columns[names[c]].push_back(f[c].getDoubleValue());
        }
        if (t1 > t0)
            r.rate = 1.0 / (t1 - t0);
        return r;
    }

    juce::File referenceFolder()
    {
        return juce::File(__FILE__).getParentDirectory().getChildFile("data").getChildFile("tb303");
    }

    /** Loudness in 5 ms blocks where the reference's is within 40 dB of its loudest: the rms of the differences. */
    double blockLoudness(const std::vector<double>& model, const std::vector<double>& spice, double rate, double& worst)
    {
        const auto block = static_cast<std::size_t>(0.005 * rate);
        const std::size_t n = std::min(model.size(), spice.size());
        std::vector<double> lm, ls;
        for (std::size_t s = 0; s + block <= n; s += block)
        {
            double p = 0.0, q = 0.0;
            for (std::size_t i = s; i < s + block; ++i)
            {
                p += model[i] * model[i];
                q += spice[i] * spice[i];
            }
            lm.push_back(10.0 * std::log10(std::max(1.0e-30, p / static_cast<double>(block))));
            ls.push_back(10.0 * std::log10(std::max(1.0e-30, q / static_cast<double>(block))));
        }
        const double loudest = *std::max_element(ls.begin(), ls.end());
        double sum = 0.0;
        int counted = 0;
        worst = 0.0;
        for (std::size_t j = 0; j < ls.size(); ++j)
            if (ls[j] > loudest - 40.0)
            {
                const double d = lm[j] - ls[j];
                sum += d * d;
                ++counted;
                worst = std::max(worst, std::abs(d));
            }
        return std::sqrt(sum / std::max(1, counted));
    }

    double lowest(const std::vector<double>& x, std::size_t from, std::size_t to)
    {
        return *std::min_element(x.begin() + static_cast<std::ptrdiff_t>(from), x.begin() + static_cast<std::ptrdiff_t>(to));
    }
}

class V3Tb303OutputTests final : public juce::UnitTest
{
public:
    V3Tb303OutputTests() : juce::UnitTest("TB-303 output stage", "V3") {}

    void runTest() override
    {
        beginTest("the_stage_matches_spice");
        {
            // out.cir alone: a 1.5 V sine at low C on pin 8, VOLUME full, into 16k: the mixer clips on its lower side.
            const auto file = referenceFolder().getChildFile("out_clip.csv");
            expect(file.existsAsFile(), "the SPICE reference out_clip.csv");
            const auto ref = loadReference(file);
            const auto& jack = ref.columns.at("njack");
            const auto& mixer = ref.columns.at("nmix");
            auto s = std::make_unique<Tb303Output>();
            expect(s->fitsItsLimits(), "the stage fits its template");
            expect(s->isWellPosed());
            s->prepare(ref.rate);
            s->settle(5.3);
            double worstJack = 0.0, worstMixer = 0.0, bottom = 1.0e9, top = -1.0e9;
            bool converged = true;
            for (std::size_t i = 0; i < ref.size(); ++i)
            {
                const double t = static_cast<double>(i) / ref.rate;
                const double y = s->process(5.3 + 1.5 * std::sin(juce::MathConstants<double>::twoPi * 65.406 * t));
                converged = converged && s->lastSolveConverged();
                if (t < 0.05)
                    continue; // the couplings settling from the sine's start
                worstJack = std::max(worstJack, std::abs(y - jack[i]));
                worstMixer = std::max(worstMixer, std::abs(s->mixer() - mixer[i]));
                bottom = std::min(bottom, s->mixer());
                top = std::max(top, s->mixer());
            }
            expect(converged, "every sample converged");
            // Within 2 mV of out.cir on a 4.5 V swing (D362): 5 mV.
            expectLessThan(worstJack, 5.0e-3, "the jack");
            expectLessThan(worstMixer, 5.0e-3, "the mixer's output");
            // From its rest (4.14 V) the sine's 3.2 V upper half goes through (to 7.07 V), its lower half stops some
            // 2.15 V down (D362).
            const double rest = mixer.front();
            expectGreaterThan(top - rest, 2.7, "it rises freely");
            expectLessThan(rest - bottom, 2.4, "it falls only so far");
            logMessage("the stage against out.cir: the jack within " + juce::String(worstJack * 1.0e3, 2) + " mV, the mixer within "
                       + juce::String(worstMixer * 1.0e3, 2) + " mV; the mixer from " + juce::String(bottom, 2) + " to "
                       + juce::String(top, 2) + " V");
        }

        beginTest("the_jack_matches_spice");
        {
            // voice.cir at its jack, every knob full (RESONANCE's share 0.96), an accented saw at low C, the gate from
            // 50 to 300 ms: Din Sync's loudest notes, which the mixer clips.
            const auto file = referenceFolder().getChildFile("voice_jack.csv");
            expect(file.existsAsFile(), "the SPICE reference voice_jack.csv");
            const auto ref = loadReference(file);
            const auto& spiceJack = ref.columns.at("njack");
            const auto& spiceVca = ref.columns.at("nout");
            Tb303Voice::Knobs k;
            k.cutoff = k.resonance = k.envMod = k.decay = k.accent = 1.0;
            auto voice = std::make_unique<Tb303Voice>();
            voice->prepare(ref.rate);
            voice->setKnobs(k);
            voice->settle(2.0);
            const std::size_t n = ref.size();
            std::vector<double> jack(n), vcaOut(n);
            bool converged = true;
            for (std::size_t i = 0; i < n; ++i)
            {
                const double t = static_cast<double>(i) / ref.rate;
                const bool gate = t > 0.05 && t < 0.3;
                jack[i] = voice->process(2.0, gate, gate, false);
                vcaOut[i] = voice->vcaOutput();
                converged = converged && voice->lastSolveConverged();
            }
            expect(converged, "every sample converged");
            double worstJack = 0.0, worstVca = 0.0;
            const double jackRms = blockLoudness(jack, spiceJack, ref.rate, worstJack);
            const double vcaRms = blockLoudness(vcaOut, spiceVca, ref.rate, worstVca);
            // The VCA's output as the voice test finds it (0.35 to 0.7 dB rms); the stage adds little to it.
            expectLessThan(vcaRms, 1.0, "the VCA's output");
            expectLessThan(jackRms, 1.0, "the jack");
            // The clip: the jack's lowest over the note's first 80 ms within 0.1 V of voice.cir's.
            const auto a = static_cast<std::size_t>(0.06 * ref.rate), b = static_cast<std::size_t>(0.13 * ref.rate);
            const double floorModel = lowest(jack, a, b), floorSpice = lowest(spiceJack, a, b);
            expectLessThan(std::abs(floorModel - floorSpice), 0.1, "the floor it clips at");
            logMessage("the voice's jack against voice.cir: " + juce::String(jackRms, 2) + " dB rms (worst " + juce::String(worstJack, 2)
                       + "), the VCA's output " + juce::String(vcaRms, 2) + " dB rms; the floor " + juce::String(floorModel, 3) + " V against "
                       + juce::String(floorSpice, 3) + " V");
        }

        beginTest("volume_and_polarity");
        {
            // VOLUME at 0 silences the jack; the jack is the VCA's output turned over. (The clip: the tests above.)
            Tb303Voice::Knobs k;
            k.cutoff = k.resonance = k.envMod = k.decay = k.accent = 1.0;
            double level[2] = {};
            double correlation = 0.0, down = 0.0, up = 0.0;
            for (int pass = 0; pass < 2; ++pass)
            {
                k.volume = pass == 0 ? 0.0 : 1.0;
                auto voice = std::make_unique<Tb303Voice>();
                voice->prepare(Tb303Voice::rate);
                voice->setKnobs(k);
                voice->settle(2.0);
                double s = 0.0, c = 0.0, v2 = 0.0, j2 = 0.0;
                const auto n = static_cast<std::size_t>(0.2 * Tb303Voice::rate);
                for (std::size_t i = 0; i < n; ++i)
                {
                    const bool gate = i > static_cast<std::size_t>(0.02 * Tb303Voice::rate);
                    const double y = voice->process(2.0, gate, gate, false);
                    s += y * y;
                    if (pass == 1 && gate)
                    {
                        const double v = voice->vcaOutput();
                        c += y * v;
                        v2 += v * v;
                        j2 += y * y;
                        down = std::min(down, y);
                        up = std::max(up, y);
                    }
                }
                level[pass] = 10.0 * std::log10(std::max(1.0e-30, s / static_cast<double>(n)));
                if (pass == 1)
                    correlation = c / std::sqrt(std::max(1.0e-30, v2 * j2));
            }
            expectLessThan(level[0], level[1] - 80.0, "silent at VOLUME 0");
            expectLessThan(correlation, -0.8, "turned over against the VCA's output");
            logMessage("VOLUME 0 " + juce::String(level[0] - level[1], 1) + " dB under full; correlation with the VCA "
                       + juce::String(correlation, 3) + "; the jack from " + juce::String(down, 2) + " to " + juce::String(up, 2) + " V");
        }
    }
};

static V3Tb303OutputTests v3Tb303OutputTests;
