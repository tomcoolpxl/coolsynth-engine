#include <juce_core/juce_core.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <memory>
#include <vector>

#include "synth/v3/CircuitWatch.h"
#include "synth/v3/EngineV3.h"
#include "synth/v3/Part.h"
#include "synth/v3/Tr808RimShot.h"
#include "synth/v3/Tr808Tom.h"

/*
    The 808 Drums' circuits (D252) against ngspice renders of the same schematics (tools/tr808/tom.cir and
    rimclave.cir, rendered by tools/tr808/render.py; the references are in tests/data/tr808): each stage's node,
    sample by sample, over the first 30 ms of a hit.
*/
namespace
{
    using coolsynth::synth::v3::Tr808RimShot;
    using coolsynth::synth::v3::Tr808Tom;

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
        const auto env = juce::SystemStats::getEnvironmentVariable("COOLSYNTH_808_REFERENCE", {});
        if (env.isNotEmpty())
            return juce::File(env);
        return juce::File(__FILE__).getParentDirectory().getChildFile("data").getChildFile("tr808");
    }

    /** The rms difference over the rms of the reference, from sample `from` on. */
    double relativeError(const std::vector<double>& model, const std::vector<double>& spice, std::size_t from)
    {
        double e = 0.0, s = 0.0;
        const std::size_t n = std::min(model.size(), spice.size());
        for (std::size_t i = from; i < n; ++i)
        {
            e += (model[i] - spice[i]) * (model[i] - spice[i]);
            s += spice[i] * spice[i];
        }
        return std::sqrt(e / std::max(1.0e-30, s));
    }

    struct TomCase
    {
        const char* file;
        Tr808Tom::Pair pair;
        bool conga;
        double volts, noise;
    };

    struct RimCase
    {
        const char* file;
        bool claves;
        double volts;
    };
}

class V3Tr808DrumsTests final : public juce::UnitTest
{
public:
    V3Tr808DrumsTests() : juce::UnitTest("V3 TR-808 drums circuits", "V3") {}

    void runTest() override
    {
        beginTest("the_toms_and_congas_match_spice_stage_by_stage");
        {
            const TomCase cases[] = {
                { "lt.csv", Tr808Tom::Pair::low, false, 4.0, 0.02 },  { "lc.csv", Tr808Tom::Pair::low, true, 4.0, 0.0 },
                { "mt.csv", Tr808Tom::Pair::mid, false, 4.0, 0.02 },  { "mc.csv", Tr808Tom::Pair::mid, true, 4.0, 0.0 },
                { "ht.csv", Tr808Tom::Pair::high, false, 4.0, 0.02 }, { "hc.csv", Tr808Tom::Pair::high, true, 4.0, 0.0 },
                { "lt14.csv", Tr808Tom::Pair::low, false, 14.0, 0.02 },
            };
            for (const auto& c : cases)
            {
                const auto file = referenceFolder().getChildFile(c.file);
                if (! file.existsAsFile())
                {
                    expect(false, juce::String("the SPICE reference is missing: ") + c.file);
                    continue;
                }
                const auto ref = loadReference(file);
                auto tom = std::make_unique<Tr808Tom>(c.pair);
                tom->prepare(ref.rate);
                tom->setConga(c.conga);
                tom->setTuning(0.5);
                tom->setLevel(0.8);
                tom->settle();
                tom->trigger(c.volts);
                const std::size_t n = ref.size();
                std::map<juce::String, std::vector<double>> model;
                int worstNewton = 0;
                bool converged = true;
                for (std::size_t i = 0; i < n; ++i)
                {
                    const double t = static_cast<double>(i) / ref.rate;
                    const double out = tom->process(c.noise * std::sin(2.0 * juce::MathConstants<double>::pi * 150.0 * t));
                    model["vout"].push_back(out);
                    model["vt"].push_back(tom->triggerNodeVolts());
                    model["np"].push_back(tom->pulseVolts());
                    model["nev"].push_back(tom->envelopeVolts());
                    model["nq"].push_back(tom->gateBaseVolts());
                    model["nfl"].push_back(tom->noiseVolts());
                    model["nm"].push_back(tom->middleVolts());
                    model["no2"].push_back(tom->resonatorVolts());
                    worstNewton = std::max(worstNewton, tom->newtonSteps());
                    converged = converged && tom->lastSolveConverged();
                }
                juce::String line = juce::String(c.file) + ":";
                for (const auto* name : { "vt", "np", "nev", "nq", "nfl", "nm", "no2", "vout" })
                {
                    const double err = relativeError(model[name], ref.columns.at(name), 0);
                    line << " " << name << " " << juce::String(err, 4);
                    // The noise only reaches the mixer for the toms; the congas' gate output is all but nothing.
                    if (juce::String(name) == "nfl" && c.conga)
                        continue;
                    expectLessThan(err, 0.05, juce::String(c.file) + " " + name);
                }
                line << " | most Newton steps " << worstNewton;
                logMessage(line);
                expect(converged, juce::String(c.file) + ": every sample converged");
            }
        }

        beginTest("the_rim_shot_and_claves_match_spice_stage_by_stage");
        {
            const RimCase cases[] = { { "rs.csv", false, 4.0 }, { "cl.csv", true, 4.0 }, { "rs14.csv", false, 14.0 } };
            for (const auto& c : cases)
            {
                const auto file = referenceFolder().getChildFile(c.file);
                if (! file.existsAsFile())
                {
                    expect(false, juce::String("the SPICE reference is missing: ") + c.file);
                    continue;
                }
                const auto ref = loadReference(file);
                auto drum = std::make_unique<Tr808RimShot>();
                drum->prepare(ref.rate);
                drum->setClaves(c.claves);
                drum->setLevel(0.8);
                drum->settle();
                drum->trigger(c.volts);
                const std::size_t n = ref.size();
                std::map<juce::String, std::vector<double>> model;
                int worstNewton = 0;
                bool converged = true;
                for (std::size_t i = 0; i < n; ++i)
                {
                    model["vout"].push_back(drum->process());
                    model["vt"].push_back(drum->triggerNodeVolts());
                    model["nenv"].push_back(drum->envelopeVolts());
                    model["no21"].push_back(drum->lowVolts());
                    model["no20b"].push_back(drum->highVolts());
                    model["ns"].push_back(drum->swingVolts());
                    model["nf"].push_back(drum->collectorVolts());
                    model["ne63"].push_back(drum->followerVolts());
                    worstNewton = std::max(worstNewton, drum->newtonSteps());
                    converged = converged && drum->lastSolveConverged();
                }
                juce::String line = juce::String(c.file) + ":";
                for (const auto* name : { "vt", "nenv", "no21", "no20b", "ns", "nf", "ne63", "vout" })
                {
                    const juce::String node(name);
                    // The swing VCA runs for the claves only while their envelope drains; its nodes do not reach them.
                    // The claves' bridged-T rings near its own oscillation, where the bilinear transform's warping
                    // puts it some 5 Hz under the reference's 2498 Hz (3 cents): the phase drifts through a 30 ms
                    // window, so they are held to their pitch and their peak below, not sample by sample.
                    const bool unused = c.claves && (node == "nf" || node == "ne63" || node == "no20b" || node == "ns" || node == "vout");
                    // The accented rim shot's swing VCA switches inside a sample in its first 3 ms: its peak there
                    // runs 8 % over the reference's, which resolves the edge in sub-microsecond steps; after them its
                    // output is within 7 %.
                    const double limit = c.volts > 10.0 && (node == "nf" || node == "ne63" || node == "vout") ? 0.10 : 0.05;
                    // Q63's emitter carries its bias (some -0.6 V): its signal is what is compared.
                    std::vector<double> m = model[name], s = ref.columns.at(name);
                    if (juce::String(name) == "ne63" || juce::String(name) == "nf")
                    {
                        const double m0 = m.front(), s0 = s.front();
                        for (auto& v : m)
                            v -= m0;
                        for (auto& v : s)
                            v -= s0;
                    }
                    const double err = relativeError(m, s, 0);
                    line << " " << name << " " << juce::String(err, 4);
                    if (! unused)
                        expectLessThan(err, limit, juce::String(c.file) + " " + name);
                }
                if (c.claves)
                {
                    const auto hz = [&ref](const std::vector<double>& v)
                    {
                        // Upward zero crossings from 5 ms to the end, interpolated: the ring's frequency.
                        double first = -1.0, last = -1.0;
                        int count = 0;
                        for (std::size_t i = static_cast<std::size_t>(0.005 * ref.rate) + 1; i < v.size(); ++i)
                            if (v[i - 1] < 0.0 && v[i] >= 0.0)
                            {
                                last = static_cast<double>(i - 1) + v[i - 1] / (v[i - 1] - v[i]);
                                if (first < 0.0)
                                    first = last;
                                ++count;
                            }
                        return count > 1 ? ref.rate * (count - 1) / (last - first) : 0.0;
                    };
                    const auto peak = [](const std::vector<double>& v)
                    {
                        double p = 0.0;
                        for (const double x : v)
                            p = std::max(p, std::abs(x));
                        return p;
                    };
                    const auto& spice = ref.columns.at("vout");
                    const double modelHz = hz(model["vout"]), spiceHz = hz(spice);
                    line << " | ring " << juce::String(modelHz, 1) << " Hz against " << juce::String(spiceHz, 1) << ", peak "
                         << juce::String(peak(model["vout"]), 3) << " V against " << juce::String(peak(spice), 3);
                    expectLessThan(std::abs(1200.0 * std::log2(modelHz / spiceHz)), 6.0, "the claves' pitch, cents");
                    expectLessThan(std::abs(peak(model["vout"]) / peak(spice) - 1.0), 0.03, "the claves' peak");
                }
                line << " | most Newton steps " << worstNewton;
                logMessage(line);
                expect(converged, juce::String(c.file) + ": every sample converged");
            }
        }

        if (const auto dump = juce::SystemStats::getEnvironmentVariable("COOLSYNTH_808_DRUMS_DUMP", {}); dump.isNotEmpty())
        {
            // Long renders of every instrument, for the decay and pitch comparison with SPICE and the units (D252):
            // vout only, one CSV each, into that folder.
            beginTest("dump");
            const double rate = 96000.0;
            const juce::File folder(dump);
            folder.createDirectory();
            struct Job { const char* name; int pair; bool conga; double volts, tune; };
            const Job jobs[] = { { "lt", 0, false, 4, 0.5 }, { "lc", 0, true, 4, 0.5 }, { "mt", 1, false, 4, 0.5 }, { "mc", 1, true, 4, 0.5 },
                                 { "ht", 2, false, 4, 0.5 }, { "hc", 2, true, 4, 0.5 }, { "lt14", 0, false, 14, 0.5 }, { "lc14", 0, true, 14, 0.5 },
                                 { "mt14", 1, false, 14, 0.5 }, { "mc14", 1, true, 14, 0.5 }, { "ht14", 2, false, 14, 0.5 }, { "hc14", 2, true, 14, 0.5 },
                                 { "lt_t0", 0, false, 4, 0.0 }, { "lt_t1", 0, false, 4, 1.0 }, { "lc_t0", 0, true, 4, 0.0 }, { "lc_t1", 0, true, 4, 1.0 },
                                 { "mt_t0", 1, false, 4, 0.0 }, { "mt_t1", 1, false, 4, 1.0 }, { "mc_t0", 1, true, 4, 0.0 }, { "mc_t1", 1, true, 4, 1.0 },
                                 { "ht_t0", 2, false, 4, 0.0 }, { "ht_t1", 2, false, 4, 1.0 }, { "hc_t0", 2, true, 4, 0.0 }, { "hc_t1", 2, true, 4, 1.0 } };
            for (const auto& j : jobs)
            {
                auto tom = std::make_unique<Tr808Tom>(static_cast<Tr808Tom::Pair>(j.pair));
                tom->prepare(rate);
                tom->setConga(j.conga);
                tom->setTuning(j.tune);
                tom->setLevel(0.8);
                tom->settle();
                tom->trigger(j.volts);
                juce::String text = "time,vout\n";
                for (int i = 0; i < static_cast<int>(0.5 * rate); ++i)
                    text << juce::String(i / rate, 9) << "," << juce::String(tom->process(0.0), 7) << "\n";
                folder.getChildFile(juce::String(j.name) + ".csv").replaceWithText(text);
            }
            for (const auto& [name, claves, volts] : { std::tuple { "rs", false, 4.0 }, std::tuple { "cl", true, 4.0 },
                                                      std::tuple { "rs14", false, 14.0 }, std::tuple { "cl14", true, 14.0 } })
            {
                auto drum = std::make_unique<Tr808RimShot>();
                drum->prepare(rate);
                drum->setClaves(claves);
                drum->setLevel(0.8);
                drum->settle();
                drum->trigger(volts);
                juce::String text = "time,vout\n";
                for (int i = 0; i < static_cast<int>(0.2 * rate); ++i)
                    text << juce::String(i / rate, 9) << "," << juce::String(drum->process(), 7) << "\n";
                folder.getChildFile(juce::String(name) + ".csv").replaceWithText(text);
            }
        }
    }
};

static V3Tr808DrumsTests v3Tr808DrumsTests;

namespace
{
    using namespace coolsynth::synth::v3;

    GeneratorSettings drumsOf(std::initializer_list<std::pair<int, float>> units = {})
    {
        GeneratorSettings g;
        g.type = generatorType::drums808;
        const auto& d = GeneratorSlot::typeTable().descriptor(g.type);
        for (int i = 0; i < d.usedParams; ++i)
            g.p[static_cast<std::size_t>(i)] = unitsToNormalised(d.params[static_cast<std::size_t>(i)], d.params[static_cast<std::size_t>(i)].def);
        for (const auto& [index, value] : units)
            g.p[static_cast<std::size_t>(index)] = unitsToNormalised(d.params[static_cast<std::size_t>(index)], value);
        g.level = 0.5f;
        return g;
    }

    PartSettings kitOf(PlayMode mode = PlayMode::kit)
    {
        PartSettings s;
        s.playMode = mode;
        s.voice.generators[0] = drumsOf();
        Envelope::Parameters amp;
        amp.attack = 0.0005f;
        amp.decay = 0.2f;
        amp.sustain = 1.0f;
        amp.release = 0.01f;
        s.voice.envelopes = { amp, amp, amp };
        return s;
    }

    struct Hit
    {
        int sample, note;
        bool off = false;
    };

    /** A part playing these hits (note-ons, or note-offs), host-rate mono out; the part's voices active at the end. */
    std::vector<float> play(const PartSettings& settings, const std::vector<Hit>& hits, double seconds, int* activeAtEnd = nullptr,
                            int shedAtBlock = -1, float* shedLevel = nullptr, double rate = 48000.0)
    {
        constexpr int block = 256;
        auto part = std::make_unique<Part>();
        part->prepare(rate, block, 11);
        const int blocks = static_cast<int>(seconds * rate) / block;
        std::vector<float> out, left(block), right(block);
        for (int b = 0; b < blocks; ++b)
        {
            if (b == shedAtBlock)
            {
                // The overload's emergency, as SoundUnit::shedQuietestReleased asks it (D222, D258).
                float level = -1.0f;
                if (const int v = part->quietestReleased(level); v >= 0)
                    part->fadeOutVoice(v);
                if (shedLevel != nullptr)
                    *shedLevel = level;
            }
            std::vector<coolsynth::synth::EngineMidiEvent> events;
            for (const auto& h : hits)
                if (h.sample >= b * block && h.sample < (b + 1) * block)
                {
                    coolsynth::synth::EngineMidiEvent e;
                    e.type = h.off ? coolsynth::synth::EngineMidiEventType::noteOff : coolsynth::synth::EngineMidiEventType::noteOn;
                    e.sampleOffset = h.sample - b * block;
                    e.noteNumber = static_cast<std::uint8_t>(h.note);
                    e.value = h.off ? 0.0f : 1.0f;
                    events.push_back(e);
                }
            std::fill(left.begin(), left.end(), 0.0f);
            std::fill(right.begin(), right.end(), 0.0f);
            part->render(left.data(), right.data(), block, events, settings, {});
            out.insert(out.end(), left.begin(), left.end());
        }
        if (activeAtEnd != nullptr)
            *activeAtEnd = part->activeVoiceCountForTesting();
        return out;
    }

    /** As play, with a watch on the circuits (CoolSynth's D313): one light's level taken after every block. */
    std::vector<float> playWatched(const PartSettings& settings, const std::vector<Hit>& hits, double seconds, CircuitWatch& watch,
                                   int light, std::vector<float>& lightPerBlock, double rate = 48000.0)
    {
        constexpr int block = 256;
        auto part = std::make_unique<Part>();
        part->prepare(rate, block, 11);
        part->setCircuitWatch(&watch);
        const int blocks = static_cast<int>(seconds * rate) / block;
        std::vector<float> out, left(block), right(block);
        for (int b = 0; b < blocks; ++b)
        {
            std::vector<coolsynth::synth::EngineMidiEvent> events;
            for (const auto& h : hits)
                if (h.sample >= b * block && h.sample < (b + 1) * block)
                {
                    coolsynth::synth::EngineMidiEvent e;
                    e.type = coolsynth::synth::EngineMidiEventType::noteOn;
                    e.sampleOffset = h.sample - b * block;
                    e.noteNumber = static_cast<std::uint8_t>(h.note);
                    e.value = 1.0f;
                    events.push_back(e);
                }
            std::fill(left.begin(), left.end(), 0.0f);
            std::fill(right.begin(), right.end(), 0.0f);
            part->render(left.data(), right.data(), block, events, settings, {});
            out.insert(out.end(), left.begin(), left.end());
            lightPerBlock.push_back(watch.take(light));
        }
        return out;
    }

    float peakOf(const std::vector<float>& x, std::size_t from = 0, std::size_t to = static_cast<std::size_t>(-1))
    {
        float p = 0.0f;
        for (std::size_t i = from; i < std::min(to, x.size()); ++i)
            p = std::max(p, std::abs(x[i]));
        return p;
    }

    /** Upward zero crossings per second between two times: a ringing drum's frequency. */
    double crossingHz(const std::vector<float>& x, double rate, double from, double to)
    {
        double first = -1.0, last = -1.0;
        int count = 0;
        for (auto i = static_cast<std::size_t>(from * rate) + 1; i < std::min(x.size(), static_cast<std::size_t>(to * rate)); ++i)
            if (x[i - 1] < 0.0f && x[i] >= 0.0f)
            {
                last = static_cast<double>(i - 1) + static_cast<double>(x[i - 1] / (x[i - 1] - x[i]));
                if (first < 0.0)
                    first = last;
                ++count;
            }
        return count > 1 ? rate * (count - 1) / (last - first) : 0.0;
    }
}

/*
    The 808 Drums in the Kit play mode (D252): one voice holds the kit, each note strikes its instrument, note-offs
    change nothing, the voice ends once everything has rung out; the generator's keys and its pairs' switches.
*/
class V3Tr808KitTests final : public juce::UnitTest
{
public:
    V3Tr808KitTests() : juce::UnitTest("V3 TR-808 drums kit", "V3") {}

    void runTest() override
    {
        beginTest("the_808_drums_is_generator_type_thirteen_with_twelve_knobs");
        {
            expectEquals(GeneratorSlot::typeCount, 16); // the 808 Metal and the 808 Snare & Clap came after it (D255, D257)
            expectEquals(juce::String(GeneratorSlot::descriptorFor(generatorType::drums808).name), juce::String("808 Drums"));
            expectEquals(Tr808DrumsGenerator::descriptor().usedParams, 12);
            bool other = false;
            const std::pair<int, int> keys[] = { { 35, 0 }, { 36, 0 }, { 37, 4 }, { 75, 4 }, { 41, 1 }, { 43, 1 }, { 64, 1 },
                                                 { 45, 2 }, { 47, 2 }, { 63, 2 }, { 48, 3 }, { 50, 3 }, { 62, 3 }, { 60, -1 } };
            for (const auto& [note, instrument] : keys)
                expectEquals(Tr808DrumsGenerator::instrumentFor(note, other), instrument, "key " + juce::String(note));
        }

        beginTest("two_hits_in_one_block_both_sound");
        {
            // The bass drum and the low tom struck in the same block, the second while the voice's note still waits
            // to start: the kit plays both, the sum of each alone (their circuits are apart).
            const auto both = play(kitOf(), { { 0, 36 }, { 0, 43 } }, 0.5);
            const auto bass = play(kitOf(), { { 0, 36 } }, 0.5);
            const auto tom = play(kitOf(), { { 0, 43 } }, 0.5);
            float worst = 0.0f;
            for (std::size_t i = 0; i < both.size(); ++i)
                worst = std::max(worst, std::abs(both[i] - bass[i] - tom[i]));
            logMessage("both against the sum: worst " + juce::String(worst, 7) + " of a peak of " + juce::String(peakOf(both), 4));
            expectGreaterThan(peakOf(bass), 0.01f);
            expectGreaterThan(peakOf(tom), 0.01f);
            expectLessThan(worst, 1.0e-4f * peakOf(both));
            // And a later hit in another block while the first rings: retriggered, not restarted.
            const auto later = play(kitOf(), { { 0, 36 }, { 4800, 43 } }, 0.5);
            const auto tomLater = play(kitOf(), { { 4800, 43 } }, 0.5);
            worst = 0.0f;
            for (std::size_t i = 0; i < later.size(); ++i)
                worst = std::max(worst, std::abs(later[i] - bass[i] - tomLater[i]));
            expectLessThan(worst, 1.0e-4f * peakOf(later), "a hit while another rings adds to it");
        }

        beginTest("a_watch_follows_each_drum_and_changes_no_sound");
        {
            // CoolSynth's D313: the panel's lights follow the circuits as they play. The sound is the same, bit for bit,
            // with a watch; the bass drum's light rises with its hit and goes out as it rings down; the others stay dark.
            const auto plain = play(kitOf(), { { 0, 36 } }, 2.0);
            auto watchOwner = std::make_unique<CircuitWatch>(); // 800 kB: not on the test runner's stack
            auto& watch = *watchOwner;
            std::vector<float> light;
            const auto watched = playWatched(kitOf(), { { 0, 36 } }, 2.0, watch, CircuitWatch::bassDrum, light);
            expect(plain == watched, "the sound is the same with a watch");
            const float top = *std::max_element(light.begin(), light.end());
            expectGreaterThan(top, 0.05f, "the bass drum lights");
            expectLessThan(light.back(), 0.01f * top, "and goes out as it rings down");
            expectEquals(watch.take(CircuitWatch::lowTom), 0.0f, "the low tom stays dark");
            expectGreaterThan(static_cast<int>(watch.reportCount()), 0);

            // A longer Decay holds the light longer, as it holds the sound.
            const auto longFor = [&](float decay)
            {
                auto s = kitOf();
                s.voice.generators[0] = drumsOf({ { Tr808DrumsGenerator::bdDecay, decay } });
                auto wOwner = std::make_unique<CircuitWatch>();
                auto& w = *wOwner;
                std::vector<float> l;
                playWatched(s, { { 0, 36 } }, 2.0, w, CircuitWatch::bassDrum, l);
                const float peak = *std::max_element(l.begin(), l.end());
                return static_cast<int>(std::count_if(l.begin(), l.end(), [peak](float v) { return v > 0.1f * peak; }));
            };
            const int shortBlocks = longFor(0.1f), longBlocks = longFor(0.9f);
            logMessage("blocks above a tenth of the peak: Decay 0.1 " + juce::String(shortBlocks) + ", Decay 0.9 " + juce::String(longBlocks));
            expectGreaterThan(longBlocks, shortBlocks + 10, "a longer Decay keeps the light longer");
        }

        beginTest("the_bass_drum_shows_its_circuit_as_it_runs");
        {
            // CoolSynth's D317: a panel watching the bass drum gets its circuit, frame by frame over a hit, named as its
            // ngspice recording is. The sound is the same, bit for bit; the frames hold what the circuit holds: Ohm's
            // law across R174 and R162, the envelope's swing, the output's.
            const auto plain = play(kitOf(), { { 0, 36 } }, 0.5);
            auto watchOwner = std::make_unique<CircuitWatch>(); // 800 kB: not on the test runner's stack
            auto& watch = *watchOwner;
            watch.watchCircuit(CircuitWatch::bassDrum);
            std::vector<float> light;
            const auto watched = playWatched(kitOf(), { { 0, 36 } }, 0.5, watch, CircuitWatch::bassDrum, light);
            expect(plain == watched, "the sound is the same while its circuit is captured");
            expectEquals(watch.capturedSeries(), Tr808BassDrum::liveCount);
            const int frames = watch.framesCaptured();
            logMessage("frames captured: " + juce::String(frames));
            expectGreaterThan(frames, 400);
            const auto indexOf = [&](const char* name)
            {
                for (int s = 0; s < watch.capturedSeries(); ++s)
                    if (juce::String(watch.capturedName(s)) == name)
                        return s;
                return -1;
            };
            const int vout = indexOf("vout"), r174 = indexOf("r174.i"), npulse = indexOf("npulse"), r162 = indexOf("r162.i"), neg = indexOf("neg");
            expect(vout >= 0 && r174 >= 0 && npulse >= 0 && r162 >= 0 && neg >= 0, "the named series are there");
            double worst = 0.0, envelopeTop = 0.0, outputTop = 0.0;
            for (int f = 0; f < frames; ++f)
            {
                const double v = watch.captured(f, vout), i = watch.captured(f, r174);
                const double p = watch.captured(f, npulse), ip = watch.captured(f, r162);
                if (std::isfinite(v) && std::isfinite(i))
                    worst = std::max(worst, std::abs(i * 100.0e3 - v) / (1.0e-3 + std::abs(v)));
                if (std::isfinite(p) && std::isfinite(ip))
                    worst = std::max(worst, std::abs(ip * 4.7e3 - p) / (1.0e-3 + std::abs(p)));
                envelopeTop = std::max(envelopeTop, static_cast<double>(watch.captured(f, neg)));
                outputTop = std::max(outputTop, std::abs(v));
            }
            logMessage("Ohm's law, worst relative error: " + juce::String(worst, 8) + "; envelope top " + juce::String(envelopeTop, 3)
                       + " V; output top " + juce::String(outputTop, 3) + " V");
            expectLessThan(worst, 1.0e-3, "the frames obey Ohm's law");
            expectGreaterThan(envelopeTop, 8.0, "the envelope rises as the recording's (12.4 V)");
            expectGreaterThan(outputTop, 0.5, "the output swings");
        }

        beginTest("note_offs_change_nothing");
        {
            const auto held = play(kitOf(), { { 0, 43 } }, 0.6);
            const auto tapped = play(kitOf(), { { 0, 43 }, { 100, 43, true } }, 0.6);
            expect(held == tapped, "a note-off leaves the drum ringing as it was");
        }

        beginTest("the_voice_ends_once_every_instrument_has_rung_out");
        {
            int activeEarly = 0, activeLate = 0, activeUnmapped = 0;
            (void) play(kitOf(), { { 0, 36 }, { 0, 37 } }, 0.1, &activeEarly);
            (void) play(kitOf(), { { 0, 36 }, { 0, 37 } }, 4.0, &activeLate);
            (void) play(kitOf(), { { 0, 60 } }, 0.5, &activeUnmapped);
            expectEquals(activeEarly, 1, "one voice holds the kit while it rings");
            expectEquals(activeLate, 0, "and lets go once it has rung down");
            expectEquals(activeUnmapped, 0, "a key with no instrument ends at once");
        }

        beginTest("a_conga_takes_over_its_toms_circuit");
        {
            // The low tom near 90 Hz; the low conga's key switches the same circuit to near 200 Hz (D252, page 14).
            // Accent 0: the unaccented trigger (4.7 V since D258; the late pitch moves well under these limits from 4 V).
            auto plain = kitOf();
            plain.voice.generators[0] = drumsOf({ { Tr808DrumsGenerator::accent, 0.0f } });
            const auto tom = play(plain, { { 0, 43 } }, 0.3);
            const auto conga = play(plain, { { 0, 43 }, { 4800, 64 } }, 0.4);
            const double tomHz = crossingHz(tom, 48000.0, 0.03, 0.09);
            const double congaHz = crossingHz(conga, 48000.0, 0.13, 0.19);
            logMessage("low tom " + juce::String(tomHz, 1) + " Hz, then its conga " + juce::String(congaHz, 1) + " Hz");
            expectWithinAbsoluteError(tomHz, 90.3, 3.0);
            expectWithinAbsoluteError(congaHz, 200.0, 6.0);
            // The claves' key switches the rim shot's circuit: its ring near 2.4 kHz, both units' 2410 to 2422 Hz since
            // R312's fit (D258).
            const auto claves = play(plain, { { 0, 75 } }, 0.1);
            const double clavesHz = crossingHz(claves, 48000.0, 0.005, 0.02);
            logMessage("claves " + juce::String(clavesHz, 1) + " Hz");
            expectWithinAbsoluteError(clavesHz, 2412.0, 30.0);
        }

        beginTest("at_a_96_kHz_host_the_voices_run_at_96_kHz_not_192");
        {
            // D258: twice the host's rate up to 48 kHz, the host's own from 88.2 kHz. The low tom rings at its pitch
            // at either host, and at 96 kHz the part has no decimator's delay.
            auto plain = kitOf();
            plain.voice.generators[0] = drumsOf({ { Tr808DrumsGenerator::accent, 0.0f } });
            const auto at48 = play(plain, { { 0, 43 } }, 0.3);
            const auto at96 = play(plain, { { 0, 43 } }, 0.3, nullptr, -1, nullptr, 96000.0);
            const double hz48 = crossingHz(at48, 48000.0, 0.03, 0.09), hz96 = crossingHz(at96, 96000.0, 0.03, 0.09);
            logMessage("low tom at a 48 kHz host " + juce::String(hz48, 2) + " Hz, at 96 kHz " + juce::String(hz96, 2));
            expectWithinAbsoluteError(hz96, hz48, 0.5);
            expectEquals(Part::oversampleFor(44100.0), 2);
            expectEquals(Part::oversampleFor(48000.0), 2);
            expectEquals(Part::oversampleFor(88200.0), 1);
            expectEquals(Part::oversampleFor(96000.0), 1);
            auto p48 = std::make_unique<Part>();
            auto p96 = std::make_unique<Part>();
            p48->prepare(48000.0, 256, 11);
            p96->prepare(96000.0, 256, 11);
            logMessage("latency " + juce::String(p48->latencySamples()) + " samples at 48 kHz, " + juce::String(p96->latencySamples()) + " at 96");
            expectLessThan(p96->latencySamples(), p48->latencySamples(), "no decimator at the host's own rate");
        }

        beginTest("the_overloads_emergency_sheds_the_kits_quietest_drum_not_the_kit");
        {
            // D258: the Kit voice is let go at every strike, but the emergency (D222) takes its quietest ringing drum,
            // not the whole kit. The bass drum and the rim shot together; 80 ms in (each drum's level a peak follower
            // of 20 ms, so earlier the rim shot's still reads the louder), the rim shot is the quieter: it fades over
            // the steal fade, and from then on the kit is the bass drum alone, sample for sample (separate circuits,
            // no shared noise), its voice still sounding.
            const auto settings = kitOf();
            float level = -1.0f;
            int active = 0;
            const auto both = play(settings, { { 0, 36 }, { 0, 37 } }, 0.3, &active, 3840 / 256, &level);
            const auto bassAlone = play(settings, { { 0, 36 } }, 0.3);
            logMessage("shed at " + juce::String(level, 6) + "; voices sounding at the end " + juce::String(active));
            expectGreaterThan(level, 0.0f, "a drum rang when the emergency asked");
            expectEquals(active, 1, "the kit's voice is not shed: the bass drum still rings");
            double worst = 0.0;
            for (std::size_t i = 6000; i < both.size(); ++i) // from 125 ms: well past the fade and the latency
                worst = std::max(worst, static_cast<double>(std::abs(both[i] - bassAlone[i])));
            logMessage("after the rim shot's fade, against the bass drum alone: " + juce::String(worst, 9));
            expectLessThan(worst, 1.0e-6, "the rim shot faded, the bass drum rang on as alone");
            expectGreaterThan(peakOf(both, 6000), 0.01f, "and it still sounds");
        }

        beginTest("after_reset_a_hit_is_the_fresh_kits_bit_for_bit");
        {
            // The determinism tests' rule (D224) at the generator: a kit reset and reseeded plays as a fresh one.
            SlotMemory memory;
            memory.allocate(Tr808DrumsGenerator::memoryFloats(96000.0, 32));
            memory.setSeed(5);
            auto fresh = std::make_unique<Tr808DrumsGenerator>();
            fresh->prepare(96000.0, 32, memory);
            SlotMemory memoryUsed;
            memoryUsed.allocate(Tr808DrumsGenerator::memoryFloats(96000.0, 32));
            memoryUsed.setSeed(5);
            auto used = std::make_unique<Tr808DrumsGenerator>();
            used->prepare(96000.0, 32, memoryUsed);
            SlotParams params;
            const auto& d = Tr808DrumsGenerator::descriptor();
            for (int i = 0; i < d.usedParams; ++i)
                params.p[static_cast<std::size_t>(i)] = d.params[static_cast<std::size_t>(i)].def;
            std::array<float, 32> l {}, r {};
            NoteContext note;
            note.noteNumber = 41;
            used->noteOn(note);
            for (int b = 0; b < 300; ++b)
                used->process(l.data(), r.data(), 32, params);
            used->reset();
            used->reseed();
            for (auto* g : { fresh.get(), used.get() })
            {
                note.noteNumber = 47;
                g->noteOn(note);
                g->noteChanged(36);
                g->strike(0.7f);
            }
            bool same = true;
            std::array<float, 32> l2 {}, r2 {};
            for (int b = 0; b < 3000 && same; ++b)
            {
                fresh->process(l.data(), r.data(), 32, params);
                used->process(l2.data(), r2.data(), 32, params);
                same = l == l2;
            }
            expect(same, "a reset kit plays its next hits as a fresh one, bit for bit");
        }
    }
};

static V3Tr808KitTests v3Tr808KitTests;

namespace
{
    /** The 808's whole kit as the app plays it: its three generators in one Kit voice, at the stock levels. */
    EngineSettings wholeKitOf(float masterGain)
    {
        EngineSettings out;
        auto& part = out.parts[0];
        part.playMode = PlayMode::kit;
        const int types[] = { generatorType::drums808, generatorType::snareClap808, generatorType::metal808 };
        for (std::size_t i = 0; i < 3; ++i)
        {
            auto& g = part.voice.generators[i];
            g.type = types[i];
            const auto& d = GeneratorSlot::typeTable().descriptor(g.type);
            for (int k = 0; k < d.usedParams; ++k)
                g.p[static_cast<std::size_t>(k)] = unitsToNormalised(d.params[static_cast<std::size_t>(k)], d.params[static_cast<std::size_t>(k)].def);
            g.level = 0.8f;
        }
        Envelope::Parameters amp;
        amp.attack = 0.0f;
        amp.decay = 1.0f;
        amp.sustain = 1.0f;
        amp.release = 10.0f;
        amp.velocitySensitivity = 0.0f;
        part.voice.envelopes[0] = amp;
        out.masterGainLinear = masterGain;
        return out;
    }

    struct OwnRender
    {
        std::vector<float> left, right;
        std::array<std::vector<float>, CircuitWatch::lightCount> taps;
    };

    /** The engine playing these hits in host blocks of `block` (longer than its prepared 256: it splits them), with a
        watch separating `separate` and taps given (each filled with `untouched` first), or neither. */
    OwnRender playEngine(const std::vector<Hit>& hits, double seconds, bool watched, std::uint32_t separate, bool tapsGiven,
                         float untouched = 7.0f)
    {
        constexpr double rate = 48000.0;
        constexpr int block = 400;
        auto engine = std::make_unique<EngineV3>(EngineShape::singleLayer());
        engine->prepare(rate, 256, 2);
        auto watchOwner = std::make_unique<CircuitWatch>(); // 800 kB: not on the test runner's stack
        watchOwner->setSeparate(separate);
        if (watched)
            engine->setCircuitWatch(watchOwner.get());
        const auto settings = wholeKitOf(0.25f); // the kit's Volume near -12 dB: under the soft clip's knee
        OwnRender out;
        juce::AudioBuffer<float> buffer(2, block);
        std::array<std::vector<float>, CircuitWatch::lightCount> tapBuffers;
        std::array<float*, CircuitWatch::lightCount> tapPointers {};
        for (std::size_t l = 0; l < tapBuffers.size(); ++l)
        {
            tapBuffers[l].assign(static_cast<std::size_t>(block), 0.0f);
            tapPointers[l] = tapBuffers[l].data();
        }
        const int blocks = static_cast<int>(seconds * rate) / block;
        for (int b = 0; b < blocks; ++b)
        {
            std::vector<coolsynth::synth::EngineMidiEvent> events;
            for (const auto& h : hits)
                if (h.sample >= b * block && h.sample < (b + 1) * block)
                {
                    coolsynth::synth::EngineMidiEvent e;
                    e.type = coolsynth::synth::EngineMidiEventType::noteOn;
                    e.sampleOffset = h.sample - b * block;
                    e.noteNumber = static_cast<std::uint8_t>(h.note);
                    e.value = 1.0f;
                    events.push_back(e);
                }
            for (auto& t : tapBuffers)
                std::fill(t.begin(), t.end(), untouched);
            buffer.clear();
            engine->render(buffer, events, settings, {}, tapsGiven ? tapPointers.data() : nullptr);
            out.left.insert(out.left.end(), buffer.getReadPointer(0), buffer.getReadPointer(0) + block);
            out.right.insert(out.right.end(), buffer.getReadPointer(1), buffer.getReadPointer(1) + block);
            for (std::size_t l = 0; l < tapBuffers.size(); ++l)
                out.taps[l].insert(out.taps[l].end(), tapBuffers[l].begin(), tapBuffers[l].end());
        }
        return out;
    }
}

/*
    The 808's drums on their own outputs (CoolSynth's D343): a separated drum leaves the main mix and comes out on its
    own, as it would have sounded there, so the main and the separated drums add up to the whole mix.
*/
class V3Tr808OwnOutputTests final : public juce::UnitTest
{
public:
    V3Tr808OwnOutputTests() : juce::UnitTest("V3 TR-808 kit own outputs", "V3") {}

    void runTest() override
    {
        beginTest("separated_drums_and_the_main_add_up_to_the_whole_mix");
        {
            // Every generator's instruments, overlapping: the bass drum, the snare and the closed hat separated, the
            // rim shot, a tom, the clap, the cowbell and the open hat left in the main.
            const std::vector<Hit> hits { { 0, 36 }, { 0, 42 }, { 2400, 46 }, { 4800, 38 }, { 4800, 42 }, { 7000, 43 },
                                          { 9600, 39 }, { 9600, 36 }, { 12000, 56 }, { 14400, 37 }, { 14400, 42 }, { 16800, 38 } };
            constexpr double seconds = 1.0;
            const std::uint32_t separate = (1u << CircuitWatch::bassDrum) | (1u << CircuitWatch::snare) | (1u << CircuitWatch::closedHat);

            const auto plain = playEngine(hits, seconds, false, 0u, false);
            const auto maskZero = playEngine(hits, seconds, true, 0u, true);
            const auto apart = playEngine(hits, seconds, true, separate, true);

            // Nothing separated: the same samples, bit for bit, as with no watch and no taps; no tap written.
            expect(plain.left == maskZero.left && plain.right == maskZero.right, "a zero mask changes nothing");
            bool written = false;
            for (const auto& t : maskZero.taps)
                for (const float v : t)
                    written = written || v != 7.0f;
            expect(! written, "a zero mask writes no tap");

            // The main without them, and each on its own.
            const float wholePeak = std::max(peakOf(plain.left), peakOf(plain.right));
            logMessage("the whole mix's peak " + juce::String(wholePeak, 4) + " (the soft clip's knee is 0.8)");
            expectLessThan(wholePeak, 0.8f);
            for (const int light : { CircuitWatch::bassDrum, CircuitWatch::snare, CircuitWatch::closedHat })
                expectGreaterThan(peakOf(apart.taps[static_cast<std::size_t>(light)]), 0.005f, "light " + juce::String(light) + " on its own output");
            expectLessThan(peakOf(apart.left, 0, 2400), 1.0e-6f, "the first bass drum and closed hat are not in the main");

            // A drum left in the main: its tap untouched.
            for (int light = 0; light < CircuitWatch::lightCount; ++light)
            {
                if (((separate >> light) & 1u) != 0u)
                    continue;
                const auto& t = apart.taps[static_cast<std::size_t>(light)];
                expect(std::all_of(t.begin(), t.end(), [](float v) { return v == 7.0f; }), "light " + juce::String(light) + "'s tap not written");
            }

            // The main and the separated drums against the whole mix, each side.
            const auto& bd = apart.taps[CircuitWatch::bassDrum];
            const auto& sd = apart.taps[CircuitWatch::snare];
            const auto& ch = apart.taps[CircuitWatch::closedHat];
            for (int side = 0; side < 2; ++side)
            {
                const auto& whole = side == 0 ? plain.left : plain.right;
                const auto& main = side == 0 ? apart.left : apart.right;
                double rest = 0.0, energy = 0.0;
                for (std::size_t i = 0; i < whole.size(); ++i)
                {
                    const double d = static_cast<double>(main[i]) + bd[i] + sd[i] + ch[i] - whole[i];
                    rest += d * d;
                    energy += static_cast<double>(whole[i]) * whole[i];
                }
                const double db = 10.0 * std::log10(std::max(rest, 1.0e-300) / energy);
                logMessage(juce::String(side == 0 ? "left" : "right") + ": the main and the three taps against the whole mix, "
                           + juce::String(db, 1) + " dB");
                expectLessThan(db, -100.0, "they add up to the whole mix");
            }
        }
    }
};

static V3Tr808OwnOutputTests v3Tr808OwnOutputTests;

/*
    The rim shot's two rings against both units (D258), opt-in: with COOLSYNTH_808_RIM_FIT set to "r308:r312:r318:r320"
    scales of the drawn parts (space separated) and COOLSYNTH_808_RIM_FIT_DIR to a folder, the rim shot circuit alone at
    96 kHz, LEVEL 0.8, a 4.7 V hit (the units' unaccented), each trial's vout over 60 ms written there as CSV for
    scratchpad rimfit.py to measure. Nothing asserted.
*/
class V3Tr808RimFit final : public juce::UnitTest
{
public:
    V3Tr808RimFit() : juce::UnitTest("V3 TR-808 rim shot fit (opt-in)", "V3") {}

    void runTest() override
    {
        const auto grid = juce::SystemStats::getEnvironmentVariable("COOLSYNTH_808_RIM_FIT", {});
        const juce::File folder(juce::SystemStats::getEnvironmentVariable("COOLSYNTH_808_RIM_FIT_DIR", {}));
        if (grid.isEmpty() || folder.getFullPathName().isEmpty())
            return;
        beginTest("rim_shot_grid");
        folder.createDirectory();
        const double rate = 96000.0;
        for (const auto& t : juce::StringArray::fromTokens(grid, " ", ""))
        {
            const auto v = juce::StringArray::fromTokens(t, ":", "");
            if (v.size() != 4)
                continue;
            for (const bool claves : { false, true })
            {
                auto drum = std::make_unique<Tr808RimShot>();
                drum->prepare(rate);
                drum->setClaves(claves);
                drum->setLevel(0.8);
                drum->setRingParts(820.0e3 * v[0].getDoubleValue(), 1.0e3 * v[1].getDoubleValue(), 220.0e3 * v[2].getDoubleValue(),
                                   22.0e3 * v[3].getDoubleValue());
                drum->settle();
                drum->trigger(4.7);
                juce::String text = "time,vout\n";
                for (int i = 0; i < static_cast<int>((claves ? 0.1 : 0.06) * rate); ++i)
                    text << juce::String(i / rate, 9) << "," << juce::String(drum->process(), 8) << "\n";
                folder.getChildFile(juce::String(claves ? "cl_" : "rim_") + t.replace(":", "_") + ".csv").replaceWithText(text);
            }
            logMessage("rendered " + t);
        }
    }
};

static V3Tr808RimFit v3Tr808RimFit;
