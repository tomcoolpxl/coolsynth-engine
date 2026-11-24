#include <juce_core/juce_core.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <tuple>
#include <vector>

#include "synth/v3/CircuitWdf.h"
#include "synth/v3/Tr808DrumsGenerator.h"
#include "synth/v3/Voice.h"

/*
    SIMD across voices (CoolSynth's D347): two circuits of one build solved side by side in a register's lanes
    (wdf::CircuitPair) are, bit for bit, each solved alone; and the 808 Drums' toms, two of them ringing at once in the
    lanes, play the same bytes as with every tom alone. With COOLSYNTH_CPU_REPORT set, the toms' cost both ways.
*/
namespace
{
    using namespace coolsynth::synth::v3;

    bool sameBits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

    /** A diode clipper behind an RC, its input a sine; `extra` adds a capacitor and a resistor (another shape). */
    using Clipper = wdf::Circuit<3, 6, 1, 0, 1, 2>;

    struct ClipperParts
    {
        int input = 0, series = 0, probe = 0;
    };

    ClipperParts build(Clipper& c, double ohms, double farads, bool extra)
    {
        ClipperParts p;
        p.input = c.addInput(1);
        p.series = c.addResistor(1, 2, ohms);
        c.addCapacitor(2, 0, farads);
        c.addDiodePair(2, 0, 1.0e-12, 0.02585);
        if (extra)
        {
            c.addCapacitor(2, 3, farads);
            c.addResistor(3, 0, 10.0e3);
        }
        p.probe = c.addProbe(2);
        c.setSampleRate(96000.0);
        return p;
    }

    /** Two clippers in a pair and their twins alone, for `samples`: how many values differ, and the Newton steps. */
    std::pair<int, int> runPair(bool otherShape, int samples)
    {
        auto lanesA = std::make_unique<Clipper>(), lanesB = std::make_unique<Clipper>();
        auto aloneA = std::make_unique<Clipper>(), aloneB = std::make_unique<Clipper>();
        const auto pa = build(*lanesA, 1.0e3, 10.0e-9, false);
        build(*aloneA, 1.0e3, 10.0e-9, false);
        const auto pb = build(*lanesB, 2.2e3, 4.7e-9, otherShape);
        build(*aloneB, 2.2e3, 4.7e-9, otherShape);
        auto pair = std::make_unique<wdf::CircuitPair<Clipper>>();
        int differ = 0, steps = 0;
        for (int n = 0; n < samples; ++n)
        {
            const double t = n / 96000.0;
            // One driven hard (its diodes conduct: Newton steps, pnjlim), the other softly (below the knees: the
            // fixed-point start), so the two solves differ; half-way a resistance changes (a new analysis: the pair's
            // copies of the matrices made again) and the soft one is driven harder.
            const double ua = 5.0 * std::sin(6.283185307179586 * 220.0 * t);
            const double ub = (n >= samples / 2 ? 3.0 : 0.3) * std::sin(6.283185307179586 * 1370.0 * t);
            if (n == samples / 2)
            {
                lanesB->setResistance(pb.series, 680.0);
                aloneB->setResistance(pb.series, 680.0);
            }
            lanesA->setInput(pa.input, ua);
            aloneA->setInput(pa.input, ua);
            lanesB->setInput(pb.input, ub);
            aloneB->setInput(pb.input, ub);
            pair->process(*lanesA, *lanesB);
            aloneA->process();
            aloneB->process();
            steps += lanesA->lastNewtonSteps() + lanesB->lastNewtonSteps();
            differ += sameBits(lanesA->probe(pa.probe), aloneA->probe(pa.probe)) ? 0 : 1;
            differ += sameBits(lanesB->probe(pb.probe), aloneB->probe(pb.probe)) ? 0 : 1;
            differ += sameBits(lanesA->portVoltage(pa.series), aloneA->portVoltage(pa.series)) ? 0 : 1;
            differ += sameBits(lanesB->portVoltage(pb.series), aloneB->portVoltage(pb.series)) ? 0 : 1;
        }
        return { differ, steps };
    }

    struct Hit
    {
        int sample, note;
        float velocity;
    };

    /** The 808 Drums at 96 kHz in blocks of 32: each block's output and the toms' stage voltages after it. */
    struct KitRun
    {
        std::vector<float> out;
        std::vector<double> toms;
        double seconds = 0.0; // spent in the generator
    };

    KitRun playKit(bool lanes, const std::vector<Hit>& hits, const std::vector<std::tuple<int, int, float>>& moves, int samples,
                   bool record = true)
    {
        constexpr int block = 32;
        SlotMemory memory;
        memory.allocate(Tr808DrumsGenerator::memoryFloats(96000.0, block));
        memory.setSeed(3);
        auto kit = std::make_unique<Tr808DrumsGenerator>();
        kit->prepare(96000.0, block, memory);
        kit->setTomLanesForTesting(lanes);
        SlotParams params;
        const auto& d = Tr808DrumsGenerator::descriptor();
        for (int i = 0; i < d.usedParams; ++i)
            params.p[static_cast<std::size_t>(i)] = d.params[static_cast<std::size_t>(i)].def;
        params.p[Tr808DrumsGenerator::accent] = 0.8f;
        std::array<float, block> l {}, r {};
        KitRun run;
        bool started = false;
        for (int b = 0; b * block < samples; ++b)
        {
            for (const auto& [at, param, value] : moves)
                if (at >= b * block && at < (b + 1) * block)
                    params.p[static_cast<std::size_t>(param)] = value;
            for (const auto& h : hits)
                if (h.sample >= b * block && h.sample < (b + 1) * block)
                {
                    if (! started)
                    {
                        NoteContext note;
                        note.noteNumber = h.note;
                        kit->noteOn(note);
                        started = true;
                    }
                    kit->noteChanged(h.note);
                    kit->strike(h.velocity);
                }
            const auto start = juce::Time::getHighResolutionTicks();
            kit->process(l.data(), r.data(), block, params);
            run.seconds += juce::Time::highResolutionTicksToSeconds(juce::Time::getHighResolutionTicks() - start);
            if (! record)
                continue;
            run.out.insert(run.out.end(), l.begin(), l.end());
            for (int t = 0; t < 3; ++t)
            {
                const auto& tom = kit->tomForTesting(t);
                for (const double v : { tom.pulseVolts(), tom.envelopeVolts(), tom.gateBaseVolts(), tom.noiseVolts(),
                                        tom.middleVolts(), tom.resonatorVolts() })
                    run.toms.push_back(v);
            }
        }
        return run;
    }

    bool sameBytes(const KitRun& a, const KitRun& b)
    {
        return a.out.size() == b.out.size() && a.toms.size() == b.toms.size()
               && std::memcmp(a.out.data(), b.out.data(), a.out.size() * sizeof(float)) == 0
               && std::memcmp(a.toms.data(), b.toms.data(), a.toms.size() * sizeof(double)) == 0;
    }

    int at(double seconds) { return static_cast<int>(seconds * 96000.0); }

    /** Toms and congas over and under one another: two at once, three at once, alone, a switch while ringing, tuning
        knobs moved while they ring, the bass drum and the rim shot among them. */
    const std::vector<Hit>& tomFill()
    {
        static const std::vector<Hit> hits {
            { at(0.00), 43, 1.0f }, { at(0.05), 47, 0.6f }, { at(0.10), 50, 0.9f },  // low, mid, high: a fill
            { at(0.60), 64, 0.8f }, { at(0.65), 63, 1.0f }, { at(0.70), 50, 0.4f },  // congas, the high tom again
            { at(1.30), 41, 1.0f }, { at(1.30), 45, 1.0f }, { at(1.30), 48, 1.0f },  // all three in one block
            { at(2.00), 36, 1.0f }, { at(2.02), 43, 0.7f }, { at(2.04), 37, 1.0f },  // among the bass drum and rim shot
            { at(2.50), 62, 1.0f }, { at(2.51), 47, 0.5f },                          // a conga and a tom
        };
        return hits;
    }

    const std::vector<std::tuple<int, int, float>>& tomMoves()
    {
        static const std::vector<std::tuple<int, int, float>> moves {
            { at(0.30), Tr808DrumsGenerator::lowTuning, 0.9f }, { at(0.31), Tr808DrumsGenerator::midTuning, 0.1f },
            { at(1.40), Tr808DrumsGenerator::highTuning, 0.8f }, { at(1.45), Tr808DrumsGenerator::midLevel, 0.4f },
        };
        return moves;
    }
}

class V3CircuitLanesTests final : public juce::UnitTest
{
public:
    V3CircuitLanesTests() : juce::UnitTest("V3 circuit lanes", "V3") {}

    void runTest() override
    {
        beginTest("a_pair_of_circuits_in_lanes_is_each_alone_bit_for_bit");
        {
            const auto [differ, steps] = runPair(false, 40000);
            logMessage("40,000 samples of two clippers in lanes: " + juce::String(differ) + " values differ; "
                       + juce::String(steps) + " Newton steps between them");
            expectEquals(differ, 0, "each lane is its circuit alone, bit for bit");
            expectGreaterThan(steps, 1000, "the lanes' solves took Newton steps");
            // Two of another shape: the pair solves them one by one, each still as alone.
            expectEquals(runPair(true, 4000).first, 0, "circuits of other shapes, one by one");
        }

        beginTest("the_808_toms_in_lanes_play_the_same_bytes_as_alone");
        {
            const int samples = at(3.2);
            const auto lanes = playKit(true, tomFill(), tomMoves(), samples);
            const auto alone = playKit(false, tomFill(), tomMoves(), samples);
            float peak = 0.0f;
            for (const float v : lanes.out)
                peak = std::max(peak, std::abs(v));
            logMessage("3.2 s of a tom fill: " + juce::String(lanes.out.size()) + " samples, peak " + juce::String(peak, 3)
                       + ", " + juce::String(lanes.toms.size()) + " stage voltages");
            expectGreaterThan(peak, 0.05f, "the toms sound");
            expect(sameBytes(lanes, alone), "the toms in lanes are the toms alone, byte for byte");
            // The lanes' copies of the matrices live in the kit: what they add to a voice's generator slot.
            logMessage("the toms' lanes " + juce::String(static_cast<int>(sizeof(Tr808Tom::Lanes))) + " bytes; the 808 Drums "
                       + juce::String(static_cast<int>(sizeof(Tr808DrumsGenerator))) + ", a generator slot's largest type "
                       + juce::String(static_cast<int>(sizeof(GeneratorTypes))));
        }

        if (juce::SystemStats::getEnvironmentVariable("COOLSYNTH_CPU_REPORT", {}).isNotEmpty())
        {
            beginTest("the_toms_cost_in_lanes_and_alone");
            // Paired runs, interleaved (another session may load the machine): the fill, and three toms held ringing.
            const std::vector<Hit> three { { 0, 43, 1.0f }, { 0, 47, 1.0f }, { 0, 50, 1.0f }, { at(0.4), 43, 1.0f },
                                           { at(0.4), 47, 1.0f }, { at(0.4), 50, 1.0f } };
            const std::vector<Hit> two { { 0, 43, 1.0f }, { 0, 47, 1.0f }, { at(0.4), 43, 1.0f }, { at(0.4), 47, 1.0f } };
            const std::vector<Hit> one { { 0, 43, 1.0f }, { at(0.4), 43, 1.0f } };
            const struct { const char* label; const std::vector<Hit>* hits; double seconds; } sets[] = {
                { "the fill", &tomFill(), 3.2 }, { "three toms", &three, 0.8 }, { "two toms", &two, 0.8 }, { "one tom", &one, 0.8 },
            };
            for (const auto& set : sets)
            {
                std::vector<double> withLanes, without;
                for (int run = 0; run < 15; ++run)
                {
                    const bool lanesFirst = (run & 1) == 0;
                    const double first = playKit(lanesFirst, *set.hits, {}, at(set.seconds), false).seconds;
                    const double second = playKit(! lanesFirst, *set.hits, {}, at(set.seconds), false).seconds;
                    (lanesFirst ? withLanes : without).push_back(first);
                    (lanesFirst ? without : withLanes).push_back(second);
                }
                std::sort(withLanes.begin(), withLanes.end());
                std::sort(without.begin(), without.end());
                const double a = without[without.size() / 2], b = withLanes[withLanes.size() / 2];
                logMessage(juce::String(set.label).paddedRight(' ', 12) + "alone " + juce::String(1000.0 * a, 2) + " ms, in lanes "
                           + juce::String(1000.0 * b, 2) + " ms (medians of 15 paired runs, " + juce::String(set.seconds, 1)
                           + " s of sound at 96 kHz): " + juce::String(100.0 * (a - b) / a, 1) + " % less");
            }
        }
    }
};

static V3CircuitLanesTests v3CircuitLanesTests;
