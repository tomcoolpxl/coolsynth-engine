#include <juce_core/juce_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <vector>

#include "synth/v3/Tb303Filter.h"

/*
    The TB-303 as its circuit (D275, D279) against ngspice renders of the same schematic (tools/tb303/vcf.cir,
    rendered by tools/tb303/render.py at 192 kHz; the references are in tests/data/tb303): each node's swing about
    the reference's mean, sample by sample, over 60 ms.
*/
namespace
{
    using coolsynth::synth::v3::Tb303Filter;

    struct Reference
    {
        std::map<juce::String, std::vector<double>> columns;
        double rate = 192000.0;
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

    /** The rms difference of the swings about the reference's mean, over the reference swing's rms; the means'
        difference apart, in volts. */
    std::pair<double, double> swingError(const std::vector<double>& model, const std::vector<double>& spice)
    {
        const std::size_t n = std::min(model.size(), spice.size());
        double mean = 0.0, modelMean = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            mean += spice[i];
            modelMean += model[i];
        }
        mean /= static_cast<double>(n);
        modelMean /= static_cast<double>(n);
        double e = 0.0, s = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const double a = model[i] - modelMean, b = spice[i] - mean;
            e += (a - b) * (a - b);
            s += b * b;
        }
        return { std::sqrt(e / std::max(1.0e-30, s)), modelMean - mean };
    }

    /** The rms of a signal about its mean. */
    double swing(const std::vector<double>& x)
    {
        double m = 0.0, s = 0.0;
        for (const double v : x)
            m += v;
        m /= static_cast<double>(x.size());
        for (const double v : x)
            s += (v - m) * (v - m);
        return std::sqrt(s / static_cast<double>(x.size()));
    }

    struct FilterCase
    {
        const char* file;
        double cutoff, resonance, envMod, envelope, accent, envelope2, accent2, step;
    };
}

class V3Tb303CircuitTests final : public juce::UnitTest
{
public:
    V3Tb303CircuitTests() : juce::UnitTest("V3 TB-303 circuit", "V3") {}

    void runTest() override
    {
        beginTest("the_filter_matches_spice");
        {
            const FilterCase cases[] = {
                { "ring.csv", 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },
                { "envmod.csv", 0.3, 0.7, 0.6, 8.0, 0.0, 8.0, 0.0, 1.0 },
                { "accent.csv", 0.4, 0.5, 0.5, 0.0, 0.0, 8.0, 6.0, 0.02 },
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
                auto filter = std::make_unique<Tb303Filter>();
                filter->prepare(ref.rate);
                filter->setKnobs(c.cutoff, c.resonance, c.envMod, 0.1642);
                const double freq = 65.406;
                const auto tolerance = juce::SystemStats::getEnvironmentVariable("COOLSYNTH_303_TOLERANCE", {});
                if (tolerance.isNotEmpty())
                    filter->setNewtonTolerance(tolerance.getDoubleValue());
                filter->settle(8.35, c.envelope, c.accent);
                const bool debug = juce::SystemStats::getEnvironmentVariable("COOLSYNTH_303_DEBUG", {}).isNotEmpty();
                auto state = [&](const juce::String& when)
                {
                    logMessage(when + ": tp6 " + juce::String(filter->tp6(), 5) + " l4 " + juce::String(filter->ladderTopLeft(), 5)
                               + " r4 " + juce::String(filter->ladderTopRight(), 5) + " b10 " + juce::String(filter->q10Base(), 5)
                               + " b11 " + juce::String(filter->q11Base(), 5) + " I " + juce::String(filter->ladderCurrent() * 1e6, 3)
                               + " uA, Newton " + juce::String(filter->newtonSteps()) + (filter->lastSolveConverged() ? "" : " NOT converged"));
                };
                if (debug)
                {
                    state(juce::String(c.file) + " settled");
                    logMessage("  reference at 0: tp6 " + juce::String(ref.columns.at("ntp6")[0], 5) + " l4 "
                               + juce::String(ref.columns.at("nl4")[0], 5) + " b10 " + juce::String(ref.columns.at("nb10")[0], 5)
                               + " b11 " + juce::String(ref.columns.at("nb11")[0], 5));
                }
                const std::size_t n = ref.size();
                std::map<juce::String, std::vector<double>> model;
                int worstNewton = 0;
                long totalNewton = 0;
                bool converged = true;
                const long analysesBefore = coolsynth::synth::v3::wdf::analysesOnThisThread;
                const auto start = std::chrono::steady_clock::now();
                for (std::size_t i = 0; i < n; ++i)
                {
                    const double t = static_cast<double>(i) / ref.rate;
                    const double phase = t * freq - std::floor(t * freq);
                    const double in = t < 1.0e-6 ? 8.35 : 11.1 - 5.5 * phase;
                    const bool after = t > c.step;
                    filter->process(in, after ? c.envelope2 : c.envelope, after ? c.accent2 : c.accent);
                    if (debug && i < 6)
                        state("  sample " + juce::String(static_cast<int>(i)));
                    if (debug && i == 6)
                        break;
                    model["ntp6"].push_back(filter->tp6());
                    model["nvcf"].push_back(filter->output());
                    model["nl4"].push_back(filter->ladderTopLeft());
                    model["nr4"].push_back(filter->ladderTopRight());
                    model["nb10"].push_back(filter->q10Base());
                    model["nb11"].push_back(filter->q11Base());
                    model["n13"].push_back(filter->c13());
                    worstNewton = std::max(worstNewton, filter->newtonSteps());
                    totalNewton += filter->newtonSteps();
                    converged = converged && filter->lastSolveConverged();
                }
                const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                juce::String line = juce::String(c.file) + ":";
                for (const auto* node : { "ntp6", "nvcf", "nl4", "nr4", "nb10", "nb11", "n13" })
                {
                    const auto [err, offset] = swingError(model[node], ref.columns.at(node));
                    line << " " << node << " " << juce::String(err, 4) << " (" << juce::String(offset * 1e3, 2) << " mV)";
                    // A node that holds still in the reference (the converter's with the envelope steady) is judged by
                    // its level; TP6's level aside (C14 takes it off; 31 mV low here, its far side within 2 mV).
                    if (swing(ref.columns.at(node)) < 1.0e-3)
                        expectLessThan(std::abs(offset), 2.0e-3, juce::String(c.file) + " " + node + ", its level");
                    else
                        expectLessThan(err, 0.05, juce::String(c.file) + " " + node);
                }
                line << " | analyses " << (coolsynth::synth::v3::wdf::analysesOnThisThread - analysesBefore) << " | Newton most " << worstNewton << ", mean " << juce::String(static_cast<double>(totalNewton) / static_cast<double>(n), 2)
                     << " | " << juce::String(1.0e9 * seconds / static_cast<double>(n), 0) << " ns a sample at "
                     << juce::String(ref.rate / 1000.0, 0) << " kHz, " << juce::String(100.0 * seconds * ref.rate / static_cast<double>(n), 1)
                     << " % of real time";
                logMessage(line);
                expect(converged, juce::String(c.file) + ": every sample converged");
            }
        }
    }
};

static V3Tb303CircuitTests v3Tb303CircuitTests;
