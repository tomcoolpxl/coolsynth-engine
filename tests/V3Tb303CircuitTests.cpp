#include <juce_core/juce_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <vector>

#include "synth/v3/Tb303Envelope.h"
#include "synth/v3/Tb303Filter.h"
#include "synth/v3/Tb303Oscillator.h"
#include "synth/v3/Tb303Vca.h"
#include "synth/v3/Tb303Voice.h"

#include <complex>

/*
    The TB-303 as its circuit (D275, D279, D290, D293) against ngspice renders of the same schematics (tools/tb303's
    vcf.cir, vco.cir and vca.cir, rendered by tools/tb303/render.py; the references are in tests/data/tb303): the
    filter's nodes sample by sample, the oscillator's pitch, harmonics and aliasing, the envelope's and the VCA's
    nodes sample by sample.
*/
namespace
{
    using coolsynth::synth::v3::Tb303Filter;

    /** The voice's rate (D294, the user's choice: 96 kHz, the square's shaper at 8x of it). */
    constexpr double voiceRate = 96000.0;

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
        // A render faster than the voice is read at the voice's samples: every second of a 192 kHz one at 96 kHz.
        const int keep = static_cast<int>(std::lround(r.rate / voiceRate));
        if (keep > 1)
        {
            for (auto& column : r.columns)
            {
                std::vector<double> kept;
                for (std::size_t i = 0; i < column.second.size(); i += static_cast<std::size_t>(keep))
                    kept.push_back(column.second[i]);
                column.second = std::move(kept);
            }
            r.rate /= keep;
        }
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

    /** The saw's falling crossings of 8 V, in samples, interpolated (the ramp is smooth there: exact to a small part
        of a sample). */
    std::vector<double> sawCrossings(const std::vector<double>& saw, std::size_t from)
    {
        std::vector<double> t;
        for (std::size_t i = std::max<std::size_t>(from, 1); i < saw.size(); ++i)
            if (saw[i - 1] > 8.0 && saw[i] <= 8.0 && saw[i - 1] - saw[i] < 1.0)
                t.push_back(static_cast<double>(i - 1) + (saw[i - 1] - 8.0) / (saw[i - 1] - saw[i]));
        return t;
    }

    /** Harmonics 1..count in dBV over whole periods from a (the mean taken off). */
    std::vector<double> harmonicLevels(const std::vector<double>& x, double a, double b, double f0, double rate, int count)
    {
        const auto i0 = static_cast<std::size_t>(std::ceil(a));
        const double periods = std::round((b - a) * f0 / rate);
        const std::size_t i1 = std::min(x.size(), i0 + static_cast<std::size_t>(std::llround(periods * rate / f0)));
        double mean = 0.0;
        for (std::size_t i = i0; i < i1; ++i)
            mean += x[i];
        mean /= static_cast<double>(i1 - i0);
        std::vector<double> out;
        for (int h = 1; h <= count; ++h)
        {
            double re = 0.0, im = 0.0;
            for (std::size_t i = i0; i < i1; ++i)
            {
                const double phase = juce::MathConstants<double>::twoPi * h * f0 * (static_cast<double>(i) / rate);
                re += (x[i] - mean) * std::cos(phase);
                im += (x[i] - mean) * std::sin(phase);
            }
            out.push_back(20.0 * std::log10(std::max(1.0e-12, 2.0 * std::sqrt(re * re + im * im) / static_cast<double>(i1 - i0))));
        }
        return out;
    }

    /** The strongest component below 20 kHz that is not a harmonic of f0, in dB re the fundamental (a 4-term
        Blackman-Harris window over the last 2^17 samples). */
    double strongestAlias(const std::vector<double>& x, double rate, double f0)
    {
        std::size_t n = 1u << 17;
        while (n > x.size())
            n >>= 1;
        std::vector<std::complex<double>> a(n);
        const std::size_t start = x.size() - n;
        double mean = 0.0;
        for (std::size_t i = 0; i < n; ++i)
            mean += x[start + i];
        mean /= static_cast<double>(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            const double t = juce::MathConstants<double>::twoPi * static_cast<double>(i) / static_cast<double>(n - 1);
            const double w = 0.35875 - 0.48829 * std::cos(t) + 0.14128 * std::cos(2.0 * t) - 0.01168 * std::cos(3.0 * t);
            a[i] = (x[start + i] - mean) * w;
        }
        for (std::size_t i = 1, j = 0; i < n; ++i) // an in-place radix-2 FFT
        {
            std::size_t bit = n >> 1;
            for (; j & bit; bit >>= 1)
                j ^= bit;
            j ^= bit;
            if (i < j)
                std::swap(a[i], a[j]);
        }
        for (std::size_t len = 2; len <= n; len <<= 1)
        {
            const double angle = -juce::MathConstants<double>::twoPi / static_cast<double>(len);
            const std::complex<double> wl(std::cos(angle), std::sin(angle));
            for (std::size_t i = 0; i < n; i += len)
            {
                std::complex<double> w(1.0);
                for (std::size_t j = 0; j < len / 2; ++j)
                {
                    const auto u = a[i + j], v = a[i + j + len / 2] * w;
                    a[i + j] = u + v;
                    a[i + j + len / 2] = u - v;
                    w *= wl;
                }
            }
        }
        const double bin = rate / static_cast<double>(n);
        double fundamental = 0.0, worst = 0.0;
        for (std::size_t k = 1; k < n / 2 && static_cast<double>(k) * bin <= 20000.0; ++k)
        {
            const double h = static_cast<double>(k) * bin / f0, apart = std::abs(h - std::round(h)) * f0 / bin;
            const double p = std::norm(a[k]);
            if (std::round(h) == 1.0 && apart <= 6.0)
                fundamental += p;
            else if (apart > 6.0)
                worst = std::max(worst, p);
        }
        return 10.0 * std::log10(std::max(1.0e-30, worst * 4.0 / std::max(1.0e-30, fundamental)));
    }
}

class V3Tb303CircuitTests final : public juce::UnitTest
{
public:
    V3Tb303CircuitTests() : juce::UnitTest("V3 TB-303 circuit", "V3") {}

    void runTest() override
    {
        beginTest("the_lighter_accuracy_stays_close_and_goes_back");
        {
            // The 303's Lighter accuracy (CoolSynth's D349): a resonant accented note at Lighter within -80 dB rms of
            // Best (measured -106.5 dB over a long pattern), and Lighter switched off again the same bytes as Best.
            using coolsynth::synth::v3::Tb303Voice;
            const auto render = [](int mode)
            {
                auto v = std::make_unique<Tb303Voice>();
                v->prepare(Tb303Voice::rate);
                if (mode > 0)
                    v->setLighter(true);
                if (mode == 2)
                    v->setLighter(false);
                Tb303Voice::Knobs k;
                k.resonance = 0.8;
                k.envMod = 0.7;
                v->setKnobs(k);
                v->settle(2.0);
                std::vector<double> out(19200);
                for (std::size_t i = 0; i < out.size(); ++i)
                    out[i] = v->process(2.0, i < 12000, true, false);
                return out;
            };
            const auto best = render(0), lighter = render(1), back = render(2);
            double diff = 0.0, power = 0.0;
            for (std::size_t i = 0; i < best.size(); ++i)
            {
                diff += (lighter[i] - best[i]) * (lighter[i] - best[i]);
                power += best[i] * best[i];
            }
            const double db = 10.0 * std::log10(std::max(1.0e-300, diff) / power);
            expectLessThan(db, -80.0, "Lighter against Best");
            expect(back == best, "Lighter switched off is Best");
            logMessage("Lighter against Best: " + juce::String(db, 1) + " dB rms");
        }

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
                expect(filter->fitsItsLimits(), "the filter's circuits fit their templates");
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
                const char* const nodes[] = { "ntp6", "nvcf", "nl4", "nr4", "nb10", "nb11", "n13" };
                std::map<juce::String, std::vector<double>> model;
                for (const auto* node : nodes)
                    model[node].assign(n, 0.0);
                // Written through plain pointers: a map's lookup each sample would cost more than the filter in the
                // Debug build, and the timing below is the filter's.
                double* out[7];
                for (int k = 0; k < 7; ++k)
                    out[k] = model[nodes[k]].data();
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
                    out[0][i] = filter->tp6();
                    out[1][i] = filter->output();
                    out[2][i] = filter->ladderTopLeft();
                    out[3][i] = filter->ladderTopRight();
                    out[4][i] = filter->q10Base();
                    out[5][i] = filter->q11Base();
                    out[6][i] = filter->c13();
                    worstNewton = std::max(worstNewton, filter->newtonSteps());
                    totalNewton += filter->newtonSteps();
                    converged = converged && filter->lastSolveConverged();
                }
                const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                const double freshShare = filter->freshShare();

                // The same sound solved exactly, every sample factoring afresh: the chord method's own error (D281).
                {
                    auto exact = std::make_unique<Tb303Filter>();
                    exact->setJacobianReuse(false);
                    exact->prepare(ref.rate);
                    exact->setKnobs(c.cutoff, c.resonance, c.envMod, 0.1642);
                    exact->settle(8.35, c.envelope, c.accent);
                    double e = 0.0;
                    std::vector<double> exactTp6(n);
                    for (std::size_t i = 0; i < n; ++i)
                    {
                        const double t = static_cast<double>(i) / ref.rate;
                        const double phase = t * freq - std::floor(t * freq);
                        const double in = t < 1.0e-6 ? 8.35 : 11.1 - 5.5 * phase;
                        const bool after = t > c.step;
                        exact->process(in, after ? c.envelope2 : c.envelope, after ? c.accent2 : c.accent);
                        exactTp6[i] = exact->tp6();
                        const double d = out[0][i] - exactTp6[i];
                        e += d * d;
                    }
                    const double share = std::sqrt(e / static_cast<double>(n)) / std::max(1.0e-12, swing(exactTp6));
                    logMessage(juce::String(c.file) + ": the chord method against the exact solve, TP6 " + juce::String(share * 1.0e6, 2)
                               + " ppm of its swing; " + juce::String(freshShare * 100.0, 2) + " % of the steps factored afresh");
                    expectLessThan(share, 1.0e-4, juce::String(c.file) + ": the chord method keeps to the exact solve");
                }
                juce::String line = juce::String(c.file) + ":";
                for (const auto* node : nodes)
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

        beginTest("the_oscillator_matches_spice");
        {
            // tools/tb303/vco.cir at low C, A and high C, and a slide up from A; the trimmers as calibrate.py sets them.
            struct OscillatorCase
            {
                const char* file;
                double cv, cv2, tcv;
                bool slide;
            };
            const OscillatorCase cases[] = {
                { "vco_c1.csv", 1.0, 1.0, 1.0, false },
                { "vco_a2.csv", 2.75, 2.75, 1.0, false },
                { "vco_c5.csv", 5.0, 5.0, 1.0, false },
                { "vco_slide.csv", 2.75, 3.75, 0.02, true },
            };
            using coolsynth::synth::v3::Tb303Oscillator;
            for (const auto& c : cases)
            {
                const auto file = referenceFolder().getChildFile(c.file);
                if (! file.existsAsFile())
                {
                    expect(false, juce::String("the SPICE reference is missing: ") + c.file);
                    continue;
                }
                const auto ref = loadReference(file);
                const std::size_t n = ref.size();
                auto osc = std::make_unique<Tb303Oscillator>();
                expect(osc->fitsItsLimits(), "the oscillator's circuits fit their templates");
                osc->prepare(ref.rate);
                osc->setTrimmers(0.5687, 0.7389, 0.5, 1.0e-6);
                osc->setSlide(c.slide);
                osc->settle(c.cv, (Tb303Oscillator::top - 8.0) / (Tb303Oscillator::top - 5.36)); // the render's C33 at 8 V
                std::vector<double> saw(n), square(n), cv(n);
                bool converged = true;
                const auto start = std::chrono::steady_clock::now();
                for (std::size_t i = 0; i < n; ++i)
                {
                    saw[i] = osc->process(static_cast<double>(i) / ref.rate > c.tcv ? c.cv2 : c.cv);
                    square[i] = osc->squareVolts();
                    cv[i] = osc->cvVolts();
                    converged = converged && osc->lastSolveConverged();
                }
                const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                expect(converged, juce::String(c.file) + ": every sample converged");
                const auto& spiceSaw = ref.columns.at("nsaw");
                const auto& spiceSquare = ref.columns.at("nsq");
                const auto& spiceCv = ref.columns.at("ncv");
                double cvError = 0.0;
                for (std::size_t i = 0; i < n; ++i)
                    cvError = std::max(cvError, std::abs(cv[i] - spiceCv[i]));
                expectLessThan(cvError, 2.0e-3, juce::String(c.file) + ": the CV (and the slide) follows SPICE's");
                juce::String line = juce::String(c.file) + ": CV within " + juce::String(cvError * 1.0e3, 2) + " mV";
                if (! c.slide) // the slide's pitch moves to the end, so its CV stands for it
                {
                    const auto model = sawCrossings(saw, 0), spice = sawCrossings(spiceSaw, 0);
                    const std::size_t k = std::min(model.size(), spice.size()) - 1;
                    expect(k >= 3, juce::String(c.file) + ": enough periods");
                    if (k < 3)
                        continue;
                    const double fm = ref.rate * static_cast<double>(k - 1) / (model[k] - model[1]);
                    const double fs = ref.rate * static_cast<double>(k - 1) / (spice[k] - spice[1]);
                    const double cents = 1200.0 * std::log2(fm / fs);
                    expectLessThan(std::abs(cents), 0.5, juce::String(c.file) + ": the pitch");
                    const std::size_t first = k >= 3 ? k - 3 : 1;
                    const auto hs = harmonicLevels(saw, model[first], model[k], fm, ref.rate, 10);
                    const auto hss = harmonicLevels(spiceSaw, spice[first], spice[k], fs, ref.rate, 10);
                    const auto hq = harmonicLevels(square, model[first], model[k], fm, ref.rate, 10);
                    const auto hqs = harmonicLevels(spiceSquare, spice[first], spice[k], fs, ref.rate, 10);
                    double worstSaw = 0.0, worstSquare = 0.0;
                    for (int h = 0; h < 10; ++h)
                    {
                        worstSaw = std::max(worstSaw, std::abs(hs[static_cast<std::size_t>(h)] - hss[static_cast<std::size_t>(h)]));
                        // A square's weak harmonics (below -25 dB re its first) are judged within 3 dB, the rest 0.5 dB.
                        const double d = std::abs(hq[static_cast<std::size_t>(h)] - hqs[static_cast<std::size_t>(h)]);
                        const bool weak = hqs[static_cast<std::size_t>(h)] - hqs[0] < -25.0;
                        expectLessThan(d, weak ? 3.0 : 0.5, juce::String(c.file) + ": the square's harmonic " + juce::String(h + 1));
                        worstSquare = std::max(worstSquare, weak ? 0.0 : d);
                    }
                    expectLessThan(worstSaw, 0.3, juce::String(c.file) + ": the saw's harmonics 1 to 10");
                    line << "; pitch " << juce::String(cents, 3) << " cents; harmonics 1 to 10 within " << juce::String(worstSaw, 2)
                         << " dB (saw), " << juce::String(worstSquare, 2) << " dB (square's strong ones)";
                }
                line << " | " << juce::String(1.0e9 * seconds / static_cast<double>(n), 0) << " ns a sample, "
                     << juce::String(100.0 * seconds * ref.rate / static_cast<double>(n), 1) << " % of real time";
                logMessage(line);
            }

            // Aliasing (D290), the user's bar at the 4x shaper: the square's strongest alias below 20 kHz at most -68 dB
            // re its fundamental at the top note, the saw's at most -85 dB.
            auto osc = std::make_unique<Tb303Oscillator>();
            osc->prepare(voiceRate);
            osc->setTrimmers(0.5687, 0.7389, 0.5, 1.0e-6);
            osc->settle(5.0);
            std::vector<double> saw(static_cast<std::size_t>(voiceRate) * 2), square(saw.size());
            for (std::size_t i = 0; i < saw.size(); ++i)
            {
                saw[i] = osc->process(5.0);
                square[i] = osc->squareVolts();
            }
            const auto crossings = sawCrossings(saw, saw.size() / 2);
            const double f0 = voiceRate * static_cast<double>(crossings.size() - 1) / (crossings.back() - crossings.front());
            const double sawAlias = strongestAlias(saw, voiceRate, f0), squareAlias = strongestAlias(square, voiceRate, f0);
            logMessage("high C (" + juce::String(f0, 1) + " Hz): strongest alias below 20 kHz, saw " + juce::String(sawAlias, 1)
                       + " dB, square " + juce::String(squareAlias, 1) + " dB re the fundamental");
            expectLessThan(sawAlias, -85.0, "the saw's aliasing at high C");
            expectLessThan(squareAlias, -68.0, "the square's aliasing at high C");
        }

        beginTest("the_envelope_and_the_vca_match_spice");
        {
            // tools/tb303/vca.cir (the envelope with the VCA's control around it): the gate from 10 ms to 72.5 ms, an
            // accent to 135 ms in the second case, a 440 Hz sine of 0.3 V as the filter's output (the RESONANCE wiper
            // half of it); rendered at 96 kHz (48 kHz for the long decay), the model at the voice's rate read at the
            // same times.
            struct AmpCase
            {
                const char* file;
                double decay, accent;
                bool accented;
                double amplitude;
            };
            const AmpCase cases[] = {
                { "vca_plain.csv", 0.5, 0.5, false, 0.3 },
                { "vca_accent.csv", 0.5, 1.0, true, 0.3 },
                { "vca_longdecay.csv", 1.0, 0.5, false, 0.0 },
            };
            using coolsynth::synth::v3::Tb303Envelope;
            using coolsynth::synth::v3::Tb303Vca;
            const double rate = voiceRate; constexpr double gateOn = 0.010, gateOff = 0.0725, accentOff = 0.135;
            for (const auto& c : cases)
            {
                const auto file = referenceFolder().getChildFile(c.file);
                if (! file.existsAsFile())
                {
                    expect(false, juce::String("the SPICE reference is missing: ") + c.file);
                    continue;
                }
                const auto ref = loadReference(file);
                const int stride = static_cast<int>(std::lround(rate / ref.rate));
                const std::size_t n = ref.size() * static_cast<std::size_t>(stride);
                auto env = std::make_unique<Tb303Envelope>();
                auto vca = std::make_unique<Tb303Vca>();
                expect(env->fitsItsLimits() && vca->fitsItsLimits(), "the envelope's and the VCA's circuits fit their templates");
                env->prepare(rate);
                vca->prepare(rate);
                expect(env->isWellPosed() && vca->isWellPosed(), "no group of nodes only junctions leave");
                env->setKnobs(c.decay, c.accent);
                env->settle();
                vca->settle(env->pinOne());
                const char* const nodes[] = { "nenv", "nb31", "ngate", "nout" };
                std::map<juce::String, std::vector<double>> model;
                bool converged = true;
                double envelopeSeconds = 0.0, vcaSeconds = 0.0;
                for (std::size_t i = 0; i < n; ++i)
                {
                    const double t = static_cast<double>(i) / rate;
                    const auto a = std::chrono::steady_clock::now();
                    env->setAccent(c.accented && t > gateOn && t < accentOff);
                    env->process(t > gateOn && t < gateOff);
                    const auto b = std::chrono::steady_clock::now();
                    const double audio = c.amplitude * std::sin(juce::MathConstants<double>::twoPi * 440.0 * t);
                    const double out = vca->process(audio, 0.5 * audio, env->pinOne());
                    const auto e = std::chrono::steady_clock::now();
                    envelopeSeconds += std::chrono::duration<double>(b - a).count();
                    vcaSeconds += std::chrono::duration<double>(e - b).count();
                    converged = converged && env->lastSolveConverged() && vca->lastSolveConverged();
                    if (i % static_cast<std::size_t>(stride) == 0)
                    {
                        model["nenv"].push_back(env->envelope());
                        model["nb31"].push_back(env->c42());
                        model["ngate"].push_back(env->gateLine());
                        model["nout"].push_back(out);
                    }
                }
                expect(converged, juce::String(c.file) + ": every step converged");
                juce::String line = juce::String(c.file) + ":";
                auto rmsError = [&](const char* node)
                {
                    const auto& x = model[node];
                    const auto& r = ref.columns.at(node);
                    const std::size_t len = std::min(x.size(), r.size());
                    double e = 0.0;
                    for (std::size_t i = 0; i < len; ++i)
                        e += (x[i] - r[i]) * (x[i] - r[i]);
                    return std::sqrt(e / static_cast<double>(std::max<std::size_t>(1, len)));
                };
                // The envelope, C42 (the volume's own decay) and GATE in millivolts, rms over the whole render (the
                // envelope's quarter rate puts it some 60 us late at the attack); the output relative to its swing.
                for (const auto* node : nodes)
                {
                    if (! ref.columns.count(node))
                        continue;
                    const double e = rmsError(node);
                    if (juce::String(node) == "nout")
                    {
                        const double share = e / std::max(1.0e-9, swing(ref.columns.at(node)));
                        line << " output " << juce::String(share * 100.0, 2) << " % of its swing";
                        expectLessThan(share, 0.02, juce::String(c.file) + ": the VCA's output");
                    }
                    else
                    {
                        line << " " << node << " " << juce::String(e * 1.0e3, 1) << " mV";
                        expectLessThan(e, juce::String(node) == "nb31" ? 0.03 : 0.06, juce::String(c.file) + ": " + node);
                    }
                }
                // The long decay's end: the envelope and C42 within a few millivolts.
                const double endEnv = model["nenv"].back() - ref.columns.at("nenv").back();
                const double endC42 = model["nb31"].back() - ref.columns.at("nb31").back();
                expectLessThan(std::abs(endEnv), 0.01, juce::String(c.file) + ": the envelope at the end");
                expectLessThan(std::abs(endC42), 0.01, juce::String(c.file) + ": C42 at the end");
                line << " | at the end " << juce::String(endEnv * 1.0e3, 1) << " mV, C42 " << juce::String(endC42 * 1.0e3, 1)
                     << " mV | envelope " << juce::String(100.0 * envelopeSeconds * rate / static_cast<double>(n), 1)
                     << " %, VCA " << juce::String(100.0 * vcaSeconds * rate / static_cast<double>(n), 1) << " % of real time";
                logMessage(line);
            }
        }

        beginTest("the_voice_matches_spice");
        {
            // tools/tb303/voice.cir, the whole voice (D295), at low C (2 V), the gate from 50 to 300 ms, the trimmers
            // as fitted: a plain saw with every knob at half, an accented square, a resonant saw (cutoff 30 %,
            // resonance full, env mod 70 %). Judged as heard: the pitch, the output's loudness in 5 ms blocks and its
            // harmonics 2 to 12 against the fundamental 100 ms into the note (a sample-by-sample comparison after a
            // resonant filter only measures a phase drift).
            using coolsynth::synth::v3::Tb303Voice;
            Tb303Voice::Knobs half;
            Tb303Voice::Knobs square = half;
            square.square = true;
            Tb303Voice::Knobs resonant = half;
            resonant.cutoff = 0.3;
            resonant.resonance = 1.0;
            resonant.envMod = 0.7;
            struct VoiceCase
            {
                const char* file;
                Tb303Voice::Knobs knobs;
                bool accented;
            };
            const VoiceCase cases[] = { { "voice_saw.csv", half, false },
                                        { "voice_square_accent.csv", square, true },
                                        { "voice_resonant.csv", resonant, false } };
            constexpr double note = 2.0, gateOn = 0.05, gateOff = 0.3;
            for (const auto& c : cases)
            {
                const auto file = referenceFolder().getChildFile(c.file);
                if (! file.existsAsFile())
                {
                    expect(false, juce::String("the SPICE reference is missing: ") + c.file);
                    continue;
                }
                const auto ref = loadReference(file);
                const std::size_t n = ref.size();
                auto voice = std::make_unique<Tb303Voice>();
                expect(voice->fitsItsLimits(), "the voice's circuits fit their templates");
                voice->prepare(ref.rate);
                voice->setKnobs(c.knobs);
                voice->settle(note);
                std::vector<double> out(n), wave(n);
                bool converged = true;
                const auto start = std::chrono::steady_clock::now();
                for (std::size_t i = 0; i < n; ++i)
                {
                    const double t = static_cast<double>(i) / ref.rate;
                    const bool gate = t > gateOn && t < gateOff;
                    out[i] = voice->process(note, gate, c.accented && gate, false);
                    wave[i] = voice->waveform();
                    converged = converged && voice->lastSolveConverged();
                }
                const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                expect(converged, juce::String(c.file) + ": every sample converged");
                const auto& spiceOut = ref.columns.at("nout");
                const auto& spiceWave = ref.columns.at("nwave");
                juce::String line = juce::String(c.file) + ":";

                // The pitch, from the waveform's own period (the saw's falls through 8 V, the square's rises through 7 V).
                auto period = [&](const std::vector<double>& w, std::size_t from, int count, double& first)
                {
                    std::vector<double> t;
                    for (std::size_t i = from; i < w.size() && static_cast<int>(t.size()) < count; ++i)
                    {
                        const double level = c.knobs.square ? 7.0 : 8.0;
                        const bool hit = c.knobs.square ? (w[i - 1] < level && w[i] >= level)
                                                        : (w[i - 1] > level && w[i] <= level && w[i - 1] - w[i] < 1.0);
                        if (hit)
                            t.push_back(static_cast<double>(i - 1) + (level - w[i - 1]) / (w[i] - w[i - 1]));
                    }
                    first = t.empty() ? 0.0 : t.front();
                    return t.size() < 2 ? 0.0 : (t.back() - t.front()) / static_cast<double>(t.size() - 1);
                };
                double firstModel = 0.0, firstSpice = 0.0;
                const auto from = static_cast<std::size_t>((gateOn + 0.1) * ref.rate);
                const double pm = period(wave, from, 6, firstModel), ps = period(spiceWave, from, 6, firstSpice);
                const double cents = 1200.0 * std::log2(ps / std::max(1.0e-9, pm));
                expectLessThan(std::abs(cents), 0.5, juce::String(c.file) + ": the pitch");
                line << " pitch " << juce::String(cents, 2) << " cents;";

                // The loudness in 5 ms blocks where SPICE's is within 40 dB of its loudest.
                const auto block = static_cast<std::size_t>(0.005 * ref.rate);
                std::vector<double> lm, ls;
                for (std::size_t s = 0; s + block <= n; s += block)
                {
                    double p = 0.0, q = 0.0;
                    for (std::size_t i = s; i < s + block; ++i)
                    {
                        p += out[i] * out[i];
                        q += spiceOut[i] * spiceOut[i];
                    }
                    lm.push_back(10.0 * std::log10(std::max(1.0e-30, p / static_cast<double>(block))));
                    ls.push_back(10.0 * std::log10(std::max(1.0e-30, q / static_cast<double>(block))));
                }
                const double loudest = *std::max_element(ls.begin(), ls.end());
                double sum = 0.0, worst = 0.0;
                int counted = 0;
                for (std::size_t j = 0; j < ls.size(); ++j)
                    if (ls[j] > loudest - 40.0)
                    {
                        const double d = lm[j] - ls[j];
                        sum += d * d;
                        ++counted;
                        worst = std::max(worst, std::abs(d));
                    }
                const double loudnessRms = std::sqrt(sum / std::max(1, counted));
                // The filter's own residual (1.5 to 5 % against vcf.cir) shows here as up to 1.5 dB on a harmonic and 0.6 dB
                // of loudness on the accented square; the same at 192 kHz, so neither the rate nor the wiring.
                expectLessThan(loudnessRms, 1.0, juce::String(c.file) + ": the loudness over the note");
                line << " loudness " << juce::String(loudnessRms, 2) << " dB rms (worst " << juce::String(worst, 2) << ");";

                // The harmonics 2 to 12 against the fundamental over four periods, each at its own pitch.
                const auto hm = harmonicLevels(out, firstModel, firstModel + 4.0 * pm, ref.rate / pm, ref.rate, 12);
                const auto hs = harmonicLevels(spiceOut, firstSpice, firstSpice + 4.0 * ps, ref.rate / ps, ref.rate, 12);
                double worstHarmonic = 0.0;
                for (int h = 1; h < 12; ++h)
                {
                    const auto k = static_cast<std::size_t>(h);
                    if (hs[k] - hs[0] < -30.0) // weaker than 30 dB under the fundamental: not judged
                        continue;
                    const double d = std::abs((hm[k] - hm[0]) - (hs[k] - hs[0]));
                    worstHarmonic = std::max(worstHarmonic, d);
                    expectLessThan(d, 1.75, juce::String(c.file) + ": harmonic " + juce::String(h + 1));
                }
                line << " harmonics 2 to 12 within " << juce::String(worstHarmonic, 2) << " dB | "
                     << juce::String(100.0 * seconds * ref.rate / static_cast<double>(n), 1) << " % of real time";
                logMessage(line);
            }
        }
    }
};

static V3Tb303CircuitTests v3Tb303CircuitTests;
