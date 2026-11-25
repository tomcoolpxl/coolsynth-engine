#include <juce_core/juce_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "diagnostics/AudioThreadAlarm.h"
#include "synth/v3/Tb303Lid.h"

/*
    The TB-303's lid tap (Tb303Lid.h): every series of the lid drawing served, its map checked against the voice's
    circuits, and its values against ngspice's recording of the drawing's phrase (tools/tb303/mockup_currents.py
    --csv: tests/data/tb303/lid.csv, each series' peak over the run and its exact values every 5 ms, the first the
    operating point). The voice as the recording sets it: voice.cir's knobs and trimmers (TM3 0.1642, C11 1 uF) at
    96 kHz.
*/
namespace
{
    using coolsynth::synth::v3::Tb303Lid;
    using coolsynth::synth::v3::Tb303Voice;

    constexpr double lidRate = 96000.0;

    // The recording's phrase (mockup_currents.py): 120 BPM sixteenths, a plain A, an accented E slid into the A an
    // octave up; its sources' edges as its PWL points have them (the old value at the point, the new one after it).
    constexpr double t1 = 0.010, t2 = 0.135, t3 = 0.260, off3 = 0.3225;

    struct Step
    {
        double cv = 2.75;
        bool gate = false, accent = false, slide = false;
    };

    Step phraseAt(double t)
    {
        Step s;
        s.cv = t <= t2 - 0.004 ? 2.75 : t <= t3 ? 2.75 + 7.0 / 12.0 : 3.75;
        s.gate = (t > t1 && t <= t1 + 0.0625) || (t > t2 && t <= off3);
        s.accent = t > t2 && t <= t3;
        s.slide = t > t3 && t <= t3 + 0.125;
        return s;
    }

    std::unique_ptr<Tb303Voice> makeVoice(bool fish = false)
    {
        auto v = std::make_unique<Tb303Voice>(fish);
        v->prepare(lidRate);
        Tb303Voice::Knobs k; // the recording's setting (the app's defaults)
        k.cutoff = 0.42;
        k.resonance = 0.72;
        k.envMod = 0.55;
        k.decay = 0.48;
        k.accent = 0.62;
        k.tuning = 0.5;
        k.square = false;
        v->setKnobs(k);
        Tb303Voice::Trimmers trim; // voice.cir's
        trim.tm3 = 0.1642;
        trim.tm4 = 0.5687;
        trim.tm5 = 0.7389;
        trim.c11 = 1.0e-6;
        v->setTrimmers(trim);
        v->settle(2.75);
        return v;
    }

    struct Reference
    {
        std::vector<std::string> names;
        std::vector<double> peak, ms;
        std::vector<std::vector<double>> values; // [series][moment]
    };

    Reference loadReference()
    {
        Reference r;
        const auto file = juce::File(__FILE__).getParentDirectory().getChildFile("data").getChildFile("tb303").getChildFile("lid.csv");
        juce::StringArray lines;
        file.readLines(lines);
        if (lines.size() < 2)
            return r;
        const auto head = juce::StringArray::fromTokens(lines[0], ",", "");
        for (int c = 2; c < head.size(); ++c)
            r.ms.push_back(head[c].getDoubleValue());
        for (int i = 1; i < lines.size(); ++i)
        {
            const auto f = juce::StringArray::fromTokens(lines[i], ",", "");
            if (f.size() != head.size())
                continue;
            r.names.push_back(f[0].toStdString());
            r.peak.push_back(f[1].getDoubleValue());
            std::vector<double> v;
            for (int c = 2; c < f.size(); ++c)
                v.push_back(f[c].getDoubleValue());
            r.values.push_back(std::move(v));
        }
        return r;
    }

    bool isAmps(const std::string& name)
    {
        for (const char* end : { ".i", ".id", ".ig", ".ic", ".ib" })
        {
            const std::string e = end;
            if (name.size() > e.size() && name.compare(name.size() - e.size(), e.size(), e) == 0)
                return true;
        }
        return false;
    }

    juce::String si(double x, bool amps)
    {
        const double a = std::abs(x);
        if (! amps)
            return a >= 1.0 ? juce::String(x, 3) + " V" : juce::String(x * 1.0e3, 2) + " mV";
        if (a >= 1.0e-3)
            return juce::String(x * 1.0e3, 3) + " mA";
        if (a >= 1.0e-6)
            return juce::String(x * 1.0e6, 3) + " uA";
        return juce::String(x * 1.0e9, 2) + " nA";
    }
}

class V3Tb303LidTests final : public juce::UnitTest
{
public:
    V3Tb303LidTests() : juce::UnitTest("TB-303 lid", "V3") {}

    void runTest() override
    {
        beginTest("every_name_is_served");
        {
            const auto ref = loadReference();
            expect(ref.names.size() == 377, "the reference has the drawing's 377 series");
            expectEquals(Tb303Lid::count(), 377);
            std::set<std::string> seen;
            int differing = 0;
            for (int i = 0; i < Tb303Lid::count(); ++i)
            {
                const std::string n = Tb303Lid::name(i);
                seen.insert(n);
                if (static_cast<std::size_t>(i) >= ref.names.size() || ref.names[static_cast<std::size_t>(i)] != n)
                {
                    if (differing++ < 5)
                        expect(false, juce::String("series ") + juce::String(i) + " is " + n.c_str() + ", the drawing's "
                                          + (static_cast<std::size_t>(i) < ref.names.size() ? ref.names[static_cast<std::size_t>(i)].c_str() : "none"));
                }
            }
            expectEquals(differing, 0, "series named and ordered as the drawing's");
            expectEquals(static_cast<int>(seen.size()), Tb303Lid::count(), "every name once");
            for (const bool fish : { false, true })
            {
                auto voice = makeVoice(fish);
                std::vector<float> out(static_cast<std::size_t>(Tb303Lid::count()), -1.0e30f);
                for (int i = 0; i < 4800; ++i)
                {
                    const auto s = phraseAt(0.135 + static_cast<double>(i) / lidRate);
                    voice->process(s.cv, s.gate, s.accent, s.slide);
                }
                Tb303Lid::read(*voice, out.data());
                int bad = 0;
                for (const float x : out)
                    bad += std::isfinite(x) && std::abs(x) < 1.0e3f ? 0 : 1;
                expectEquals(bad, 0, juce::String(fish ? "a Devil Fish" : "the stock voice") + ": every value read, finite");
            }
        }

        beginTest("the_map_matches_the_circuits");
        {
            for (const bool fish : { false, true })
            {
                auto voice = makeVoice(fish);
                const char* first = "";
                const int wrong = Tb303Lid::check(*voice, &first);
                expectEquals(wrong, 0, juce::String(fish ? "a Devil Fish" : "the stock voice") + ": ports as the map has them (first wrong: "
                                           + first + ")");
            }
        }

        beginTest("the_values_match_spice");
        {
            const auto ref = loadReference();
            if (ref.names.size() != static_cast<std::size_t>(Tb303Lid::count()) || ref.ms.empty())
            {
                expect(false, "the SPICE reference tests/data/tb303/lid.csv is missing or not the drawing's");
                return;
            }
            const std::size_t series = ref.names.size(), moments = ref.ms.size();
            std::vector<std::vector<double>> model(series, std::vector<double>(moments, 0.0));
            std::vector<float> out(series);
            auto voice = makeVoice();
            auto take = [&](std::size_t m)
            {
                Tb303Lid::read(*voice, out.data());
                for (std::size_t s = 0; s < series; ++s)
                    model[s][m] = out[s];
            };
            take(0); // the operating point
            const auto last = static_cast<long>(std::lround(ref.ms.back() * 1.0e-3 * lidRate));
            std::size_t next = 1;
            bool converged = true;
            for (long k = 1; k <= last && next < moments; ++k)
            {
                const double t = static_cast<double>(k) / lidRate;
                const auto s = phraseAt(t);
                voice->process(s.cv, s.gate, s.accent, s.slide);
                converged = converged && voice->lastSolveConverged();
                if (k == std::lround(ref.ms[next] * 1.0e-3 * lidRate))
                    take(next++);
            }
            expect(converged, "every sample converged");
            expect(next == moments, "every moment taken");

            // Each series judged against its own peak over the run (as the drawing scales it): at rest (the operating
            // point) within 2 % of it, over the phrase within 25 % of it at every moment, the 5 ms moments falling at
            // any phase of the oscillator and of the filter's ring; the engine's known residuals with allowances of
            // their own (below). A few series are left out where the engine has no such part (Tb303Lid.h): those are
            // reported, not judged.
            struct Miss
            {
                double rest = 0.0, phrase = 0.0;
                std::size_t at = 0;
            };
            std::vector<Miss> miss(series);
            for (std::size_t s = 0; s < series; ++s)
            {
                const bool amps = isAmps(ref.names[s]);
                const double floor = amps ? 1.0e-9 : 1.0e-3; // a nanoamp, a millivolt
                const double scale = std::max(ref.peak[s], floor);
                miss[s].rest = std::abs(model[s][0] - ref.values[s][0]) / scale;
                for (std::size_t m = 1; m < moments; ++m)
                {
                    const double e = std::abs(model[s][m] - ref.values[s][m]) / scale;
                    if (e > miss[s].phrase)
                    {
                        miss[s].phrase = e;
                        miss[s].at = m;
                    }
                }
            }
            auto unjudged = [](const std::string& n)
            {
                static const std::set<std::string> list = {
                    // the thyristor, an event in the engine (its standing leaks)
                    "xvco.q25.ic", "xvco.q25.ib", "xvco.q27.ic", "xvco.q27.ib", "xvco.d25.id", "xvco.q24.ic",
                    "xvco.q24.ib", "xvco.r101.i", "xvco.nb", "xvco.nb27", "xvco.nk", "xvco.ne24",
                    // J28's gate charge, Q39's gate leak
                    "xvco.j28.ig", "xvca.j39.ig",
                };
                return list.count(n) > 0;
            };
            // The engine's known residuals, each series' allowance at rest and over the phrase, of its peak.
            struct Allowance
            {
                double rest, phrase;
            };
            auto allowance = [](const std::string& n) -> Allowance
            {
                // The filter's 2SC945s carry Gummel-Poon's base recombination since CoolSynth's D360 (their base
                // currents, R66's and R99's, Q18's emitter within the default allowances, at rest within 1.3 %). The
                // accented note's ring on C14's side at 150 ms, its amplitude under ngspice's (vcf.cir's accent case
                // within 1.1 % of its swing): 24.4 % there before D360, 26.8 % after, the moment at the ring's phase.
                // With ENV MOD's share on its fitted track (0.235 at the default knob, CoolSynth's D364) the ladder's
                // C24 at that moment too: 28.7 %.
                static const std::set<std::string> accentRing = {
                    "nvcf", "nresw", "xvcf.rres1.i", "xvcf.rres3.i", "xvcf.c14.i", "xvcf.c15.i", "xvcf.r67.i", "xvcf.c22.i",
                    "xvcf.c24.i" };
                // Nodes that only nanoamps hold at rest, where the engine's 1 GOhm leaks (SPICE's GMIN is 1 TOhm) and
                // the missing junction leakage place them: the BA662's mirrors with no tail current, the diode
                // string, Q41's emitter.
                static const std::set<std::string> floating = {
                    "xvca.xic15.x", "xvca.xic15.b6", "xvca.xic15.t", "xvca.xic15.ca", "xvca.xic15.cb", "xvca.nn3",
                    "xvca.nn4", "xvca.ne41" };
                // At rest the filter settles on the saw's mean (8.35 V), ngspice on the saw at its start (8.10 V).
                static const std::set<std::string> settledInput = { "nwave", "xvcf.n17" };
                if (accentRing.count(n) > 0)
                    return { 0.02, 0.32 };
                if (settledInput.count(n) > 0)
                    return { 0.035, 0.25 };
                if (floating.count(n) > 0)
                    return { 0.2, 0.25 };
                if (n == "xvco.q26a.ib") // Q26's junction charge, which the saw's resets kick (not in the engine)
                    return { 0.02, 0.75 };
                if (n == "xvcf.r68.i") // C16's few mV of ripple through 2.2k, at the ring's phase
                    return { 0.02, 0.7 };
                if (n == "xvca.niabc") // the VCA's control falls a few ms sooner after the gate (D293's 30 mV)
                    return { 0.02, 0.55 };
                if (n == "xvcf.c19.i") // the ladder's top capacitor, the ring itself: at the default knob's resonance
                    return { 0.02, 0.4 }; // on the fitted track (0.87 of the pot, CoolSynth's D324) its phase drift shows
                // The voice's residual (the filter 0.4 to 1.1 % off vcf.cir since D360, the ring's amplitude under
                // ngspice's at the accent) at moments that fall at any phase of the oscillator and the ring.
                return { 0.02, 0.25 };
            };
            std::vector<std::size_t> order(series);
            for (std::size_t s = 0; s < series; ++s)
                order[s] = s;
            int failedRest = 0, failedPhrase = 0;
            for (std::size_t s = 0; s < series; ++s)
            {
                if (unjudged(ref.names[s]))
                    continue;
                const auto a = allowance(ref.names[s]);
                if (miss[s].rest > a.rest)
                {
                    ++failedRest;
                    expect(false, juce::String(ref.names[s].c_str()) + " at rest: " + juce::String(100.0 * miss[s].rest, 1) + " %");
                }
                if (miss[s].phrase > a.phrase)
                {
                    ++failedPhrase;
                    expect(false, juce::String(ref.names[s].c_str()) + " over the phrase: " + juce::String(100.0 * miss[s].phrase, 1) + " %");
                }
            }
            auto report = [&](bool rest)
            {
                std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b)
                          { return rest ? miss[a].rest > miss[b].rest : miss[a].phrase > miss[b].phrase; });
                juce::String line = rest ? "at rest, the worst against their peaks:" : "over the phrase, the worst against their peaks:";
                for (std::size_t j = 0; j < series; ++j)
                {
                    const std::size_t s = order[j];
                    if (j >= 16 && (rest ? miss[s].rest <= 0.02 : miss[s].phrase <= 0.25))
                        break;
                    const std::size_t m = rest ? 0 : miss[s].at;
                    const bool amps = isAmps(ref.names[s]);
                    line << "\n  " << ref.names[s].c_str() << (unjudged(ref.names[s]) ? " (not judged)" : "") << ": "
                         << juce::String(100.0 * (rest ? miss[s].rest : miss[s].phrase), 1) << " % of "
                         << si(ref.peak[s], amps) << " at " << juce::String(ref.ms[m], 0) << " ms (engine "
                         << si(model[s][m], amps) << ", ngspice " << si(ref.values[s][m], amps) << ")";
                }
                logMessage(line);
            };
            report(true);
            report(false);
            expectEquals(failedRest, 0, "at rest within 2 % of each series' peak (the known residuals' own allowances)");
            expectEquals(failedPhrase, 0, "over the phrase within 25 % of each series' peak (the known residuals' own allowances)");
        }

        beginTest("a_read_is_cheap_and_real_time_safe");
        {
            auto voice = makeVoice();
            std::vector<float> out(static_cast<std::size_t>(Tb303Lid::count()));
            const bool watching = coolsynth::diag::watchBlockingCalls();
            coolsynth::diag::resetAudioAllocations();
            // 2000 reads a second at 96 kHz, a read every 48 samples, in batches of 100: the fastest batch's mean is the
            // cost, the mean of all as the machine's other work let it run.
            constexpr int reads = 2000, samplesBetween = 48, batch = 100;
            double readSeconds = 0.0, sampleSeconds = 0.0, fastest = 1.0, batchSeconds = 0.0;
            {
                const coolsynth::diag::AudioBlockScope block;
                for (int r = 0; r < reads; ++r)
                {
                    const auto a = std::chrono::steady_clock::now();
                    for (int k = 0; k < samplesBetween; ++k)
                    {
                        const auto s = phraseAt(0.135 + static_cast<double>(r * samplesBetween + k) / lidRate);
                        voice->process(s.cv, s.gate, s.accent, s.slide);
                    }
                    const auto b = std::chrono::steady_clock::now();
                    Tb303Lid::read(*voice, out.data());
                    const auto c = std::chrono::steady_clock::now();
                    const double read = std::chrono::duration<double>(c - b).count();
                    sampleSeconds += std::chrono::duration<double>(b - a).count();
                    readSeconds += read;
                    batchSeconds += read;
                    if ((r + 1) % batch == 0)
                    {
                        fastest = std::min(fastest, batchSeconds / batch);
                        batchSeconds = 0.0;
                    }
                }
            }
            const int allocations = coolsynth::diag::audioAllocations.count.load();
            const int blocking = coolsynth::diag::audioBlockingCalls.count.load();
            expectEquals(allocations, 0, "no allocation in a read");
            if (watching)
                expectEquals(blocking, 0, "no lock, wait, sleep or file in a read");
            const double perRead = readSeconds / reads, perSample = sampleSeconds / (reads * samplesBetween);
            logMessage("a read: " + juce::String(fastest * 1.0e6, 2) + " us (" + juce::String(Tb303Lid::count())
                       + " series; the mean " + juce::String(perRead * 1.0e6, 2) + " us, a voice sample's "
                       + juce::String(perSample * 1.0e6, 2) + " us); at 2000 reads a second "
                       + juce::String(100.0 * fastest * 2000.0, 2) + " % of a core");
            expectLessThan(fastest, 10.0e-6, "a read under 10 us");
        }
    }
};

static V3Tb303LidTests v3Tb303LidTests;
