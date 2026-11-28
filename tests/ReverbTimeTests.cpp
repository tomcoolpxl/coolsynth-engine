#include <juce_core/juce_core.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "synth/PlateReverb.h"

/*
    The length of a reverb's tail as its Size knob shows it (CoolSynth's D373): the time the plate says
    for a Size against the time measured on its impulse response, by Schroeder's backward integration (the level
    of the energy still to come), its fall from -5 dB to -35 dB doubled.
*/
namespace
{
    double measuredSeconds(const std::vector<float>& response, double sampleRate)
    {
        std::vector<double> toCome(response.size() + 1, 0.0);
        for (std::size_t i = response.size(); i-- > 0;)
            toCome[i] = toCome[i + 1] + static_cast<double>(response[i]) * static_cast<double>(response[i]);
        if (toCome[0] <= 0.0)
            return 0.0;
        auto firstBelow = [&](double decibels)
        {
            const double level = toCome[0] * std::pow(10.0, decibels / 10.0);
            for (std::size_t i = 0; i < response.size(); ++i)
                if (toCome[i] <= level)
                    return static_cast<double>(i);
            return static_cast<double>(response.size());
        };
        return 2.0 * (firstBelow(-35.0) - firstBelow(-5.0)) / sampleRate;
    }

    double plateSeconds(float size, float damping, double sampleRate)
    {
        coolsynth::synth::PlateReverb plate;
        plate.prepare(sampleRate);
        plate.setSize(size);
        plate.setDamping(damping);
        const int length = static_cast<int>(sampleRate * (2.0 * coolsynth::synth::PlateReverb::decaySeconds(size) + 1.0));
        std::vector<float> left(static_cast<std::size_t>(length), 0.0f), right(static_cast<std::size_t>(length), 0.0f);
        left[0] = right[0] = 1.0f;
        for (int at = 0; at < length; at += 512)
            plate.process(left.data() + at, right.data() + at, std::min(512, length - at));
        return measuredSeconds(left, sampleRate);
    }
}

class ReverbTimeTests final : public juce::UnitTest
{
public:
    ReverbTimeTests() : juce::UnitTest("Reverb time", "Effects") {}

    void runTest() override
    {
        beginTest("the_plates_tail_lasts_what_its_size_says");
        {
            for (const float size : { 0.0f, 0.2f, 0.4f, 0.6f, 0.8f })
                for (const float damping : { 0.0f, 0.5f })
                {
                    const double said = coolsynth::synth::PlateReverb::decaySeconds(size);
                    const double measured = plateSeconds(size, damping, 48000.0);
                    logMessage("size " + juce::String(size, 1) + " damping " + juce::String(damping, 1) + ": says "
                               + juce::String(said, 2) + " s, measured " + juce::String(measured, 2) + " s");
                    // What it says is the tail with Damping at its middle; with none the treble rings a little longer.
                    expectWithinAbsoluteError(measured / said, damping > 0.0f ? 1.0 : 1.06, damping > 0.0f ? 0.04 : 0.08,
                                              "size " + juce::String(size, 1) + ", damping " + juce::String(damping, 1));
                }
        }

        beginTest("the_time_grows_with_size_and_is_the_same_at_any_sample_rate");
        {
            double before = 0.0;
            for (int step = 0; step <= 20; ++step)
            {
                const double said = coolsynth::synth::PlateReverb::decaySeconds(static_cast<float>(step) / 20.0f);
                expect(said > before);
                before = said;
            }
            const double at44 = plateSeconds(0.4f, 0.0f, 44100.0), at96 = plateSeconds(0.4f, 0.0f, 96000.0);
            expectWithinAbsoluteError(at44 / at96, 1.0, 0.1);
        }
    }
};

static ReverbTimeTests reverbTimeTests;
