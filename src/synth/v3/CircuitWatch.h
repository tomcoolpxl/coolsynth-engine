#pragma once

#include <array>
#include <atomic>

/*
    What the 808's circuits are doing, for a panel to show as they play (CoolSynth's D313): each instrument's level,
    as its generator's follower has it (the circuit's output volts, before the slot's level and pan), raised from the
    audio thread and taken by the interface. Owned by the app and handed to the engine (EngineV3::setCircuitWatch); a
    null watch, the default, costs nothing and changes nothing. Every write is lock-free, as generators may run on
    worker threads.
*/
namespace coolsynth::synth::v3
{
    class CircuitWatch final
    {
    public:
        /** The 808's instruments, as their three generators hold them. */
        enum Light : int
        {
            bassDrum = 0, lowTom, midTom, highTom, rimShot, // the 808 Drums (a tom's light is its conga's too, and claves the rim shot's)
            snare, clap,                                    // the 808 Snare & Clap
            cowbell, cymbal, openHat, closedHat,            // the 808 Metal
            lightCount
        };

        /** From the audio thread: the instrument's level now; the highest since the last take is kept. */
        void raise(int light, float level) noexcept
        {
            if (light < 0 || light >= lightCount)
                return;
            auto& slot = lights[static_cast<std::size_t>(light)];
            float seen = slot.load(std::memory_order_relaxed);
            while (level > seen && ! slot.compare_exchange_weak(seen, level, std::memory_order_relaxed))
            {
            }
        }

        /** From the interface: the highest level since the last take, and a fresh start. */
        [[nodiscard]] float take(int light) noexcept
        {
            return light < 0 || light >= lightCount ? 0.0f : lights[static_cast<std::size_t>(light)].exchange(0.0f, std::memory_order_relaxed);
        }

        /** How many times a generator has reported: the interface can tell a running engine from a stopped one. */
        void reported() noexcept { reports.fetch_add(1, std::memory_order_relaxed); }
        [[nodiscard]] unsigned int reportCount() const noexcept { return reports.load(std::memory_order_relaxed); }

    private:
        std::array<std::atomic<float>, lightCount> lights {};
        std::atomic<unsigned int> reports { 0 };
    };
}
