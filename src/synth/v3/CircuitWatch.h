#pragma once

#include <algorithm>
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

        // ---- One drum's circuit captured as it is struck (CoolSynth's D317) ------------------------------------
        // The interface names the instrument it shows; at each of its hits, its generator writes the circuit's named
        // node voltages and part currents (as its ngspice recording names them) frame by frame over the hit: every 4
        // samples for the first 8 ms, then every 0.5 ms, as the recordings' own grid. The interface reads the frames
        // written so far; a hit while it copies them starts no new capture.
        static constexpr int captureSeries = 192, captureFrames = 1024;

        /** The interface: the instrument to capture (a Light), or -1 for none. */
        void watchCircuit(int light) noexcept { watched.store(light, std::memory_order_release); }
        [[nodiscard]] int watchedCircuit() const noexcept { return watched.load(std::memory_order_acquire); }

        /** The audio thread, at a hit: true if this instrument is watched and a capture begins (its names given). */
        bool beginCapture(int light, const char* const* names, int count) noexcept
        {
            if (light != watched.load(std::memory_order_acquire) || held.load(std::memory_order_acquire))
                return false;
            seriesNames = names;
            seriesCount = std::min(count, captureSeries);
            frames.store(0, std::memory_order_release);
            hits.fetch_add(1, std::memory_order_acq_rel);
            return true;
        }
        [[nodiscard]] float* frame(int index) noexcept { return values[static_cast<std::size_t>(index)].data(); }
        void frameWritten(int index, float ms) noexcept
        {
            times[static_cast<std::size_t>(index)] = ms;
            frames.store(index + 1, std::memory_order_release);
        }

        /** The interface: hold the frames still while copying them (a hit meanwhile starts no capture). */
        void holdCapture(bool on) noexcept { held.store(on, std::memory_order_release); }
        [[nodiscard]] unsigned int captureCount() const noexcept { return hits.load(std::memory_order_acquire); }
        [[nodiscard]] int framesCaptured() const noexcept { return frames.load(std::memory_order_acquire); }
        [[nodiscard]] int capturedSeries() const noexcept { return seriesCount; }
        [[nodiscard]] const char* capturedName(int series) const noexcept { return series >= 0 && series < seriesCount && seriesNames != nullptr ? seriesNames[series] : ""; }
        [[nodiscard]] float capturedTime(int index) const noexcept { return times[static_cast<std::size_t>(index)]; }
        [[nodiscard]] float captured(int index, int series) const noexcept { return values[static_cast<std::size_t>(index)][static_cast<std::size_t>(series)]; }

    private:
        std::array<std::atomic<float>, lightCount> lights {};
        std::atomic<unsigned int> reports { 0 };
        std::atomic<int> watched { -1 }, frames { 0 };
        std::atomic<unsigned int> hits { 0 };
        std::atomic<bool> held { false };
        const char* const* seriesNames = nullptr;
        int seriesCount = 0;
        std::array<float, captureFrames> times {};
        std::array<std::array<float, captureSeries>, captureFrames> values {};
    };
}
