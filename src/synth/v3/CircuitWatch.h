#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>

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
        /** The metal oscillators' sheet watches the bank itself, which no light stands for (D317). */
        static constexpr int metalBankSheet = lightCount;

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
        static constexpr int captureSeries = 192, captureFrames = 1536; // the cymbal's 600 ms on the grid: some 1380 frames

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

    /**
        A generator's capture of one of its instruments' circuit at a hit, into a watch (D317, D318): begun at
        the hit, then after each of that instrument's samples a frame on the recordings' grid (every 1/24 ms for the
        first 8 ms, then every 0.5 ms), for as long as its recording runs (its liveMs: the bass drum's 300 ms, the
        tom's 600, the rim shot's 150 and so on). The instrument class gives liveNames, liveCount, liveMs and
        tap(float*), and liveEverySample when its recording is on a 20 us grid throughout (the metal oscillators').
    */
    class CircuitCapture final
    {
    public:
        /** At a hit of `instrument` (the generator's own number) shown as `light`: true when a capture begins. */
        template <class Circuit>
        bool begin(CircuitWatch* watch, int light, int instrument) noexcept
        {
            static_assert(Circuit::liveCount <= CircuitWatch::captureSeries, "a watch's frames hold its series");
            if (watch == nullptr || ! watch->beginCapture(light, Circuit::liveNames.data(), Circuit::liveCount))
                return false;
            into = watch;
            which = instrument;
            frame = 0;
            sample = 0;
            return true;
        }
        /** The instrument being captured, or -1. */
        [[nodiscard]] int instrument() const noexcept { return into != nullptr ? which : -1; }
        /** After a sample of the captured instrument. */
        template <class Circuit>
        void after(const Circuit& circuit, double sampleRate) noexcept
        {
            if (into == nullptr)
                return;
            const double ms = 1000.0 * static_cast<double>(sample) / sampleRate;
            // the recordings' grid in time, whatever rate the generator runs at (the metal one's 96 kHz)
            const auto every = [sampleRate](double seconds) { return std::max<long>(1, std::lround(seconds * sampleRate)); };
            long stride = ms < 8.0 ? every(1.0 / 24000.0) : every(0.0005);
            if constexpr (requires { Circuit::liveEverySample; })
                if (Circuit::liveEverySample)
                    stride = every(1.0 / 50000.0); // the oscillators' recording: every 20 us
            if (sample % stride == 0 && frame < CircuitWatch::captureFrames)
            {
                circuit.tap(into->frame(frame));
                into->frameWritten(frame, static_cast<float>(ms));
                ++frame;
            }
            ++sample;
            if (ms >= Circuit::liveMs || frame >= CircuitWatch::captureFrames)
                into = nullptr;
        }
        void stop() noexcept { into = nullptr; }

    private:
        CircuitWatch* into = nullptr;
        int which = -1, frame = 0;
        long sample = 0;
    };
}
