#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string_view>

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

        // ---- The instruments heard (the 808's WAV export, CoolSynth's D331) ---------------------------------------
        // A drum's own file: every instrument plays as ever (it runs, rings and rests as in the mix, and the voice lasts
        // as long), but only those heard go into their generators' outputs. All heard by default.
        static constexpr std::uint32_t allHeard = (1u << lightCount) - 1u;
        void hearOnly(std::uint32_t lightMask) noexcept { heard.store(lightMask, std::memory_order_relaxed); }
        [[nodiscard]] std::uint32_t heardMask() const noexcept { return heard.load(std::memory_order_relaxed); }

        // ---- The 808 Metal's Lighter accuracy (CoolSynth's D339) ---------------------------------------------------
        // On: its five swing VCAs run only while their drum sounds, and its circuits' Newton ends at 10 uV, not 100 nV.
        void setLighter(bool on) noexcept { lighterOn.store(on, std::memory_order_relaxed); }
        [[nodiscard]] bool lighter() const noexcept { return lighterOn.load(std::memory_order_relaxed); }

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

        // ---- Every drum's board nodes as it plays (CoolSynth's D320) ----------------------------------------------
        // While a panel shows the whole board (the lid lifted), each instrument's generator reports, after each hit
        // (every 1/24 ms for 8 ms, then every quarter of a millisecond), how far a few of its nodes are from their rest:
        // those the whole board's sheet and the lifted lid stand for (the trigger, the envelopes, the core, the
        // output). The rest is the interface's, from the recordings (a circuit struck from rest has computed no
        // sample yet). The largest swing since the interface last took it is kept, with its sign.
        static constexpr int boardSlots = 9;
        /** Each instrument's board nodes, as its ngspice recording names them (empty past its last). */
        static constexpr std::array<std::array<std::string_view, boardSlots>, lightCount> boardNodes { {
            { "trig", "npulse", "nout", "vout" },                                    // bass drum
            { "vt", "nev", "nc52", "np", "no1", "vout" },                            // low tom
            { "vt", "nev", "nc52", "np", "no1", "vout" },                            // mid tom
            { "vt", "nev", "nc52", "np", "no1", "vout" },                            // high tom
            { "vt", "np21", "no21", "np20", "no20a", "ns", "nenv", "nf", "vout" },   // rim shot
            { "vt", "ne47", "nc48", "np1", "no1", "np2", "no2", "vout" },            // snare
            { "vt", "n144", "np8", "n139", "nc70", "vout" },                         // clap
            { "ncb5", "ncbe", "nc15", "nc14", "n6ap" },                              // cowbell
            { "ncy6", "nj", "nc16", "nl", "nc17", "nv", "nc18", "vcy" },             // cymbal
            { "noh7", "ne1", "nc27", "voh" },                                        // open hat
            { "nch8", "nce", "nc30", "vch" } } };                                    // closed hat

        /** The interface: whether the board is shown (the generators report nothing while it is not). */
        void watchBoard(bool on) noexcept { boardOn.store(on, std::memory_order_release); }
        [[nodiscard]] bool boardWatched() const noexcept { return boardOn.load(std::memory_order_acquire); }

        /** The interface, once: a node's rest (volts), from its recording. */
        void setBoardRest(int light, int slot, float volts) noexcept
        {
            if (light >= 0 && light < lightCount && slot >= 0 && slot < boardSlots)
                boardRest[static_cast<std::size_t>(light)][static_cast<std::size_t>(slot)].store(volts, std::memory_order_relaxed);
        }
        [[nodiscard]] float boardRestOf(int light, int slot) const noexcept
        {
            return boardRest[static_cast<std::size_t>(light)][static_cast<std::size_t>(slot)].load(std::memory_order_relaxed);
        }

        /** The audio thread: an instrument struck. */
        void boardHit(int light) noexcept
        {
            if (light >= 0 && light < lightCount)
                boardHits[static_cast<std::size_t>(light)].fetch_add(1, std::memory_order_acq_rel);
        }
        /** The audio thread: a node's swing from its rest now; the largest since the last take is kept. */
        void boardSwing(int light, int slot, float swing) noexcept
        {
            auto& s = board[static_cast<std::size_t>(light)][static_cast<std::size_t>(slot)];
            float seen = s.load(std::memory_order_relaxed);
            while (std::abs(swing) > std::abs(seen) && ! s.compare_exchange_weak(seen, swing, std::memory_order_relaxed))
            {
            }
        }

        /** The interface: how many times the instrument was struck while the board was shown. */
        [[nodiscard]] unsigned int boardHitCount(int light) const noexcept
        {
            return light < 0 || light >= lightCount ? 0u : boardHits[static_cast<std::size_t>(light)].load(std::memory_order_acquire);
        }
        /** The interface: a node's largest swing (signed, volts) since the last take, and a fresh start. */
        [[nodiscard]] float takeBoardSwing(int light, int slot) noexcept
        {
            if (light < 0 || light >= lightCount || slot < 0 || slot >= boardSlots)
                return 0.0f;
            return board[static_cast<std::size_t>(light)][static_cast<std::size_t>(slot)].exchange(0.0f, std::memory_order_relaxed);
        }

    private:
        std::array<std::atomic<float>, lightCount> lights {};
        std::atomic<bool> boardOn { false };
        std::array<std::atomic<unsigned int>, lightCount> boardHits {};
        std::array<std::array<std::atomic<float>, boardSlots>, lightCount> board {}, boardRest {};
        std::atomic<unsigned int> reports { 0 };
        std::atomic<std::uint32_t> heard { allHeard };
        std::atomic<bool> lighterOn { false };
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

    /**
        A generator's feed of its instruments' board nodes into a watch (CoolSynth's D320): after each of a struck
        instrument's samples, every 1/24 ms for 8 ms (the recordings' grid, short pulses and all) and then every quarter
        of a millisecond, its nodes' swing from their rest, for 4 s. The instrument class gives liveNames, liveCount and
        tap(float*); the nodes are found in its names once, when the code is compiled.
    */
    class BoardFeed final
    {
    public:
        /** At a hit of the instrument shown as `light`. */
        template <class Circuit>
        void hit(CircuitWatch* watch, int light, const Circuit& circuit) noexcept
        {
            auto& s = state[static_cast<std::size_t>(light)];
            s.on = watch != nullptr && watch->boardWatched();
            if (! s.on)
                return;
            s.sample = 0;
            watch->boardHit(light);
        }
        /** After a sample of the instrument shown as `light`. */
        template <class Circuit>
        void after(CircuitWatch* watch, int light, const Circuit& circuit, double sampleRate) noexcept
        {
            auto& s = state[static_cast<std::size_t>(light)];
            if (! s.on || watch == nullptr)
                return;
            const bool early = static_cast<double>(s.sample) < 0.008 * sampleRate;
            const long stride = std::max<long>(1, std::lround((early ? 1.0 / 24000.0 : 0.00025) * sampleRate));
            if (s.sample++ % stride != 0)
                return;
            if (static_cast<double>(s.sample) > 4.0 * sampleRate || ! watch->boardWatched())
            {
                s.on = false;
                return;
            }
            std::array<float, CircuitWatch::boardSlots> now {};
            read<Circuit>(light, circuit, now);
            for (int k = 0; k < CircuitWatch::boardSlots; ++k)
                if (index<Circuit>[static_cast<std::size_t>(light)][static_cast<std::size_t>(k)] >= 0)
                    watch->boardSwing(light, k, now[static_cast<std::size_t>(k)] - watch->boardRestOf(light, k));
        }
        void stop() noexcept
        {
            for (auto& s : state)
                s.on = false;
        }

    private:
        struct State { bool on = false; long sample = 0; };
        std::array<State, CircuitWatch::lightCount> state {};

        /** Where each instrument's board nodes sit in a circuit's names (-1 where it has none). */
        template <class Circuit>
        static constexpr auto index = []
        {
            std::array<std::array<int, CircuitWatch::boardSlots>, CircuitWatch::lightCount> t {};
            for (std::size_t l = 0; l < t.size(); ++l)
                for (std::size_t k = 0; k < t[l].size(); ++k)
                {
                    t[l][k] = -1;
                    const auto name = CircuitWatch::boardNodes[l][k];
                    if (name.empty())
                        continue;
                    for (int i = 0; i < Circuit::liveCount; ++i)
                        if (std::string_view(Circuit::liveNames[static_cast<std::size_t>(i)]) == name)
                            t[l][k] = i;
                }
            return t;
        }();

        template <class Circuit>
        static void read(int light, const Circuit& circuit, std::array<float, CircuitWatch::boardSlots>& out) noexcept
        {
            std::array<float, CircuitWatch::captureSeries> all {};
            circuit.tap(all.data());
            for (std::size_t k = 0; k < out.size(); ++k)
            {
                const int i = index<Circuit>[static_cast<std::size_t>(light)][k];
                out[k] = i >= 0 ? all[static_cast<std::size_t>(i)] : 0.0f;
            }
        }
    };
}
