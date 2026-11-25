#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "sync/SyncCore.h"

/*
    The lock-free handoff between the beat sync's network thread and the audio and UI threads (CoolSynth's D361,
    docs/research/beat-sync-protocol.md, "Handing state to the audio thread"). Every channel is a fixed array of
    atomic 64-bit words under a sequence counter (a seqlock) with one writer at a time: the writer makes the counter
    odd, writes the words, makes it even; a reader reads the counter, the words and the counter again, and keeps its
    previous copy if the two differ or are odd. No locks, no allocation, no system calls, no data races.
*/
namespace coolsynth::sync
{
    template <std::size_t N>
    class SeqLock
    {
    public:
        using Words = std::array<std::int64_t, N>;
        static_assert(std::atomic<std::int64_t>::is_always_lock_free && std::atomic<std::uint64_t>::is_always_lock_free);

        /** One writer at a time. */
        void write(const Words& values) noexcept
        {
            const std::uint64_t s = sequence.load(std::memory_order_relaxed);
            sequence.store(s + 1, std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_release);
            for (std::size_t i = 0; i < N; ++i)
                words[i].store(values[i], std::memory_order_relaxed);
            sequence.store(s + 2, std::memory_order_release);
        }

        /** One try: false, with `out` unspecified, if a write was under way. */
        [[nodiscard]] bool tryRead(Words& out) const noexcept
        {
            const std::uint64_t before = sequence.load(std::memory_order_acquire);
            if ((before & 1u) != 0)
                return false;
            for (std::size_t i = 0; i < N; ++i)
                out[i] = words[i].load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            return sequence.load(std::memory_order_relaxed) == before;
        }

        /** A few tries; `out` is only changed by a whole, consistent copy. */
        bool read(Words& out, int attempts = 4) const noexcept
        {
            Words copy {};
            for (int i = 0; i < attempts; ++i)
                if (tryRead(copy))
                {
                    out = copy;
                    return true;
                }
            return false;
        }

        [[nodiscard]] std::uint64_t writes() const noexcept { return sequence.load(std::memory_order_acquire) / 2; }

    private:
        alignas(64) std::atomic<std::uint64_t> sequence { 0 };
        std::array<std::atomic<std::int64_t>, N> words {};
    };

    /** The app's own grid in host time, written by the audio thread: its beat `appBeat` at host time `hostTime`. The
        network thread founds a session from it, so switching on neither moves the beat nor its phase. */
    struct LocalGrid
    {
        Micros tempo = 500000;
        MicroBeats appBeat = 0;
        Micros hostTime = 0;
    };

    inline constexpr std::size_t kSnapshotWords = 15;
    inline constexpr std::size_t kGridWords = 5;
    inline constexpr std::size_t kPlayWords = 5;
    inline constexpr std::size_t kLocalWords = 3;

    [[nodiscard]] inline std::int64_t packId(const PeerId& id) noexcept
    {
        std::uint64_t u = 0;
        for (const auto b : id)
            u = (u << 8) | b;
        return static_cast<std::int64_t>(u);
    }

    [[nodiscard]] inline PeerId unpackId(std::int64_t v) noexcept
    {
        PeerId id {};
        auto u = static_cast<std::uint64_t>(v);
        for (int i = 7; i >= 0; --i, u >>= 8)
            id[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(u & 0xFF);
        return id;
    }

    [[nodiscard]] inline SeqLock<kSnapshotWords>::Words encode(const Snapshot& s) noexcept
    {
        return { s.number, packId(s.session), s.peers, (s.running ? 1 : 0) | (s.startStopSharing ? 2 : 0),
                 s.timeline.tempo, s.timeline.beatOrigin, s.timeline.timeOrigin, s.clockOffset,
                 s.playState.playing ? 1 : 0, s.playState.beat, s.playState.time,
                 s.playChange, s.gridApplied, s.playApplied, s.epoch };
    }

    [[nodiscard]] inline Snapshot decodeSnapshot(const SeqLock<kSnapshotWords>::Words& w) noexcept
    {
        Snapshot s;
        s.number = w[0];
        s.session = unpackId(w[1]);
        s.peers = static_cast<int>(w[2]);
        s.running = (w[3] & 1) != 0;
        s.startStopSharing = (w[3] & 2) != 0;
        s.timeline = { w[4] > 0 ? w[4] : 500000, w[5], w[6] };
        s.clockOffset = w[7];
        s.playState = { w[8] != 0, w[9], w[10] };
        s.playChange = w[11];
        s.gridApplied = w[12];
        s.playApplied = w[13];
        s.epoch = w[14];
        return s;
    }

    [[nodiscard]] inline SeqLock<kGridWords>::Words encode(const GridProposal& g) noexcept
    {
        return { g.number, g.tempo, g.beat, g.hostTime, g.epoch };
    }

    [[nodiscard]] inline GridProposal decodeGrid(const SeqLock<kGridWords>::Words& w) noexcept
    {
        return { w[0], w[1], w[2], w[3], w[4] };
    }

    [[nodiscard]] inline SeqLock<kPlayWords>::Words encode(const PlayProposal& p) noexcept
    {
        return { p.number, p.playing ? 1 : 0, p.hostTime, p.beat, p.epoch };
    }

    [[nodiscard]] inline PlayProposal decodePlay(const SeqLock<kPlayWords>::Words& w) noexcept
    {
        return { w[0], w[1] != 0, w[2], w[3], w[4] };
    }

    [[nodiscard]] inline SeqLock<kLocalWords>::Words encode(const LocalGrid& l) noexcept
    {
        return { l.tempo, l.appBeat, l.hostTime };
    }

    [[nodiscard]] inline LocalGrid decodeLocal(const SeqLock<kLocalWords>::Words& w) noexcept
    {
        return { w[0] > 0 ? w[0] : 500000, w[1], w[2] };
    }

    /** Everything the threads share. The network thread writes `snapshot`; the audio thread writes `grid`, `play`
        and `local`; the message thread writes `startStopSharing`. */
    struct Handoff
    {
        SeqLock<kSnapshotWords> snapshot;
        SeqLock<kGridWords> grid;
        SeqLock<kPlayWords> play;
        SeqLock<kLocalWords> local;
        alignas(64) std::atomic<bool> startStopSharing { true };

        [[nodiscard]] Snapshot readSnapshot() const noexcept
        {
            SeqLock<kSnapshotWords>::Words w {};
            while (! snapshot.read(w))
            {
            }
            return decodeSnapshot(w);
        }
    };
}
