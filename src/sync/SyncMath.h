#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

/*
    The beat sync's maths (CoolSynth's D361, docs/research/beat-sync-protocol.md, "The maths every peer must agree
    on"): the units, the session timeline, tempo conversion and clamping, phase, snapping and the quantized start.
    Every peer rounds the same way, llround on a double in the order written here, so peers compute identical grids.
    Header only, no state, no JUCE.
*/
namespace coolsynth::sync
{
    /** A time in microseconds, of the host clock or the session clock. */
    using Micros = std::int64_t;
    /** A beat count in micro-beats (a millionth of a beat). */
    using MicroBeats = std::int64_t;

    inline constexpr MicroBeats kMicroBeatsPerBeat = 1000000;
    inline constexpr double kMinBpm = 20.0;
    inline constexpr double kMaxBpm = 999.0;
    inline constexpr double kMicrosPerMinute = 60000000.0;

    /** Beats in a duration d at tempo m (microseconds per beat). */
    [[nodiscard]] inline MicroBeats beatsInDuration(Micros d, Micros m) noexcept
    {
        return std::llround(static_cast<double>(d) / static_cast<double>(m) * 1.0e6);
    }

    /** The duration of b micro-beats at tempo m. */
    [[nodiscard]] inline Micros durationOfBeats(MicroBeats b, Micros m) noexcept
    {
        return std::llround(static_cast<double>(b) / 1.0e6 * static_cast<double>(m));
    }

    /** A beat count from a value in beats. */
    [[nodiscard]] inline MicroBeats toMicroBeats(double beats) noexcept { return std::llround(beats * 1.0e6); }

    [[nodiscard]] inline double toBeats(MicroBeats b) noexcept { return static_cast<double>(b) / 1.0e6; }

    [[nodiscard]] inline double clampBpm(double bpm) noexcept
    {
        if (! (bpm == bpm)) // NaN
            return 120.0;
        return std::clamp(bpm, kMinBpm, kMaxBpm);
    }

    /** Microseconds per beat from a BPM, the BPM clamped to 20 to 999 first. */
    [[nodiscard]] inline Micros tempoFromBpm(double bpm) noexcept
    {
        return std::llround(kMicrosPerMinute / clampBpm(bpm));
    }

    [[nodiscard]] inline double bpmFromTempo(Micros m) noexcept
    {
        return m > 0 ? kMicrosPerMinute / static_cast<double>(m) : 120.0;
    }

    /** A tempo read from the wire, clamped through its BPM; zero or below is invalid (returns 0). */
    [[nodiscard]] inline Micros tempoFromWire(std::int64_t m) noexcept
    {
        if (m <= 0)
            return 0;
        return tempoFromBpm(kMicrosPerMinute / static_cast<double>(m));
    }

    /** The session timeline: a tempo, an anchor beat and the anchor's session time. */
    struct Timeline
    {
        Micros tempo = 500000;     // microseconds per beat
        MicroBeats beatOrigin = 0; // the anchor beat
        Micros timeOrigin = 0;     // the anchor's session time

        [[nodiscard]] MicroBeats beatAt(Micros sessionTime) const noexcept
        {
            return beatOrigin + beatsInDuration(sessionTime - timeOrigin, tempo);
        }

        [[nodiscard]] Micros timeAt(MicroBeats beat) const noexcept
        {
            return timeOrigin + durationOfBeats(beat - beatOrigin, tempo);
        }

        /** The beat at a session time as a double, for the audio thread: the same line without the rounding. */
        [[nodiscard]] double beatAtPrecise(Micros sessionTime) const noexcept
        {
            return toBeats(beatOrigin) + static_cast<double>(sessionTime - timeOrigin) / static_cast<double>(tempo);
        }

        friend bool operator==(const Timeline&, const Timeline&) = default;
    };

    /** The play state: playing or not, the session beat at which that changed and the session time of the change. */
    struct PlayState
    {
        bool playing = false;
        MicroBeats beat = 0;
        Micros time = 0;

        friend bool operator==(const PlayState&, const PlayState&) = default;
    };

    /** Rounds a division up for a positive divisor. */
    [[nodiscard]] inline std::int64_t ceilDiv(std::int64_t x, std::int64_t q) noexcept
    {
        const std::int64_t d = x / q; // truncates toward zero, which is the ceiling for negative x
        return (x % q != 0 && x > 0) ? d + 1 : d;
    }

    /** The phase of b in a quantum q, both in micro-beats: in [0, q), also for negative b; zero for a quantum of zero. */
    [[nodiscard]] inline MicroBeats phase(MicroBeats b, MicroBeats q) noexcept
    {
        if (q <= 0)
            return 0;
        const MicroBeats r = b % q;
        return r < 0 ? r + q : r;
    }

    /** An offset moved to the multiple of q nearest to it, ties going down: q * ceil((o - q / 2) / q). A quantum of
        zero never snaps. */
    [[nodiscard]] inline MicroBeats snapToQuantum(MicroBeats o, MicroBeats q) noexcept
    {
        if (q <= 0)
            return o;
        const MicroBeats half = std::llround(0.5 * static_cast<double>(q));
        return q * ceilDiv(o - half, q);
    }

    /** The quantized start: the smallest whole multiple of q at or after the session beat b of the press. */
    [[nodiscard]] inline MicroBeats quantizedStart(MicroBeats b, MicroBeats q) noexcept
    {
        if (q <= 0)
            return b;
        return q * ceilDiv(b, q);
    }

    /** A new timeline of tempo m passing through beat p at session time g, with an anchor beat larger than the
        current one's: anchor b* = max(p, current anchor + one beat), anchor time g plus the time b* - p takes at m.
        The tempo-change rule, with p the session beat at g on the current timeline. */
    [[nodiscard]] inline Timeline timelineThrough(const Timeline& current, Micros m, MicroBeats p, Micros g) noexcept
    {
        const MicroBeats anchor = std::max(p, current.beatOrigin + kMicroBeatsPerBeat);
        return { m, anchor, g + durationOfBeats(anchor - p, m) };
    }

    /** The tempo-change rule: the new tempo from session time g, the grid continuous at g. */
    [[nodiscard]] inline Timeline changeTempo(const Timeline& current, Micros m, Micros g) noexcept
    {
        return timelineThrough(current, m, current.beatAt(g), g);
    }
}
