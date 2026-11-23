#pragma once

#include <array>
#include <cstdint>

#include "SynthParameters.h"

/*
    The step sequencer (CoolSynth 808, D266): lanes of sixteen sixteenth-note steps, each lane a drum's note, with an
    accent lane for the whole kit; A and B variations played as A, B or AB; a length per lane (polymeter); a chance
    and a roll of one to three hits per step; mute and solo; one swing for the whole kit (Roger Linn's: the even
    sixteenths delayed, 0.5 straight); sixteen patterns a bank and a fill pattern; chains of patterns.

    Time is the beat position the clock gives (quarter notes from the start, as a host's ppq): the app's own clock,
    MIDI clock or a DAW. Every step's place is worked out from it, so the notes do not depend on the block size and
    follow a host that loops or jumps. A step's chance is decided by a hash of the seed, the lane and the step's
    place, not a running random source: the same step decides the same way on every play (D224).

    Bars are four beats, sixteen steps. Bar 0 starts at beat 0. The chain plays from the bar it was set at: each
    pattern one bar (A or B) or two (AB), in a loop. A lane counts its steps from where its pattern began to play:
    while one pattern plays on, a short lane runs on across the bars (polymeter, as Elektron's per-track lengths);
    a chain of different patterns starts each one afresh. A new chain set while playing waits for the end of the pattern
    that plays; a fill asked for plays the bank's fill pattern in the next bar, once, while the chain counts on.

    A plain object for the audio thread: it never allocates or locks. The app hands it edits.
*/
namespace coolsynth::synth
{
    struct SequencerStep
    {
        bool on = false;
        float chance = 1.0f; // 0..1: how often it plays
        std::uint8_t hits = 1; // 1, 2 or 3: a roll within the step
    };

    struct SequencerLane
    {
        static constexpr int steps = 16;
        std::array<SequencerStep, steps> a {}; // variation A
        std::array<SequencerStep, steps> b {}; // variation B
        int length = steps;                     // 1..16: the lane loops on its own (polymeter)
        int note = -1;                          // the drum's key; -1 plays nothing
        bool mute = false;
        bool solo = false;
    };

    struct SequencerPattern
    {
        static constexpr int maxLanes = 16;
        std::array<SequencerLane, maxLanes> lanes {};
        SequencerLane accent {}; // its note is unused: an accented step strikes the drums on it harder
    };

    struct SequencerBank
    {
        static constexpr int patterns = 16;
        std::array<SequencerPattern, patterns> pattern {};
        SequencerPattern fill {};
    };

    enum class SequencerVariation : int { a = 0, b = 1, ab = 2 };

    struct SequencerSettings
    {
        float swing = 0.5f;            // 0.5 straight .. 0.75: where the even sixteenth falls within its eighth
        SequencerVariation variation = SequencerVariation::a;
        float velocity = 1.0f / 127.0f; // a step's velocity; the 808 kit strikes it at its lowest trigger (D258)
        float accentVelocity = 1.0f;    // an accented step's: the Accent knob's level
        std::uint32_t seed = 0x808u;
    };

    /** Where a beat position falls: for recording a hit onto the nearest step. */
    struct SequencerPlace
    {
        int pattern = -1;  // index in the bank; -1 is the fill
        bool variationB = false;
        std::int64_t sixteenth = 0; // steps since the pattern began to play; a lane's step is this modulo its length
    };

    class StepSequencer final
    {
    public:
        static constexpr int maxChain = 64;
        static constexpr int maxPending = 128;

        void setBank(const SequencerBank& bank) noexcept { bankData = bank; }
        [[nodiscard]] const SequencerBank& bank() const noexcept { return bankData; }
        void setSettings(const SequencerSettings& settings) noexcept;

        /** The patterns to play in a row (indices into the bank). While stopped it takes effect at once, from bar 0;
            while playing at the end of the pattern that plays. */
        void setChain(const int* patterns, int count) noexcept;
        /** As setChain, but while playing the chain begins at the given bar, its first pattern from its start: at once
            when the bar is already being played or looked ahead to. A beat auditioned in time comes in on a bar line. */
        void setChainFrom(const int* patterns, int count, std::int64_t bar) noexcept;
        /** Plays the fill pattern once, in the bar after the one that plays (while it plays, or is waiting to, a
            second ask changes nothing; asked while stopped, nothing). */
        void requestFill() noexcept { fillAsked = true; }

        /** Notes for one block: the transport's beat position at the block's start, its tempo, whether it plays.
            Writes up to maxEvents events, sorted by sample offset, and returns how many. A stop sends every pending
            note-off at once. */
        int process(double beatAtBlockStart, double bpm, bool playing, int blockSamples, double sampleRate,
                    EngineMidiEvent* out, int maxEvents) noexcept;

        /** Stops and forgets where it was: the chain starts again from bar 0, and pending note-offs are dropped. */
        void reset() noexcept;

        /** The step nearest a beat position, swing counted: where a recorded hit goes. */
        [[nodiscard]] SequencerPlace placeAt(double beat) const noexcept;

        /** The beat position of a sixteenth with the swing applied. */
        [[nodiscard]] double swungBeat(std::int64_t sixteenth) const noexcept;

        /** Whether a step plays at a sixteenth, chance included: the same answer every time it is asked. */
        [[nodiscard]] bool passesChance(int lane, std::int64_t sixteenth, float chance) const noexcept;

    private:
        struct Segment
        {
            int pattern = 0;          // -1: the fill
            bool variationB = false;
            std::int64_t startBar = 0; // where this turn of the pattern began
            std::int64_t runBar = 0;   // where the lanes count from: the chain's start when it is one pattern
        };
        struct PendingOff
        {
            double beat = 0.0;
            int note = 0;
        };

        [[nodiscard]] int barsPerPattern() const noexcept { return settings.variation == SequencerVariation::ab ? 2 : 1; }
        [[nodiscard]] Segment segmentAt(std::int64_t bar) const noexcept;
        void advanceTo(std::int64_t bar) noexcept;
        void switchChainAt(std::int64_t bar) noexcept;
        /** A time as a hit's place: the sixteenth plus how far into it, whatever the swing. */
        [[nodiscard]] double hitPlaceAt(double beat) const noexcept;
        [[nodiscard]] const SequencerPattern& patternFor(const Segment& s) const noexcept;

        SequencerBank bankData {};
        SequencerSettings settings {};

        std::array<int, maxChain> chain { 0 };
        int chainLength = 1;
        std::int64_t chainStartBar = 0;
        std::array<int, maxChain> nextChain { 0 };
        int nextChainLength = 0; // > 0: waiting for the end of the pattern that plays
        bool nextChainAtBar = false; // waiting for nextChainBar instead
        std::int64_t nextChainBar = 0;
        std::array<int, maxChain> previousChain { 0 }; // the chain before the latest switch, for its last bars
        int previousChainLength = 0;
        std::int64_t previousChainStartBar = 0;

        bool fillAsked = false;
        std::int64_t fillBar = -1;

        bool wasPlaying = false;
        double lastBeatEnd = 0.0;
        std::int64_t lastBar = 0;
        // Every hit up to this place has had its turn (played, or not by chance or a mute): a hit is never played
        // twice nor skipped when the swing or the tempo moves it across a block's edge. Not valid after a start or
        // a jump, where what came before is not this run's.
        double doneUpTo = 0.0;
        bool doneValid = false;

        std::array<PendingOff, maxPending> pending {};
        int pendingCount = 0;
    };
}
