#include "StepSequencer.h"

#include <algorithm>
#include <cmath>

namespace coolsynth::synth
{
    namespace
    {
        constexpr int stepsPerBar = 16;
        constexpr double beatsPerStep = 0.25;
        constexpr int accentLaneId = SequencerPattern::maxLanes; // the accent lane's own dice
        // A host's position and ours may differ by rounding; past this the transport has jumped.
        constexpr double jumpTolerance = 1.0e-3;
        // Far above a double's rounding of a position in samples, far below a sample.
        constexpr double sampleTolerance = 1.0e-4;

        [[nodiscard]] std::int64_t floorDiv(std::int64_t a, std::int64_t b) noexcept
        {
            const auto q = a / b;
            return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
        }

        [[nodiscard]] std::int64_t floorMod(std::int64_t a, std::int64_t b) noexcept { return a - floorDiv(a, b) * b; }

        [[nodiscard]] std::int64_t barOf(double beat) noexcept
        {
            return floorDiv(static_cast<std::int64_t>(std::floor(beat / beatsPerStep)), stepsPerBar);
        }

        [[nodiscard]] std::uint64_t mix(std::uint64_t x) noexcept // splitmix64's finaliser
        {
            x += 0x9E3779B97F4A7C15ull;
            x = (x ^ (x >> 30)) * 0xBF58476D1CE5E9B9ull;
            x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
            return x ^ (x >> 31);
        }

        [[nodiscard]] int clampedLength(int length) noexcept { return std::clamp(length, 1, SequencerLane::steps); }
    }

    void StepSequencer::setSettings(const SequencerSettings& s) noexcept
    {
        settings = s;
        settings.swing = std::clamp(s.swing, 0.5f, 0.75f);
        settings.velocity = std::clamp(s.velocity, 0.0f, 1.0f);
        settings.accentVelocity = std::clamp(s.accentVelocity, 0.0f, 1.0f);
    }

    void StepSequencer::setChain(const int* patterns, int count) noexcept
    {
        if (patterns == nullptr || count <= 0)
            return;
        count = std::min(count, maxChain);
        auto& target = wasPlaying ? nextChain : chain;
        for (int i = 0; i < count; ++i)
            target[static_cast<std::size_t>(i)] = std::clamp(patterns[i], 0, SequencerBank::patterns - 1);
        if (wasPlaying)
            nextChainLength = count;
        else
        {
            chainLength = count;
            chainStartBar = 0;
            nextChainLength = 0;
            previousChainLength = 0;
        }
    }

    void StepSequencer::reset() noexcept
    {
        if (nextChainLength > 0)
        {
            chain = nextChain;
            chainLength = nextChainLength;
            nextChainLength = 0;
        }
        chainStartBar = 0;
        previousChainLength = 0;
        fillAsked = false;
        fillBar = -1;
        wasPlaying = false;
        pendingCount = 0;
    }

    double StepSequencer::swungBeat(std::int64_t sixteenth) const noexcept
    {
        const double beat = static_cast<double>(sixteenth) * beatsPerStep;
        // Linn's swing: the even sixteenth of each eighth (the odd index, counting from 0) moves later.
        return floorMod(sixteenth, 2) == 1 ? beat + (2.0 * settings.swing - 1.0) * beatsPerStep : beat;
    }

    bool StepSequencer::passesChance(int lane, std::int64_t sixteenth, float chance) const noexcept
    {
        if (chance >= 1.0f)
            return true;
        if (chance <= 0.0f)
            return false;
        const auto h = mix(mix(static_cast<std::uint64_t>(settings.seed) ^ (static_cast<std::uint64_t>(lane) << 40))
                           ^ static_cast<std::uint64_t>(sixteenth));
        const double u = static_cast<double>(h >> 11) * (1.0 / 9007199254740992.0); // 0..1
        return u < static_cast<double>(chance);
    }

    StepSequencer::Segment StepSequencer::segmentAt(std::int64_t bar) const noexcept
    {
        if (bar == fillBar)
            return { -1, false, bar, bar };

        // A bar before the latest switch belongs to the chain that played then.
        const bool earlier = bar < chainStartBar && previousChainLength > 0;
        const auto& list = earlier ? previousChain : chain;
        const int length = earlier ? previousChainLength : chainLength;
        const std::int64_t start = earlier ? previousChainStartBar : chainStartBar;

        const int per = barsPerPattern();
        const auto cycle = static_cast<std::int64_t>(length) * per;
        const auto m = floorMod(bar - start, cycle);
        const auto within = floorMod(m, per);
        const bool b = settings.variation == SequencerVariation::b
                       || (settings.variation == SequencerVariation::ab && within == 1);
        bool onePattern = true;
        for (int i = 1; i < length; ++i)
            onePattern = onePattern && list[static_cast<std::size_t>(i)] == list[0];
        return { list[static_cast<std::size_t>(m / per)], b, bar - within, onePattern ? start : bar - within };
    }

    const SequencerPattern& StepSequencer::patternFor(const Segment& s) const noexcept
    {
        return s.pattern < 0 ? bankData.fill : bankData.pattern[static_cast<std::size_t>(s.pattern)];
    }

    void StepSequencer::advanceTo(std::int64_t bar) noexcept
    {
        while (lastBar < bar)
        {
            ++lastBar;
            // A chain set while playing takes over at the end of the pattern that plays.
            if (nextChainLength > 0 && floorMod(lastBar - chainStartBar, barsPerPattern()) == 0)
            {
                previousChain = chain;
                previousChainLength = chainLength;
                previousChainStartBar = chainStartBar;
                chain = nextChain;
                chainLength = nextChainLength;
                chainStartBar = lastBar;
                nextChainLength = 0;
            }
        }
    }

    SequencerPlace StepSequencer::placeAt(double beat) const noexcept
    {
        const auto k0 = static_cast<std::int64_t>(std::floor(beat / beatsPerStep));
        auto nearest = k0;
        double best = 1.0e9;
        for (auto k = k0 - 1; k <= k0 + 1; ++k)
            if (const double d = std::abs(swungBeat(k) - beat); d < best)
            {
                best = d;
                nearest = k;
            }
        const auto segment = segmentAt(floorDiv(nearest, stepsPerBar));
        return { segment.pattern, segment.variationB, nearest - segment.runBar * stepsPerBar };
    }

    void StepSequencer::addPending(double beat, int note) noexcept
    {
        if (pendingCount < maxPending)
            pending[static_cast<std::size_t>(pendingCount++)] = { beat, note };
    }

    int StepSequencer::process(double beat0, double bpm, bool playing, int blockSamples, double sampleRate,
                               EngineMidiEvent* out, int maxEvents) noexcept
    {
        if (out == nullptr || maxEvents <= 0 || blockSamples <= 0 || sampleRate <= 0.0 || bpm <= 0.0)
            return 0;

        int count = 0;
        const auto emit = [&](EngineMidiEventType type, int offset, int note, float value)
        {
            if (count >= maxEvents)
                return;
            EngineMidiEvent e;
            e.type = type;
            e.sampleOffset = offset;
            e.noteNumber = static_cast<std::uint8_t>(note);
            e.value = value;
            out[count++] = e;
        };
        const auto flushPending = [&]
        {
            for (int i = 0; i < pendingCount; ++i)
                emit(EngineMidiEventType::noteOff, 0, pending[static_cast<std::size_t>(i)].note, 0.0f);
            pendingCount = 0;
        };

        if (! playing)
        {
            if (wasPlaying)
            {
                flushPending();
                fillAsked = false;
                fillBar = -1;
                // Stopped, the next start begins at bar 0 with the chain last chosen.
                if (nextChainLength > 0)
                {
                    chain = nextChain;
                    chainLength = nextChainLength;
                    nextChainLength = 0;
                }
                chainStartBar = 0;
                previousChainLength = 0;
            }
            wasPlaying = false;
            return count;
        }

        const double beatsPerSample = bpm / 60.0 / sampleRate;
        const double beat1 = beat0 + static_cast<double>(blockSamples) * beatsPerSample;

        if (! wasPlaying || std::abs(beat0 - lastBeatEnd) > jumpTolerance)
        {
            if (wasPlaying)
                flushPending(); // a jump: the notes that rang belong to the old place
            lastBar = barOf(beat0);
            fillBar = -1;
        }
        wasPlaying = true;
        lastBeatEnd = beat1;

        if (fillAsked)
        {
            fillBar = barOf(beat0) + 1;
            fillAsked = false;
        }

        // A note sounds at the first sample at or after its time. Which block it falls in is decided by that whole
        // sample, not by comparing beats: two blocks' edges, worked out from two beat positions, may differ by a
        // rounding, and a note on the edge would then fall in both or in neither.
        const auto sampleOf = [&](double beat)
        {
            return static_cast<std::int64_t>(std::ceil((beat - beat0) / beatsPerSample - sampleTolerance));
        };

        // Every sixteenth whose hits can fall in this block: a swung or rolled one may start before it.
        const auto kFirst = static_cast<std::int64_t>(std::floor(beat0 / beatsPerStep)) - 1;
        const auto kLast = static_cast<std::int64_t>(std::floor(beat1 / beatsPerStep)) + 1;
        for (auto k = kFirst; k <= kLast; ++k)
        {
            const auto bar = floorDiv(k, stepsPerBar);
            advanceTo(bar);
            const auto segment = segmentAt(bar);
            const auto& pattern = patternFor(segment);
            const auto since = k - segment.runBar * stepsPerBar;

            const auto stepOf = [&](const SequencerLane& lane) -> const SequencerStep&
            {
                const auto position = static_cast<std::size_t>(floorMod(since, clampedLength(lane.length)));
                return segment.variationB ? lane.b[position] : lane.a[position];
            };

            const auto& accentStep = stepOf(pattern.accent);
            const bool accented = accentStep.on && passesChance(accentLaneId, k, accentStep.chance);
            const float velocity = accented ? settings.accentVelocity : settings.velocity;

            bool anySolo = false;
            for (const auto& lane : pattern.lanes)
                anySolo = anySolo || (lane.solo && lane.note >= 0);

            const double start = swungBeat(k);
            const double span = swungBeat(k + 1) - start;

            for (int l = 0; l < SequencerPattern::maxLanes; ++l)
            {
                const auto& lane = pattern.lanes[static_cast<std::size_t>(l)];
                if (lane.note < 0 || lane.note > 127 || lane.mute || (anySolo && ! lane.solo))
                    continue;
                const auto& step = stepOf(lane);
                if (! step.on || ! passesChance(l, k, step.chance))
                    continue;
                const int hits = std::clamp(static_cast<int>(step.hits), 1, 3);
                const double spacing = span / hits;
                for (int h = 0; h < hits; ++h)
                {
                    const double at = start + spacing * h;
                    const auto offset = sampleOf(at);
                    if (offset < 0 || offset >= blockSamples)
                        continue;
                    emit(EngineMidiEventType::noteOn, static_cast<int>(offset), lane.note, velocity);
                    addPending(at + spacing * 0.5, lane.note); // a short gate: the 808's drums play their tails out
                }
            }
        }

        // The note-offs due in this block.
        for (int i = 0; i < pendingCount;)
        {
            auto& p = pending[static_cast<std::size_t>(i)];
            if (const auto offset = sampleOf(p.beat); offset < blockSamples)
            {
                emit(EngineMidiEventType::noteOff, static_cast<int>(std::max<std::int64_t>(0, offset)), p.note, 0.0f);
                p = pending[static_cast<std::size_t>(--pendingCount)];
            }
            else
                ++i;
        }

        // In time order; at the same sample note-offs first, then by note: the same order whatever the block size.
        // An insertion sort: no allocation.
        for (int i = 1; i < count; ++i)
        {
            const auto e = out[i];
            const auto rank = [](const EngineMidiEvent& x) { return x.type == EngineMidiEventType::noteOff ? 0 : 1; };
            const auto before = [&](const EngineMidiEvent& x)
            {
                if (x.sampleOffset != e.sampleOffset)
                    return x.sampleOffset > e.sampleOffset;
                if (rank(x) != rank(e))
                    return rank(x) > rank(e);
                return x.noteNumber > e.noteNumber;
            };
            int j = i - 1;
            while (j >= 0 && before(out[j]))
            {
                out[j + 1] = out[j];
                --j;
            }
            out[j + 1] = e;
        }
        return count;
    }
}
