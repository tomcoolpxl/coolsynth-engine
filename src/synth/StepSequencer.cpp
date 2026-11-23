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
        settings.swing = std::isfinite(s.swing) ? std::clamp(s.swing, 0.5f, 0.75f) : 0.5f;
        settings.velocity = std::isfinite(s.velocity) ? std::clamp(s.velocity, 0.0f, 1.0f) : 1.0f / 127.0f;
        settings.accentVelocity = std::isfinite(s.accentVelocity) ? std::clamp(s.accentVelocity, 0.0f, 1.0f) : 1.0f;
    }

    void StepSequencer::setChain(const int* patterns, int count) noexcept
    {
        if (patterns == nullptr || count <= 0)
            return;
        count = std::min(count, maxChain);
        auto& target = wasPlaying ? nextChain : chain;
        for (int i = 0; i < count; ++i)
            target[static_cast<std::size_t>(i)] = std::clamp(patterns[i], 0, SequencerBank::patterns - 1);
        nextChainAtBar = false;
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

    void StepSequencer::setChainFrom(const int* patterns, int count, std::int64_t bar) noexcept
    {
        if (patterns == nullptr || count <= 0)
            return;
        setChain(patterns, count);
        nextChainAtBar = wasPlaying;
        nextChainBar = bar;
    }

    void StepSequencer::reset() noexcept
    {
        if (nextChainLength > 0)
        {
            chain = nextChain;
            chainLength = nextChainLength;
            nextChainLength = 0;
            nextChainAtBar = false;
        }
        chainStartBar = 0;
        previousChainLength = 0;
        fillAsked = false;
        fillBar = -1;
        wasPlaying = false;
        doneValid = false;
        pendingCount = 0;
    }

    double StepSequencer::hitPlaceAt(double beat) const noexcept
    {
        // A swung sixteenth starts at or after its straight place and before the next one's.
        auto k = static_cast<std::int64_t>(std::floor(beat / beatsPerStep));
        if (beat < swungBeat(k))
            --k;
        const double start = swungBeat(k);
        return static_cast<double>(k) + (beat - start) / (swungBeat(k + 1) - start);
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

    void StepSequencer::switchChainAt(std::int64_t bar) noexcept
    {
        previousChain = chain;
        previousChainLength = chainLength;
        previousChainStartBar = chainStartBar;
        chain = nextChain;
        chainLength = nextChainLength;
        chainStartBar = bar;
        nextChainLength = 0;
        nextChainAtBar = false;
    }

    void StepSequencer::advanceTo(std::int64_t bar) noexcept
    {
        while (lastBar < bar)
        {
            ++lastBar;
            // A chain set while playing takes over at the end of the pattern that plays, or at its own bar.
            if (nextChainLength > 0
                && (nextChainAtBar ? lastBar >= nextChainBar : floorMod(lastBar - chainStartBar, barsPerPattern()) == 0))
                switchChainAt(nextChainAtBar ? nextChainBar : lastBar);
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

    int StepSequencer::process(double beat0, double bpm, bool playing, int blockSamples, double sampleRate,
                               EngineMidiEvent* out, int maxEvents) noexcept
    {
        // A position or tempo no transport gives (not a number, or past any song's length) plays nothing; the tempo is
        // held to what a drum machine plays, so a host's bad value never runs the step loop away (review of D266).
        if (out == nullptr || maxEvents <= 0 || blockSamples <= 0 || ! (sampleRate > 0.0) || ! (bpm > 0.0)
            || ! std::isfinite(beat0) || ! std::isfinite(bpm) || std::abs(beat0) > 1.0e12)
            return 0;
        bpm = std::clamp(bpm, 1.0, 1000.0);

        int count = 0;
        const auto emit = [&](EngineMidiEventType type, int offset, int note, float value)
        {
            if (count >= maxEvents)
                return false;
            EngineMidiEvent e;
            e.type = type;
            e.sampleOffset = offset;
            e.noteNumber = static_cast<std::uint8_t>(note);
            e.value = value;
            out[count++] = e;
            return true;
        };
        // The note-offs at offset 0, as far as there is room; any left wait for the next block.
        const auto flushPending = [&]
        {
            int kept = 0;
            for (int i = 0; i < pendingCount; ++i)
                if (const auto& p = pending[static_cast<std::size_t>(i)]; ! emit(EngineMidiEventType::noteOff, 0, p.note, 0.0f))
                    pending[static_cast<std::size_t>(kept++)] = { -1.0e300, p.note };
            pendingCount = kept;
        };

        if (! playing)
        {
            if (wasPlaying || pendingCount > 0)
                flushPending();
            fillAsked = false; // asked while stopped: nothing (review of D266)
            doneValid = false;
            if (wasPlaying)
            {
                fillBar = -1;
                // Stopped, the next start begins at bar 0 with the chain last chosen.
                if (nextChainLength > 0)
                {
                    chain = nextChain;
                    chainLength = nextChainLength;
                    nextChainLength = 0;
                    nextChainAtBar = false;
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
            {
                flushPending(); // a jump: the notes that rang belong to the old place
                // ...and so does the chain that played before the last switch: every bar now plays the chain
                // chosen, in its own place (a loop back to before a switch, review of D266).
                previousChainLength = 0;
            }
            lastBar = barOf(beat0);
            fillBar = -1;
            doneValid = false;
        }
        wasPlaying = true;
        lastBeatEnd = beat1;

        const auto barNow = barOf(beat0);
        if (fillAsked)
        {
            // Asked again while the fill plays or waits: the one asked for stands (review of D266).
            if (fillBar < barNow)
                fillBar = barNow + 1;
            fillAsked = false;
        }
        // A chain set just after the last look ahead crossed the end of the pattern that plays: it takes over there,
        // not a whole pattern later (review of D266).
        if (nextChainLength > 0)
        {
            const int per = barsPerPattern();
            const auto boundary = nextChainAtBar ? nextChainBar : barNow + per - floorMod(barNow - chainStartBar, per);
            if (boundary <= lastBar)
                switchChainAt(boundary);
        }

        // A note sounds at the first sample at or after its time. Which block it falls in is decided by that whole
        // sample, not by comparing beats: two blocks' edges, worked out from two beat positions, may differ by a
        // rounding, and a note on the edge would then fall in both or in neither.
        const auto sampleOf = [&](double beat)
        {
            return static_cast<std::int64_t>(std::ceil((beat - beat0) / beatsPerSample - sampleTolerance));
        };

        // The place of this block's last sample: the hits up to it play in this block.
        const double doneNow = hitPlaceAt(beat0 + (static_cast<double>(blockSamples) - 1.0 + sampleTolerance) * beatsPerSample);

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
                    // Each hit has its turn once, in the block whose last sample's place it reaches. Its place (the
                    // sixteenth and how far into it) stays when the swing or the tempo moves its time: one that had
                    // its turn is not played again, and one a move put behind this block's start, which never had
                    // it, plays at once (review of D266). After a start or a jump, what lies before the block is not
                    // this run's.
                    const double place = static_cast<double>(k) + static_cast<double>(h) / hits;
                    if (place > doneNow || (doneValid && place <= doneUpTo))
                        continue;
                    auto offset = sampleOf(at);
                    if (offset < 0 && ! doneValid)
                        continue;
                    offset = std::clamp<std::int64_t>(offset, 0, blockSamples - 1);
                    // With nowhere to keep its note-off (hundreds of hits in one block), a hit is let go rather than
                    // left sounding.
                    if (pendingCount >= maxPending || ! emit(EngineMidiEventType::noteOn, static_cast<int>(offset), lane.note, velocity))
                        continue;
                    // A short gate: the 808's drums play their tails out.
                    pending[static_cast<std::size_t>(pendingCount++)] = { at + spacing * 0.5, lane.note };
                }
            }
        }
        // Every hit up to the last sample of this block has had its turn (never back: a swing turned up moves the
        // places behind a time).
        doneUpTo = doneValid ? std::max(doneUpTo, doneNow) : doneNow;
        doneValid = true;

        // The note-offs due in this block (one past its time, or kept from a full block, at once).
        for (int i = 0; i < pendingCount;)
        {
            auto& p = pending[static_cast<std::size_t>(i)];
            const auto offset = p.beat <= beat0 ? std::int64_t { 0 } : sampleOf(p.beat);
            if (offset < blockSamples && emit(EngineMidiEventType::noteOff, static_cast<int>(std::max<std::int64_t>(0, offset)), p.note, 0.0f))
                p = pending[static_cast<std::size_t>(--pendingCount)];
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
