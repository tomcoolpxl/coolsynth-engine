// The beat sync's audio side (CoolSynth's D361): the handoff, the audio thread's calls and the block clock, from
// docs/research/beat-sync-protocol.md's test plan ("The handoff and the audio thread"), with snapshots written by the
// test as the network thread would.

#include <juce_core/juce_core.h>

#include <atomic>
#include <cmath>
#include <memory>
#include <random>
#include <thread>

#include "diagnostics/AudioThreadAlarm.h"
#include "sync/BeatSync.h"
#include "sync/BlockClock.h"

namespace
{
    using namespace coolsynth::sync;

    constexpr double rate = 48000.0, usPerSample = 1.0e6 / rate;
    constexpr int blockSamples = 480; // 10 ms
    constexpr Micros blockMicros = 10000;
    constexpr Micros startHost = 1'000'000'000;

    /** An audio side over a handoff, driven block by block on a clock of the test's. */
    struct Rig
    {
        std::unique_ptr<Handoff> handoff = std::make_unique<Handoff>();
        std::unique_ptr<BeatSyncAudio> audio;
        Micros host = startHost;
        std::int64_t number = 0;
        BlockBeats last {};

        explicit Rig(double bpm = 120.0) { audio = std::make_unique<BeatSyncAudio>(*handoff, bpm, startHost); }

        const BlockBeats& block()
        {
            last = audio->beginBlock(host, blockSamples, usPerSample);
            host += blockMicros;
            return last;
        }

        void blocks(int n)
        {
            for (int i = 0; i < n; ++i)
                block();
        }

        /** The session founded as the network thread founds it, from the audio side's own grid. */
        Snapshot found(Micros at)
        {
            SeqLock<kLocalWords>::Words w {};
            handoff->local.read(w);
            const LocalGrid local = decodeLocal(w);
            Snapshot s;
            s.running = true;
            s.timeline = { local.tempo, local.appBeat + beatsInDuration(at - local.hostTime, local.tempo), 0 };
            s.clockOffset = -at;
            s.epoch = 1;
            return s;
        }

        void publish(Snapshot s)
        {
            s.number = ++number;
            handoff->snapshot.write(encode(s));
        }

        GridProposal grid() const
        {
            SeqLock<kGridWords>::Words w {};
            handoff->grid.read(w);
            return decodeGrid(w);
        }

        PlayProposal play() const
        {
            SeqLock<kPlayWords>::Words w {};
            handoff->play.read(w);
            return decodePlay(w);
        }

        /** The session beat at host time h of a snapshot. */
        static double sessionBeat(const Snapshot& s, Micros h) { return s.timeline.beatAtPrecise(h + s.clockOffset); }
    };
}

class SyncHandoffTests final : public juce::UnitTest
{
public:
    SyncHandoffTests() : juce::UnitTest("Beat sync handoff", "Sync") {}

    void runTest() override
    {
        beginTest("a_reader_racing_a_writer_never_sees_a_torn_snapshot");
        {
            auto lock = std::make_unique<SeqLock<kSnapshotWords>>();
            std::atomic<bool> done { false };
            std::thread writer([&]
            {
                SeqLock<kSnapshotWords>::Words w {};
                for (std::int64_t i = 1; ! done.load(std::memory_order_relaxed); ++i)
                {
                    for (std::size_t j = 0; j < w.size(); ++j)
                        w[j] = i * 1000 + static_cast<std::int64_t>(j);
                    lock->write(w);
                }
            });
            int torn = 0, good = 0, busy = 0;
            std::int64_t newest = 0;
            const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
            while (std::chrono::steady_clock::now() < end)
            {
                SeqLock<kSnapshotWords>::Words w {};
                if (! lock->tryRead(w))
                {
                    ++busy;
                    continue;
                }
                if (w[0] == 0 && newest == 0)
                    continue; // the zeros from before the first write
                ++good;
                for (std::size_t j = 1; j < w.size(); ++j)
                    if (w[j] - w[0] != static_cast<std::int64_t>(j))
                    {
                        ++torn;
                        break;
                    }
                if (w[0] / 1000 < newest)
                    ++torn; // never older than a copy already read
                newest = std::max(newest, w[0] / 1000);
            }
            done = true;
            writer.join();
            logMessage(juce::String(good) + " consistent reads, " + juce::String(busy) + " during a write, " + juce::String(torn) + " torn; "
                       + juce::String((juce::int64) lock->writes()) + " writes");
            expectEquals(torn, 0, "never torn");
            expectGreaterThan(good, 1000);
        }

        beginTest("snapshots_and_proposals_round_trip");
        {
            Snapshot s;
            s.number = 7;
            s.session = { 0x21, 0x7E, 0x40, 0x41, 0x42, 0x43, 0xFF, 0x00 };
            s.peers = 3;
            s.running = true;
            s.startStopSharing = false;
            s.timeline = { 468750, -16500000, 90000000 };
            s.clockOffset = -123456789;
            s.playState = { true, 16500000, 90000123 };
            s.playChange = 4;
            s.gridApplied = 9;
            s.playApplied = 2;
            s.epoch = 5;
            const Snapshot t = decodeSnapshot(encode(s));
            expect(t.number == 7 && t.session == s.session && t.peers == 3 && t.running && ! t.startStopSharing
                   && t.timeline == s.timeline && t.clockOffset == s.clockOffset && t.playState == s.playState
                   && t.playChange == 4 && t.gridApplied == 9 && t.playApplied == 2 && t.epoch == 5);
        }
    }
};

class SyncAudioSideTests final : public juce::UnitTest
{
public:
    SyncAudioSideTests() : juce::UnitTest("Beat sync audio side", "Sync") {}

    void runTest() override
    {
        beginTest("alone_the_beat_runs_from_zero");
        {
            auto r = std::make_unique<Rig>(120.0);
            r->blocks(100); // one second
            const auto& b = r->block();
            expectWithinAbsoluteError(b.beat, 2.0, 1.0e-9);
            expectWithinAbsoluteError(b.beatsPerSample, 2.0 / rate, 1.0e-15);
            expect(! b.connected && ! b.playing && b.peers == 0);
            expectWithinAbsoluteError(b.phase, 2.0, 1.0e-9);
        }

        beginTest("founding_keeps_the_beat_joining_moves_it_at_most_half_a_quantum");
        {
            auto r = std::make_unique<Rig>(120.0);
            r->blocks(523);
            const double before = r->block().beat;
            r->publish(r->found(r->host));
            const double founded = r->block().beat;
            expectWithinAbsoluteError(founded - before, blockMicros / 500000.0, 1.0e-6, "founding: the beat continuous");

            // Joined: another session's grid at 128 BPM, its phase elsewhere.
            Snapshot joined;
            joined.running = true;
            joined.peers = 1;
            joined.epoch = 2;
            joined.clockOffset = 777'000'000 - r->host;
            joined.timeline = { tempoFromBpm(128.0), toMicroBeats(37.3), 776'000'000 };
            joined.playState = { true, toMicroBeats(30.0), 775'000'000 };
            const double expected = r->last.beat + r->last.beatsPerSample * blockSamples;
            r->publish(joined);
            const auto& b = r->block();
            const double session = Rig::sessionBeat(joined, r->host - blockMicros);
            logMessage("joining moved the beat by " + juce::String(b.beat - expected, 4) + " beats");
            expect(std::abs(b.beat - expected) <= 2.0 + 1.0e-9, "by at most half a quantum");
            const double phaseGap = std::fmod(b.beat - session + 400.0, 4.0);
            expect(phaseGap < 1.0e-6 || phaseGap > 4.0 - 1.0e-6, "on the session's phase");
            expectWithinAbsoluteError(b.bpm, 128.0, 1.0e-9);
            expect(b.connected && b.peers == 1);
            expect(! b.playing, "the session plays, but a joining peer does not start");
        }

        beginTest("following_a_start_and_a_stop");
        {
            auto r = std::make_unique<Rig>(120.0);
            r->blocks(10);
            Snapshot s = r->found(r->host);
            s.peers = 1;
            r->publish(s);
            r->blocks(3);
            // Another peer starts at session beat 13.7: we start at 16, counting in from -2.3.
            const Micros pressedHost = s.timeline.timeAt(toMicroBeats(13.7)) - s.clockOffset;
            r->host = pressedHost;
            s.playState = { true, toMicroBeats(13.7), pressedHost + s.clockOffset };
            s.playChange = 1;
            r->publish(s);
            const auto& b = r->block();
            expect(b.playing, "follows the start");
            expectWithinAbsoluteError(b.beat, -2.3, 1.0e-6, "counting in to the next bar");
            r->blocks(200);
            expect(r->last.playing && r->last.beat > 0.0);

            // A stop at a beat already past: at once.
            s.playState = { false, toMicroBeats(14.0), pressedHost + s.clockOffset + 1 };
            s.playChange = 2;
            r->publish(s);
            expect(! r->block().playing, "follows the stop");

            // With sharing off, a start is not followed.
            r->handoff->startStopSharing = false;
            s.playState = { true, toMicroBeats(20.0), pressedHost + s.clockOffset + 2 };
            s.playChange = 3;
            r->publish(s);
            expect(! r->block().playing, "sharing off: not followed");
        }

        beginTest("our_own_proposal_stands_until_applied");
        {
            auto r = std::make_unique<Rig>(120.0);
            r->blocks(10);
            Snapshot s = r->found(r->host);
            r->publish(s);
            r->blocks(2);
            const double start = r->last.beat + r->last.beatsPerSample * blockSamples;
            r->block();
            const auto& t = r->audio->setTempo(140.0);
            expectWithinAbsoluteError(t.bpm, 140.0, 1.0e-3);
            expectWithinAbsoluteError(t.beat, start, 1.0e-6, "continuous at the block's start");
            const auto g = r->grid();
            expect(g.number == 1 && g.tempo == 428571, "a grid proposal posted");
            r->publish(s); // a snapshot made before the network thread saw it
            expectWithinAbsoluteError(r->block().bpm, 140.0, 1.0e-3, "does not undo it");
            s.gridApplied = 1;
            s.timeline = timelineThrough(s.timeline, g.tempo, g.beat, g.hostTime + s.clockOffset);
            const double before = r->last.beat + r->last.beatsPerSample * blockSamples;
            r->publish(s);
            const auto& c = r->block();
            expectWithinAbsoluteError(c.bpm, 140.0, 1.0e-3);
            expectWithinAbsoluteError(c.beat, before, 1.0e-6, "the applied grid is the same grid");
            expect(r->audio->setTempo(140.0).bpm > 139.0 && r->grid().number == 1, "the same tempo again posts nothing");
        }

        beginTest("alone_play_starts_at_once_with_peers_on_the_bar");
        {
            auto r = std::make_unique<Rig>(120.0);
            r->blocks(137); // beat 2.74
            r->block();
            const auto& b = r->audio->play(100);
            expect(b.playing);
            expectWithinAbsoluteError(b.beat + 100 * b.beatsPerSample, 0.0, 1.0e-6, "alone: beat 0 at the press");
            expect(r->grid().number == 1 && r->grid().beat == toMicroBeats(4.0), "the grid shifted to put a bar at the press");
            expect(r->play().number == 1 && r->play().playing, "a play proposal posted");
            r->blocks(50);
            expectWithinAbsoluteError(r->last.phase, std::fmod(r->last.beat, 4.0), 1.0e-9);

            auto q = std::make_unique<Rig>(120.0);
            q->blocks(10);
            Snapshot s = q->found(q->host);
            s.peers = 1;
            q->publish(s);
            q->host = s.timeline.timeAt(toMicroBeats(13.7)) - s.clockOffset;
            q->block();
            const auto& c = q->audio->play(0);
            expectWithinAbsoluteError(c.beat, -2.3, 1.0e-6, "with peers: counting in to the next bar");
            expect(q->grid().number == 0, "and the grid untouched");

            const auto& d = q->audio->stop(200);
            expect(d.playing && d.stopSample == 200, "a stop inside the block");
            expect(! q->block().playing, "stopped from the next block");
            expect(q->play().number == 2 && ! q->play().playing);
        }

        beginTest("the_audio_calls_take_no_memory_and_never_wait");
        {
            auto r = std::make_unique<Rig>(120.0);
            r->blocks(5);
            Snapshot s = r->found(r->host);
            const bool watching = coolsynth::diag::watchBlockingCalls();
            coolsynth::diag::resetAudioAllocations();
            for (int i = 0; i < 400; ++i)
            {
                if (i % 50 == 0)
                {
                    s.peers = (i / 50) % 2;
                    s.playChange += 1;
                    s.playState = { (i / 50) % 3 == 0, s.timeline.beatAt(r->host + s.clockOffset), r->host + s.clockOffset };
                    s.gridApplied = r->grid().number;
                    s.playApplied = r->play().number;
                    r->publish(s); // outside the audio block, as the network thread
                }
                const coolsynth::diag::AudioBlockScope block;
                r->block();
                if (i % 7 == 0)
                    r->audio->setTempo(100.0 + i % 40);
                if (i % 31 == 0)
                    r->audio->play(i % 480);
                if (i % 43 == 0)
                    r->audio->stop(i % 480);
                if (i % 97 == 0)
                    r->audio->setQuantum(i % 2 == 0 ? 4.0 : 8.0);
            }
            expectEquals(coolsynth::diag::audioAllocations.count.load(), 0, "no allocation");
            if (watching)
                expectEquals(coolsynth::diag::audioBlockingCalls.count.load(), 0, "no lock, wait, sleep or file");
        }
    }
};

class SyncBlockClockTests final : public juce::UnitTest
{
public:
    SyncBlockClockTests() : juce::UnitTest("Beat sync block clock", "Sync") {}

    void runTest() override
    {
        beginTest("recovers_a_line_from_callbacks_with_1_ms_of_jitter");
        {
            auto clock = std::make_unique<BlockClock>(rate);
            std::mt19937 rng(5);
            std::uniform_real_distribution<double> jitter(-500.0, 500.0); // 1 ms from earliest to latest
            const double truePerSample = usPerSample * (1.0 + 30.0e-6);   // the device 30 ppm off the host clock
            constexpr int block = 256;
            double squares = 0.0, worst = 0.0;
            int counted = 0;
            BlockClock::Time t {};
            for (int i = 0; i < 6000; ++i) // 32 s
            {
                const double truth = 5.0e9 + static_cast<double>(i) * block * truePerSample;
                t = clock->next(std::llround(truth + jitter(rng)), block);
                if (i >= BlockClock::kPairs)
                {
                    const double e = static_cast<double>(t.hostMicros) - truth;
                    squares += e * e;
                    worst = std::max(worst, std::abs(e));
                    ++counted;
                }
            }
            const double rms = std::sqrt(squares / counted);
            logMessage("after the ring fills: " + juce::String(rms, 1) + " us rms, worst " + juce::String(worst, 1)
                       + " us; slope " + juce::String((t.microsPerSample / truePerSample - 1.0) * 1.0e9, 2) + " ppb off");
            expectLessThan(rms, 20.0, "within 20 us rms");
            expectLessThan(worst, 80.0);
            expectLessThan(std::abs(t.microsPerSample / truePerSample - 1.0), 1.0e-6, "its slope the real rate");
        }

        beginTest("a_dropout_restarts_it");
        {
            auto clock = std::make_unique<BlockClock>(rate);
            Micros h = 1'000'000;
            for (int i = 0; i < 100; ++i, h += 5333)
                clock->next(h, 256);
            h += 300000; // a 300 ms gap
            const auto t = clock->next(h, 256);
            expectEquals(t.hostMicros, h, "starts again from the callback's own time");
            expectWithinAbsoluteError(t.microsPerSample, usPerSample, 1.0e-12);
        }
    }
};

static SyncHandoffTests syncHandoffTests;
static SyncAudioSideTests syncAudioSideTests;
static SyncBlockClockTests syncBlockClockTests;
