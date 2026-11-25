// The beat sync over real sockets (CoolSynth's D361): two of our peers in one process meeting over the loopback
// gateway. These need UDP multicast on the loopback interface; COOLSYNTH_SKIP_NETWORK_TESTS=1 skips them, and they
// skip themselves (with a message) where no gateway opens.

#include <juce_core/juce_core.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "sync/BeatSync.h"
#include "sync/SyncClock.h"
#include "sync/SyncNode.h"

namespace
{
    using namespace coolsynth::sync;

    bool networkTestsSkipped()
    {
        return juce::SystemStats::getEnvironmentVariable("COOLSYNTH_SKIP_NETWORK_TESTS", {}).getIntValue() != 0;
    }

    /** Polls a condition every millisecond for up to `limit`; the time it took, or -1. */
    double waitFor(double limitSeconds, const std::function<bool()>& condition)
    {
        const auto start = std::chrono::steady_clock::now();
        for (;;)
        {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (condition())
                return t;
            if (t > limitSeconds)
                return -1.0;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void writeLocalGrid(Handoff& h, double bpm, double beat)
    {
        h.local.write(encode(LocalGrid { tempoFromBpm(bpm), toMicroBeats(beat), hostMicrosNow() }));
    }
}

class SyncNodeNetworkTests final : public juce::UnitTest
{
public:
    SyncNodeNetworkTests() : juce::UnitTest("Beat sync network nodes", "Sync network") {}

    void runTest() override
    {
        beginTest("two_nodes_meet_on_the_loopback_and_part");
        if (networkTestsSkipped())
        {
            logMessage("skipped: COOLSYNTH_SKIP_NETWORK_TESTS");
            return;
        }
        {
            auto ha = std::make_unique<Handoff>(), hb = std::make_unique<Handoff>();
            writeLocalGrid(*ha, 120.0, 0.0);
            writeLocalGrid(*hb, 97.0, 3.0);
            SyncNode::Options options;
            options.loopbackOnly = true;
            auto a = std::make_unique<SyncNode>(*ha, options), b = std::make_unique<SyncNode>(*hb, options);

            a->start();
            waitFor(2.0, [&] { return a->settled(); });
            if (a->openGateways() == 0)
            {
                logMessage("skipped: no loopback gateway could open (no multicast on the loopback interface?)");
                return;
            }
            const Snapshot first = ha->readSnapshot();
            expect(first.running && first.peers == 0 && first.timeline.tempo == tempoFromBpm(120.0));
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            b->start();
            const double met = waitFor(3.0, [&]
            {
                const auto sa = ha->readSnapshot(), sb = hb->readSnapshot();
                return sa.peers == 1 && sb.peers == 1 && sa.session == sb.session;
            });
            logMessage("met in " + juce::String(met * 1000.0, 1) + " ms");
            expect(met >= 0.0 && met < 1.0, "they find each other within a second");
            const auto sa = ha->readSnapshot(), sb = hb->readSnapshot();
            expect(sb.session == first.session, "the one switched on second joins the first");
            expect(sb.timeline == sa.timeline, "one timeline");
            const Micros offsetGap = sb.clockOffset - sa.clockOffset;
            logMessage("the clock offsets differ by " + juce::String((int) offsetGap) + " us on one clock");
            expect(std::abs(offsetGap) < 50, "within 50 us");

            b->stop();
            const double parted = waitFor(1.0, [&] { return ha->readSnapshot().peers == 0; });
            logMessage("parted in " + juce::String(parted * 1000.0, 1) + " ms");
            expect(parted >= 0.0 && parted < 0.1, "the peer count falls to 0 within 100 ms");
            expect(ha->readSnapshot().session != first.session, "and a new session is founded");
            expect(! hb->readSnapshot().running);
            a->stop();
        }
    }
};

namespace
{
    /** Two apps' beat syncs in one process, their audio sides driven by this thread in 1 ms blocks on the real host
        clock, block after block without gaps, as a device would. */
    struct TwoApps
    {
        static constexpr int samples = 48;
        static constexpr double usPerSample = 1.0e6 / 48000.0; // 48 samples = 1000 us

        std::unique_ptr<BeatSync> a, b;
        Micros origin = 0;
        std::int64_t blocks = 0;
        BlockBeats lastA {}, lastB {};
        double endA = 0.0, worstJumpA = 0.0; // A's largest step between one block's end and the next's start
        bool watchA = false;

        TwoApps(double bpmA, double bpmB)
        {
            BeatSync::Options o;
            o.loopbackOnly = true;
            a = std::make_unique<BeatSync>(bpmA, o);
            b = std::make_unique<BeatSync>(bpmB, o);
            origin = hostMicrosNow();
        }

        Micros hostOf(std::int64_t block) const { return origin + block * 1000; }

        /** The blocks due by now. */
        void catchUp()
        {
            const std::int64_t due = (hostMicrosNow() - origin) / 1000;
            while (blocks < due)
                step();
        }

        void step()
        {
            const Micros h = hostOf(blocks++);
            lastA = a->audio().beginBlock(h, samples, usPerSample);
            lastB = b->audio().beginBlock(h, samples, usPerSample);
            if (watchA)
                worstJumpA = std::max(worstJumpA, std::abs(lastA.beat - endA));
            endA = lastA.beat + lastA.beatsPerSample * samples;
        }

        /** Runs blocks until the condition holds; seconds it took, or -1. */
        double until(double limitSeconds, const std::function<bool()>& condition)
        {
            return waitFor(limitSeconds, [&] { catchUp(); return condition(); });
        }

        void runFor(double seconds) { until(seconds, [] { return false; }); }

        /** How far apart the two grids are at the same moment, in microseconds: their phases in a bar. */
        double phaseGapMicros() const
        {
            double gap = std::fmod(lastA.beat - lastB.beat, 4.0);
            if (gap > 2.0) gap -= 4.0;
            if (gap < -2.0) gap += 4.0;
            return gap * 60.0e6 / lastA.bpm;
        }
    };
}

class SyncTwoPeerNetworkTests final : public juce::UnitTest
{
public:
    SyncTwoPeerNetworkTests() : juce::UnitTest("Beat sync network two apps", "Sync network") {}

    void runTest() override
    {
        beginTest("two_apps_on_the_loopback");
        if (networkTestsSkipped())
        {
            logMessage("skipped: COOLSYNTH_SKIP_NETWORK_TESTS");
            return;
        }
        auto rig = std::make_unique<TwoApps>(120.0, 97.0);
        auto& r = *rig;
        r.a->setEnabled(true);
        r.until(2.0, [&] { return r.a->status().gateways > 0; });
        if (r.a->status().gateways == 0)
        {
            logMessage("skipped: no loopback gateway could open (no multicast on the loopback interface?)");
            return;
        }
        r.runFor(1.0);
        const PeerId first = r.a->status().session;

        // Discovery and joining.
        r.watchA = true;
        r.b->setEnabled(true);
        const double met = r.until(2.0, [&] { return r.lastA.peers == 1 && r.lastB.peers == 1; });
        logMessage("discovery and joining: " + juce::String(met * 1000.0, 1) + " ms");
        expect(met >= 0.0 && met < 1.0, "they find each other within a second");
        r.runFor(0.05);
        expect(r.b->status().session == first && r.a->status().session == first, "the one switched on second joins the first");
        expectWithinAbsoluteError(r.lastB.bpm, 120.0, 1.0e-9, "and takes its tempo");
        logMessage("grids apart by " + juce::String(r.phaseGapMicros(), 2) + " us; A's largest step "
                   + juce::String(r.worstJumpA * 500000.0, 2) + " us");
        expect(std::abs(r.phaseGapMicros()) < 50.0, "the beat aligned within 50 us");
        expect(r.worstJumpA * 500000.0 < 1.0, "A's beat did not move");

        // Tempo both ways.
        for (const bool fromA : { true, false })
        {
            const double bpm = fromA ? 133.5 : 97.0;
            auto& from = fromA ? *r.a : *r.b;
            r.catchUp();
            from.audio().setTempo(bpm);
            const Micros expected = tempoFromBpm(bpm);
            const double took = r.until(1.0, [&]
            {
                return tempoFromBpm((fromA ? r.lastB : r.lastA).bpm) == expected;
            });
            r.runFor(0.02);
            logMessage(juce::String(fromA ? "A" : "B") + " to " + juce::String(bpm, 1) + " BPM: the other followed in "
                       + juce::String(took * 1000.0, 1) + " ms; grids apart by " + juce::String(r.phaseGapMicros(), 2) + " us");
            expect(took >= 0.0 && took < 0.1, "a tempo change reaches the other within 100 ms");
            expect(std::abs(r.phaseGapMicros()) < 50.0, "the grids agree within 50 us");
        }
        expect(r.a->status().bpm > 96.9 && r.a->status().bpm < 97.1);

        // Start and stop both ways, sharing on in both.
        for (const bool fromA : { true, false })
        {
            auto& from = fromA ? *r.a : *r.b;
            r.catchUp();
            from.audio().play(0);
            const double started = r.until(1.0, [&] { return r.lastA.playing && r.lastB.playing; });
            r.runFor(0.01);
            const double gap = (r.lastA.beat - r.lastB.beat) * 60.0e6 / r.lastA.bpm;
            logMessage(juce::String(fromA ? "A" : "B") + " started: the other in " + juce::String(started * 1000.0, 1)
                       + " ms, their beats " + juce::String(gap, 2) + " us apart (" + juce::String(r.lastA.beat, 3) + ")");
            expect(started >= 0.0 && started < 0.1, "a start on one starts the other");
            expect(std::abs(gap) < 50.0, "on the same quantum boundary");
            r.runFor(0.2);
            r.catchUp();
            (fromA ? *r.b : *r.a).audio().stop(0);
            const double stopped = r.until(1.0, [&] { return ! r.lastA.playing && ! r.lastB.playing; });
            expect(stopped >= 0.0 && stopped < 0.1, "a stop on the other stops both");
        }

        // Sharing off in B: B does not follow, but still relays.
        r.b->setStartStopSharing(false);
        r.catchUp();
        r.a->audio().play(0);
        r.runFor(0.3);
        expect(r.lastA.playing && ! r.lastB.playing, "B, sharing off, does not follow");
        expect(r.b->status().sessionPlaying, "but takes and relays the session's play state");
        r.catchUp();
        r.a->audio().stop(0);
        r.runFor(0.05);
        r.b->setStartStopSharing(true);

        // B switched off: A alone again at once, its beat continuous.
        r.worstJumpA = 0.0;
        r.b->setEnabled(false);
        const double alone = r.until(1.0, [&] { return r.lastA.peers == 0; });
        r.runFor(0.05);
        logMessage("B off: A alone after " + juce::String(alone * 1000.0, 1) + " ms, its largest step "
                   + juce::String(r.worstJumpA * 60.0e6 / r.lastA.bpm, 2) + " us");
        expect(alone >= 0.0 && alone < 0.1, "the peer count falls to 0 within 100 ms");
        expect(r.a->status().session != first, "a new session founded");
        expect(r.worstJumpA * 60.0e6 / r.lastA.bpm < 1.0, "with a continuous beat");
        r.a->setEnabled(false);
    }
};

static SyncNodeNetworkTests syncNodeNetworkTests;
static SyncTwoPeerNetworkTests syncTwoPeerNetworkTests;
