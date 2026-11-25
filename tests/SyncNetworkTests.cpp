// The beat sync over real sockets (CoolSynth's D361): two of our peers in one process meeting over the loopback
// gateway. These need UDP multicast on the loopback interface; COOLSYNTH_SKIP_NETWORK_TESTS=1 skips them, and they
// skip themselves (with a message) where no gateway opens.

#include <juce_core/juce_core.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

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

static SyncNodeNetworkTests syncNodeNetworkTests;
