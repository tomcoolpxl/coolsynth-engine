// beatsync-probe: a console peer of the beat sync (CoolSynth's D361, docs/research/beat-sync-protocol.md). It joins
// the session on the network like any peer, never proposes a change, and logs every datagram in and out, the
// protocol's events, and once a second the session's timeline, its clock offset and the beat. For checks against
// other programs that speak the protocol.
//
//   beatsync-probe [--loopback] [--seconds N] [--bpm X] [--no-pings]
//
// --loopback: this computer only; --seconds: stop after N seconds (default: until Ctrl+C); --bpm: the tempo of
// the session it founds before it meets anyone (default 120); --no-pings: leave the clock measurements' pings and
// pongs out of the log.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "sync/SyncClock.h"
#include "sync/SyncNode.h"
#include "sync/SyncSockets.h"

namespace
{
    using namespace coolsynth::sync;

    std::atomic<bool> stopRequested { false };
    Micros startedAt = 0;

    void onSignal(int) { stopRequested = true; }

    double secondsSinceStart() { return static_cast<double>(hostMicrosNow() - startedAt) / 1.0e6; }

    std::string endpointText(Endpoint4 e) { return addressText(e.address) + ":" + std::to_string(e.port); }

    const char* socketName(SocketKind k)
    {
        switch (k)
        {
            case SocketKind::group: return "group";
            case SocketKind::measurement: return "measure";
            case SocketKind::discovery:
            default: return "discovery";
        }
    }

    std::string timelineText(const Timeline& t)
    {
        char s[160];
        std::snprintf(s, sizeof(s), "%.6f BPM (%lld us/beat), beat %.6f at %lld us", bpmFromTempo(t.tempo), static_cast<long long>(t.tempo),
                      toBeats(t.beatOrigin), static_cast<long long>(t.timeOrigin));
        return s;
    }

    std::string playText(const PlayState& p)
    {
        char s[120];
        std::snprintf(s, sizeof(s), "%s from beat %.6f, changed at %lld us", p.playing ? "playing" : "stopped", toBeats(p.beat),
                      static_cast<long long>(p.time));
        return s;
    }

    std::string describe(std::span<const std::uint8_t> bytes)
    {
        switch (kindOf(bytes))
        {
            case DatagramKind::discovery:
            {
                const auto m = readDiscovery(bytes);
                if (! m)
                    return "a discovery datagram that does not read (" + std::to_string(bytes.size()) + " bytes)";
                if (m->type == DiscoveryType::leaving)
                    return "leaving, peer " + idText(m->sender);
                std::string s = m->type == DiscoveryType::announce ? "announce" : "reply";
                s += ", peer " + idText(m->sender) + ", session " + idText(m->state.sessionId) + ", ttl " + std::to_string(m->timeToLive);
                s += "\n      timeline " + timelineText(m->state.timeline) + "\n      " + playText(m->state.playState);
                if (m->state.measurement)
                    s += "\n      measure at " + endpointText(*m->state.measurement);
                return s;
            }
            case DatagramKind::measurement:
            {
                const auto m = readMeasurement(bytes);
                if (! m)
                    return "a measurement datagram that does not read (" + std::to_string(bytes.size()) + " bytes)";
                std::string s = m->type == MeasurementType::ping ? "ping" : "pong";
                if (m->sessionId) s += ", session " + idText(*m->sessionId);
                if (m->sessionTime) s += ", gt " + std::to_string(*m->sessionTime);
                if (m->hostTime) s += ", ht " + std::to_string(*m->hostTime);
                if (m->prevSessionTime) s += ", pgt " + std::to_string(*m->prevSessionTime);
                return s;
            }
            case DatagramKind::unknown:
            default:
                return "an unknown datagram of " + std::to_string(bytes.size()) + " bytes";
        }
    }
}

int main(int argc, char** argv)
{
    bool loopbackOnly = false, pings = true;
    double seconds = 0.0, bpm = 120.0;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--loopback") == 0)
            loopbackOnly = true;
        else if (std::strcmp(argv[i], "--no-pings") == 0)
            pings = false;
        else if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc)
            seconds = std::atof(argv[++i]);
        else if (std::strcmp(argv[i], "--bpm") == 0 && i + 1 < argc)
            bpm = std::atof(argv[++i]);
        else
        {
            std::printf("usage: beatsync-probe [--loopback] [--seconds N] [--bpm X] [--no-pings]\n");
            return 2;
        }
    }
    std::signal(SIGINT, onSignal);
    startedAt = hostMicrosNow();

    Handoff handoff;
    handoff.local.write(encode(LocalGrid { tempoFromBpm(bpm), 0, startedAt }));

    SyncNode::Options options;
    options.loopbackOnly = loopbackOnly;
    options.logger = [](const std::string& text) { std::printf("%9.3f  %s\n", secondsSinceStart(), text.c_str()); std::fflush(stdout); };
    options.observer = [pings](bool outgoing, std::uint32_t gateway, SocketKind socket, Endpoint4 peer, std::span<const std::uint8_t> bytes)
    {
        if (! pings && kindOf(bytes) == DatagramKind::measurement)
            return;
        std::printf("%9.3f  %s %s %s %s  %s\n", secondsSinceStart(), addressText(gateway).c_str(), socketName(socket),
                    outgoing ? "->" : "<-", endpointText(peer).c_str(), describe(bytes).c_str());
        std::fflush(stdout);
    };
    options.everySecond = [](const SyncCore& core, Micros now)
    {
        const Micros sessionNow = core.sessionTime(now);
        std::printf("%9.3f  == session %s, %d peer(s), %d gateway(s), peer id %s\n"
                    "           timeline %s\n           clock offset %lld us, session time %lld us, beat %.6f, phase %.6f in 4\n"
                    "           %s\n",
                    secondsSinceStart(), idText(core.sessionId()).c_str(), core.peerCount(), core.gatewayCount(),
                    idText(core.peerId()).c_str(), timelineText(core.timeline()).c_str(), static_cast<long long>(core.clockOffset()),
                    static_cast<long long>(sessionNow), core.timeline().beatAtPrecise(sessionNow),
                    toBeats(phase(core.timeline().beatAt(sessionNow), 4 * kMicroBeatsPerBeat)), playText(core.playState()).c_str());
        std::fflush(stdout);
    };

    SyncNode node(handoff, options);
    node.start();
    while (! stopRequested && (seconds <= 0.0 || secondsSinceStart() < seconds))
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    node.stop();
    std::printf("%9.3f  stopped\n", secondsSinceStart());
    return 0;
}
