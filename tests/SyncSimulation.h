#pragma once

// A simulated network for the beat sync's core (CoolSynth's D361): peers with clocks of their own, one gateway each,
// datagrams delivered after random delays, losses and partitions chosen by the test. Fast and repeatable.

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "sync/SyncCore.h"

namespace coolsynth::sync::testing
{
    class SimNetwork
    {
    public:
        struct Peer
        {
            std::unique_ptr<SyncCore> core;
            Micros clockBase = 0; // host time = simulated time + clockBase, running fast by ppm parts per million
            double ppm = 0.0;
            std::uint32_t address = 0;
            std::uint16_t discoveryPort = 0, measurementPort = 0;
            int segment = 0;
            bool alive = false;
        };

        struct Sent
        {
            Micros at = 0; // simulated time
            int peer = 0;
            Outgoing out;
        };

        explicit SimNetwork(std::uint32_t seed = 1) : rng(seed) {}

        Micros now = 0;
        Micros minDelay = 0, maxDelay = 0;
        Micros tickStep = 1000;
        bool bridged = false;
        /** Return true to lose a datagram. */
        std::function<bool(int fromPeer, const Outgoing&)> lose;
        std::vector<Sent> sent;

        int add(Micros clockBase, int segment = 0)
        {
            auto p = std::make_unique<Peer>();
            const int index = static_cast<int>(peers.size());
            p->core = std::make_unique<SyncCore>(1000u + static_cast<unsigned>(index));
            p->clockBase = clockBase;
            p->address = 0x0A000001u + static_cast<std::uint32_t>(index);
            p->discoveryPort = static_cast<std::uint16_t>(40000 + index);
            p->measurementPort = static_cast<std::uint16_t>(41000 + index);
            p->segment = segment;
            peers.push_back(std::move(p));
            return index;
        }

        Peer& peer(int i) { return *peers[static_cast<std::size_t>(i)]; }
        SyncCore& core(int i) { return *peer(i).core; }
        Micros host(int i) { return peer(i).clockBase + now + std::llround(static_cast<double>(now) * peer(i).ppm * 1.0e-6); }
        int size() const { return static_cast<int>(peers.size()); }

        void start(int i, double bpm = 120.0, MicroBeats beat = 0)
        {
            auto& p = peer(i);
            p.alive = true;
            p.core->found(host(i), tempoFromBpm(bpm), beat, {});
            p.core->addGateway(0, { p.address, p.measurementPort }, host(i));
            flush(i);
        }

        /** Switched off: leaving messages, then silent. */
        void stop(int i)
        {
            peer(i).core->leaveAll(host(i));
            flush(i);
            peer(i).alive = false;
        }

        /** Gone without a word. */
        void kill(int i)
        {
            peer(i).core->outbox().clear();
            peer(i).alive = false;
        }

        void flush(int i)
        {
            auto& p = peer(i);
            auto& box = p.core->outbox();
            for (const auto& o : box)
            {
                sent.push_back({ now, i, o });
                if (lose && lose(i, o))
                    continue;
                const std::uint16_t fromPort = o.from == SocketKind::measurement ? p.measurementPort : p.discoveryPort;
                const Endpoint4 from { p.address, fromPort };
                if (o.to.address == kMulticastGroup)
                {
                    for (int j = 0; j < size(); ++j)
                        if (linked(i, j))
                            schedule(j, SocketKind::group, from, o.datagram);
                    continue;
                }
                for (int j = 0; j < size(); ++j)
                {
                    auto& q = peer(j);
                    if (q.address != o.to.address || ! linked(i, j))
                        continue;
                    if (o.to.port == q.discoveryPort)
                        schedule(j, SocketKind::discovery, from, o.datagram);
                    else if (o.to.port == q.measurementPort)
                        schedule(j, SocketKind::measurement, from, o.datagram);
                }
            }
            box.clear();
        }

        void runFor(Micros duration) { runUntil(now + duration); }

        void runUntil(Micros end)
        {
            for (;;)
            {
                const Micros nextTick = lastTick + tickStep;
                auto first = std::min_element(flights.begin(), flights.end(),
                                              [](const Flight& a, const Flight& b) { return a.at < b.at || (a.at == b.at && a.order < b.order); });
                const bool flightFirst = first != flights.end() && first->at <= nextTick;
                const Micros next = flightFirst ? first->at : nextTick;
                if (next > end)
                {
                    now = end;
                    return;
                }
                now = next;
                if (flightFirst)
                {
                    // Datagrams bouncing at one instant without end (a zero delay can do it) stop the run.
                    if (next != stuckAt)
                        stuckAt = next, stuckCount = 0;
                    else if (++stuckCount > 100000)
                        throw std::runtime_error("the simulation is stuck at one instant");
                    const Flight f = *first;
                    flights.erase(first);
                    auto& q = peer(f.to);
                    if (q.alive)
                    {
                        q.core->receive(0, f.socket, f.from, f.datagram.view(), host(f.to));
                        flush(f.to);
                    }
                    continue;
                }
                lastTick = now;
                for (int i = 0; i < size(); ++i)
                    if (peer(i).alive)
                    {
                        peer(i).core->tick(host(i));
                        flush(i);
                    }
            }
        }

        /** Runs until the condition holds or the time is up; true if it held. */
        bool runUntilTrue(Micros limit, const std::function<bool()>& condition)
        {
            const Micros end = now + limit;
            while (now < end)
            {
                if (condition())
                    return true;
                runFor(std::min<Micros>(1000, end - now));
            }
            return condition();
        }

        /** The true clock offset of peer i's session as seen from peer j's host clock. */
        Micros trueOffset(int i, int j) { return host(i) + core(i).clockOffset() - host(j); }

    private:
        struct Flight
        {
            Micros at = 0;
            std::uint64_t order = 0;
            int to = 0;
            SocketKind socket = SocketKind::group;
            Endpoint4 from {};
            Datagram datagram {};
        };

        bool linked(int i, int j) { return peer(j).alive && (bridged || peer(i).segment == peer(j).segment); }

        void schedule(int to, SocketKind socket, Endpoint4 from, const Datagram& d)
        {
            Micros delay = minDelay;
            if (maxDelay > minDelay)
                delay = std::uniform_int_distribution<Micros>(minDelay, maxDelay)(rng);
            flights.push_back({ now + delay, ++order, to, socket, from, d });
        }

        std::vector<std::unique_ptr<Peer>> peers;
        std::vector<Flight> flights;
        std::mt19937 rng;
        std::uint64_t order = 0;
        Micros lastTick = 0;
        Micros stuckAt = -1;
        int stuckCount = 0;
    };
}
