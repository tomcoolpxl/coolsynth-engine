#include "sync/SyncNode.h"

#include <algorithm>
#include <memory>
#include <random>
#include <vector>

#include "sync/SyncClock.h"
#include "sync/SyncSockets.h"

namespace coolsynth::sync
{
    //==================================================================================================================
    void HandoffPump::found(SyncCore& core, Handoff& handoff, Micros hostNow)
    {
        const Snapshot carry = handoff.readSnapshot();
        snapshotNumber = carry.number;

        // The proposals first, then the grid: the audio thread writes its grid before it posts a proposal, so the
        // grid read after a proposal already holds it.
        SeqLock<kGridWords>::Words g {};
        SeqLock<kPlayWords>::Words p {};
        SeqLock<kLocalWords>::Words l {};
        while (! handoff.grid.read(g)) {}
        while (! handoff.play.read(p)) {}
        while (! handoff.local.read(l)) {}
        const LocalGrid local = decodeLocal(l);
        grid = std::max(decodeGrid(g).number, carry.gridApplied);
        play = std::max(decodePlay(p).number, carry.playApplied);

        const MicroBeats beatNow = local.appBeat + beatsInDuration(hostNow - local.hostTime, local.tempo);
        core.setStartStopSharing(handoff.startStopSharing.load(std::memory_order_acquire));
        core.found(hostNow, local.tempo, beatNow, carry);
        core.acknowledge(grid, play);
    }

    void HandoffPump::takeProposals(SyncCore& core, Handoff& handoff, Micros hostNow)
    {
        core.setStartStopSharing(handoff.startStopSharing.load(std::memory_order_acquire));
        SeqLock<kGridWords>::Words g {};
        if (handoff.grid.read(g))
            if (const auto proposal = decodeGrid(g); proposal.number > grid)
            {
                grid = proposal.number;
                core.applyGrid(proposal, hostNow);
            }
        SeqLock<kPlayWords>::Words p {};
        if (handoff.play.read(p))
            if (const auto proposal = decodePlay(p); proposal.number > play)
            {
                play = proposal.number;
                core.applyPlay(proposal, hostNow);
            }
    }

    void HandoffPump::publish(SyncCore& core, Handoff& handoff, Micros hostNow, bool force)
    {
        if (! core.takeSnapshotChanged() && ! force)
            return;
        Snapshot s = core.snapshot(hostNow);
        s.number = ++snapshotNumber;
        handoff.snapshot.write(encode(s));
    }

    //==================================================================================================================
    SyncNode::SyncNode(Handoff& h, Options o) : handoff(h), options(std::move(o)) {}

    SyncNode::~SyncNode() { stop(); }

    void SyncNode::start()
    {
        if (thread.joinable())
            return;
        quit.store(false);
        firstScanDone.store(false);
        thread = std::thread([this] { run(); });
    }

    void SyncNode::stop()
    {
        if (! thread.joinable())
            return;
        quit.store(true);
        thread.join();
    }

    namespace
    {
        struct OpenGateway
        {
            int id = 0;
            std::unique_ptr<GatewaySockets> sockets;
        };
    }

    void SyncNode::run()
    {
        raiseCurrentThreadPriority();
        const FineTimerResolution fineTimers;
        const NetworkSystem network;

        std::random_device entropy;
        SyncCore core((static_cast<std::uint64_t>(entropy()) << 32) ^ static_cast<std::uint64_t>(entropy())
                      ^ static_cast<std::uint64_t>(hostMicrosNow()));
        core.setLogger(options.logger);
        HandoffPump pump;
        pump.found(core, handoff, hostMicrosNow());

        std::vector<OpenGateway> gateways;
        int nextId = 1;
        bool rescanSoon = true;
        Micros nextScan = 0, nextSecond = hostMicrosNow() + 1000000;

        auto findGateway = [&](int id) -> GatewaySockets*
        {
            for (auto& g : gateways)
                if (g.id == id)
                    return g.sockets.get();
            return nullptr;
        };

        auto socketOf = [](GatewaySockets& g, SocketKind kind) -> UdpSocket&
        {
            switch (kind)
            {
                case SocketKind::group: return g.group;
                case SocketKind::measurement: return g.measurement;
                case SocketKind::discovery:
                default: return g.discovery;
            }
        };

        auto closeGateway = [&](int id, bool sayLeaving, Micros now)
        {
            core.removeGateway(id, sayLeaving, now);
            // The leaving message (if any) goes out before its sockets close.
            auto& box = core.outbox();
            if (auto* g = findGateway(id))
                for (const auto& o : box)
                    if (o.gateway == id)
                        socketOf(*g, o.from).sendTo(o.to, o.datagram.view());
            box.erase(std::remove_if(box.begin(), box.end(), [id](const Outgoing& o) { return o.gateway == id; }), box.end());
            gateways.erase(std::remove_if(gateways.begin(), gateways.end(), [id](const OpenGateway& g) { return g.id == id; }),
                           gateways.end());
        };

        // Sends what the core wants sent; a failed send closes that gateway and asks for a rescan at once.
        auto flush = [&](Micros now)
        {
            for (int round = 0; round < 4 && ! core.outbox().empty(); ++round)
            {
                std::vector<Outgoing> box;
                box.swap(core.outbox());
                std::vector<int> failed;
                for (const auto& o : box)
                {
                    auto* g = findGateway(o.gateway);
                    if (g == nullptr || std::find(failed.begin(), failed.end(), o.gateway) != failed.end())
                        continue;
                    const auto result = socketOf(*g, o.from).sendTo(o.to, o.datagram.view());
                    if (options.observer)
                        options.observer(true, g->address, o.from, o.to, o.datagram.view());
                    if (result == SendResult::failed)
                        failed.push_back(o.gateway);
                }
                for (const int id : failed)
                {
                    if (options.logger)
                        options.logger("a send failed: gateway closed");
                    closeGateway(id, false, now);
                    rescanSoon = true;
                }
            }
            gatewaysOpen.store(static_cast<int>(gateways.size()), std::memory_order_relaxed);
        };

        auto scan = [&](Micros now)
        {
            const auto addresses = options.loopbackOnly ? std::vector<std::uint32_t> { kLoopbackAddress } : interfaceAddresses();
            std::vector<int> gone;
            for (const auto& g : gateways)
                if (std::find(addresses.begin(), addresses.end(), g.sockets->address) == addresses.end())
                    gone.push_back(g.id);
            for (const int id : gone)
                closeGateway(id, true, now);
            for (const auto address : addresses)
            {
                const bool open = std::any_of(gateways.begin(), gateways.end(),
                                              [address](const OpenGateway& g) { return g.sockets->address == address; });
                if (open)
                    continue;
                auto sockets = std::make_unique<GatewaySockets>();
                if (! sockets->open(address))
                {
                    if (options.logger)
                        options.logger("no gateway on " + addressText(address));
                    continue;
                }
                const int id = nextId++;
                const Endpoint4 measurement { address, sockets->measurement.port() };
                gateways.push_back({ id, std::move(sockets) });
                if (options.logger)
                    options.logger("gateway on " + addressText(address) + ", measurement port " + std::to_string(measurement.port));
                core.addGateway(id, measurement, now);
            }
            flush(now);
            nextScan = now + 5000000;
            rescanSoon = false;
            firstScanDone.store(true, std::memory_order_release);
        };

        std::vector<std::intptr_t> handles;
        std::vector<char> readable;
        std::vector<std::pair<int, SocketKind>> owners;
        auto datagram = std::make_unique<Datagram>();
        pump.publish(core, handoff, hostMicrosNow(), true);

        while (! quit.load(std::memory_order_acquire))
        {
            Micros now = hostMicrosNow();
            if (rescanSoon || now >= nextScan)
                scan(now);
            pump.takeProposals(core, handoff, now);
            core.tick(now);
            flush(now);
            pump.publish(core, handoff, now);
            if (options.everySecond && now >= nextSecond)
            {
                options.everySecond(core, now);
                nextSecond = now + 1000000;
            }

            const Micros wait = std::clamp<Micros>(core.nextWake(now) - now, 0, 5000);
            handles.clear();
            owners.clear();
            for (const auto& g : gateways)
                for (const auto kind : { SocketKind::group, SocketKind::discovery, SocketKind::measurement })
                {
                    handles.push_back(socketOf(*g.sockets, kind).nativeHandle());
                    owners.push_back({ g.id, kind });
                }
            waitForReadable(handles, readable, static_cast<int>((wait + 999) / 1000));

            for (std::size_t i = 0; i < handles.size(); ++i)
            {
                if (readable[i] == 0)
                    continue;
                auto* g = findGateway(owners[i].first);
                if (g == nullptr)
                    continue;
                auto& socket = socketOf(*g, owners[i].second);
                for (int n = 0; n < 64; ++n)
                {
                    Endpoint4 from {};
                    const auto result = socket.receive(*datagram, from);
                    if (result == ReceiveResult::empty || result == ReceiveResult::closed)
                        break;
                    if (result == ReceiveResult::error)
                        continue; // the next receive follows
                    const Micros at = hostMicrosNow();
                    if (options.observer)
                        options.observer(false, g->address, owners[i].second, from, datagram->view());
                    core.receive(owners[i].first, owners[i].second, from, datagram->view(), at);
                    flush(at); // a pong goes out at once
                    g = findGateway(owners[i].first);
                    if (g == nullptr)
                        break;
                }
                if (findGateway(owners[i].first) == nullptr)
                    continue;
            }
            pump.publish(core, handoff, hostMicrosNow());
        }

        const Micros end = hostMicrosNow();
        core.leaveAll(end);
        auto& box = core.outbox();
        for (const auto& o : box)
            if (auto* g = findGateway(o.gateway))
            {
                socketOf(*g, o.from).sendTo(o.to, o.datagram.view());
                if (options.observer)
                    options.observer(true, g->address, o.from, o.to, o.datagram.view());
            }
        box.clear();
        gateways.clear();
        gatewaysOpen.store(0, std::memory_order_relaxed);
        pump.publish(core, handoff, end, true);
    }
}
