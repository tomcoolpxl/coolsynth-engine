#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <span>
#include <thread>

#include "sync/SyncCore.h"
#include "sync/SyncHandoff.h"

/*
    The beat sync's network thread (CoolSynth's D361, docs/research/beat-sync-protocol.md, "Threads"): it owns the
    sockets and the core and moves bytes and times between them. It waits in one poll over every socket, at most
    5 ms, so it sees the audio thread's proposals within 5 ms without the audio thread making a system call. It may
    allocate; it never touches audio.
*/
namespace coolsynth::sync
{
    /** The network side of the handoff, shared by the network thread and the tests' simulated peers. */
    struct HandoffPump
    {
        std::int64_t grid = 0, play = 0, snapshotNumber = 0;

        /** Switching on: founds a session from the audio thread's own grid (the proposals already in it counted as
            applied) and carries the snapshot counters on. */
        void found(SyncCore& core, Handoff& handoff, Micros hostNow);
        /** The message thread's sharing switch and the audio thread's newest proposals into the core. */
        void takeProposals(SyncCore& core, Handoff& handoff, Micros hostNow);
        /** A new snapshot if anything it shows changed (or always, when forced). */
        void publish(SyncCore& core, Handoff& handoff, Micros hostNow, bool force = false);
    };

    class SyncNode
    {
    public:
        /** Every datagram in or out, for the console peer's log. */
        using DatagramObserver = std::function<void(bool outgoing, std::uint32_t gatewayAddress, SocketKind socket,
                                                    Endpoint4 peer, std::span<const std::uint8_t> bytes)>;

        struct Options
        {
            /** Only the loopback gateway: other programs on this computer, none elsewhere (tests). */
            bool loopbackOnly = false;
            SyncCore::Logger logger;
            DatagramObserver observer;
            /** Called on the network thread about once a second, with the core, for the console peer. */
            std::function<void(const SyncCore&, Micros hostNow)> everySecond;
        };

        SyncNode(Handoff& handoff, Options options);
        ~SyncNode();
        SyncNode(const SyncNode&) = delete;
        SyncNode& operator=(const SyncNode&) = delete;

        /** Starts the network thread; it founds a session from the audio thread's grid. */
        void start();
        /** Leaving messages on every gateway, a last snapshot, then the thread ends; joins it. */
        void stop();

        [[nodiscard]] bool isRunning() const noexcept { return thread.joinable(); }
        /** Gateways open now; zero if no socket could be opened (no network stack, the port refused). */
        [[nodiscard]] int openGateways() const noexcept { return gatewaysOpen.load(std::memory_order_relaxed); }
        /** True once the thread has opened, or failed to open, its first gateways. */
        [[nodiscard]] bool settled() const noexcept { return firstScanDone.load(std::memory_order_acquire); }

    private:
        void run();

        Handoff& handoff;
        Options options;
        std::thread thread;
        std::atomic<bool> quit { false };
        std::atomic<int> gatewaysOpen { 0 };
        std::atomic<bool> firstScanDone { false };
    };
}
