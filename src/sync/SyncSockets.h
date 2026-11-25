#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "sync/SyncWire.h"

/*
    The beat sync's sockets (CoolSynth's D361, docs/research/beat-sync-protocol.md, "Gateways and sockets"): the three
    UDP sockets of a gateway on Winsock and on BSD sockets, the computer's interface addresses, and one poll over all
    sockets. Plain sockets rather than JUCE's DatagramSocket, which binds its multicast membership to the socket's own
    address, has no outbound interface and cannot switch off Windows' port-unreachable reports. No system header
    leaks out of this file.
*/
namespace coolsynth::sync
{
    /** The IPv4 addresses of the interfaces that are up (and take multicast), 127.0.0.1 always among them. */
    [[nodiscard]] std::vector<std::uint32_t> interfaceAddresses();

    /** Winsock's start and end on Windows; nothing elsewhere. */
    class NetworkSystem
    {
    public:
        NetworkSystem();
        ~NetworkSystem();
        NetworkSystem(const NetworkSystem&) = delete;
        NetworkSystem& operator=(const NetworkSystem&) = delete;
        [[nodiscard]] bool ok() const noexcept { return started; }

    private:
        bool started = false;
    };

    enum class ReceiveResult { datagram, empty, error, closed };
    enum class SendResult { sent, dropped, failed };

    class UdpSocket
    {
    public:
        UdpSocket() = default;
        ~UdpSocket() { close(); }
        UdpSocket(const UdpSocket&) = delete;
        UdpSocket& operator=(const UdpSocket&) = delete;

        /** The group socket: address reuse, bound to 0.0.0.0:20808, the group joined on the interface. */
        bool openGroup(std::uint32_t interfaceAddress, bool multicastLoop);
        /** A discovery or measurement socket: address reuse, bound to the interface with a port the system picks,
            multicast sent out of that interface. */
        bool openUnicast(std::uint32_t interfaceAddress, bool multicastLoop);
        void close() noexcept;

        [[nodiscard]] bool isOpen() const noexcept { return handle != invalidHandle; }
        [[nodiscard]] std::uint16_t port() const noexcept { return boundPort; }
        [[nodiscard]] std::intptr_t nativeHandle() const noexcept { return handle; }

        SendResult sendTo(Endpoint4 to, std::span<const std::uint8_t> bytes) noexcept;
        /** One datagram, if one is waiting; one of 512 bytes or more is dropped (an error). */
        ReceiveResult receive(Datagram& into, Endpoint4& from) noexcept;

        static constexpr std::intptr_t invalidHandle = -1;

    private:
        bool create(bool multicastLoop);
        std::intptr_t handle = invalidHandle;
        std::uint16_t boundPort = 0;
    };

    /** A gateway's three sockets on one interface address. */
    struct GatewaySockets
    {
        std::uint32_t address = 0;
        UdpSocket group, discovery, measurement;

        /** Multicast loopback on for the loopback gateway, off for the others. */
        bool open(std::uint32_t interfaceAddress);
        void close() noexcept;
    };

    /** Waits until one of the sockets has a datagram or the time is up; marks the readable ones. False on an error. */
    bool waitForReadable(std::span<const std::intptr_t> handles, std::vector<char>& readable, int timeoutMilliseconds);

    /** The network thread asks for 1 ms timer resolution while it runs (Windows' default is 15.6 ms). */
    class FineTimerResolution
    {
    public:
        FineTimerResolution();
        ~FineTimerResolution();
        FineTimerResolution(const FineTimerResolution&) = delete;
        FineTimerResolution& operator=(const FineTimerResolution&) = delete;

    private:
        bool raised = false;
    };

    /** The calling thread a step above normal priority, so pings are answered promptly under load. */
    void raiseCurrentThreadPriority() noexcept;

    /** An address as dotted text. */
    [[nodiscard]] std::string addressText(std::uint32_t address);
}
