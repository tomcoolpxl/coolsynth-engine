#include "sync/SyncSockets.h"

#include <algorithm>

#if defined(_WIN32)
 #ifndef NOMINMAX
  #define NOMINMAX
 #endif
 #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
 #endif
 #include <winsock2.h>
 #include <ws2tcpip.h>
 #include <mstcpip.h>
 #include <iphlpapi.h>
 #include <windows.h>
 #include <timeapi.h>
 #ifndef SIO_UDP_CONNRESET
  #define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
 #endif
#else
 #include <arpa/inet.h>
 #include <cerrno>
 #include <fcntl.h>
 #include <ifaddrs.h>
 #include <net/if.h>
 #include <netinet/in.h>
 #include <poll.h>
 #include <pthread.h>
 #include <sys/socket.h>
 #include <unistd.h>
#endif

namespace coolsynth::sync
{
    namespace
    {
       #if defined(_WIN32)
        using Native = SOCKET;
        Native toNative(std::intptr_t h) noexcept { return static_cast<Native>(static_cast<std::uintptr_t>(h)); }
        std::intptr_t fromNative(Native s) noexcept { return s == INVALID_SOCKET ? UdpSocket::invalidHandle : static_cast<std::intptr_t>(s); }
        void closeNative(Native s) noexcept { ::closesocket(s); }
        int lastError() noexcept { return ::WSAGetLastError(); }
       #else
        using Native = int;
        Native toNative(std::intptr_t h) noexcept { return static_cast<Native>(h); }
        std::intptr_t fromNative(Native s) noexcept { return s < 0 ? UdpSocket::invalidHandle : static_cast<std::intptr_t>(s); }
        void closeNative(Native s) noexcept { ::close(s); }
        int lastError() noexcept { return errno; }
       #endif

        sockaddr_in socketAddress(std::uint32_t address, std::uint16_t port) noexcept
        {
            sockaddr_in a {};
            a.sin_family = AF_INET;
            a.sin_addr.s_addr = htonl(address);
            a.sin_port = htons(port);
            return a;
        }

        bool setFlag(Native s, int level, int name, int value) noexcept
        {
           #if defined(_WIN32)
            const BOOL v = value != 0 ? TRUE : FALSE;
            return ::setsockopt(s, level, name, reinterpret_cast<const char*>(&v), sizeof(v)) == 0;
           #else
            return ::setsockopt(s, level, name, &value, sizeof(value)) == 0;
           #endif
        }

        bool setMulticastLoop(Native s, bool on) noexcept
        {
           #if defined(_WIN32)
            const DWORD v = on ? 1 : 0;
            return ::setsockopt(s, IPPROTO_IP, IP_MULTICAST_LOOP, reinterpret_cast<const char*>(&v), sizeof(v)) == 0;
           #else
            const unsigned char v = on ? 1 : 0;
            return ::setsockopt(s, IPPROTO_IP, IP_MULTICAST_LOOP, &v, sizeof(v)) == 0;
           #endif
        }

        bool setNonBlocking(Native s) noexcept
        {
           #if defined(_WIN32)
            u_long on = 1;
            return ::ioctlsocket(s, FIONBIO, &on) == 0;
           #else
            const int flags = ::fcntl(s, F_GETFL, 0);
            return flags >= 0 && ::fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
           #endif
        }

        void addUnique(std::vector<std::uint32_t>& list, std::uint32_t address)
        {
            if (std::find(list.begin(), list.end(), address) == list.end())
                list.push_back(address);
        }
    }

    //==================================================================================================================
    std::vector<std::uint32_t> interfaceAddresses()
    {
        std::vector<std::uint32_t> list { kLoopbackAddress };
       #if defined(_WIN32)
        ULONG size = 16 * 1024;
        std::vector<unsigned char> buffer;
        ULONG result = ERROR_BUFFER_OVERFLOW;
        for (int attempt = 0; attempt < 4 && result == ERROR_BUFFER_OVERFLOW; ++attempt)
        {
            buffer.resize(size);
            result = ::GetAdaptersAddresses(AF_INET,
                                            GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER
                                                | GAA_FLAG_SKIP_FRIENDLY_NAME,
                                            nullptr, reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.data()), &size);
        }
        if (result != NO_ERROR)
            return list;
        for (auto* adapter = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.data()); adapter != nullptr; adapter = adapter->Next)
        {
            if (adapter->OperStatus != IfOperStatusUp || (adapter->Flags & IP_ADAPTER_NO_MULTICAST) != 0)
                continue;
            for (auto* unicast = adapter->FirstUnicastAddress; unicast != nullptr; unicast = unicast->Next)
            {
                const auto* sa = unicast->Address.lpSockaddr;
                if (sa == nullptr || sa->sa_family != AF_INET)
                    continue;
                addUnique(list, ntohl(reinterpret_cast<const sockaddr_in*>(sa)->sin_addr.s_addr));
            }
        }
       #else
        ifaddrs* all = nullptr;
        if (::getifaddrs(&all) != 0)
            return list;
        for (auto* i = all; i != nullptr; i = i->ifa_next)
        {
            if (i->ifa_addr == nullptr || i->ifa_addr->sa_family != AF_INET)
                continue;
            const auto flags = i->ifa_flags;
            if ((flags & IFF_UP) == 0 || (flags & IFF_RUNNING) == 0 || ((flags & IFF_MULTICAST) == 0 && (flags & IFF_LOOPBACK) == 0))
                continue;
            addUnique(list, ntohl(reinterpret_cast<const sockaddr_in*>(i->ifa_addr)->sin_addr.s_addr));
        }
        ::freeifaddrs(all);
       #endif
        return list;
    }

    std::string addressText(std::uint32_t address)
    {
        return std::to_string((address >> 24) & 0xFF) + "." + std::to_string((address >> 16) & 0xFF) + "."
             + std::to_string((address >> 8) & 0xFF) + "." + std::to_string(address & 0xFF);
    }

    //==================================================================================================================
    NetworkSystem::NetworkSystem()
    {
       #if defined(_WIN32)
        WSADATA data {};
        started = ::WSAStartup(MAKEWORD(2, 2), &data) == 0;
       #else
        started = true;
       #endif
    }

    NetworkSystem::~NetworkSystem()
    {
       #if defined(_WIN32)
        if (started)
            ::WSACleanup();
       #endif
    }

    //==================================================================================================================
    bool UdpSocket::create(bool multicastLoop)
    {
        close();
        const Native s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        handle = fromNative(s);
        if (! isOpen())
            return false;
        bool ok = setFlag(s, SOL_SOCKET, SO_REUSEADDR, 1) && setNonBlocking(s) && setMulticastLoop(s, multicastLoop);
       #if defined(_WIN32)
        // Without this, one "port unreachable" from a peer that has just quit can stop the socket hearing anything.
        BOOL reportResets = FALSE;
        DWORD returned = 0;
        ok = ok && ::WSAIoctl(s, SIO_UDP_CONNRESET, &reportResets, sizeof(reportResets), nullptr, 0, &returned, nullptr, nullptr) == 0;
       #endif
        if (! ok)
            close();
        return ok;
    }

    bool UdpSocket::openGroup(std::uint32_t interfaceAddress, bool multicastLoop)
    {
        if (! create(multicastLoop))
            return false;
        const Native s = toNative(handle);
        bool ok = true;
       #if defined(__APPLE__)
        ok = setFlag(s, SOL_SOCKET, SO_REUSEPORT, 1);
       #endif
       #if defined(IP_MULTICAST_ALL)
        ok = ok && setFlag(s, IPPROTO_IP, IP_MULTICAST_ALL, 0); // only the group joined on this interface
       #endif
        const sockaddr_in any = socketAddress(0, kDiscoveryPort);
        ok = ok && ::bind(s, reinterpret_cast<const sockaddr*>(&any), sizeof(any)) == 0;
        ip_mreq membership {};
        membership.imr_multiaddr.s_addr = htonl(kMulticastGroup);
        membership.imr_interface.s_addr = htonl(interfaceAddress);
        ok = ok && ::setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<const char*>(&membership), sizeof(membership)) == 0;
        if (! ok)
        {
            close();
            return false;
        }
        boundPort = kDiscoveryPort;
        return true;
    }

    bool UdpSocket::openUnicast(std::uint32_t interfaceAddress, bool multicastLoop)
    {
        if (! create(multicastLoop))
            return false;
        const Native s = toNative(handle);
        const sockaddr_in local = socketAddress(interfaceAddress, 0);
        in_addr out {};
        out.s_addr = htonl(interfaceAddress);
        bool ok = ::bind(s, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) == 0
               && ::setsockopt(s, IPPROTO_IP, IP_MULTICAST_IF, reinterpret_cast<const char*>(&out), sizeof(out)) == 0;
        sockaddr_in bound {};
       #if defined(_WIN32)
        int length = sizeof(bound);
       #else
        socklen_t length = sizeof(bound);
       #endif
        ok = ok && ::getsockname(s, reinterpret_cast<sockaddr*>(&bound), &length) == 0;
        if (! ok)
        {
            close();
            return false;
        }
        boundPort = ntohs(bound.sin_port);
        return true;
    }

    void UdpSocket::close() noexcept
    {
        if (isOpen())
            closeNative(toNative(handle));
        handle = invalidHandle;
        boundPort = 0;
    }

    SendResult UdpSocket::sendTo(Endpoint4 to, std::span<const std::uint8_t> bytes) noexcept
    {
        if (! isOpen())
            return SendResult::failed;
        const sockaddr_in a = socketAddress(to.address, to.port);
       #if defined(_WIN32)
        const int sent = ::sendto(toNative(handle), reinterpret_cast<const char*>(bytes.data()), static_cast<int>(bytes.size()), 0,
                                  reinterpret_cast<const sockaddr*>(&a), sizeof(a));
        if (sent == SOCKET_ERROR)
            return lastError() == WSAEWOULDBLOCK ? SendResult::dropped : SendResult::failed;
       #else
        const auto sent = ::sendto(toNative(handle), bytes.data(), bytes.size(), 0, reinterpret_cast<const sockaddr*>(&a), sizeof(a));
        if (sent < 0)
        {
            const int e = lastError();
            return (e == EAGAIN || e == EWOULDBLOCK || e == ENOBUFS) ? SendResult::dropped : SendResult::failed;
        }
       #endif
        return SendResult::sent;
    }

    ReceiveResult UdpSocket::receive(Datagram& into, Endpoint4& from) noexcept
    {
        if (! isOpen())
            return ReceiveResult::closed;
        sockaddr_in source {};
       #if defined(_WIN32)
        int length = sizeof(source);
        const int got = ::recvfrom(toNative(handle), reinterpret_cast<char*>(into.bytes.data()), static_cast<int>(into.bytes.size()), 0,
                                   reinterpret_cast<sockaddr*>(&source), &length);
        if (got == SOCKET_ERROR)
        {
            switch (lastError())
            {
                case WSAEWOULDBLOCK: return ReceiveResult::empty;
                case WSAENOTSOCK:
                case WSAESHUTDOWN:
                case WSAEINVAL:
                case WSANOTINITIALISED: return ReceiveResult::closed;
                default: return ReceiveResult::error; // a datagram too large, a reset: the next receive follows
            }
        }
       #else
        socklen_t length = sizeof(source);
        const auto got = ::recvfrom(toNative(handle), into.bytes.data(), into.bytes.size(), 0, reinterpret_cast<sockaddr*>(&source), &length);
        if (got < 0)
        {
            const int e = lastError();
            if (e == EAGAIN || e == EWOULDBLOCK)
                return ReceiveResult::empty;
            if (e == EBADF || e == ENOTSOCK)
                return ReceiveResult::closed;
            return ReceiveResult::error;
        }
       #endif
        if (static_cast<std::size_t>(got) >= kReceiveBufferSize)
            return ReceiveResult::error; // did not fit: dropped
        into.size = static_cast<std::size_t>(got);
        from = { ntohl(source.sin_addr.s_addr), ntohs(source.sin_port) };
        return ReceiveResult::datagram;
    }

    //==================================================================================================================
    bool GatewaySockets::open(std::uint32_t interfaceAddress)
    {
        address = interfaceAddress;
        const bool loop = isLoopback(interfaceAddress);
        if (group.openGroup(interfaceAddress, loop) && discovery.openUnicast(interfaceAddress, loop)
            && measurement.openUnicast(interfaceAddress, loop))
            return true;
        close();
        return false;
    }

    void GatewaySockets::close() noexcept
    {
        group.close();
        discovery.close();
        measurement.close();
    }

    bool waitForReadable(std::span<const std::intptr_t> handles, std::vector<char>& readable, int timeoutMilliseconds)
    {
        readable.assign(handles.size(), 0);
       #if defined(_WIN32)
        thread_local std::vector<WSAPOLLFD> fds;
        fds.resize(handles.size());
        for (std::size_t i = 0; i < handles.size(); ++i)
            fds[i] = { toNative(handles[i]), POLLRDNORM, 0 };
        if (fds.empty())
        {
            ::Sleep(static_cast<DWORD>(std::max(0, timeoutMilliseconds)));
            return true;
        }
        const int n = ::WSAPoll(fds.data(), static_cast<ULONG>(fds.size()), timeoutMilliseconds);
       #else
        thread_local std::vector<pollfd> fds;
        fds.resize(handles.size());
        for (std::size_t i = 0; i < handles.size(); ++i)
            fds[i] = { toNative(handles[i]), POLLIN, 0 };
        const int n = ::poll(fds.data(), static_cast<nfds_t>(fds.size()), timeoutMilliseconds);
       #endif
        if (n < 0)
            return false;
        for (std::size_t i = 0; i < handles.size(); ++i)
            readable[i] = (fds[i].revents & (POLLIN | POLLRDNORM | POLLERR | POLLHUP)) != 0 ? 1 : 0;
        return true;
    }

    FineTimerResolution::FineTimerResolution()
    {
       #if defined(_WIN32)
        raised = ::timeBeginPeriod(1) == TIMERR_NOERROR;
       #endif
    }

    FineTimerResolution::~FineTimerResolution()
    {
       #if defined(_WIN32)
        if (raised)
            ::timeEndPeriod(1);
       #endif
    }

    void raiseCurrentThreadPriority() noexcept
    {
       #if defined(_WIN32)
        ::SetThreadPriority(::GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
       #endif
    }
}
