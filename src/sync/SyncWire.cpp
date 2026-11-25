#include "sync/SyncWire.h"

#include <algorithm>
#include <cstring>

namespace coolsynth::sync
{
    void Writer::raw(const std::uint8_t* data, std::size_t n) noexcept
    {
        if (failed || datagram.size + n > kMaxDatagramSize)
        {
            failed = true;
            return;
        }
        std::memcpy(datagram.bytes.data() + datagram.size, data, n);
        datagram.size += n;
    }

    void Writer::u16(std::uint16_t v) noexcept
    {
        const std::uint8_t b[2] { static_cast<std::uint8_t>(v >> 8), static_cast<std::uint8_t>(v) };
        raw(b, 2);
    }

    void Writer::u32(std::uint32_t v) noexcept
    {
        const std::uint8_t b[4] { static_cast<std::uint8_t>(v >> 24), static_cast<std::uint8_t>(v >> 16),
                                  static_cast<std::uint8_t>(v >> 8), static_cast<std::uint8_t>(v) };
        raw(b, 4);
    }

    void Writer::i64(std::int64_t v) noexcept
    {
        const auto u = static_cast<std::uint64_t>(v);
        std::uint8_t b[8];
        for (int i = 0; i < 8; ++i)
            b[i] = static_cast<std::uint8_t>(u >> (56 - 8 * i));
        raw(b, 8);
    }

    std::int64_t readI64(const std::uint8_t* p) noexcept
    {
        std::uint64_t u = 0;
        for (int i = 0; i < 8; ++i)
            u = (u << 8) | p[i];
        return static_cast<std::int64_t>(u);
    }

    namespace
    {
        std::uint32_t readU32(const std::uint8_t* p) noexcept
        {
            return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16)
                 | (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
        }

        void writeDiscoveryHeader(Writer& w, DiscoveryType type, std::uint8_t ttl, const PeerId& sender) noexcept
        {
            w.raw(kDiscoveryMagic.data(), kDiscoveryMagic.size());
            w.u8(static_cast<std::uint8_t>(type));
            w.u8(ttl);
            w.u16(0); // the group
            w.raw(sender.data(), sender.size());
        }

        /** Walks a payload's entries, calling visit(key, value pointer, size) for each; false if an entry runs past
            the end or visit refuses a known entry. */
        template <typename Visit>
        bool forEachEntry(std::span<const std::uint8_t> payload, Visit&& visit) noexcept
        {
            std::size_t at = 0;
            while (at < payload.size())
            {
                if (payload.size() - at < 8)
                    return false;
                const std::uint32_t key = readU32(payload.data() + at);
                const std::uint32_t size = readU32(payload.data() + at + 4);
                at += 8;
                if (size > payload.size() - at)
                    return false;
                if (! visit(key, payload.data() + at, static_cast<std::size_t>(size)))
                    return false;
                at += size;
            }
            return true;
        }
    }

    bool buildDiscovery(Datagram& out, DiscoveryType type, const PeerId& sender, const PeerState& state) noexcept
    {
        if (type == DiscoveryType::leaving)
            return buildLeaving(out, sender);
        Writer w(out);
        writeDiscoveryHeader(w, type, kAnnounceTimeToLive, sender);
        w.entryHeader(keys::timeline, 24);
        w.i64(state.timeline.tempo);
        w.i64(state.timeline.beatOrigin);
        w.i64(state.timeline.timeOrigin);
        w.entryHeader(keys::session, 8);
        w.raw(state.sessionId.data(), state.sessionId.size());
        w.entryHeader(keys::playState, 17);
        w.u8(state.playState.playing ? 1 : 0);
        w.i64(state.playState.beat);
        w.i64(state.playState.time);
        if (state.measurement)
        {
            w.entryHeader(keys::endpoint4, 6);
            w.u32(state.measurement->address);
            w.u16(state.measurement->port);
        }
        return w.ok();
    }

    bool buildLeaving(Datagram& out, const PeerId& sender) noexcept
    {
        Writer w(out);
        writeDiscoveryHeader(w, DiscoveryType::leaving, 0, sender);
        return w.ok();
    }

    bool buildPing(Datagram& out, Micros hostTime, std::optional<Micros> previousSessionTime) noexcept
    {
        Writer w(out);
        w.raw(kMeasurementMagic.data(), kMeasurementMagic.size());
        w.u8(static_cast<std::uint8_t>(MeasurementType::ping));
        w.entryHeader(keys::hostTime, 8);
        w.i64(hostTime);
        if (previousSessionTime)
        {
            w.entryHeader(keys::prevSessionTime, 8);
            w.i64(*previousSessionTime);
        }
        return w.ok();
    }

    bool buildPong(Datagram& out, const PeerId& sessionId, Micros sessionTime,
                   std::span<const std::uint8_t> pingPayload) noexcept
    {
        Writer w(out);
        w.raw(kMeasurementMagic.data(), kMeasurementMagic.size());
        w.u8(static_cast<std::uint8_t>(MeasurementType::pong));
        w.entryHeader(keys::session, 8);
        w.raw(sessionId.data(), sessionId.size());
        w.entryHeader(keys::sessionTime, 8);
        w.i64(sessionTime);
        w.raw(pingPayload.data(), pingPayload.size());
        return w.ok();
    }

    DatagramKind kindOf(std::span<const std::uint8_t> bytes) noexcept
    {
        if (bytes.size() >= kDiscoveryMagic.size())
        {
            if (std::equal(kDiscoveryMagic.begin(), kDiscoveryMagic.end(), bytes.begin()))
                return DatagramKind::discovery;
            if (std::equal(kMeasurementMagic.begin(), kMeasurementMagic.end(), bytes.begin()))
                return DatagramKind::measurement;
        }
        return DatagramKind::unknown;
    }

    std::optional<DiscoveryMessage> readDiscovery(std::span<const std::uint8_t> bytes) noexcept
    {
        if (bytes.size() < kDiscoveryHeaderSize || bytes.size() >= kReceiveBufferSize
            || kindOf(bytes) != DatagramKind::discovery)
            return std::nullopt;
        const std::uint8_t type = bytes[8];
        if (type < 1 || type > 3)
            return std::nullopt;
        if (bytes[10] != 0 || bytes[11] != 0)
            return std::nullopt;

        DiscoveryMessage message;
        message.type = static_cast<DiscoveryType>(type);
        message.timeToLive = bytes[9];
        std::copy_n(bytes.begin() + 12, 8, message.sender.begin());
        if (message.type == DiscoveryType::leaving)
            return message;

        bool hasTimeline = false, hasSession = false, badTempo = false;
        const bool parsed = forEachEntry(bytes.subspan(kDiscoveryHeaderSize),
            [&](std::uint32_t key, const std::uint8_t* value, std::size_t size)
            {
                switch (key)
                {
                    case keys::timeline:
                    {
                        if (size != 24)
                            return false;
                        const Micros tempo = tempoFromWire(readI64(value));
                        badTempo = tempo <= 0;
                        message.state.timeline = { tempo, readI64(value + 8), readI64(value + 16) };
                        hasTimeline = true;
                        return true;
                    }
                    case keys::session:
                        if (size != 8)
                            return false;
                        std::copy_n(value, 8, message.state.sessionId.begin());
                        hasSession = true;
                        return true;
                    case keys::playState:
                        if (size != 17)
                            return false;
                        message.state.playState = { value[0] != 0, readI64(value + 1), readI64(value + 9) };
                        return true;
                    case keys::endpoint4:
                        if (size != 6)
                            return false;
                        message.state.measurement = Endpoint4 { readU32(value),
                            static_cast<std::uint16_t>((value[4] << 8) | value[5]) };
                        return true;
                    default:
                        return true; // unknown: skipped by its size
                }
            });
        if (! parsed || badTempo || ! hasTimeline || ! hasSession)
            return std::nullopt;
        return message;
    }

    std::optional<MeasurementMessage> readMeasurement(std::span<const std::uint8_t> bytes) noexcept
    {
        if (bytes.size() < kMeasurementHeaderSize || bytes.size() >= kReceiveBufferSize
            || kindOf(bytes) != DatagramKind::measurement)
            return std::nullopt;
        const std::uint8_t type = bytes[8];
        if (type != 1 && type != 2)
            return std::nullopt;

        MeasurementMessage message;
        message.type = static_cast<MeasurementType>(type);
        message.payload = bytes.subspan(kMeasurementHeaderSize);
        const bool parsed = forEachEntry(message.payload,
            [&](std::uint32_t key, const std::uint8_t* value, std::size_t size)
            {
                switch (key)
                {
                    case keys::hostTime:
                        if (size != 8)
                            return false;
                        message.hostTime = readI64(value);
                        return true;
                    case keys::sessionTime:
                        if (size != 8)
                            return false;
                        message.sessionTime = readI64(value);
                        return true;
                    case keys::prevSessionTime:
                        if (size != 8)
                            return false;
                        message.prevSessionTime = readI64(value);
                        return true;
                    case keys::session:
                    {
                        if (size != 8)
                            return false;
                        PeerId id {};
                        std::copy_n(value, 8, id.begin());
                        message.sessionId = id;
                        return true;
                    }
                    default:
                        return true;
                }
            });
        if (! parsed)
            return std::nullopt;
        return message;
    }
}
