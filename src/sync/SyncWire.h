#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "sync/SyncMath.h"

/*
    The beat sync's datagrams (CoolSynth's D361, docs/research/beat-sync-protocol.md, "The wire"): building and
    reading every datagram into plain structs. No sockets. Everything multi-byte is big-endian; a datagram is at most
    511 bytes.
*/
namespace coolsynth::sync
{
    using PeerId = std::array<std::uint8_t, 8>;

    inline constexpr std::size_t kMaxDatagramSize = 511;
    inline constexpr std::size_t kReceiveBufferSize = 512;
    inline constexpr std::uint16_t kDiscoveryPort = 20808;
    inline constexpr std::uint32_t kMulticastGroup = 0xE04C4E4Bu; // 224.76.78.75
    inline constexpr std::uint32_t kLoopbackAddress = 0x7F000001u; // 127.0.0.1
    inline constexpr std::uint8_t kAnnounceTimeToLive = 5;
    inline constexpr std::size_t kDiscoveryHeaderSize = 20;
    inline constexpr std::size_t kMeasurementHeaderSize = 9;
    inline constexpr std::size_t kMaxPingPayload = 32;

    inline constexpr std::array<std::uint8_t, 8> kDiscoveryMagic { 0x5F, 0x61, 0x73, 0x64, 0x70, 0x5F, 0x76, 0x01 };
    inline constexpr std::array<std::uint8_t, 8> kMeasurementMagic { 0x5F, 0x6C, 0x69, 0x6E, 0x6B, 0x5F, 0x76, 0x01 };

    /** The payload entries' keys, four ASCII characters read as a big-endian number. */
    namespace keys
    {
        inline constexpr std::uint32_t timeline = 0x746D6C6Eu;        // tmln
        inline constexpr std::uint32_t session = 0x73657373u;         // sess
        inline constexpr std::uint32_t playState = 0x73747374u;       // stst
        inline constexpr std::uint32_t endpoint4 = 0x6D657034u;       // mep4
        inline constexpr std::uint32_t hostTime = 0x5F5F6874u;        // __ht
        inline constexpr std::uint32_t sessionTime = 0x5F5F6774u;     // __gt
        inline constexpr std::uint32_t prevSessionTime = 0x5F706774u; // _pgt
    }

    /** An IPv4 address and UDP port, both in host byte order. */
    struct Endpoint4
    {
        std::uint32_t address = 0;
        std::uint16_t port = 0;

        friend bool operator==(const Endpoint4&, const Endpoint4&) = default;
    };

    [[nodiscard]] inline bool isLoopback(std::uint32_t address) noexcept { return (address >> 24) == 127u; }

    /** A datagram's bytes in a fixed buffer. */
    struct Datagram
    {
        std::array<std::uint8_t, kReceiveBufferSize> bytes {};
        std::size_t size = 0;

        [[nodiscard]] std::span<const std::uint8_t> view() const noexcept { return { bytes.data(), size }; }
    };

    /** Appends big-endian values to a datagram; fails, and stays failed, if it would pass 511 bytes. */
    class Writer
    {
    public:
        explicit Writer(Datagram& d) noexcept : datagram(d) { datagram.size = 0; }

        void raw(const std::uint8_t* data, std::size_t n) noexcept;
        void u8(std::uint8_t v) noexcept { raw(&v, 1); }
        void u16(std::uint16_t v) noexcept;
        void u32(std::uint32_t v) noexcept;
        void i64(std::int64_t v) noexcept;
        void entryHeader(std::uint32_t key, std::uint32_t size) noexcept { u32(key); u32(size); }

        [[nodiscard]] bool ok() const noexcept { return ! failed; }

    private:
        Datagram& datagram;
        bool failed = false;
    };

    enum class DiscoveryType : std::uint8_t { announce = 1, reply = 2, leaving = 3 };
    enum class MeasurementType : std::uint8_t { ping = 1, pong = 2 };
    enum class DatagramKind { discovery, measurement, unknown };

    /** What an announce or reply says about its sender. */
    struct PeerState
    {
        PeerId sessionId {};
        Timeline timeline {};
        PlayState playState {};
        std::optional<Endpoint4> measurement;
    };

    struct DiscoveryMessage
    {
        DiscoveryType type = DiscoveryType::announce;
        std::uint8_t timeToLive = 0;
        PeerId sender {};
        PeerState state {}; // announces and replies only
    };

    struct MeasurementMessage
    {
        MeasurementType type = MeasurementType::ping;
        std::optional<Micros> hostTime;        // __ht
        std::optional<Micros> sessionTime;     // __gt
        std::optional<Micros> prevSessionTime; // _pgt
        std::optional<PeerId> sessionId;       // sess
        std::span<const std::uint8_t> payload; // the entries after the header, as they came
    };

    /** Builds an announce or reply (with the four entries) or a leaving message (header alone). False if it does
        not fit. */
    bool buildDiscovery(Datagram& out, DiscoveryType type, const PeerId& sender, const PeerState& state) noexcept;
    bool buildLeaving(Datagram& out, const PeerId& sender) noexcept;
    /** A ping: __ht, and _pgt from the second ping of a measurement on. */
    bool buildPing(Datagram& out, Micros hostTime, std::optional<Micros> previousSessionTime) noexcept;
    /** A pong: sess, __gt, then the ping's payload byte for byte. */
    bool buildPong(Datagram& out, const PeerId& sessionId, Micros sessionTime,
                   std::span<const std::uint8_t> pingPayload) noexcept;

    [[nodiscard]] DatagramKind kindOf(std::span<const std::uint8_t> bytes) noexcept;
    /** Reads a discovery datagram; nothing if it is to be ignored or dropped: a wrong magic, under 20 bytes, a
        group other than 0, an unknown type, a broken payload, an announce or reply without tmln or sess, a tempo of
        zero or below. Tempos are clamped to 20 to 999 BPM. */
    [[nodiscard]] std::optional<DiscoveryMessage> readDiscovery(std::span<const std::uint8_t> bytes) noexcept;
    /** Reads a measurement datagram; nothing if it is to be ignored or dropped. */
    [[nodiscard]] std::optional<MeasurementMessage> readMeasurement(std::span<const std::uint8_t> bytes) noexcept;

    [[nodiscard]] std::int64_t readI64(const std::uint8_t* p) noexcept;
}
