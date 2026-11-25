#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <vector>

#include "sync/SyncMath.h"
#include "sync/SyncWire.h"

/*
    The beat sync's protocol as a state machine (CoolSynth's D361, docs/research/beat-sync-protocol.md, "A peer's
    life"), with no sockets and no clock of its own. It is fed events (a datagram arrived on gateway g at host time
    h, the time is now h, the app proposed this, a gateway opened or closed) and answers with datagrams to send
    (outbox), the time it next wants to be woken (nextWake) and a snapshot for the audio thread. The peer table, the
    clock measurements, the sessions and the timeline and play state rules all live here. It runs on one thread (the
    network thread, or a test's simulated network) and may allocate.
*/
namespace coolsynth::sync
{
    /** The socket of a gateway a datagram is sent from or arrived on. */
    enum class SocketKind : std::uint8_t { group, discovery, measurement };

    struct Outgoing
    {
        int gateway = 0;
        SocketKind from = SocketKind::discovery;
        Endpoint4 to {};
        Datagram datagram {};
    };

    /** A grid proposal from the audio thread: a tempo, and one point the new grid must pass through (a session beat
        and the host time it falls at). The epoch is the session numbering the beat was counted in. */
    struct GridProposal
    {
        std::int64_t number = 0;
        Micros tempo = 500000;
        MicroBeats beat = 0;
        Micros hostTime = 0;
        std::int64_t epoch = 0;
    };

    /** A play proposal from the audio thread: playing or not, the host time of the press and the session beat there. */
    struct PlayProposal
    {
        std::int64_t number = 0;
        bool playing = false;
        Micros hostTime = 0;
        MicroBeats beat = 0;
        std::int64_t epoch = 0;
    };

    /** What the network thread hands the audio thread and the UI. */
    struct Snapshot
    {
        std::int64_t number = 0;     // grows with every publish
        PeerId session {};
        int peers = 0;               // other peers in our session
        bool running = false;        // the sync is switched on
        bool startStopSharing = true;
        Timeline timeline {};
        Micros clockOffset = 0;      // session time = host time + clock offset (eased over a second after a re-measure)
        PlayState playState {};
        std::int64_t playChange = 0; // grows when the app should follow a play state from another peer
        std::int64_t gridApplied = 0;
        std::int64_t playApplied = 0;
        std::int64_t epoch = 0;      // grows when the session's beat numbering changes (switching on, joining)
    };

    /** The outcome of a clock measurement, for tests, the probe and the log. */
    struct MeasurementReport
    {
        PeerId session {};
        bool ok = false;
        bool ownSession = false;
        int estimates = 0;
        Micros offset = 0;
        Micros startedAt = 0;
        Micros finishedAt = 0;
    };

    /** Up to two clock offset estimates from one pong: gt against the middle of our round trip, and our ping's send
        time against the middle of the two answers around it. */
    void addOffsetEstimates(std::vector<double>& estimates, Micros arrival, Micros gt, Micros ht, Micros pgt);
    /** The median, the mean of the middle two for an even count. Reorders the values. */
    [[nodiscard]] double median(std::vector<double>& values);

    enum class SessionChoice { join, stay };
    /** Which session wins, from the other session's clock offset minus ours: older by more than half a second wins;
        within half a second the smaller id. */
    [[nodiscard]] SessionChoice chooseSession(Micros otherMinusOurs, const PeerId& other, const PeerId& ours) noexcept;

    class SyncCore
    {
    public:
        static constexpr Micros kAnnounceInterval = 250000;
        static constexpr Micros kAnnounceGap = 50000;
        static constexpr Micros kPingTimeout = 50000;
        static constexpr int kMaxTimeouts = 6;
        static constexpr std::size_t kEnoughEstimates = 100;
        /** Our own guard: a peer whose pongs never give estimates (a session time of 0 reads as absent) would be
            pinged without end; a measurement gives up after this many pongs. */
        static constexpr int kMaxPongs = 250;
        static constexpr Micros kRemeasureInterval = 30000000;
        static constexpr Micros kRetryAfterFailure = 1000000;
        static constexpr Micros kRowGrace = 1000000;
        static constexpr Micros kWakeGap = 2000000;
        static constexpr Micros kOffsetEase = 1000000;
        static constexpr Micros kHalfSecond = 500000;

        using IdSource = std::function<PeerId()>;
        using Logger = std::function<void(const std::string&)>;

        explicit SyncCore(std::uint64_t seed);

        /** Tests choose the peer ids; the default draws each byte from 33 to 126. */
        void setIdSource(IdSource source) { idSource = std::move(source); }
        void setLogger(Logger l) { logger = std::move(l); }

        /** Switching on: founds a session of our own, the beat `beatNow` at session time zero now. The counters of
            the previous snapshot carry on, so the audio thread's comparisons keep working across a restart. */
        void found(Micros hostNow, Micros tempo, MicroBeats beatNow, const Snapshot& carry);
        /** Switching off: a leaving message on every gateway, and every gateway gone. */
        void leaveAll(Micros hostNow);

        /** A gateway opened: its address and its measurement socket's port. It announces at once. */
        void addGateway(int id, Endpoint4 measurementEndpoint, Micros hostNow);
        /** A gateway closing (address gone, a failed send): its rows go, with a leaving message if asked. */
        void removeGateway(int id, bool sayLeaving, Micros hostNow);

        void receive(int gateway, SocketKind socket, Endpoint4 from, std::span<const std::uint8_t> bytes, Micros hostNow);
        void tick(Micros hostNow);

        void setStartStopSharing(bool on);
        void applyGrid(const GridProposal& proposal, Micros hostNow);
        void applyPlay(const PlayProposal& proposal, Micros hostNow);
        /** Marks proposals already contained in the state the session was founded from. */
        void acknowledge(std::int64_t gridNumber, std::int64_t playNumber);

        /** The datagrams to send, in order; the caller sends and clears them. */
        [[nodiscard]] std::vector<Outgoing>& outbox() noexcept { return pending; }
        /** The host time the core next needs a tick at. */
        [[nodiscard]] Micros nextWake(Micros hostNow) const;
        /** True once after anything the snapshot shows changed. */
        [[nodiscard]] bool takeSnapshotChanged() noexcept;
        [[nodiscard]] Snapshot snapshot(Micros hostNow) const;

        [[nodiscard]] bool isRunning() const noexcept { return running; }
        [[nodiscard]] const PeerId& peerId() const noexcept { return self; }
        [[nodiscard]] const PeerId& sessionId() const noexcept { return session; }
        [[nodiscard]] const Timeline& timeline() const noexcept { return line; }
        [[nodiscard]] Micros clockOffset() const noexcept { return offset; }
        [[nodiscard]] const PlayState& playState() const noexcept { return play; }
        [[nodiscard]] int peerCount() const noexcept { return peers; }
        [[nodiscard]] std::int64_t playChangeCount() const noexcept { return playChange; }
        [[nodiscard]] Micros sessionTime(Micros hostNow) const noexcept { return hostNow + offset; }
        [[nodiscard]] int rowCount() const noexcept { return static_cast<int>(rows.size()); }
        [[nodiscard]] int gatewayCount() const noexcept { return static_cast<int>(gateways.size()); }
        [[nodiscard]] const std::vector<MeasurementReport>& measurementReports() const noexcept { return reports; }

    private:
        struct Gateway
        {
            int id = 0;
            Endpoint4 endpoint {};
            Micros lastAnnounce = std::numeric_limits<Micros>::min() / 2;
            Micros lastSend = std::numeric_limits<Micros>::min() / 2;
            bool announceDue = true;
        };

        struct Row
        {
            PeerId peer {};
            int gateway = 0;
            PeerState state {};
            Micros expiry = 0;
        };

        struct KnownSession
        {
            PeerId id {};
            Timeline best {};
        };

        struct Measurement
        {
            PeerId session {};
            int gateway = 0;
            Endpoint4 target {};
            std::vector<double> estimates;
            int timeouts = 0;
            int pongs = 0;
            Micros deadline = 0;
            Micros startedAt = 0;
            bool own = false;
        };

        PeerId newPeerId();
        void becomeFounder(Micros hostNow, Micros tempo, MicroBeats beatNow);
        void refound(Micros hostNow, const char* why);
        void markChanged();
        void log(const std::string& text) const;

        void announce(Gateway& gateway, Micros hostNow);
        void send(int gateway, SocketKind from, Endpoint4 to, const Datagram& d);
        [[nodiscard]] PeerState stateFor(const Gateway& gateway) const;
        [[nodiscard]] Gateway* findGateway(int id);

        void onDiscovery(int gateway, Endpoint4 from, const DiscoveryMessage& message, Micros hostNow);
        void onMeasurement(int gateway, SocketKind socket, Endpoint4 from, const MeasurementMessage& message, Micros hostNow);
        void timelineRule(const Timeline& heard);
        void playStateRule(const PlayState& heard);
        void updatePeerCount(Micros hostNow);
        void removeRows(const std::function<bool(const Row&)>& which, Micros hostNow);

        [[nodiscard]] KnownSession* findKnown(const PeerId& id);
        [[nodiscard]] bool recentlyFailed(const PeerId& id, Micros hostNow) const;
        void startMeasurement(const PeerId& sessionToMeasure, bool own, Micros hostNow);
        void sendPing(Measurement& m, Micros hostNow, std::optional<Micros> previousSessionTime);
        void finishMeasurement(std::size_t index, Micros hostNow);
        void failMeasurement(std::size_t index, Micros hostNow);
        void join(const PeerId& other, Micros otherOffset, Micros hostNow);

        std::mt19937_64 random;
        IdSource idSource;
        Logger logger;

        bool running = false;
        PeerId self {}, session {};
        Timeline line {};
        Micros offset = 0;
        PlayState play {};
        bool joinedOther = false;
        Micros remeasureAt = 0;
        bool sharing = true;
        int peers = 0;

        // The clock offset the audio thread sees eases from the old to a re-measured one over a second.
        bool easing = false;
        Micros easeFrom = 0, easeStart = 0;

        std::int64_t playChange = 0, gridApplied = 0, playApplied = 0, epoch = 0;
        bool changed = true;
        std::optional<Micros> lastTick;

        std::vector<Gateway> gateways;
        std::vector<Row> rows;
        std::vector<KnownSession> known;
        std::vector<Measurement> measurements;
        std::vector<std::pair<PeerId, Micros>> failures;
        std::vector<Outgoing> pending;
        std::vector<MeasurementReport> reports;
    };

    /** A peer id as text (its bytes are printable). */
    [[nodiscard]] std::string idText(const PeerId& id);
}
