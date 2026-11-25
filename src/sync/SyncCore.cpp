#include "sync/SyncCore.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace coolsynth::sync
{
    void addOffsetEstimates(std::vector<double>& estimates, Micros arrival, Micros gt, Micros ht, Micros pgt)
    {
        const double g = static_cast<double>(gt), h = static_cast<double>(ht);
        if (gt != 0 && ht != 0)
            estimates.push_back(g - (static_cast<double>(arrival) + h) / 2.0);
        if (pgt != 0)
            estimates.push_back((g + static_cast<double>(pgt)) / 2.0 - h);
    }

    double median(std::vector<double>& values)
    {
        if (values.empty())
            return 0.0;
        const std::size_t n = values.size(), mid = n / 2;
        std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
        const double upper = values[mid];
        if (n % 2 == 1)
            return upper;
        const double lower = *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid));
        return (lower + upper) / 2.0;
    }

    SessionChoice chooseSession(Micros otherMinusOurs, const PeerId& other, const PeerId& ours) noexcept
    {
        if (otherMinusOurs > SyncCore::kHalfSecond)
            return SessionChoice::join;
        if (otherMinusOurs > -SyncCore::kHalfSecond && otherMinusOurs < SyncCore::kHalfSecond)
            return other < ours ? SessionChoice::join : SessionChoice::stay; // bytewise, as unsigned numbers
        return SessionChoice::stay;
    }

    std::string idText(const PeerId& id)
    {
        std::string s;
        for (const auto b : id)
            s += (b >= 33 && b <= 126) ? static_cast<char>(b) : '?';
        return s;
    }

    SyncCore::SyncCore(std::uint64_t seed) : random(seed) {}

    PeerId SyncCore::newPeerId()
    {
        if (idSource)
            return idSource();
        std::uniform_int_distribution<int> printable(33, 126);
        PeerId id {};
        for (auto& b : id)
            b = static_cast<std::uint8_t>(printable(random));
        return id;
    }

    void SyncCore::log(const std::string& text) const
    {
        if (logger)
            logger(text);
    }

    void SyncCore::markChanged()
    {
        changed = true;
        for (auto& g : gateways)
            g.announceDue = true;
    }

    //==================================================================================================================
    void SyncCore::found(Micros hostNow, Micros tempo, MicroBeats beatNow, const Snapshot& carry)
    {
        running = true;
        playChange = carry.playChange;
        gridApplied = carry.gridApplied;
        playApplied = carry.playApplied;
        epoch = carry.epoch + 1;
        lastTick = hostNow;
        becomeFounder(hostNow, tempoFromWire(tempo > 0 ? tempo : 500000), beatNow);
        char bpm[32];
        std::snprintf(bpm, sizeof(bpm), "%.3f", bpmFromTempo(line.tempo));
        log("founded session " + idText(session) + " at " + bpm + " BPM");
    }

    void SyncCore::becomeFounder(Micros hostNow, Micros tempo, MicroBeats beatNow)
    {
        self = newPeerId();
        session = self;
        offset = -hostNow;
        line = { tempo, beatNow, 0 };
        play = {};
        joinedOther = false;
        easing = false;
        rows.clear();
        known.clear();
        measurements.clear();
        failures.clear();
        peers = 0;
        markChanged();
    }

    void SyncCore::refound(Micros hostNow, const char* why)
    {
        const MicroBeats beatNow = line.beatAt(hostNow + offset);
        becomeFounder(hostNow, line.tempo, beatNow);
        log(std::string("founded session ") + idText(session) + " (" + why + ")");
    }

    void SyncCore::leaveAll(Micros hostNow)
    {
        (void) hostNow;
        for (const auto& g : gateways)
        {
            Datagram d;
            if (buildLeaving(d, self))
                send(g.id, SocketKind::discovery, { kMulticastGroup, kDiscoveryPort }, d);
        }
        gateways.clear();
        rows.clear();
        known.clear();
        measurements.clear();
        peers = 0;
        running = false;
        changed = true;
    }

    //==================================================================================================================
    SyncCore::Gateway* SyncCore::findGateway(int id)
    {
        for (auto& g : gateways)
            if (g.id == id)
                return &g;
        return nullptr;
    }

    void SyncCore::addGateway(int id, Endpoint4 measurementEndpoint, Micros hostNow)
    {
        if (findGateway(id) != nullptr)
            return;
        gateways.push_back({ id, measurementEndpoint });
        if (running)
            announce(gateways.back(), hostNow);
    }

    void SyncCore::removeGateway(int id, bool sayLeaving, Micros hostNow)
    {
        if (findGateway(id) == nullptr)
            return;
        if (sayLeaving && running)
        {
            Datagram d;
            if (buildLeaving(d, self))
                send(id, SocketKind::discovery, { kMulticastGroup, kDiscoveryPort }, d);
        }
        gateways.erase(std::remove_if(gateways.begin(), gateways.end(), [id](const Gateway& g) { return g.id == id; }),
                       gateways.end());
        // Measurements through it stop; their sessions are measured again when heard on another gateway.
        for (std::size_t i = 0; i < measurements.size();)
        {
            if (measurements[i].gateway == id)
            {
                const PeerId s = measurements[i].session;
                const bool own = measurements[i].own;
                measurements.erase(measurements.begin() + static_cast<std::ptrdiff_t>(i));
                if (own)
                    remeasureAt = hostNow + kRemeasureInterval;
                else
                    known.erase(std::remove_if(known.begin(), known.end(), [&s](const KnownSession& k) { return k.id == s; }),
                                known.end());
            }
            else
                ++i;
        }
        removeRows([id](const Row& r) { return r.gateway == id; }, hostNow);
    }

    PeerState SyncCore::stateFor(const Gateway& gateway) const
    {
        PeerState s;
        s.sessionId = session;
        s.timeline = line;
        s.playState = play;
        s.measurement = gateway.endpoint;
        return s;
    }

    void SyncCore::send(int gateway, SocketKind from, Endpoint4 to, const Datagram& d)
    {
        pending.push_back({ gateway, from, to, d });
    }

    void SyncCore::announce(Gateway& gateway, Micros hostNow)
    {
        Datagram d;
        if (buildDiscovery(d, DiscoveryType::announce, self, stateFor(gateway)))
            send(gateway.id, SocketKind::discovery, { kMulticastGroup, kDiscoveryPort }, d);
        gateway.lastAnnounce = gateway.lastSend = hostNow;
        gateway.announceDue = false;
    }

    //==================================================================================================================
    void SyncCore::receive(int gateway, SocketKind socket, Endpoint4 from, std::span<const std::uint8_t> bytes, Micros hostNow)
    {
        if (! running)
            return;
        const Gateway* g = findGateway(gateway);
        if (g == nullptr)
            return;
        // The loopback gateway hears only this computer.
        if (isLoopback(g->endpoint.address) && ! isLoopback(from.address))
            return;

        switch (kindOf(bytes))
        {
            case DatagramKind::discovery:
                if (socket == SocketKind::measurement)
                    return;
                if (const auto message = readDiscovery(bytes); message && message->sender != self)
                    onDiscovery(gateway, from, *message, hostNow);
                return;
            case DatagramKind::measurement:
                if (socket != SocketKind::measurement)
                    return;
                if (const auto message = readMeasurement(bytes))
                    onMeasurement(gateway, socket, from, *message, hostNow);
                return;
            case DatagramKind::unknown:
            default:
                return;
        }
    }

    void SyncCore::onDiscovery(int gateway, Endpoint4 from, const DiscoveryMessage& message, Micros hostNow)
    {
        if (message.type == DiscoveryType::leaving)
        {
            removeRows([&](const Row& r) { return r.peer == message.sender && r.gateway == gateway; }, hostNow);
            return;
        }

        auto row = std::find_if(rows.begin(), rows.end(),
                                [&](const Row& r) { return r.peer == message.sender && r.gateway == gateway; });
        if (row == rows.end())
        {
            rows.push_back({ message.sender, gateway });
            row = rows.end() - 1;
        }
        row->state = message.state;
        row->expiry = hostNow + static_cast<Micros>(message.timeToLive) * 1000000;

        if (message.state.sessionId == session)
        {
            timelineRule(message.state.timeline);
            playStateRule(message.state.playState);
        }
        else if (auto* k = findKnown(message.state.sessionId))
        {
            if (message.state.timeline.beatOrigin > k->best.beatOrigin)
                k->best = message.state.timeline;
        }
        else if (! recentlyFailed(message.state.sessionId, hostNow))
        {
            known.push_back({ message.state.sessionId, message.state.timeline });
            log("heard session " + idText(message.state.sessionId) + " from peer " + idText(message.sender));
            startMeasurement(message.state.sessionId, false, hostNow);
        }

        // Sessions nobody is in any more are forgotten, unless being measured.
        known.erase(std::remove_if(known.begin(), known.end(), [this](const KnownSession& k)
        {
            const bool hasRow = std::any_of(rows.begin(), rows.end(), [&](const Row& r) { return r.state.sessionId == k.id; });
            const bool measuring = std::any_of(measurements.begin(), measurements.end(),
                                               [&](const Measurement& m) { return m.session == k.id; });
            return ! hasRow && ! measuring;
        }), known.end());
        updatePeerCount(hostNow);

        if (message.type == DiscoveryType::announce && running)
            if (auto* g = findGateway(gateway))
            {
                Datagram d;
                if (buildDiscovery(d, DiscoveryType::reply, self, stateFor(*g)))
                    send(gateway, SocketKind::discovery, from, d);
                g->lastSend = hostNow;
            }
    }

    void SyncCore::timelineRule(const Timeline& heard)
    {
        if (heard.beatOrigin > line.beatOrigin)
        {
            line = heard;
            markChanged();
        }
    }

    void SyncCore::playStateRule(const PlayState& heard)
    {
        if (heard.time > play.time)
        {
            const bool was = play.playing;
            play = heard;
            markChanged();
            if (sharing && heard.playing != was)
                ++playChange;
        }
    }

    void SyncCore::removeRows(const std::function<bool(const Row&)>& which, Micros hostNow)
    {
        const auto before = rows.size();
        rows.erase(std::remove_if(rows.begin(), rows.end(), which), rows.end());
        if (rows.size() != before)
            known.erase(std::remove_if(known.begin(), known.end(), [this](const KnownSession& k)
            {
                const bool hasRow = std::any_of(rows.begin(), rows.end(), [&](const Row& r) { return r.state.sessionId == k.id; });
                const bool measuring = std::any_of(measurements.begin(), measurements.end(),
                                                   [&](const Measurement& m) { return m.session == k.id; });
                return ! hasRow && ! measuring;
            }), known.end());
        updatePeerCount(hostNow);
    }

    void SyncCore::updatePeerCount(Micros hostNow)
    {
        std::vector<PeerId> members;
        for (const auto& r : rows)
            if (r.state.sessionId == session && std::find(members.begin(), members.end(), r.peer) == members.end())
                members.push_back(r.peer);
        const int count = static_cast<int>(members.size());
        if (count == peers)
            return;
        const int was = peers;
        peers = count;
        changed = true;
        if (was > 0 && count == 0 && running)
            refound(hostNow, "the last other peer went");
    }

    //==================================================================================================================
    SyncCore::KnownSession* SyncCore::findKnown(const PeerId& id)
    {
        for (auto& k : known)
            if (k.id == id)
                return &k;
        return nullptr;
    }

    bool SyncCore::recentlyFailed(const PeerId& id, Micros hostNow) const
    {
        return std::any_of(failures.begin(), failures.end(), [&](const auto& f)
        {
            return f.first == id && hostNow - f.second < kRetryAfterFailure;
        });
    }

    void SyncCore::startMeasurement(const PeerId& sessionToMeasure, bool own, Micros hostNow)
    {
        if (std::any_of(measurements.begin(), measurements.end(), [&](const Measurement& m) { return m.session == sessionToMeasure; }))
            return;

        auto usable = [&](const Row& r)
        {
            if (r.state.sessionId != sessionToMeasure || ! r.state.measurement)
                return false;
            const Gateway* g = findGateway(r.gateway);
            return g != nullptr && (! isLoopback(g->endpoint.address) || isLoopback(r.state.measurement->address));
        };
        // The founder if it is in the table, otherwise the first member.
        auto target = std::find_if(rows.begin(), rows.end(), [&](const Row& r) { return usable(r) && r.peer == sessionToMeasure; });
        if (target == rows.end())
            target = std::find_if(rows.begin(), rows.end(), usable);
        if (target == rows.end())
        {
            if (own)
                remeasureAt = hostNow + kRemeasureInterval;
            else
            {
                failures.push_back({ sessionToMeasure, hostNow });
                known.erase(std::remove_if(known.begin(), known.end(), [&](const KnownSession& k) { return k.id == sessionToMeasure; }),
                            known.end());
            }
            return;
        }

        Measurement m;
        m.session = sessionToMeasure;
        m.gateway = target->gateway;
        m.target = *target->state.measurement;
        m.startedAt = hostNow;
        m.own = own;
        m.estimates.reserve(kEnoughEstimates + 2);
        measurements.push_back(std::move(m));
        sendPing(measurements.back(), hostNow, std::nullopt);
    }

    void SyncCore::sendPing(Measurement& m, Micros hostNow, std::optional<Micros> previousSessionTime)
    {
        Datagram d;
        if (buildPing(d, hostNow, previousSessionTime))
            send(m.gateway, SocketKind::measurement, m.target, d);
        m.deadline = hostNow + kPingTimeout;
    }

    void SyncCore::onMeasurement(int gateway, SocketKind socket, Endpoint4 from, const MeasurementMessage& message, Micros hostNow)
    {
        (void) socket;
        if (message.type == MeasurementType::ping)
        {
            if (message.payload.size() <= kMaxPingPayload)
            {
                Datagram d;
                if (buildPong(d, session, hostNow + offset, message.payload))
                    send(gateway, SocketKind::measurement, from, d);
            }
            return;
        }

        const auto it = std::find_if(measurements.begin(), measurements.end(),
                                     [&](const Measurement& m) { return m.target == from && m.gateway == gateway; });
        if (it == measurements.end())
            return;
        const auto index = static_cast<std::size_t>(it - measurements.begin());
        if (! message.sessionId || *message.sessionId != it->session)
        {
            log("a pong from " + idText(it->session) + "'s peer named another session");
            failMeasurement(index, hostNow);
            return;
        }
        if (! message.sessionTime)
            return;
        addOffsetEstimates(it->estimates, hostNow, *message.sessionTime, message.hostTime.value_or(0),
                           message.prevSessionTime.value_or(0));
        if (it->estimates.size() > kEnoughEstimates)
            finishMeasurement(index, hostNow);
        else if (++it->pongs > kMaxPongs)
            failMeasurement(index, hostNow);
        else
            sendPing(*it, hostNow, *message.sessionTime);
    }

    void SyncCore::finishMeasurement(std::size_t index, Micros hostNow)
    {
        Measurement m = std::move(measurements[index]);
        measurements.erase(measurements.begin() + static_cast<std::ptrdiff_t>(index));
        const int count = static_cast<int>(m.estimates.size());
        const Micros measured = std::llround(median(m.estimates));
        reports.push_back({ m.session, true, m.own, count, measured, m.startedAt, hostNow });

        if (m.own)
        {
            if (m.session != session)
                return;
            const Micros shown = snapshot(hostNow).clockOffset;
            log("re-measured " + idText(session) + ": offset moved by " + std::to_string(measured - offset) + " us");
            easeFrom = shown;
            easeStart = hostNow;
            easing = true;
            offset = measured;
            remeasureAt = hostNow + kRemeasureInterval;
            changed = true;
            return;
        }
        if (m.session == session)
            return;
        const Micros difference = measured - offset;
        log("measured " + idText(m.session) + ": " + std::to_string(difference) + " us against ours, "
            + std::to_string(count) + " estimates");
        if (chooseSession(difference, m.session, session) == SessionChoice::join)
            join(m.session, measured, hostNow);
    }

    void SyncCore::failMeasurement(std::size_t index, Micros hostNow)
    {
        const Measurement m = std::move(measurements[index]);
        measurements.erase(measurements.begin() + static_cast<std::ptrdiff_t>(index));
        reports.push_back({ m.session, false, m.own, static_cast<int>(m.estimates.size()), 0, m.startedAt, hostNow });
        log("measurement of " + idText(m.session) + " failed");
        if (m.own)
        {
            remeasureAt = hostNow + kRemeasureInterval;
            return;
        }
        failures.push_back({ m.session, hostNow });
        known.erase(std::remove_if(known.begin(), known.end(), [&](const KnownSession& k) { return k.id == m.session; }),
                    known.end());
        removeRows([&](const Row& r) { return r.state.sessionId == m.session; }, hostNow);
    }

    void SyncCore::join(const PeerId& other, Micros otherOffset, Micros hostNow)
    {
        const PeerId previous = session;
        const Timeline previousLine = line;
        if (const auto* k = findKnown(other))
            line = k->best;
        known.erase(std::remove_if(known.begin(), known.end(), [&](const KnownSession& k) { return k.id == other; }),
                    known.end());
        if (findKnown(previous) == nullptr)
            known.push_back({ previous, previousLine });

        session = other;
        offset = otherOffset;
        easing = false;
        // The session's newest play state among its peers, taken over but not applied to the app.
        play = {};
        for (const auto& r : rows)
            if (r.state.sessionId == other && r.state.playState.time > play.time)
                play = r.state.playState;
        joinedOther = true;
        remeasureAt = hostNow + kRemeasureInterval;
        ++epoch;
        measurements.erase(std::remove_if(measurements.begin(), measurements.end(), [](const Measurement& m) { return m.own; }),
                           measurements.end());
        log("joined session " + idText(other));
        markChanged();
        updatePeerCount(hostNow);
    }

    //==================================================================================================================
    void SyncCore::tick(Micros hostNow)
    {
        if (! running)
            return;
        if (lastTick && hostNow - *lastTick > kWakeGap)
            refound(hostNow, "a wake from sleep");
        lastTick = hostNow;

        removeRows([hostNow](const Row& r) { return hostNow > r.expiry + kRowGrace; }, hostNow);

        for (std::size_t i = 0; i < measurements.size();)
        {
            auto& m = measurements[i];
            if (hostNow < m.deadline)
            {
                ++i;
                continue;
            }
            if (++m.timeouts >= kMaxTimeouts)
            {
                failMeasurement(i, hostNow);
                continue;
            }
            sendPing(m, hostNow, std::nullopt);
            ++i;
        }

        if (joinedOther && hostNow >= remeasureAt
            && std::none_of(measurements.begin(), measurements.end(), [](const Measurement& m) { return m.own; }))
        {
            remeasureAt = hostNow + kRemeasureInterval;
            startMeasurement(session, true, hostNow);
        }

        for (auto& g : gateways)
            if (hostNow >= g.lastAnnounce + kAnnounceInterval || (g.announceDue && hostNow >= g.lastSend + kAnnounceGap))
                announce(g, hostNow);

        if (easing)
        {
            changed = true;
            if (hostNow >= easeStart + kOffsetEase)
                easing = false;
        }

        failures.erase(std::remove_if(failures.begin(), failures.end(),
                                      [hostNow](const auto& f) { return hostNow - f.second >= kRetryAfterFailure; }),
                       failures.end());
    }

    Micros SyncCore::nextWake(Micros hostNow) const
    {
        if (! running)
            return hostNow + 1000000;
        Micros wake = hostNow + 1000000;
        for (const auto& g : gateways)
        {
            wake = std::min(wake, g.lastAnnounce + kAnnounceInterval);
            if (g.announceDue)
                wake = std::min(wake, g.lastSend + kAnnounceGap);
        }
        for (const auto& m : measurements)
            wake = std::min(wake, m.deadline);
        for (const auto& r : rows)
            wake = std::min(wake, r.expiry + kRowGrace + 1);
        if (joinedOther)
            wake = std::min(wake, remeasureAt);
        if (easing)
            wake = std::min(wake, hostNow + 5000);
        return std::max(wake, hostNow);
    }

    //==================================================================================================================
    void SyncCore::setStartStopSharing(bool on)
    {
        if (on != sharing)
        {
            sharing = on;
            changed = true;
        }
    }

    void SyncCore::applyGrid(const GridProposal& proposal, Micros hostNow)
    {
        (void) hostNow;
        if (proposal.number <= gridApplied)
            return;
        gridApplied = proposal.number;
        changed = true;
        if (! running)
            return;
        const Micros tempo = proposal.tempo > 0 ? tempoFromWire(proposal.tempo) : line.tempo;
        const Micros at = proposal.hostTime + offset;
        const MicroBeats beat = proposal.epoch == epoch ? proposal.beat : line.beatAt(at);
        line = timelineThrough(line, tempo, beat, at);
        markChanged();
    }

    void SyncCore::applyPlay(const PlayProposal& proposal, Micros hostNow)
    {
        if (proposal.number <= playApplied)
            return;
        playApplied = proposal.number;
        changed = true;
        if (! running)
            return;
        const MicroBeats beat = proposal.epoch == epoch ? proposal.beat : line.beatAt(proposal.hostTime + offset);
        Micros time = hostNow + offset;
        if (time <= play.time)
            time = play.time + 1;
        play = { proposal.playing, beat, time };
        markChanged();
    }

    void SyncCore::acknowledge(std::int64_t gridNumber, std::int64_t playNumber)
    {
        gridApplied = std::max(gridApplied, gridNumber);
        playApplied = std::max(playApplied, playNumber);
        changed = true;
    }

    bool SyncCore::takeSnapshotChanged() noexcept
    {
        const bool c = changed;
        changed = false;
        return c;
    }

    Snapshot SyncCore::snapshot(Micros hostNow) const
    {
        Snapshot s;
        s.session = session;
        s.peers = peers;
        s.running = running;
        s.startStopSharing = sharing;
        s.timeline = line;
        s.clockOffset = offset;
        if (easing)
        {
            const double f = std::clamp(static_cast<double>(hostNow - easeStart) / static_cast<double>(kOffsetEase), 0.0, 1.0);
            s.clockOffset = easeFrom + std::llround(static_cast<double>(offset - easeFrom) * f);
        }
        s.playState = play;
        s.playChange = playChange;
        s.gridApplied = gridApplied;
        s.playApplied = playApplied;
        s.epoch = epoch;
        return s;
    }
}
