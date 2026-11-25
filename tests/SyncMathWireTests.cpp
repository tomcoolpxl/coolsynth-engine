// The beat sync's maths and wire (CoolSynth's D361): the unit tests of docs/research/beat-sync-protocol.md's test plan,
// "Unit tests of the maths and the wire", with its byte examples.

#include <juce_core/juce_core.h>

#include <memory>
#include <string_view>
#include <vector>

#include "sync/SyncMath.h"
#include "sync/SyncWire.h"

namespace
{
    using namespace coolsynth::sync;

    std::vector<std::uint8_t> hex(std::string_view text)
    {
        std::vector<std::uint8_t> out;
        int nibbles = 0, value = 0;
        for (const char c : text)
        {
            int v = -1;
            if (c >= '0' && c <= '9') v = c - '0';
            else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
            else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
            if (v < 0)
                continue;
            value = value * 16 + v;
            if (++nibbles == 2)
            {
                out.push_back(static_cast<std::uint8_t>(value));
                nibbles = value = 0;
            }
        }
        return out;
    }

    std::vector<std::uint8_t> bytesOf(const Datagram& d) { return { d.bytes.begin(), d.bytes.begin() + static_cast<std::ptrdiff_t>(d.size) }; }

    std::vector<std::uint8_t> i64Bytes(std::int64_t v)
    {
        Datagram d;
        Writer w(d);
        w.i64(v);
        return bytesOf(d);
    }

    PeerId idOf(std::string_view eight)
    {
        PeerId id {};
        for (std::size_t i = 0; i < 8; ++i)
            id[i] = static_cast<std::uint8_t>(eight[i]);
        return id;
    }

    // The announce of the document's byte examples: peer ABCDEFGH, its own session, 120 BPM, anchor beat 0 at
    // session time 0, stopped, measurement socket at 192.168.1.20 port 50123.
    constexpr std::string_view exampleAnnounce =
        "5F 61 73 64 70 5F 76 01 01 05 00 00 41 42 43 44 45 46 47 48"
        "74 6D 6C 6E 00 00 00 18 00 00 00 00 00 07 A1 20"
        "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
        "73 65 73 73 00 00 00 08 41 42 43 44 45 46 47 48"
        "73 74 73 74 00 00 00 11 00 00 00 00 00 00 00 00 00"
        "00 00 00 00 00 00 00 00"
        "6D 65 70 34 00 00 00 06 C0 A8 01 14 C3 CB";

    PeerState exampleState()
    {
        PeerState s;
        s.sessionId = idOf("ABCDEFGH");
        s.timeline = { 500000, 0, 0 };
        s.measurement = Endpoint4 { 0xC0A80114u, 50123 };
        return s;
    }
}

class SyncMathTests final : public juce::UnitTest
{
public:
    SyncMathTests() : juce::UnitTest("Beat sync maths", "Sync") {}

    void runTest() override
    {
        beginTest("tempo_encodes_and_clamps");
        {
            expectEquals(tempoFromBpm(120.0), (Micros) 500000);
            expect(i64Bytes(tempoFromBpm(120.0)) == hex("00 00 00 00 00 07 A1 20"));
            expectEquals(tempoFromBpm(128.0), (Micros) 468750);
            expect(i64Bytes(tempoFromBpm(128.0)) == hex("00 00 00 00 00 07 27 0E"));
            expectEquals(tempoFromBpm(140.0), (Micros) 428571);
            expectEquals(tempoFromBpm(20.0), (Micros) 3000000);
            expectEquals(tempoFromBpm(999.0), (Micros) 60060);
            expectEquals(tempoFromWire(60060), (Micros) 60060, "999 BPM reads back clamped to 999 and encodes the same");
            expectWithinAbsoluteError(clampBpm(bpmFromTempo(60060)), 999.0, 1.0e-12);
            expectEquals(tempoFromBpm(1000.0), (Micros) 60060, "1000 BPM clamps to 999");
            expectEquals(tempoFromBpm(10.0), (Micros) 3000000, "10 BPM clamps to 20");
            expectEquals(tempoFromWire(0), (Micros) 0);
            expectEquals(tempoFromWire(-5), (Micros) 0);
            expectEquals(tempoFromWire(10), (Micros) 60060, "a wire tempo over 999 BPM clamps");
            expectEquals(tempoFromWire(468750), (Micros) 468750);
            expectEquals(tempoFromWire(428571), (Micros) 428571);
        }

        beginTest("beats_and_times");
        {
            expectEquals(toMicroBeats(16.5), (MicroBeats) 16500000);
            expect(i64Bytes(16500000) == hex("00 00 00 00 00 FB C5 20"));
            expect(i64Bytes(-7) == hex("FF FF FF FF FF FF FF F9"));
            expectEquals(readI64(hex("FF FF FF FF FF FF FF F9").data()), (std::int64_t) -7);

            const Timeline t { 500000, 2000000, 1000000 };
            expectEquals(t.beatAt(1750000), (MicroBeats) 3500000);
            expectEquals(t.timeAt(5000000), (Micros) 2500000);

            // Round trips: a time is whole microseconds, so beat to time to beat is exact to half a microsecond's
            // worth of beats (8 micro-beats at 999 BPM), and time to beat to time to half a micro-beat's worth of
            // time (1.5 us at 20 BPM), each plus the last rounding.
            int worstBeat = 0, worstTime = 0;
            bool allWithin = true;
            for (const double bpm : { 20.0, 123.0, 128.0, 999.0 })
            {
                const Micros m = tempoFromBpm(bpm);
                const Timeline line { m, 3333333, -987654 };
                const MicroBeats beatSlack = 1 + static_cast<MicroBeats>(std::ceil(0.5e6 / static_cast<double>(m)));
                const Micros timeSlack = 1 + static_cast<Micros>(std::ceil(0.5e-6 * static_cast<double>(m)));
                for (double beats = -5.0; beats <= 1.0e8; beats = beats < 1.0 ? beats + 0.37 : beats * 1.618)
                {
                    const MicroBeats b = toMicroBeats(beats);
                    const MicroBeats back = line.beatAt(line.timeAt(b));
                    const auto beatError = std::abs(back - b);
                    worstBeat = std::max(worstBeat, (int) beatError);
                    allWithin = allWithin && beatError <= beatSlack;
                    const Micros s = line.timeAt(b) + 17;
                    const auto timeError = std::abs(line.timeAt(line.beatAt(s)) - s);
                    worstTime = std::max(worstTime, (int) timeError);
                    allWithin = allWithin && timeError <= timeSlack;
                }
            }
            expect(allWithin, "round trips within the rounding of one step");
            logMessage("round trips: worst " + juce::String(worstBeat) + " micro-beats, " + juce::String(worstTime) + " us");
        }

        beginTest("phase_and_quantum");
        {
            constexpr MicroBeats beat = kMicroBeatsPerBeat, q = 4 * beat;
            expectEquals(phase(9 * beat, q), beat);
            expectEquals(phase(-1 * beat, q), 3 * beat);
            expectEquals(phase(-4 * beat, q), (MicroBeats) 0);
            expectEquals(phase(123456789, 0), (MicroBeats) 0);

            expectEquals(snapToQuantum(0, q), (MicroBeats) 0);
            expectEquals(snapToQuantum(1, q), (MicroBeats) 0);
            expectEquals(snapToQuantum(-1, q), (MicroBeats) 0);
            expectEquals(snapToQuantum(2 * beat, q), (MicroBeats) 0, "a tie goes down");
            expectEquals(snapToQuantum(-2 * beat, q), -4 * beat, "a tie goes down");
            expectEquals(snapToQuantum(2 * beat + 1, q), 4 * beat);
            expectEquals(snapToQuantum(toMicroBeats(5.3), q), 4 * beat);
            expectEquals(snapToQuantum(toMicroBeats(-2.5), q), -4 * beat);
            expectEquals(snapToQuantum(toMicroBeats(1.7), 0), toMicroBeats(1.7), "a quantum of zero never snaps");

            // The app-beat example: session beat 2.5, the app's free count 13.5 (offset 11): app beat 14.5.
            const MicroBeats session = toMicroBeats(2.5);
            const MicroBeats offset = snapToQuantum(toMicroBeats(13.5) - session, q);
            expectEquals(offset, 12 * beat);
            expectEquals(session + offset, toMicroBeats(14.5));
            expectEquals(phase(session + offset, q), session);

            expectEquals(quantizedStart(toMicroBeats(13.7), q), 16 * beat);
            expectEquals(quantizedStart(16 * beat, q), 16 * beat);
            expectEquals(quantizedStart(toMicroBeats(-1.5), q), (MicroBeats) 0);
            expectEquals(quantizedStart(toMicroBeats(-4.5), q), -4 * beat);
        }

        beginTest("tempo_change_rule");
        {
            const Timeline before { 500000, 8000000, 10000000 };
            const Timeline after = changeTempo(before, tempoFromBpm(140.0), 12345678);
            expect(after == Timeline { 428571, 12691356, 12345678 });
            expectEquals(after.beatAt(12345678), before.beatAt(12345678), "continuous at the change");
            expectEquals(before.beatAt(12345678 + 500000), (MicroBeats) 13691356);
            expectEquals(after.beatAt(12345678 + 500000), (MicroBeats) 13858024);

            const Timeline again = changeTempo(after, tempoFromBpm(141.0), 12345679);
            expect(again.beatOrigin >= after.beatOrigin + kMicroBeatsPerBeat, "a second change a moment later is a beat higher");
            expect(std::abs(again.beatAt(12345679) - after.beatAt(12345679)) <= 1, "and continuous");
        }
    }
};

class SyncWireTests final : public juce::UnitTest
{
public:
    SyncWireTests() : juce::UnitTest("Beat sync wire", "Sync") {}

    void runTest() override
    {
        beginTest("the_announce_of_the_examples");
        {
            auto d = std::make_unique<Datagram>();
            expect(buildDiscovery(*d, DiscoveryType::announce, idOf("ABCDEFGH"), exampleState()));
            expectEquals((int) d->size, 107);
            expect(bytesOf(*d) == hex(exampleAnnounce));

            const auto m = readDiscovery(d->view());
            expect(m.has_value());
            if (m)
            {
                expect(m->type == DiscoveryType::announce);
                expectEquals((int) m->timeToLive, 5);
                expect(m->sender == idOf("ABCDEFGH"));
                expect(m->state.sessionId == idOf("ABCDEFGH"));
                expect(m->state.timeline == Timeline { 500000, 0, 0 });
                expect(m->state.playState == PlayState {});
                expect(m->state.measurement.has_value() && *m->state.measurement == Endpoint4 { 0xC0A80114u, 50123 });
            }

            expect(buildDiscovery(*d, DiscoveryType::reply, idOf("ABCDEFGH"), exampleState()));
            auto reply = hex(exampleAnnounce);
            reply[8] = 2;
            expect(bytesOf(*d) == reply);
            const auto r = readDiscovery(d->view());
            expect(r.has_value() && r->type == DiscoveryType::reply);
        }

        beginTest("timeline_and_play_state_entries");
        {
            auto d = std::make_unique<Datagram>();
            PeerState s = exampleState();
            s.timeline = { 468750, 16500000, 90000000 };
            s.playState = { true, 16500000, 90000123 };
            expect(buildDiscovery(*d, DiscoveryType::announce, idOf("ABCDEFGH"), s));
            const auto bytes = bytesOf(*d);
            const auto tmln = hex("74 6D 6C 6E 00 00 00 18 00 00 00 00 00 07 27 0E 00 00 00 00 00 FB C5 20 00 00 00 00 05 5D 4A 80");
            const auto stst = hex("73 74 73 74 00 00 00 11 01 00 00 00 00 00 FB C5 20 00 00 00 00 05 5D 4A FB");
            expect(std::equal(tmln.begin(), tmln.end(), bytes.begin() + 20));
            expect(std::equal(stst.begin(), stst.end(), bytes.begin() + 20 + 32 + 16));
            const auto m = readDiscovery(d->view());
            expect(m.has_value() && m->state.timeline == s.timeline && m->state.playState == s.playState);
        }

        beginTest("the_leaving_message");
        {
            auto d = std::make_unique<Datagram>();
            expect(buildLeaving(*d, idOf("ABCDEFGH")));
            expect(bytesOf(*d) == hex("5F 61 73 64 70 5F 76 01 03 00 00 00 41 42 43 44 45 46 47 48"));
            const auto m = readDiscovery(d->view());
            expect(m.has_value() && m->type == DiscoveryType::leaving && m->sender == idOf("ABCDEFGH"));
        }

        beginTest("ping_pong_and_second_ping");
        {
            auto d = std::make_unique<Datagram>();
            expect(buildPing(*d, 1000000, std::nullopt));
            const auto ping = hex("5F 6C 69 6E 6B 5F 76 01 01 5F 5F 68 74 00 00 00 08 00 00 00 00 00 0F 42 40");
            expectEquals((int) d->size, 25);
            expect(bytesOf(*d) == ping);

            const auto p = readMeasurement(d->view());
            expect(p.has_value() && p->type == MeasurementType::ping && p->hostTime == Micros { 1000000 });
            expect(p.has_value() && ! p->prevSessionTime.has_value() && p->payload.size() == 16);

            auto pong = std::make_unique<Datagram>();
            expect(buildPong(*pong, idOf("ABCDEFGH"), 5000250, p->payload));
            expectEquals((int) pong->size, 57);
            expect(bytesOf(*pong) == hex("5F 6C 69 6E 6B 5F 76 01 02 73 65 73 73 00 00 00 08 41 42 43 44 45 46 47 48"
                                         "5F 5F 67 74 00 00 00 08 00 00 00 00 00 4C 4C 3A"
                                         "5F 5F 68 74 00 00 00 08 00 00 00 00 00 0F 42 40"));
            const auto q = readMeasurement(pong->view());
            expect(q.has_value() && q->type == MeasurementType::pong);
            expect(q.has_value() && q->sessionId == idOf("ABCDEFGH") && q->sessionTime == Micros { 5000250 }
                   && q->hostTime == Micros { 1000000 } && ! q->prevSessionTime.has_value());

            expect(buildPing(*d, 1000500, Micros { 5000250 }));
            expectEquals((int) d->size, 41);
            expect(bytesOf(*d) == hex("5F 6C 69 6E 6B 5F 76 01 01 5F 5F 68 74 00 00 00 08 00 00 00 00 00 0F 44 34"
                                      "5F 70 67 74 00 00 00 08 00 00 00 00 00 4C 4C 3A"));

            // A pong echoes a 32-byte ping payload byte for byte.
            const auto p2 = readMeasurement(d->view());
            expect(p2.has_value() && p2->payload.size() == 32 && p2->prevSessionTime == Micros { 5000250 });
            expect(buildPong(*pong, idOf("ABCDEFGH"), 5000760, p2->payload));
            expectEquals((int) pong->size, 9 + 32 + 32);
            expect(std::equal(p2->payload.begin(), p2->payload.end(), pong->bytes.begin() + 41));
        }

        beginTest("reading_drops_and_ignores");
        {
            const auto good = hex(exampleAnnounce);
            auto read = [](const std::vector<std::uint8_t>& b) { return readDiscovery({ b.data(), b.size() }); };
            expect(read(good).has_value());

            // An unknown entry is skipped by its size.
            {
                auto b = good;
                const auto unknown = hex("61 65 70 34 00 00 00 06 0A 00 00 01 12 34");
                b.insert(b.begin() + 20, unknown.begin(), unknown.end());
                const auto m = read(b);
                expect(m.has_value() && m->state.timeline == Timeline { 500000, 0, 0 }, "an unknown entry is skipped");
            }
            // An entry whose size runs past the end drops the datagram.
            {
                auto b = good;
                b[b.size() - 14 + 7] = 0x07; // mep4's size 6 -> 7
                expect(! read(b).has_value(), "an entry past the end drops it");
            }
            // A sess of 4 bytes drops it.
            {
                auto b = hex("5F 61 73 64 70 5F 76 01 01 05 00 00 41 42 43 44 45 46 47 48"
                             "74 6D 6C 6E 00 00 00 18 00 00 00 00 00 07 A1 20"
                             "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
                             "73 65 73 73 00 00 00 04 41 42 43 44");
                expect(! read(b).has_value(), "a sess of 4 bytes drops it");
            }
            {
                auto b = good;
                b[11] = 1;
                expect(! read(b).has_value(), "a group of 1 is ignored");
            }
            {
                auto b = good;
                b[3] = 0x65;
                expect(! read(b).has_value(), "a wrong magic is ignored");
            }
            {
                auto b = hex("5F 61 73 64 70 5F 76 01 03 00 00 00 41 42 43 44 45 46 47");
                expectEquals((int) b.size(), 19);
                expect(! read(b).has_value(), "19 bytes are ignored");
            }
            {
                auto b = good;
                b[8] = 4;
                expect(! read(b).has_value(), "an unknown type is ignored");
            }
            // Without tmln, without sess: ignored.
            {
                auto b = good;
                b.erase(b.begin() + 20, b.begin() + 52);
                expect(! read(b).has_value(), "an announce without tmln is ignored");
                auto c = good;
                c.erase(c.begin() + 52, c.begin() + 68);
                expect(! read(c).has_value(), "an announce without sess is ignored");
            }
            // A tempo of 0 or -5 makes it invalid.
            for (const std::int64_t tempo : { (std::int64_t) 0, (std::int64_t) -5 })
            {
                auto d = std::make_unique<Datagram>();
                auto s = exampleState();
                s.timeline.tempo = tempo;
                expect(buildDiscovery(*d, DiscoveryType::announce, idOf("ABCDEFGH"), s));
                expect(! readDiscovery(d->view()).has_value(), "a tempo of zero or below is invalid");
            }
            // A tempo over 999 BPM reads clamped, anchor kept.
            {
                auto d = std::make_unique<Datagram>();
                auto s = exampleState();
                s.timeline = { 50000, 7000000, 123 };
                expect(buildDiscovery(*d, DiscoveryType::announce, idOf("ABCDEFGH"), s));
                const auto m = readDiscovery(d->view());
                expect(m.has_value() && m->state.timeline == Timeline { 60060, 7000000, 123 });
            }
            // A measurement entry of the wrong size drops it; a wrong measurement type is ignored.
            {
                auto b = hex("5F 6C 69 6E 6B 5F 76 01 01 5F 5F 68 74 00 00 00 07 00 00 00 00 00 0F 42");
                expect(! readMeasurement({ b.data(), b.size() }).has_value());
                auto c = hex("5F 6C 69 6E 6B 5F 76 01 03 5F 5F 68 74 00 00 00 08 00 00 00 00 00 0F 42 40");
                expect(! readMeasurement({ c.data(), c.size() }).has_value());
            }
            // 512 bytes or more are dropped on reading.
            {
                std::vector<std::uint8_t> big(512, 0);
                std::copy(good.begin(), good.end(), big.begin());
                expect(! read(big).has_value());
            }
        }

        beginTest("building_512_bytes_or_more_fails");
        {
            auto d = std::make_unique<Datagram>();
            std::vector<std::uint8_t> filler(511, 0xAB);
            {
                Writer w(*d);
                w.raw(filler.data(), 511);
                expect(w.ok(), "511 bytes fit");
            }
            {
                Writer w(*d);
                w.raw(filler.data(), 511);
                w.u8(1);
                expect(! w.ok(), "512 bytes do not");
            }
            std::vector<std::uint8_t> payload(500, 0);
            expect(! buildPong(*d, idOf("ABCDEFGH"), 1, { payload.data(), payload.size() }), "a pong past 511 bytes fails");
        }
    }
};

static SyncMathTests syncMathTests;
static SyncWireTests syncWireTests;
