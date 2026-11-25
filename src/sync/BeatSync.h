#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "sync/SyncCore.h"
#include "sync/SyncHandoff.h"

/*
    What an app sees of the beat sync (CoolSynth's D361, docs/research/beat-sync-protocol.md, "Our design"): a tempo,
    beat grid and play state shared with every program on the local network that speaks the protocol, DAWs among
    them. BeatSync lives on the message thread (switch on and off, options, status); BeatSyncAudio on the audio thread
    (one call per block, and tempo, play and stop), with no locks, no allocation and no system calls.

    A block's beats are those at the moment its first sample is heard: host time of the callback (filtered by
    BlockClock) plus the output latency. The app's beat is the session's beat plus a whole number of quanta, so peers
    with the same quantum share bar lines; joining a session moves the app's beat by at most half a quantum.

    The apps' defaults (the user's picks): the sync on, start/stop sharing on, a quantum of one bar of 4 beats; while
    the sync has peers it wins over incoming MIDI clock (BlockBeats::connected, BeatSync::peerCount).
*/
namespace coolsynth::sync
{
    class SyncNode;

    /** Everything at the moment the block's first sample is heard. */
    struct BlockBeats
    {
        bool connected = false;      // another peer is in our session
        int peers = 0;
        double bpm = 120.0;
        double beat = 0.0;           // the app's beat at sample 0
        double beatsPerSample = 0.0; // constant over the block
        double phase = 0.0;          // of the beat at sample 0, in [0, quantum)
        bool playing = false;        // the app's transport runs (counting in while beat < 0)
        int stopSample = -1;         // when it stops inside this block: the sample it stops at; playing until then
    };

    class BeatSyncAudio
    {
    public:
        BeatSyncAudio(Handoff& handoff, double initialBpm, Micros hostNow) noexcept;

        /** Once per block, first: the host time its first sample is heard, its length, and the real microseconds per
            sample (BlockClock's slope). Takes the newest snapshot and follows a start or stop from another peer. */
        const BlockBeats& beginBlock(Micros firstSampleHeardAtHostMicros, int numSamples, double microsPerSample) noexcept;

        /** The quantum in beats, 4 for one bar of 4/4; 0 for none. The app's beat moves to the nearest multiple. */
        void setQuantum(double beats) noexcept;
        /** A new tempo from this block's first sample; nothing if it is the tempo already. */
        const BlockBeats& setTempo(double bpm) noexcept;
        /** Play from this sample of the block: with other peers, a quantized start (the block's beat goes negative
            while counting in to the next quantum); alone, at once, the grid shifted by at most half a quantum so a
            quantum starts here. With start/stop sharing on, the session starts too. */
        const BlockBeats& play(int sampleOffset) noexcept;
        /** Stop at this sample of the block; with start/stop sharing on, the session stops too. */
        const BlockBeats& stop(int sampleOffset) noexcept;

        [[nodiscard]] const BlockBeats& current() const noexcept { return block; }
        [[nodiscard]] double quantum() const noexcept { return toBeats(quantumBeats); }
        [[nodiscard]] bool isPlaying() const noexcept { return playing; }

    private:
        [[nodiscard]] double appBeatAt(Micros host) const noexcept;
        [[nodiscard]] Micros hostAtSample(int sampleOffset) const noexcept;
        void adopt(const Snapshot& s) noexcept;
        void follow(const PlayState& state) noexcept;
        void publishLocal() noexcept;
        void postGrid(MicroBeats beat, Micros hostTime) noexcept;
        void postPlay(bool start, Micros hostTime, MicroBeats beat) noexcept;
        void compute(bool smooth) noexcept;

        Handoff& handoff;
        Timeline line {};
        Micros offset = 0;
        MicroBeats appOffset = 0;
        MicroBeats quantumBeats = 4 * kMicroBeatsPerBeat;
        std::int64_t epoch = 0;

        std::int64_t snapshotNumber = 0, playChangeSeen = 0;
        bool playChangeBaseline = false;
        int peers = 0;
        std::int64_t gridPosted = 0, playPosted = 0;

        bool playing = false;
        std::optional<Micros> stopAtHost;
        bool jumped = false;

        Micros blockHost = 0;
        int blockSamples = 0;
        double microsPerSample = 1.0e6 / 48000.0;
        bool havePrevious = false;
        double previousEnd = 0.0;
        BlockBeats block {};
    };

    class BeatSync
    {
    public:
        static constexpr bool kDefaultEnabled = true;
        static constexpr bool kDefaultStartStopSharing = true;
        static constexpr double kDefaultQuantum = 4.0;

        struct Options
        {
            /** Only peers on this computer (the loopback gateway); for tests. */
            bool loopbackOnly = false;
            /** Protocol events (founded, joined, measured), called on the network thread. */
            SyncCore::Logger logger;
        };

        struct Status
        {
            bool enabled = false;
            int peers = 0;
            int gateways = 0;
            double bpm = 120.0;
            bool sessionPlaying = false;
            PeerId session {};
        };

        explicit BeatSync(double initialBpm, Options options = {});
        ~BeatSync();
        BeatSync(const BeatSync&) = delete;
        BeatSync& operator=(const BeatSync&) = delete;

        /** Switching on founds a session of our own and meets the others; switching off says goodbye and waits for
            the network thread to end. Message thread. */
        void setEnabled(bool on);
        [[nodiscard]] bool isEnabled() const noexcept { return node != nullptr; }
        void setStartStopSharing(bool on) noexcept;
        [[nodiscard]] bool startStopSharing() const noexcept;
        /** Other peers in our session; any thread but the audio thread, which has BlockBeats::peers. */
        [[nodiscard]] int peerCount() const noexcept;
        [[nodiscard]] Status status() const noexcept;

        [[nodiscard]] BeatSyncAudio& audio() noexcept { return *audioSide; }

    private:
        Options options;
        std::unique_ptr<Handoff> handoff;
        std::unique_ptr<BeatSyncAudio> audioSide;
        std::unique_ptr<SyncNode> node;
    };
}
