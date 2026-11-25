#include "sync/BeatSync.h"

#include <algorithm>
#include <cmath>

#include "sync/SyncClock.h"
#include "sync/SyncNode.h"

namespace coolsynth::sync
{
    //==================================================================================================================
    BeatSyncAudio::BeatSyncAudio(Handoff& h, double initialBpm, Micros hostNow) noexcept : handoff(h)
    {
        // Session time equals host time until a session is founded; the app's beat is 0 now.
        line = { tempoFromBpm(initialBpm), 0, hostNow };
        blockHost = hostNow;
        publishLocal();
        compute(false);
    }

    double BeatSyncAudio::appBeatAt(Micros host) const noexcept
    {
        return line.beatAtPrecise(host + offset) + toBeats(appOffset);
    }

    Micros BeatSyncAudio::hostAtSample(int sampleOffset) const noexcept
    {
        return blockHost + std::llround(static_cast<double>(std::max(0, sampleOffset)) * microsPerSample);
    }

    void BeatSyncAudio::publishLocal() noexcept
    {
        handoff.local.write(encode(LocalGrid { line.tempo, line.beatOrigin + appOffset, line.timeOrigin - offset }));
    }

    void BeatSyncAudio::postGrid(MicroBeats beat, Micros hostTime) noexcept
    {
        publishLocal(); // first: a proposal the network thread sees is already in the grid it founds from
        handoff.grid.write(encode(GridProposal { ++gridPosted, line.tempo, beat, hostTime, epoch }));
    }

    void BeatSyncAudio::postPlay(bool start, Micros hostTime, MicroBeats beat) noexcept
    {
        handoff.play.write(encode(PlayProposal { ++playPosted, start, hostTime, beat, epoch }));
    }

    void BeatSyncAudio::adopt(const Snapshot& s) noexcept
    {
        const double before = appBeatAt(blockHost);
        line = s.timeline;
        offset = s.clockOffset;
        epoch = s.epoch;
        // The app's beat continuous at this block's start, then on the session's phase: at most half a quantum away.
        const double session = line.beatAtPrecise(blockHost + offset);
        appOffset = snapToQuantum(toMicroBeats(before - session), quantumBeats);
        publishLocal();
    }

    void BeatSyncAudio::follow(const PlayState& state) noexcept
    {
        if (state.playing)
        {
            // Each peer starts at its own next quantum boundary at or after the press; its beat 0 falls there.
            appOffset = -quantizedStart(state.beat, quantumBeats);
            playing = true;
            stopAtHost.reset();
            jumped = true;
            publishLocal();
            return;
        }
        const Micros at = line.timeAt(state.beat) - offset;
        if (at <= blockHost)
        {
            playing = false;
            stopAtHost.reset();
        }
        else
            stopAtHost = at;
    }

    const BlockBeats& BeatSyncAudio::beginBlock(Micros firstSampleHeardAtHostMicros, int numSamples, double usPerSample) noexcept
    {
        blockHost = firstSampleHeardAtHostMicros;
        blockSamples = std::max(1, numSamples);
        if (usPerSample > 0.0)
            microsPerSample = usPerSample;

        SeqLock<kSnapshotWords>::Words words {};
        if (handoff.snapshot.read(words, 2))
        {
            const Snapshot s = decodeSnapshot(words);
            if (s.number > snapshotNumber)
            {
                snapshotNumber = s.number;
                peers = s.peers;
                // Our own proposals stand until a snapshot says they were applied, so a snapshot made before the
                // network thread saw a change cannot undo it.
                const bool gridPending = gridPosted > s.gridApplied;
                const bool playPending = playPosted > s.playApplied;
                if (! gridPending)
                {
                    adopt(s);
                    if (! playChangeBaseline)
                    {
                        playChangeBaseline = true;
                        playChangeSeen = s.playChange;
                    }
                    else if (s.playChange != playChangeSeen)
                    {
                        playChangeSeen = s.playChange;
                        if (! playPending && handoff.startStopSharing.load(std::memory_order_relaxed))
                            follow(s.playState);
                    }
                }
            }
        }

        block.stopSample = -1;
        if (stopAtHost)
        {
            const double into = static_cast<double>(*stopAtHost - blockHost) / microsPerSample;
            if (into <= 0.0)
            {
                playing = false;
                stopAtHost.reset();
            }
            else if (into < blockSamples)
                block.stopSample = static_cast<int>(std::llround(into));
        }
        compute(true);
        return block;
    }

    void BeatSyncAudio::compute(bool smooth) noexcept
    {
        const double tempo = static_cast<double>(line.tempo);
        double start = appBeatAt(blockHost);
        double perSample = microsPerSample / tempo;
        // Successive blocks are re-anchored from the filtered clock, so a block's start can differ from the last
        // one's end by a few microseconds: under a millisecond's worth, the start is the last end and the block's
        // rate takes up the difference (at most a tenth of its length per block), so no step plays twice or is lost.
        if (smooth && ! jumped && havePrevious)
        {
            const double difference = start - previousEnd;
            if (std::abs(difference) < 1000.0 / tempo)
            {
                const double length = perSample * blockSamples;
                const double taken = std::clamp(difference, -0.1 * length, 0.1 * length);
                start = previousEnd;
                perSample += taken / blockSamples;
            }
        }
        jumped = false;
        block.connected = peers > 0;
        block.peers = peers;
        block.bpm = bpmFromTempo(line.tempo);
        block.beat = start;
        block.beatsPerSample = perSample;
        const double q = toBeats(quantumBeats);
        block.phase = q > 0.0 ? start - q * std::floor(start / q) : 0.0;
        block.playing = playing;
        previousEnd = start + perSample * blockSamples;
        havePrevious = true;
    }

    void BeatSyncAudio::setQuantum(double beats) noexcept
    {
        quantumBeats = std::max<MicroBeats>(0, toMicroBeats(beats));
        appOffset = snapToQuantum(appOffset, quantumBeats);
        publishLocal();
    }

    const BlockBeats& BeatSyncAudio::setTempo(double bpm) noexcept
    {
        const Micros tempo = tempoFromBpm(bpm);
        if (tempo == line.tempo)
            return block;
        const Micros at = blockHost + offset;
        const MicroBeats beat = line.beatAt(at);
        line = timelineThrough(line, tempo, beat, at);
        postGrid(beat, blockHost);
        const double start = block.beat;
        compute(false);
        block.beat = start; // continuous at the block's start
        previousEnd = start + block.beatsPerSample * blockSamples;
        return block;
    }

    const BlockBeats& BeatSyncAudio::play(int sampleOffset) noexcept
    {
        const Micros at = hostAtSample(sampleOffset);
        const MicroBeats pressed = line.beatAt(at + offset);
        MicroBeats startBeat = pressed;
        if (peers == 0)
        {
            // Alone: at once. The grid shifts by the least amount (at most half a quantum) that puts a quantum
            // boundary at the press, as a timeline change through that boundary at that moment.
            startBeat = snapToQuantum(pressed, quantumBeats);
            if (startBeat != pressed)
            {
                line = timelineThrough(line, line.tempo, startBeat, at + offset);
                postGrid(startBeat, at);
            }
            appOffset = -startBeat;
        }
        else
            appOffset = -quantizedStart(pressed, quantumBeats);
        playing = true;
        stopAtHost.reset();
        jumped = true;
        publishLocal();
        if (handoff.startStopSharing.load(std::memory_order_relaxed))
            postPlay(true, at, startBeat);
        block.stopSample = -1;
        compute(false);
        return block;
    }

    const BlockBeats& BeatSyncAudio::stop(int sampleOffset) noexcept
    {
        const Micros at = hostAtSample(sampleOffset);
        if (handoff.startStopSharing.load(std::memory_order_relaxed))
            postPlay(false, at, line.beatAt(at + offset));
        if (sampleOffset <= 0)
        {
            playing = false;
            stopAtHost.reset();
            block.stopSample = -1;
            block.playing = false;
        }
        else if (playing)
        {
            stopAtHost = at;
            block.stopSample = std::min(sampleOffset, blockSamples);
        }
        return block;
    }

    //==================================================================================================================
    BeatSync::BeatSync(double initialBpm, Options o)
        : options(std::move(o)),
          handoff(std::make_unique<Handoff>()),
          audioSide(std::make_unique<BeatSyncAudio>(*handoff, initialBpm, hostMicrosNow()))
    {
        handoff->startStopSharing.store(kDefaultStartStopSharing);
        audioSide->setQuantum(kDefaultQuantum);
    }

    BeatSync::~BeatSync() { setEnabled(false); }

    void BeatSync::setEnabled(bool on)
    {
        if (on == isEnabled())
            return;
        if (on)
        {
            SyncNode::Options nodeOptions;
            nodeOptions.loopbackOnly = options.loopbackOnly;
            nodeOptions.logger = options.logger;
            node = std::make_unique<SyncNode>(*handoff, std::move(nodeOptions));
            node->start();
        }
        else
        {
            node->stop();
            node.reset();
        }
    }

    void BeatSync::setStartStopSharing(bool on) noexcept
    {
        handoff->startStopSharing.store(on, std::memory_order_release);
    }

    bool BeatSync::startStopSharing() const noexcept
    {
        return handoff->startStopSharing.load(std::memory_order_acquire);
    }

    int BeatSync::peerCount() const noexcept
    {
        return isEnabled() ? handoff->readSnapshot().peers : 0;
    }

    BeatSync::Status BeatSync::status() const noexcept
    {
        const Snapshot s = handoff->readSnapshot();
        Status status;
        status.enabled = isEnabled();
        status.peers = status.enabled ? s.peers : 0;
        status.gateways = node != nullptr ? node->openGateways() : 0;
        SeqLock<kLocalWords>::Words local {};
        while (! handoff->local.read(local)) {}
        status.bpm = bpmFromTempo(decodeLocal(local).tempo); // the app's own grid: right also before any session
        status.sessionPlaying = s.playState.playing;
        status.session = s.session;
        return status;
    }
}
