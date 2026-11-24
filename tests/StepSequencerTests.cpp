#include <juce_core/juce_core.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "synth/StepSequencer.h"

/*
    The step sequencer (D266): where its steps fall, swing, lanes of their own length, accent, chance, rolls, mute and
    solo, A and B, chains, fills, note-offs, recording's nearest step, and notes that do not depend on the block size.
    At 120 BPM and 48 kHz a beat is 24000 samples and a sixteenth 6000.
*/
namespace
{
    using namespace coolsynth::synth;

    constexpr double kRate = 48000.0;
    constexpr double kBpm = 120.0;
    constexpr std::int64_t kSixteenth = 6000;
    constexpr float kSoft = 1.0f / 127.0f;

    struct Note
    {
        std::int64_t sample = 0;
        int note = 0;
        float velocity = 0.0f;
        bool on = true;
        bool operator==(const Note& o) const { return sample == o.sample && note == o.note && velocity == o.velocity && on == o.on; }
    };

    struct Rig
    {
        StepSequencer sequencer;
        std::int64_t position = 0; // samples from beat 0

        void play(double beats, int block, bool playing = true, std::vector<Note>* out = nullptr)
        {
            const double beatsPerSample = kBpm / 60.0 / kRate;
            const auto end = position + static_cast<std::int64_t>(std::llround(beats / beatsPerSample));
            std::array<EngineMidiEvent, 512> events {};
            while (position < end)
            {
                const int n = static_cast<int>(std::min<std::int64_t>(block, end - position));
                const int count = sequencer.process(static_cast<double>(position) * beatsPerSample, kBpm, playing, n, kRate,
                                                    events.data(), static_cast<int>(events.size()));
                for (int i = 0; i < count; ++i)
                    if (out != nullptr)
                        out->push_back({ position + events[static_cast<std::size_t>(i)].sampleOffset,
                                         events[static_cast<std::size_t>(i)].noteNumber,
                                         events[static_cast<std::size_t>(i)].value,
                                         events[static_cast<std::size_t>(i)].type == EngineMidiEventType::noteOn });
                position += n;
            }
        }

        std::vector<Note> run(double beats, int block = 512)
        {
            std::vector<Note> notes;
            play(beats, block, true, &notes);
            return notes;
        }
    };

    std::vector<Note> ons(const std::vector<Note>& all, int note = -1)
    {
        std::vector<Note> r;
        for (const auto& n : all)
            if (n.on && (note < 0 || n.note == note))
                r.push_back(n);
        return r;
    }

    std::vector<std::int64_t> samplesOf(const std::vector<Note>& notes)
    {
        std::vector<std::int64_t> r;
        for (const auto& n : notes)
            r.push_back(n.sample);
        return r;
    }

    void lane(SequencerPattern& p, int l, int note, std::initializer_list<int> steps, bool variationB = false)
    {
        p.lanes[static_cast<std::size_t>(l)].note = note;
        for (const int s : steps)
            (variationB ? p.lanes[static_cast<std::size_t>(l)].b : p.lanes[static_cast<std::size_t>(l)].a)[static_cast<std::size_t>(s)].on = true;
    }
}

class StepSequencerTests final : public juce::UnitTest
{
public:
    StepSequencerTests() : juce::UnitTest("Step sequencer", "Sequencer") {}

    void runTest() override
    {
        beginTest("steps_fall_on_the_sixteenths");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0, 4, 8, 12 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            const auto hits = ons(rig->run(4.0));
            expect(samplesOf(hits) == std::vector<std::int64_t> { 0, 24000, 48000, 72000 });
            for (const auto& h : hits)
                expectEquals(h.velocity, kSoft);
        }

        beginTest("swing_moves_the_even_sixteenths_later");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 42, { 0, 1, 2, 3 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            SequencerSettings s;
            s.swing = 0.66f; // Linn's 66 %: the second sixteenth of each eighth at 0.66 of the eighth
            rig->sequencer.setSettings(s);
            const auto hits = samplesOf(ons(rig->run(1.0)));
            expectEquals(static_cast<int>(hits.size()), 4);
            expectEquals(hits[0], std::int64_t { 0 });
            expectWithinAbsoluteError(static_cast<double>(hits[1]), 0.66 * 12000.0, 1.0);
            expectEquals(hits[2], std::int64_t { 12000 });
            expectWithinAbsoluteError(static_cast<double>(hits[3]), 12000.0 + 0.66 * 12000.0, 1.0);
        }

        beginTest("a_short_lane_runs_on_across_the_bars");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            bank->pattern[0].lanes[0].length = 3;
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            const auto hits = samplesOf(ons(rig->run(8.0))); // two bars: every third sixteenth, across the bar line
            std::vector<std::int64_t> expected;
            for (std::int64_t k = 0; k < 32; k += 3)
                expected.push_back(k * kSixteenth);
            expect(hits == expected);
        }

        beginTest("an_accented_step_strikes_harder");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0, 4 });
            bank->pattern[0].accent.a[4].on = true;
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            const auto hits = ons(rig->run(4.0));
            expectEquals(static_cast<int>(hits.size()), 2);
            expectEquals(hits[0].velocity, kSoft);
            expectEquals(hits[1].velocity, 1.0f);
        }

        beginTest("chance_plays_as_often_as_set_and_the_same_every_play");
        {
            auto bank = std::make_unique<SequencerBank>();
            auto& l = bank->pattern[0].lanes[0];
            l.note = 42;
            for (auto& step : l.a)
            {
                step.on = true;
                step.chance = 0.5f;
            }
            auto first = std::make_unique<Rig>();
            first->sequencer.setBank(*bank);
            const auto a = ons(first->run(256.0)); // 64 bars, 1024 steps
            expectGreaterThan(static_cast<int>(a.size()), 1024 * 4 / 10);
            expectLessThan(static_cast<int>(a.size()), 1024 * 6 / 10);

            auto again = std::make_unique<Rig>();
            again->sequencer.setBank(*bank);
            expect(ons(again->run(256.0)) == a, "the same seed plays the same");

            auto other = std::make_unique<Rig>();
            other->sequencer.setBank(*bank);
            SequencerSettings s;
            s.seed = 12345u;
            other->sequencer.setSettings(s);
            expect(ons(other->run(256.0)) != a, "another seed plays otherwise");

            for (auto& step : l.a)
                step.chance = 0.0f;
            auto never = std::make_unique<Rig>();
            never->sequencer.setBank(*bank);
            expect(ons(never->run(16.0)).empty());
        }

        beginTest("a_roll_splits_the_step_evenly");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 42, { 0 });
            bank->pattern[0].lanes[0].a[0].hits = 3;
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            expect(samplesOf(ons(rig->run(1.0))) == std::vector<std::int64_t> { 0, 2000, 4000 });
        }

        beginTest("mute_silences_a_lane_and_solo_leaves_only_its_lane");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            lane(bank->pattern[0], 1, 38, { 4 });
            lane(bank->pattern[0], 2, 42, { 8 });
            bank->pattern[0].lanes[0].mute = true;
            auto muted = std::make_unique<Rig>();
            muted->sequencer.setBank(*bank);
            const auto m = ons(muted->run(4.0));
            expect(ons(m, 36).empty() && ons(m, 38).size() == 1 && ons(m, 42).size() == 1);

            bank->pattern[0].lanes[0].mute = false;
            bank->pattern[0].lanes[1].solo = true;
            auto solo = std::make_unique<Rig>();
            solo->sequencer.setBank(*bank);
            const auto s = ons(solo->run(4.0));
            expectEquals(static_cast<int>(s.size()), 1);
            expectEquals(s[0].note, 38);
        }

        beginTest("ab_plays_a_then_b");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            lane(bank->pattern[0], 0, 36, { 8 }, true);
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            SequencerSettings s;
            s.variation = SequencerVariation::ab;
            rig->sequencer.setSettings(s);
            expect(samplesOf(ons(rig->run(16.0))) == std::vector<std::int64_t> { 0, 24 * kSixteenth, 32 * kSixteenth, 56 * kSixteenth });
        }

        beginTest("a_chain_plays_its_patterns_in_a_row");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            lane(bank->pattern[1], 0, 38, { 0 });
            lane(bank->pattern[2], 0, 42, { 0 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            const int chain[] { 0, 1, 2 };
            rig->sequencer.setChain(chain, 3);
            const auto hits = ons(rig->run(16.0));
            expectEquals(static_cast<int>(hits.size()), 4);
            expect(hits[0].note == 36 && hits[1].note == 38 && hits[2].note == 42 && hits[3].note == 36);
        }

        beginTest("a_new_chain_waits_for_the_pattern_to_end");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0, 12 });
            lane(bank->pattern[1], 0, 38, { 0, 12 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            std::vector<Note> notes;
            rig->play(2.0, 256, true, &notes);
            const int next[] { 1 };
            rig->sequencer.setChain(next, 1);
            rig->play(6.0, 256, true, &notes);
            const auto hits = ons(notes);
            expectEquals(static_cast<int>(hits.size()), 4);
            expect(hits[0].note == 36 && hits[1].note == 36, "the bar that plays plays on");
            expect(hits[2].note == 38 && hits[2].sample == 16 * kSixteenth, "the new pattern from the next bar");
        }

        beginTest("fill_plays_once_in_the_next_bar");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            lane(bank->fill, 0, 39, { 0, 8 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            std::vector<Note> notes;
            rig->play(1.0, 256, true, &notes);
            rig->sequencer.requestFill();
            rig->play(11.0, 256, true, &notes);
            const auto hits = ons(notes);
            expectEquals(static_cast<int>(hits.size()), 4);
            expect(hits[0].note == 36 && hits[1].note == 39 && hits[2].note == 39 && hits[3].note == 36);
            expectEquals(hits[1].sample, 16 * kSixteenth);
        }

        beginTest("every_note_on_has_its_note_off_and_stop_sends_the_rest");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0, 4, 8, 12 });
            lane(bank->pattern[0], 1, 42, { 0, 2, 4, 6, 8, 10, 12, 14 });
            bank->pattern[0].lanes[1].a[14].hits = 3;
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            std::vector<Note> notes;
            rig->play(8.0, 333, true, &notes);
            rig->play(0.001, 333, true, &notes); // the third bar's first hits: their gates still open
            int balance = 0;
            for (const auto& n : notes)
                balance += n.on ? 1 : -1;
            expectGreaterThan(balance, 0);
            rig->play(0.01, 333, false, &notes);
            balance = 0;
            for (const auto& n : notes)
                balance += n.on ? 1 : -1;
            expectEquals(balance, 0);
            for (std::size_t i = 1; i < notes.size(); ++i)
                expect(notes[i].sample >= notes[i - 1].sample, "in time order");
        }

        beginTest("a_recorded_hit_goes_to_the_nearest_step");
        {
            auto rig = std::make_unique<Rig>();
            expectEquals(rig->sequencer.placeAt(1.06).sixteenth, std::int64_t { 4 });
            expectEquals(rig->sequencer.placeAt(0.93).sixteenth, std::int64_t { 4 });
            SequencerSettings s;
            s.swing = 0.75f; // the second sixteenth at 0.375 beats
            rig->sequencer.setSettings(s);
            expectEquals(rig->sequencer.placeAt(0.35).sixteenth, std::int64_t { 1 });
            expectEquals(rig->sequencer.placeAt(0.47).sixteenth, std::int64_t { 2 });
            expectEquals(rig->sequencer.placeAt(0.2).pattern, 0);
        }

        beginTest("a_host_that_jumps_back_is_followed");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            std::vector<Note> notes;
            rig->play(2.0, 512, true, &notes);
            rig->position = 0; // the host loops back to the start
            rig->play(1.0, 512, true, &notes);
            const auto hits = samplesOf(ons(notes));
            expect(hits == std::vector<std::int64_t> { 0, 0 });
        }

        beginTest("swing_turned_while_playing_never_doubles_or_drops_a_step");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 42, { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 });
            bank->pattern[0].lanes[0].a[5].hits = 3;
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            std::vector<Note> notes;
            const double blockBeats = 256.0 * kBpm / 60.0 / kRate;
            for (int i = 0; i < static_cast<int>(8.0 / blockBeats); ++i)
            {
                SequencerSettings s;
                s.swing = 0.5f + 0.025f * static_cast<float>((i * 7) % 11); // turned all the time, up and down
                rig->sequencer.setSettings(s);
                rig->play(blockBeats, 256, true, &notes);
            }
            const auto hits = samplesOf(ons(notes));
            expectEquals(static_cast<int>(hits.size()), 32 + 2 * 2, "each sixteenth once, the two rolls of three");
            std::int64_t closest = 1 << 30;
            for (std::size_t i = 1; i < hits.size(); ++i)
                closest = std::min(closest, hits[i] - hits[i - 1]);
            expectGreaterThan(static_cast<int>(closest), 1000, "no hit twice");
        }

        beginTest("a_fill_asked_again_while_it_plays_plays_whole");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            lane(bank->fill, 0, 39, { 0, 8 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            std::vector<Note> notes;
            rig->play(1.0, 256, true, &notes);
            rig->sequencer.requestFill();
            rig->play(4.5, 256, true, &notes); // into the fill's bar
            rig->sequencer.requestFill();
            rig->play(6.5, 256, true, &notes);
            const auto hits = ons(notes);
            expectEquals(static_cast<int>(hits.size()), 4);
            expect(hits[1].note == 39 && hits[2].note == 39 && hits[2].sample == 24 * kSixteenth, "the fill's second hit");
            expect(hits[3].note == 36 && hits[3].sample == 32 * kSixteenth, "then the pattern again");
        }

        beginTest("a_fill_asked_while_stopped_plays_nothing");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            lane(bank->fill, 0, 39, { 0 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            rig->sequencer.requestFill();
            rig->play(0.1, 256, false);
            rig->position = 0;
            for (const auto& n : ons(rig->run(8.0)))
                expectEquals(n.note, 36);
        }

        beginTest("a_chain_chosen_in_the_last_sixteenth_takes_over_at_the_next_bar");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            lane(bank->pattern[1], 0, 38, { 0 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            std::vector<Note> notes;
            rig->play(3.8, 256, true, &notes); // the look ahead has reached the next bar
            const int next[] { 1 };
            rig->sequencer.setChain(next, 1);
            rig->play(4.2, 256, true, &notes);
            const auto hits = ons(notes);
            expectEquals(static_cast<int>(hits.size()), 2);
            expect(hits[1].note == 38 && hits[1].sample == 16 * kSixteenth);
        }

        beginTest("a_chain_set_from_a_bar_begins_there_and_not_at_the_pattern_end");
        {
            // A then B: a pattern lasts two bars, and a chain from bar 1 begins mid-pattern, with its first pattern's A.
            auto bank = std::make_unique<SequencerBank>();
            for (const bool b : { false, true })
            {
                lane(bank->pattern[0], 0, 36, { 0 }, b);
                lane(bank->pattern[1], 0, 38, { 0 }, b);
                lane(bank->pattern[2], 0, 42, { 0 }, b);
            }
            SequencerSettings s;
            s.variation = SequencerVariation::ab;
            const int next[] { 1, 2 };
            for (const double before : { 1.0, 3.9 }) // in the bar, and with the look ahead already in the next one
            {
                auto rig = std::make_unique<Rig>();
                rig->sequencer.setBank(*bank);
                rig->sequencer.setSettings(s);
                std::vector<Note> notes;
                rig->play(before, 256, true, &notes);
                rig->sequencer.setChainFrom(next, 2, 1);
                rig->play(20.0 - before, 256, true, &notes);
                const auto hits = ons(notes);
                expectEquals(static_cast<int>(hits.size()), 5);
                if (hits.size() == 5)
                {
                    expect(hits[0].note == 36 && hits[1].note == 38 && hits[2].note == 38 && hits[3].note == 42
                           && hits[4].note == 42, "the old bar, then the new chain from its start");
                    expectEquals(hits[1].sample, 16 * kSixteenth);
                }
            }

            auto stopped = std::make_unique<Rig>(); // stopped, as setChain: from bar 0
            stopped->sequencer.setBank(*bank);
            stopped->sequencer.setChainFrom(next, 2, 7);
            const auto hits = ons(stopped->run(4.0));
            expect(hits.size() == 1 && hits[0].note == 38 && hits[0].sample == 0);
        }

        beginTest("with_the_loop_off_a_chain_plays_once_through_then_nothing");
        {
            // CoolSynth's D297: the chain 0, 1 plays its two bars, then the sequencer is silent and says it has ended.
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 }, false);
            lane(bank->pattern[1], 0, 38, { 0 }, false);
            SequencerSettings s;
            s.loop = false;
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            rig->sequencer.setSettings(s);
            const int chain[] { 0, 1 };
            rig->sequencer.setChain(chain, 2);
            const auto hits = ons(rig->run(16.0));
            expectEquals(static_cast<int>(hits.size()), 2);
            if (hits.size() == 2)
                expect(hits[0].note == 36 && hits[1].note == 38 && hits[1].sample == 16 * kSixteenth, "once through, in order");
            expect(! rig->sequencer.endedAt(7.9), "still in its last bar");
            expect(rig->sequencer.endedAt(8.0), "ended at its end");

            s.loop = true;
            rig->sequencer.setSettings(s);
            expect(! rig->sequencer.endedAt(8.0), "with the loop on it never ends");
        }

        beginTest("a_loop_back_to_before_a_switch_plays_the_chain_chosen");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0 });
            lane(bank->pattern[1], 0, 38, { 0 });
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            std::vector<Note> notes;
            rig->play(2.0, 512, true, &notes);
            const int next[] { 1 };
            rig->sequencer.setChain(next, 1);
            rig->play(6.0, 512, true, &notes);
            rig->position = 0; // the clock's master loops back
            rig->play(2.0, 512, true, &notes);
            const auto hits = ons(notes);
            expectEquals(static_cast<int>(hits.size()), 3);
            expect(hits[2].note == 38 && hits[2].sample == 0);
        }

        beginTest("a_crowded_block_leaves_no_note_hanging");
        {
            auto bank = std::make_unique<SequencerBank>();
            for (int l = 0; l < SequencerPattern::maxLanes; ++l)
            {
                lane(bank->pattern[0], l, 36 + l, { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 });
                for (auto& step : bank->pattern[0].lanes[static_cast<std::size_t>(l)].a)
                    step.hits = 3;
            }
            auto rig = std::make_unique<Rig>();
            rig->sequencer.setBank(*bank);
            std::vector<Note> notes;
            rig->play(4.0, 96000, true, &notes); // 768 hits in one block
            rig->play(0.1, 256, false, &notes);
            rig->play(0.1, 256, false, &notes);
            std::array<int, 128> open {};
            for (const auto& n : notes)
                open[static_cast<std::size_t>(n.note)] += n.on ? 1 : -1;
            for (int note = 0; note < 128; ++note)
                expectEquals(open[static_cast<std::size_t>(note)], 0, "note " + juce::String(note));
            expectGreaterThan(static_cast<int>(ons(notes).size()), 100);
        }

        beginTest("a_position_or_tempo_no_transport_gives_plays_nothing");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0, 4, 8, 12 });
            StepSequencer sequencer;
            sequencer.setBank(*bank);
            std::array<EngineMidiEvent, 64> events {};
            expectEquals(sequencer.process(std::nan(""), kBpm, true, 512, kRate, events.data(), 64), 0);
            expectEquals(sequencer.process(0.0, std::numeric_limits<double>::infinity(), true, 512, kRate, events.data(), 64), 0);
            expectEquals(sequencer.process(1.0e300, kBpm, true, 512, kRate, events.data(), 64), 0);
            // A tempo past any drum machine's is held to 1000 BPM: a few steps, not millions.
            const int count = sequencer.process(0.0, 1.0e9, true, 512, kRate, events.data(), 64);
            expect(count > 0 && count <= 64);
        }

        beginTest("the_same_notes_whatever_the_block_size");
        {
            auto bank = std::make_unique<SequencerBank>();
            lane(bank->pattern[0], 0, 36, { 0, 3, 7, 10 });
            lane(bank->pattern[0], 1, 42, { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 });
            lane(bank->pattern[0], 2, 38, { 4, 12 }, true);
            lane(bank->pattern[1], 0, 39, { 2, 6, 9 });
            bank->pattern[0].lanes[0].length = 7;
            bank->pattern[0].lanes[1].length = 5;
            for (auto& step : bank->pattern[0].lanes[1].a)
                step.chance = 0.6f;
            bank->pattern[0].lanes[1].a[3].hits = 3;
            bank->pattern[0].accent.a[0].on = true;
            bank->pattern[0].accent.length = 6;
            SequencerSettings s;
            s.swing = 0.62f;
            s.variation = SequencerVariation::ab;
            const int chain[] { 0, 0, 1 };

            std::vector<Note> reference;
            for (const int block : { 4096, 480, 64, 1 })
            {
                auto rig = std::make_unique<Rig>();
                rig->sequencer.setBank(*bank);
                rig->sequencer.setSettings(s);
                rig->sequencer.setChain(chain, 3);
                const auto notes = rig->run(32.0, block);
                if (reference.empty())
                {
                    reference = notes;
                    expectGreaterThan(static_cast<int>(ons(notes).size()), 50);
                }
                else if (notes != reference)
                {
                    std::size_t i = 0;
                    while (i < notes.size() && i < reference.size() && notes[i] == reference[i])
                        ++i;
                    const auto describe = [](const std::vector<Note>& v, std::size_t at)
                    {
                        if (at >= v.size())
                            return juce::String("none");
                        const auto& n = v[at];
                        return juce::String(n.on ? "on " : "off ") + juce::String(n.note) + " at " + juce::String(n.sample)
                               + " v " + juce::String(n.velocity);
                    };
                    expect(false, "block " + juce::String(block) + ": " + juce::String(static_cast<int>(notes.size())) + " notes, "
                                      + juce::String(static_cast<int>(reference.size())) + " in the reference; the first difference, note "
                                      + juce::String(static_cast<int>(i)) + ": " + describe(notes, i) + " where the reference has "
                                      + describe(reference, i));
                }
            }
        }
    }
};

static StepSequencerTests stepSequencerTests;
