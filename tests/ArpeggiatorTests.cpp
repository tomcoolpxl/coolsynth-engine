#include <juce_core/juce_core.h>

#include <array>
#include <vector>

#include "synth/Arpeggiator.h"

/*
    The arpeggiator's line under Latch (CoolSynth's D373): every key press is a step, the same key again too, up to
    64; As Played plays the line as pressed, the other patterns its notes once each. On its own clock, 120 BPM,
    sixteenths.
*/
namespace
{
    using namespace coolsynth::synth;

    struct ArpRig
    {
        Arpeggiator arp;
        ArpParametersV2 settings;

        ArpRig(bool latch, coolsynth::parameters::ArpPatternChoice pattern)
        {
            arp.prepare(48000.0);
            settings.enabled = true;
            settings.latch = latch;
            settings.pattern = pattern;
            arp.setParameters(settings);
        }

        /** The next `count` notes the arp starts. */
        std::vector<int> next(int count)
        {
            std::vector<int> notes;
            std::array<EngineMidiEvent, maxArpEventsPerBlock> events {};
            for (int block = 0; block < 4000 && static_cast<int>(notes.size()) < count; ++block)
            {
                const int written = arp.generateEventsForBlock(256, 48000.0, events.data(), static_cast<int>(events.size()));
                for (int i = 0; i < written && static_cast<int>(notes.size()) < count; ++i)
                    if (events[static_cast<std::size_t>(i)].type == EngineMidiEventType::noteOn)
                        notes.push_back(events[static_cast<std::size_t>(i)].noteNumber);
            }
            return notes;
        }
    };

    juce::String text(const std::vector<int>& notes)
    {
        juce::StringArray parts;
        for (const int n : notes)
            parts.add(juce::String(n));
        return parts.joinIntoString(" ");
    }
}

class ArpeggiatorTests final : public juce::UnitTest
{
public:
    ArpeggiatorTests() : juce::UnitTest("Arpeggiator", "Arpeggiator") {}

    void runTest() override
    {
        using Pattern = coolsynth::parameters::ArpPatternChoice;

        beginTest("under_latch_every_press_is_a_step_the_same_key_too");
        {
            ArpRig rig(true, Pattern::asPlayed);
            rig.arp.onNoteOn(60, 0.8f);
            rig.arp.onNoteOn(64, 0.8f);
            rig.arp.onNoteOff(60);
            rig.arp.onNoteOn(60, 0.8f); // again, with E still down
            rig.arp.onNoteOn(67, 0.8f);
            rig.arp.onNoteOff(60);
            rig.arp.onNoteOff(64);
            rig.arp.onNoteOff(67);
            expectEquals(rig.arp.getLatchedNoteCountForTesting(), 4);
            expectEquals(text(rig.next(8)), juce::String("60 64 60 67 60 64 60 67"));
        }

        beginTest("a_note_on_for_a_key_that_is_down_is_not_a_new_step");
        {
            ArpRig rig(true, Pattern::asPlayed);
            rig.arp.onNoteOn(60, 0.8f);
            rig.arp.onNoteOn(60, 0.5f); // the same press, sent twice
            rig.arp.onNoteOn(64, 0.8f);
            expectEquals(rig.arp.getLatchedNoteCountForTesting(), 2);
            expectEquals(text(rig.next(4)), juce::String("60 64 60 64"));
        }

        beginTest("the_other_patterns_play_each_note_of_the_line_once");
        {
            ArpRig rig(true, Pattern::up);
            rig.arp.onNoteOn(64, 0.8f);
            rig.arp.onNoteOn(60, 0.8f);
            rig.arp.onNoteOff(64);
            rig.arp.onNoteOn(64, 0.8f);
            rig.arp.onNoteOn(67, 0.8f);
            expectEquals(text(rig.next(6)), juce::String("60 64 67 60 64 67"));
        }

        beginTest("the_line_holds_64_presses_and_drops_its_oldest_after");
        {
            ArpRig rig(true, Pattern::asPlayed);
            rig.arp.onNoteOn(36, 0.8f); // kept down throughout
            for (int press = 0; press < 70; ++press)
            {
                rig.arp.onNoteOn(60 + press % 2, 0.8f);
                rig.arp.onNoteOff(60 + press % 2);
            }
            expectEquals(rig.arp.getLatchedNoteCountForTesting(), 64);

            // Up, Down and the rest still take 16 notes at most from it.
            ArpRig wide(true, Pattern::up);
            wide.arp.onNoteOn(36, 0.8f);
            for (int key = 0; key < 30; ++key)
            {
                wide.arp.onNoteOn(40 + key, 0.8f);
                wide.arp.onNoteOff(40 + key);
            }
            const auto notes = wide.next(17);
            expectEquals(notes.front(), 54, "the 16 pressed last, from the lowest");
            expectEquals(notes[15], 69);
            expectEquals(notes[16], 54, "and round again");
        }

        beginTest("after_every_key_is_up_the_next_key_starts_a_new_line");
        {
            ArpRig rig(true, Pattern::asPlayed);
            rig.arp.onNoteOn(60, 0.8f);
            rig.arp.onNoteOn(64, 0.8f);
            rig.arp.onNoteOff(60);
            rig.arp.onNoteOff(64);
            rig.arp.onNoteOn(72, 0.8f);
            expectEquals(rig.arp.getLatchedNoteCountForTesting(), 1);
            expectEquals(text(rig.next(3)), juce::String("72 72 72"));
        }

        beginTest("without_latch_as_played_follows_the_keys_that_are_down");
        {
            ArpRig rig(false, Pattern::asPlayed);
            rig.arp.onNoteOn(64, 0.8f);
            rig.arp.onNoteOn(60, 0.8f);
            expectEquals(text(rig.next(4)), juce::String("64 60 64 60"));
            rig.arp.onNoteOff(64);
            expectEquals(rig.arp.getHeldNoteCountForTesting(), 1);
        }
    }
};

static ArpeggiatorTests arpeggiatorTests;
