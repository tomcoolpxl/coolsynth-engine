#pragma once

#include <memory>

#include "Tb303Envelope.h"
#include "Tb303Filter.h"
#include "Tb303Oscillator.h"
#include "Tb303Output.h"
#include "Tb303Vca.h"

/*
    The Roland TB-303's voice as its circuit (D275, D295): the four stages wired as tools/tb303/voice.cir wires them.

        the VCO's saw or square (the WAVEFORM switch) into the filter;
        the envelope into ENV MOD, the ACCENT knob's wiper into the filter's D24 (whose current loads the wiper, handed
        back to the envelope a step late);
        the filter's output and its RESONANCE wiper (loaded by the VCA's C22 and R122) into the VCA;
        the VCA's control current from the envelope's circuit (pin 1);
        the VCA's pin 8 into the output stage (C38, VOLUME and the mixer; CoolSynth's D362), out at the OUTPUT jack.

    At 96 kHz (D294): the square's shaper at 768 kHz inside the oscillator, the envelope near 48 kHz. The trimmers as
    page 7 sets them (TM4, TM5) and as fitted to Din Sync's unit (TM3 0.35, C11 0.82 uF; D275). In: the CPU's note as
    the ladder's volts (1 V an octave, the A key unshifted 2.75 V), the gate, the accent and the slide of each step;
    out: the OUTPUT jack's volts into a recorder (Tb303Output::recorderOhms), turned over against the VCA's as the
    mixer turns it. The VCA's own output (C38 into 100k, as before D362) is vcaOutput().
*/
namespace coolsynth::synth::v3
{
    class Tb303Voice final
    {
    public:
        static constexpr double rate = 96000.0;
        static constexpr double tm3 = 0.35, tm4 = 0.5687, tm5 = 0.7389, c11 = 0.82e-6;

        /** devilFish: the voice with Robin Whittle's Devil Fish modifications (CoolSynth's D309); off, the stock
            TB-303, part for part as before. */
        explicit Tb303Voice(bool devilFish = false)
            : osc(std::make_unique<Tb303Oscillator>()), filter(std::make_unique<Tb303Filter>(true, devilFish)),
              env(std::make_unique<Tb303Envelope>(devilFish)), vca(std::make_unique<Tb303Vca>()),
              stage(std::make_unique<Tb303Output>()), fish(devilFish)
        {
            if (fish)
                setDevilFish(fishSettings);
        }

        void prepare(double sampleRate = rate) noexcept
        {
            voiceRate = sampleRate;
            osc->prepare(sampleRate);
            filter->prepare(sampleRate);
            env->prepare(sampleRate);
            vca->prepare(sampleRate);
            stage->prepare(sampleRate);
            setKnobs(knobs);
            if (fish)
                setDevilFish(fishSettings);
        }

        [[nodiscard]] bool isDevilFish() const noexcept { return fish; }

        /** The looser solve of the 303's Light accuracy (CoolSynth's D349): every circuit to 1 uV but the oscillator's
            pitch, which stays at 10 nV (its error drifts the pitch). Alone, on a pattern of both voices: 8 % cheaper,
            -106.5 dB rms against a circuit solved to 10 pV (Best -128.6), every third-octave band within 0.05 dB of
            Best. Off: Best. The app's Light also runs the voice at the host's rate, set with prepare. */
        static constexpr double lighterTolerance = 1.0e-6;
        void setLighter(bool on) noexcept
        {
            const double volts = on ? lighterTolerance : Tb303Filter::newtonTolerance;
            osc->setShaperTolerance(volts);
            filter->setNewtonTolerance(volts);
            env->setNewtonTolerance(volts);
            vca->setNewtonTolerance(volts);
            stage->setNewtonTolerance(volts);
            lighter = on;
        }
        [[nodiscard]] bool isLighter() const noexcept { return lighter; }

        /** The Devil Fish's new knobs (each its turn 0..1) and switches, after Whittle's manual (2019). The defaults
            are where it sounds most like a TB-303 (his page 8), with our own scales where his were not published:
                overdrive    the oscillator into the filter: none at 0, the stock level at 0.2, 66.6 times at 1
                slideTime    60 ms (0) to some 360 ms (1): 3.3k and a 500k pot in series with the ladder's 100k
                softAttack   plain notes' attack, 0.3 ms (0) to 30 ms (1), over the circuit's own
                normalDecay, accentDecay   the main envelope on plain and accented notes, 30 ms (0) to 3 s (1)
                tracking     the filter following the note: none (0) to 1.5 octaves a volt (1), C (2 V) unmoved
                filterFm     the VCA's output into the filter's control, AC coupled (a 100k audio pot, 0.1 uF, 20k)
                sweepSpeed   the accent sweep: 0 fast, 1 normal (the TB-303's), 2 slow
                accentSweep  Whittle's Acc Sweep - Hi Resonance switch: 2 the stock, 1 high resonance, 0 high
                             resonance and no accent sweep
                muffler      after the VCA: 0 off, 1 soft, 2 hard
            Whittle's other changes are fixed in a Devil Fish: Env Mod from zero to three times, Cut Off lower and
            higher, the fuller bass, and DECAY on the volume envelope (16 ms to 3.5 s in its first half, the long
            decay holding the note from silence to full in the second). */
        struct DevilFish
        {
            double overdrive = 0.2, slideTime = 0.0, softAttack = 0.0, normalDecay = 0.62, accentDecay = 0.41;
            double tracking = 0.0, filterFm = 0.0;
            int sweepSpeed = 1, accentSweep = 2, muffler = 0;
        };

        /** On a Devil Fish only; a stock voice ignores it. */
        void setDevilFish(const DevilFish& f) noexcept
        {
            fishSettings = f;
            if (!fish)
                return;
            const double o = std::clamp(f.overdrive, 0.0, 1.0);
            const double gain = o < 0.2 ? o / 0.2 : std::pow(66.6, (o - 0.2) / 0.8);
            const int sweep = std::clamp(f.sweepSpeed, 0, 2), mode = std::clamp(f.accentSweep, 0, 2);
            const double c13 = sweep == 0 ? 0.22e-6 : sweep == 1 ? 1.0e-6 : 3.3e-6;
            const double r46 = sweep == 2 ? 22.0e3 : 47.0e3;
            filter->setChanges(gain, true, mode < 2, c13, r46, true);
            vca->setFullBass(true);
            osc->setSlideOhms(3.3e3 + 500.0e3 * std::clamp(f.slideTime, 0.0, 1.0));
            attackSeconds = 0.3e-3 * std::pow(100.0, std::clamp(f.softAttack, 0.0, 1.0));
            setFishEnvelope();
        }
        [[nodiscard]] const DevilFish& devilFish() const noexcept { return fishSettings; }

        /** The main envelope's decay resistance (R138 and the pot) for a time: the stock circuit's 200 ms at 68k and
            2 s at 1068k give time ~ R^0.836, held through Whittle's 30 ms to 3 s. */
        static double decayOhms(double turn) noexcept
        {
            const double seconds = 0.03 * std::pow(100.0, std::clamp(turn, 0.0, 1.0));
            return 68.0e3 * std::pow(seconds / 0.2, 1.0 / 0.836);
        }
        /** Filter Tracking's current an octave moves the cutoff by, into Q10's base (some 2.15k to VQ9). */
        static constexpr double octaveAmps = 8.3e-6;

        /** RESONANCE's track (VR4, 50k B, two gangs): the share of the pot at a turn of the knob, fitted to Din Sync's
            unit (CoolSynth's D324). Its marks fall on a straight track at about 0.10 to 0.15 (a quarter turn), 0.5,
            0.88 and 1: an S-shaped track, Whittle's "log curve at both ends". t^2 / (t^2 + (1 - t)^2) gives 0.10, 0.5,
            0.90 and 1; a quarter turn then comes within 1.7 dB of the unit where it was 4.5 dB quiet. The unit's
            loop also falls some 4 % short of the schematic's (Lemus's ro-mono-303 put it at 98 % for the same unit):
            at full resonance with the cutoff swept to its top, the schematic whistles above 12.8 kHz 12 dB louder
            than the unit. As a share of the pot, 0.96 brings that within 1 dB, and set E's notes within about 1 dB
            where they were 2 to 4 dB off; the sets at lower knobs are unchanged. */
        static constexpr double resonanceLoop = 0.96;
        static double resonanceTrack(double turn) noexcept
        {
            const double t = std::clamp(turn, 0.0, 1.0), u = 1.0 - t;
            return resonanceLoop * t * t / (t * t + u * u);
        }

        /** The panel's knobs, each its turn 0..1, and the WAVEFORM switch. VOLUME (VR8) is the output stage's pot,
            ahead of the mixer: full, as Din Sync recorded the unit, loud notes clip on the mixer's lower side. */
        struct Knobs
        {
            double tuning = 0.5, cutoff = 0.5, resonance = 0.5, envMod = 0.5, decay = 0.5, accent = 0.5;
            bool square = false;
            double volume = 1.0;
        };

        /** The trimmers and the part known to vary, as set at the factory or by a player: TM3 the cutoff's range, TM4
            the tune, TM5 the scale's width, C11 the shaper's capacitor. Their defaults are the fitted unit's above. */
        struct Trimmers
        {
            double tm3 = Tb303Voice::tm3, tm4 = Tb303Voice::tm4, tm5 = Tb303Voice::tm5, c11 = Tb303Voice::c11;
        };

        void setTrimmers(const Trimmers& t) noexcept
        {
            trim = t;
            setKnobs(knobs);
        }
        [[nodiscard]] const Trimmers& trimmers() const noexcept { return trim; }

        void setKnobs(const Knobs& k) noexcept
        {
            knobs = k;
            osc->setTrimmers(trim.tm4, trim.tm5, k.tuning, trim.c11);
            filter->setKnobs(k.cutoff, resonanceTrack(k.resonance), k.envMod, trim.tm3);
            env->setKnobs(k.decay, k.accent);
            stage->setVolume(k.volume);
            if (fish)
                setFishEnvelope();
        }

        /** VOLUME alone (the app's knob moves it as it plays; nothing else is re-analysed). */
        void setVolume(double turn) noexcept
        {
            knobs.volume = turn;
            stage->setVolume(turn);
        }

        /** At rest at this note with the gate low (C33 at 8 V, as the reference starts it). */
        void settle(double noteVolts) noexcept
        {
            env->setAccent(false);
            env->setAccentLoad(0.0);
            env->settle();
            osc->setSlide(false);
            osc->settle(noteVolts, (Tb303Oscillator::top - 8.0) / (Tb303Oscillator::top - 5.36));
            filter->setFishCurrent(0.0);
            filter->settle(8.35, env->envelope(), env->accentWiper());
            vca->settle(env->pinOne());
            stage->settle(vca->pinEight());
            out = stage->output();
            vcaOut = 0.0;
            fmHigh = fmLast = mufflerLow = 0.0;
            attack = 1.0;
            gateBefore = false;
        }

        /** Back from rest at this note, the rest of the circuit as settle() left it: only the oscillator settles at
            the new pitch (0.02 ms against the whole voice's 1 ms, most of it the filter's). A host that rests the
            voice settles it whole while resting, when that costs nothing, and wakes it with this (CoolSynth's D340):
            the first block of a note then carries only the attack's own work. */
        void wake(double noteVolts) noexcept
        {
            osc->setSlide(false);
            osc->settle(noteVolts, (Tb303Oscillator::top - 8.0) / (Tb303Oscillator::top - 5.36));
            fmHigh = fmLast = mufflerLow = 0.0;
            attack = 1.0;
            gateBefore = false;
        }

        /** One sample: the note's volts, the gate, the accent and the slide as the sequencer sets them. */
        double process(double noteVolts, bool gate, bool accent, bool slide) noexcept
        {
            env->setAccent(accent);
            env->setAccentLoad(filter->accentCurrent());
            env->process(gate);
            osc->setSlide(slide);
            if (fish)
                filter->setFishCurrent(fishCurrent());
            const double saw = osc->process(noteVolts);
            wave = knobs.square ? osc->squareVolts() : saw;
            filter->process(wave, env->envelope(), env->accentWiper());
            vcaOut = vca->process(filter->output(), filter->resonanceWiper(), env->pinOne());
            double pinEight = vca->pinEight();
            if (fish)
            {
                // The Devil Fish's Soft Attack and Muffler on the VCA's output, ahead of VOLUME: what they take off
                // the VCA's output (C38 into 100k) comes off pin 8 as well.
                const double fished = fishOutput(vcaOut, gate, accent);
                pinEight += fished - vcaOut;
                vcaOut = fished;
            }
            out = stage->process(pinEight);
            return out;
        }

        /** The OUTPUT jack's volts, as process() returned them. */
        [[nodiscard]] double output() const noexcept { return out; }
        /** The VCA's output (C38 into 100k), the voice's output before the output stage (D362), a Devil Fish's after
            its Soft Attack and Muffler. */
        [[nodiscard]] double vcaOutput() const noexcept { return vcaOut; }
        [[nodiscard]] double waveform() const noexcept { return wave; }
        [[nodiscard]] const Tb303Oscillator& oscillator() const noexcept { return *osc; }
        [[nodiscard]] const Tb303Filter& vcf() const noexcept { return *filter; }
        [[nodiscard]] const Tb303Envelope& envelope() const noexcept { return *env; }
        [[nodiscard]] const Tb303Vca& amplifier() const noexcept { return *vca; }
        [[nodiscard]] const Tb303Output& outputStage() const noexcept { return *stage; }
        /** The recorder's input on the OUTPUT jack (Tb303Output::recorderOhms unless set). */
        void setLoad(double ohms) noexcept { stage->setLoad(ohms); }
        [[nodiscard]] bool lastSolveConverged() const noexcept
        {
            return osc->lastSolveConverged() && filter->lastSolveConverged() && env->lastSolveConverged()
                   && vca->lastSolveConverged() && stage->lastSolveConverged();
        }
        [[nodiscard]] bool fitsItsLimits() const noexcept
        {
            return osc->fitsItsLimits() && filter->fitsItsLimits() && env->fitsItsLimits() && vca->fitsItsLimits()
                   && stage->fitsItsLimits();
        }

    private:
        /** The main envelope's two decays and the volume envelope from the knobs (DECAY is the volume's). */
        void setFishEnvelope() noexcept
        {
            const double d = std::clamp(knobs.decay, 0.0, 1.0);
            // The first half: 16 ms to 3.5 s (R123 1.5M is the stock 3.5 s); the second: that, holding the note.
            const double seconds = d < 0.5 ? 0.016 * std::pow(3.5 / 0.016, d / 0.5) : 3.5;
            const double vegOhms = 1.5e6 * seconds / 3.5;
            const double hold = d < 0.5 ? 0.0 : (d - 0.5) / 0.5;
            env->setDevilFish(decayOhms(fishSettings.normalDecay) - Tb303Envelope::fishR138,
                              decayOhms(fishSettings.accentDecay) - Tb303Envelope::fishR138, vegOhms,
                              Tb303Envelope::supply * (1.0 - hold), fishSettings.accentSweep > 0);
        }

        /** Filter Tracking (from the CV after the slide, as the Devil Fish takes it) and Filter FM (the output a
            sample back, through 0.1 uF and 20k into a node some 2.2k from VQ9: a high-pass near 72 Hz). The FM takes
            the output turned over, as the 303's jack gives it (Din Sync's recordings are inverted against the VCA's
            own): taken as it was, it fed back against the filter and darkened the note where Whittle's heavy FM
            (his rw13, rw10) spreads it bright and gritty (CoolSynth's D336). */
        double fishCurrent() noexcept
        {
            // a current into Q10's base closes the filter (measured), so a higher note draws current out of it
            const double tracking = -std::clamp(fishSettings.tracking, 0.0, 1.0) * 1.5 * octaveAmps * (osc->cvVolts() - 2.0);
            const double a = std::exp(-1.0 / (voiceRate * 22.2e3 * 0.1e-6));
            fmHigh = a * (fmHigh + vcaOut - fmLast);
            fmLast = vcaOut;
            const double fm = -Tb303Filter::audioTaper(fishSettings.filterFm) * fmHigh / 22.2e3;
            return tracking + fm;
        }

        /** Soft Attack on plain notes (a new gate without accent starts it), then the Muffler: the part above some
            100 Hz soft-clipped, the bass passed, so loud notes buzz and soften while quiet ones are untouched. */
        double fishOutput(double v, bool gate, bool accent) noexcept
        {
            if (gate && !gateBefore)
                attack = accent ? 1.0 : 0.0;
            gateBefore = gate;
            attack += (1.0 - attack) * (1.0 - std::exp(-1.0 / (voiceRate * attackSeconds)));
            v *= attack;
            const int m = std::clamp(fishSettings.muffler, 0, 2);
            if (m == 0)
                return v;
            mufflerLow += (v - mufflerLow) * (1.0 - std::exp(-2.0 * 3.141592653589793 * 100.0 / voiceRate));
            const double t = m == 1 ? 0.6 : 0.25;
            return mufflerLow + t * std::tanh((v - mufflerLow) / t);
        }

        std::unique_ptr<Tb303Oscillator> osc;
        std::unique_ptr<Tb303Filter> filter;
        std::unique_ptr<Tb303Envelope> env;
        std::unique_ptr<Tb303Vca> vca;
        std::unique_ptr<Tb303Output> stage;
        Knobs knobs;
        Trimmers trim;
        double wave = 8.0, out = 0.0, vcaOut = 0.0;
        bool fish = false, lighter = false;
        DevilFish fishSettings;
        double voiceRate = rate, attackSeconds = 0.3e-3, attack = 1.0, fmHigh = 0.0, fmLast = 0.0, mufflerLow = 0.0;
        bool gateBefore = false;
    };
}
