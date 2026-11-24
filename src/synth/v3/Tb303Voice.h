#pragma once

#include <memory>

#include "Tb303Envelope.h"
#include "Tb303Filter.h"
#include "Tb303Oscillator.h"
#include "Tb303Vca.h"

/*
    The Roland TB-303's voice as its circuit (D275, D295): the four stages wired as tools/tb303/voice.cir wires them.

        the VCO's saw or square (the WAVEFORM switch) into the filter;
        the envelope into ENV MOD, the ACCENT knob's wiper into the filter's D24 (whose current loads the wiper, handed
        back to the envelope a step late);
        the filter's output and its RESONANCE wiper (loaded by the VCA's C22 and R122) into the VCA;
        the VCA's control current from the envelope's circuit (pin 1).

    At 96 kHz (D294): the square's shaper at 768 kHz inside the oscillator, the envelope near 48 kHz. The trimmers as
    page 7 sets them (TM4, TM5) and as fitted to Din Sync's unit (TM3 0.35, C11 0.82 uF; D275). In: the CPU's note as
    the ladder's volts (1 V an octave, the A key unshifted 2.75 V), the gate, the accent and the slide of each step;
    out: the VCA's output into the mixer, volts.
*/
namespace coolsynth::synth::v3
{
    class Tb303Voice final
    {
    public:
        static constexpr double rate = 96000.0;
        static constexpr double tm3 = 0.35, tm4 = 0.5687, tm5 = 0.7389, c11 = 0.82e-6;

        Tb303Voice()
            : osc(std::make_unique<Tb303Oscillator>()), filter(std::make_unique<Tb303Filter>(true)),
              env(std::make_unique<Tb303Envelope>()), vca(std::make_unique<Tb303Vca>())
        {
        }

        void prepare(double sampleRate = rate) noexcept
        {
            osc->prepare(sampleRate);
            filter->prepare(sampleRate);
            env->prepare(sampleRate);
            vca->prepare(sampleRate);
            setKnobs(knobs);
        }

        /** The panel's knobs, each its turn 0..1, and the WAVEFORM switch. */
        struct Knobs
        {
            double tuning = 0.5, cutoff = 0.5, resonance = 0.5, envMod = 0.5, decay = 0.5, accent = 0.5;
            bool square = false;
        };

        void setKnobs(const Knobs& k) noexcept
        {
            knobs = k;
            osc->setTrimmers(tm4, tm5, k.tuning, c11);
            filter->setKnobs(k.cutoff, k.resonance, k.envMod, tm3);
            env->setKnobs(k.decay, k.accent);
        }

        /** At rest at this note with the gate low (C33 at 8 V, as the reference starts it). */
        void settle(double noteVolts) noexcept
        {
            env->setAccent(false);
            env->setAccentLoad(0.0);
            env->settle();
            osc->setSlide(false);
            osc->settle(noteVolts, (Tb303Oscillator::top - 8.0) / (Tb303Oscillator::top - 5.36));
            filter->settle(8.35, env->envelope(), env->accentWiper());
            vca->settle(env->pinOne());
        }

        /** One sample: the note's volts, the gate, the accent and the slide as the sequencer sets them. */
        double process(double noteVolts, bool gate, bool accent, bool slide) noexcept
        {
            env->setAccent(accent);
            env->setAccentLoad(filter->accentCurrent());
            env->process(gate);
            osc->setSlide(slide);
            const double saw = osc->process(noteVolts);
            wave = knobs.square ? osc->squareVolts() : saw;
            filter->process(wave, env->envelope(), env->accentWiper());
            out = vca->process(filter->output(), filter->resonanceWiper(), env->pinOne());
            return out;
        }

        [[nodiscard]] double output() const noexcept { return out; }
        [[nodiscard]] double waveform() const noexcept { return wave; }
        [[nodiscard]] const Tb303Oscillator& oscillator() const noexcept { return *osc; }
        [[nodiscard]] const Tb303Filter& vcf() const noexcept { return *filter; }
        [[nodiscard]] const Tb303Envelope& envelope() const noexcept { return *env; }
        [[nodiscard]] const Tb303Vca& amplifier() const noexcept { return *vca; }
        [[nodiscard]] bool lastSolveConverged() const noexcept
        {
            return osc->lastSolveConverged() && filter->lastSolveConverged() && env->lastSolveConverged()
                   && vca->lastSolveConverged();
        }
        [[nodiscard]] bool fitsItsLimits() const noexcept
        {
            return osc->fitsItsLimits() && filter->fitsItsLimits() && env->fitsItsLimits() && vca->fitsItsLimits();
        }

    private:
        std::unique_ptr<Tb303Oscillator> osc;
        std::unique_ptr<Tb303Filter> filter;
        std::unique_ptr<Tb303Envelope> env;
        std::unique_ptr<Tb303Vca> vca;
        Knobs knobs;
        double wave = 8.0, out = 0.0;
    };
}
