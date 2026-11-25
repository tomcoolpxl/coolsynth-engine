#pragma once

#include <array>
#include <cmath>
#include <cstdint>

/*
    The host-time filter for audio callbacks (CoolSynth's D361, docs/research/beat-sync-protocol.md, "Following the
    transport sample-accurately"). Callbacks do not run at even times, so the host time read in one jitters by up to a
    millisecond against the sample clock. BlockClock keeps the last 512 pairs (samples rendered so far, host time at
    the callback) in a fixed ring and fits a straight line; a block's host time is the line's value at its sample
    count. The line's slope comes from every pair since the last restart (a running, centred least-squares sum), its
    height from the ring's 512, so the slope settles to a few parts per billion while the height follows the last
    few seconds. It works relative to its first pair, so doubles keep full precision, and restarts when told (the
    device restarted, the rate changed) or when a gap of more than 100 ms shows a dropout. Its slope is the real
    number of microseconds per sample. Audio thread only; no allocation, no system call.
*/
namespace coolsynth::sync
{
    class BlockClock
    {
    public:
        struct Time
        {
            std::int64_t hostMicros = 0; // the host time of the block's first sample, filtered
            double microsPerSample = 0.0;
        };

        static constexpr int kPairs = 512;
        static constexpr std::int64_t kDropoutMicros = 100000;
        static constexpr int kPairsForSlope = 32;

        explicit BlockClock(double sampleRate = 48000.0) noexcept { reset(sampleRate); }

        /** The device started or its rate changed. */
        void reset(double sampleRate) noexcept
        {
            nominal = sampleRate > 0.0 ? 1.0e6 / sampleRate : 1.0e6 / 48000.0;
            started = false;
            rendered = 0;
        }

        /** Called once per audio callback, with the host time read in it and the block's length. */
        Time next(std::int64_t callbackHostMicros, int numSamples) noexcept
        {
            if (started)
            {
                const double expected = static_cast<double>(lastHost) + static_cast<double>(rendered - lastRendered) * nominal;
                if (std::abs(static_cast<double>(callbackHostMicros) - expected) > static_cast<double>(kDropoutMicros))
                    started = false;
            }
            if (! started)
            {
                started = true;
                originSamples = rendered;
                originHost = callbackHostMicros;
                count = head = 0;
                n = meanX = meanY = cxx = cxy = 0.0;
            }

            const double x = static_cast<double>(rendered - originSamples);
            const double y = static_cast<double>(callbackHostMicros - originHost);
            xs[static_cast<std::size_t>(head)] = x;
            ys[static_cast<std::size_t>(head)] = y;
            head = (head + 1) % kPairs;
            if (count < kPairs)
                ++count;

            n += 1.0;
            const double dx = x - meanX;
            meanX += dx / n;
            const double dy = y - meanY;
            meanY += dy / n;
            cxx += dx * (x - meanX);
            cxy += dx * (y - meanY);

            double slope = nominal;
            if (n >= kPairsForSlope && cxx > 0.0)
            {
                const double fitted = cxy / cxx;
                if (std::abs(fitted / nominal - 1.0) < 0.01)
                    slope = fitted;
            }

            double sx = 0.0, sy = 0.0;
            for (int i = 0; i < count; ++i)
            {
                sx += xs[static_cast<std::size_t>(i)];
                sy += ys[static_cast<std::size_t>(i)];
            }
            const double mx = sx / count, my = sy / count;
            const double value = my + slope * (x - mx);

            lastHost = callbackHostMicros;
            lastRendered = rendered;
            rendered += numSamples;
            return { originHost + std::llround(value), slope };
        }

    private:
        std::array<double, kPairs> xs {}, ys {};
        int count = 0, head = 0;
        bool started = false;
        double nominal = 1.0e6 / 48000.0;
        std::int64_t rendered = 0, originSamples = 0, originHost = 0, lastHost = 0, lastRendered = 0;
        double n = 0.0, meanX = 0.0, meanY = 0.0, cxx = 0.0, cxy = 0.0;
    };
}
