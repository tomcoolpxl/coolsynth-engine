#pragma once

#include <cstdint>

namespace coolsynth::sync
{
    /** The host clock in microseconds: the high-resolution tick count (QueryPerformanceCounter on Windows,
        mach_absolute_time on macOS), llround(ticks * 1e6 / ticksPerSecond). Steady; no system call on Windows. */
    [[nodiscard]] std::int64_t hostMicrosNow() noexcept;
}
