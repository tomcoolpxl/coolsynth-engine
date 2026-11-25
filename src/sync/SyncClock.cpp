#include "sync/SyncClock.h"

#include <cmath>

#include <juce_core/juce_core.h>

namespace coolsynth::sync
{
    std::int64_t hostMicrosNow() noexcept
    {
        static const double microsPerTick = 1.0e6 / static_cast<double>(juce::Time::getHighResolutionTicksPerSecond());
        return std::llround(static_cast<double>(juce::Time::getHighResolutionTicks()) * microsPerTick);
    }
}
