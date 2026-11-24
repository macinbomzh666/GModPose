#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace GModPose
{
    class BreathingCycle
    {
    public:
        double Update(bool idle, double dt)
        {
            constexpr double period = 4.5;
            constexpr double lift = 0.20;
            constexpr double fadeIn = 0.55, fadeOut = 0.25;
            dt = std::isfinite(dt) ? std::clamp(dt, 0.0, 0.05) : 0;
            blend = idle ? std::min(1.0, blend + dt / fadeIn) : std::max(0.0, blend - dt / fadeOut);
            if (blend == 0) {
                phase = 0;
                return 0;
            }
            phase = std::fmod(phase + dt, period);
            const auto envelope = blend * blend * (3 - 2 * blend);
            return lift * envelope * (1 - std::cos(2 * std::numbers::pi * phase / period)) * 0.5;
        }

    private:
        double phase{}, blend{};
    };
}
