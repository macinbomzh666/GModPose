#include "FingerPose.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace GModPose::Fingers
{
    namespace
    {
        constexpr double curlProjection = .38;
    }

    Vec3 DiagramSegment::Tip(Angles angles) const
    {
        const Vec3 across{along.y, -along.x, 0};
        const auto direction = along * std::cos(angles.spread) + across * std::sin(angles.spread);
        return direction * (length * (1 - curlProjection * angles.curl));
    }

    Angles DiagramSegment::Drag(Vec3 tip, Angles start) const
    {
        const auto forward = Dot(tip, along);
        const auto across = Dot(tip, {along.y, -along.x, 0});
        const auto clamp = [](double value, double initial, double low, double high) {
            return std::clamp(value, std::min(initial, low), std::max(initial, high));
        };
        const auto radius = forward > 0 ? Length(tip) : 0.0;
        const auto spread = radius > .0001 ? std::atan2(across, forward) : start.spread;
        return {clamp((1 - radius / length) / curlProjection, start.curl, -.80, 1.52),
            clamp(spread, start.spread, -.85, .85), start.roll};
    }

    Angles ReadAngles(Quaternion rotation)
    {
        const auto forward = Rotate(rotation, {0, 0, 1}).value_or(Vec3{0, 0, 1});
        const auto curl = std::asin(std::clamp(-forward.y, -1.0, 1.0));
        const auto spread = std::atan2(forward.x, forward.z);
        const auto swing = FromAngles({curl, spread});
        const auto twist = Multiply(Conjugate(swing), rotation);
        return {curl, spread, std::remainder(2 * std::atan2(twist.z, twist.w), 2 * std::numbers::pi)};
    }

    Quaternion FromAngles(Angles angles)
    {
        return Multiply(Multiply(AxisAngle({0, 1, 0}, angles.spread), AxisAngle({1, 0, 0}, angles.curl)),
                        AxisAngle({0, 0, 1}, angles.roll));
    }
}
