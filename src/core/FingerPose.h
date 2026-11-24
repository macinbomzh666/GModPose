#pragma once

#include "PoseMath.h"
#include <array>

namespace GModPose::Fingers
{
    constexpr std::size_t digitCount = 5, jointsPerDigit = 3, jointCount = digitCount * jointsPerDigit;

    enum class Side { Left, Right };
    using Pose = std::array<Quaternion, jointCount>;
    using Mask = std::uint32_t;
    constexpr Mask allJoints = (Mask{1} << jointCount) - 1;

    constexpr Mask Joint(std::size_t index) { return Mask{1} << index; }

    struct Angles { double curl{}, spread{}, roll{}; };

    struct DiagramSegment
    {
        Vec3 along;
        double length;

        Vec3 Tip(Angles angles) const;
        Angles Drag(Vec3 tip, Angles start) const;
    };

    inline Quaternion Conjugate(Quaternion q) { return {-q.x, -q.y, -q.z, q.w}; }

    inline Quaternion ForSide(Quaternion q, Side side)
    {
        // Skyrim's paired finger frames mirror across local X. A rotation is
        // axial, so the other two quaternion components change sign.
        return side == Side::Left ? Quaternion{q.x, -q.y, -q.z, q.w} : q;
    }

    inline Quaternion FromLocal(Quaternion rest, Quaternion local, Side side)
    {
        return ForSide(Multiply(Conjugate(rest), local), side);
    }

    inline Quaternion ToLocal(Quaternion rest, Quaternion canonical, Side side)
    {
        return Multiply(rest, ForSide(canonical, side));
    }

    Angles ReadAngles(Quaternion rotation);
    Quaternion FromAngles(Angles angles);
}
