#pragma once

#include "PoseMath.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace GModPose
{
    inline constexpr std::size_t tailJointCount = 5;

    struct TailReach
    {
        double min, max;
    };

    inline TailReach TailReachFor(std::span<const double> lengths)
    {
        double sum = 0, longest = 0;
        for (const auto length : lengths) {
            sum += length;
            longest = std::max(longest, length);
        }
        return {std::max(0.0, 2 * longest - sum), sum};
    }

    // Choose two feasible radii, then aim at the target on their intersection.
    // Each radius interval is the reach of one half of the chain.
    inline Vec3 TailJointBetween(Vec3 base, Vec3 tip, Vec3 target, Vec3 preferred, TailReach fromBase, TailReach fromTip)
    {
        const auto offset = tip - base;
        const double distance = Length(offset);
        if (distance < 1e-8) {
            const auto axis = Unit(target - base).value_or(Unit(preferred - base).value_or(Vec3{1, 0, 0}));
            const double radius = std::max(std::max(fromBase.min, fromTip.min),
                std::min({Length(target - base), fromBase.max, fromTip.max}));
            return base + axis * radius;
        }
        const auto axis = offset / distance;
        const auto aim = target - base;
        const double lowA = std::max({fromBase.min, distance - fromTip.max, fromTip.min - distance});
        const double highA = std::min(fromBase.max, distance + fromTip.max);
        const double a = std::max(lowA, std::min(Length(aim), highA));
        const double lowB = std::max(fromTip.min, std::abs(distance - a));
        const double highB = std::min(fromTip.max, distance + a);
        const double b = std::max(lowB, std::min(Length(target - tip), highB));
        const double along = (distance * distance + (a - b) * (a + b)) / (2 * distance);
        const double radius = std::sqrt(std::max(0.0, (a - along) * (a + along)));
        auto radial = Unit(aim - axis * Dot(aim, axis));
        if (!radial) {
            const auto rest = preferred - base;
            radial = Unit(rest - axis * Dot(rest, axis));
        }
        if (!radial) radial = Unit(Cross(axis, std::abs(axis.z) < .9 ? Vec3{0, 0, 1} : Vec3{0, 1, 0}));
        return base + axis * along + radial.value_or(Vec3{}) * radius;
    }

    struct TailMove
    {
        std::array<Vec3, tailJointCount> joints;
        bool limited;
    };

    inline std::optional<TailMove> MoveTail(std::span<const Vec3> baseline, std::size_t held, Vec3 target)
    {
        if (baseline.size() < 2 || baseline.size() > tailJointCount || !held || held >= baseline.size() ||
            !IsFinite(target)) return std::nullopt;
        const auto tip = baseline.size() - 1;
        std::array<double, tailJointCount - 1> lengths{};
        for (std::size_t i = 0; i < tip; ++i) {
            lengths[i] = Length(baseline[i + 1] - baseline[i]);
            if (!std::isfinite(lengths[i]) || lengths[i] < 1e-6) return std::nullopt;
        }
        const auto links = std::span(lengths).first(tip);
        TailMove result{};
        auto& joints = result.joints;
        joints[0] = baseline.front();
        joints[tip] = baseline.back();
        if (held == tip) {
            const auto offset = target - joints[0];
            const auto reach = TailReachFor(links);
            const auto aim = Unit(offset).value_or(Unit(joints[tip] - joints[0]).value_or(Vec3{1, 0, 0}));
            joints[tip] = joints[0] + aim * std::clamp(Length(offset), reach.min, reach.max);
        } else {
            joints[held] = TailJointBetween(joints[0], joints[tip], target, baseline[held],
                TailReachFor(links.first(held)), TailReachFor(links.subspan(held)));
        }
        const auto solve = [&](std::size_t first, std::size_t last) {
            for (std::size_t i = first; i + 1 < last; ++i) {
                joints[i + 1] = TailJointBetween(joints[i], joints[last], baseline[i + 1], baseline[i + 1],
                    {lengths[i], lengths[i]}, TailReachFor(links.subspan(i + 1, last - i - 1)));
            }
        };
        solve(0, held);
        solve(held, tip);

        // Reject numerical degeneracies before applying rotations to the real skeleton.
        for (std::size_t i = 0; i < tip; ++i) {
            if (!IsFinite(joints[i + 1]) ||
                std::abs(Length(joints[i + 1] - joints[i]) - lengths[i]) > lengths[i] * 1e-5)
                return std::nullopt;
        }
        result.limited = Length(joints[held] - target) > .001;
        return result;
    }

    inline Quaternion TailTwist(Quaternion start, Quaternion target, Vec3 axis)
    {
        const auto change = Multiply(target, {-start.x, -start.y, -start.z, start.w});
        const auto roll = axis * Dot({change.x, change.y, change.z}, axis);
        const auto twist = Unit(Quaternion{roll.x, roll.y, roll.z, change.w});
        return twist ? Multiply(*twist, start) : start;
    }

    // The next controlled joint's actual parent may sit below the preceding
    // controlled joint. Its relative frame stays fixed during the solve.
    inline Quaternion TailLocalRotation(Quaternion world, Quaternion previous, Quaternion parentOffset)
    {
        const auto parent = Multiply(previous, parentOffset);
        return Multiply({-parent.x, -parent.y, -parent.z, parent.w}, world);
    }
}
