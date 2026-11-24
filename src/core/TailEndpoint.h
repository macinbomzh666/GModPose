#pragma once

#include "LimbPose.h"
#include "TailPose.h"

#include <numbers>

namespace GModPose
{
    struct TailClearance
    {
        TorsoClearance torso;
        struct Thigh { Vec3 hip, knee; double radius; };
        std::array<std::optional<Thigh>, 2> thighs{};

        double Overlap(std::span<const Vec3> joints) const
        {
            double score = 0;
            for (std::size_t i = 1; i < joints.size(); ++i) {
                const auto link = joints[i] - joints[i - 1];
                for (int sample = 1; sample <= 8; ++sample) {
                    // The base is attached to the pelvis. Give it room to exit the body.
                    const auto t = sample / 8.;
                    const auto point = joints[i - 1] + link * t;
                    double depth = torso.Penetration(point);
                    for (const auto& leg : thighs) {
                        if (!leg) continue;
                        const auto axis = leg->knee - leg->hip;
                        const auto station = std::clamp(Dot(point - leg->hip, axis) /
                            std::max(Dot(axis, axis), 1e-8), 0., 1.);
                        depth = std::max(depth, 1 - Length(point - (leg->hip + axis * station)) / leg->radius);
                    }
                    score += depth * depth * (i == 1 ? t * t : 1.);
                }
            }
            return score;
        }
    };

    // Use equal curvature as the starting shape. With unequal links there
    // may be no circular arc at this reach, in which case the guide is fitted
    // back to the rigid link lengths with both ends held.
    inline std::array<Vec3, tailJointCount> TailArc(Vec3 root, Vec3 tip, Vec3 axis, Vec3 bend,
                                                   std::span<const double> links)
    {
        const double distance = Length(tip - root);
        const double length = TailReachFor(links).max;
        const auto sweep = [&](double radius) {
            double angle = 0;
            for (double link : links) angle += 2 * std::asin(std::min(1., link / (2 * radius)));
            return angle;
        };
        auto low = *std::ranges::max_element(links) * .5;
        auto high = length * 4096;
        for (int i = 0; i < 42; ++i) {
            const auto radius = (low + high) * .5;
            const auto angle = sweep(radius);
            if (angle > 2 * std::numbers::pi || 2 * radius * std::sin(angle * .5) < distance) low = radius;
            else high = radius;
        }
        const auto angle = sweep(high);
        const bool circle = std::abs(2 * high * std::sin(angle * .5) - distance) < length * 1e-6;
        std::array<Vec3, tailJointCount> joints{};
        joints[0] = root;
        joints[links.size()] = tip;
        double arc = 0, travel = 0;
        for (std::size_t i = 1; i < links.size(); ++i) {
            arc += 2 * std::asin(std::min(1., links[i - 1] / (2 * high)));
            travel += links[i - 1];
            const auto fraction = travel / length;
            const auto guide = circle ?
                root + axis * (distance * .5 + high * std::sin(arc - angle * .5)) +
                    bend * (high * (std::cos(arc - angle * .5) - std::cos(angle * .5))) :
                root + (tip - root) * fraction + bend * ((length - distance) * std::sin(fraction * std::numbers::pi) * .5);
            joints[i] = TailJointBetween(joints[i - 1], tip, guide, root + bend * length,
                {links[i - 1], links[i - 1]}, TailReachFor(links.subspan(i)));
        }
        return joints;
    }

    inline std::optional<TailMove> SolveTailEndpoint(Vec3 root, std::span<const double> links, Vec3 target,
        Vec3 preferredBend, const TailClearance* clearance = nullptr, std::optional<Vec3> tipTangent = {})
    {
        if (links.empty() || links.size() >= tailJointCount || !IsFinite(root) || !IsFinite(target) ||
            !std::ranges::all_of(links, [](double length) { return std::isfinite(length) && length > 1e-6; }))
            return std::nullopt;
        const auto reach = TailReachFor(links);
        const auto axis = Unit(target - root).value_or(Unit(preferredBend).value_or(Vec3{0, -1, 0}));
        const auto tip = root + axis * std::clamp(Length(target - root), reach.min, reach.max);
        const auto normal = Unit(preferredBend - axis * Dot(preferredBend, axis)).value_or(
            Unit(Cross(axis, std::abs(axis.z) < .9 ? Vec3{0, 0, 1} : Vec3{0, 1, 0})).value_or(Vec3{1, 0, 0}));
        const auto tangent = tipTangent ? Unit(*tipTangent) : std::nullopt;
        const auto last = links.size();
        const auto score = [&](const std::array<Vec3, tailJointCount>& joints) {
            double cost = clearance ? 8 * clearance->Overlap(std::span(joints).first(last + 1)) : 0;
            auto previous = (joints[1] - joints[0]) / links.front();
            Vec3 previousBendStep{};
            for (std::size_t i = 1; i < last; ++i) {
                const auto direction = (joints[i + 1] - joints[i]) / links[i];
                const auto bendStep = Cross(previous, direction) * (reach.max / (links[i - 1] + links[i]));
                cost += .2 * (1 - Dot(previous, direction));
                if (i > 1) {
                    const auto change = bendStep - previousBendStep;
                    cost += .35 * Dot(change, change);
                }
                previousBendStep = bendStep;
                previous = direction;
            }
            if (tangent) cost += 1.2 * (1 - Dot(previous, *tangent));
            // Keep a deterministic preferred side when similar arcs fit.
            if (last > 1) cost -= .035 * Dot(joints[last / 2] - root, normal) / reach.max;
            return cost;
        };
        const auto referenceArc = TailArc(root, tip, axis, normal, links);
        auto best = referenceArc;
        double bestCost = score(best);
        // Search both sides of the body and align the approach to a turned tip.
        for (int direction = 1; direction < 12; ++direction) {
            const auto turn = AxisAngle(axis, direction * std::numbers::pi / 6);
            auto candidate = referenceArc;
            for (std::size_t i = 1; i < last; ++i)
                candidate[i] = root + *Rotate(turn, referenceArc[i] - root);
            const auto cost = score(candidate);
            if (cost < bestCost - 1e-9) {
                bestCost = cost;
                best = candidate;
            }
        }
        // Improve the selected arc without disturbing the endpoint or link lengths.
        for (double step : {.28, .14, .07, .035}) {
            for (std::size_t i = 1; i < last; ++i) {
                const auto chord = Unit(best[i + 1] - best[i - 1]);
                if (!chord) continue;
                const auto original = best[i];
                auto selected = original;
                for (double angle : {-step, step}) {
                    best[i] = best[i - 1] + *Rotate(AxisAngle(*chord, angle), original - best[i - 1]);
                    const auto cost = score(best);
                    if (cost < bestCost - 1e-9) {
                        bestCost = cost;
                        selected = best[i];
                    }
                }
                best[i] = selected;
            }
        }
        for (std::size_t i = 0; i < last; ++i)
            if (!IsFinite(best[i + 1]) || std::abs(Length(best[i + 1] - best[i]) - links[i]) > links[i] * 1e-5)
                return std::nullopt;
        return TailMove{best, Length(tip - target) > .001};
    }
}
