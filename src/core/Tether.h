#pragma once
#include "PoseMath.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <limits>
#include <vector>

namespace GModPose
{
    inline Vec3 LimitDisplacement(Vec3 offset, double limit)
    {
        const double length = Length(offset);
        return length > limit ? offset * (limit / length) : offset;
    }

    // Place the picked point, including a wrist or a limb midpoint offset
    // from the center of mass. Turning the body must leave the contact fixed.
    inline std::optional<Transform> GripPose(Vec3 targetPoint, Vec3 localPoint, Quaternion orientation)
    {
        const auto rotation = Unit(orientation);
        if (!rotation || !IsFinite(targetPoint)) return std::nullopt;
        const auto offset = Rotate(*rotation, localPoint);
        if (!offset || !IsFinite(targetPoint - *offset)) return std::nullopt;
        return Transform{targetPoint - *offset, *rotation};
    }

    struct HoldDistance
    {
        double depth, lateral, vertical;

        static std::optional<HoldDistance> Capture(Ray aim, Vec3 right, Vec3 up, Vec3 point)
        {
            const auto displacement = point - aim.origin;
            const double depth = Dot(displacement, aim.direction);
            if (!IsFinite(displacement) || !std::isfinite(depth) || depth <= 0) return std::nullopt;
            return HoldDistance{depth, Dot(displacement, right), Dot(displacement, up)};
        }

        void Nudge(double notches)
        {
            if (!std::isfinite(notches)) return;
            constexpr double unitsPerNotch = 2;
            depth = std::clamp(depth + unitsPerNotch * std::clamp(notches, -24.0, 24.0),
                std::min(depth, 10.0), std::max(depth, 4096.0));
        }

        Vec3 Target(Ray aim, Vec3 right, Vec3 up) const
        {
            return aim.origin + aim.direction * depth + right * lateral + up * vertical;
        }
    };

    struct GripRotation
    {
        Quaternion current;
        Vec3 yawAxis, pitchAxis, rollAxis;

        static std::optional<GripRotation> Begin(Quaternion orientation, Vec3 right, Vec3 up, Vec3 forward)
        {
            const auto start = Unit(orientation);
            const auto pitch = Unit(right), yaw = Unit(up), roll = Unit(forward);
            if (!start || !pitch || !yaw || !roll) return std::nullopt;
            return GripRotation{*start, *yaw, *pitch, *roll};
        }

        Quaternion Update(double dx, double dy, double wheel = 0, double rotationScale = 1, Vec3 keyboardAngles = {},
                          std::optional<Vec3> keyboardRollAxis = {}, Vec3 directions = {1, 1, 1})
        {
            constexpr double sensitivity = 0.008;
            const double yaw = std::isfinite(dx) ? -dx * sensitivity * rotationScale : 0;
            const double pitch = std::isfinite(dy) ? -dy * sensitivity * rotationScale : 0;
            constexpr double rollPerNotch = std::numbers::pi / 36;
            const double roll = std::isfinite(wheel) ? std::clamp(wheel, -24.0, 24.0) * rollPerNotch * rotationScale : 0;
            const auto turn = yawAxis * ((yaw + keyboardAngles.x * rotationScale) * directions.x) +
                pitchAxis * ((pitch + keyboardAngles.y * rotationScale) * directions.y);
            const double angle = Length(turn);
            auto next = angle > 0 ? Multiply(AxisAngle(turn / angle, angle), current) : current;
            if (roll != 0) next = Multiply(AxisAngle(rollAxis, roll * directions.z), next);
            if (keyboardAngles.z != 0)
                next = Multiply(AxisAngle(keyboardRollAxis.value_or(rollAxis),
                    keyboardAngles.z * rotationScale * directions.z), next);
            current = Unit(next).value_or(current);
            return current;
        }
    };

    inline std::optional<Vec3> AngularCorrection(Quaternion current, Quaternion target, double gain, double limit)
    {
        const auto error = Unit(Multiply(target, {-current.x, -current.y, -current.z, current.w}));
        if (!error) return std::nullopt;
        const Vec3 vector{error->x, error->y, error->z};
        const auto axis = Unit(vector);
        if (!axis) return Vec3{};
        const auto angle = 2 * std::atan2(Length(vector), std::abs(error->w));
        return *axis * (std::copysign(std::min(gain * angle, limit), error->w));
    }

    inline Quaternion NeckFollow(Quaternion neck, Quaternion head, Quaternion target)
    {
        const auto turn = AngularCorrection(head, target, .35, .40);
        const auto axis = turn ? Unit(*turn) : std::nullopt;
        return axis ? Multiply(AxisAngle(*axis, Length(*turn)), neck) : neck;
    }

    inline Vec3 CubicBezier(Vec3 start, Vec3 first, Vec3 second, Vec3 end, double fraction)
    {
        const double t = std::clamp(fraction, 0.0, 1.0), u = 1 - t;
        return start * (u * u * u) + first * (3 * u * u * t) +
               second * (3 * u * t * t) + end * (t * t * t);
    }

    struct TetherCurve
    {
        struct ControlPoint
        {
            std::optional<Vec3> point;
            Vec3 filtered, velocity;

            void Update(Vec3 desired, double frequency, double maxBend, double dt)
            {
                if (!point || Length(filtered - desired) > 80) {
                    filtered = desired;
                    velocity = {};
                } else {
                    const auto error = filtered - desired, rate = velocity + error * frequency;
                    const auto decay = std::exp(-frequency * dt);
                    filtered = desired + (error + rate * dt) * decay;
                    velocity = (velocity - rate * (frequency * dt)) * decay;
                }
                // Reflect the damped motion so a sideways pull bows the beam
                // in the requested direction. The endpoints track immediately.
                point = desired + LimitDisplacement(desired - filtered, maxBend);
            }
        };
        ControlPoint first, second;

        void Update(Vec3 start, Vec3 tip, double dt, Vec3 cameraForward)
        {
            const auto chord = tip - start;
            const double length = Length(chord);
            const auto rise = Vec3{0, 0, std::min(length * 0.065, 17.0)};
            const auto launch = cameraForward * std::min(length * 0.14, 26.0);
            const double maxBend = std::min(length * 0.28, 36.0);
            first.Update(start + chord * 0.21 + launch + rise, 18, maxBend, dt);
            second.Update(tip - chord * 0.28 + rise * 0.6, 13, maxBend, dt);
        }

        Vec3 Point(Vec3 start, Vec3 tip, double t) const
        {
            const auto third = (tip - start) / 3;
            return CubicBezier(start, first.point.value_or(start + third), second.point.value_or(tip - third), tip, t);
        }
    };

    struct ScreenSegment { Vec3 start, end; };

    // The three coordinates here are clip x, clip y and the perspective
    // divisor w. Clip before dividing, including at the camera near plane.
    inline std::optional<ScreenSegment> ClipTetherSegment(Vec3 from, Vec3 to, double nearPlane)
    {
        if (!IsFinite(from) || !IsFinite(to) || !std::isfinite(nearPlane) || nearPlane <= 0) return std::nullopt;
        const auto distances = [nearPlane](Vec3 q) {
            return std::array{q.z - nearPlane, q.z + q.x, q.z - q.x, q.z + q.y, q.z - q.y};
        };
        const auto a = distances(from), b = distances(to);
        double enter = 0, exit = 1;
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (a[i] < 0 && b[i] < 0) return std::nullopt;
            if (a[i] < 0) enter = std::max(enter, a[i] / (a[i] - b[i]));
            else if (b[i] < 0) exit = std::min(exit, a[i] / (a[i] - b[i]));
        }
        if (enter > exit) return std::nullopt;
        const auto span = to - from;
        const auto start = from + span * enter, end = from + span * exit;
        return ScreenSegment{start / start.z, end / end.z};
    }
}
