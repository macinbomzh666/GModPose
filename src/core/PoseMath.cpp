#include "PoseMath.h"

#include <algorithm>
#include <numbers>
#include <array>
#include <cmath>
#include <limits>

namespace GModPose
{
    Vec3 Vec3::operator+(Vec3 b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 Vec3::operator-(Vec3 b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 Vec3::operator*(double scale) const { return {x * scale, y * scale, z * scale}; }
    Vec3 Vec3::operator/(double scale) const { return {x / scale, y / scale, z / scale}; }
    double Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    Vec3 Cross(Vec3 a, Vec3 b)
    {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }
    double Length(Vec3 value) { return std::hypot(value.x, value.y, value.z); }
    bool IsFinite(Vec3 value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    std::optional<Vec3> Unit(Vec3 value)
    {
        const double length = Length(value);
        if (!IsFinite(value) || !std::isfinite(length) || length < 1e-12) return std::nullopt;
        return value / length;
    }

    std::optional<Quaternion> Unit(Quaternion value)
    {
        const double length = std::hypot(std::hypot(value.x, value.y), std::hypot(value.z, value.w));
        if (!std::isfinite(length) || length < 1e-12) return std::nullopt;
        return Quaternion{value.x / length, value.y / length, value.z / length, value.w / length};
    }

    Quaternion Multiply(Quaternion a, Quaternion b)
    {
        return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
    }

    namespace
    {
        Quaternion Conjugate(Quaternion q) { return {-q.x, -q.y, -q.z, q.w}; }

        Vec3 Orthogonal(Vec3 direction)
        {
            const std::array<Vec3, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
            const auto axis = *std::min_element(axes.begin(), axes.end(), [&](Vec3 a, Vec3 b) {
                return std::abs(Dot(a, direction)) < std::abs(Dot(b, direction));
            });
            return *Unit(Cross(direction, axis));
        }

        std::optional<Vec3> ProjectBend(Vec3 candidate, Vec3 direction, double minimum)
        {
            const auto projected = candidate - direction * Dot(candidate, direction);
            if (Length(projected) < minimum) return std::nullopt;
            return Unit(projected);
        }
    }

    std::optional<Quaternion> FromTo(Vec3 from, Vec3 to)
    {
        const auto a = Unit(from), b = Unit(to);
        if (!a || !b) return std::nullopt;
        const double cosine = std::clamp(Dot(*a, *b), -1.0, 1.0);
        const auto axis = Cross(*a, *b);
        const double sine = Length(axis);
        if (sine < 1e-12 && cosine < 0) {
            const auto fallbackAxis = Orthogonal(*a);
            return Quaternion{fallbackAxis.x, fallbackAxis.y, fallbackAxis.z, 0};
        }
        if (sine < 1e-12) return Quaternion{};
        const double halfAngle = std::atan2(sine, cosine) * 0.5;
        const auto imaginary = axis * (std::sin(halfAngle) / sine);
        return Quaternion{imaginary.x, imaginary.y, imaginary.z, std::cos(halfAngle)};
    }

    std::optional<Vec3> Rotate(Quaternion rotation, Vec3 value)
    {
        const auto q = Unit(rotation);
        if (!q || !IsFinite(value)) return std::nullopt;
        const Vec3 imaginary{q->x, q->y, q->z};
        const auto twiceCross = Cross(imaginary, value) * 2.0;
        const auto result = value + twiceCross * q->w + Cross(imaginary, twiceCross);
        return IsFinite(result) ? std::optional{result} : std::nullopt;
    }

    std::optional<Transform> Compose(Transform parent, Transform local)
    {
        const auto parentRotation = Unit(parent.rotation), localRotation = Unit(local.rotation);
        const auto translated = Rotate(parent.rotation, local.position);
        if (!parentRotation || !localRotation || !translated || !IsFinite(parent.position)) return std::nullopt;
        const auto position = parent.position + *translated;
        if (!IsFinite(position)) return std::nullopt;
        return Transform{position, Multiply(*parentRotation, *localRotation)};
    }

    std::optional<Transform> RelativeTo(Transform parent, Transform world)
    {
        const auto parentRotation = Unit(parent.rotation), worldRotation = Unit(world.rotation);
        if (!parentRotation || !worldRotation || !IsFinite(parent.position)) return std::nullopt;
        const auto inverse = Conjugate(*parentRotation);
        const auto localPosition = Rotate(inverse, world.position - parent.position);
        if (!localPosition) return std::nullopt;
        return Transform{*localPosition, Multiply(inverse, *worldRotation)};
    }

    Quaternion MirrorBoneRotation(Quaternion rotation, BoneMirrorFrame source, BoneMirrorFrame destination, Vec3 normal)
    {
        const auto sourceDelta = Multiply(rotation, Conjugate(source.local));
        const auto modelDelta = Multiply(Multiply(source.parent, sourceDelta), Conjugate(source.parent));
        // A reflection reverses the perpendicular components of an axial vector.
        const Vec3 axis{modelDelta.x, modelDelta.y, modelDelta.z};
        const auto reflected = normal * (2 * Dot(normal, axis)) - axis;
        const Quaternion mirrored{reflected.x, reflected.y, reflected.z, modelDelta.w};
        const auto inParent = Multiply(Multiply(Conjugate(destination.parent), mirrored), destination.parent);
        return Multiply(inParent, destination.local);
    }

    std::optional<Vec3> LocalTranslationOffset(Vec3 worldOffset, Quaternion parentRotation, double parentScale)
    {
        const auto parent = Unit(parentRotation);
        if (!parent || !std::isfinite(parentScale) || parentScale <= 0) return std::nullopt;
        const auto local = Rotate(Conjugate(*parent), worldOffset / parentScale);
        if (local && IsFinite(*local)) return local;
        return std::nullopt;
    }

    Vec3 PoseRefreshOffset(Vec3 position, Vec3 direction, double strength)
    {
        const auto unit = Unit(direction);
        if (!unit || !IsFinite(position) || !std::isfinite(strength) || strength <= 0) return {};
        const std::array coordinates{position.x, position.y, position.z};
        const std::array axes{unit->x, unit->y, unit->z};
        constexpr double scale = 16;
        constexpr int minimum = -32768, maximum = 32767;
        const auto infinity = std::numeric_limits<float>::infinity();
        double closest = std::numeric_limits<double>::infinity();
        double shortest = std::numeric_limits<double>::infinity();
        Vec3 fallback{};
        constexpr std::array<Vec3, 3> basis{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        for (std::size_t i = 0; i < axes.size(); ++i) {
            const auto cell = std::round(coordinates[i] * scale);
            if (cell < minimum || cell > maximum) continue;
            for (const int side : {-1, 1}) {
                if (cell + side < minimum || cell + side > maximum) continue;
                // Cross the rounding boundary even after converting the result back to a float.
                const auto boundary = static_cast<float>((cell + side * .5) / scale);
                const auto pastBoundary = std::nextafter(boundary, side * infinity);
                const double offset = pastBoundary - coordinates[i];
                if (std::abs(offset) < shortest) {
                    shortest = std::abs(offset);
                    fallback = basis[i] * offset;
                }
                if (std::abs(axes[i]) > 1e-6) {
                    const double distance = offset / axes[i];
                    if (std::abs(distance) < std::abs(closest)) closest = distance;
                }
            }
        }
        if (std::abs(closest) <= strength) return *unit * closest;
        // A diagonal sightline can leave the rounding cell later than its nearest face.
        return shortest <= strength ? fallback : Vec3{};
    }

    std::optional<Transform> RotateAround(Transform world, Vec3 pivot, Quaternion rotation)
    {
        const auto turn = Unit(rotation);
        if (!turn || !IsFinite(pivot)) return std::nullopt;
        const auto offset = Rotate(*turn, world.position - pivot);
        const auto orientation = Unit(Multiply(*turn, world.rotation));
        if (!offset || !orientation || !IsFinite(pivot + *offset)) return std::nullopt;
        return Transform{pivot + *offset, *orientation};
    }

    Quaternion AxisAngle(Vec3 unitAxis, double angle)
    {
        const auto v = unitAxis * std::sin(angle * 0.5);
        return {v.x, v.y, v.z, std::cos(angle * 0.5)};
    }

    Quaternion CameraLookDelta(Vec3 forward, Vec3 right, double yaw, double pitch)
    {
        const auto turn = AxisAngle({0, 0, 1}, yaw);
        constexpr double limit = std::numbers::pi / 2 - 0.001;
        const auto elevation = std::asin(std::clamp(forward.z, -1.0, 1.0));
        const auto tilt = std::clamp(elevation + pitch, -limit, limit) - elevation;
        return Multiply(AxisAngle(*Rotate(turn, right), tilt), turn);
    }

    std::optional<TwoBoneResult> SolveTwoBone(const TwoBoneInput& input)
    {
        for (const auto point : {input.root, input.joint, input.tip, input.target, input.pole}) {
            if (!IsFinite(point)) return std::nullopt;
        }
        if (input.previousBend && !IsFinite(*input.previousBend)) return std::nullopt;
        const double upper = Length(input.joint - input.root), lower = Length(input.tip - input.joint);
        const double reach = upper + lower;
        if (!std::isfinite(reach) || upper < 1e-8 || lower < 1e-8) return std::nullopt;
        const auto requested = input.target - input.root;
        const double requestedDistance = Length(requested);
        if (!std::isfinite(requestedDistance)) return std::nullopt;
        auto direction = Unit(requested);
        if (!direction) direction = Unit(input.tip - input.root);
        if (!direction) direction = Unit(input.joint - input.root);
        if (!direction) return std::nullopt;

        // A tiny positive radius avoids division by zero at a fully folded equal-length chain.
        const double minimum = std::max(std::abs(upper - lower), reach * 1e-8);
        const double distance = std::clamp(requestedDistance, minimum, reach);
        auto bend = ProjectBend(input.pole - input.root, *direction, reach * 1e-6);
        if (!bend && input.previousBend) bend = ProjectBend(*input.previousBend, *direction, 1e-6);
        if (!bend) bend = ProjectBend(input.joint - input.root, *direction, reach * 1e-6);
        if (!bend) bend = Orthogonal(*direction);

        // Work in units of total reach to keep the squared terms bounded.
        const double u = upper / reach, l = lower / reach, d = distance / reach;
        const double along = (u * u - l * l + d * d) / (2.0 * d);
        const double away = std::sqrt(std::max(0.0, u * u - along * along));
        const auto joint = input.root + (*direction * along + *bend * away) * reach;
        const auto tip = input.root + *direction * distance;
        if (!IsFinite(joint) || !IsFinite(tip)) return std::nullopt;
        const auto upperSwing = FromTo(input.joint - input.root, joint - input.root);
        const auto lowerSwing = FromTo(input.tip - input.joint, tip - joint);
        if (!upperSwing || !lowerSwing) return std::nullopt;
        return TwoBoneResult{joint, tip, *bend, *upperSwing, *lowerSwing,
            requestedDistance < minimum || requestedDistance > reach};
    }

    std::optional<Vec3> IntersectPlane(Ray ray, Vec3 point, Vec3 normal)
    {
        const auto direction = Unit(ray.direction), planeNormal = Unit(normal);
        if (!direction || !planeNormal || !IsFinite(ray.origin) || !IsFinite(point)) return std::nullopt;
        const double denominator = Dot(*direction, *planeNormal);
        if (std::abs(denominator) < 1e-6) return std::nullopt;
        const double distance = Dot(point - ray.origin, *planeNormal) / denominator;
        if (!std::isfinite(distance) || distance < 0) return std::nullopt;
        const auto result = ray.origin + *direction * distance;
        return IsFinite(result) ? std::optional{result} : std::nullopt;
    }

    std::optional<DragPlane> BeginDrag(Ray ray, Vec3 handle, Vec3 cameraForward)
    {
        const auto normal = Unit(cameraForward);
        const auto hit = IntersectPlane(ray, handle, cameraForward);
        if (!normal || !hit) return std::nullopt;
        const auto offset = handle - *hit;
        if (!IsFinite(offset)) return std::nullopt;
        return DragPlane{handle, *normal, offset};
    }

    std::optional<Vec3> DragTarget(const DragPlane& drag, Ray ray, double depthOffset)
    {
        const auto normal = Unit(drag.normal);
        if (!normal || !IsFinite(drag.grabOffset) || !std::isfinite(depthOffset)) return std::nullopt;
        const auto hit = IntersectPlane(ray, drag.point + *normal * depthOffset, *normal);
        if (!hit) return std::nullopt;
        const auto result = *hit + drag.grabOffset;
        return IsFinite(result) ? std::optional{result} : std::nullopt;
    }

    std::optional<std::uint32_t> PickHandle(std::span<const ScreenHandle> handles,
        double x, double y, double radius)
    {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(radius) || radius < 0) return std::nullopt;
        std::optional<std::uint32_t> selected;
        double nearest = radius, depth = std::numeric_limits<double>::infinity();
        for (const auto& handle : handles) {
            if (!std::isfinite(handle.x) || !std::isfinite(handle.y) ||
                !std::isfinite(handle.cameraDepth) || handle.cameraDepth <= 0) continue;
            const double distance = std::hypot(handle.x - x, handle.y - y);
            if (distance < nearest || (distance == nearest && handle.cameraDepth < depth)) {
                selected = handle.id;
                nearest = distance;
                depth = handle.cameraDepth;
            }
        }
        return selected;
    }
}
