#pragma once

#include <cstdint>
#include <optional>
#include <span>

namespace GModPose
{
    struct Vec3
    {
        double x{}, y{}, z{};
        Vec3 operator+(Vec3 b) const;
        Vec3 operator-(Vec3 b) const;
        Vec3 operator*(double scale) const;
        Vec3 operator/(double scale) const;
    };

    double Dot(Vec3 a, Vec3 b);
    Vec3 Cross(Vec3 a, Vec3 b);
    double Length(Vec3 value);
    bool IsFinite(Vec3 value);
    std::optional<Vec3> Unit(Vec3 value);

    struct Quaternion
    {
        double x{}, y{}, z{}, w{1.0};
    };

    std::optional<Quaternion> Unit(Quaternion value);
    Quaternion Multiply(Quaternion a, Quaternion b);
    std::optional<Quaternion> FromTo(Vec3 from, Vec3 to);
    std::optional<Vec3> Rotate(Quaternion rotation, Vec3 value);

    struct BoneMirrorFrame { Quaternion local, parent; };
    Quaternion MirrorBoneRotation(Quaternion rotation, BoneMirrorFrame source, BoneMirrorFrame destination, Vec3 normal);

    struct Transform
    {
        Vec3 position;
        Quaternion rotation;
    };

    // Rigid transforms only. Engine scale and skeleton bind offsets belong in the adapter.
    std::optional<Transform> Compose(Transform parentWorld, Transform childLocal);
    std::optional<Transform> RelativeTo(Transform parentWorld, Transform childWorld);
    std::optional<Vec3> LocalTranslationOffset(Vec3 worldOffset, Quaternion parentRotation, double parentScale);
    Vec3 PoseRefreshOffset(Vec3 position, Vec3 direction, double strength);
    std::optional<Transform> RotateAround(Transform world, Vec3 pivot, Quaternion rotation);
    Quaternion AxisAngle(Vec3 unitAxis, double angle);
    Quaternion CameraLookDelta(Vec3 forward, Vec3 right, double yaw, double pitch);

    struct TwoBoneInput
    {
        Vec3 root, joint, tip, target, pole;
        std::optional<Vec3> previousBend;
    };

    struct TwoBoneResult
    {
        Vec3 joint, tip, bend;
        // World-space swing deltas from the input segments, not skeleton local rotations.
        Quaternion upperSwingWorld, lowerSwingWorld;
        bool targetClamped{};
    };

    std::optional<TwoBoneResult> SolveTwoBone(const TwoBoneInput& input);

    struct Ray
    {
        Vec3 origin, direction;
    };

    std::optional<Vec3> IntersectPlane(Ray ray, Vec3 point, Vec3 normal);

    struct DragPlane
    {
        Vec3 point, normal, grabOffset;
    };

    // Capture the plane once when grabbing. Camera motion must not replace its normal.
    std::optional<DragPlane> BeginDrag(Ray ray, Vec3 handle, Vec3 cameraForward);
    std::optional<Vec3> DragTarget(const DragPlane& drag, Ray ray, double depthOffset = 0.0);

    struct ScreenHandle
    {
        std::uint32_t id{};
        double x{}, y{}, cameraDepth{};
    };

    // Pixel-distance picking keeps handles equally usable at different camera distances.
    std::optional<std::uint32_t> PickHandle(std::span<const ScreenHandle> handles,
        double cursorX, double cursorY, double radiusPixels);
}
