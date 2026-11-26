#pragma once

#include "Tether.h"
#include <chrono>
#include <vector>

namespace GModPose::DragView
{
    struct MiddlePulse
    {
        ScreenSegment span;
        std::chrono::steady_clock::time_point began;

        float Intensity(std::chrono::steady_clock::time_point now) const
        {
            constexpr float duration = 0.15F;
            const auto fade = std::clamp(1 - std::chrono::duration<float>(now - began).count() / duration, 0.0F, 1.0F);
            return fade * fade;
        }
    };

    struct Frame
    {
        bool visible{}, drawWorld{}, limited{}, rotating{}, mirror{}, nudge{}, settings{}, finger{}, face{}, body{};
        std::uint64_t circleHandles{}, triangleHandles{}, plantedHandles{};
        std::vector<std::optional<Vec3>> markers{};
        std::optional<Vec3> orb{};
        std::optional<MiddlePulse> middlePulse{};
        std::optional<std::size_t> hover{}, held{};
        const char* hoverLabel{};
        std::array<ScreenSegment, 48> tether{};
        std::size_t count{};
    };

    struct Camera
    {
        Ray aim;
        Vec3 emitter, right, up;
        std::array<double, 4> x, y, w;
        double nearPlane, width, height;

        Vec3 Clip(Vec3 p) const
        {
            const auto eval = [p](const std::array<double, 4>& row) {
                return row[0] * p.x + row[1] * p.y + row[2] * p.z + row[3];
            };
            return {eval(x), eval(y), eval(w)};
        }

        bool SetAim(Vec3 eye)
        {
            // The reticle is at clip x=y=0. These two planes give the same
            // center ray used to project the handles, even for an offset port.
            const Vec3 horizontal{x[0], x[1], x[2]}, vertical{y[0], y[1], y[2]};
            const auto direction = Unit(Cross(horizontal, vertical));
            const auto across = Unit(horizontal);
            if (!direction || !across || !IsFinite(eye) || !IsFinite(Clip(eye)) ||
                !std::isfinite(nearPlane) || nearPlane <= 0) return false;
            const Vec3 depthRow{w[0], w[1], w[2]};
            const auto depth = Dot(*direction, depthRow);
            if (!std::isfinite(depth) || std::abs(depth) < 1e-6) return false;
            aim = {eye, depth > 0 ? *direction : *direction * -1};
            right = *across;
            const auto verticalAxis = Unit(vertical - right * Dot(right, vertical));
            if (!verticalAxis) return false;
            up = *verticalAxis;

            // Place the particle inside the lower-right quadrant at every FOV.
            // The tether starts at this exact world point, even when its tip is clipped.
            const auto originRay = Unit(Cross(horizontal - depthRow * 0.43, vertical + depthRow * 0.50));
            if (!originRay || std::abs(Dot(*originRay, depthRow)) < 1e-6) return false;
            emitter = eye + *originRay * ((nearPlane + 24) / Dot(*originRay, depthRow));
            return true;
        }

        void Project(std::span<const std::optional<Vec3>> points, Frame& frame) const
        {
            std::vector<ScreenHandle> candidates;
            candidates.reserve(points.size());
            frame.markers.assign(points.size(), std::nullopt);
            for (std::size_t i = 0; i < points.size(); ++i) {
                if (!points[i]) continue;
                const auto clip = Clip(*points[i]);
                const auto point = ClipTetherSegment(clip, clip, nearPlane);
                if (!point) continue;
                frame.markers[i] = point->start;
                candidates.push_back({static_cast<std::uint32_t>(i), point->start.x * width * 0.5,
                                       point->start.y * height * 0.5, clip.z});
            }
            frame.hover = PickHandle(candidates, 0, 0,
                                     22 * std::clamp(height / 1080, 0.75, 2.0));
        }

        void ProjectMiddlePulse(Frame& frame) const
        {
            const auto source = Clip(emitter);
            if (const auto start = ClipTetherSegment(source, source, nearPlane)) {
                const Vec3 tip = frame.hover ? frame.markers[*frame.hover].value_or(Vec3{}) : Vec3{};
                frame.middlePulse = MiddlePulse{{start->start, tip}, std::chrono::steady_clock::now()};
            }
        }

        void ProjectTether(const TetherCurve& curve, Vec3 tip, Frame& frame) const
        {
            frame.count = 0;
            auto previous = Clip(emitter);
            if (const auto point = ClipTetherSegment(previous, previous, nearPlane)) frame.orb = point->start;
            for (std::size_t i = 1; i <= frame.tether.size(); ++i) {
                const double t = static_cast<double>(i) / static_cast<double>(frame.tether.size());
                const auto point = Clip(curve.Point(emitter, tip, t));
                if (const auto segment = ClipTetherSegment(previous, point, nearPlane))
                    frame.tether[frame.count++] = *segment;
                previous = point;
            }
        }
    };

    bool Install();
    bool Available();
    void DrawOverlay(const Frame& view, float width, float height, float framebufferScale, float pulseStrength);
    const char* InputBlockReason();
    std::optional<Camera> ReadCamera();
    void Publish(Frame frame);
    void Clear();
}
