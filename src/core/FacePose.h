#pragma once
#include "PoseMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <utility>

namespace GModPose::FacePose
{
    // FaceGen's first 14 modifier slots are facial controls. The remaining
    // three are head rotation, which the body editor already controls.
    enum Channel : std::size_t {
        BlinkLeft, BlinkRight, BrowDownLeft, BrowDownRight, BrowInLeft, BrowInRight,
        BrowUpLeft, BrowUpRight, LookDown, LookLeft, LookRight, LookUp, SquintLeft, SquintRight,
        Aah, BigAah, BMP, ChJSh, DST, Eee, Eh, FV, I, K, N, Oh, OohQ, R, Th, W,
        MoodSlot
    };
    constexpr std::size_t modifierCount = Aah, phonemeCount = MoodSlot - Aah, weightCount = MoodSlot;
    using Mask = std::uint32_t;
    constexpr Mask Bit(Channel channel) { return Mask{1} << channel; }
    constexpr Mask Range(Channel first, Channel last) { return (Bit(last) - 1) & ~(Bit(first) - 1); }
    constexpr Mask blink = Bit(BlinkLeft) | Bit(BlinkRight);
    constexpr Mask lids = blink | Bit(SquintLeft) | Bit(SquintRight);
    constexpr Mask gazeMask = Range(LookDown, SquintLeft);
    constexpr Mask brows = Range(BrowDownLeft, LookDown);
    constexpr Mask mouth = Range(Aah, MoodSlot);
    constexpr Mask all = (Bit(MoodSlot) << 1) - 1;
    constexpr unsigned neutralMood = 7, moodCount = 8;

    struct State
    {
        std::array<float, weightCount> weights{};
        unsigned mood{neutralMood};
        float strength{};

        constexpr float& operator[](Channel channel) { return weights[channel]; }
        constexpr bool operator==(const State&) const = default;
    };

    using Expressions = std::array<float, 17>;
    struct Edit { State pose; Mask changed{}; Expressions expressions; };

    inline std::optional<State> AimGaze(Vec3 target, Vec3 right, Vec3 forward, Vec3 up)
    {
        if (!Unit(target)) return std::nullopt;
        const auto across = Dot(target, right), ahead = Dot(target, forward), height = Dot(target, up);
        constexpr auto yawRange = 35 * std::numbers::pi / 180;
        constexpr auto pitchRange = 25 * std::numbers::pi / 180;
        const auto yaw = std::clamp(std::atan2(across, ahead) / yawRange, -1.0, 1.0);
        const auto pitch = std::clamp(std::atan2(height, std::hypot(across, ahead)) / pitchRange, -1.0, 1.0);
        State gaze;
        gaze[LookLeft] = static_cast<float>(std::max(0.0, -yaw));
        gaze[LookRight] = static_cast<float>(std::max(0.0, yaw));
        gaze[LookUp] = static_cast<float>(std::max(0.0, pitch));
        gaze[LookDown] = static_cast<float>(std::max(0.0, -pitch));
        return gaze;
    }

    struct Template
    {
        const char* name;
        State pose;
        Mask region;
        enum Art { OpenEyes, ClosedEyes, WinkLeft, WinkRight, Squint, RestBrow, RaisedBrow,
            Furrow, RestLips, Grin, Frown, OpenMouth, Pressed, Pout } art;
    };

    inline constexpr auto Shape(std::array<float, weightCount> weights) { return State{weights}; }

    constexpr auto With(Channel id, float value, std::array<float, weightCount> weights = {})
    {
        weights[id] = value;
        return weights;
    }

    inline constexpr std::array eyeTemplates{
        Template{"Closed", Shape(With(BlinkLeft, 1, With(BlinkRight, 1))), lids, Template::ClosedEyes},
        Template{"Left wink", Shape(With(BlinkLeft, 1)), lids, Template::WinkLeft},
        Template{"Open", {}, lids, Template::OpenEyes},
        Template{"Right wink", Shape(With(BlinkRight, 1)), lids, Template::WinkRight},
        Template{"Squint", Shape(With(SquintLeft, .74F, With(SquintRight, .74F))), lids, Template::Squint},
    };
    inline constexpr std::array browTemplates{
        Template{"Furrowed", Shape(With(BrowDownLeft, .65F, With(BrowDownRight, .65F,
            With(BrowInLeft, .60F, With(BrowInRight, .60F))))), brows, Template::Furrow},
        Template{"Raised", Shape(With(BrowUpLeft, .80F, With(BrowUpRight, .80F))), brows, Template::RaisedBrow},
        Template{"Relaxed", {}, brows, Template::RestBrow},
    };
    inline constexpr std::array mouthTemplates{
        Template{"Grin", Shape(With(Eee, .78F, With(Eh, .14F))), mouth, Template::Grin},
        Template{"Open", Shape(With(BigAah, .82F)), mouth, Template::OpenMouth},
        Template{"Pout", Shape(With(OohQ, .76F, With(W, .18F))), mouth, Template::Pout},
        Template{"Pressed", Shape(With(BMP, .82F)), mouth, Template::Pressed},
        Template{"Rest", {}, mouth, Template::RestLips},
    };

    inline constexpr std::array moodLabels{"Neutral", "Angry", "Afraid", "Smile", "Frown",
        "Surprised", "Puzzled", "Disgusted"};
    inline constexpr std::array<unsigned, moodCount> moodOrder{2, 1, 7, 4, 0, 6, 3, 5};
    inline constexpr std::array moodStrength{0.0F, .82F, .72F, .80F, .82F, .78F, .72F, .75F};

    inline constexpr State Expression(unsigned index)
    {
        return index < moodCount ? State{{}, neutralMood + index, moodStrength[index]} : State{};
    }

    inline constexpr std::array moodTemplates{
        Template{"Frown", Expression(4), Bit(MoodSlot), Template::Frown},
        Template{"Neutral", Expression(0), Bit(MoodSlot), Template::RestLips},
        Template{"Smile", Expression(3), Bit(MoodSlot), Template::Grin},
    };

    inline void Merge(State& destination, const State& source, Mask mask)
    {
        for (std::size_t i = 0; i < destination.weights.size(); ++i)
            if (mask & (Mask{1} << i)) destination.weights[i] = source.weights[i];
        if (mask & Bit(MoodSlot)) {
            destination.mood = source.mood;
            destination.strength = source.strength;
        }
    }
}
