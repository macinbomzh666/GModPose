#pragma once
#include <array>
#include <optional>
#include <span>

namespace GModPose
{
    enum class MouseEdge { None, Up, Down };

    struct GrabButton
    {
        bool held{}, armed{}, previousRaw{};

        // A delivered transition wins over a stale sample. On entering the
        // editor or regaining focus require a release before picking anything.
        bool Update(bool focused, std::optional<bool> raw, MouseEdge edge)
        {
            const bool wasDown = previousRaw;
            previousRaw = raw.value_or(false);
            if (!focused) {
                armed = held = false;
                return false;
            }
            if (!armed) {
                armed = edge == MouseEdge::Up || (!previousRaw && edge == MouseEdge::None);
                return false;
            }
            if (edge == MouseEdge::Down) {
                const bool pressed = !held;
                held = true;
                return pressed;
            }
            if (edge == MouseEdge::Up) held = false;
            else if (raw && *raw != wasDown) {
                const bool pressed = *raw && !held;
                held = *raw;
                return pressed;
            }
            return false;
        }
    };

    enum class EditorButton { Grab, Rotate, Guide, Settings, Nudge, Count };
    struct ButtonChange
    {
        EditorButton button;
        MouseEdge edge;
        bool operator==(const ButtonChange&) const = default;
    };

    struct EditorButtons
    {
        static constexpr auto count = static_cast<std::size_t>(EditorButton::Count);
        using Samples = std::array<std::optional<bool>, count>;
        std::array<GrabButton, count> buttons{};

        bool Held(EditorButton button) const { return buttons[static_cast<std::size_t>(button)].held; }

        // Deliver native transitions in order across buttons. Poll only the buttons
        // without a delivered edge this tick, since a device sample can lag an event.
        template <class OnChange>
        void Update(bool focused, const Samples& raw, std::span<const ButtonChange> changes, OnChange onChange)
        {
            std::array<bool, count> delivered{};
            const auto update = [&](EditorButton button, MouseEdge edge) {
                const auto i = static_cast<std::size_t>(button);
                const bool held = buttons[i].held;
                if (buttons[i].Update(focused, raw[i], edge)) onChange(button, MouseEdge::Down);
                else if (held && !buttons[i].held) onChange(button, MouseEdge::Up);
            };
            for (const auto change : changes) {
                delivered[static_cast<std::size_t>(change.button)] = true;
                update(change.button, change.edge);
            }
            for (std::size_t i = 0; i < count; ++i)
                if (!delivered[i]) update(static_cast<EditorButton>(i), MouseEdge::None);
        }
    };
}
