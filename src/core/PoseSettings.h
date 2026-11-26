#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include "PoseHistory.h"

struct ID3D11Device;
struct ID3D11DeviceContext;
struct ImGuiContext;

namespace GModPose::DragView { struct Frame; }

namespace GModPose::PoseSettings
{
    enum class Action : std::size_t {
        FlyForward, FlyBack, FlyLeft, FlyRight, SlowFlight,
        PoseForward, PoseBack, PoseLeft, PoseRight, Raise, Lower, Clockwise, Counterclockwise,
        ToggleEditor, ToggleGuide, ToggleSettings, Mirror, Undo, Redo, Grab, Rotate, OpenPoser, Nudge, WheelUp, WheelDown, Count
    };

    constexpr unsigned mouseBase = 256, mouseButtons = 8;
    constexpr unsigned MouseBinding(unsigned id) { return id < mouseButtons ? mouseBase + id : 0; }

    enum class Corner { UpperLeft, UpperRight, LowerLeft, LowerRight };

    struct Bindings
    {
        std::array<unsigned, static_cast<std::size_t>(Action::Count)> keys{};
        std::array<unsigned, static_cast<std::size_t>(Action::Count)> alternates{};
        Corner corner{Corner::UpperLeft};
        bool useWheel{true};
        bool breathing{};
        bool invertYaw{}, invertPitch{}, invertRoll{};
        int uiScale{100};
        int rotationSensitivityPercent{100};

        unsigned Key(Action action) const { return keys[static_cast<std::size_t>(action)]; }
        unsigned Alternate(Action action) const { return alternates[static_cast<std::size_t>(action)]; }
        bool Matches(Action action, unsigned code) const
        {
            return code && (code == Key(action) || code == Alternate(action));
        }
    };

    void Load();
    Bindings Get();
    bool RebindKey(unsigned code);
    void ResetMenu();
    bool PresetNameActive();
    bool PresetNameInput(const char* id, const char* hint, char* name, std::size_t size);
    std::optional<HistoryAction> HistoryButtons(bool canUndo, bool canRedo);
    struct TextEvent { std::uint32_t code; bool character, down; };
    void QueuePointer(double x, double y, double wheel, std::span<const std::uint8_t> clicks,
        std::span<const TextEvent> text);

    ImGuiContext* CreateUI(ID3D11Device* device, ID3D11DeviceContext* context);
    void DestroyUI(ImGuiContext* context);
    void DrawUI(const DragView::Frame& view, float frameWidth, float frameHeight, float pulseStrength);
}
