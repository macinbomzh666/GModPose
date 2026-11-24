#pragma once
#include <RE/Skyrim.h>
#include <bitset>

namespace GModPose::EditorKeys
{
    using Key = RE::BSKeyboardDevice::Key;
    using Mouse = RE::BSWin32MouseDevice::Key;

    inline bool WheelButton(std::uint32_t code)
    {
        return code == Mouse::kWheelUp || code == Mouse::kWheelDown;
    }

    struct State
    {
        std::bitset<256> keys;
        std::bitset<8> mouse;
        std::bitset<256> pressedKeys;
        std::bitset<8> pressedMouse;

        void Deliver(RE::INPUT_DEVICE device, std::uint32_t code, bool pressed, bool fresh)
        {
            if (device == RE::INPUT_DEVICE::kKeyboard && code < keys.size() && (!pressed || fresh)) {
                keys.set(code, pressed);
                if (fresh && pressed) pressedKeys.set(code);
            }
            if (device == RE::INPUT_DEVICE::kMouse && code < mouse.size()) {
                mouse.set(code, pressed);
                if (fresh && pressed) pressedMouse.set(code);
            }
        }

        bool Down(std::uint32_t code) const { return code != 0 && code < keys.size() && keys[code]; }
    };
}
