#pragma once

#include <cstdint>

namespace RE
{
    class Actor;
    class TESObjectREFR;
}

namespace GModPose::SeatBlock
{
    void Install();
    void Begin(const RE::TESObjectREFR& furniture, std::uint32_t marker);
    void End();
    bool TakenByOther(const RE::TESObjectREFR& furniture, std::uint32_t marker, RE::Actor& player);
}
