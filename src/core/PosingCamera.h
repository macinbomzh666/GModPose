#pragma once

namespace RE { class PlayerCharacter; }

namespace GModPose::PosingCamera
{
    struct FlightInput
    {
        double mouseX{}, mouseY{};
        int forward{}, right{};
        bool slow{}, rotating{};
    };

    bool Install();
    // Begin, Update and End run exclusively on the bound game thread.
    void Begin(RE::PlayerCharacter& player, bool seated);
    void Update();
    void Fly(const FlightInput& input, double dt);
    void End(RE::PlayerCharacter* player);
    void WorldChanged();
}
