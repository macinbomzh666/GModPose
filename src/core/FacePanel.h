#pragma once

#include "FacePose.h"
#include <optional>
#include <vector>

namespace GModPose::FacePanel
{
    void Load();
    void Begin(const FacePose::State& pose, FacePose::Mask supported, const FacePose::Expressions& expressions);
    void End();
    struct Change { FacePose::Edit edit; bool checkpoint{}; };
    void ShowPose(const FacePose::State& pose, const FacePose::Expressions& expressions);
    void SetCameraGaze(std::optional<FacePose::State> gaze);
    std::vector<Change> TakeEdits(bool finish);
    void Draw(float width, float height);
}

namespace RE { class PlayerCharacter; }

namespace GModPose::FaceActor
{
    struct Sample {
        FacePose::State pose;
        FacePose::Mask supported;
        FacePose::Expressions expressions{};
    };
    std::optional<Sample> Snapshot();
    bool Update(RE::PlayerCharacter& player, const std::optional<FacePose::Edit>& edit);
    bool OwnsFace() noexcept;
    void End(RE::PlayerCharacter* player);
}
