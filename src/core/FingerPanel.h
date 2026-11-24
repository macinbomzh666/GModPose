#pragma once

#include "FingerPose.h"
#include <vector>

namespace GModPose::FingerPanel
{
    struct Edit { Fingers::Pose pose; Fingers::Mask changed; bool checkpoint{}; Fingers::Mask mirrored{}; };

    void Load();
    void Begin(Fingers::Side side, const Fingers::Pose& initial, Fingers::Mask available,
        Fingers::Mask mirrorAvailable, bool mirror);
    void End();
    void ShowPose(const Fingers::Pose& pose);
    std::vector<Edit> TakeEdits(bool finish);
    void Draw(float width, float height);
}
