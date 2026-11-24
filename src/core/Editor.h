#pragma once
#include "PoseHistory.h"
#include <string>

namespace RE { class ControlMap; }

namespace GModPose::Runtime
{
    bool InstallEarlyUpdate();
    bool Install();
    bool Running();
    bool RotatingHandle();
    bool SettingsOpen();
    void CloseSettings();
    void RequestHistory(HistoryAction action);
    bool CanUseHistory(HistoryAction action);
    struct BodyPresetCommand
    {
        enum class Kind { Load, Save, Overwrite };
        Kind kind;
        std::string name;
    };
    struct BodyPresetView
    {
        struct Entry { std::string name; bool premade; };
        std::vector<Entry> entries;
        std::string status;
    };
    BodyPresetView ReadBodyPresets();
    void RequestBodyPreset(BodyPresetCommand command);
    bool OwnsTextInput(const RE::ControlMap* controls);
    void Invalidate();
    void SetLoading(bool loading);
}
