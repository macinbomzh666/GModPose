#include <RE/Skyrim.h>
#include <SKSE/SKSE.h>
#include <Windows.h>
#include <imgui.h>
#include <backends/imgui_impl_dx11.h>
#include "PoseSettings.h"
#include "PresetStorage.h"
#include "Editor.h"
#include "../Version.h"
#include "FingerPanel.h"
#include "FacePanel.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <locale>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace GModPose::PoseSettings
{
    namespace
    {
        using Key = RE::BSKeyboardDevice::Key;
        // ImGui publishes focus when it draws the field. The input hook uses the
        // published value and never queries ImGui from the input thread.
        std::atomic_bool presetNameActive{false};
        thread_local bool presetNameDrawnActive = false;
        constexpr auto Index(Action action) { return static_cast<std::size_t>(action); }

        struct ActionInfo
        {
            const wchar_t* id;
            const char* label;
            unsigned key;
            unsigned alternate{};
            const char* gameMapping{};
        };

        constexpr std::array<ActionInfo, Index(Action::Count)> actions{{
            {L"FlyForward", "Fly forward", Key::kW, 0, "Forward"},
            {L"FlyBack", "Fly backward", Key::kS, 0, "Back"},
            {L"FlyLeft", "Fly left", Key::kA, 0, "Strafe Left"},
            {L"FlyRight", "Fly right", Key::kD, 0, "Strafe Right"},
            {L"SlowFlight", "Slow flight", Key::kLeftShift, Key::kRightShift},
            {L"PoseForward", "Slide pose forward", Key::kUp},
            {L"PoseBack", "Slide pose backward", Key::kDown},
            {L"PoseLeft", "Slide pose left", Key::kLeft},
            {L"PoseRight", "Slide pose right", Key::kRight},
            {L"Raise", "Raise pose", Key::kSpacebar, 0, "Jump"},
            {L"Lower", "Lower pose", Key::kLeftControl, Key::kRightControl},
            {L"Clockwise", "Rotate clockwise", Key::kMinus, Key::kKP_Subtract},
            {L"Counterclockwise", "Rotate counterclockwise", Key::kEquals, Key::kKP_Plus},
            {L"ToggleEditor", "Enter / exit posing", Key::kApostrophe},
            {L"ToggleGuide", "Show / hide posing HUD", Key::kSlash},
            {L"ToggleSettings", "Open / close settings", Key::kSemicolon},
            {L"Mirror", "Toggle mirror mode", Key::kBackslash},
            {L"Undo", "Undo pose action", Key::kComma},
            {L"Redo", "Redo pose action", Key::kPeriod},
            {L"Grab", "Drag", MouseBinding(0)},
            {L"Rotate", "Rotate handle", MouseBinding(1)},
            {L"OpenPoser", "Open poser", MouseBinding(2)},
            {L"Nudge", "Toggle nudge mode", Key::kLeftAlt},
            {L"WheelUp", "Wheel up action", Key::kPageUp},
            {L"WheelDown", "Wheel down action", Key::kPageDown},
        }};
        constexpr std::array corners{"Upper left", "Upper right", "Lower left", "Lower right"};
        constexpr int minUiScale = 75, maxUiScale = 200;
        constexpr int minRotationSensitivity = 10, maxRotationSensitivity = 200;
        struct InversionOption
        {
            const wchar_t* key;
            const char* label;
            bool Bindings::* enabled;
        };
        constexpr std::array inversionOptions{
            InversionOption{L"InvertYaw", "Invert yaw", &Bindings::invertYaw},
            InversionOption{L"InvertPitch", "Invert pitch", &Bindings::invertPitch},
            InversionOption{L"InvertRoll", "Invert roll", &Bindings::invertRoll},
        };
        std::mutex mutex;
        Bindings defaults, bindings;
        std::filesystem::path file;
        std::optional<Action> capture;
        std::string notice;
        int uiScaleDraft{};
        bool adjustingUiScale{};
        int rotationSensitivityDraft{};
        bool adjustingRotationSensitivity{};
        struct BodyForm
        {
            std::array<char, PosePresetJson::nameBufferSize> name{};
            std::optional<std::string> overwrite;
        };
        thread_local BodyForm bodyForm;
        struct Pointer
        {
            double x{}, y{}, wheel{};
            std::vector<std::uint8_t> clicks;
            std::vector<TextEvent> text;
            bool reset{};
        } pending;

        bool PointerAction(Action action) { return action >= Action::Grab && action < Action::Count; }

        bool Valid(Action action, unsigned code)
        {
            if (code == 0) return action == Action::WheelUp || action == Action::WheelDown ||
                action == Action::Mirror || action == Action::Undo || action == Action::Redo || action == Action::Nudge;
            if (code >= mouseBase) return PointerAction(action) && code < mouseBase + mouseButtons;
            return code < 255 && code != Key::kEscape;
        }

        void UpdateAlternates(Bindings& value)
        {
            for (std::size_t i = 0; i < actions.size(); ++i) {
                const auto alternate = actions[i].alternate;
                value.alternates[i] = value.keys[i] == defaults.keys[i] && alternate &&
                    std::ranges::find(value.keys, alternate) == value.keys.end() ? alternate : 0;
            }
        }

        std::optional<unsigned> ReadSetting(const std::filesystem::path& path, const wchar_t* section, const wchar_t* name)
        {
            wchar_t text[64]{};
            const auto size = ::GetPrivateProfileStringW(section, name, L"", text, static_cast<DWORD>(std::size(text)), path.c_str());
            if (!size || size == std::size(text) - 1) return std::nullopt;
            std::wistringstream input(text);
            input.imbue(std::locale::classic());
            long long value{};
            if (!(input >> value) || !(input >> std::ws).eof() ||
                value < 0 || value > std::numeric_limits<unsigned>::max()) return std::nullopt;
            return static_cast<unsigned>(value);
        }

        Bindings ReadBindings(const std::filesystem::path& path, Bindings result)
        {
            for (std::size_t i = 0; i < actions.size(); ++i) {
                const auto key = ReadSetting(path, L"Controls", actions[i].id);
                if (key && Valid(static_cast<Action>(i), *key)) result.keys[i] = *key;
            }
            const auto wheel = ReadSetting(path, L"Controls", L"UseMouseWheel");
            if (wheel && *wheel <= 1) result.useWheel = *wheel == 1;
            const auto breathing = ReadSetting(path, L"Controls", L"BreathingMovement");
            if (breathing && *breathing <= 1) result.breathing = *breathing == 1;
            const auto sensitivity = ReadSetting(path, L"Controls", L"RotationSensitivityPercent");
            if (sensitivity && *sensitivity >= minRotationSensitivity && *sensitivity <= maxRotationSensitivity)
                result.rotationSensitivityPercent = static_cast<int>(*sensitivity);
            for (const auto& option : inversionOptions)
                if (const auto value = ReadSetting(path, L"Controls", option.key); value && *value <= 1)
                    result.*option.enabled = *value == 1;
            const auto corner = ReadSetting(path, L"Display", L"GuideCorner");
            if (corner && *corner < corners.size()) result.corner = static_cast<Corner>(*corner);
            const auto scale = ReadSetting(path, L"Display", L"UIScalePercent");
            if (scale && *scale >= minUiScale && *scale <= maxUiScale) result.uiScale = static_cast<int>(*scale);
            return result;
        }

        void Save()
        {
            if (file.empty()) return;
            bool ok = true;
            const auto write = [&](const wchar_t* section, const wchar_t* name, unsigned value) {
                ok = (::WritePrivateProfileStringW(section, name, std::to_wstring(value).c_str(), file.c_str()) != 0) && ok;
            };
            for (std::size_t i = 0; i < actions.size(); ++i) write(L"Controls", actions[i].id, bindings.keys[i]);
            write(L"Controls", L"UseMouseWheel", bindings.useWheel);
            write(L"Controls", L"BreathingMovement", bindings.breathing);
            write(L"Controls", L"RotationSensitivityPercent", bindings.rotationSensitivityPercent);
            for (const auto& option : inversionOptions) write(L"Controls", option.key, bindings.*option.enabled);
            write(L"Display", L"GuideCorner", static_cast<unsigned>(bindings.corner));
            write(L"Display", L"UIScalePercent", bindings.uiScale);
            if (!ok) {
                notice = "Could not save. Changes apply until Skyrim closes.";
                SKSE::log::warn("Failed to write GModPose.ini");
            }
        }

        std::string KeyName(unsigned code)
        {
            switch (code) {
            case 0: return "Unbound";
            case MouseBinding(0): return "Left mouse";
            case MouseBinding(1): return "Right mouse";
            case MouseBinding(2): return "Middle mouse";
            case Key::kLeftShift: return "L Shift";
            case Key::kRightShift: return "R Shift";
            case Key::kLeftControl: return "L Ctrl";
            case Key::kRightControl: return "R Ctrl";
            case Key::kLeftAlt: return "L Alt";
            case Key::kRightAlt: return "R Alt";
            case Key::kSpacebar: return "Space";
            case Key::kLeft: return "Left";
            case Key::kRight: return "Right";
            case Key::kUp: return "Up";
            case Key::kDown: return "Down";
            case Key::kComma: return ",";
            case Key::kPeriod: return ".";
            case Key::kSlash: return "/";
            case Key::kBackslash: return "\\";
            }
            if (code >= mouseBase && code < mouseBase + mouseButtons)
                return "Mouse " + std::to_string(code - mouseBase + 1);
            const auto scan = static_cast<LONG>(((code & 0x7F) << 16) | ((code & 0x80) ? 1 << 24 : 0));
            wchar_t name[64]{};
            const auto length = ::GetKeyNameTextW(scan, name, static_cast<int>(std::size(name)));
            char utf8[256]{};
            if (length && ::WideCharToMultiByte(CP_UTF8, 0, name, length, utf8, sizeof(utf8), nullptr, nullptr))
                return utf8;
            return "Key " + std::to_string(code);
        }

        void Style()
        {
            auto& s = ImGui::GetStyle();
            ImGui::StyleColorsDark();
            s.WindowRounding = s.ChildRounding = s.FrameRounding = s.PopupRounding = 0;
            s.TabRounding = s.ScrollbarRounding = s.GrabRounding = 0;
            s.WindowBorderSize = s.PopupBorderSize = 1;
            s.FrameBorderSize = s.TabBorderSize = 0;
            s.WindowPadding = {20, 15};
            s.FramePadding = {12, 5};
            s.ItemSpacing = {10, 8};
            s.ItemInnerSpacing = {8, 6};
            s.ScrollbarSize = 12;
            s.WindowTitleAlign = {0.5F, 0.5F};
            s.FontSizeBase = 20;
            auto& c = s.Colors;
            c[ImGuiCol_Text] = {0.92F, 0.93F, 0.91F, 1};
            c[ImGuiCol_TextDisabled] = {0.54F, 0.54F, 0.51F, 1};
            c[ImGuiCol_WindowBg] = {0.03F, 0.035F, 0.04F, 0.96F};
            c[ImGuiCol_ChildBg] = c[ImGuiCol_BorderShadow] = {0, 0, 0, 0};
            c[ImGuiCol_PopupBg] = {0.055F, 0.058F, 0.061F, 0.98F};
            c[ImGuiCol_Border] = {0.49F, 0.48F, 0.44F, 0.8F};
            c[ImGuiCol_TitleBg] = c[ImGuiCol_TitleBgActive] = c[ImGuiCol_TitleBgCollapsed] = {0.065F, 0.068F, 0.071F, 1};
            c[ImGuiCol_FrameBg] = {0.09F, 0.095F, 0.10F, 0.8F};
            c[ImGuiCol_Button] = {0.07F, 0.075F, 0.08F, 0.5F};
            c[ImGuiCol_FrameBgHovered] = c[ImGuiCol_ButtonHovered] = {0.24F, 0.25F, 0.26F, 1};
            c[ImGuiCol_FrameBgActive] = c[ImGuiCol_ButtonActive] = {0.33F, 0.34F, 0.35F, 1};
            c[ImGuiCol_Tab] = c[ImGuiCol_TabDimmed] = {0.07F, 0.075F, 0.08F, 1};
            c[ImGuiCol_TabHovered] = {0.27F, 0.28F, 0.29F, 1};
            c[ImGuiCol_TabSelected] = c[ImGuiCol_TabDimmedSelected] = {0.19F, 0.20F, 0.21F, 1};
            c[ImGuiCol_TabSelectedOverline] = c[ImGuiCol_TabDimmedSelectedOverline] = {0.78F, 0.80F, 0.79F, 1};
            c[ImGuiCol_Header] = {0.16F, 0.17F, 0.18F, 1};
            c[ImGuiCol_HeaderHovered] = {0.25F, 0.26F, 0.27F, 1};
            c[ImGuiCol_HeaderActive] = {0.32F, 0.33F, 0.34F, 1};
            c[ImGuiCol_Separator] = {0.48F, 0.50F, 0.50F, 0.7F};
            c[ImGuiCol_ScrollbarBg] = {0, 0, 0, 0.15F};
            c[ImGuiCol_ScrollbarGrab] = {0.13F, 0.14F, 0.15F, 1};
            c[ImGuiCol_ScrollbarGrabHovered] = {0.23F, 0.24F, 0.25F, 1};
            c[ImGuiCol_ScrollbarGrabActive] = {0.34F, 0.35F, 0.36F, 1};
            c[ImGuiCol_ResizeGrip] = {0.50F, 0.49F, 0.45F, 0.2F};
            c[ImGuiCol_ResizeGripHovered] = {0.68F, 0.67F, 0.61F, 0.6F};
            c[ImGuiCol_ResizeGripActive] = c[ImGuiCol_NavCursor] = {0.83F, 0.81F, 0.74F, 1};
            c[ImGuiCol_TextSelectedBg] = {0.36F, 0.37F, 0.38F, 0.8F};
            c[ImGuiCol_CheckMark] = c[ImGuiCol_NavCursor];
            c[ImGuiCol_SliderGrab] = {0.24F, 0.46F, 0.70F, 1};
            c[ImGuiCol_SliderGrabActive] = {0.42F, 0.68F, 0.95F, 1};
        }

        void LoadFont()
        {
            const auto path = std::filesystem::path(REL::Module::get().filePath().data()).parent_path() /
                L"Data/SKSE/Plugins/GModPose/fonts/Sovngarde Light.ttf";
            const auto utf8 = path.u8string();
            const std::string name(utf8.begin(), utf8.end());
            auto& io = ImGui::GetIO();
            std::error_code error;
            if (std::filesystem::is_regular_file(path, error)) io.FontDefault = io.Fonts->AddFontFromFileTTF(name.c_str(), 20);
            if (!io.FontDefault) {
                SKSE::log::warn("GModPose Sovngarde font unavailable, using the default UI font");
                ImFontConfig fallback;
                fallback.SizePixels = 20;
                io.FontDefault = io.Fonts->AddFontDefaultVector(&fallback);
            }
        }

        struct ContextScope
        {
            ImGuiContext* original = ImGui::GetCurrentContext();
            ~ContextScope() { ImGui::SetCurrentContext(original); }
        };

        void ControlsPanel(float width, float height)
        {
            const std::scoped_lock lock(mutex);
            const ImVec2 maximum{width * .98F, height * .98F};
            ImGui::SetNextWindowPos({width / 2, height / 2}, ImGuiCond_FirstUseEver, {0.5F, 0.5F});
            ImGui::SetNextWindowSize({std::min(790.0F, width * 0.95F), std::min(720.0F, height * 0.94F)},
                ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSizeConstraints({std::min(360.0F, maximum.x), std::min(300.0F, maximum.y)}, maximum);
            bool visible = true;
            if (ImGui::Begin("GMOD Pose v" GMODPOSE_VERSION_TEXT "###GModPoseSettings", &visible,
                    ImGuiWindowFlags_NoCollapse)) {
                ImGui::TextUnformatted("Shortcut guide position");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(150);
                auto selected = static_cast<int>(bindings.corner);
                if (ImGui::Combo("##corner", &selected, corners.data(), static_cast<int>(corners.size()))) {
                    bindings.corner = static_cast<Corner>(selected);
                    notice.clear();
                    Save();
                }
                if (!adjustingUiScale) uiScaleDraft = bindings.uiScale;
                ImGui::SetNextItemWidth(150);
                ImGui::SliderInt("UI scale", &uiScaleDraft, minUiScale, maxUiScale, "%d%%", ImGuiSliderFlags_AlwaysClamp);
                adjustingUiScale = ImGui::IsItemActive();
                if (ImGui::IsItemDeactivatedAfterEdit() && bindings.uiScale != uiScaleDraft) {
                    bindings.uiScale = uiScaleDraft;
                    notice.clear();
                    Save();
                }
                if (!adjustingRotationSensitivity) rotationSensitivityDraft = bindings.rotationSensitivityPercent;
                ImGui::SetNextItemWidth(150);
                ImGui::SliderInt("Rotation sensitivity", &rotationSensitivityDraft, minRotationSensitivity,
                    maxRotationSensitivity, "%d%%", ImGuiSliderFlags_AlwaysClamp);
                adjustingRotationSensitivity = ImGui::IsItemActive();
                if (ImGui::IsItemDeactivatedAfterEdit() && bindings.rotationSensitivityPercent != rotationSensitivityDraft) {
                    bindings.rotationSensitivityPercent = rotationSensitivityDraft;
                    notice.clear();
                    Save();
                }
                for (const auto& option : inversionOptions) {
                    if (&option != inversionOptions.data()) ImGui::SameLine();
                    if (ImGui::Checkbox(option.label, &(bindings.*option.enabled))) {
                        notice.clear();
                        Save();
                    }
                }
                if (ImGui::Checkbox("Breathing movement", &bindings.breathing)) {
                    notice.clear();
                    Save();
                }
                ImGui::Separator();
                const auto group = [](const char* label, Action first, Action end) {
                    ImGui::SeparatorText(label);
                    if (ImGui::BeginTable(label, 2, ImGuiTableFlags_SizingStretchProp)) {
                        ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("Key", ImGuiTableColumnFlags_WidthFixed, 113);
                        for (auto i = Index(first); i < Index(end); ++i) {
                            const auto action = static_cast<Action>(i);
                            ImGui::PushID(static_cast<int>(i));
                            ImGui::TableNextRow();
                            ImGui::TableNextColumn();
                            ImGui::AlignTextToFramePadding();
                            ImGui::TextUnformatted(actions[i].label);
                            ImGui::TableNextColumn();
                            const auto title = (capture == action ? std::string(PointerAction(action) ? "Press input" : "Press a key") :
                                KeyName(bindings.keys[i])) + "###binding";
                            ImGui::BeginDisabled(bindings.useWheel && action >= Action::WheelUp);
                            if (ImGui::Button(title.c_str(), {113, 0})) {
                                capture = capture == action ? std::nullopt : std::optional{action};
                                notice.clear();
                            }
                            ImGui::EndDisabled();
                            ImGui::PopID();
                        }
                        ImGui::EndTable();
                    }
                };
                if (ImGui::BeginTable("Binding sections", 2, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_BordersInnerV)) {
                    ImGui::TableNextColumn();
                    group("Camera", Action::FlyForward, Action::PoseForward);
                    group("Editor", Action::ToggleEditor, Action::Grab);
                    ImGui::TableNextColumn();
                    group("Pose", Action::PoseForward, Action::ToggleEditor);
                    ImGui::EndTable();
                }
                if (ImGui::Checkbox("Use mouse wheel for depth and roll", &bindings.useWheel)) {
                    if (bindings.useWheel && capture && *capture >= Action::WheelUp) capture.reset();
                    notice.clear();
                    Save();
                }
                group("Pose tools", Action::Grab, Action::Count);
                ImGui::Separator();
                if (ImGui::Button("Reset defaults")) {
                    bindings = defaults;
                    capture.reset();
                    notice.clear();
                    Save();
                }
                if (!notice.empty()) ImGui::TextWrapped("%s", notice.c_str());
                else if (capture) ImGui::TextDisabled("Escape cancels the binding.");
            }
            ImGui::End();
            if (!visible) Runtime::CloseSettings();
        }

        void BodyPanel(float width, float height)
        {
            const auto view = Runtime::ReadBodyPresets();
            const ImVec2 maximum{width * .98F, height * .98F};
            ImGui::SetNextWindowPos({width * .5F, height * .5F}, ImGuiCond_FirstUseEver, {.5F, .5F});
            ImGui::SetNextWindowSize({std::min(500.0F, maximum.x), std::min(310.0F, maximum.y)}, ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSizeConstraints({std::min(300.0F, maximum.x), std::min(245.0F, maximum.y)}, maximum);
            bool open = true;
            if (ImGui::Begin("Body poser###BodyPoser", &open, ImGuiWindowFlags_NoCollapse)) {
                using Kind = Runtime::BodyPresetCommand::Kind;
                if (const auto action = HistoryButtons(Runtime::CanUseHistory(HistoryAction::Undo),
                    Runtime::CanUseHistory(HistoryAction::Redo))) Runtime::RequestHistory(*action);
                ImGui::BeginDisabled(view.entries.empty());
                ImGui::SetNextItemWidth(-1);
                if (ImGui::BeginCombo("##Body presets", view.entries.empty() ? "No saved poses" : "Load pose")) {
                    for (const bool premade : {false, true}) {
                        if (std::ranges::none_of(view.entries, [premade](const auto& item) { return item.premade == premade; }))
                            continue;
                        ImGui::SeparatorText(premade ? "Premade" : "Custom");
                        for (const auto& item : view.entries) {
                            if (item.premade != premade) continue;
                            ImGui::PushID(item.name.c_str());
                            if (ImGui::Selectable(item.name.c_str())) Runtime::RequestBodyPreset({Kind::Load, item.name});
                            ImGui::PopID();
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::EndDisabled();
                if (bodyForm.overwrite) {
                    ImGui::TextWrapped("Overwrite %s?", bodyForm.overwrite->c_str());
                    if (ImGui::Button("Overwrite pose")) {
                        Runtime::RequestBodyPreset({Kind::Overwrite, *bodyForm.overwrite});
                        bodyForm.overwrite.reset();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel")) bodyForm.overwrite.reset();
                } else {
                    ImGui::SetNextItemWidth(-1);
                    const bool enter = PresetNameInput("##Body pose name", "Pose name", bodyForm.name.data(), bodyForm.name.size());
                    if (ImGui::Button("Save pose") || enter) {
                        const auto name = PresetStorage::TrimName(bodyForm.name.data());
                        const auto key = PresetStorage::Fold(name);
                        const auto existing = std::ranges::find_if(view.entries, [&](const auto& item) {
                            return PresetStorage::Fold(item.name) == key;
                        });
                        if (existing != view.entries.end()) bodyForm.overwrite = existing->name;
                        else Runtime::RequestBodyPreset({Kind::Save, std::string(name)});
                    }
                }
                if (!view.status.empty()) ImGui::TextWrapped("%s", view.status.c_str());
            }
            ImGui::End();
            if (!open) Runtime::CloseSettings();
        }

        enum class MouseIcon { Move, Left, Right, Middle, Wheel };

        void MouseGlyph(MouseIcon kind)
        {
            const auto p = ImGui::GetCursorScreenPos();
            const auto h = ImGui::GetStyle().FontSizeBase + 4;
            const float w = h * 1.10F;
            ImGui::Dummy({w, h});
            auto* draw = ImGui::GetWindowDrawList();
            const auto at = [&](float x, float y) { return ImVec2{p.x + w * x, p.y + h * y}; };
            const auto ink = IM_COL32(220, 231, 231, 255);
            const auto active = IM_COL32(121, 207, 221, 255);
            draw->AddRect(at(.22F, .05F), at(.75F, .92F), ink, 5, 1.5F);
            draw->AddLine(at(.22F, .40F), at(.75F, .40F), ink, 1);
            draw->AddLine(at(.49F, .05F), at(.49F, .40F), ink, 1);
            if (kind == MouseIcon::Left || kind == MouseIcon::Right) {
                const float x = kind == MouseIcon::Left ? .29F : .55F;
                draw->AddRectFilled(at(x, .13F), at(x + .12F, .33F), active, 1);
            }
            if (kind == MouseIcon::Wheel || kind == MouseIcon::Middle)
                draw->AddRectFilled(at(.44F, .17F), at(.54F, kind == MouseIcon::Wheel ? .51F : .35F), active, 1);
            if (kind == MouseIcon::Move) {
                draw->AddLine(at(.01F, .55F), at(.16F, .55F), active, 1);
                draw->AddLine(at(.01F, .55F), at(.09F, .43F), active, 1);
                draw->AddLine(at(.01F, .55F), at(.09F, .67F), active, 1);
                draw->AddLine(at(.82F, .55F), at(.98F, .55F), active, 1);
                draw->AddLine(at(.98F, .55F), at(.90F, .43F), active, 1);
                draw->AddLine(at(.98F, .55F), at(.90F, .67F), active, 1);
            } else if (kind == MouseIcon::Wheel) {
                draw->AddLine(at(.88F, .22F), at(.88F, .72F), active, 1);
                for (const float side : {.79F, .97F}) {
                    draw->AddLine(at(.88F, .22F), at(side, .35F), active, 1);
                    draw->AddLine(at(.88F, .72F), at(side, .59F), active, 1);
                }
            }
        }

        void KeyCap(unsigned code, std::string label = {})
        {
            const bool arrow = code == Key::kLeft || code == Key::kRight || code == Key::kUp || code == Key::kDown;
            if (label.empty() && !arrow) label = KeyName(code);
            const auto p = ImGui::GetCursorScreenPos();
            const auto h = ImGui::GetFontSize() + 4;
            const auto w = std::max(h, ImGui::CalcTextSize(label.c_str()).x + 9);
            ImGui::Dummy({w, h});
            auto* draw = ImGui::GetWindowDrawList();
            const auto ink = IM_COL32(220, 231, 231, 255);
            draw->AddRectFilled(p, {p.x + w, p.y + h}, IM_COL32(28, 33, 37, 240), 3);
            draw->AddRect(p, {p.x + w, p.y + h}, IM_COL32(105, 116, 121, 255), 3, 1.0F);
            if (!arrow) {
                draw->AddText({p.x + (w - ImGui::CalcTextSize(label.c_str()).x) / 2, p.y + 2}, ink, label.c_str());
                return;
            }
            const ImVec2 d{float(int(code == Key::kRight) - int(code == Key::kLeft)),
                           float(int(code == Key::kDown) - int(code == Key::kUp))};
            const auto c = ImVec2{p.x + w / 2, p.y + h / 2};
            const auto arrowPoint = [&](float forward, float side) {
                return ImVec2{c.x + d.x * forward - d.y * side, c.y + d.y * forward + d.x * side};
            };
            draw->AddLine(arrowPoint(-5, 0), arrowPoint(5, 0), ink, 1.4F);
            draw->AddLine(arrowPoint(5, 0), arrowPoint(1, -4), ink, 1.4F);
            draw->AddLine(arrowPoint(5, 0), arrowPoint(1, 4), ink, 1.4F);
        }

        void Guide(float width, float height)
        {
            const auto value = Get();
            const bool right = value.corner == Corner::UpperRight || value.corner == Corner::LowerRight;
            const bool lower = value.corner == Corner::LowerLeft || value.corner == Corner::LowerRight;
            const auto margin = std::clamp(height / 70, 8.0F, 24.0F);
            ImGui::SetNextWindowPos({right ? width - margin : margin, lower ? height - margin : margin},
                ImGuiCond_Always, {right ? 1.0F : 0.0F, lower ? 1.0F : 0.0F});
            ImGui::SetNextWindowBgAlpha(0.72F);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {10, 8});
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {4, 3});
            constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoFocusOnAppearing;
            if (ImGui::Begin("Pose shortcut guide", nullptr, flags)) {
                ImGui::PushFont(nullptr, std::clamp(height / 65, 12.0F, 19.0F));
                if (ImGui::BeginTable("Shortcuts", 2, ImGuiTableFlags_SizingFixedFit)) {
                    const auto row = [](const char* label, auto draw) {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        draw();
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(label);
                    };
                    const auto section = [](const char* label) {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::TextDisabled("%s", label);
                    };
                    const auto keys = [&value](std::initializer_list<Action> keys) {
                        bool first = true;
                        for (const auto key : keys) {
                            if (!std::exchange(first, false)) ImGui::SameLine();
                            const unsigned code = value.Key(key);
                            const auto alias = value.Alternate(key);
                            if (code == MouseBinding(0)) MouseGlyph(MouseIcon::Left);
                            else if (code == MouseBinding(1)) MouseGlyph(MouseIcon::Right);
                            else if (code == MouseBinding(2)) MouseGlyph(MouseIcon::Middle);
                            else KeyCap(code, alias == Key::kRightShift ? "Shift" : alias == Key::kRightControl ? "Ctrl" : "");
                        }
                    };
                    const auto plus = [] {
                        ImGui::SameLine();
                        ImGui::TextUnformatted("+");
                        ImGui::SameLine();
                    };
                    const auto mouseMove = [&] {
                        plus();
                        MouseGlyph(MouseIcon::Move);
                    };
                    const auto wheel = [&] {
                        plus();
                        if (value.useWheel) MouseGlyph(MouseIcon::Wheel);
                        else keys({Action::WheelUp, Action::WheelDown});
                    };
                    section("Camera");
                    row("Look", [] { MouseGlyph(MouseIcon::Move); });
                    row("Fly", [&] { keys({Action::FlyForward, Action::FlyLeft, Action::FlyBack, Action::FlyRight}); });
                    row("Slow flight", [&] { keys({Action::SlowFlight}); });

                    section("Handles");
                    row("Drag", [&] { keys({Action::Grab}); mouseMove(); });
                    row("Move closer / farther", [&] { keys({Action::Grab}); wheel(); });
                    row("Yaw / pitch", [&] { keys({Action::Rotate}); mouseMove(); });
                    row("Roll", [&] { keys({Action::Rotate}); wheel(); });
                    row("Click a circle for Face, Finger, or Body Poser", [&] { keys({Action::OpenPoser}); });

                    const auto plane = {Action::PoseForward, Action::PoseLeft, Action::PoseBack, Action::PoseRight};
                    const auto vertical = {Action::Raise, Action::Lower};
                    section("Whole pose");
                    row("Slide", [&] { keys(plane); });
                    row("Raise / lower", [&] { keys(vertical); });
                    row("Turn clockwise / counterclockwise", [&] { keys({Action::Clockwise, Action::Counterclockwise}); });

                    section("Nudge mode");
                    row("Toggle", [&] { keys({Action::Nudge}); });
                    row("Latch / switch beam", [&] { keys({Action::Grab, Action::Rotate}); });
                    row("Move or yaw / pitch", [&] { keys(plane); });
                    row("Raise / lower or roll", [&] { keys(vertical); });

                    section("Editor");
                    row("Toggle mirror mode", [&] { keys({Action::Mirror}); });
                    row("Undo / redo", [&] { keys({Action::Undo, Action::Redo}); });
                    row("Settings", [&] { keys({Action::ToggleSettings}); });
                    row("Show / hide posing HUD", [&] { keys({Action::ToggleGuide}); });
                    row("Exit posing", [&] { keys({Action::ToggleEditor}); });
                    ImGui::EndTable();
                }
                ImGui::PopFont();
            }
            ImGui::End();
            ImGui::PopStyleVar(2);
        }
    }

    void Load()
    {
        const std::scoped_lock lock(mutex);
        const auto plugins = std::filesystem::path(REL::Module::get().filePath().data()).parent_path() /
            L"Data/SKSE/Plugins";
        file = plugins / L"GModPose.ini";
        const auto legacy = plugins / L"GModPose-settings.ini";
        const auto* map = RE::ControlMap::GetSingleton();
        for (std::size_t i = 0; i < actions.size(); ++i) defaults.keys[i] = actions[i].key;
        if (map) {
            for (std::size_t i = 0; i < actions.size(); ++i) {
                if (!actions[i].gameMapping) continue;
                const auto key = map->GetMappedKey(actions[i].gameMapping, RE::INPUT_DEVICE::kKeyboard);
                if (Valid(static_cast<Action>(i), key) && std::ranges::find(defaults.keys, key) == defaults.keys.end()) defaults.keys[i] = key;
            }
        }
        UpdateAlternates(defaults);
        auto previous = ReadBindings(legacy, defaults);
        const auto corner = ReadSetting(legacy, L"Controls", L"GuideCorner");
        if (corner && *corner < corners.size()) previous.corner = static_cast<Corner>(*corner);
        auto& settingsKey = previous.keys[Index(Action::ToggleSettings)];
        const auto revision = ReadSetting(legacy, L"Controls", L"BindingsRevision").value_or(0);
        if (revision < 2 &&
            settingsKey == Key::kComma && std::ranges::find(previous.keys, Key::kSemicolon) == previous.keys.end())
            settingsKey = Key::kSemicolon;
        auto& guideKey = previous.keys[Index(Action::ToggleGuide)];
        if (revision < 3 && guideKey == Key::kPeriod && std::ranges::find(previous.keys, Key::kSlash) == previous.keys.end())
            guideKey = Key::kSlash;
        const auto checkKeys = [](Bindings& candidate) {
            // A new factory key can collide with a binding from an older settings file.
            for (const auto action : {Action::Mirror, Action::Undo, Action::Redo, Action::Nudge, Action::WheelUp, Action::WheelDown}) {
                auto& key = candidate.keys[Index(action)];
                if (key == defaults.Key(action) && std::ranges::count(candidate.keys, key) > 1) key = 0;
            }
            auto assigned = candidate.keys;
            std::ranges::sort(assigned);
            return std::adjacent_find(std::ranges::upper_bound(assigned, 0u), assigned.end()) == assigned.end();
        };
        bindings = ReadBindings(file, previous);
        if (!checkKeys(bindings)) {
            if (checkKeys(previous)) {
                bindings.keys = previous.keys;
                SKSE::log::warn("GModPose.ini contains conflicting controls; using previous bindings");
            } else {
                bindings.keys = defaults.keys;
                SKSE::log::warn("Duplicate GModPose controls; using default bindings");
            }
        }
        UpdateAlternates(bindings);
    }

    Bindings Get()
    {
        const std::scoped_lock lock(mutex);
        return bindings;
    }

    bool RebindKey(unsigned code)
    {
        const std::scoped_lock lock(mutex);
        if (!capture) return false;
        if (code == Key::kEscape) {
            capture.reset();
            notice.clear();
            return true;
        }
        const auto action = *capture;
        if (code >= mouseBase && !PointerAction(action)) return false;
        if (!Valid(action, code)) return true;
        const auto position = Index(action);
        const auto conflict = std::ranges::find(bindings.keys, code);
        if (conflict != bindings.keys.end() && conflict != bindings.keys.begin() + position) {
            const auto other = static_cast<Action>(conflict - bindings.keys.begin());
            if (PointerAction(action) && PointerAction(other) && Valid(other, bindings.keys[position])) {
                *conflict = bindings.keys[position];
            } else {
                notice = KeyName(code) + " is already assigned to " + actions[Index(other)].label + ".";
                return true;
            }
        }
        bindings.keys[position] = code;
        UpdateAlternates(bindings);
        capture.reset();
        notice.clear();
        Save();
        return true;
    }

    void ResetMenu()
    {
        const std::scoped_lock lock(mutex);
        presetNameActive.store(false);
        capture.reset();
        adjustingUiScale = false;
        adjustingRotationSensitivity = false;
        notice.clear();
        pending = {};
        pending.reset = true;
    }

    bool PresetNameActive() { return presetNameActive.load(); }

    bool PresetNameInput(const char* id, const char* hint, char* name, std::size_t size)
    {
        const bool submit = ImGui::InputTextWithHint(id, hint, name, size, ImGuiInputTextFlags_EnterReturnsTrue);
        presetNameDrawnActive = ImGui::IsItemActive();
        return submit;
    }

    std::optional<HistoryAction> HistoryButtons(bool canUndo, bool canRedo)
    {
        const auto bindings = Get();
        const auto button = [&](const char* label, Action action, bool enabled) {
            std::string title = label;
            if (const auto key = bindings.Key(action)) title += " (" + KeyName(key) + ")";
            ImGui::BeginDisabled(!enabled);
            const bool clicked = ImGui::Button(title.c_str());
            ImGui::EndDisabled();
            return clicked;
        };
        const bool undo = button("Undo", Action::Undo, canUndo);
        ImGui::SameLine();
        const bool redo = button("Redo", Action::Redo, canRedo);
        if (undo) return HistoryAction::Undo;
        if (redo) return HistoryAction::Redo;
        return std::nullopt;
    }

    void QueuePointer(double x, double y, double wheel, std::span<const std::uint8_t> clicks, std::span<const TextEvent> text)
    {
        const std::scoped_lock lock(mutex);
        pending.x += x;
        pending.y += y;
        pending.wheel += wheel;
        pending.clicks.insert(pending.clicks.end(), clicks.begin(), clicks.end());
        pending.text.insert(pending.text.end(), text.begin(), text.end());
    }

    ImGuiContext* CreateUI(ID3D11Device* device, ID3D11DeviceContext* context)
    {
        ContextScope restore;
        IMGUI_CHECKVERSION();
        auto* gui = ImGui::CreateContext();
        ImGui::SetCurrentContext(gui);
        auto& io = ImGui::GetIO();
        io.IniFilename = nullptr;
        io.LogFilename = nullptr;
        io.ConfigWindowsMoveFromTitleBarOnly = true;
        Style();
        LoadFont();
        if (ImGui_ImplDX11_Init(device, context) && ImGui_ImplDX11_CreateDeviceObjects()) return gui;
        if (io.BackendRendererUserData) ImGui_ImplDX11_Shutdown();
        ImGui::DestroyContext(gui);
        return nullptr;
    }

    void DestroyUI(ImGuiContext* context)
    {
        if (!context) return;
        ContextScope restore;
        ImGui::SetCurrentContext(context);
        ImGui_ImplDX11_Shutdown();
        ImGui::DestroyContext(context);
    }

    void DrawUI(ImGuiContext* context, bool settings, bool finger, bool face, bool body, float width, float height,
        const char* hoverLabel, bool nudge, bool mirror)
    {
        if (!context) return;
        ContextScope restore;
        ImGui::SetCurrentContext(context);
        auto& io = ImGui::GetIO();
        const auto reticleScale = std::clamp(height / 1080.0F, .75F, 2.0F);
        const auto scale = std::min(width / 1440.0F, height / 1080.0F);
        ImGui::GetStyle().FontScaleMain = Get().uiScale / 100.0F;
        const auto previousSize = io.DisplaySize;
        width /= scale;
        height /= scale;
        io.DisplaySize = {width, height};
        io.DisplayFramebufferScale = {scale, scale};
        static auto last = std::chrono::steady_clock::now();
        const auto now = std::chrono::steady_clock::now();
        io.DeltaTime = std::clamp(std::chrono::duration<float>(now - last).count(), 0.001F, 0.1F);
        last = now;
        io.MouseDrawCursor = settings || finger || face || body;
        if (io.MouseDrawCursor) {
            Pointer input;
            {
                const std::scoped_lock lock(mutex);
                std::swap(input, pending);
            }
            if (input.reset) bodyForm = {};
            if (input.reset || io.MousePos.x < 0 || io.MousePos.y < 0) {
                io.AddFocusEvent(false);
                io.AddFocusEvent(true);
                io.AddMouseButtonEvent(0, false);
                io.AddMouseButtonEvent(1, false);
            }
            const auto start = input.reset || io.MousePos.x < 0 || io.MousePos.y < 0 ?
                ImVec2{width * .5F, height * .5F} :
                ImVec2{io.MousePos.x * width / previousSize.x, io.MousePos.y * height / previousSize.y};
            const auto speed = height / 1080.0F;
            io.AddMousePosEvent(std::clamp(start.x + float(input.x) * speed, 0.0F, width),
                                std::clamp(start.y + float(input.y) * speed, 0.0F, height));
            for (const auto click : input.clicks) io.AddMouseButtonEvent(click / 2, (click & 1) != 0);
            if (input.wheel != 0) io.AddMouseWheelEvent(0, float(input.wheel));
            if (finger || face || body) {
                const auto toImGui = [](std::uint32_t key) {
                    switch (key) {
                    case Key::kLeft: return ImGuiKey_LeftArrow;
                    case Key::kRight: return ImGuiKey_RightArrow;
                    case Key::kBackspace: return ImGuiKey_Backspace;
                    case Key::kDelete: return ImGuiKey_Delete;
                    case Key::kHome: return ImGuiKey_Home;
                    case Key::kEnd: return ImGuiKey_End;
                    case Key::kEnter: return ImGuiKey_Enter;
                    case Key::kEscape: return ImGuiKey_Escape;
                    case Key::kKP_Enter: return ImGuiKey_KeypadEnter;
                    case Key::kLeftShift: return ImGuiKey_LeftShift;
                    case Key::kRightShift: return ImGuiKey_RightShift;
                    case Key::kLeftControl: return ImGuiKey_LeftCtrl;
                    case Key::kRightControl: return ImGuiKey_RightCtrl;
                    case Key::kA: return ImGuiKey_A;
                    case Key::kC: return ImGuiKey_C;
                    case Key::kV: return ImGuiKey_V;
                    case Key::kX: return ImGuiKey_X;
                    case Key::kY: return ImGuiKey_Y;
                    case Key::kZ: return ImGuiKey_Z;
                    default: return ImGuiKey_None;
                    }
                };
                for (const auto& event : input.text) {
                    if (event.character) {
                        if (event.code <= 0xFFFF) io.AddInputCharacterUTF16(static_cast<ImWchar16>(event.code));
                        else io.AddInputCharacter(event.code);
                    } else if (const auto key = toImGui(event.code); key != ImGuiKey_None) {
                        io.AddKeyEvent(key, event.down);
                        if (key == ImGuiKey_LeftShift || key == ImGuiKey_RightShift) {
                            const auto other = key == ImGuiKey_LeftShift ? ImGuiKey_RightShift : ImGuiKey_LeftShift;
                            io.AddKeyEvent(ImGuiMod_Shift, event.down || ImGui::IsKeyDown(other));
                        } else if (key == ImGuiKey_LeftCtrl || key == ImGuiKey_RightCtrl) {
                            const auto other = key == ImGuiKey_LeftCtrl ? ImGuiKey_RightCtrl : ImGuiKey_LeftCtrl;
                            io.AddKeyEvent(ImGuiMod_Ctrl, event.down || ImGui::IsKeyDown(other));
                        }
                    }
                }
            }
        } else {
            io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
            io.AddMouseButtonEvent(0, false);
            io.AddMouseButtonEvent(1, false);
        }
        ImGui_ImplDX11_NewFrame();
        ImGui::NewFrame();
        presetNameDrawnActive = false;
        if (finger) FingerPanel::Draw(width, height);
        else if (face) FacePanel::Draw(width, height);
        else if (body) BodyPanel(width, height);
        else if (settings) ControlsPanel(width, height);
        else {
            Guide(width, height);
            const auto drawLabel = [&](const char* label, float offset) {
                if (!label) return;
                ImGui::PushFont(nullptr, 16.0F);
                const auto size = ImGui::CalcTextSize(label);
                const ImVec2 position{std::floor((width - size.x) * .5F), std::floor(height * .5F + offset)};
                auto* draw = ImGui::GetBackgroundDrawList();
                draw->AddText({position.x + 1, position.y + 1}, IM_COL32(0, 0, 0, 220), label);
                draw->AddText(position, IM_COL32(233, 239, 241, 230), label);
                ImGui::PopFont();
            };
            const char* mode = nudge ? (mirror ? "Nudge mode  /  Mirror mode" : "Nudge mode") :
                mirror ? "Mirror mode" : nullptr;
            drawLabel(mode, -25 * reticleScale / scale - 16);
            drawLabel(hoverLabel, 25 * reticleScale / scale);
        }
        presetNameActive.store(presetNameDrawnActive);
        ImGui::Render();
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
    }
}
