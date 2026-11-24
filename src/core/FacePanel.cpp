#include <SKSE/SKSE.h>
#include <imgui.h>
#include "FacePanel.h"
#include "Editor.h"
#include "PresetStorage.h"
#include "PoseSerialization.h"
#include "PoseSettings.h"
#include "../Version.h"

#include <cmath>
#include <iomanip>
#include <limits>
#include <mutex>
#include <span>
#include <sstream>
#include <utility>
#include <vector>

namespace GModPose::FacePanel
{
    namespace
    {
        using namespace FacePose;
        using PresetStorage::FindName;
        using PresetStorage::Fold;
        using PresetStorage::ValidName;
        constexpr std::string_view fileHeader = "GModPose face presets 1";
        constexpr std::string_view poseHeader = "GModPose face pose 1";
        constexpr std::size_t maxPresets = 128;
        using SavedPreset = PoseSerialization::FacePreset;
        std::filesystem::path file;
        std::vector<SavedPreset> presets;
        std::vector<SavedPreset> premades;

        struct Panel
        {
            State pose, initial;
            Mask supported{}, changed{};
            bool edited{};
            std::vector<Change> finished;
            std::optional<State> cameraGaze;
            Expressions expressions{}, initialExpressions{};
            std::optional<std::size_t> selected;
            std::optional<std::size_t> confirmingOverwrite;
            bool confirmingDelete{};
            std::array<char, PosePresetJson::nameBufferSize> name{};
            std::string status;
        };
        std::mutex mutex;
        std::optional<Panel> panel;

        void SetMoodExpressions(Panel& state)
        {
            state.expressions.fill(0.0F);
            state.expressions[state.pose.mood] = state.pose.strength;
        }

        void Apply(Panel& state, const State& pose, Mask region)
        {
            region &= state.supported;
            Merge(state.pose, pose, region);
            state.changed |= region;
            state.edited |= region != 0;
            if (region & Bit(MoodSlot)) SetMoodExpressions(state);
            state.selected.reset();
            state.confirmingOverwrite.reset();
            state.confirmingDelete = false;
            state.status.clear();
        }

        void Checkpoint(Panel& state)
        {
            if (!state.edited) return;
            // Changes already sent to the editor only need a history checkpoint.
            state.finished.push_back({{state.pose, state.changed, state.expressions}, true});
            state.changed = 0;
            state.edited = false;
        }

        bool PremadeName(std::string_view name)
        {
            return FindName(premades, name) != premades.end();
        }

        void Sort(std::vector<SavedPreset>& saved)
        {
            std::ranges::sort(saved, {}, [](const SavedPreset& item) { return Fold(item.name); });
        }

        std::filesystem::path FaceFolder() { return PresetStorage::SharedFaceFolder(file.parent_path()); }

        bool ReadPreset(std::istream& in, SavedPreset& entry)
        {
            return !PremadeName(entry.name) && PoseSerialization::DecodeLegacyFace(in, entry.pose);
        }

        bool Store(const std::filesystem::path& path, const SavedPreset& entry)
        {
            using namespace PosePresetJson;
            auto document = Document("face", entry.name, entry.category, PoseSerialization::EncodeFacePose(entry.pose));
            document["source"] = entry.source.empty() ? "GModPose" : entry.source;
            return PresetStorage::WriteJson(path, document);
        }

        void ReloadFacePresets()
        {
            const auto folder = FaceFolder();
            const auto oldFolder = PresetStorage::Folder(file.parent_path(), L"Face");
            presets = PoseSerialization::ReadFolder<SavedPreset>(folder, "face", maxPresets,
                [](const auto& object, SavedPreset& entry) {
                    return PoseSerialization::DecodeFacePose(object, entry.pose);
                });
            PresetStorage::MigrateFiles(folder, presets,
                PresetStorage::ReadLegacyFolder<SavedPreset>(oldFolder, poseHeader, maxPresets, ReadPreset),
                maxPresets, Store);
            const auto marker = oldFolder / L".legacy-migrated";
            std::error_code error;
            if (!std::filesystem::exists(marker, error)) {
                std::vector<SavedPreset> legacy;
                std::ifstream input(file, std::ios::binary);
                std::string line;
                if (std::getline(input, line) && line == fileHeader) {
                    while (legacy.size() < maxPresets && std::getline(input, line)) {
                        if (line.size() > 4096) continue;
                        std::istringstream row(line);
                        row.imbue(std::locale::classic());
                        SavedPreset entry;
                        row >> std::ws;
                        if (row.peek() != '"' || !(row >> std::quoted(entry.name)) || !ValidName(entry.name) ||
                            FindName(legacy, entry.name) != legacy.end() || !ReadPreset(row, entry)) continue;
                        row >> std::ws;
                        if (row.eof()) legacy.push_back(std::move(entry));
                    }
                    std::filesystem::create_directories(oldFolder, error);
                    PresetStorage::MigrateLegacy(folder, marker, presets, std::move(legacy), maxPresets, Store);
                }
            }
            Sort(presets);
        }

        void SaveCustom(Panel& state, std::optional<std::size_t> overwrite = std::nullopt)
        {
            if (state.supported != all) {
                state.status = "Saving requires all face controls.";
                return;
            }
            std::string_view name(state.name.data());
            if (overwrite) {
                if (*overwrite >= presets.size()) return;
            } else {
                const auto first = name.find_first_not_of(' ');
                if (first != name.npos) name = name.substr(first, name.find_last_not_of(' ') - first + 1);
                if (first == name.npos || !ValidName(name)) {
                    state.status = "Enter a preset name of up to 64 characters.";
                    return;
                }
                if (const auto saved = FindName(presets, name); saved != presets.end()) {
                    state.confirmingOverwrite = saved - presets.begin();
                    state.confirmingDelete = false;
                    state.status.clear();
                    return;
                }
                if (PremadeName(name)) {
                    state.status = "That name belongs to a premade preset.";
                    return;
                }
                if (presets.size() >= maxPresets) {
                    state.status = "Face preset library is full.";
                    return;
                }
            }
            auto updated = overwrite ? presets[*overwrite] : SavedPreset{};
            if (!overwrite) updated.name = name;
            updated.pose = {state.pose, all};
            const auto path = updated.path.extension() == ".json" ? std::optional{updated.path} :
                PresetStorage::NewFile(FaceFolder(), updated.name);
            state.confirmingOverwrite.reset();
            if (!path || !Store(*path, updated)) {
                state.status = "Could not save the face preset.";
                return;
            }
            if (updated.path.extension() == ".gmodpose") PresetStorage::ArchiveLegacyFile(updated.path);
            updated.path = *path;
            const auto savedName = updated.name;
            if (overwrite) presets[*overwrite] = std::move(updated);
            else {
                presets.push_back(std::move(updated));
                Sort(presets);
            }
            const auto index = FindName(presets, savedName) - presets.begin();
            state.selected = index;
            state.confirmingDelete = false;
            state.name = {};
            state.status = overwrite ? "Face preset overwritten." : "Face preset saved.";
        }

        void DeleteCustom(Panel& state)
        {
            if (!state.selected || *state.selected >= presets.size()) return;
            state.confirmingDelete = false;
            std::error_code error;
            const auto& path = presets[*state.selected].path;
            if (path.empty() || !std::filesystem::remove(path, error)) {
                state.status = "Could not delete the face preset.";
                return;
            }
            presets.erase(presets.begin() + *state.selected);
            state.selected.reset();
            state.status = "Face preset deleted.";
        }

        void PresetControls(Panel& state)
        {
            const bool custom = state.selected && *state.selected < presets.size();
            ImGui::SetNextItemWidth(192);
            if (ImGui::BeginCombo("##Full face presets", custom ? presets[*state.selected].name.c_str() : "Apply full-face preset")) {
                for (const auto category : {PosePresetJson::Category::Custom, PosePresetJson::Category::Premade}) {
                    const bool bundled = category == PosePresetJson::Category::Premade && !premades.empty();
                    if (!bundled && std::ranges::none_of(presets, [category](const auto& item) { return item.category == category; }))
                        continue;
                    ImGui::SeparatorText(PosePresetJson::categories[static_cast<std::size_t>(category)]);
                    for (std::size_t i = 0; i < presets.size(); ++i) {
                        if (presets[i].category != category) continue;
                        ImGui::PushID(static_cast<int>(i));
                        if (ImGui::Selectable(presets[i].name.c_str(), state.selected == i)) {
                            Apply(state, presets[i].pose.pose, presets[i].pose.mask);
                            state.selected = i;
                        }
                        ImGui::PopID();
                    }
                    if (bundled) {
                        for (const auto& preset : premades)
                            if (ImGui::Selectable(preset.name.c_str())) Apply(state, preset.pose.pose, preset.pose.mask);
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            if (ImGui::Button("Restore face")) {
                Apply(state, state.initial, all);
                state.expressions = state.initialExpressions;
            }
            ImGui::SetItemTooltip("Restore the face from when this panel opened.");
            if (!state.confirmingOverwrite && state.selected && *state.selected < presets.size()) {
                if (state.confirmingDelete) {
                    ImGui::Text("Delete %s?", presets[*state.selected].name.c_str());
                    if (ImGui::Button("Delete preset")) DeleteCustom(state);
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel")) state.confirmingDelete = false;
                } else {
                    ImGui::SameLine();
                    if (ImGui::Button("Delete")) state.confirmingDelete = true;
                }
            }
            if (state.confirmingOverwrite) {
                ImGui::TextWrapped("Overwrite %s?", presets[*state.confirmingOverwrite].name.c_str());
                if (ImGui::Button("Overwrite preset")) SaveCustom(state, state.confirmingOverwrite);
                ImGui::SameLine();
                if (ImGui::Button("Cancel")) state.confirmingOverwrite.reset();
            } else {
                ImGui::SetNextItemWidth(std::max(115.0F, ImGui::GetContentRegionAvail().x - 115.0F));
                const bool enter = PoseSettings::PresetNameInput("##FacePresetName", "Preset name",
                    state.name.data(), state.name.size());
                ImGui::SameLine();
                if (ImGui::Button("Save preset") || enter) SaveCustom(state);
            }
            if (!state.status.empty()) ImGui::TextWrapped("%s", state.status.c_str());
        }

        void DrawThumbnail(ImDrawList& draw, ImVec2 top, float width, Template::Art art, ImU32 ink, ImU32 accent)
        {
            const auto point = [&](float x, float y) { return ImVec2{top.x + width * x, top.y + y}; };
            const auto curve = [&](ImVec2 a, ImVec2 b, ImVec2 c, ImVec2 d) {
                draw.AddBezierCubic(point(a.x, a.y), point(b.x, b.y), point(c.x, c.y),
                    point(d.x, d.y), ink, 1.65F);
            };
            if (art <= Template::Squint) {
                for (const float x : {.30F, .70F}) {
                    const bool closed = art == Template::ClosedEyes ||
                        (art == Template::WinkLeft && x > .5F) || (art == Template::WinkRight && x < .5F);
                    const float arch = closed ? 0 : art == Template::Squint ? 6 : 11;
                    curve({x-.16F, 19}, {x-.085F, 19-arch}, {x+.085F, 19-arch}, {x+.16F, 19});
                    curve({x-.16F, 19}, {x-.09F, closed ? 25.0F : 19+arch*.70F},
                        {x+.09F, closed ? 25.0F : 19+arch*.70F}, {x+.16F, 19});
                    if (!closed) draw.AddCircleFilled(point(x, 18), arch * .23F, accent);
                }
            } else if (art <= Template::Furrow) {
                const float lift = art == Template::RaisedBrow ? 12.0F : 6.0F;
                const float inner = art == Template::Furrow ? 23.0F : 15.0F;
                curve({.13F, 20}, {.22F, 20-lift}, {.36F, 15-lift}, {.45F, inner});
                curve({.55F, inner}, {.64F, 15-lift}, {.78F, 20-lift}, {.87F, 20});
            } else if (art == Template::OpenMouth || art == Template::Pout) {
                draw.AddEllipse(point(.5F, 19), {width * (art == Template::Pout ? .16F : .22F), 9}, ink, 0, 24, 1.65F);
            } else if (art == Template::Frown) {
                curve({.19F, 24}, {.33F, 9}, {.67F, 9}, {.81F, 24});
            } else {
                const float middle = art == Template::Grin ? 29.0F : art == Template::Pressed ? 16.0F : 20.0F;
                curve({.18F, 16}, {.36F, middle}, {.64F, middle}, {.82F, 16});
                if (art == Template::Grin)
                    curve({.18F, 16}, {.38F, 20}, {.62F, 20}, {.82F, 16});
                if (art == Template::RestLips)
                    curve({.26F, 17}, {.42F, 10}, {.58F, 10}, {.74F, 17});
            }
        }

        void Templates(Panel& state, std::span<const Template> choices)
        {
            const auto gap = ImGui::GetStyle().ItemSpacing.x;
            const auto width = (ImGui::GetContentRegionAvail().x - gap * (choices.size() - 1)) / choices.size();
            for (std::size_t i = 0; i < choices.size(); ++i) {
                if (i) ImGui::SameLine();
                const auto& choice = choices[i];
                ImGui::PushID(static_cast<int>(i));
                const auto top = ImGui::GetCursorScreenPos();
                if (ImGui::InvisibleButton("Face template", {width, 58}))
                    Apply(state, choice.pose, choice.region);
                const auto corner = ImVec2{top.x + width, top.y + 58};
                auto& draw = *ImGui::GetWindowDrawList();
                const auto fill = ImGui::IsItemHovered() ? IM_COL32(62,65,66,255) : IM_COL32(26,29,32,255);
                draw.AddRectFilled(top, corner, fill);
                draw.AddRect(top, corner, IM_COL32(111,109,99,255));
                DrawThumbnail(draw, top, width, choice.art, IM_COL32(225,217,195,255), IM_COL32(145,203,203,255));
                const auto text = ImGui::CalcTextSize(choice.name);
                draw.AddText({top.x + (width - text.x) * .5F, top.y + 34}, IM_COL32(236,232,220,255), choice.name);
                ImGui::PopID();
            }
        }

        struct Slider { Channel channel; const char* name; const char* description{}; };

        void Sliders(Panel& state, std::span<const Slider> choices)
        {
            if (!ImGui::BeginTable("Face sliders", 2, ImGuiTableFlags_SizingStretchSame)) return;
            ImGui::TableSetupColumn("Feature", ImGuiTableColumnFlags_WidthFixed, 156);
            ImGui::TableSetupColumn("Amount", ImGuiTableColumnFlags_WidthStretch);
            for (const auto& control : choices) {
                ImGui::PushID(static_cast<int>(control.channel));
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(control.name);
                if (control.description) ImGui::SetItemTooltip("%s", control.description);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1);
                ImGui::BeginDisabled(!(state.supported & Bit(control.channel)));
                float amount = state.pose[control.channel] * 100;
                if (ImGui::SliderFloat("##Amount", &amount, 0, 100, "%.0f%%", ImGuiSliderFlags_AlwaysClamp)) {
                    state.pose[control.channel] = amount / 100;
                    state.changed |= Bit(control.channel);
                    state.edited = true;
                }
                ImGui::EndDisabled();
                ImGui::PopID();
            }
            ImGui::EndTable();
        }

        void Eyes(Panel& state)
        {
            constexpr std::array eyelids{Slider{BlinkLeft,"Close left eye"}, Slider{BlinkRight,"Close right eye"},
                Slider{SquintLeft,"Left squint"}, Slider{SquintRight,"Right squint"}};
            constexpr std::array gaze{Slider{LookLeft,"Look left"}, Slider{LookRight,"Look right"},
                Slider{LookUp,"Look up"}, Slider{LookDown,"Look down"}};
            Templates(state, eyeTemplates);
            ImGui::SeparatorText("Eyelids");
            Sliders(state, eyelids);
            ImGui::SeparatorText("Gaze");
            ImGui::BeginDisabled(!state.cameraGaze || !(state.supported & gazeMask));
            if (ImGui::Button("Look at camera") && state.cameraGaze) Apply(state, *state.cameraGaze, gazeMask);
            ImGui::EndDisabled();
            Sliders(state, gaze);
        }

        void Brows(Panel& state)
        {
            constexpr std::array controls{Slider{BrowUpLeft,"Raise left"}, Slider{BrowUpRight,"Raise right"},
                Slider{BrowDownLeft,"Lower left"}, Slider{BrowDownRight,"Lower right"},
                Slider{BrowInLeft,"Draw left inward"}, Slider{BrowInRight,"Draw right inward"}};
            Templates(state, browTemplates);
            ImGui::Spacing();
            Sliders(state, controls);
        }

        void Mouth(Panel& state)
        {
            constexpr std::array common{Slider{BigAah,"Jaw open"}, Slider{Aah,"Ah"}, Slider{Eee,"Ee"},
                Slider{BMP,"Lips pressed"}, Slider{Oh,"Oh"}, Slider{OohQ,"Pucker"}};
            constexpr std::array others{
                Slider{ChJSh,"Round lips (Sh)", "FaceGen Ch / J / Sh, as in chat, jam, or ship."},
                Slider{DST,"Close teeth (S)", "FaceGen D / S / T, as in day, sip, or tip."},
                Slider{Eh,"Part lips (Eh)", "FaceGen Eh, as in bed."},
                Slider{FV,"Lip to teeth (F / V)", "FaceGen F / V, as in fan or van."},
                Slider{I,"Short vowel (I)", "FaceGen I, as in sit."},
                Slider{K,"Back tongue (K)", "FaceGen K, as in key."},
                Slider{N,"Tongue up (N)", "FaceGen N, as in no."},
                Slider{R,"R-shaped lips", "FaceGen R, as in red."},
                Slider{Th,"Tongue at teeth (Th)", "FaceGen Th, as in thin."},
                Slider{W,"Round lips (W)", "FaceGen W, as in way."}};
            Templates(state, mouthTemplates);
            ImGui::Spacing();
            Sliders(state, common);
            if (ImGui::CollapsingHeader("More mouth shapes")) Sliders(state, others);
        }

        void Mood(Panel& state)
        {
            ImGui::BeginDisabled(!(state.supported & Bit(MoodSlot)));
            Templates(state, moodTemplates);
            ImGui::Spacing();
            const auto expression = state.pose.mood - neutralMood;
            if (ImGui::BeginCombo("Expression", moodLabels[expression])) {
                for (const auto i : moodOrder)
                    if (ImGui::Selectable(moodLabels[i], i == expression)) Apply(state, Expression(i), Bit(MoodSlot));
                ImGui::EndCombo();
            }
            float strength = state.pose.strength * 100;
            if (ImGui::SliderFloat("Strength", &strength, 0, 100, "%.0f%%", ImGuiSliderFlags_AlwaysClamp)) {
                state.pose.strength = strength / 100;
                state.changed |= Bit(MoodSlot);
                state.edited = true;
                SetMoodExpressions(state);
            }
            ImGui::EndDisabled();
        }
    }

    void Load()
    {
        const std::scoped_lock lock(mutex);
        file = std::filesystem::path(REL::Module::get().filePath().data()).parent_path() /
            L"Data/SKSE/Plugins/GModPose-face-presets.txt";
        premades = PoseSerialization::ReadFolder<SavedPreset>(
            PresetStorage::Folder(file.parent_path(), L"Premade") / L"Face", "face", maxPresets,
            [](const auto& object, SavedPreset& entry) {
                return entry.category == PosePresetJson::Category::Premade &&
                    PoseSerialization::DecodeFacePose(object, entry.pose);
            });
        Sort(premades);
        ReloadFacePresets();
    }

    void Begin(const FacePose::State& pose, FacePose::Mask supported, const FacePose::Expressions& expressions)
    {
        const std::scoped_lock lock(mutex);
        ReloadFacePresets();
        panel.emplace();
        panel->pose = panel->initial = pose;
        panel->supported = supported;
        panel->expressions = panel->initialExpressions = expressions;
    }

    void End()
    {
        const std::scoped_lock lock(mutex);
        panel.reset();
    }

    void ShowPose(const State& pose, const Expressions& expressions)
    {
        const std::scoped_lock lock(mutex);
        if (!panel) return;
        Apply(*panel, pose, all);
        panel->expressions = expressions;
        panel->changed = 0;
        panel->edited = false;
        panel->finished.clear();
    }

    std::vector<Change> TakeEdits(bool finish)
    {
        const std::scoped_lock lock(mutex);
        if (!panel) return {};
        if (finish) Checkpoint(*panel);
        auto result = std::exchange(panel->finished, {});
        if (const auto changed = std::exchange(panel->changed, 0))
            result.push_back({{panel->pose, changed, panel->expressions}});
        return result;
    }

    void SetCameraGaze(std::optional<State> gaze)
    {
        const std::scoped_lock lock(mutex);
        if (panel) panel->cameraGaze = std::move(gaze);
    }

    void Draw(float width, float height)
    {
        const std::scoped_lock lock(mutex);
        if (!panel) return;
        const auto panelWidth = std::min(493.0F, width * .93F);
        const auto panelHeight = std::min(570.0F, height * .94F);
        const ImVec2 maximum{width * .98F, height * .98F};
        ImGui::SetNextWindowPos({std::clamp(width * .045F, 12.0F, std::max(12.0F, width - panelWidth - 12)),
            height / 2}, ImGuiCond_FirstUseEver, {0, .5F});
        ImGui::SetNextWindowSize({panelWidth, panelHeight}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints({std::min(300.0F, maximum.x), std::min(300.0F, maximum.y)}, maximum);
        bool open = true;
        if (ImGui::Begin("GMOD Pose v" GMODPOSE_VERSION_TEXT "###FacePoser", &open, ImGuiWindowFlags_NoCollapse)) {
            auto& state = *panel;
            ImGui::SeparatorText("Face poser");
            if (const auto action = PoseSettings::HistoryButtons(Runtime::CanUseHistory(HistoryAction::Undo),
                Runtime::CanUseHistory(HistoryAction::Redo))) Runtime::RequestHistory(*action);
            PresetControls(state);
            if (ImGui::BeginTabBar("Face regions", ImGuiTabBarFlags_FittingPolicyScroll)) {
                if (ImGui::BeginTabItem("Eyes")) { Eyes(state); ImGui::EndTabItem(); }
                if (ImGui::BeginTabItem("Brows")) { Brows(state); ImGui::EndTabItem(); }
                if (ImGui::BeginTabItem("Mouth")) { Mouth(state); ImGui::EndTabItem(); }
                if (ImGui::BeginTabItem("Mood")) { Mood(state); ImGui::EndTabItem(); }
                ImGui::EndTabBar();
            }
            if (ImGui::IsMouseReleased(0) || ImGui::IsMouseReleased(1) ||
                (!ImGui::IsMouseDown(0) && !ImGui::IsMouseDown(1))) Checkpoint(state);
        }
        ImGui::End();
        if (!open) Runtime::CloseSettings();
    }
}
