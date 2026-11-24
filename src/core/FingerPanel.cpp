#include <SKSE/SKSE.h>
#include <imgui.h>
#include "FingerPanel.h"
#include "Editor.h"
#include "PresetStorage.h"
#include "PoseSerialization.h"
#include "PoseSettings.h"
#include "../Version.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <mutex>
#include <numbers>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace GModPose::FingerPanel
{
    namespace
    {
        using namespace Fingers;
        using PresetStorage::FindName;
        using PresetStorage::Fold;
        using PresetStorage::ValidName;
        constexpr std::string_view fileHeader = "GModPose gestures 1";
        constexpr std::string_view poseHeader = "GModPose hand pose 1";
        constexpr std::size_t maxGestures = 128;
        constexpr float diagramWidth = 320, diagramHeight = 350;
        using SavedGesture = PoseSerialization::HandPreset;
        std::mutex mutex;
        std::filesystem::path file;
        std::vector<SavedGesture> gestures;
        std::vector<SavedGesture> premades;

        struct Panel
        {
            Side side{};
            Pose pose, initial;
            Mask available{}, mirrorAvailable{}, changed{}, edited{}, mirrored{}, mirroredAction{};
            bool mirror{};
            std::vector<Edit> finished;
            int selected{-1};
            std::optional<std::size_t> confirmingOverwrite;
            bool confirmingDelete{};
            std::array<char, PosePresetJson::nameBufferSize> name{};
            std::string status;
            struct Drag { std::size_t joint; Angles start; ImVec2 movement{}; };
            std::optional<Drag> drag;
        };
        std::optional<Panel> panel;

        void Sort(std::vector<SavedGesture>& entries)
        {
            std::ranges::sort(entries, {}, [](const SavedGesture& item) { return Fold(item.name); });
        }

        std::filesystem::path GestureFolder() { return PresetStorage::Folder(file.parent_path(), L"Hand"); }

        bool ReadGesture(std::istream& in, SavedGesture& entry)
        {
            return FindName(premades, entry.name) == premades.end() && PoseSerialization::DecodeLegacyHand(in, entry.pose);
        }

        bool Store(const std::filesystem::path& path, const SavedGesture& entry)
        {
            using namespace PosePresetJson;
            auto document = Document("hand", entry.name, entry.category, PoseSerialization::EncodeHand(entry.pose));
            document["source"] = entry.source.empty() ? "GModPose" : entry.source;
            return PresetStorage::WriteJson(path, document);
        }

        void ReloadGestures()
        {
            const auto folder = GestureFolder();
            gestures = PoseSerialization::ReadFolder<SavedGesture>(folder, "hand", maxGestures,
                [](const auto& object, SavedGesture& entry) { return PoseSerialization::DecodeHand(object, entry.pose); });
            PresetStorage::MigrateFiles(folder, gestures,
                PresetStorage::ReadLegacyFolder<SavedGesture>(folder, poseHeader, maxGestures, ReadGesture), maxGestures, Store);
            const auto marker = folder / L".legacy-migrated";
            std::error_code error;
            if (!std::filesystem::exists(marker, error)) {
                std::vector<SavedGesture> legacy;
                std::ifstream input(file, std::ios::binary);
                std::string line;
                if (std::getline(input, line) && line == fileHeader) {
                    while (legacy.size() < maxGestures && std::getline(input, line)) {
                        if (line.size() > 4096) continue;
                        std::istringstream row(line);
                        row.imbue(std::locale::classic());
                        SavedGesture entry;
                        row >> std::ws;
                        if (row.peek() != '"' || !(row >> std::quoted(entry.name)) || !ValidName(entry.name) ||
                            FindName(legacy, entry.name) != legacy.end() || !ReadGesture(row, entry)) continue;
                        row >> std::ws;
                        if (row.eof()) legacy.push_back(std::move(entry));
                    }
                    std::filesystem::create_directories(folder, error);
                    PresetStorage::MigrateLegacy(folder, marker, gestures, std::move(legacy), maxGestures, Store);
                }
            }
            Sort(gestures);
        }

        void SaveCustom(Panel& edit, std::optional<std::size_t> overwrite = std::nullopt)
        {
            if (edit.available != allJoints) {
                edit.status = "Saving requires all 15 finger bones.";
                return;
            }
            std::string_view name(edit.name.data());
            if (overwrite) {
                if (*overwrite >= gestures.size()) return;
            } else {
                const auto first = name.find_first_not_of(' ');
                if (first != name.npos) name = name.substr(first, name.find_last_not_of(' ') - first + 1);
                if (first == name.npos || !ValidName(name)) {
                    edit.status = "Enter a gesture name of up to 64 characters.";
                    return;
                }
                const auto& premade = premades;
                if (FindName(premade, name) != premade.end()) {
                    edit.status = "That name belongs to a premade gesture.";
                    return;
                }
                if (const auto saved = FindName(gestures, name); saved != gestures.end()) {
                    edit.confirmingOverwrite = saved - gestures.begin();
                    edit.confirmingDelete = false;
                    edit.status.clear();
                    return;
                }
                if (gestures.size() >= maxGestures) {
                    edit.status = "Gesture library is full.";
                    return;
                }
            }
            auto updated = overwrite ? gestures[*overwrite] : SavedGesture{};
            if (!overwrite) updated.name = name;
            updated.pose = edit.pose;
            const auto path = updated.path.extension() == ".json" ? std::optional{updated.path} :
                PresetStorage::NewFile(GestureFolder(), updated.name);
            edit.confirmingOverwrite.reset();
            if (!path || !Store(*path, updated)) {
                edit.status = "Could not save the gesture.";
                return;
            }
            if (updated.path.extension() == ".gmodpose") PresetStorage::ArchiveLegacyFile(updated.path);
            updated.path = *path;
            const auto savedName = updated.name;
            if (overwrite) gestures[*overwrite] = std::move(updated);
            else {
                gestures.push_back(std::move(updated));
                Sort(gestures);
            }
            const auto index = FindName(gestures, savedName) - gestures.begin();
            edit.selected = static_cast<int>(premades.size() + index);
            edit.confirmingDelete = false;
            edit.name = {};
            edit.status = overwrite ? "Gesture overwritten." : "Gesture saved.";
        }

        bool DeleteCustom(Panel& edit)
        {
            if (edit.selected < static_cast<int>(premades.size())) return false;
            const auto index = static_cast<std::size_t>(edit.selected) - premades.size();
            if (index >= gestures.size()) return false;
            std::error_code error;
            if (gestures[index].path.empty() || !std::filesystem::remove(gestures[index].path, error)) {
                edit.status = "Could not delete the gesture.";
                return false;
            }
            gestures.erase(gestures.begin() + index);
            edit.selected = -1;
            edit.status = "Gesture deleted.";
            return true;
        }

        void MarkEdited(Panel& edit, Mask joints)
        {
            edit.changed |= joints;
            edit.edited |= joints;
            if (edit.mirror) {
                edit.mirrored |= joints & edit.mirrorAvailable;
                edit.mirroredAction |= joints & edit.mirrorAvailable;
            }
        }

        void Apply(Panel& edit, const Pose& pose, int selection)
        {
            edit.pose = pose;
            MarkEdited(edit, edit.available);
            edit.selected = selection;
            edit.drag.reset();
            edit.confirmingDelete = false;
            edit.confirmingOverwrite.reset();
            edit.status.clear();
        }

        void Checkpoint(Panel& edit)
        {
            if (!edit.edited) return;
            edit.finished.push_back({edit.pose, edit.edited, true, edit.mirroredAction});
            edit.changed = 0;
            edit.edited = edit.mirrored = edit.mirroredAction = 0;
        }

        const char* SelectedName(int selection)
        {
            if (selection < 0) return "Custom pose";
            const auto index = static_cast<std::size_t>(selection);
            if (index < premades.size()) return premades[index].name.c_str();
            const auto custom = index - premades.size();
            return custom < gestures.size() ? gestures[custom].name.c_str() : "Custom pose";
        }

        void GesturePicker(Panel& edit)
        {
            const auto restoreWidth = ImGui::CalcTextSize("Restore hand").x +
                2 * ImGui::GetStyle().FramePadding.x + ImGui::GetStyle().ItemSpacing.x;
            ImGui::SetNextItemWidth(std::clamp(ImGui::GetContentRegionAvail().x - restoreWidth, 80.0F, 208.0F));
            if (ImGui::BeginCombo("##Gestures", SelectedName(edit.selected))) {
                for (const auto category : {PosePresetJson::Category::Custom, PosePresetJson::Category::Premade}) {
                    const bool bundled = category == PosePresetJson::Category::Premade && !premades.empty();
                    if (!bundled && std::ranges::none_of(gestures, [category](const auto& item) { return item.category == category; }))
                        continue;
                    ImGui::SeparatorText(PosePresetJson::categories[static_cast<std::size_t>(category)]);
                    for (std::size_t i = 0; i < gestures.size(); ++i) {
                        if (gestures[i].category != category) continue;
                        const auto selection = static_cast<int>(premades.size() + i);
                        ImGui::PushID(selection);
                        if (ImGui::Selectable(gestures[i].name.c_str(), edit.selected == selection))
                            Apply(edit, gestures[i].pose, selection);
                        ImGui::PopID();
                    }
                    if (bundled) {
                        for (std::size_t i = 0; i < premades.size(); ++i)
                            if (ImGui::Selectable(premades[i].name.c_str(), edit.selected == static_cast<int>(i)))
                                Apply(edit, premades[i].pose, static_cast<int>(i));
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            if (ImGui::Button("Restore hand")) Apply(edit, edit.initial, -1);
            if (!edit.confirmingOverwrite && edit.selected >= static_cast<int>(premades.size())) {
                if (edit.confirmingDelete) {
                    ImGui::Text("Delete %s?", SelectedName(edit.selected));
                    ImGui::SameLine();
                    if (ImGui::Button("Delete gesture")) {
                        DeleteCustom(edit);
                        edit.confirmingDelete = false;
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel")) edit.confirmingDelete = false;
                } else {
                    ImGui::SameLine();
                    if (ImGui::Button("Delete")) edit.confirmingDelete = true;
                }
            }
        }

        void HandDiagram(Panel& edit, float height)
        {
            constexpr std::array digitLabels{"Thumb", "Index", "Middle", "Ring", "Little"};
            constexpr std::array<ImVec2, digitCount> roots{{{-57,238},{-43,191},{-7,180},{28,189},{60,207}}};
            constexpr std::array<float, digitCount> directions{-0.98F,-0.24F,-0.03F,0.14F,0.31F};
            constexpr std::array<std::array<float, jointsPerDigit>, digitCount> lengths{{
                {35,30,23}, {37,31,25}, {43,33,26}, {40,31,24}, {32,25,20}}};
            constexpr std::array<float, digitCount> widths{14,11,12,11,10};
            const auto size = ImVec2{ImGui::GetContentRegionAvail().x, height};
            const auto top = ImGui::GetCursorScreenPos();
            ImGui::InvisibleButton("Hand diagram", size);
            const auto mirror = edit.side == Side::Left ? -1.0F : 1.0F;
            const auto scale = std::min(size.x / diagramWidth, size.y / diagramHeight);
            const auto map = [&](Vec3 p) {
                return ImVec2{top.x + size.x / 2 + float(p.x) * mirror * scale,
                    top.y + (size.y - diagramHeight * scale) / 2 + float(p.y) * scale};
            };

            std::array<ImVec2, jointCount> anchors{}, tips{};
            std::array<DiagramSegment, jointCount> segments{};
            std::array<double, jointCount> headings{};
            const auto along = [](double heading) -> Vec3 { return {std::sin(heading), -std::cos(heading), 0}; };
            const auto buildDiagram = [&] {
                for (std::size_t digit = 0; digit < digitCount; ++digit) {
                    Vec3 p{roots[digit].x, roots[digit].y, 0};
                    auto heading = double(directions[digit]);
                    for (std::size_t joint = 0; joint < jointsPerDigit; ++joint) {
                        const auto index = digit * jointsPerDigit + joint;
                        const auto angles = ReadAngles(edit.pose[index]);
                        segments[index] = {along(heading), lengths[digit][joint]};
                        anchors[index] = map(p);
                        p = p + segments[index].Tip(angles);
                        tips[index] = map(p);
                        heading -= angles.spread;
                        headings[index] = heading;
                    }
                }
            };
            buildDiagram();

            const auto mouse = ImGui::GetIO().MousePos;
            std::optional<std::size_t> hover;
            float distance = std::max(7.0F, 11 * scale);
            if (ImGui::IsItemHovered() || ImGui::IsItemActivated()) {
                for (std::size_t i = 0; i < tips.size(); ++i) {
                    if (!(edit.available & Joint(i))) continue;
                    const float next = std::hypot(mouse.x - tips[i].x, mouse.y - tips[i].y);
                    if (next < distance) {
                        distance = next;
                        hover = i;
                    }
                }
            }
            if (ImGui::IsItemActivated() && hover) edit.drag = Panel::Drag{*hover, ReadAngles(edit.pose[*hover])};
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) edit.drag.reset();
            const auto setAngles = [&](std::size_t index, Angles angles) {
                edit.pose[index] = FromAngles(angles);
                MarkEdited(edit, Joint(index));
                edit.selected = -1;
                edit.confirmingDelete = false;
                edit.confirmingOverwrite.reset();
            };
            if (edit.drag && ImGui::IsItemActive()) {
                // Drag deltas are cumulative since pickup. Apply them to the original joint angles.
                const auto movement = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, 2);
                auto& drag = *edit.drag;
                if (movement.x != drag.movement.x || movement.y != drag.movement.y) {
                    drag.movement = movement;
                    const auto index = drag.joint;
                    const auto& segment = segments[index];
                    const auto tip = segment.Tip(drag.start) + Vec3{movement.x * mirror / scale, movement.y / scale, 0};
                    setAngles(index, segment.Drag(tip, drag.start));
                    buildDiagram();
                }
            }
            const auto active = edit.drag ? std::optional{edit.drag->joint} : hover;
            if (active && ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY)) {
                if (const float wheel = ImGui::GetIO().MouseWheel; wheel != 0) {
                    constexpr auto radiansPerStep = std::numbers::pi / 36;
                    auto angles = ReadAngles(edit.pose[*active]);
                    angles.roll = std::remainder(angles.roll + wheel * radiansPerStep, 2 * std::numbers::pi);
                    setAngles(*active, angles);
                    // Keep scroll edits when the next mouse move recomputes the drag.
                    if (edit.drag) edit.drag->start.roll = angles.roll;
                }
            }

            auto* draw = ImGui::GetWindowDrawList();
            const auto outline = IM_COL32(162,152,127,255);
            const auto skin = IM_COL32(70,62,51,255);
            const auto crease = IM_COL32(127,110,87,255);
            constexpr std::array<Vec3, 9> palm{{{-66,239},{-54,196},{-14,179},{28,184},{65,204},
                {73,252},{43,323},{-26,323},{-45,282}}};
            if (mirror > 0) {
                for (const auto point : palm) draw->PathLineTo(map(point));
            } else {
                for (auto point = palm.rbegin(); point != palm.rend(); ++point) draw->PathLineTo(map(*point));
            }
            draw->PathFillConvex(skin);
            for (std::size_t i = 0; i < palm.size(); ++i)
                draw->AddLine(map(palm[i]), map(palm[(i + 1) % palm.size()]), outline, 1.7F * scale);
            draw->AddBezierCubic(map({-41,253}), map({-10,245}), map({8,266}), map({3,300}), crease, 1.6F * scale);
            draw->AddBezierCubic(map({-27,231}), map({0,215}), map({22,230}), map({53,226}), crease, 1.6F * scale);
            draw->AddLine(map({-22,309}), map({38,308}), crease, 1.5F * scale);
            for (std::size_t digit = 0; digit < digitCount; ++digit) {
                for (std::size_t bone = 0; bone < jointsPerDigit; ++bone) {
                    const auto index = digit * jointsPerDigit + bone;
                    const auto from = anchors[index], tip = tips[index];
                    const auto radius = (widths[digit] - float(bone) * 1.4F) * scale;
                    draw->AddLine(from, tip, outline, radius * 2);
                    draw->AddCircleFilled(tip, radius, outline, 18);
                    draw->AddLine(from, tip, skin, (radius - 1.5F * scale) * 2);
                    draw->AddCircleFilled(tip, radius - 1.5F * scale, skin, 18);
                }
            }
            for (std::size_t index = 0; index < tips.size(); ++index) {
                const bool available = (edit.available & Joint(index)) != 0;
                const bool selected = (edit.drag && edit.drag->joint == index) || hover == index;
                const auto fill = selected ? IM_COL32(238,191,110,255) : available ? IM_COL32(153,203,207,255) : IM_COL32(84,76,63,255);
                draw->AddCircle(anchors[index], std::max(2.0F, 3.5F * scale), outline, 14);
                const auto heading = headings[index] + ReadAngles(edit.pose[index]).roll;
                const ImVec2 forward{float(std::sin(heading)) * mirror, float(-std::cos(heading))};
                const auto radius = std::max(6.0F, 8.8F * scale);
                const auto arrow = [&](float length, float across) {
                    return ImVec2{tips[index].x + radius * (forward.x * length - forward.y * across),
                        tips[index].y + radius * (forward.y * length + forward.x * across)};
                };
                const auto tip = arrow(1, 0), left = arrow(-.62F, .64F), right = arrow(-.62F, -.64F);
                draw->AddTriangleFilled(tip, left, right, fill);
                draw->AddTriangle(tip, left, right, IM_COL32(24,26,25,255), std::max(1.0F, 1.6F * scale));
            }
            if (active) {
                const auto angles = ReadAngles(edit.pose[*active]);
                constexpr double degrees = 180.0 / std::numbers::pi;
                ImGui::SetTooltip("%s, joint %zu\nCurl %+.0f°   Spread %+.0f°\nTwist %+.0f°",
                    digitLabels[*active / jointsPerDigit], *active % jointsPerDigit + 1,
                    angles.curl * degrees, angles.spread * degrees, angles.roll * degrees);
            }
        }
    }

    void Load()
    {
        const std::scoped_lock lock(mutex);
        file = std::filesystem::path(REL::Module::get().filePath().data()).parent_path() /
            L"Data/SKSE/Plugins/GModPose-gestures.txt";
        premades = PoseSerialization::ReadFolder<SavedGesture>(
            PresetStorage::Folder(file.parent_path(), L"Premade") / L"Hand", "hand", maxGestures,
            [](const auto& object, SavedGesture& entry) {
                return entry.category == PosePresetJson::Category::Premade && PoseSerialization::DecodeHand(object, entry.pose);
            });
        Sort(premades);
    }

    void Begin(Fingers::Side side, const Fingers::Pose& initial, Fingers::Mask available, Fingers::Mask mirrorAvailable,
               bool mirror)
    {
        const std::scoped_lock lock(mutex);
        ReloadGestures();
        panel.emplace();
        panel->side = side;
        panel->initial = panel->pose = initial;
        panel->available = available;
        panel->mirrorAvailable = mirrorAvailable;
        panel->mirror = mirror && mirrorAvailable != 0;
    }

    void End()
    {
        const std::scoped_lock lock(mutex);
        panel.reset();
    }

    void ShowPose(const Fingers::Pose& pose)
    {
        const std::scoped_lock lock(mutex);
        if (!panel) return;
        Apply(*panel, pose, -1);
        panel->changed = 0;
        panel->edited = panel->mirrored = panel->mirroredAction = 0;
        panel->finished.clear();
    }

    std::vector<Edit> TakeEdits(bool finish)
    {
        const std::scoped_lock lock(mutex);
        if (!panel) return {};
        if (finish) Checkpoint(*panel);
        auto result = std::exchange(panel->finished, {});
        if (const auto changed = std::exchange(panel->changed, 0))
            result.push_back({panel->pose, changed, false, std::exchange(panel->mirrored, 0)});
        return result;
    }

    void Draw(float width, float height)
    {
        const std::scoped_lock lock(mutex);
        if (!panel) return;
        constexpr float controlsHeight = 216, maxDiagramHeight = diagramHeight * 2.5F, initialScale = .70F;
        const float panelHeight = std::min(controlsHeight + maxDiagramHeight, height * .94F);
        const float canvasHeight = std::clamp(panelHeight - controlsHeight, 115.0F, maxDiagramHeight);
        const float horizontalPadding = 2 * ImGui::GetStyle().WindowPadding.x;
        const float panelWidth = std::min(std::max(465.0F, horizontalPadding + canvasHeight * diagramWidth / diagramHeight),
            width * .93F);
        const ImVec2 maximum{width * .98F, height * .98F};
        ImGui::SetNextWindowPos({0, height / 2}, ImGuiCond_FirstUseEver, {0, .5F});
        ImGui::SetNextWindowSize({panelWidth * initialScale, panelHeight * initialScale}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints({std::min(300.0F, maximum.x), std::min(300.0F, maximum.y)}, maximum);
        bool open = true;
        if (ImGui::Begin("GMOD Pose v" GMODPOSE_VERSION_TEXT "###FingerPoser", &open, ImGuiWindowFlags_NoCollapse)) {
            auto& edit = *panel;
            ImGui::SeparatorText(edit.side == Fingers::Side::Left ? "Left hand" : "Right hand");
            const auto rowEnd = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
            if (const auto action = PoseSettings::HistoryButtons(Runtime::CanUseHistory(HistoryAction::Undo),
                Runtime::CanUseHistory(HistoryAction::Redo))) Runtime::RequestHistory(*action);
            const auto mirrorWidth = ImGui::CalcTextSize("Mirror mode").x + ImGui::GetFrameHeight() +
                ImGui::GetStyle().ItemInnerSpacing.x;
            if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + mirrorWidth <= rowEnd) ImGui::SameLine();
            ImGui::BeginDisabled(!edit.mirrorAvailable);
            ImGui::Checkbox("Mirror mode", &edit.mirror);
            ImGui::EndDisabled();
            GesturePicker(edit);
            ImGui::TextWrapped("Drag a triangle toward the palm to curl or sideways to spread. "
                "Hover over a triangle and scroll to twist.");
            HandDiagram(edit, std::clamp(ImGui::GetWindowHeight() - controlsHeight, 115.0F, maxDiagramHeight));
            if (edit.confirmingOverwrite) {
                ImGui::TextWrapped("Overwrite %s?", gestures[*edit.confirmingOverwrite].name.c_str());
                if (ImGui::Button("Overwrite gesture")) SaveCustom(edit, edit.confirmingOverwrite);
                ImGui::SameLine();
                if (ImGui::Button("Cancel")) edit.confirmingOverwrite.reset();
            } else {
                const auto saveWidth = ImGui::CalcTextSize("Save gesture").x +
                    2 * ImGui::GetStyle().FramePadding.x + ImGui::GetStyle().ItemSpacing.x;
                ImGui::SetNextItemWidth(std::max(80.0F, ImGui::GetContentRegionAvail().x - saveWidth));
                const bool enter = PoseSettings::PresetNameInput("##GestureName", "Gesture name",
                    edit.name.data(), edit.name.size());
                ImGui::SameLine();
                if (ImGui::Button("Save gesture") || enter) SaveCustom(edit);
            }
            if (!edit.status.empty()) ImGui::TextWrapped("%s", edit.status.c_str());
            else if (edit.available != Fingers::allJoints)
                ImGui::TextDisabled("This skeleton is missing some finger bones.");
            if (ImGui::IsMouseReleased(0) || ImGui::IsMouseReleased(1) ||
                (!ImGui::IsMouseDown(0) && !ImGui::IsMouseDown(1))) Checkpoint(edit);
        }
        ImGui::End();
        if (!open) Runtime::CloseSettings();
    }
}
