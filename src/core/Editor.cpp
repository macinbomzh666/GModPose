#include <RE/Skyrim.h>
#include <SKSE/SKSE.h>
#include <REX/W32/KERNEL32.h>
#include <Windows.h>

#include "DragView.h"
#include "Editor.h"
#include "EditorKeys.h"
#include "GraphBone.h"
#include "PosingCamera.h"
#include "PoseSettings.h"
#include "FingerPanel.h"
#include "FacePanel.h"
#include "HistoryStorage.h"
#include "PoseSerialization.h"
#include "HookOriginal.h"
#include "LimbPose.h"
#include "EndpointModel.h"
#include "Breathing.h"
#include "GrabButton.h"
#include "SeatBlock.h"
#include "TailEndpoint.h"

#include <atomic>
#include <bit>
#include <chrono>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <limits>
#include <locale>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>

namespace GModPose::Runtime
{
    namespace
    {
        using Action = PoseSettings::Action;
        using JointNames = std::array<std::string_view, 3>;
        struct RefreshConfig
        {
            std::chrono::seconds period{3};
            int resolution{30};
        } refreshConfig;
        bool sharingEnabled{true};
        bool detachSeatedPose{true};
        EndpointModel wristModel, ankleModel;

        void LoadSharingConfig()
        {
            const auto file = std::filesystem::path(REL::Module::get().filePath().data()).parent_path() /
                L"Data/SKSE/Plugins/GModPose.ini";
            const auto read = [&](const wchar_t* key, int fallback) {
                wchar_t text[64]{};
                const auto size = ::GetPrivateProfileStringW(L"PlayerHub", key, L"", text, 64, file.c_str());
                if (!size || size == 63) return fallback;
                std::wistringstream input(text);
                input.imbue(std::locale::classic());
                int value{};
                return (input >> value) && (input >> std::ws).eof() ? value : fallback;
            };
            sharingEnabled = read(L"Enabled", 1) != 0;
            detachSeatedPose = read(L"DetachSeatedPose", 1) != 0;
            const auto period = read(L"PeriodSeconds", 3);
            if (period >= 0 && period <= 10) refreshConfig.period = std::chrono::seconds{period};
            const auto resolution = read(L"Resolution", refreshConfig.resolution);
            if (resolution > 0) refreshConfig.resolution = resolution;
        }
        constexpr std::array kLimbs{
            JointNames{"NPC L UpperArm [LUar]", "NPC L Forearm [LLar]", "NPC L Hand [LHnd]"},
            JointNames{"NPC R UpperArm [RUar]", "NPC R Forearm [RLar]", "NPC R Hand [RHnd]"},
            JointNames{"NPC L Thigh [LThg]", "NPC L Calf [LClf]", "NPC L Foot [Lft ]"},
            JointNames{"NPC R Thigh [RThg]", "NPC R Calf [RClf]", "NPC R Foot [Rft ]"}};
        constexpr std::array kLimbLabels{
            std::array{"Left shoulder", "Left elbow", "Left hand"},
            std::array{"Right shoulder", "Right elbow", "Right hand"},
            std::array{"Left hip", "Left knee", "Left foot"},
            std::array{"Right hip", "Right knee", "Right foot"}};
        constexpr std::array<std::string_view, 2> kWeaponAttachments{"Shield", "Weapon"};
        constexpr std::array kWeaponLabels{"Left weapon", "Right weapon"};
        constexpr std::array kForearmTwists{
            std::array<std::string_view, 2>{"NPC L ForearmTwist1 [LLt1]", "NPC L ForearmTwist2 [LLt2]"},
            std::array<std::string_view, 2>{"NPC R ForearmTwist1 [RLt1]", "NPC R ForearmTwist2 [RLt2]"}};
        constexpr std::array kUpperArmTwists{
            std::array<std::string_view, 2>{"NPC L UpperarmTwist1 [LUt1]", "NPC L UpperarmTwist2 [LUt2]"},
            std::array<std::string_view, 2>{"NPC R UpperarmTwist1 [RUt1]", "NPC R UpperarmTwist2 [RUt2]"}};
        constexpr std::array kTwistNames{kUpperArmTwists, kForearmTwists};
        constexpr std::array kTwistLabels{
            std::array{std::array{"Left upper-arm twist 1", "Left upper-arm twist 2"},
                       std::array{"Right upper-arm twist 1", "Right upper-arm twist 2"}},
            std::array{std::array{"Left forearm twist 1", "Left forearm twist 2"},
                       std::array{"Right forearm twist 1", "Right forearm twist 2"}}};
        constexpr std::array<std::string_view, 2> kCenter{"NPC COM [COM ]", "NPC Pelvis [Pelv]"};
        constexpr std::array<std::string_view, 3> kChest{"NPC Spine2 [Spn2]", "NPC Spine1 [Spn1]", "NPC Spine [Spn0]"};
        constexpr std::array<std::string_view, 2> kHead{"NPC Neck [Neck]", "NPC Head [Head]"};
        constexpr std::string_view kEye = "NPCEyeBone";
        constexpr std::array<std::string_view, 2> kClavicles{"NPC L Clavicle [LClv]", "NPC R Clavicle [RClv]"};
        constexpr std::array<std::string_view, 2> kToes{"NPC L Toe0 [LToe]", "NPC R Toe0 [RToe]"};
        constexpr std::array<std::string_view, tailJointCount> kTail{"TailBone01", "TailBone02", "TailBone03", "TailBone04", "TailBone05"};
        constexpr std::array kTailLabels{"Tail base", "Tail 2", "Tail 3", "Tail 4"};
        constexpr auto kSceneBoneIndex = std::numeric_limits<std::uint32_t>::max();
        struct ExtraHandle
        {
            std::string_view bone, markerGuide;
            const char* label;
            Vec3 markerOffset;
        };
        constexpr std::array kExtras{
            ExtraHandle{kCenter[1], {}, "Pelvis tilt", {0, 0, -7}},
            ExtraHandle{kChest[2], {}, "Lower back", {0, 0, 8}},
            ExtraHandle{kClavicles[0], kLimbs[0][0], "Left clavicle", {}},
            ExtraHandle{kClavicles[1], kLimbs[1][0], "Right clavicle", {}},
            ExtraHandle{kToes[0], {}, "Left toes", {}},
            ExtraHandle{kToes[1], {}, "Right toes", {}},
        };
        enum class HandleAction { Rotate, Torso, Bend, Endpoint, Tail, RotationOnly, Twist, Weapon };

        bool IsTriangle(HandleAction action)
        {
            return action == HandleAction::RotationOnly || action == HandleAction::Twist || action == HandleAction::Weapon;
        }

        bool IsTwistName(std::string_view name)
        {
            return !std::ranges::search(name, std::string_view{"twist"},
                [](unsigned char value, char lower) { return std::tolower(value) == lower; }).empty();
        }

        struct FrozenBone
        {
            std::uint32_t index;
            BoneRef ref;
            RE::NiTransform local;
            std::optional<RE::NiTransform> rootWorld;
            std::string_view name;
            bool carriesTranslation{}, aboveCom{}, manualRotation{}, seatAnchor{};
            std::optional<RE::NiTransform> seatParent;
            BoneRef tailSlot{};
            struct RigReference { Transform local, world; };
            std::optional<RigReference> reference{};
        };

        struct Limb
        {
            struct TwistBone
            {
                std::size_t bone, depth;
                std::optional<Quaternion> relativeRotation;
                double fraction;
            };
            struct SegmentTwist
            {
                SegmentTwistFrame frame;
                std::vector<TwistBone> bones;
            };
            std::array<std::size_t, 3> joints{};
            std::vector<BoneRef> path;
            std::size_t middle{};
            std::optional<LimbHinge> hinge;
            std::array<std::optional<SegmentTwist>, 2> twists;
            struct Clavicle
            {
                std::size_t bone;
                Quaternion capturedLocal;
            };
            std::optional<Clavicle> clavicle;
        };

        struct Handle
        {
            std::size_t bone;
            HandleAction action;
            const char* label;
            std::size_t limb{};
            bool poser{};
            Vec3 lever{6, 0, 0};
            std::size_t rollSegment{};
        };

        struct Hand
        {
            struct Finger { std::size_t bone; Quaternion rest; };
            std::optional<std::size_t> wrist;
            std::array<std::optional<Finger>, Fingers::jointCount> fingers;
        };

        struct MirrorPair
        {
            struct Side { std::size_t bone; BoneMirrorFrame reference; };
            std::array<Side, 2> sides;
        };

        struct GazeRig
        {
            std::size_t head;
            std::optional<std::size_t> eye;
            Vec3 right, forward, up;
        };

        struct EditorFrame
        {
            struct Bone
            {
                Transform local;
                float scale{};
                std::optional<Transform> root;
                float rootScale{};
                bool manual{};
            };
            Vec3 translation;
            std::vector<Bone> bones;
            std::array<double, kLimbs.size() * 4> twistPhases{};
            std::optional<FaceActor::Sample> face;
        };

        struct SameEditorFrame
        {
            static bool SameFace(const std::optional<FaceActor::Sample>& a, const std::optional<FaceActor::Sample>& b)
            {
                if (!a || !b) return a.has_value() == b.has_value();
                const auto sameWeight = [](float left, float right) { return std::abs(left - right) <= 1e-5F; };
                return a->supported == b->supported && a->pose.mood == b->pose.mood &&
                    sameWeight(a->pose.strength, b->pose.strength) &&
                    std::ranges::equal(a->pose.weights, b->pose.weights, sameWeight) &&
                    std::ranges::equal(a->expressions, b->expressions, sameWeight);
            }

            bool operator()(const EditorFrame& a, const EditorFrame& b) const
            {
                const auto same = [](const Transform& left, const Transform& right) {
                    return Length(left.position - right.position) <= .0002 &&
                        HistoryStorage::SameRotation(left.rotation, right.rotation);
                };
                return SameFace(a.face, b.face) && Length(a.translation - b.translation) <= .0002 &&
                    std::ranges::equal(a.twistPhases, b.twistPhases, TwistPhase::SameAngle) &&
                    std::ranges::equal(a.bones, b.bones, [&](const EditorFrame::Bone& left, const EditorFrame::Bone& right) {
                        return left.manual == right.manual && std::abs(left.scale - right.scale) <= 1e-5F &&
                            same(left.local, right.local) && left.root.has_value() == right.root.has_value() &&
                            (!left.root || (std::abs(left.rootScale - right.rootScale) <= 1e-5F && same(*left.root, *right.root)));
                    });
            }
        };
        PoseHistory<EditorFrame, SameEditorFrame> editorHistory;
        std::filesystem::path editorHistoryFile;
        std::atomic_uint historyActions{};
        using BodyPreset = PoseSerialization::Preset<EditorFrame>;
        std::vector<BodyPreset> bodyPresets;
        std::filesystem::path bodyPresetsFile;
        std::string bodyPresetRig;
        std::string previousBodyPresetRig;
        std::string bodyPresetStatus;
        constexpr std::size_t maxBodyPresets = 128;
        constexpr std::string_view bodyPresetHeader = "GModPose body preset 1 ";

        struct Capture
        {
            const RE::BShkbAnimationGraph* graph{};
            const RE::hkaSkeleton* skeleton{};
            const RE::Actor* actor{};
            const RE::TESObjectCELL* cell{};
            const RE::NiAVObject* root{};
            std::uint32_t count{};
            std::vector<FrozenBone> bones;
            std::size_t graphBoneCount{};
            RE::NiPointer<RE::BSFlattenedBoneTree> tailTree;
            std::array<RE::NiPointer<RE::NiAVObject>, kTail.size()> sceneTail;
            std::vector<std::size_t> tailJoints;
            std::unordered_map<const RE::NiTransform*, std::size_t> byWorld;
            std::array<std::optional<Limb>, kLimbs.size()> limbs;
            std::array<Hand, 2> hands;
            std::array<std::vector<MirrorPair>, 2> mirrorPairs;
            Vec3 mirrorNormal{1, 0, 0};
            std::optional<GazeRig> gaze;
            struct Breathing
            {
                std::size_t chest;
                Vec3 chestAxis;
                BreathingCycle cycle;
                std::optional<RE::NiPoint3> offset{};
            };
            std::optional<Breathing> breathing;
            std::vector<Handle> handles;
            std::optional<std::size_t> comIndex;
            Vec3 translation{};
            Transform historyOrigin{};
            std::optional<FaceActor::Sample> historyFace;
            std::optional<Fingers::Side> editingHand;
            bool historyReady{};
            std::chrono::steady_clock::time_point nextSharingRefresh{};
            std::optional<Ray> sharingRefreshView;
            bool seated{}, rebaseForSharing{}, translationReported{}, sharingRefreshOffset{};
        } pose;

        // The main loop holds the capture while Skyrim plays the normal exit.
        struct PendingSeat
        {
            Capture capture;
            RE::ObjectRefHandle furniture;
            std::uint32_t markerID;
            std::chrono::steady_clock::time_point deadline;
            std::uint64_t generation;
        };
        std::optional<PendingSeat> pendingSeat;
        struct PendingThirdPerson
        {
            const RE::TESObjectCELL* cell;
            std::chrono::steady_clock::time_point deadline;
            std::uint64_t generation;
        };
        std::optional<PendingThirdPerson> pendingThirdPerson;

        struct Input
        {
            std::vector<ButtonChange> buttons;
            std::vector<std::variant<HistoryAction, BodyPresetCommand>> actions;
            std::vector<std::uint8_t> panelClicks;
            std::vector<PoseSettings::TextEvent> panelText;
            bool poserPressed{}, toggleMirror{};
            std::array<double, 2> motion{};
            double wheel{};
        } pendingInput;

        // Compare pointer input polled from the device with what reaches the sink.
        // An idle mouse gives no evidence that another overlay took input.
        EditorKeys::State deliveredKeys;
        bool mousePending{}, mouseDelivered{}, mouseCaptured{};
        std::atomic_bool closeSettings{false};
        // Hooks and poser callbacks hand off input under inputMutex. Animation
        // hooks read the captured skeleton and its edits under poseMutex.
        std::mutex poseMutex, inputMutex;
        std::atomic_bool active{false}, ready{false}, loading{false};
        std::atomic_bool rotationHeld{false}, settingsOpen{false}, fingerOpen{false}, faceOpen{false}, bodyOpen{false};
        std::atomic<RE::BSFlattenedBoneTree*> activeTailTree{};
        std::atomic_uint commands{}, threadID{};
        std::atomic<RE::ControlMap*> textInputOwner{nullptr};
        std::atomic_uint64_t generation{};
        bool updateInstalled{};

        struct EndpointGrip
        {
            struct Link
            {
                std::optional<std::size_t> bone;
                Quaternion bind;
            };
            struct Twist
            {
                std::size_t bone;
                Quaternion bindRelative;
                double fraction;
            };
            EndpointReference rig;
            Quaternion parent;
            std::vector<Link> links;
            std::array<std::vector<Twist>, 2> twists;
        };

        // Main-loop only, with no engine pointers retained.
        struct Interaction
        {
            EditorButtons buttons;
            struct Grip
            {
                std::size_t handle;
                bool rotating;
                HoldDistance distance;
                Vec3 point, lastTarget, translation, keyboardOffset{};
                Transform bone;
                std::optional<LimbPose> limb;
                std::optional<Vec3> bend;
                std::optional<Quaternion> targetRotation;
                std::array<std::optional<Transform>, kLimbs.size()> anchors;
                std::optional<Quaternion> followingLowerRotation{};
                struct Support
                {
                    Transform initial;
                    std::optional<Quaternion> preferred;
                    bool applied{};
                };
                std::optional<Support> clavicle{};
                std::optional<TorsoClearance> clearance{};
                Vec3 viewForward{};
                std::optional<EndpointGrip> endpoint{};
                std::optional<Quaternion> endpointFlex{};
                bool learnedStance{};
                struct Neck { std::size_t bone; Quaternion rotation; };
                std::optional<Neck> neck{};
                std::array<bool, kLimbs.size()> supportChecked{};
                struct Tail
                {
                    std::array<Transform, tailJointCount> joints;
                    std::array<Quaternion, tailJointCount> parentOffsets;
                    std::array<Vec3, tailJointCount - 1> linkAxes;
                    std::array<double, tailJointCount - 1> lengths;
                    std::size_t held;
                };
                std::optional<Tail> tail{};
            };
            std::optional<Grip> grip;
            std::optional<GripRotation> rotation;
            TetherCurve beam;
            struct Sharing
            {
                const RE::NiAVObject* root;
                const RE::TESObjectCELL* cell;
                std::optional<RE::WEAPON_STATE> weapon;
                double retry{}, stableFor{};
                unsigned attempts{};
                bool started{}, requestedDrawn{};
            };
            std::optional<Sharing> sharing;
            RE::ObjectRefHandle seat;
            std::chrono::steady_clock::time_point nextSupportSample{};
            std::array<bool, kLimbs.size()> supportedFeet{};
            bool hidden{}, playerHub{}, movingPose{}, mirror{}, nudge{};
        } editor;

        void Notify(const char* text)
        {
            auto* queue = RE::UIMessageQueue::GetSingleton();
            auto* factories = RE::MessageDataFactoryManager::GetSingleton();
            if (!queue || !factories) return;
            auto* factory = factories->GetCreator<RE::HUDData>(RE::HUDData::CLASS_NAME);
            if (!factory) return;
            auto* data = factory->Create();
            if (!data) return;
            data->type = RE::HUD_MESSAGE_TYPE::kNotification;
            data->text = text;
            queue->AddMessage(RE::BSFixedString{RE::HUDMenu::MENU_NAME}, RE::UI_MESSAGE_TYPE::kUpdate, data);
        }

        const RE::hkaSkeleton* Skeleton(const RE::BShkbAnimationGraph& graph)
        {
            const auto* setup = graph.characterInstance.setup.get();
            return setup ? setup->animationSkeleton.get() : nullptr;
        }

        std::string_view Name(const RE::hkStringPtr& value)
        {
            static_assert(sizeof(value) == sizeof(std::uintptr_t));
            std::uintptr_t bits{};
            std::memcpy(&bits, &value, sizeof(bits));
            // Havok uses the low bit as an ownership flag on the string pointer.
            const auto* text = reinterpret_cast<const char*>(bits & ~std::uintptr_t{1});
            return text ? std::string_view{text} : std::string_view{};
        }

        std::optional<Transform> ReadTransform(const RE::NiTransform* value)
        {
            if (!value || !std::isfinite(value->scale) || value->scale <= 0) return std::nullopt;
            const Vec3 position{value->translate.x, value->translate.y, value->translate.z};
            if (!IsFinite(position)) return std::nullopt;
            for (const auto& row : value->rotate.entry)
                for (float component : row)
                    if (!std::isfinite(component)) return std::nullopt;
            const RE::NiQuaternion rotation{value->rotate};
            const auto q = Unit(Quaternion{rotation.x, rotation.y, rotation.z, rotation.w});
            if (q) return Transform{position, *q};
            return std::nullopt;
        }

        RE::NiPoint3 Native(Vec3 v) { return {float(v.x), float(v.y), float(v.z)}; }

        void WriteRotation(RE::NiTransform& transform, Quaternion q)
        {
            RE::NiQuaternion{float(q.w), float(q.x), float(q.y), float(q.z)}.ToRotation(transform.rotate);
        }

        std::optional<EditorFrame> ReadEditorFrame()
        {
            EditorFrame frame;
            // Saved frames follow the actor's position and heading at the next capture.
            frame.translation = *Rotate(Fingers::Conjugate(pose.historyOrigin.rotation), pose.translation);
            frame.face = pose.historyFace;
            frame.bones.reserve(pose.bones.size());
            for (const auto& frozen : pose.bones) {
                const auto local = ReadTransform(&frozen.local);
                if (!local) return std::nullopt;
                EditorFrame::Bone saved{*local, frozen.local.scale, {}, 0, frozen.manualRotation};
                if (frozen.rootWorld) {
                    const auto world = ReadTransform(&*frozen.rootWorld);
                    saved.root = world ? RelativeTo(pose.historyOrigin, *world) : std::nullopt;
                    if (!saved.root) return std::nullopt;
                    saved.rootScale = frozen.rootWorld->scale;
                }
                frame.bones.push_back(saved);
            }
            std::size_t phase = 0;
            for (const auto& limb : pose.limbs)
                for (std::size_t segment = 0; segment < 2; ++segment, phase += 2)
                    if (limb && limb->twists[segment]) {
                        frame.twistPhases[phase] = limb->twists[segment]->frame.proximal.angle;
                        frame.twistPhases[phase + 1] = limb->twists[segment]->frame.distal.angle;
                    }
            return frame;
        }

        FacePose::Mask RestoreFace(const FacePose::Edit& edit)
        {
            if (!pose.historyFace) return 0;
            auto& face = *pose.historyFace;
            const auto touched = edit.changed & face.supported;
            FacePose::Merge(face.pose, edit.pose, touched);
            if (touched & FacePose::Bit(FacePose::MoodSlot)) face.expressions = edit.expressions;
            return touched;
        }

        enum class FrameSource { History, Preset };

        FacePose::Mask ApplyEditorFrame(const EditorFrame& frame, FrameSource source)
        {
            const auto write = [source](RE::NiTransform& target, Transform value, float scale,
                                        std::string_view bone, const char* space) {
                target.translate = Native(value.position);
                WriteRotation(target, value.rotation);
                // Presets can come from another actor with the same rig layout.
                if (source == FrameSource::History) target.scale = scale;
                else if (std::abs(target.scale - scale) > 1e-4F)
                    SKSE::log::debug("Body preset {} scale on {}: saved {:.4f}, kept {:.4f}",
                        space, bone, scale, target.scale);
            };
            if (!pose.seated) pose.translation = *Rotate(pose.historyOrigin.rotation, frame.translation);
            for (std::size_t i = 0; i < std::min(pose.bones.size(), frame.bones.size()); ++i) {
                auto& frozen = pose.bones[i];
                if (frozen.seatAnchor) continue;
                const auto& saved = frame.bones[i];
                write(frozen.local, saved.local, saved.scale, frozen.name, "local");
                if (frozen.rootWorld)
                    write(*frozen.rootWorld, *Compose(pose.historyOrigin, *saved.root), saved.rootScale, frozen.name, "root");
                frozen.manualRotation = saved.manual;
            }
            std::size_t phase = 0;
            for (auto& limb : pose.limbs)
                for (std::size_t segment = 0; segment < 2; ++segment, phase += 2)
                    if (limb && limb->twists[segment]) {
                        limb->twists[segment]->frame.proximal.angle = frame.twistPhases[phase];
                        limb->twists[segment]->frame.distal.angle = frame.twistPhases[phase + 1];
                    }
            return frame.face ? RestoreFace({frame.face->pose, frame.face->supported, frame.face->expressions}) : 0;
        }

        constexpr unsigned HistoryBit(HistoryAction action) { return 1U << static_cast<unsigned>(action); }

        void PublishHistoryActions()
        {
            unsigned enabled = 0;
            for (const auto action : {HistoryAction::Undo, HistoryAction::Redo})
                if (editorHistory.CanMove(action)) enabled |= HistoryBit(action);
            historyActions.store(enabled);
        }

        void CommitHistory()
        {
            if (!pose.historyReady) return;
            if (auto frame = ReadEditorFrame()) editorHistory.Commit(std::move(*frame));
            PublishHistoryActions();
        }

        void WriteEditorFrame(std::ostream& out, const EditorFrame& frame)
        {
            const auto point = [&](Vec3 value) { out << ' ' << value.x << ' ' << value.y << ' ' << value.z; };
            const auto write = [&](Transform value, float scale) {
                point(value.position);
                HistoryStorage::WriteQuaternion(out, value.rotation);
                out << ' ' << scale;
            };
            point(frame.translation);
            for (const auto& bone : frame.bones) {
                write(bone.local, bone.scale);
                out << ' ' << bone.manual;
                if (bone.root) write(*bone.root, bone.rootScale);
            }
            for (const auto angle : frame.twistPhases) out << ' ' << angle;
            out << ' ' << frame.face.has_value();
            if (frame.face) {
                const auto& face = *frame.face;
                out << ' ' << face.supported << ' ' << face.pose.mood << ' ' << face.pose.strength;
                for (const auto value : face.pose.weights) out << ' ' << value;
                for (const auto value : face.expressions) out << ' ' << value;
            }
        }

        bool DecodeEditorFrame(std::istream& in, EditorFrame& frame, std::size_t boneCount)
        {
            const auto number = [&](double& value) { return (in >> value) && std::abs(value) <= 1e7; };
            const auto point = [&](Vec3& value) { return number(value.x) && number(value.y) && number(value.z); };
            const auto read = [&](Transform& value, float& scale) {
                return point(value.position) && HistoryStorage::ReadQuaternion(in, value.rotation) &&
                    (in >> scale) && scale > 0 && scale <= 100;
            };
            if (!point(frame.translation)) return false;
            frame.bones.resize(boneCount);
            for (std::size_t i = 0; i < frame.bones.size(); ++i) {
                auto& bone = frame.bones[i];
                if (!read(bone.local, bone.scale) || !(in >> bone.manual)) return false;
                if (pose.bones[i].rootWorld && !read(bone.root.emplace(), bone.rootScale))
                    return false;
            }
            bool hasFace{};
            if (!std::ranges::all_of(frame.twistPhases, number) || !(in >> hasFace)) return false;
            if (!hasFace) return true;
            auto& face = frame.face.emplace();
            const auto weight = [&](float& value) { return (in >> value) && value >= 0 && value <= 1; };
            return (in >> face.supported >> face.pose.mood) && (face.supported & ~FacePose::all) == 0 &&
                face.pose.mood >= FacePose::neutralMood && face.pose.mood < FacePose::neutralMood + FacePose::moodCount &&
                weight(face.pose.strength) && std::ranges::all_of(face.pose.weights, weight) &&
                std::ranges::all_of(face.expressions, weight);
        }

        void SortBodyPresets()
        {
            std::ranges::sort(bodyPresets, {}, [](const BodyPreset& entry) { return PresetStorage::Fold(entry.name); });
        }

        bool StoreBodyPreset(const std::filesystem::path& path, const BodyPreset& entry)
        {
            using namespace PosePresetJson;
            auto document = Document("body", entry.name, entry.category, PoseSerialization::EncodeBody(entry.pose, bodyPresetRig));
            document["source"] = entry.source.empty() ? "GModPose" : entry.source;
            return PresetStorage::WriteJson(path, document);
        }

        void OpenBodyPresets()
        {
            bodyPresets = PoseSerialization::ReadFolder<BodyPreset>(bodyPresetsFile, "body", maxBodyPresets,
                [](const auto& object, BodyPreset& item) {
                    const auto rooted = [](std::size_t index) { return pose.bones[index].rootWorld.has_value(); };
                    if (!previousBodyPresetRig.empty() && PosePresetJson::StringIs(
                            PosePresetJson::Field(object, "rig"), previousBodyPresetRig))
                        return PoseSerialization::DecodeBody(object, item.pose, previousBodyPresetRig, pose.graphBoneCount, rooted);
                    return PoseSerialization::DecodeBody(object, item.pose, bodyPresetRig, pose.bones.size(), rooted);
                });
            auto legacy = PresetStorage::ReadLegacyFolder<BodyPreset>(bodyPresetsFile,
                std::string(bodyPresetHeader) + bodyPresetRig, maxBodyPresets,
                [](std::istream& in, BodyPreset& item) { return DecodeEditorFrame(in, item.pose, pose.bones.size()); });
            PresetStorage::MigrateFiles(bodyPresetsFile, bodyPresets, std::move(legacy), maxBodyPresets, StoreBodyPreset);
            if (!previousBodyPresetRig.empty()) {
                auto graphPresets = PresetStorage::ReadLegacyFolder<BodyPreset>(bodyPresetsFile,
                    std::string(bodyPresetHeader) + previousBodyPresetRig, maxBodyPresets,
                    [](std::istream& in, BodyPreset& item) { return DecodeEditorFrame(in, item.pose, pose.graphBoneCount); });
                // Old body presets leave the newly captured tail joints at their current pose.
                for (auto& item : graphPresets) {
                    if (bodyPresets.size() == maxBodyPresets) break;
                    if (PresetStorage::FindName(bodyPresets, item.name) == bodyPresets.end())
                        bodyPresets.push_back(std::move(item));
                }
            }
            SortBodyPresets();
            bodyPresetStatus.clear();
        }

        bool ProcessBodyPreset(const BodyPresetCommand& command, FacePose::Mask& faceTouched)
        {
            using Kind = BodyPresetCommand::Kind;
            const auto name = PresetStorage::TrimName(command.name);
            if (!PresetStorage::ValidName(name)) {
                bodyPresetStatus = "Enter a pose name of up to 64 characters.";
                return false;
            }
            const auto choice = PresetStorage::FindName(bodyPresets, name);
            if (command.kind == Kind::Load) {
                if (choice == bodyPresets.end()) {
                    bodyPresetStatus = "That pose is unavailable.";
                    return false;
                }
                faceTouched |= ApplyEditorFrame(choice->pose, FrameSource::Preset);
                CommitHistory();
                bodyPresetStatus = "Loaded " + choice->name + ".";
                return true;
            }
            const bool overwrite = command.kind == Kind::Overwrite;
            if (overwrite != (choice != bodyPresets.end())) {
                bodyPresetStatus = overwrite ? "That pose is unavailable." : "A pose with that name already exists.";
                return false;
            }
            if (!overwrite && bodyPresets.size() >= maxBodyPresets) {
                bodyPresetStatus = "Body preset library is full.";
                return false;
            }
            auto frame = ReadEditorFrame();
            auto saved = overwrite ? *choice : BodyPreset{};
            if (!overwrite) saved.name = name;
            const auto path = saved.path.extension() == ".json" ? std::optional{saved.path} :
                PresetStorage::NewFile(bodyPresetsFile, name);
            if (!frame || !path) {
                bodyPresetStatus = "Could not save the pose.";
                return false;
            }
            saved.pose = std::move(*frame);
            if (!StoreBodyPreset(*path, saved)) {
                bodyPresetStatus = "Could not save the pose.";
                return false;
            }
            const auto savedName = saved.name;
            const bool hasFace = saved.pose.face.has_value();
            if (saved.path.extension() == ".gmodpose") PresetStorage::ArchiveLegacyFile(saved.path);
            saved.path = *path;
            if (overwrite) *choice = std::move(saved);
            else bodyPresets.push_back(std::move(saved));
            SortBodyPresets();
            bodyPresetStatus = (overwrite ? "Overwrote " : "Saved ") + savedName + ".";
            if (!hasFace) bodyPresetStatus += " Face controls were unavailable.";
            return false;
        }

        void SaveHistory()
        {
            CommitHistory();
            if (!HistoryStorage::Save(editorHistoryFile, "editor", editorHistory, WriteEditorFrame)) SKSE::log::warn("Could not save GModPose editor history");
        }

        void BeginHistory()
        {
            // Frame transforms are stored in capture order. Hash the rig so a
            // different skeleton cannot load transforms by the wrong indices.
            std::uint64_t rig = 14695981039346656037ULL;
            const auto hash = [&](std::uint64_t value) { rig = (rig ^ value) * 1099511628211ULL; };
            hash(pose.count);
            std::optional<std::uint64_t> previousRig;
            for (std::size_t i = 0; i < pose.bones.size(); ++i) {
                if (i == pose.graphBoneCount) previousRig = rig;
                const auto& bone = pose.bones[i];
                hash(bone.index);
                hash(bone.rootWorld.has_value());
                for (const unsigned char letter : bone.name) hash(letter);
                if (pose.skeleton && bone.index < static_cast<std::uint32_t>(pose.skeleton->referencePose.size())) {
                    std::array<float, 4> offset;
                    _mm_storeu_ps(offset.data(), pose.skeleton->referencePose[bone.index].translation.quad);
                    for (const auto value : offset) hash(std::bit_cast<std::uint32_t>(value));
                }
            }
            const auto historyPath = [](std::uint64_t key) {
                return std::filesystem::path(REL::Module::get().filePath().data()).parent_path() /
                    (L"Data/SKSE/Plugins/GModPose-history-" + std::to_wstring(key) + L".txt");
            };
            const auto path = historyPath(rig);
            auto frame = ReadEditorFrame();
            if (path != editorHistoryFile) {
                editorHistoryFile = path;
                editorHistory = {};
                const auto read = [](std::istream& in, EditorFrame& state) { return DecodeEditorFrame(in, state, pose.bones.size()); };
                if (!HistoryStorage::Load(path, "editor", editorHistory, read) && previousRig && frame) {
                    // Existing checkpoints predate the tail capture. Use its entry pose for those joints.
                    HistoryStorage::Load(historyPath(*previousRig), "editor", editorHistory, [&](std::istream& in, EditorFrame& state) {
                        if (!DecodeEditorFrame(in, state, pose.graphBoneCount)) return false;
                        state.bones.insert(state.bones.end(), frame->bones.begin() + pose.graphBoneCount, frame->bones.end());
                        return true;
                    });
                }
                bodyPresetsFile = PresetStorage::Folder(path.parent_path(), L"Body");
                bodyPresetRig = std::to_string(rig);
                previousBodyPresetRig = previousRig ? std::to_string(*previousRig) : std::string{};
            }
            if (frame) {
                editorHistory.Commit(std::move(*frame));
                pose.historyReady = true;
            }
            PublishHistoryActions();
        }

        RE::NiTransform RootWorld(const FrozenBone& bone)
        {
            auto world = *bone.rootWorld;
            if (bone.carriesTranslation) world.translate += Native(pose.translation);
            return world;
        }

        class WorldPose
        {
        public:
            std::optional<RE::NiTransform> Get(BoneRef bone, unsigned depth = 0)
            {
                const auto* key = bone.World();
                if (!key || depth >= 64) return std::nullopt;
                if (const auto known = worlds.find(key); known != worlds.end()) return known->second;
                const auto index = pose.byWorld.find(key);
                const auto* frozen = index != pose.byWorld.end() ? &pose.bones[index->second] : nullptr;
                std::optional<RE::NiTransform> result;
                if (frozen && frozen->rootWorld) result = RootWorld(*frozen);
                else if (frozen && frozen->seatParent) {
                    result = *frozen->seatParent * frozen->local;
                    if (frozen->carriesTranslation) result->translate += Native(pose.translation);
                }
                else if (const auto parent = bone.Parent(); parent.node) {
                    const auto parentWorld = Get(parent, depth + 1);
                    const auto* local = frozen ? &frozen->local : bone.Local();
                    if (parentWorld && local) {
                        result = *parentWorld * *local;
                        if (frozen && frozen->carriesTranslation) result->translate += Native(pose.translation);
                    }
                } else result = *key;
                if (result) worlds.emplace(key, *result);
                return result;
            }

            std::optional<Transform> Read(BoneRef bone)
            {
                const auto transform = Get(bone);
                return transform ? ReadTransform(&*transform) : std::nullopt;
            }

        private:
            std::unordered_map<const RE::NiTransform*, RE::NiTransform> worlds;
        };

        bool TurnWholePose(double radians)
        {
            if (radians == 0) return false;
            const auto firstCarrier = std::ranges::find(pose.bones, true, &FrozenBone::carriesTranslation);
            if (firstCarrier == pose.bones.end()) return false;
            WorldPose worlds;
            const auto center = worlds.Read(pose.comIndex ? pose.bones[*pose.comIndex].ref : firstCarrier->ref);
            if (!center) return false;
            const auto turn = AxisAngle({0, 0, 1}, radians);
            struct Write { FrozenBone* bone; RE::NiTransform transform; };
            std::vector<Write> writes;
            for (auto& bone : pose.bones) {
                if (!bone.carriesTranslation) continue;
                auto world = worlds.Get(bone.ref);
                const auto frame = world ? ReadTransform(&*world) : std::nullopt;
                const auto target = frame ? RotateAround(*frame, center->position, turn) : std::nullopt;
                if (!target) return false;
                world->translate = Native(target->position - pose.translation);
                WriteRotation(*world, target->rotation);
                if (!bone.rootWorld) {
                    const auto parent = worlds.Get(bone.ref.Parent());
                    if (!parent || !ReadTransform(&*parent)) return false;
                    *world = parent->Invert() * *world;
                    if (!ReadTransform(&*world)) return false;
                }
                writes.push_back({&bone, *world});
            }
            // Commit only after all carriers can be rotated. Keeping the turn
            // on the COM and separate branches also preserves pose sharing.
            for (auto& write : writes) {
                if (write.bone->rootWorld) write.bone->rootWorld = write.transform;
                else write.bone->local = write.transform;
            }
            return true;
        }

        bool SameGraph(const RE::BShkbAnimationGraph& graph, const Capture& capture)
        {
            return &graph == capture.graph && graph.holder == capture.actor && graph.rootNode == capture.root &&
                Skeleton(graph) == capture.skeleton && graph.boneNodes.size() == capture.count;
        }

        template <std::size_t Slot> struct TailUpdateHook
        {
            static void Thunk(RE::BSFlattenedBoneTree* tree, RE::NiUpdateData& data, std::uint32_t flags)
            {
                if (Running() && tree == activeTailTree.load(std::memory_order_acquire)) {
                    const std::scoped_lock lock(poseMutex);
                    if (Running() && tree == pose.tailTree.get()) {
                        // The scene tree can update tail joints outside the body's Havok pose copy.
                        // Hold both forms of the local transform before Skyrim builds skin matrices.
                        for (const auto& bone : std::span(pose.bones).subspan(pose.graphBoneCount)) {
                            if (const auto* slot = bone.tailSlot.Slot()) {
                                *bone.tailSlot.Local() = bone.local;
                                if (slot->node) slot->node->local = bone.local;
                            }
                        }
                    }
                }
                original(tree, data, flags);
            }
            static inline HookOriginal<decltype(Thunk)> original;
        };

        std::optional<RE::NiTransform> LiveParentFrame(BoneRef bone, unsigned depth = 0)
        {
            const auto* world = bone.World();
            if (!world || depth >= 64) return std::nullopt;
            const auto parent = bone.Parent();
            const auto* local = bone.Local();
            const auto frozen = pose.byWorld.find(world);
            if (!parent.node || !local || (bone.node == pose.root && frozen == pose.byWorld.end()))
                return ReadTransform(world) ? std::optional{*world} : std::nullopt;
            if (frozen != pose.byWorld.end() && !pose.bones[frozen->second].aboveCom)
                return std::nullopt;
            const auto parentWorld = LiveParentFrame(parent, depth + 1);
            if (!parentWorld || !ReadTransform(local)) return std::nullopt;
            return *parentWorld * *local;
        }

        void Apply(RE::BShkbAnimationGraph& graph)
        {
            if (!Running()) return;
            std::lock_guard lock(poseMutex);
            if (!Running() || !SameGraph(graph, pose)) return;
            WorldPose frozenWorlds;
            for (const auto& frozen : pose.bones) {
                if (pose.rebaseForSharing && frozen.aboveCom) continue;
                const auto ref = frozen.index == kSceneBoneIndex ? frozen.ref : GraphBoneRef(graph.boneNodes[frozen.index]);
                const auto parent = ref.Parent();
                const BoneTransforms bone{ref.Local(), ref.World(), parent.World()};
                if (!bone.local) continue;
                *bone.local = frozen.local;
                // PlayerHub samples COM in parent space. Put the difference
                // between the live and captured parent frames into the COM
                // itself, so landing after a capture survives replication.
                if (pose.rebaseForSharing && frozen.carriesTranslation && !frozen.rootWorld) {
                    const auto frame = LiveParentFrame(parent);
                    const auto target = frozenWorlds.Get(frozen.ref);
                    const auto parentFrame = frame ? ReadTransform(&*frame) : std::nullopt;
                    if (parentFrame && target) {
                        const auto local = frame->Invert() * *target;
                        if (ReadTransform(&local)) {
                            *bone.local = local;
                            if (pose.sharingRefreshOffset && &frozen == &pose.bones[*pose.comIndex]) {
                                const auto& t = local.translate;
                                const Vec3 position{t.x, t.y, t.z};
                                Vec3 direction{1, 0, 0};
                                if (pose.sharingRefreshView) {
                                    const auto& view = *pose.sharingRefreshView;
                                    const auto& world = target->translate;
                                    const auto sightline = Unit(Vec3{world.x, world.y, world.z} - view.origin);
                                    if (const auto localDirection =
                                        LocalTranslationOffset(sightline.value_or(view.direction), parentFrame->rotation, 1))
                                        direction = *localDirection;
                                }
                                const auto refreshLimit = 1.0 / refreshConfig.resolution;
                                bone.local->translate = Native(position + PoseRefreshOffset(position, direction, refreshLimit));
                            }
                            continue;
                        }
                    }
                }
                if (frozen.rootWorld) {
                    const auto world = RootWorld(frozen);
                    if (!bone.parentWorld) *bone.local = world;
                    else if (ReadTransform(bone.parentWorld)) {
                        const auto local = bone.parentWorld->Invert() * world;
                        if (ReadTransform(&local)) *bone.local = local;
                    }
                } else if (frozen.carriesTranslation && Length(pose.translation) > 0 && bone.parentWorld) {
                    if (const auto parent = ReadTransform(bone.parentWorld)) {
                        if (const auto offset = LocalTranslationOffset(pose.translation, parent->rotation, bone.parentWorld->scale))
                            bone.local->translate += Native(*offset);
                    }
                }
            }
            // Add the idle offset only to the live skeleton. Editing and
            // serialization always read the frozen transforms.
            if (const auto& breathing = pose.breathing; breathing && breathing->offset) {
                const auto& chest = pose.bones[breathing->chest];
                if (auto* local = GraphBoneRef(graph.boneNodes[chest.index]).Local())
                    local->translate += *breathing->offset;
            }
        }

        template <unsigned Site> struct CopyHook
        {
            static void Thunk(RE::BShkbAnimationGraph* graph, const std::uint8_t* flag)
            {
                // Let Havok copy the native pose first. Skinning and pose sync
                // then read the editor's replacement transforms from those bones.
                original(graph, flag);
                if (graph) Apply(*graph);
            }
            static inline HookOriginal<decltype(Thunk)> original;
        };

        template <class Callback> bool WithGraph(RE::PlayerCharacter& player, Callback&& callback)
        {
            RE::BSTSmartPointer<RE::BSAnimationGraphManager> manager;
            const auto* root = player.Get3D(false);
            if (!root || !player.GetAnimationGraphManager(manager) || !manager) return false;
            RE::BSSpinLockGuard lock(manager->GetRuntimeData().updateLock);
            for (const auto& graph : manager->graphs) {
                if (graph && graph->rootNode == root && graph->holder == &player)
                    return callback(*graph);
            }
            return false;
        }

        std::optional<Limb> CaptureLimb(const Capture& next, const JointNames& names)
        {
            Limb limb;
            for (std::size_t i = 0; i < names.size(); ++i) {
                const auto found = std::ranges::find(next.bones, names[i], &FrozenBone::name);
                if (found == next.bones.end()) return std::nullopt;
                limb.joints[i] = found - next.bones.begin();
            }
            const auto upper = next.bones[limb.joints[0]].ref, middle = next.bones[limb.joints[1]].ref;
            for (auto bone = next.bones[limb.joints[2]].ref; bone.node && limb.path.size() < 32; bone = bone.Parent()) {
                if (std::ranges::find(limb.path, bone) != limb.path.end()) break;
                limb.path.push_back(bone);
                if (bone == upper) break;
            }
            if (limb.path.size() < 3 || limb.path.back() != upper) return std::nullopt;
            std::ranges::reverse(limb.path);
            const auto joint = std::ranges::find(limb.path, middle);
            if (joint == limb.path.begin() || joint == limb.path.end() || joint + 1 == limb.path.end()) return std::nullopt;
            limb.middle = joint - limb.path.begin();
            return limb;
        }

        bool IsAncestor(BoneRef ancestor, BoneRef node)
        {
            for (unsigned depth = 0; node.node && depth < 64; ++depth, node = node.Parent())
                if (node == ancestor) return true;
            return false;
        }

        const RE::NiAVObject* VisibleWeapon(const RE::Actor& actor, BoneRef attachment)
        {
            if (actor.AsActorState()->GetWeaponState() != RE::WEAPON_STATE::kDrawn || !attachment.node || attachment.index >= 0)
                return nullptr;
            const auto& biped = actor.GetBiped(false);
            if (!biped) return nullptr;
            const auto* left = actor.GetEquippedObject(true);
            const auto* right = actor.GetEquippedObject(false);
            // Raised fists can report a drawn stance. Require an equipped weapon mesh on this hand branch.
            for (const auto& part : biped->objects) {
                if (!part.item || (part.item != left && part.item != right) || !part.partClone) continue;
                const auto* weapon = part.item->As<RE::TESObjectWEAP>();
                if (!weapon || weapon->IsHandToHandMelee()) continue;
                const auto* mesh = part.partClone.get();
                const auto* node = mesh;
                bool attached = false;
                for (unsigned depth = 0; node && depth < 64 && !node->GetAppCulled(); ++depth, node = node->parent)
                    attached |= node == attachment.node;
                if (!node && attached) return mesh;
            }
            return nullptr;
        }

        void CaptureLimbTwists(const Capture& next, Limb& limb, const std::array<Transform, 3>& joints)
        {
            for (std::size_t i = 0; i < limb.twists.size(); ++i) {
                const auto start = next.bones[limb.joints[i]].ref;
                const auto end = next.bones[limb.joints[i + 1]].ref;
                const auto distal = RelativeTo(joints[i], joints[i + 1]);
                const auto axis = distal ? Unit(distal->position) : std::nullopt;
                if (!axis) continue;
                Limb::SegmentTwist twist{};
                const auto length = Length(distal->position);
                for (std::size_t index = 0; index < next.bones.size(); ++index) {
                    const auto& bone = next.bones[index];
                    if (!IsTwistName(bone.name) || IsAncestor(bone.ref, end) || IsAncestor(end, bone.ref)) continue;
                    auto ancestor = bone.ref.Parent();
                    std::size_t depth = 1;
                    while (ancestor.node && ancestor != start && depth < 64) {
                        ancestor = ancestor.Parent();
                        ++depth;
                    }
                    if (ancestor != start) continue;
                    const auto world = ReadTransform(bone.ref.World());
                    const auto relative = world ? RelativeTo(joints[i], *world) : std::nullopt;
                    if (!relative) continue;
                    const auto fraction = std::clamp(Dot(relative->position, *axis) / length, 0.0, 1.0);
                    twist.bones.push_back({index, depth, {}, fraction});
                }
                // Nested skin helpers need their updated parent frame first.
                std::ranges::stable_sort(twist.bones, {}, &Limb::TwistBone::depth);
                if (!twist.bones.empty()) limb.twists[i] = std::move(twist);
            }
        }

        void CaptureLimbGeometry(Capture& next, double yaw)
        {
            std::array<std::optional<std::array<Transform, 3>>, kLimbs.size()> frames;
            for (std::size_t i = 0; i < frames.size(); ++i) {
                if (!next.limbs[i]) continue;
                const auto& joints = next.limbs[i]->joints;
                const auto upper = ReadTransform(next.bones[joints[0]].ref.World());
                const auto lower = ReadTransform(next.bones[joints[1]].ref.World());
                const auto end = ReadTransform(next.bones[joints[2]].ref.World());
                if (upper && lower && end) frames[i] = std::array{*upper, *lower, *end};
            }
            Vec3 right{std::cos(yaw), -std::sin(yaw), 0}, forward{std::sin(yaw), std::cos(yaw), 0};
            if (std::ranges::all_of(frames, [](const auto& limb) { return limb.has_value(); })) {
                const auto leftHip = (*frames[2])[0].position, rightHip = (*frames[3])[0].position;
                const auto shoulders = ((*frames[0])[0].position + (*frames[1])[0].position) * .5;
                const auto up = shoulders - (leftHip + rightHip) * .5;
                right = Unit(rightHip - leftHip).value_or(right);
                forward = Unit(Cross(up, right)).value_or(forward);
            }
            for (std::size_t i = 0; i < frames.size(); ++i) {
                if (!frames[i]) continue;
                const bool leg = i >= 2;
                const double side = i % 2 == 0 ? -1 : 1;
                const auto fallback = leg ? forward + right * (side * .12) : forward * -1 + right * (side * .35);
                auto& limb = *next.limbs[i];
                limb.hinge = CaptureLimbHinge(*frames[i], fallback, leg);
                CaptureLimbTwists(next, limb, *frames[i]);
                if (!leg) {
                    const auto bone = std::ranges::find(next.bones, kClavicles[i], &FrozenBone::name);
                    const auto upper = next.bones[limb.joints[0]].ref;
                    if (bone != next.bones.end() && bone->ref != upper && IsAncestor(bone->ref, upper))
                        if (const auto local = ReadTransform(&bone->local))
                            limb.clavicle = Limb::Clavicle{static_cast<std::size_t>(bone - next.bones.begin()), local->rotation};
                }
            }
        }

        void AssignTranslationCarriers(Capture& capture, const std::optional<std::size_t>& comIndex)
        {
            // Pose replication reads NPC COM's local translation. Leave the
            // frozen frames above it stationary, and offset each sibling
            // branch once so the local pose still moves as a whole.
            std::unordered_set<const RE::NiTransform*> aboveCom;
            if (comIndex) {
                auto parent = capture.bones[*comIndex].ref.Parent();
                for (unsigned depth = 0; parent.node && depth < 64; ++depth, parent = parent.Parent())
                    if (!aboveCom.insert(parent.World()).second) break;
            }
            for (std::size_t i = 0; i < capture.bones.size(); ++i) {
                auto& bone = capture.bones[i];
                bone.aboveCom = aboveCom.contains(bone.ref.World());
                if (bone.aboveCom) continue;
                if (bone.rootWorld || comIndex == i) {
                    bone.carriesTranslation = true;
                    continue;
                }
                auto parent = bone.ref.Parent();
                for (unsigned depth = 0; parent.node && depth < 64; ++depth, parent = parent.Parent()) {
                    if (capture.byWorld.contains(parent.World())) {
                        bone.carriesTranslation = aboveCom.contains(parent.World());
                        break;
                    }
                    if (aboveCom.contains(parent.World())) {
                        bone.carriesTranslation = true;
                        break;
                    }
                }
            }
        }

        bool FingerName(std::string_view name)
        {
            return name.starts_with("NPC L Finger") || name.starts_with("NPC R Finger");
        }

        void MapFingers(Capture& captured)
        {
            if (!captured.skeleton) return;
            const auto& skeleton = *captured.skeleton;
            for (std::size_t side = 0; side < captured.hands.size(); ++side) {
                auto& hand = captured.hands[side];
                const auto wrist = std::ranges::find(captured.bones, kLimbs[side].back(), &FrozenBone::name);
                if (wrist == captured.bones.end()) continue;
                hand.wrist = wrist - captured.bones.begin();
                const char prefix = side == 0 ? 'L' : 'R';
                for (std::size_t i = 0; i < hand.fingers.size(); ++i) {
                    const auto digit = i / Fingers::jointsPerDigit, joint = i % Fingers::jointsPerDigit;
                    const auto name = std::format("NPC {} Finger{}{} [{}F{}{}]", prefix, digit, joint, prefix, digit, joint);
                    const auto bone = std::ranges::find(captured.bones, name, &FrozenBone::name);
                    if (bone == captured.bones.end() || bone->rootWorld ||
                        bone->index >= static_cast<std::uint32_t>(skeleton.referencePose.size()) ||
                        bone->index >= static_cast<std::uint32_t>(skeleton.bones.size()) ||
                        Name(skeleton.bones[bone->index].name) != name) continue;
                    std::array<float, 4> rotation;
                    _mm_storeu_ps(rotation.data(), skeleton.referencePose[bone->index].rotation.vec.quad);
                    if (const auto rest = Unit(Quaternion{rotation[0], rotation[1], rotation[2], rotation[3]}))
                        hand.fingers[i] = Hand::Finger{std::size_t(bone - captured.bones.begin()), *rest};
                }
            }
        }

        void CaptureBreathing(Capture& captured, std::size_t chestIndex)
        {
            const auto& chest = captured.bones[chestIndex];
            if (chest.rootWorld || chest.carriesTranslation || chest.aboveCom || chest.seatAnchor) return;
            const auto find = [&](std::string_view name) {
                return std::ranges::find(captured.bones, name, &FrozenBone::name);
            };
            const auto neck = find(kHead.front());
            if (neck == captured.bones.end() || !IsAncestor(chest.ref, neck->ref)) return;
            // Only animate a branch that leaves both feet in place.
            for (std::size_t i = 2; i < kLimbs.size(); ++i) {
                const auto foot = find(kLimbs[i].back());
                if (foot == captured.bones.end() || IsAncestor(chest.ref, foot->ref)) return;
            }
            const auto chestWorld = ReadTransform(chest.ref.World());
            const auto neckWorld = ReadTransform(neck->ref.World());
            if (!chestWorld || !neckWorld) return;
            const auto direction = Unit(neckWorld->position - chestWorld->position);
            if (!direction) return;
            const auto localAxis = Rotate(Fingers::Conjugate(chestWorld->rotation), *direction);
            if (localAxis) captured.breathing = Capture::Breathing{chestIndex, *localAxis, {}};
        }

        void CenterHeadHandles(Capture& captured)
        {
            const auto find = [&](std::string_view name) {
                return std::ranges::find_if(captured.handles, [&](const Handle& handle) {
                    return captured.bones[handle.bone].name == name;
                });
            };
            const auto neck = find(kHead[0]), head = find(kHead[1]);
            if (neck == captured.handles.end() || head == captured.handles.end()) return;
            const auto& neckBone = captured.bones[neck->bone];
            const auto& headBone = captured.bones[head->bone];
            const auto neckWorld = ReadTransform(neckBone.ref.World()), headWorld = ReadTransform(headBone.ref.World());
            if (!neckWorld || !headWorld) return;
            const auto separation = headWorld->position - neckWorld->position;
            const auto axis = Unit(separation);
            if (!axis) return;
            const auto toLocal = [](Quaternion orientation, Vec3 direction) {
                return Rotate({-orientation.x, -orientation.y, -orientation.z, orientation.w}, direction);
            };
            auto neckAxis = toLocal(neckWorld->rotation, *axis);
            auto headAxis = toLocal(headWorld->rotation, *axis);
            // The bind pose preserves the head's anatomical axis even if the captured head is tilted.
            const auto* skeleton = captured.skeleton;
            if (skeleton && headBone.index < static_cast<std::uint32_t>(skeleton->parentIndices.size()) &&
                headBone.index < static_cast<std::uint32_t>(skeleton->referencePose.size()) &&
                skeleton->parentIndices[headBone.index] == static_cast<std::int32_t>(neckBone.index)) {
                const auto& rest = skeleton->referencePose[headBone.index];
                std::array<float, 4> offset, rotation;
                _mm_storeu_ps(offset.data(), rest.translation.quad);
                _mm_storeu_ps(rotation.data(), rest.rotation.vec.quad);
                const auto restAxis = Unit(Vec3{offset[0], offset[1], offset[2]});
                const auto restRotation = Unit(Quaternion{rotation[0], rotation[1], rotation[2], rotation[3]});
                if (restAxis && restRotation) {
                    neckAxis = restAxis;
                    headAxis = toLocal(*restRotation, *restAxis);
                }
            }
            if (neckAxis) neck->lever = *neckAxis * (Length(separation) * 0.5);
            if (headAxis) head->lever = *headAxis * 6;
        }

        FingerPanel::Edit ReadFingers(Fingers::Side side)
        {
            const auto& hand = pose.hands[static_cast<std::size_t>(side)];
            FingerPanel::Edit current{};
            for (std::size_t i = 0; i < hand.fingers.size(); ++i) {
                const auto& joint = hand.fingers[i];
                if (!joint) continue;
                if (const auto local = ReadTransform(&pose.bones[joint->bone].local)) {
                    current.pose[i] = Fingers::FromLocal(joint->rest, local->rotation, side);
                    current.changed |= Fingers::Joint(i);
                }
            }
            return current;
        }

        void WriteFingers(Fingers::Side side, const Fingers::Pose& fingers, Fingers::Mask mask)
        {
            const auto& hand = pose.hands[static_cast<std::size_t>(side)];
            for (std::size_t i = 0; i < hand.fingers.size(); ++i) {
                if (const auto& joint = hand.fingers[i]; joint && (mask & Fingers::Joint(i)))
                    WriteRotation(pose.bones[joint->bone].local, Fingers::ToLocal(joint->rest, fingers[i], side));
            }
        }

        Fingers::Side OtherHand(Fingers::Side side)
        {
            return side == Fingers::Side::Left ? Fingers::Side::Right : Fingers::Side::Left;
        }

        bool OpenFingerPanel(std::size_t selected, bool mirror)
        {
            for (std::size_t handIndex = 0; handIndex < pose.hands.size(); ++handIndex) {
                const auto& hand = pose.hands[handIndex];
                if (hand.wrist != pose.handles[selected].bone) continue;
                const auto side = static_cast<Fingers::Side>(handIndex);
                const auto current = ReadFingers(side);
                if (!current.changed) {
                    Notify("GModPose: this hand has no compatible finger bones");
                    return false;
                }
                FingerPanel::Begin(side, current.pose, current.changed,
                    current.changed & ReadFingers(OtherHand(side)).changed, mirror);
                pose.editingHand = side;
                PoseSettings::ResetMenu();
                fingerOpen.store(true);
                return true;
            }
            return false;
        }

        FacePose::Mask ApplyPendingEdits(bool finish)
        {
            if (const auto side = pose.editingHand) {
                for (const auto& edit : FingerPanel::TakeEdits(finish)) {
                    WriteFingers(*side, edit.pose, edit.changed);
                    if (edit.mirrored) WriteFingers(OtherHand(*side), edit.pose, edit.mirrored);
                    if (edit.checkpoint) CommitHistory();
                }
            }
            FacePose::Mask touched = 0;
            for (const auto& change : FacePanel::TakeEdits(finish)) {
                touched |= RestoreFace(change.edit);
                if (change.checkpoint) CommitHistory();
            }
            return touched;
        }

        void ShowHistoryPose()
        {
            if (pose.editingHand) FingerPanel::ShowPose(ReadFingers(*pose.editingHand).pose);
            if (const auto& face = pose.historyFace) FacePanel::ShowPose(face->pose, face->expressions);
        }

        void AddTwistHandles(Capture& next)
        {
            for (std::size_t segment = 0; segment < kTwistNames.size(); ++segment) {
                for (std::size_t side = 0; side < kTwistNames[segment].size(); ++side) {
                    const auto& limb = next.limbs[side];
                    if (!limb) continue;
                    const auto parent = next.bones[limb->joints[segment]].ref;
                    const auto endpoint = next.bones[limb->joints[2]].ref;
                    const auto& names = kTwistNames[segment][side];
                    for (std::size_t joint = 0; joint < names.size(); ++joint) {
                        const auto bone = std::ranges::find(next.bones, names[joint], &FrozenBone::name);
                        // The twist helpers must be off the chain that positions the hand.
                        if (bone == next.bones.end() || !IsAncestor(parent, bone->ref) || IsAncestor(bone->ref, endpoint))
                            continue;
                        next.handles.push_back({static_cast<std::size_t>(bone - next.bones.begin()), HandleAction::Twist,
                            kTwistLabels[segment][side][joint], side, false, {}, segment});
                    }
                }
            }
        }

        void AddWeaponHandles(Capture& next, const RE::Actor& actor)
        {
            for (std::size_t side = 0; side < kWeaponAttachments.size(); ++side) {
                const auto bone = std::ranges::find(next.bones, kWeaponAttachments[side], &FrozenBone::name);
                if (bone == next.bones.end()) continue;
                const auto* weapon = VisibleWeapon(actor, bone->ref);
                const auto frame = ReadTransform(bone->ref.World());
                if (!weapon || !frame) continue;
                // Put the marker on the mesh instead of stacking it over the wrist.
                const auto& center = weapon->worldBound.center;
                const auto offset = Rotate(Fingers::Conjugate(frame->rotation),
                    Vec3{center.x, center.y, center.z} - frame->position);
                const auto lever = offset && Length(*offset) > 5 ? *offset : Vec3{9, 0, 0};
                next.handles.push_back({static_cast<std::size_t>(bone - next.bones.begin()), HandleAction::Weapon,
                    kWeaponLabels[side], side, false, lever});
            }
        }

        void AddTailHandles(Capture& next, RE::NiAVObject& root)
        {
            const auto& biped = next.actor->GetBiped(false);
            const auto* tail = biped ? &biped->objects[RE::BIPED_OBJECTS::kTail] : nullptr;
            const bool equippedTail = tail && tail->partClone && !tail->partClone->GetAppCulled();

            // TailProject animates a separate chain on the same scene skeleton as the pelvis.
            // Use the captured pelvis to identify that tree among attached skeletal objects.
            const auto pelvis = std::ranges::find(next.bones, kCenter[1], &FrozenBone::name);
            const auto findSlot = [](RE::BSFlattenedBoneTree& tree, const RE::BSFixedString& name) {
                const auto& map = tree.GetRuntimeData().boneMap;
                const auto found = map.find(name);
                if (found == map.end() || found->second < 0) return BoneRef{};
                const BoneRef slot{&tree, found->second};
                return slot.Slot() ? slot : BoneRef{};
            };
            using Visit = RE::BSVisit::BSVisitControl;
            RE::BSFlattenedBoneTree* skeleton = nullptr;
            if (pelvis != next.bones.end()) {
                const RE::BSFixedString pelvisName{kCenter[1].data()};
                RE::BSVisit::TraverseScenegraphObjects(&root, [&](RE::NiAVObject* object) {
                    auto* tree = netimmerse_cast<RE::BSFlattenedBoneTree*>(object);
                    if (!tree) return Visit::kContinue;
                    const auto slot = findSlot(*tree, pelvisName);
                    if (slot != pelvis->ref && slot.Resolve() != pelvis->ref) return Visit::kContinue;
                    skeleton = tree;
                    return Visit::kStop;
                });
            }
            if (!skeleton) {
                if (equippedTail) SKSE::log::warn("Tail posing: could not identify the player scene skeleton by its pelvis");
                return;
            }

            std::array<BoneRef, kTail.size()> slots{};
            for (std::size_t i = 0; i < kTail.size(); ++i)
                slots[i] = findSlot(*skeleton, RE::BSFixedString{kTail[i].data()});
            if (std::ranges::none_of(slots, [](BoneRef slot) { return slot.node; })) {
                if (equippedTail) SKSE::log::warn("Tail posing: no TailBone slots on the player scene skeleton");
                return;
            }

            // Universal rigs expose tail slots even on tailless races. A custom tail
            // can be skinned into the body mesh without using the biped tail slot.
            const auto checkSkin = [&](RE::BSGeometry* geometry) {
                const auto& skin = geometry->GetGeometryRuntimeData().skinInstance;
                if (geometry->GetAppCulled() || !skin || !skin->skinData) return Visit::kContinue;
                const auto count = skin->skinData->GetBoneCount();
                if (count > 4096) return Visit::kContinue;
                for (std::size_t i = 0; i < count; ++i) {
                    const auto* bone = skin->bones ? skin->bones[i] : nullptr;
                    const auto* world = skin->boneWorldTransforms ? skin->boneWorldTransforms[i] : nullptr;
                    if (std::ranges::any_of(slots, [&](BoneRef slot) {
                        if (const auto* entry = slot.Slot())
                            return (bone && bone == entry->node) ||
                                (world && (world == &entry->world || world == slot.Resolve().World()));
                        return false;
                    })) return Visit::kStop;
                }
                return Visit::kContinue;
            };
            if (!equippedTail && RE::BSVisit::TraverseScenegraphGeometries(&root, checkSkin) != Visit::kStop)
                return;

            std::array<std::optional<std::size_t>, kTail.size()> joints{};
            for (std::size_t i = 0; i < joints.size(); ++i) {
                const auto ref = slots[i].Resolve();
                if (!ReadTransform(ref.Local()) || !ReadTransform(ref.World())) continue;
                const auto [existing, inserted] = next.byWorld.try_emplace(ref.World(), next.bones.size());
                joints[i] = existing->second;
                if (!inserted) continue;
                next.sceneTail[i].reset(ref.node);
                next.bones.push_back({kSceneBoneIndex, ref, *ref.Local(), {}, kTail[i]});
                next.bones.back().tailSlot = slots[i];
            }
            if (next.bones.size() > next.graphBoneCount) next.tailTree.reset(skeleton);
            SKSE::log::info("Tail posing: {} scene skeleton joints, equipped tail={}",
                std::ranges::count_if(joints, [](auto joint) { return joint.has_value(); }), equippedTail);

            // The named joints must be on one branch. Rig helpers between them
            // stay fixed relative to the preceding controlled joint.
            for (const auto joint : joints) {
                if (!joint) break;
                if (!next.tailJoints.empty()) {
                    const auto& preceding = next.bones[next.tailJoints.back()];
                    const auto& current = next.bones[*joint];
                    if (!IsAncestor(preceding.ref, current.ref.Parent())) {
                        SKSE::log::warn("Tail posing: {} does not descend from {}", current.name, preceding.name);
                        break;
                    }
                }
                next.tailJoints.push_back(*joint);
            }
            if (next.tailJoints.size() < 2) return;
            for (std::size_t i = 0; i < next.tailJoints.size(); ++i)
                next.handles.push_back({next.tailJoints[i], HandleAction::Tail,
                    i + 1 == next.tailJoints.size() ? "Tail tip" : kTailLabels[i]});
            SKSE::log::info("Tail posing: registered {} IK handles", next.tailJoints.size());
        }

        void CaptureReferenceControls(Capture& next)
        {
            if (!next.skeleton) return;
            const auto& skeleton = *next.skeleton;
            const auto count = std::min({skeleton.referencePose.size(), skeleton.parentIndices.size(), skeleton.bones.size()});
            if (count <= 0 || count > 4096) return;
            struct Reference { Transform local, world; Quaternion parent; };
            std::vector<std::optional<Reference>> references(count);
            const auto resolve = [&](auto&& self, std::size_t index, unsigned depth) -> std::optional<Reference> {
                if (index >= references.size() || depth >= 64) return std::nullopt;
                if (references[index]) return references[index];
                std::array<float, 4> translation, rotation;
                const auto& raw = skeleton.referencePose[static_cast<int>(index)];
                _mm_storeu_ps(translation.data(), raw.translation.quad);
                _mm_storeu_ps(rotation.data(), raw.rotation.vec.quad);
                const auto q = Unit(Quaternion{rotation[0], rotation[1], rotation[2], rotation[3]});
                const Vec3 position{translation[0], translation[1], translation[2]};
                if (!q || !IsFinite(position)) return std::nullopt;
                const Transform local{position, *q};
                const auto parent = skeleton.parentIndices[static_cast<int>(index)];
                if (parent < 0) return references[index] = Reference{local, local, {}};
                const auto ancestor = self(self, static_cast<std::size_t>(parent), depth + 1);
                const auto world = ancestor ? Compose(ancestor->world, local) : std::nullopt;
                if (!world) return std::nullopt;
                return references[index] = Reference{local, *world, ancestor->world.rotation};
            };
            const auto boneIndex = [&](std::string_view name) -> std::optional<std::size_t> {
                const auto bone = std::ranges::find(next.bones, name, &FrozenBone::name);
                if (bone == next.bones.end()) return std::nullopt;
                return static_cast<std::size_t>(bone - next.bones.begin());
            };
            const auto reference = [&](std::size_t bone) -> std::optional<Reference> {
                const auto index = next.bones[bone].index;
                if (index >= references.size() || Name(skeleton.bones[index].name) != next.bones[bone].name)
                    return std::nullopt;
                return resolve(resolve, index, 0);
            };
            for (std::size_t i = 0; i < next.bones.size(); ++i) {
                if (const auto rest = reference(i))
                    next.bones[i].reference = FrozenBone::RigReference{rest->local, rest->world};
            }
            const auto left = boneIndex(kLimbs[2][0]);
            const auto right = boneIndex(kLimbs[3][0]);
            if (left && right) {
                const auto l = reference(*left), r = reference(*right);
                if (l && r) next.mirrorNormal = Unit(r->world.position - l->world.position).value_or(next.mirrorNormal);
            }
            const auto addPair = [&](std::size_t group, std::string_view leftName, std::string_view rightName) {
                const auto l = boneIndex(leftName), r = boneIndex(rightName);
                if (!l || !r) return;
                const auto a = reference(*l), b = reference(*r);
                if (a && b) next.mirrorPairs[group].push_back({std::array{
                    MirrorPair::Side{*l, {a->local.rotation, a->parent}},
                    MirrorPair::Side{*r, {b->local.rotation, b->parent}}}});
            };
            for (std::size_t joint = 0; joint < kLimbs[0].size(); ++joint) {
                addPair(0, kLimbs[0][joint], kLimbs[1][joint]);
                addPair(1, kLimbs[2][joint], kLimbs[3][joint]);
            }
            addPair(0, kClavicles[0], kClavicles[1]);
            addPair(1, kToes[0], kToes[1]);
            for (const auto& segment : kTwistNames)
                for (std::size_t joint = 0; joint < segment[0].size(); ++joint)
                    addPair(0, segment[0][joint], segment[1][joint]);

            const auto head = boneIndex(kHead.back());
            const auto headRef = head ? reference(*head) : std::nullopt;
            if (!headRef) return;
            const Vec3 vertical{0, 0, 1};
            const auto up = Unit(vertical - next.mirrorNormal * Dot(vertical, next.mirrorNormal));
            if (!up) return;
            const auto eye = boneIndex(kEye);
            auto forward = Cross(*up, next.mirrorNormal);
            if (const auto eyeRef = eye ? reference(*eye) : std::nullopt;
                eyeRef && Dot(eyeRef->world.position - headRef->world.position, forward) < 0)
                forward = forward * -1;
            const auto toLocal = Fingers::Conjugate(headRef->world.rotation);
            next.gaze = GazeRig{*head, eye, *Rotate(toLocal, next.mirrorNormal), *Rotate(toLocal, forward), *Rotate(toLocal, *up)};
        }

        void MirrorLimb(std::size_t bone)
        {
            for (std::size_t limbGroup = 0; limbGroup < pose.mirrorPairs.size(); ++limbGroup) {
                const auto& group = pose.mirrorPairs[limbGroup];
                for (std::size_t side = 0; side < 2; ++side) {
                    if (!std::ranges::any_of(group, [&](const MirrorPair& pair) { return pair.sides[side].bone == bone; }))
                        continue;
                    for (const auto& pair : group) {
                        const auto& source = pair.sides[side];
                        const auto& target = pair.sides[1 - side];
                        const auto& from = pose.bones[source.bone];
                        auto& to = pose.bones[target.bone];
                        if (const auto local = ReadTransform(&from.local)) {
                            WriteRotation(to.local, MirrorBoneRotation(local->rotation, source.reference,
                                target.reference, pose.mirrorNormal));
                            to.manualRotation = from.manualRotation;
                        }
                    }
                    if (limbGroup == 0) {
                        const auto source = static_cast<Fingers::Side>(side);
                        const auto fingers = ReadFingers(source);
                        WriteFingers(OtherHand(source), fingers.pose, fingers.changed);
                    }
                    return;
                }
            }
        }

        std::optional<FacePose::State> CameraGaze(const DragView::Camera& camera, WorldPose& worlds)
        {
            if (!pose.gaze) return std::nullopt;
            const auto& rig = *pose.gaze;
            const auto head = worlds.Read(pose.bones[rig.head].ref);
            if (!head) return std::nullopt;
            const auto eye = rig.eye ? worlds.Read(pose.bones[*rig.eye].ref) : std::nullopt;
            const auto right = Rotate(head->rotation, rig.right);
            const auto forward = Rotate(head->rotation, rig.forward);
            const auto up = Rotate(head->rotation, rig.up);
            if (!right || !forward || !up) return std::nullopt;
            return FacePose::AimGaze(camera.aim.origin - (eye ? eye->position : head->position), *right, *forward, *up);
        }

        bool AnchorSeat(Capture& capture)
        {
            // Keep the chair's pelvis and ancestors fixed while editing the limbs.
            const auto pelvis = std::ranges::find(capture.bones, kCenter[1], &FrozenBone::name);
            if (pelvis == capture.bones.end() && !capture.comIndex) return false;
            auto bone = pelvis != capture.bones.end() ? pelvis->ref : capture.bones[*capture.comIndex].ref;
            for (unsigned depth = 0; bone.node && depth < 64; ++depth, bone = bone.Parent()) {
                if (const auto found = capture.byWorld.find(bone.World()); found != capture.byWorld.end())
                    capture.bones[found->second].seatAnchor = true;
            }
            return true;
        }

        bool PublishCapture(Capture next, std::uint64_t stamp)
        {
            std::lock_guard lock(poseMutex);
            if (loading.load() || generation.load() != stamp) return false;
            pose = std::move(next);
            activeTailTree.store(pose.tailTree.get(), std::memory_order_release);
            active.store(true, std::memory_order_release);
            SKSE::log::info("Captured {} bones, {} body handles and {} IK chains", pose.bones.size(),
                pose.handles.size(), std::ranges::count_if(pose.limbs, [](const auto& limb) { return limb.has_value(); }));
            SKSE::log::info("Pose translation carrier: {}", pose.comIndex ? "NPC COM" : "outer skeleton roots");
            SKSE::log::info("Arm twist handles mapped: {}",
                std::ranges::count(pose.handles, HandleAction::Twist, &Handle::action));
            SKSE::log::debug("Mirror bone pairs mapped: arms={} legs={}; camera gaze={}",
                pose.mirrorPairs[0].size(), pose.mirrorPairs[1].size(), pose.gaze.has_value());
            for (std::size_t i = 0; i < pose.limbs.size(); ++i) {
                const auto& limb = pose.limbs[i];
                if (!limb) continue;
                const auto& proximal = limb->twists[0];
                const auto& distal = limb->twists[1];
                SKSE::log::debug("Automatic twist helpers {}: proximal={} distal={}", kLimbLabels[i][2],
                    proximal ? proximal->bones.size() : 0, distal ? distal->bones.size() : 0);
            }
            const auto countJoints = [](const Hand& hand) {
                return std::ranges::count_if(hand.fingers, [](const auto& bone) { return bone.has_value(); });
            };
            SKSE::log::debug("Finger joints mapped: left={} right={}", countJoints(pose.hands[0]), countJoints(pose.hands[1]));
            return true;
        }

        bool CapturePose(RE::PlayerCharacter& player, bool seated, Capture* waiting = nullptr)
        {
            const auto stamp = generation.load();
            const bool captured = WithGraph(player, [&](RE::BShkbAnimationGraph& graph) {
                const auto count = graph.boneNodes.size();
                if (count == 0 || count > 4096) {
                    SKSE::log::warn("Invalid graph bone count: {}", count);
                    return false;
                }
                Capture next;
                next.graph = &graph;
                next.skeleton = Skeleton(graph);
                next.actor = &player;
                next.cell = player.GetParentCell();
                next.root = graph.rootNode;
                next.count = count;
                next.seated = seated;
                next.historyOrigin = {Vec3{player.GetPosition().x, player.GetPosition().y, player.GetPosition().z},
                    AxisAngle({0, 0, 1}, player.GetAngleZ())};
                std::optional<std::size_t> comIndex;
                for (std::uint32_t i = 0; i < count; ++i) {
                    const auto bone = GraphBoneRef(graph.boneNodes[i]);
                    if (!bone.World() || !ReadTransform(bone.Local())) continue;
                    const auto [mapped, inserted] = next.byWorld.try_emplace(bone.World(), next.bones.size());
                    const auto animationName = next.skeleton && i < static_cast<std::uint32_t>(next.skeleton->bones.size()) ?
                        Name(next.skeleton->bones[i].name) : std::string_view{};
                    if (!comIndex && animationName == kCenter[0]) comIndex = mapped->second;
                    if (!inserted) continue;
                    auto name = bone.Name();
                    const auto landmark = [](std::string_view value) {
                        return FingerName(value) || IsTwistName(value) || std::ranges::find(kCenter, value) != kCenter.end() ||
                            std::ranges::find(kChest, value) != kChest.end() ||
                            value == kEye || std::ranges::find(kHead, value) != kHead.end() ||
                            std::ranges::find(kExtras, value, &ExtraHandle::bone) != kExtras.end() ||
                            std::ranges::find(kWeaponAttachments, value) != kWeaponAttachments.end() ||
                            std::ranges::any_of(kLimbs, [&](const JointNames& limb) {
                                return std::ranges::find(limb, value) != limb.end();
                            });
                    };
                    if (!landmark(name) && (name.empty() || landmark(animationName))) name = animationName;
                    next.bones.push_back({i, bone, *bone.Local(), {}, name});
                }
                if (next.bones.empty()) return false;
                next.graphBoneCount = next.bones.size();
                for (auto& frozen : next.bones) {
                    auto parent = frozen.ref.Parent();
                    if (!ReadTransform(frozen.ref.World())) continue;
                    for (unsigned depth = 0; parent.node && depth < 64; ++depth) {
                        if (next.byWorld.contains(parent.World())) break;
                        parent = parent.Parent();
                    }
                    if (!parent.node) frozen.rootWorld = *frozen.ref.World();
                }
                if (!comIndex) {
                    const auto com = std::ranges::find(next.bones, kCenter[0], &FrozenBone::name);
                    if (com != next.bones.end()) comIndex = static_cast<std::size_t>(com - next.bones.begin());
                }
                AssignTranslationCarriers(next, comIndex);
                next.comIndex = comIndex;
                if (seated && !AnchorSeat(next)) {
                    SKSE::log::warn("Could not map a pelvis or COM to anchor the seated pose");
                    return false;
                }
                next.rebaseForSharing = sharingEnabled && (!seated || waiting) && comIndex && !next.bones[*comIndex].rootWorld &&
                    REX::W32::GetModuleHandleW(L"PlayerHub.dll");
                if (waiting) {
                    if (!next.rebaseForSharing) return false;
                    // Hold the seat-space parent transforms through the exit.
                    // The live frames are used only to encode the offset at COM.
                    for (auto& bone : next.bones) {
                        if (!bone.carriesTranslation || bone.rootWorld) continue;
                        const auto* parentWorld = bone.ref.Parent().World();
                        if (!ReadTransform(parentWorld)) return false;
                        bone.seatParent = *parentWorld;
                    }
                }
                if (next.rebaseForSharing && refreshConfig.period.count() > 0)
                    next.nextSharingRefresh = std::chrono::steady_clock::now() + refreshConfig.period;
                const auto addHandle = [&](std::string_view name, HandleAction action, const char* label) {
                    const auto bone = std::ranges::find(next.bones, name, &FrozenBone::name);
                    if (bone == next.bones.end()) return false;
                    next.handles.push_back({static_cast<std::size_t>(bone - next.bones.begin()), action, label, 0,
                        name == kHead.back() || name == kLimbs[0].back() || name == kLimbs[1].back()});
                    return true;
                };
                // COM and pelvis represent one central handle. Prefer COM on
                // skeletons that provide both, matching the physics editor.
                if (!addHandle(kCenter[0], HandleAction::Torso, "Pelvis"))
                    addHandle(kCenter[1], HandleAction::Torso, "Pelvis");
                for (const auto name : kChest) {
                    if (!addHandle(name, HandleAction::Torso, "Chest")) continue;
                    next.handles.back().poser = true;
                    CaptureBreathing(next, next.handles.back().bone);
                    break;
                }
                addHandle(kHead[0], HandleAction::Rotate, "Neck");
                addHandle(kHead[1], HandleAction::Rotate, "Head");
                CenterHeadHandles(next);
                for (std::size_t i = 0; i < kLimbs.size(); ++i) {
                    next.limbs[i] = CaptureLimb(next, kLimbs[i]);
                    const auto& labels = kLimbLabels[i];
                    if (const auto& limb = next.limbs[i]) {
                        next.handles.push_back({limb->joints[0], HandleAction::Bend, labels[0], i});
                        next.handles.push_back({limb->joints[1], HandleAction::Bend, labels[1], i});
                        next.handles.push_back({limb->joints[2], HandleAction::Endpoint, labels[2], i, i < 2});
                    } else {
                        SKSE::log::warn("IK chain unavailable for {}", kLimbs[i].back());
                        for (std::size_t joint = 0; joint < labels.size(); ++joint)
                            addHandle(kLimbs[i][joint], HandleAction::Rotate, labels[joint]);
                    }
                }
                for (const auto& extra : kExtras) {
                    if (std::ranges::any_of(next.handles, [&](const Handle& handle) {
                        return next.bones[handle.bone].name == extra.bone;
                    })) continue;
                    if (!addHandle(extra.bone, HandleAction::RotationOnly, extra.label)) continue;
                    auto& handle = next.handles.back();
                    handle.lever = {};
                    const auto frame = ReadTransform(next.bones[handle.bone].ref.World());
                    if (!frame) continue;
                    auto offset = extra.markerOffset;
                    if (!extra.markerGuide.empty()) {
                        const auto guide = std::ranges::find(next.bones, extra.markerGuide, &FrozenBone::name);
                        if (guide != next.bones.end()) {
                            if (const auto shoulder = ReadTransform(guide->ref.World()))
                                offset = (shoulder->position - frame->position) * 0.55;
                        }
                    }
                    const auto q = frame->rotation;
                    handle.lever = Rotate({-q.x, -q.y, -q.z, q.w}, offset).value_or(Vec3{});
                }
                AddTwistHandles(next);
                AddWeaponHandles(next, player);
                AddTailHandles(next, *graph.rootNode);
                CaptureLimbGeometry(next, player.GetAngleZ());
                MapFingers(next);
                CaptureReferenceControls(next);
                if (waiting) {
                    *waiting = std::move(next);
                    return true;
                }
                return PublishCapture(std::move(next), stamp);
            });
            if (!captured) SKSE::log::warn("Could not capture a usable player animation graph");
            return captured;
        }
        bool Eligible(RE::PlayerCharacter* player)
        {
            auto* ui = RE::UI::GetSingleton();
            return !loading.load() && player && player->Is3DLoaded() && player->GetParentCell() &&
                !player->IsDead() && !player->IsOnMount() && !player->IsInKillMove() && !player->IsInCombat() &&
                !player->IsInRagdollState() && !player->AsActorState()->IsStaggered() &&
                player->AsActorState()->GetKnockState() == RE::KNOCK_STATE_ENUM::kNormal &&
                ui && !ui->IsMenuOpen("Loading Menu");
        }

        constexpr char kDrawnSharingStart[] = "GModPoseShareDrawnStart";
        constexpr char kDrawnSharingStop[] = "GModPoseShareDrawnStop";

        const RE::hkbBehaviorGraphStringData* GraphStrings(const RE::BShkbAnimationGraph& graph)
        {
            const auto* data = graph.behaviorGraph ? graph.behaviorGraph->data.get() : nullptr;
            return data ? data->stringData.get() : nullptr;
        }

        bool SharingAvailable(RE::PlayerCharacter& player)
        {
            if (!sharingEnabled || !REX::W32::GetModuleHandleW(L"PlayerHub.dll")) return false;
            const RE::BSResourceNiBinaryStream asset{
                R"(meshes\actors\character\animations\GModPoseShare\ShareStill.hkx)"};
            return asset.good() && WithGraph(player, [](const RE::BShkbAnimationGraph& graph) {
                const auto* strings = GraphStrings(graph);
                return strings && std::ranges::find(strings->variableNames, "GModPoseShareRevision", Name) !=
                    strings->variableNames.end();
            });
        }

        bool RequestSharedSeat(RE::PlayerCharacter& player, RE::ObjectRefHandle furniture)
        {
            if (!sharingEnabled || !detachSeatedPose) return false;
            const auto seat = furniture.get();
            const auto* form = seat ? seat->GetBaseObject() : nullptr;
            const auto* chair = form ? form->As<RE::TESFurniture>() : nullptr;
            auto* process = player.GetActorRuntimeData().currentProcess;
            if (!chair || chair->workBenchData.benchType.get() != RE::TESFurniture::WorkBenchData::BenchType::kNone ||
                !process || !process->middleHigh || !SharingAvailable(player)) return false;

            PendingSeat request{{}, furniture, process->middleHigh->currentFurnitureMarkerID,
                std::chrono::steady_clock::now() + std::chrono::seconds{8}, generation.load()};
            if (!CapturePose(player, true, &request.capture)) return false;
            {
                const std::scoped_lock lock(poseMutex);
                if (loading.load() || generation.load() != request.generation) return false;
                // Cover the handoff from native occupancy through pose publication.
                SeatBlock::Begin(*seat, request.markerID);
            }

            // Skyrim's get-up procedure selects the correct exit idle for this furniture.
            using StandUp = bool(RE::AIProcess*, RE::Actor*);
            static const REL::Relocation<StandUp> standUp{RELOCATION_ID(38835, 39865)};
            const bool accepted = standUp(process, &player);
            if (!accepted && player.AsActorState()->GetSitSleepState() != RE::SIT_SLEEP_STATE::kWantToStand) {
                SeatBlock::End();
                return false;
            }
            pendingSeat = std::move(request);
            SKSE::log::info("Captured the seated pose and requested a native furniture exit");
            return true;
        }

        std::optional<std::array<Transform, 3>> ReadLimbJoints(const Limb& limb, WorldPose& worlds)
        {
            std::array<Transform, 3> joints;
            for (std::size_t i = 0; i < limb.joints.size(); ++i) {
                const auto world = worlds.Read(pose.bones[limb.joints[i]].ref);
                if (!world) return std::nullopt;
                joints[i] = *world;
            }
            return joints;
        }

        std::optional<LimbPose> Sample(const Limb& limb, WorldPose& worlds)
        {
            const auto joints = ReadLimbJoints(limb, worlds);
            if (!joints) return std::nullopt;
            LimbPose sample;
            sample.world = *joints;
            sample.middle = limb.middle;
            sample.hinge = HingeAtGrip(sample.world, limb.hinge);
            const auto parent = worlds.Read(limb.path.front().Parent());
            if (!parent) return std::nullopt;
            sample.parentRotation = parent->rotation;
            for (const auto bone : limb.path) {
                const auto index = pose.byWorld.find(bone.World());
                const auto* local = index == pose.byWorld.end() ? bone.Local() : &pose.bones[index->second].local;
                const auto transform = ReadTransform(local);
                if (!transform) return std::nullopt;
                sample.localRotations.push_back(transform->rotation);
            }
            return sample;
        }

        void ApplyLimb(const Limb& limb, const LimbEdit& edit)
        {
            for (std::size_t i = 0; i < limb.joints.size(); ++i)
                WriteRotation(pose.bones[limb.joints[i]].local, edit.localRotations[i]);
        }

        std::optional<TorsoClearance> ReadTorsoClearance(WorldPose& worlds,
                                                        const std::array<Vec3, 2>* referenceShoulders = nullptr)
        {
            std::array<Vec3, kLimbs.size()> roots;
            for (std::size_t i = 0; i < roots.size(); ++i) {
                if (referenceShoulders && i < referenceShoulders->size()) {
                    roots[i] = (*referenceShoulders)[i];
                    continue;
                }
                const auto& limb = pose.limbs[i];
                if (!limb) return std::nullopt;
                const auto root = worlds.Read(pose.bones[limb->joints[0]].ref);
                if (!root) return std::nullopt;
                roots[i] = root->position;
            }
            auto clearance = MakeTorsoClearance((roots[2] + roots[3]) * .5, (roots[0] + roots[1]) * .5, roots[0], roots[1]);
            const auto neck = std::ranges::find(pose.bones, kHead.front(), &FrozenBone::name);
            const auto head = std::ranges::find(pose.bones, kHead.back(), &FrozenBone::name);
            if (clearance && neck != pose.bones.end() && head != pose.bones.end()) {
                const auto base = worlds.Read(neck->ref), skull = worlds.Read(head->ref);
                if (base && skull) {
                    const auto shaft = skull->position - base->position;
                    const auto radius = Length(shaft) * .85;
                    if (radius > .01)
                        clearance->head = TorsoClearance::Head{skull->position + shaft * .5, radius};
                }
            }
            return clearance;
        }

        std::optional<EndpointGrip> CaptureEndpointGrip(std::size_t side, WorldPose& worlds,
                                                         std::optional<Transform> oppositeAnchor = {})
        {
            const auto& limb = *pose.limbs[side];
            const bool leg = side >= 2;
            const auto chest = std::ranges::find(pose.bones, kChest.front(), &FrozenBone::name);
            const auto pelvis = std::ranges::find(pose.bones, kCenter[1], &FrozenBone::name);
            if (chest == pose.bones.end() || pelvis == pose.bones.end()) return std::nullopt;
            const auto& basis = leg ? *pelvis : *chest;
            if (!basis.reference) return std::nullopt;
            const auto scaledBasis = worlds.Get(basis.ref);
            const auto bodyFrame = scaledBasis ? ReadTransform(&*scaledBasis) : std::nullopt;
            const auto torso = worlds.Read(leg ? chest->ref : pelvis->ref);
            std::optional<Transform> other;
            if (leg) {
                if (oppositeAnchor) other = oppositeAnchor;
                else if (const auto& opposite = pose.limbs[side ^ 1])
                    other = worlds.Read(pose.bones[opposite->joints.back()].ref);
            } else {
                const auto head = std::ranges::find(pose.bones, kHead.back(), &FrozenBone::name);
                if (head != pose.bones.end()) other = worlds.Read(head->ref);
            }
            if (!bodyFrame || !torso || !other)
                return std::nullopt;

            EndpointGrip grip{};
            auto& rig = grip.rig;
            rig.frame = *bodyFrame;
            rig.context = {torso->position, other->position};
            rig.left = side % 2 == 0;
            rig.leg = leg;
            for (std::size_t i = 0; i < rig.joints.size(); ++i) {
                const auto& rest = pose.bones[limb.joints[i]].reference;
                const auto relative = rest ? RelativeTo(basis.reference->world, rest->world) : std::nullopt;
                if (!relative) return std::nullopt;
                rig.frameBindRotations[i] = relative->rotation;
            }
            if (!leg) {
                std::array<Vec3, 2> shoulderRoots;
                for (std::size_t i = 0; i < shoulderRoots.size(); ++i) {
                    if (!pose.limbs[i]) return std::nullopt;
                    const auto& rest = pose.bones[pose.limbs[i]->joints[0]].reference;
                    const auto relative = rest ? RelativeTo(basis.reference->world, rest->world) : std::nullopt;
                    if (!relative) return std::nullopt;
                    shoulderRoots[i] = bodyFrame->position +
                        *Rotate(bodyFrame->rotation, relative->position * scaledBasis->scale);
                }
                rig.clearance = ReadTorsoClearance(worlds, &shoulderRoots);
            }

            std::vector<BoneRef> chain;
            if (limb.clavicle) {
                const auto& support = pose.bones[limb.clavicle->bone];
                for (auto ref = limb.path.front().Parent(); ref.node && chain.size() < 32; ref = ref.Parent()) {
                    chain.push_back(ref);
                    if (ref == support.ref) break;
                }
                if (chain.empty() || chain.back() != support.ref) return std::nullopt;
                std::ranges::reverse(chain);
            }
            chain.insert(chain.end(), limb.path.begin(), limb.path.end());
            auto parent = worlds.Get(chain.front().Parent());
            const auto firstParent = parent ? ReadTransform(&*parent) : std::nullopt;
            if (!firstParent) return std::nullopt;
            grip.parent = firstParent->rotation;
            for (const auto ref : chain) {
                const auto index = pose.byWorld.find(ref.World());
                const auto* bone = index == pose.byWorld.end() ? nullptr : &pose.bones[index->second];
                const auto* local = bone ? &bone->local : ref.Local();
                if (!local || (bone && (bone->rootWorld || bone->carriesTranslation || bone->seatParent)))
                    return std::nullopt;
                auto bind = *local;
                if (bone && bone->reference) {
                    WriteRotation(bind, bone->reference->local.rotation);
                } else if (bone || !ref.Name().starts_with("CME ")) {
                    return std::nullopt;
                }
                // A non-animated RaceMenu node contributes its rig adjustment.
                // Animated links always use the rig reference rotation.
                const auto fixed = ReadTransform(&bind);
                if (!fixed) return std::nullopt;
                grip.links.push_back({bone ? std::optional{index->second} : std::nullopt, fixed->rotation});
                if (ref == limb.path.front()) rig.shoulderParent = *ReadTransform(&*parent);
                *parent = *parent * bind;
                const auto world = ReadTransform(&*parent);
                if (!world) return std::nullopt;
                if (bone && limb.clavicle && index->second == limb.clavicle->bone) rig.clavicle = *world;
                for (std::size_t i = 0; i < rig.joints.size(); ++i)
                    if (bone && index->second == limb.joints[i]) rig.joints[i] = *world;
            }
            if (rig.Reach() < .01) return std::nullopt;

            for (std::size_t i = 0; i < limb.twists.size(); ++i) {
                if (!limb.twists[i]) continue;
                const auto& start = pose.bones[limb.joints[i]].reference->world;
                const auto shaft = pose.bones[limb.joints[i + 1]].reference->world.position - start.position;
                const auto length = Dot(shaft, shaft);
                if (length < 1e-5) continue;
                for (const auto& helper : limb.twists[i]->bones) {
                    const auto& rest = pose.bones[helper.bone].reference;
                    const auto relative = rest ? RelativeTo(start, rest->world) : std::nullopt;
                    if (!relative) return std::nullopt;
                    const double fraction = std::clamp(Dot(rest->world.position - start.position, shaft) / length, 0.0, 1.0);
                    grip.twists[i].push_back({helper.bone, relative->rotation, fraction});
                }
            }
            return grip;
        }

        std::optional<Vec3> Marker(std::size_t index, WorldPose& worlds)
        {
            const auto handle = pose.handles[index];
            const auto bone = pose.bones[handle.bone].ref;
            if (handle.action == HandleAction::Weapon && !VisibleWeapon(*pose.actor, bone)) return std::nullopt;
            const auto current = worlds.Read(bone);
            if (!current) return std::nullopt;
            if (handle.action == HandleAction::Bend) {
                const auto& limb = *pose.limbs[handle.limb];
                if (handle.bone == limb.joints[0]) {
                    if (const auto middle = worlds.Read(pose.bones[limb.joints[1]].ref))
                        return (current->position + middle->position) * 0.5;
                }
            }
            if (handle.action == HandleAction::Tail && handle.bone == pose.tailJoints.front()) {
                if (const auto next = worlds.Read(pose.bones[pose.tailJoints[1]].ref))
                    return (current->position + next->position) * .5;
            }
            if (handle.action == HandleAction::Rotate || handle.action == HandleAction::RotationOnly ||
                handle.action == HandleAction::Weapon) {
                const auto lever = Rotate(current->rotation, handle.lever);
                if (lever) return current->position + *lever;
            }
            return current->position;
        }

        void UpdateGripTwists(const Interaction::Grip& grip, WorldPose& worlds, bool begin);

        bool HasFootSupport(const Limb& limb, std::size_t side, Transform ankle, WorldPose& worlds)
        {
            auto* world = pose.cell ? pose.cell->GetbhkWorld() : nullptr;
            const auto& bone = pose.bones[limb.joints.back()];
            const auto foot = worlds.Get(bone.ref);
            const auto scale = RE::bhkWorld::GetWorldScale();
            if (!world || !foot || !bone.reference || !std::isfinite(scale) || scale <= 0) return true;

            struct SupportHit final : RE::hkpClosestRayHitCollector
            {
                void AddRayHit(const RE::hkpCdBody& body, const RE::hkpShapeRayCastCollectorOutput& hit) override
                {
                    const auto* root = &body;
                    while (root->parent) root = root->parent;
                    const auto* collidable = static_cast<const RE::hkpCollidable*>(root);
                    using Layer = RE::COL_LAYER;
                    const auto layer = collidable->GetCollisionLayer();
                    if (layer != Layer::kStatic && layer != Layer::kAnimStatic && layer != Layer::kTerrain &&
                        layer != Layer::kGround && layer != Layer::kStairHelper && layer != Layer::kProps &&
                        layer != Layer::kClutter) return;
                    std::array<float, 4> normal;
                    _mm_storeu_ps(normal.data(), hit.normal.quad);
                    if (normal[2] < .55F || hit.hitFraction >= earlyOutHitFraction) return;
                    rayHit.normal = hit.normal;
                    rayHit.hitFraction = hit.hitFraction;
                    rayHit.rootCollidable = collidable;
                    earlyOutHitFraction = hit.hitFraction;
                }
            };
            const auto touches = [&](Vec3 sole) {
                RE::bhkPickData pick{};
                const auto point = [&](double height) {
                    return RE::hkVector4(static_cast<float>(sole.x) * scale, static_cast<float>(sole.y) * scale,
                        static_cast<float>(sole.z + height * foot->scale) * scale, 0.0F);
                };
                pick.rayInput.from = point(4);
                pick.rayInput.to = point(-3);
                pick.rayInput.filterInfo.SetCollisionLayer(RE::COL_LAYER::kCharController);
                SupportHit hit;
                pick.closestRayHitCollector = &hit;
                world->PickObject(pick);
                return hit.HasHit();
            };
            const auto toe = std::ranges::find(pose.bones, kToes[side % 2], &FrozenBone::name);
            const double heelDepth = toe != pose.bones.end() && toe->reference ?
                std::max(0.0, bone.reference->world.position.z - toe->reference->world.position.z) : 6.0;
            // The ankle pivot is above the sole. Rotate the bind-pose heel
            // offset with the current foot so a raised heel can leave contact.
            const auto bindHeel = Rotate(Fingers::Conjugate(bone.reference->world.rotation), {0, 0, -heelDepth});
            if (bindHeel) {
                if (const auto heel = Rotate(ankle.rotation, *bindHeel * foot->scale); heel && touches(ankle.position + *heel))
                    return true;
            }
            const auto toeFrame = toe != pose.bones.end() ? worlds.Read(toe->ref) : std::nullopt;
            return toeFrame && touches(toeFrame->position);
        }

        void BeginGrip(std::size_t index, Vec3 point, bool rotate, const DragView::Camera& camera, WorldPose& worlds)
        {
            const auto handle = pose.handles[index];
            const auto& heldBone = pose.bones[handle.bone];
            const auto bone = worlds.Read(heldBone.ref);
            const auto distance = editor.nudge ? std::optional{HoldDistance{}} :
                HoldDistance::Capture(camera.aim, camera.right, camera.up, point);
            if (!bone || !distance) return;
            Interaction::Grip grip{index, rotate, *distance, point, point, pose.translation, {}, *bone, {}, {}, {}, {}};
            if (handle.action == HandleAction::Tail) {
                Interaction::Grip::Tail tail{};
                for (std::size_t i = 0; i < pose.tailJoints.size(); ++i) {
                    const auto joint = pose.tailJoints[i];
                    const auto ref = pose.bones[joint].ref;
                    const auto world = worlds.Read(ref);
                    if (!world) return;
                    tail.joints[i] = *world;
                    if (i > 0) {
                        const auto parent = worlds.Read(ref.Parent());
                        if (!parent) return;
                        tail.parentOffsets[i] = Multiply(Fingers::Conjugate(tail.joints[i - 1].rotation), parent->rotation);
                    }
                    if (joint == handle.bone) tail.held = i;
                }
                if (tail.held + 1 == pose.tailJoints.size()) {
                    // Removing the joint rotation leaves the rig's offset,
                    // including any unchanged helper nodes between controls.
                    for (std::size_t i = 0; i < tail.held; ++i) {
                        const auto offset = tail.joints[i + 1].position - tail.joints[i].position;
                        tail.lengths[i] = Length(offset);
                        const auto axis = Unit(offset);
                        if (!axis) return;
                        tail.linkAxes[i] = *Rotate(Fingers::Conjugate(tail.joints[i].rotation), *axis);
                    }
                }
                grip.tail = tail;
            } else if (handle.action == HandleAction::Bend || handle.action == HandleAction::Endpoint) {
                const auto& limb = *pose.limbs[handle.limb];
                const bool moveEndpoint = !rotate && handle.action == HandleAction::Endpoint;
                if (moveEndpoint) {
                    grip.endpoint = CaptureEndpointGrip(handle.limb, worlds);
                    if (!grip.endpoint) SKSE::log::warn("Could not map reference chain for {}", heldBone.name);
                    // Carry the authored hand or foot rotation relative to the forearm or calf.
                    if (const auto middle = worlds.Read(pose.bones[limb.joints[1]].ref))
                        grip.endpointFlex = Multiply(Fingers::Conjugate(middle->rotation), bone->rotation);
                }
                if (!grip.endpoint) {
                    grip.limb = Sample(limb, worlds);
                    if (!grip.limb) return;
                    grip.clearance = ReadTorsoClearance(worlds);
                }
                if (!grip.endpoint && !rotate && limb.clavicle) {
                    const auto& bone = pose.bones[limb.clavicle->bone];
                    if (const auto frame = worlds.Read(bone.ref)) {
                        if (!bone.manualRotation || moveEndpoint) grip.clavicle = Interaction::Grip::Support{*frame, {}};
                        else if (const auto parent = worlds.Read(bone.ref.Parent()))
                            grip.clavicle = Interaction::Grip::Support{*frame,
                                Multiply(parent->rotation, limb.clavicle->capturedLocal)};
                    }
                }
            } else if (handle.action != HandleAction::Twist && handle.action != HandleAction::Weapon &&
                (rotate || !IsTriangle(handle.action))) {
                const bool torso = handle.action == HandleAction::Torso;
                grip.learnedStance = torso && std::ranges::find(kChest, heldBone.name) != kChest.end();
                const bool carriesUpperBody = torso || heldBone.name == kChest.back();
                for (std::size_t i = 0; i < pose.limbs.size(); ++i) {
                    // Hands and head inherit torso motion through the skeleton.
                    if (!pose.limbs[i] || (carriesUpperBody && i < 2)) continue;
                    const auto endpoint = pose.bones[pose.limbs[i]->joints[2]].ref;
                    if (torso || IsAncestor(heldBone.ref, endpoint)) {
                        const auto anchor = worlds.Read(endpoint);
                        if (!anchor) continue;
                        const bool torsoEdit = torso || carriesUpperBody || heldBone.name == kCenter[1];
                        grip.supportChecked[i] = i >= 2 && torsoEdit;
                        if (!grip.supportChecked[i] || HasFootSupport(*pose.limbs[i], i, *anchor, worlds))
                            grip.anchors[i] = anchor;
                    }
                }
                if (heldBone.name == kHead.back()) {
                    const auto neck = std::ranges::find(pose.bones, kHead.front(), &FrozenBone::name);
                    if (neck != pose.bones.end() && IsAncestor(neck->ref, heldBone.ref)) {
                        if (const auto frame = worlds.Read(neck->ref))
                            grip.neck = Interaction::Grip::Neck{static_cast<std::size_t>(neck - pose.bones.begin()), frame->rotation};
                    }
                }
            }
            grip.viewForward = camera.aim.direction;
            UpdateGripTwists(grip, worlds, true);
            editor.grip = std::move(grip);
        }

        bool WriteWorldRotation(FrozenBone& bone, Quaternion rotation, WorldPose& worlds)
        {
            if (bone.rootWorld) WriteRotation(*bone.rootWorld, rotation);
            else if (const auto parent = worlds.Read(bone.ref.Parent())) {
                const auto q = parent->rotation;
                WriteRotation(bone.local, Multiply({-q.x, -q.y, -q.z, q.w}, rotation));
            } else return false;
            return true;
        }

        bool ApplyTailEndpoint(const Interaction::Grip::Tail& grip, Vec3 target, std::optional<Quaternion> turn)
        {
            const auto tip = grip.held;
            WorldPose worlds;
            const auto root = worlds.Read(pose.bones[pose.tailJoints.front()].ref.Parent());
            if (!root) return true;
            const auto facing = AxisAngle({0, 0, 1}, pose.actor->GetAngleZ());
            const auto bend = Rotate(facing, {0, -1, -.8}).value_or(Vec3{0, -1, -.8});
            std::optional<Vec3> tangent;
            if (turn) {
                const auto neutralTipAxis = Rotate(Fingers::Conjugate(grip.parentOffsets[tip]), grip.linkAxes[tip - 1]);
                if (neutralTipAxis) tangent = Rotate(*turn, *neutralTipAxis);
            }
            const auto solution = SolveTailEndpoint(grip.joints[0].position, std::span(grip.lengths).first(tip),
                target, bend, nullptr, tangent);
            if (!solution) return true;

            // Parallel-transport the parent frame along the solved directions.
            // None of the previously posed tail rotations are used as a seed.
            auto previous = root->rotation;
            std::array<Quaternion, tailJointCount> locals{};
            for (std::size_t i = 0; i < tip; ++i) {
                const auto parent = Multiply(previous, grip.parentOffsets[i]);
                const auto neutralAxis = Rotate(parent, grip.linkAxes[i]);
                const auto link = solution->joints[i + 1] - solution->joints[i];
                const auto swing = neutralAxis ? FromTo(*neutralAxis, link) : std::nullopt;
                if (!swing) return true;
                const auto world = Multiply(*swing, parent);
                locals[i] = TailLocalRotation(world, previous, grip.parentOffsets[i]);
                previous = world;
            }
            if (turn) locals[tip] = TailLocalRotation(*turn, previous, grip.parentOffsets[tip]);
            for (std::size_t i = 0; i <= tip; ++i)
                WriteRotation(pose.bones[pose.tailJoints[i]].local, locals[i]);
            return solution->limited;
        }

        bool ApplyTailGrip(const Interaction::Grip::Tail& grip, Vec3 delta, std::optional<Quaternion> turn)
        {
            const auto tip = pose.tailJoints.size() - 1;
            if (grip.held == tip) return ApplyTailEndpoint(grip, grip.joints[tip].position + delta, turn);
            std::array<Quaternion, tailJointCount> world{};
            for (std::size_t i = 0; i <= tip; ++i) world[i] = grip.joints[i].rotation;
            bool limited = false;
            if (turn) {
                const auto axis = Unit(grip.joints[grip.held + 1].position - grip.joints[grip.held].position);
                if (!axis) return true;
                // Roll around the outgoing link. Keeping all other world
                // frames fixed cancels the roll at the next joint.
                world[grip.held] = TailTwist(world[grip.held], *turn, *axis);
            } else {
                if (!grip.held && tip == 1) return true;
                std::array<Vec3, tailJointCount> positions{};
                for (std::size_t i = 0; i <= tip; ++i) positions[i] = grip.joints[i].position;
                const auto held = std::max(grip.held, std::size_t{1});
                const auto solution = MoveTail(std::span(positions).first(tip + 1), held,
                    positions[held] + delta * (grip.held ? 1 : 2));
                if (!solution) return true;
                limited = solution->limited;
                for (std::size_t i = 0; i < tip; ++i) {
                    const auto swing = FromTo(positions[i + 1] - positions[i],
                        solution->joints[i + 1] - solution->joints[i]);
                    if (!swing) return true;
                    world[i] = Multiply(*swing, world[i]);
                }
            }
            WorldPose worlds;
            const auto parent = worlds.Read(pose.bones[pose.tailJoints.front()].ref.Parent());
            if (!parent) return true;
            auto previous = parent->rotation;
            for (std::size_t i = 0; i <= tip; ++i) {
                auto& bone = pose.bones[pose.tailJoints[i]];
                WriteRotation(bone.local, TailLocalRotation(world[i], previous, grip.parentOffsets[i]));
                previous = world[i];
            }
            return limited;
        }

        void ApplyEndpointSolution(Limb& limb, const EndpointGrip& grip, const EndpointSolution& solution,
                                   bool resetTwists = true)
        {
            Quaternion parent = grip.parent;
            for (const auto& link : grip.links) {
                auto local = link.bind;
                if (link.bone) {
                    const auto index = *link.bone;
                    const auto inverse = Fingers::Conjugate(parent);
                    if (limb.clavicle && index == limb.clavicle->bone) local = Multiply(inverse, solution.clavicle);
                    for (std::size_t i = 0; i < limb.joints.size(); ++i)
                        if (index == limb.joints[i]) local = Multiply(inverse, solution.joints[i].rotation);
                    auto& bone = pose.bones[index];
                    WriteRotation(bone.local, local);
                    bone.manualRotation = false;
                }
                parent = Multiply(parent, local);
            }

            if (!resetTwists) return;
            for (std::size_t i = 0; i < grip.twists.size(); ++i) {
                if (grip.twists[i].empty()) continue;
                const auto twist = EndpointTwistFrame(grip.rig, solution, i);
                if (!twist) continue;
                limb.twists[i]->frame = *twist;
                const auto proximal = TwistPhase::Deformation(twist->proximal.angle);
                const auto distal = TwistPhase::Deformation(twist->distal.angle);
                for (const auto& helper : grip.twists[i]) {
                    const auto relative = DistributeLimbTwist(helper.bindRelative, twist->axis, proximal, distal, helper.fraction);
                    WorldPose updatedParents;
                    auto& bone = pose.bones[helper.bone];
                    if (WriteWorldRotation(bone, Multiply(solution.joints[i].rotation, relative), updatedParents))
                        bone.manualRotation = false;
                }
            }
        }

        void UpdateGripTwists(const Interaction::Grip& grip, WorldPose& worlds, bool begin)
        {
            const auto& handle = pose.handles[grip.handle];
            for (std::size_t index = 0; index < pose.limbs.size(); ++index) {
                auto& limb = pose.limbs[index];
                const bool held = grip.limb && handle.limb == index;
                if (!limb || (!held && !grip.anchors[index])) continue;
                for (std::size_t i = 0; i < limb->twists.size(); ++i) {
                    auto& twist = limb->twists[i];
                    if (!twist) continue;
                    if (begin) for (auto& helper : twist->bones) helper.relativeRotation.reset();
                    const auto ref = pose.bones[limb->joints[i]].ref;
                    const auto start = worlds.Read(ref);
                    const auto end = worlds.Read(pose.bones[limb->joints[i + 1]].ref);
                    const auto parent = worlds.Read(ref.Parent());
                    if (!start || !end || !parent) continue;
                    const auto proximal = RelativeTo(*parent, *start);
                    const auto distal = RelativeTo(*start, *end);
                    if (!proximal || !distal) continue;
                    if (begin) {
                        if (!twist->frame.Begin(proximal->rotation, *distal)) continue;
                        // Loaded and mirrored poses already carry their authored skin rotations.
                        // Apply only the additional twist produced by this grip.
                        for (auto& helper : twist->bones) {
                            const auto world = worlds.Read(pose.bones[helper.bone].ref);
                            const auto local = world ? RelativeTo(*start, *world) : std::nullopt;
                            if (local) helper.relativeRotation = local->rotation;
                        }
                        continue;
                    }
                    const auto [proximalRoll, distalRoll] = twist->frame.Measure(proximal->rotation, distal->rotation);
                    for (const auto& helper : twist->bones) {
                        auto& bone = pose.bones[helper.bone];
                        // A positional solve gives the skin helpers back to the limb.
                        if (!helper.relativeRotation || (bone.manualRotation && (!held || grip.rotating))) continue;
                        const auto rotation = DistributeLimbTwist(*helper.relativeRotation, twist->frame.axis,
                            proximalRoll, distalRoll, helper.fraction);
                        WorldPose updatedParents;
                        if (WriteWorldRotation(bone, Multiply(start->rotation, rotation), updatedParents))
                            bone.manualRotation = false;
                    }
                }
            }
        }

        std::optional<Vec3> TwistRollAxis(const Handle& handle, WorldPose& worlds)
        {
            const auto& limb = *pose.limbs[handle.limb];
            const auto from = worlds.Read(pose.bones[limb.joints[handle.rollSegment]].ref);
            const auto to = worlds.Read(pose.bones[limb.joints[handle.rollSegment + 1]].ref);
            if (!from || !to) return std::nullopt;
            return Unit(to->position - from->position);
        }

        bool SolveAnchors(const Interaction::Grip& grip)
        {
            for (std::size_t i = 0; i < grip.anchors.size(); ++i) {
                if (!grip.anchors[i]) continue;
                WorldPose worlds;
                const auto& limb = *pose.limbs[i];
                const auto sample = Sample(limb, worlds);
                if (!sample) return false;
                const auto& anchor = *grip.anchors[i];
                const auto& current = sample->world.back();
                if (Length(current.position - anchor.position) <= 0.001 &&
                    HistoryStorage::SameRotation(current.rotation, anchor.rotation)) continue;
                if (grip.learnedStance && i >= 2) {
                    if (const auto reference = CaptureEndpointGrip(i, worlds, grip.anchors[i ^ 1])) {
                        const auto predictions = ankleModel.Predict(reference->rig, anchor.position);
                        if (!predictions.View().empty()) {
                            const auto learned = SolveEndpoint(reference->rig, anchor.position, predictions.View(), anchor.rotation);
                            if (learned && Length(learned->joints.back().position - anchor.position) < .001) {
                                ApplyEndpointSolution(*pose.limbs[i], *reference, *learned, false);
                                continue;
                            }
                        }
                    }
                }
                const auto clearance = ReadTorsoClearance(worlds);
                const auto edit = SolveAnchoredLimb(*sample, anchor, clearance ? &*clearance : nullptr);
                if (!edit) return false;
                ApplyLimb(limb, *edit);
            }
            return true;
        }

        void DrawGrip(const Interaction::Grip& grip, const DragView::Camera& camera, DragView::Frame& frame,
                      WorldPose& worlds, double dt)
        {
            if (const auto point = Marker(grip.handle, worlds)) {
                editor.beam.Update(camera.emitter, *point, dt, camera.aim.direction);
                frame.held = grip.handle;
                frame.rotating = grip.rotating;
                camera.ProjectTether(editor.beam, *point, frame);
            }
        }

        void EditPose(const DragView::Camera& camera, DragView::Frame& frame, const std::array<double, 2>& mouse,
                      double wheel, Vec3 travel, Vec3 keyAngles, double dt, double rotationScale, Vec3 rotationDirections)
        {
            auto& grip = *editor.grip;
            const auto handle = pose.handles[grip.handle];
            const bool seatLocked = pose.bones[handle.bone].seatAnchor ||
                (pose.seated && !grip.rotating && handle.action == HandleAction::Torso);
            if (seatLocked || (!grip.rotating && IsTriangle(handle.action))) {
                frame.limited = seatLocked;
                WorldPose worlds;
                DrawGrip(grip, camera, frame, worlds, dt);
                return;
            }
            const bool bend = handle.action == HandleAction::Bend;
            const bool endpoint = handle.action == HandleAction::Endpoint;
            const bool tail = grip.tail.has_value();
            const bool twist = handle.action == HandleAction::Twist;
            const bool weapon = handle.action == HandleAction::Weapon;
            const bool turning = grip.rotating;
            if (turning && !editor.rotation) {
                WorldPose worlds;
                const auto current = worlds.Read(pose.bones[handle.bone].ref);
                auto roll = twist ? TwistRollAxis(handle, worlds).value_or(camera.aim.direction) : camera.aim.direction;
                if (grip.tail && grip.tail->held + 1 < pose.tailJoints.size()) {
                    const auto& chain = *grip.tail;
                    roll = Unit(chain.joints[chain.held + 1].position - chain.joints[chain.held].position).value_or(roll);
                }
                if (current) editor.rotation = GripRotation::Begin(current->rotation, camera.right, camera.up, roll);
            }
            if (!turning) editor.rotation.reset();
            const auto priorRotation = editor.rotation ? editor.rotation->current : grip.bone.rotation;
            if (turning && editor.rotation)
                grip.targetRotation = editor.rotation->Update(
                    mouse[0], mouse[1], wheel, rotationScale, keyAngles,
                    tail ? editor.rotation->rollAxis : grip.viewForward, rotationDirections);
            grip.keyboardOffset = grip.keyboardOffset + travel;
            if (!turning) grip.distance.Nudge(wheel);
            // A latched nudge target stays in world space while the camera flies.
            const auto target = turning ? grip.point :
                (editor.nudge ? grip.point : grip.distance.Target(camera.aim, camera.right, camera.up)) + grip.keyboardOffset;
            const auto delta = target - grip.point;
            const bool turnEndpoint = endpoint && turning;
            const bool rotateInput = mouse[0] != 0 || mouse[1] != 0 || wheel != 0 || Length(keyAngles) > 0;
            const auto targetTravel = Length(target - grip.lastTarget);
            bool changed = targetTravel > 0.001 || (turning && (!(endpoint || tail || twist || weapon) || rotateInput));
            if (changed) grip.lastTarget = target;
            if (changed && grip.tail) {
                if (!turning || grip.targetRotation)
                    frame.limited = ApplyTailGrip(*grip.tail, delta, grip.targetRotation);
            } else if (changed && grip.endpoint) {
                const auto& reference = *grip.endpoint;
                const auto& model = reference.rig.leg ? ankleModel : wristModel;
                const auto predictions = model.Predict(reference.rig, target);
                const auto solution = SolveEndpoint(reference.rig, target, predictions.View(), {}, grip.endpointFlex);
                frame.limited = !solution || solution->limited;
                if (solution) ApplyEndpointSolution(*pose.limbs[handle.limb], reference, *solution);
            } else if (changed && grip.limb) {
                const auto& baseline = *grip.limb;
                const auto& limb = *pose.limbs[handle.limb];
                const Vec3 tip = endpoint ? target : baseline.world[2].position;
                const Vec3 pole = endpoint ? baseline.world[1].position : baseline.world[1].position +
                    delta * (handle.bone == limb.joints[0] ? 2 : 1);
                const auto turn = bend && turning && grip.targetRotation ?
                    std::optional{LimbRotation{handle.bone == limb.joints[0] ? 0U : 1U, *grip.targetRotation}} : std::nullopt;
                WorldPose worlds;
                const auto* clearance = grip.clearance ? &*grip.clearance : nullptr;
                std::optional<ArmDrag> armDrag;
                if (endpoint && !turning && handle.limb < 2)
                    if (const auto previous = ReadLimbJoints(limb, worlds)) armDrag = ArmDrag{*previous, targetTravel};
                const auto* arm = armDrag ? &*armDrag : nullptr;
                const auto poleTravel = bend && !turning ?
                    std::optional{targetTravel * (handle.bone == limb.joints[0] ? 2 : 1)} : std::nullopt;
                auto edit = turnEndpoint && grip.targetRotation ?
                    SolveEndpointRotation(baseline, priorRotation, *grip.targetRotation,
                        grip.followingLowerRotation.value_or(baseline.world[1].rotation), handle.limb >= 2, grip.bend, clearance) :
                    SolveLimb(baseline, tip, pole, grip.bend,
                        bend ? std::optional{baseline.world[2].rotation} : std::nullopt, turn, bend, clearance, arm,
                        handle.limb < 2 ? poleTravel : std::nullopt);
                if (edit) {
                    if (grip.clavicle) {
                        auto& support = *grip.clavicle;
                        const auto assisted = AssistLimbRoot(baseline, *edit, support.initial, tip, pole,
                            bend, grip.bend, clearance, arm, support.preferred, poleTravel);
                        auto& clavicle = pose.bones[limb.clavicle->bone];
                        if ((assisted.adjusted || support.applied) &&
                            WriteWorldRotation(clavicle, assisted.support, worlds)) {
                            edit = assisted.limb;
                            support.applied = assisted.adjusted;
                            if (support.preferred)
                                clavicle.manualRotation = !HistoryStorage::SameRotation(assisted.support, *support.preferred);
                        }
                    }
                    if (endpoint && !turning) edit->localRotations[2] = baseline.localRotations.back();
                    ApplyLimb(limb, *edit);
                    grip.bend = edit->bend;
                    frame.limited = edit->limited;
                    if (turnEndpoint) grip.followingLowerRotation = edit->lowerRotation;
                } else if (turnEndpoint && grip.targetRotation) {
                    WorldPose worlds;
                    WriteWorldRotation(pose.bones[handle.bone], *grip.targetRotation, worlds);
                }
            } else if (changed && (twist || weapon)) {
                if (grip.targetRotation) {
                    WorldPose worlds;
                    auto& bone = pose.bones[handle.bone];
                    if (WriteWorldRotation(bone, *grip.targetRotation, worlds)) bone.manualRotation = true;
                }
            } else if (changed) {
                const auto saved = pose.bones;
                const auto before = pose.translation;
                WorldPose worlds;
                if (handle.action == HandleAction::Torso && !turning) {
                    pose.translation = grip.translation + delta;
                } else {
                    const auto swing = FromTo(grip.point - grip.bone.position, target - grip.bone.position);
                    if (grip.neck && (grip.targetRotation || swing)) {
                        auto& head = pose.bones[handle.bone];
                        auto& neck = pose.bones[grip.neck->bone];
                        auto desired = grip.targetRotation.value_or(swing ? Multiply(*swing, grip.bone.rotation) : grip.bone.rotation);
                        WriteWorldRotation(neck, NeckFollow(grip.neck->rotation, grip.bone.rotation, desired), worlds);
                        WorldPose withNeck;
                        if (!grip.targetRotation) {
                            if (const auto moved = withNeck.Read(head.ref))
                                if (const auto aim = FromTo(grip.point - grip.bone.position, target - moved->position))
                                    desired = Multiply(*aim, grip.bone.rotation);
                        }
                        WriteWorldRotation(head, desired, withNeck);
                    } else if (grip.targetRotation) {
                        auto& bone = pose.bones[handle.bone];
                        if (WriteWorldRotation(bone, *grip.targetRotation, worlds) && rotateInput && IsTriangle(handle.action))
                            bone.manualRotation = true;
                    }
                    else if (swing) WriteWorldRotation(pose.bones[handle.bone], Multiply(*swing, grip.bone.rotation), worlds);
                }
                if (!SolveAnchors(grip)) {
                    pose.bones = saved;
                    pose.translation = before;
                    frame.limited = true;
                    changed = false;
                }
            }
            WorldPose worlds;
            if (changed) UpdateGripTwists(grip, worlds, false);
            DrawGrip(grip, camera, frame, worlds, dt);
        }

        bool DrawnSharingSupported(RE::PlayerCharacter& player)
        {
            // The live revision value can differ from the generated default.
            // Check the actual events on the third-person graph PlayerHub observes.
            return WithGraph(player, [](const RE::BShkbAnimationGraph& graph) {
                const auto* strings = GraphStrings(graph);
                if (!strings) return false;
                const auto& events = strings->eventNames;
                return std::ranges::find(events, kDrawnSharingStart, Name) != events.end() &&
                    std::ranges::find(events, kDrawnSharingStop, Name) != events.end();
            });
        }

        bool DrawnSharingActive(RE::PlayerCharacter& player)
        {
            return WithGraph(player, [](const RE::BShkbAnimationGraph& graph) {
                const auto* behavior = graph.behaviorGraph;
                if (!behavior || !behavior->activeNodes) return false;
                static const REL::Relocation<std::uintptr_t> stateMachineTable{RE::VTABLE_hkbStateMachine[0]};
                for (const auto& activeNode : *behavior->activeNodes) {
                    const auto* node = activeNode.nodeClone ? activeNode.nodeClone : activeNode.nodeTemplate;
                    if (!node || *reinterpret_cast<const std::uintptr_t*>(node) != stateMachineTable.address()) continue;
                    const auto& machine = *static_cast<const RE::hkbStateMachine*>(node);
                    if (Name(machine.name) != "Default_Behavior") continue;
                    for (const auto* state : machine.states) {
                        // CommonLib leaves the Havok state name and ID unnamed.
                        if (!state || static_cast<std::int32_t>(state->unk68) != machine.currentStateID) continue;
                        const auto* name = reinterpret_cast<const char*>(state->unk60 & ~std::uintptr_t{1});
                        if (name && std::string_view{name} == "GModPoseShareDrawn") return true;
                    }
                }
                return false;
            });
        }

        void StartSharing(RE::PlayerCharacter& player, double dt)
        {
            if (!editor.sharing) return;
            auto& share = *editor.sharing;
            if (player.Get3D(false) != share.root || player.GetParentCell() != share.cell) return;
            const auto weapon = player.AsActorState()->GetWeaponState();
            const bool drawn = weapon == RE::WEAPON_STATE::kDrawn;
            if (player.IsInMidair() || (!drawn && weapon != RE::WEAPON_STATE::kSheathed)) {
                share.stableFor = 0;
                return;
            }
            if (share.weapon != weapon) {
                if (std::exchange(share.requestedDrawn, false))
                    player.NotifyAnimationGraph(RE::BSFixedString{kDrawnSharingStop});
                share.started = false;
                share.weapon = weapon;
                share.stableFor = 0;
                share.retry = 0;
                share.attempts = 0;
            }
            if (share.started) return;
            // ActorState can settle before the graph finishes its weapon transition.
            constexpr double settleSeconds = .4;
            share.stableFor += dt;
            if (share.stableFor < settleSeconds) return;
            if (drawn && share.requestedDrawn && DrawnSharingActive(player)) {
                share.started = true;
                SKSE::log::info("Started GModPose sharing in the native drawn-weapon graph");
                return;
            }
            share.retry -= dt;
            if (share.retry > 0) return;
            if (drawn && share.attempts == 0 && !DrawnSharingSupported(player)) {
                SKSE::log::warn("The third-person animation graph is missing GModPose's drawn-weapon sharing events");
                Notify("GModPose: the loaded Pandora graph is missing drawn-weapon sharing events");
                share.retry = std::numeric_limits<double>::infinity();
                return;
            }
            const auto event = drawn ? kDrawnSharingStart : "GModPoseShareStart";
            const bool accepted = player.NotifyAnimationGraph(RE::BSFixedString{event});
            if (drawn && share.attempts == 0)
                SKSE::log::info("Requested GModPose drawn-weapon sharing: event={} accepted={}", event, accepted);
            share.requestedDrawn |= drawn && accepted;
            if (!drawn && accepted) {
                share.started = true;
                SKSE::log::info("Started GModPose sharing animation after the sheathed weapon state settled");
                return;
            }
            if (++share.attempts >= 12) {
                SKSE::log::warn("GModPose sharing did not start: event={} accepted={}",
                    event, accepted || share.requestedDrawn);
                Notify("GModPose: the animation graph could not start pose sharing");
                // Keep ownership of a queued request so closing can still stop it.
                share.retry = std::numeric_limits<double>::infinity();
            } else share.retry = 0.25;
        }

        void StopSharing(RE::PlayerCharacter* player)
        {
            const auto share = std::exchange(editor.sharing, std::nullopt);
            if (!share || (!share->started && !share->requestedDrawn) || loading.load() || !player || player->Get3D(false) != share->root ||
                player->GetParentCell() != share->cell || player->IsDead() || player->IsInCombat() ||
                player->IsInKillMove() || player->IsInRagdollState() || player->GetOccupiedFurniture() ||
                player->AsActorState()->GetSitSleepState() != RE::SIT_SLEEP_STATE::kNormal) return;
            const auto weapon = player->AsActorState()->GetWeaponState();
            if (!share->requestedDrawn && weapon != RE::WEAPON_STATE::kSheathed) {
                SKSE::log::info("Skipping the sharing idle reset on editor exit: weapon state={}",
                    static_cast<unsigned>(weapon));
                return;
            }
            const auto event = share->requestedDrawn ? kDrawnSharingStop : "IdleForceDefaultState";
            const bool accepted = player->NotifyAnimationGraph(RE::BSFixedString{event});
            SKSE::log::info("Stopped GModPose sharing animation: event={} accepted={} weapon={}",
                event, accepted, static_cast<unsigned>(weapon));
        }

        void ReleaseTextInput()
        {
            auto* owner = textInputOwner.exchange(nullptr);
            if (!owner || owner != RE::ControlMap::GetSingleton()) return;
            const int before = owner->GetRuntimeData().textEntryCount;
            if (before > 0) owner->AllowTextInput(false);
            SKSE::log::debug("Released GModPose text input: entries={} -> {}", before,
                static_cast<int>(owner->GetRuntimeData().textEntryCount));
        }

        void EnablePanelTextInput()
        {
            auto* controls = RE::ControlMap::GetSingleton();
            if (!controls) return;
            const auto count = controls->GetRuntimeData().textEntryCount;
            if (OwnsTextInput(controls) && count > 0) return;
            // Skyrim emits CharEvents only with text input enabled. PlayerHub
            // 2.1 routes those events but does not request them itself.
            if (count != 0) return;
            controls->AllowTextInput(true);
            if (controls->GetRuntimeData().textEntryCount > 0) {
                textInputOwner.store(controls);
                SKSE::log::debug("Enabled Skyrim character events while posing");
            }
        }

        void CancelPendingSeat()
        {
            if (pendingSeat) {
                SeatBlock::End();
                pendingSeat.reset();
            }
        }

        void Tick()
        {
            if (!ready.load()) return;
            const unsigned thread = REX::W32::GetCurrentThreadId();
            unsigned unbound = 0;
            threadID.compare_exchange_strong(unbound, thread);
            if (threadID.load() != thread) return;
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!Running()) {
                ReleaseTextInput();
                FaceActor::End(player);
                StopSharing(player);
                editor = {};
                PosingCamera::End(player);
            }
            Input input;
            EditorKeys::State keys;
            const bool gameBlocked = DragView::InputBlockReason() != nullptr;
            bool blocked = gameBlocked;
            {
                std::lock_guard lock(inputMutex);
                if (std::exchange(mousePending, false)) {
                    const bool captured = !std::exchange(mouseDelivered, false);
                    if (mouseCaptured != captured)
                        SKSE::log::debug("Posing mouse delivery: {}", captured ? "captured by another input owner" : "resumed");
                    mouseCaptured = captured;
                }
                blocked |= mouseCaptured;
                std::swap(input, pendingInput);
                if (blocked) deliveredKeys = {};
                keys = deliveredKeys;
                deliveredKeys.pressedKeys.reset();
                deliveredKeys.pressedMouse.reset();
            }
            const bool closePanel = closeSettings.exchange(false);
            if (closePanel && settingsOpen.exchange(false)) {
                PoseSettings::ResetMenu();
                std::lock_guard lock(inputMutex);
                deliveredKeys = {};
                keys = {};
            }
            const auto command = commands.exchange(0);
            if (!DragView::Available()) {
                CancelPendingSeat();
                pendingThirdPerson.reset();
                if (Running()) {
                    Invalidate();
                    StopSharing(player);
                    PosingCamera::End(player);
                }
                if (command) Notify("GModPose: posing renderer unavailable");
                return;
            }
            if (!gameBlocked && command && Running()) {
                SKSE::log::debug("Closing pose editor: weapon state={}",
                    player ? static_cast<int>(player->AsActorState()->GetWeaponState()) : -1);
                Invalidate();
                StopSharing(player);
                PosingCamera::End(player);
                return;
            }
            if (!Eligible(player)) {
                CancelPendingSeat();
                pendingThirdPerson.reset();
                if (Running()) Invalidate();
                return;
            }
            const auto sitState = player->AsActorState()->GetSitSleepState();
            const auto furniture = (command || Running() || pendingSeat || pendingThirdPerson) ?
                player->GetOccupiedFurniture() : RE::ObjectRefHandle{};
            const bool standing = sitState == RE::SIT_SLEEP_STATE::kNormal && !furniture;
            const bool seated = sitState == RE::SIT_SLEEP_STATE::kIsSitting && furniture;
            if (pendingThirdPerson) {
                const auto& wait = *pendingThirdPerson;
                if (command || gameBlocked || (!standing && !seated) ||
                    wait.generation != generation.load() || wait.cell != player->GetParentCell()) {
                    pendingThirdPerson.reset();
                    return;
                }
                if (std::chrono::steady_clock::now() >= wait.deadline) {
                    pendingThirdPerson.reset();
                    SKSE::log::warn("Third-person camera or player graph was unavailable after switching from first person");
                    Notify("GModPose: could not open the editor in third person");
                    return;
                }
            }
            bool resumeSeatedPose = false;
            if (pendingSeat) {
                const auto& wait = *pendingSeat;
                if (command || wait.generation != generation.load() || player->Get3D(false) != wait.capture.root ||
                    player->GetParentCell() != wait.capture.cell || (furniture && furniture != wait.furniture) ||
                    std::chrono::steady_clock::now() >= wait.deadline) {
                    CancelPendingSeat();
                    Notify("GModPose: seated pose capture cancelled");
                    return;
                }
                if (!standing || blocked) return;
                resumeSeatedPose = true;
            }
            if ((!standing && !seated) || (Running() && furniture != editor.seat)) {
                if (Running()) {
                    if (editor.seat) SKSE::log::info("Seated posing ended because the furniture state changed");
                    Invalidate();
                    PosingCamera::End(player);
                } else if (!blocked && command) Notify("GModPose: finish sitting down or exit the furniture interaction");
                return;
            }
            if (gameBlocked) {
                if (Running()) {
                    FaceActor::Update(*player, std::nullopt);
                    editor.grip.reset();
                    editor.rotation.reset();
                    editor.buttons = {};
                    editor.beam = {};
                    rotationHeld.store(false);
                    DragView::Clear();
                }
                return;
            }
            if (!blocked && (command || resumeSeatedPose || pendingThirdPerson)) {
                auto* camera = RE::PlayerCamera::GetSingleton();
                if (pendingThirdPerson) {
                    if (!camera || !camera->IsInThirdPerson() ||
                        !WithGraph(*player, [](const RE::BShkbAnimationGraph& graph) { return !graph.boneNodes.empty(); }))
                        return;
                    pendingThirdPerson.reset();
                } else if (command && camera && camera->IsInFirstPerson()) {
                    const PendingThirdPerson wait{player->GetParentCell(),
                        std::chrono::steady_clock::now() + std::chrono::seconds{2}, generation.load()};
                    // Let Skyrim update the third-person camera and graph before capturing either.
                    if (camera->ForceThirdPerson()) pendingThirdPerson = wait;
                    else Notify("GModPose: could not switch to third person");
                    return;
                }
                const bool furnitureCamera = seated && camera && camera->currentState &&
                    camera->currentState->id == RE::CameraState::kFurniture;
                if (!camera || (!camera->IsInThirdPerson() && !camera->IsInFreeCameraMode() && !furnitureCamera)) {
                    // The native exit can clear occupancy before the camera returns.
                    if (!resumeSeatedPose) {
                        SKSE::log::warn("Editor requires third-person or free camera: camera state={}",
                            camera && camera->currentState ? static_cast<int>(camera->currentState->id) : -1);
                        Notify("GModPose: switch to third person first");
                    }
                    return;
                }
                if (seated && RequestSharedSeat(*player, furniture)) return;
                bool captured = false;
                if (resumeSeatedPose) {
                    auto wait = std::exchange(pendingSeat, std::nullopt);
                    const auto seat = wait->furniture.get();
                    if (!seat || SeatBlock::TakenByOther(*seat, wait->markerID, *player)) {
                        SeatBlock::End();
                        Notify("GModPose: the seat is no longer available");
                        return;
                    }
                    captured = WithGraph(*player, [&](const RE::BShkbAnimationGraph& graph) {
                        if (!SameGraph(graph, wait->capture) || !std::ranges::all_of(wait->capture.bones,
                            [&](const FrozenBone& bone) {
                                return bone.index == kSceneBoneIndex || GraphBoneRef(graph.boneNodes[bone.index]) == bone.ref;
                            }))
                            return false;
                        return PublishCapture(std::move(wait->capture), wait->generation);
                    });
                    if (captured) {
                        SKSE::log::info("Opened the editor with the saved seated pose after the native exit");
                    } else SeatBlock::End();
                } else captured = CapturePose(*player, seated);
                if (!captured) {
                    Notify("GModPose: player pose capture failed");
                    return;
                }
                // Capture the rendered face before the sharing animation can change it.
                FaceActor::Update(*player, std::nullopt);
                {
                    const std::scoped_lock lock(poseMutex);
                    if (!Running()) return;
                    pose.historyFace = FaceActor::Snapshot();
                    BeginHistory();
                }
                editor.seat = furniture;
                PosingCamera::Begin(*player, seated);
                if (seated) SKSE::log::info("Entered seated posing on furniture handle {:08X}", furniture.native_handle());
                editor.playerHub = REX::W32::GetModuleHandleW(L"PlayerHub.dll") != nullptr;
                if (!seated && SharingAvailable(*player))
                    editor.sharing = Interaction::Sharing{player->Get3D(false), player->GetParentCell()};
            }
            if (!Running()) return;
            const auto bindings = PoseSettings::Get();
            const double rotationScale = bindings.rotationSensitivityPercent / 100.0;
            const Vec3 rotationDirections{bindings.invertYaw ? -1.0 : 1.0,
                bindings.invertPitch ? -1.0 : 1.0, bindings.invertRoll ? -1.0 : 1.0};
            const bool editingFingers = fingerOpen.load();
            const bool editingFace = faceOpen.load();
            const bool editingBody = bodyOpen.load();
            const bool panelOpen = SettingsOpen() || editingFingers || editingFace || editingBody;
            const bool navigate = !blocked && !SettingsOpen() && !input.actions.empty();
            if (!blocked && input.toggleMirror) editor.mirror = !editor.mirror;
            const auto buttonDown = [&](unsigned code) {
                if (code >= PoseSettings::mouseBase)
                    return code - PoseSettings::mouseBase < keys.mouse.size() && keys.mouse[code - PoseSettings::mouseBase];
                return keys.Down(code);
            };
            const auto down = [&](Action action) {
                return buttonDown(bindings.Key(action)) || buttonDown(bindings.Alternate(action));
            };
            const auto delta = RE::Main::QFrameAnimTime();
            const auto dt = std::isfinite(delta) ? std::clamp<double>(delta, 0, 0.05) : 0;
            if (!blocked) {
                PosingCamera::Update();
                if (editor.playerHub || editingFingers || editingFace || editingBody) EnablePanelTextInput();
                if (!panelOpen) PosingCamera::Fly({
                    input.motion[0], input.motion[1],
                    int(down(Action::FlyForward)) - int(down(Action::FlyBack)),
                    int(down(Action::FlyRight)) - int(down(Action::FlyLeft)),
                    down(Action::SlowFlight), !editor.nudge && (RotatingHandle() || down(Action::Rotate))}, dt);
                else if (!closePanel) PoseSettings::QueuePointer(
                    input.motion[0], input.motion[1], input.wheel, input.panelClicks, input.panelText);
            }
            const auto camera = DragView::ReadCamera();
            DragView::Frame frame;
            bool openFace = false;
            std::optional<FacePose::Edit> faceEdit;
            if (!blocked) StartSharing(*player, dt);
            const bool valid = WithGraph(*player, [&](RE::BShkbAnimationGraph& graph) {
                std::lock_guard lock(poseMutex);
                if (!Running() || !SameGraph(graph, pose) || pose.cell != player->GetParentCell()) return false;
                if (pose.nextSharingRefresh != std::chrono::steady_clock::time_point{}) {
                    const auto now = std::chrono::steady_clock::now();
                    if (now >= pose.nextSharingRefresh) {
                        pose.nextSharingRefresh = now + refreshConfig.period;
                        pose.sharingRefreshOffset = !pose.sharingRefreshOffset;
                        pose.sharingRefreshView = pose.sharingRefreshOffset && camera ? std::optional{camera->aim} : std::nullopt;
                    }
                }
                if (!pose.translationReported && pose.comIndex && Length(pose.translation) >= 1) {
                    const auto& frozen = pose.bones[*pose.comIndex];
                    const auto com = GraphBone(graph.boneNodes[frozen.index]);
                    if (com.local) {
                        const auto actual = com.local->translate - frozen.local.translate;
                        SKSE::log::info("NPC COM local translation readback after pose movement: {:.2f}, {:.2f}, {:.2f}",
                            actual.x, actual.y, actual.z);
                        pose.translationReported = true;
                    }
                }
                auto faceTouched = ApplyPendingEdits(closePanel || navigate);
                if (navigate) {
                    CommitHistory();
                    bool restored = false;
                    for (const auto& command : input.actions) {
                        if (const auto* action = std::get_if<HistoryAction>(&command)) {
                            if (const auto* saved = editorHistory.Move(*action)) {
                                faceTouched |= ApplyEditorFrame(*saved, FrameSource::History);
                                restored = true;
                            }
                        } else if (editingBody) {
                            restored |= ProcessBodyPreset(std::get<BodyPresetCommand>(command), faceTouched);
                        }
                    }
                    PublishHistoryActions();
                    if (restored) {
                        ShowHistoryPose();
                        editor.grip.reset();
                        editor.rotation.reset();
                        editor.beam = {};
                        editor.movingPose = false;
                    }
                }
                if (faceTouched && pose.historyFace) {
                    const auto& face = *pose.historyFace;
                    faceEdit = FacePose::Edit{face.pose, faceTouched, face.expressions};
                }
                if (closePanel && (editingFingers || editingFace || editingBody)) SaveHistory();
                if (closePanel && editingBody) {
                    bodyOpen.store(false);
                    PoseSettings::ResetMenu();
                    if (!editor.playerHub) ReleaseTextInput();
                }
                if (closePanel && editingFingers) {
                    fingerOpen.store(false);
                    pose.editingHand.reset();
                    FingerPanel::End();
                    PoseSettings::ResetMenu();
                    if (!editor.playerHub) ReleaseTextInput();
                }
                WorldPose worlds;
                if (editingFace) FacePanel::SetCameraGaze(camera ? CameraGaze(*camera, worlds) : std::nullopt);
                std::vector<std::optional<Vec3>> markers;
                const auto projectHandles = [&] {
                    WorldPose current;
                    markers.clear();
                    markers.reserve(pose.handles.size());
                    for (std::size_t i = 0; i < pose.handles.size(); ++i)
                        markers.push_back(Marker(i, current));
                    camera->Project(markers, frame);
                };
                if (camera) projectHandles();
                bool newGrip = false;
                const bool focused = !blocked && camera.has_value();
                frame.mirror = focused && !panelOpen && editor.mirror;
                const auto resetGrip = [&] {
                    editor.grip.reset();
                    editor.rotation.reset();
                    editor.beam = {};
                };
                if (editor.grip) {
                    const auto& handle = pose.handles[editor.grip->handle];
                    if (handle.action == HandleAction::Weapon && !VisibleWeapon(*player, pose.bones[handle.bone].ref)) {
                        CommitHistory();
                        resetGrip();
                    }
                }
                const auto startGrip = [&](std::size_t index, bool rotate, bool mirror) {
                    if (!focused || !markers[index]) return;
                    WorldPose current;
                    const auto point = Marker(index, current);
                    if (!point) return;
                    BeginGrip(index, *point, rotate, *camera, current);
                    newGrip = editor.grip.has_value();
                    if (newGrip && mirror) MirrorLimb(pose.handles[index].bone);
                };
                const bool firedMiddle = focused && !panelOpen && input.poserPressed;
                if (firedMiddle) camera->ProjectMiddlePulse(frame);
                const bool openedBody = firedMiddle && frame.hover && pose.handles[*frame.hover].poser &&
                    pose.handles[*frame.hover].action == HandleAction::Torso;
                if (openedBody) {
                    OpenBodyPresets();
                    PoseSettings::ResetMenu();
                    bodyOpen.store(true);
                }
                if (firedMiddle && frame.hover)
                    openFace = pose.bones[pose.handles[*frame.hover].bone].name == kHead.back();
                const bool openedFingers = firedMiddle && !openedBody && !openFace && frame.hover &&
                    OpenFingerPanel(*frame.hover, editor.mirror);
                if (openFace || openedFingers || openedBody) {
                    if (editor.grip) CommitHistory();
                    resetGrip();
                    editor.nudge = false;
                }
                if ((openedFingers || openedBody) && !editor.playerHub) EnablePanelTextInput();
                const bool posingDetails = editingFingers || editingFace || editingBody || openedFingers || openFace || openedBody;
                using Button = EditorButton;
                editor.buttons.Update(focused && !posingDetails, {}, input.buttons,
                    [&](Button button, MouseEdge edge) {
                        if (posingDetails) return;
                        if (button == Button::Guide) {
                            if (edge == MouseEdge::Down) editor.hidden = !editor.hidden;
                            return;
                        }
                        if (button == Button::Settings) {
                            if (edge == MouseEdge::Down) {
                                if (editor.grip) CommitHistory();
                                settingsOpen.store(!SettingsOpen());
                                PoseSettings::ResetMenu();
                                {
                                    std::lock_guard inputLock(inputMutex);
                                    deliveredKeys = {};
                                }
                                resetGrip();
                                editor.nudge = false;
                            }
                            return;
                        }
                        if (SettingsOpen() || panelOpen) return;
                        if (button == Button::Nudge) {
                            if (edge == MouseEdge::Down) {
                                if (editor.grip) CommitHistory();
                                resetGrip();
                                editor.nudge = !editor.nudge;
                            }
                            return;
                        }
                        const bool rotate = button == Button::Rotate;
                        if (editor.nudge) {
                            if (edge != MouseEdge::Down) return;
                            // The other button changes the beam on the latched handle even off target.
                            const bool switchMode = editor.grip && editor.grip->rotating != rotate;
                            const auto index = switchMode ? std::optional{editor.grip->handle} : frame.hover;
                            if (!index || (editor.grip && editor.grip->handle == *index && !switchMode)) return;
                            CommitHistory();
                            resetGrip();
                            startGrip(*index, rotate, frame.mirror);
                            return;
                        }
                        if (edge == MouseEdge::Up) CommitHistory();
                        if (edge == MouseEdge::Down) {
                            if (!rotate && editor.grip) return;
                            if (!editor.grip && editor.movingPose) CommitHistory();
                            const auto index = editor.grip ? std::optional{editor.grip->handle} : frame.hover;
                            if (!index) return;
                            resetGrip();
                            startGrip(*index, rotate, frame.mirror);
                        } else if (editor.grip && editor.grip->rotating == rotate) {
                            const auto index = editor.grip->handle;
                            resetGrip();
                            if (rotate && editor.buttons.Held(Button::Grab)) startGrip(index, false, frame.mirror);
                        }
                    });
                const auto axis = [&](Action positive, Action negative, bool includeTap) {
                    const auto pressed = [&](unsigned code) {
                        if (code >= PoseSettings::mouseBase) {
                            const auto index = code - PoseSettings::mouseBase;
                            return index < keys.pressedMouse.size() && keys.pressedMouse[index];
                        }
                        return code != 0 && code < keys.pressedKeys.size() && keys.pressedKeys[code];
                    };
                    const auto active = [&](Action action) {
                        return down(action) || (includeTap &&
                            (pressed(bindings.Key(action)) || pressed(bindings.Alternate(action))));
                    };
                    return int(active(positive)) - int(active(negative));
                };
                Vec3 travel{}, keyAngles{};
                bool movingPose = false;
                bool turnedWholePose = false;
                if (focused && !panelOpen && !posingDetails && !SettingsOpen()) {
                    // Movement keys edit a latched handle. With no latch in Nudge mode, they leave the pose alone.
                    const bool nudging = editor.nudge && editor.grip.has_value();
                    const int forward = axis(Action::PoseForward, Action::PoseBack, nudging);
                    const int across = axis(Action::PoseRight, Action::PoseLeft, nudging);
                    const int vertical = axis(Action::Raise, Action::Lower, nudging);
                    constexpr double turnRate = std::numbers::pi / 3;
                    if (nudging) {
                        if (editor.grip->rotating)
                            keyAngles = Vec3{-double(across), -double(forward), double(vertical)} * (turnRate * dt);
                        else
                            travel = PoseTravel(camera->aim.direction, camera->right, forward, across, vertical, dt / 3);
                    } else if (!pose.seated && !editor.nudge) {
                        const auto slide = PoseTravel(camera->aim.direction, camera->right, forward, across, vertical, dt);
                        pose.translation = pose.translation + slide;
                        movingPose = Length(slide) > 0;
                    }
                    if (!pose.seated) {
                        turnedWholePose = TurnWholePose(axis(Action::Counterclockwise, Action::Clockwise, false) *
                            turnRate * dt * rotationScale);
                        movingPose |= turnedWholePose;
                    }
                }
                if (std::exchange(editor.movingPose, movingPose) && !movingPose && !editor.grip)
                    CommitHistory();
                if (movingPose) projectHandles();
                // Keep the regular beam attached to the translated pose.
                if (movingPose && editor.grip) {
                    const auto index = editor.grip->handle;
                    const bool rotate = editor.grip->rotating;
                    resetGrip();
                    startGrip(index, rotate, frame.mirror);
                }
                if (!focused) {
                    if (editor.nudge && editor.grip) CommitHistory();
                    resetGrip();
                    editor.nudge = false;
                }
                rotationHeld.store(!editor.nudge && editor.grip && editor.grip->rotating);
                if (focused && editor.grip) {
                    const bool ignorePointer = newGrip || editor.nudge;
                    EditPose(*camera, frame, ignorePointer ? std::array<double, 2>{} : input.motion,
                        ignorePointer ? 0 : input.wheel, travel, keyAngles, dt, rotationScale, rotationDirections);
                    if (frame.mirror) MirrorLimb(pose.handles[editor.grip->handle].bone);
                }
                if (auto& breathing = pose.breathing) {
                    const bool idle = focused && !editor.grip && !movingPose && !posingDetails && !navigate;
                    const auto lift = breathing->cycle.Update(bindings.breathing && idle, dt);
                    breathing->offset.reset();
                    if (lift > 0) {
                        const auto chest = ReadTransform(&pose.bones[breathing->chest].local);
                        if (chest) {
                            if (const auto offset = Rotate(chest->rotation, breathing->chestAxis * lift))
                                breathing->offset = Native(*offset);
                        }
                    }
                }
                frame.nudge = focused && editor.nudge;
                if (focused && !editor.grip && frame.hover) frame.hoverLabel = pose.handles[*frame.hover].label;
                if (editor.grip && !editor.grip->rotating) {
                    const auto action = pose.handles[editor.grip->handle].action;
                    if (IsTriangle(action))
                        frame.hoverLabel = "Triangle nodes can’t move";
                }
                const bool showSupport = focused && !editor.hidden && !posingDetails && !SettingsOpen();
                const auto now = std::chrono::steady_clock::now();
                // Limit physics queries for the HUD. Active torso grips show their captured pins.
                const bool sampleSupport = showSupport && now >= editor.nextSupportSample;
                if (sampleSupport) editor.nextSupportSample = now + std::chrono::milliseconds(100);
                WorldPose supportWorlds;
                for (std::size_t i = 0; i < pose.handles.size(); ++i) {
                    const auto& handle = pose.handles[i];
                    const auto bit = std::uint64_t{1} << i;
                    if (handle.poser) frame.circleHandles |= bit;
                    if (IsTriangle(handle.action)) frame.triangleHandles |= bit;
                    if (!showSupport || handle.action != HandleAction::Endpoint || handle.limb < 2) continue;
                    const auto leg = handle.limb;
                    if (editor.grip && editor.grip->supportChecked[leg]) {
                        if (editor.grip->anchors[leg]) frame.plantedHandles |= bit;
                    } else {
                        if (sampleSupport) {
                            const auto ankle = supportWorlds.Read(pose.bones[handle.bone].ref);
                            editor.supportedFeet[leg] = ankle && HasFootSupport(*pose.limbs[leg], leg, *ankle, supportWorlds);
                        }
                        if (editor.supportedFeet[leg]) frame.plantedHandles |= bit;
                    }
                }
                return true;
            });
            if (!valid) {
                Invalidate();
                return;
            }
            const bool faceValid = FaceActor::Update(*player, faceEdit);
            if (editingFace && (closePanel || !faceValid)) {
                faceOpen.store(false);
                FacePanel::End();
                PoseSettings::ResetMenu();
                if (!editor.playerHub) ReleaseTextInput();
            }
            if (openFace) {
                const auto face = FaceActor::Snapshot();
                const std::scoped_lock lock(poseMutex);
                if (!Running()) return;
                if (face) {
                    if (!pose.historyFace) pose.historyFace = face;
                    FacePanel::Begin(face->pose, face->supported, face->expressions);
                    PoseSettings::ResetMenu();
                    faceOpen.store(true);
                    EnablePanelTextInput();
                } else Notify("GModPose: facial animation controls are unavailable");
            }
            frame.settings = SettingsOpen();
            frame.finger = fingerOpen.load();
            frame.face = faceOpen.load();
            frame.body = bodyOpen.load();
            frame.drawWorld = !blocked;
            frame.visible = !gameBlocked && ((frame.settings || frame.finger || frame.face || frame.body) ? !blocked : !editor.hidden);
            DragView::Publish(std::move(frame));
        }

        struct MainHook
        {
            static void Thunk(std::int64_t context) { original(context); Tick(); }
            static inline HookOriginal<decltype(Thunk)> original;
        };

        bool PointerEvent(const RE::InputEvent& event)
        {
            if (event.GetDevice() != RE::INPUT_DEVICE::kMouse) return false;
            if (const auto* motion = event.AsMouseMoveEvent()) return motion->mouseInputX || motion->mouseInputY;
            const auto* button = event.AsButtonEvent();
            return button && button->IsDown();
        }

        struct MousePoll
        {
            static void Thunk(RE::BSWin32MouseDevice* self, float dt)
            {
                original(self, dt);
                if (!Running()) return;
                auto* queue = RE::BSInputEventQueue::GetSingleton();
                for (auto* event = queue ? queue->GetQueueHead() : nullptr; event; event = event->next) {
                    if (!PointerEvent(*event)) continue;
                    // Compare an actual queued pointer event with what survives
                    // input filtering. An idle mouse tells us nothing about focus.
                    std::lock_guard lock(inputMutex);
                    mousePending = true;
                    mouseDelivered = false;
                    break;
                }
            }

            static inline HookOriginal<decltype(Thunk)> original;
        };

        bool TypingPresetName()
        {
            return Running() && (fingerOpen.load() || faceOpen.load() || bodyOpen.load()) && PoseSettings::PresetNameActive();
        }

        struct KeyboardTrace
        {
            // Follow fresh button events through the hook chain without recording
            // any printable characters. The poll and sink run on the dispatch thread.
            enum SampleFlag { letter = 1, character = 2, enter = 4 };
            static constexpr int unknown = std::numeric_limits<int>::min();
            static inline std::atomic_int lastPoll{unknown}, lastDispatch{unknown};
            enum class Probe { None, Console, Enter };
            static inline thread_local Probe watchingDispatch{};

            static int Sample(const RE::InputEvent* event)
            {
                using Key = EditorKeys::Key;
                int sample = 0;
                for (; event; event = event->next) {
                    if (event->AsCharEvent()) sample |= character;
                    if (const auto* button = event->AsButtonEvent(); button && button->IsDown() &&
                        event->GetDevice() == RE::INPUT_DEVICE::kKeyboard) {
                        const auto key = button->GetIDCode();
                        if ((key >= Key::kQ && key <= Key::kP) || (key >= Key::kA && key <= Key::kL) ||
                            (key >= Key::kZ && key <= Key::kM)) sample |= letter;
                        if (key == Key::kEnter || key == Key::kKP_Enter) sample |= enter;
                    }
                }
                return sample;
            }

            static void Log(Probe probe, const char* stage, int sample, std::atomic_int& last)
            {
                const auto* controls = RE::ControlMap::GetSingleton();
                const auto* data = controls ? &controls->GetRuntimeData() : nullptr;
                const int count = data ? data->textEntryCount : 0;
                const int context = data && !data->contextPriorityStack.empty() ?
                    static_cast<int>(data->contextPriorityStack.back()) : -1;
                if (probe == Probe::Enter) {
                    SKSE::log::debug("Enter input {}: enter button={} context={} text entries={} posing={} preset name={}",
                        stage, (sample & enter) != 0, context, count, Running(), TypingPresetName());
                } else {
                    const int signature = count * 8 + sample;
                    if (last.exchange(signature, std::memory_order_relaxed) != signature)
                        SKSE::log::debug("Console input {}: letter button={} character event={} text entries={} context={} posing={}",
                            stage, (sample & letter) != 0, (sample & character) != 0, count, context, Running());
                }
            }

            static void Poll(bool consoleOpen)
            {
                watchingDispatch = Probe::None;
                if (!consoleOpen) {
                    lastPoll.store(unknown, std::memory_order_relaxed);
                    lastDispatch.store(unknown, std::memory_order_relaxed);
                }
                if (auto* queue = RE::BSInputEventQueue::GetSingleton()) {
                    const auto sample = Sample(queue->GetQueueHead());
                    if (consoleOpen && (sample & letter)) watchingDispatch = Probe::Console;
                    else if (!consoleOpen && (sample & enter)) watchingDispatch = Probe::Enter;
                    if (watchingDispatch != Probe::None)
                        Log(watchingDispatch, "after keyboard poll", sample, lastPoll);
                }
            }

            static void Dispatch(RE::InputEvent* const* events)
            {
                const auto probe = std::exchange(watchingDispatch, Probe::None);
                if (probe != Probe::None)
                    Log(probe, "at GModPose sink", Sample(events ? *events : nullptr), lastDispatch);
            }
        };

        struct KeyboardPoll
        {
            static void Thunk(RE::BSWin32KeyboardDevice* self, float dt)
            {
                original(self, dt);
                auto* ui = RE::UI::GetSingleton();
                const bool consoleOpen = ui && ui->IsMenuOpen(RE::Console::MENU_NAME);
                KeyboardTrace::Poll(consoleOpen);
                if (consoleOpen) return;
                if (!TypingPresetName() || DragView::InputBlockReason()) return;
                auto* queue = RE::BSInputEventQueue::GetSingleton();
                if (!queue) return;

                // Claim text before the dispatch hook where PlayerHub checks hotkeys.
                std::lock_guard lock(inputMutex);
                if (mouseCaptured) return;
                auto** link = &queue->GetQueueHead();
                RE::InputEvent* tail = nullptr;
                while (auto* event = *link) {
                    if (const auto* text = event->AsCharEvent()) {
                        pendingInput.panelText.push_back({text->keyCode, true, false});
                        *link = event->next;
                    } else if (const auto* button = event->AsButtonEvent(); button &&
                        event->GetDevice() == RE::INPUT_DEVICE::kKeyboard && button->IsPressed()) {
                        if (button->IsDown()) {
                            const auto code = button->GetIDCode();
                            if (code == EditorKeys::Key::kEnter || code == EditorKeys::Key::kKP_Enter)
                                SKSE::log::debug("GModPose preset-name field received Enter before overlay dispatch");
                            pendingInput.panelText.push_back({code, false, true});
                        }
                        *link = event->next;
                    } else {
                        // Let releases through so other listeners can clear keys they already own.
                        tail = event;
                        link = &event->next;
                    }
                }
                queue->GetQueueTail() = tail;
            }

            static inline HookOriginal<decltype(Thunk)> original;
        };

        class InputSink final : public RE::BSTEventSink<RE::InputEvent*>
        {
            RE::BSEventNotifyControl ProcessEvent(RE::InputEvent* const* events,
                RE::BSTEventSource<RE::InputEvent*>*) override
            {
                KeyboardTrace::Dispatch(events);
                // PlayerHub takes its events before dispatch reaches the sinks.
                const auto bindings = PoseSettings::Get();
                const bool running = Running();
                const bool gameAllowsInput = running && !DragView::InputBlockReason();
                std::lock_guard lock(inputMutex);
                const bool uiAvailable = gameAllowsInput && !mouseCaptured;
                const bool typingFingers = uiAvailable && fingerOpen.load();
                const bool typingFace = uiAvailable && faceOpen.load();
                const bool settings = uiAvailable && SettingsOpen();
                const bool typing = typingFingers || typingFace || (uiAvailable && bodyOpen.load());
                const bool typingName = typing && TypingPresetName();
                const bool panelOpen = settings || typing;
                const auto* controls = RE::ControlMap::GetSingleton();
                const auto* userEvents = RE::UserEvents::GetSingleton();
                const auto consoleKey = controls && userEvents ?
                    controls->GetMappedKey(userEvents->console, RE::INPUT_DEVICE::kKeyboard) : EditorKeys::Key::kTilde;
                bool allowConsole = false;
                bool hasInput = false;
                for (auto* event = events ? *events : nullptr; event; event = event->next) {
                    if (running && PointerEvent(*event)) {
                        mouseDelivered = true;
                        if (!mousePending) mouseCaptured = false;
                    }
                    if (const auto* motion = event->AsMouseMoveEvent(); motion && running) {
                        hasInput = true;
                        pendingInput.motion[0] += motion->mouseInputX;
                        pendingInput.motion[1] += motion->mouseInputY;
                    }
                    if (const auto* character = event->AsCharEvent(); character && typing) {
                        hasInput = true;
                        pendingInput.panelText.push_back({character->keyCode, true, false});
                    }
                    const auto* button = event->AsButtonEvent();
                    if (!button) continue;
                    const auto id = button->GetIDCode();
                    const auto device = event->GetDevice();
                    const bool keyboard = device == RE::INPUT_DEVICE::kKeyboard;
                    const bool mouse = device == RE::INPUT_DEVICE::kMouse;
                    if (!keyboard && !mouse) continue;
                    if (running && keyboard && id == consoleKey && !typingName) {
                        allowConsole = true;
                        continue;
                    }
                    hasInput = true;
                    const auto code = keyboard ? id : PoseSettings::MouseBinding(id);
                    if (settings && code && button->IsDown() && PoseSettings::RebindKey(code)) continue;

                    if (keyboard) {
                        if (panelOpen && !typingName && button->IsDown() && id == EditorKeys::Key::kEscape) {
                            CloseSettings();
                            continue;
                        }
                        if (running && !typingName && !settings && button->IsDown()) {
                            if (bindings.Matches(Action::Mirror, code)) {
                                pendingInput.toggleMirror = !pendingInput.toggleMirror;
                                continue;
                            }
                            if (bindings.Matches(Action::Undo, code)) {
                                pendingInput.actions.emplace_back(HistoryAction::Undo);
                                continue;
                            }
                            if (bindings.Matches(Action::Redo, code)) {
                                pendingInput.actions.emplace_back(HistoryAction::Redo);
                                continue;
                            }
                        }
                        if (typing) {
                            if (!button->IsHeld()) pendingInput.panelText.push_back({id, false, button->IsPressed()});
                            continue;
                        }
                        if (button->IsDown() && bindings.Matches(Action::ToggleEditor, code)) commands.store(1);
                    }
                    if (!running) continue;
                    deliveredKeys.Deliver(device, id, button->IsPressed(), button->IsDown());

                    if (mouse && panelOpen && (id == EditorKeys::Mouse::kLeftButton || id == EditorKeys::Mouse::kRightButton) &&
                        (button->IsDown() || !button->IsPressed()))
                        pendingInput.panelClicks.push_back((id == EditorKeys::Mouse::kRightButton ? 2 : 0) +
                            (button->IsPressed() ? 1 : 0));

                    if (button->IsDown()) {
                        if (mouse && (panelOpen || bindings.useWheel) && EditorKeys::WheelButton(id))
                            pendingInput.wheel += id == EditorKeys::Mouse::kWheelUp ? 1 : -1;
                        if (!panelOpen) {
                            if (bindings.Matches(Action::OpenPoser, code)) pendingInput.poserPressed = true;
                            if (!bindings.useWheel) {
                                pendingInput.wheel += int(bindings.Matches(Action::WheelUp, code)) -
                                    int(bindings.Matches(Action::WheelDown, code));
                            }
                        }
                    }

                    std::optional<EditorButton> control;
                    if (bindings.Matches(Action::ToggleGuide, code)) control = EditorButton::Guide;
                    else if (bindings.Matches(Action::ToggleSettings, code)) control = EditorButton::Settings;
                    else if (!panelOpen && bindings.Matches(Action::Nudge, code)) control = EditorButton::Nudge;
                    else if ((!panelOpen || !button->IsPressed()) && bindings.Matches(Action::Grab, code))
                        control = EditorButton::Grab;
                    else if ((!panelOpen || !button->IsPressed()) && bindings.Matches(Action::Rotate, code))
                        control = EditorButton::Rotate;
                    if (!control) continue;
                    if (button->IsDown()) pendingInput.buttons.push_back({*control, MouseEdge::Down});
                    else if (!button->IsPressed()) pendingInput.buttons.push_back({*control, MouseEdge::Up});
                }
                // Reserve input for the settings panel after overlay mods have
                // filtered their events so a newly bound key cannot open a game menu.
                return panelOpen && hasInput && !allowConsole ? RE::BSEventNotifyControl::kStop : RE::BSEventNotifyControl::kContinue;
            }
        } inputSink;

    }

    bool Running() { return active.load(std::memory_order_acquire); }
    bool RotatingHandle() { return Running() && rotationHeld.load(); }
    bool SettingsOpen() { return Running() && settingsOpen.load(); }
    void CloseSettings() { closeSettings.store(true); }
    void RequestHistory(HistoryAction action)
    {
        const std::scoped_lock lock(inputMutex);
        if (Running()) pendingInput.actions.emplace_back(action);
    }

    bool CanUseHistory(HistoryAction action) { return (historyActions.load() & HistoryBit(action)) != 0; }
    BodyPresetView ReadBodyPresets()
    {
        const std::scoped_lock lock(poseMutex);
        BodyPresetView view;
        view.status = bodyPresetStatus;
        view.entries.reserve(bodyPresets.size());
        for (const auto& preset : bodyPresets)
            view.entries.push_back({preset.name, preset.category == PosePresetJson::Category::Premade});
        return view;
    }

    void RequestBodyPreset(BodyPresetCommand command)
    {
        const std::scoped_lock lock(inputMutex);
        if (Running() && bodyOpen.load()) pendingInput.actions.emplace_back(std::move(command));
    }
    bool OwnsTextInput(const RE::ControlMap* controls)
    {
        return controls && textInputOwner.load() == controls;
    }

    void Invalidate()
    {
        generation.fetch_add(1);
        active.store(false, std::memory_order_release);
        if (threadID.load() == REX::W32::GetCurrentThreadId()) {
            ReleaseTextInput();
            FaceActor::End(RE::PlayerCharacter::GetSingleton());
        }
        commands.store(0);
        {
            std::lock_guard lock(inputMutex);
            pendingInput = {};
            deliveredKeys = {};
            mousePending = mouseDelivered = mouseCaptured = false;
        }
        {
            std::lock_guard lock(poseMutex);
            SeatBlock::End();
            active.store(false, std::memory_order_release);
            rotationHeld.store(false);
            settingsOpen.store(false);
            fingerOpen.store(false);
            faceOpen.store(false);
            bodyOpen.store(false);
            closeSettings.store(false);
            ApplyPendingEdits(true);
            SaveHistory();
            historyActions.store(0);
            pose = {};
        }
        PoseSettings::ResetMenu();
        FingerPanel::End();
        FacePanel::End();
        DragView::Clear();
    }

    void SetLoading(bool value)
    {
        loading.store(true);
        Invalidate();
        if (threadID.load() == REX::W32::GetCurrentThreadId())
            PosingCamera::End(RE::PlayerCharacter::GetSingleton());
        PosingCamera::WorldChanged();
        loading.store(value);
    }

    bool Install()
    {
        if (ready.load()) return true;
        auto* devices = RE::BSInputDeviceManager::GetSingleton();
        if (!updateInstalled || !devices) return false;
        const REL::Relocation<std::uintptr_t> copy{RELOCATION_ID(62416, 63358), REL::VariantOffset(0x267, 0x319, 0)};
        const REL::Relocation<std::uintptr_t> deferred{RELOCATION_ID(62417, 63359), REL::VariantOffset(0x66, 0x77, 0)};
        for (const auto site : {copy.address(), deferred.address()})
            if (*reinterpret_cast<const std::uint8_t*>(site) != 0xE8) return false;
            if (!DragView::Install() || !PosingCamera::Install() || !CopyHook<0>::original.InstallCall(copy.address(), CopyHook<0>::Thunk) ||
            !CopyHook<1>::original.InstallCall(deferred.address(), CopyHook<1>::Thunk)) return false;
        PoseSettings::Load();
        const auto plugins = std::filesystem::path(REL::Module::get().filePath().data()).parent_path() / L"Data/SKSE/Plugins";
        if (wristModel.Load(plugins / L"GModPose/ArmPrior.bin"))
            SKSE::log::info("Loaded absolute wrist pose model");
        else
            SKSE::log::info("Wrist model unavailable. Using the reference-pose wrist solver.");
        if (ankleModel.Load(plugins / L"GModPose/LegPrior.bin", true))
            SKSE::log::info("Loaded ankle and chest stance pose model");
        else
            SKSE::log::info("Ankle model unavailable. Using procedural leg guidance.");
        for (const auto section : {L"Hand", L"Face", L"Body"}) {
            std::error_code error;
            std::filesystem::create_directories(PresetStorage::Folder(plugins, section), error);
            if (error) SKSE::log::warn("Could not create a GModPose preset folder: {}", error.message());
        }
        FingerPanel::Load();
        FacePanel::Load();
        REL::Relocation<std::uintptr_t> mouse{RE::VTABLE_BSWin32MouseDevice[0]};
        REL::Relocation<std::uintptr_t> keyboard{RE::VTABLE_BSWin32KeyboardDevice[0]};
        REL::Relocation<std::uintptr_t> tree{RE::VTABLE_BSFlattenedBoneTree[0]};
        if (!MousePoll::original.InstallVirtual(mouse, 2, MousePoll::Thunk) ||
            !KeyboardPoll::original.InstallVirtual(keyboard, 2, KeyboardPoll::Thunk) ||
            !TailUpdateHook<0x2C>::original.InstallVirtual(tree, 0x2C, TailUpdateHook<0x2C>::Thunk) ||
            !TailUpdateHook<0x2D>::original.InstallVirtual(tree, 0x2D, TailUpdateHook<0x2D>::Thunk) ||
            !TailUpdateHook<0x2E>::original.InstallVirtual(tree, 0x2E, TailUpdateHook<0x2E>::Thunk)) return false;
        devices->PrependEventSink(&inputSink);
        ready.store(true);
        SKSE::log::info("Full skeleton posing editor ready");
        return true;
    }

    bool InstallEarlyUpdate()
    {
        if (updateInstalled) return true;
        const REL::Relocation<std::uintptr_t> site{RELOCATION_ID(35565, 36564), REL::VariantOffset(0x53, 0x6E, 0x68)};
        updateInstalled = MainHook::original.InstallCall(site.address(), MainHook::Thunk);
        if (updateInstalled) {
            LoadSharingConfig();
            if (sharingEnabled && detachSeatedPose) SeatBlock::Install();
        }
        return updateInstalled;
    }
}
