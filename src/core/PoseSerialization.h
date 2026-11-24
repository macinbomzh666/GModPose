// SPDX-License-Identifier: MIT
#pragma once

#include "FacePanel.h"
#include "FingerPose.h"
#include "PosePresetJson.h"

#include <fstream>

namespace GModPose::PoseSerialization
{
    using namespace PosePresetJson;

    template <class Pose>
    struct Preset
    {
        std::string name;
        Pose pose;
        std::filesystem::path path;
        Category category = Category::Custom;
        std::string source;
    };

    struct FacePoseValue { FacePose::State pose; FacePose::Mask mask{}; };
    using FacePreset = Preset<FacePoseValue>;
    using HandPreset = Preset<Fingers::Pose>;

    inline bool DecodeFacePose(const Json& object, FacePoseValue& value)
    {
        static_assert(FacePose::weightCount == channels.size());
        static_assert(FacePose::moodCount == moods.size());
        Face input;
        if (!DecodeFace(object, input)) return false;
        value = {};
        if (input.mood) {
            value.pose.mood = input.mood->index + FacePose::neutralMood;
            value.pose.strength = input.mood->strength;
            value.mask |= FacePose::Bit(FacePose::MoodSlot);
        }
        for (std::size_t i = 0; i < input.weights.size(); ++i) {
            if (!input.weights[i]) continue;
            value.pose.weights[i] = *input.weights[i];
            value.mask |= FacePose::Mask{1} << i;
        }
        return true;
    }

    inline Json EncodeFacePose(const FacePoseValue& value)
    {
        Face face;
        if (value.mask & FacePose::Bit(FacePose::MoodSlot)) {
            face.mood = Mood{value.pose.mood - FacePose::neutralMood, value.pose.strength};
        }
        for (std::size_t i = 0; i < face.weights.size(); ++i) {
            if (value.mask & (FacePose::Mask{1} << i)) face.weights[i] = value.pose.weights[i];
        }
        return EncodeFace(face);
    }

    inline bool DecodeLegacyFace(std::istream& in, FacePoseValue& value)
    {
        const auto valid = [](float weight) { return std::isfinite(weight) && weight >= 0 && weight <= 1; };
        auto& pose = value.pose;
        value.mask = FacePose::all;
        return (in >> pose.mood >> pose.strength) && pose.mood >= FacePose::neutralMood &&
            pose.mood < FacePose::neutralMood + FacePose::moodCount && valid(pose.strength) &&
            std::ranges::all_of(pose.weights, [&](float& weight) { return (in >> weight) && valid(weight); });
    }

    inline Json Rotation(Quaternion q) { return Json::array({q.x, q.y, q.z, q.w}); }

    inline bool DecodeRotation(const Json& object, Quaternion& q)
    {
        Quaternion value;
        if (!object.is_array() || object.size() != 4 ||
            !Number(object[0], value.x, -1e7, 1e7) || !Number(object[1], value.y, -1e7, 1e7) ||
            !Number(object[2], value.z, -1e7, 1e7) || !Number(object[3], value.w, -1e7, 1e7)) return false;
        const auto unit = Unit(value);
        if (!unit) return false;
        q = *unit;
        return true;
    }

    constexpr std::array digits{"thumb", "index", "middle", "ring", "little"};
    static_assert(digits.size() == Fingers::digitCount);

    inline Json EncodeHand(const Fingers::Pose& pose)
    {
        auto fingers = Json::object();
        std::size_t joint = 0;
        for (const auto digit : digits) {
            auto rotations = Json::array();
            for (std::size_t i = 0; i < Fingers::jointsPerDigit; ++i) rotations.push_back(Rotation(pose[joint++]));
            fingers[digit] = std::move(rotations);
        }
        return {{"fingers", std::move(fingers)}};
    }

    inline bool DecodeHand(const Json& object, Fingers::Pose& pose)
    {
        const auto& fingers = Field(object, "fingers");
        if (!fingers.is_object() || fingers.size() != digits.size()) return false;
        std::size_t joint = 0;
        for (const auto digit : digits) {
            const auto& rotations = Field(fingers, digit);
            if (!rotations.is_array() || rotations.size() != Fingers::jointsPerDigit) return false;
            for (const auto& rotation : rotations) {
                if (!DecodeRotation(rotation, pose[joint++])) return false;
            }
        }
        return true;
    }

    inline bool DecodeLegacyHand(std::istream& in, Fingers::Pose& pose)
    {
        for (auto& q : pose) {
            Quaternion value;
            if (!(in >> value.x >> value.y >> value.z >> value.w)) return false;
            const auto normalized = Unit(value);
            if (!normalized) return false;
            q = *normalized;
        }
        return true;
    }

    inline Json Point(Vec3 value) { return Json::array({value.x, value.y, value.z}); }

    inline bool DecodePoint(const Json& object, Vec3& value)
    {
        return object.is_array() && object.size() == 3 && Number(object[0], value.x, -1e7, 1e7) &&
            Number(object[1], value.y, -1e7, 1e7) && Number(object[2], value.z, -1e7, 1e7);
    }

    inline Json ScaledTransform(const Transform& value, float scale)
    {
        return {{"position", Point(value.position)}, {"rotation", Rotation(value.rotation)}, {"scale", scale}};
    }

    inline bool DecodeTransform(const Json& object, Transform& transform, float& scale)
    {
        return DecodePoint(Field(object, "position"), transform.position) &&
            DecodeRotation(Field(object, "rotation"), transform.rotation) &&
            Number(Field(object, "scale"), scale, 0, 100) && scale > 0;
    }

    inline Json EncodeFaceSnapshot(const FaceActor::Sample& face)
    {
        return {{"supported", face.supported}, {"pose", EncodeFacePose({face.pose, FacePose::all})},
            {"expressions", face.expressions}};
    }

    inline bool DecodeFaceSnapshot(const Json& object, FaceActor::Sample& face)
    {
        const auto& mask = Field(object, "supported");
        const auto& expressions = Field(object, "expressions");
        FacePoseValue value;
        if (!mask.is_number_integer() || !Number(mask, face.supported, 0, FacePose::all) ||
            !DecodeFacePose(Field(object, "pose"), value) || value.mask != FacePose::all ||
            !expressions.is_array() || expressions.size() != face.expressions.size()) return false;
        face.pose = value.pose;
        for (std::size_t i = 0; i < face.expressions.size(); ++i) {
            if (!Number(expressions[i], face.expressions[i], 0, 1)) return false;
        }
        return true;
    }

    template <class Frame>
    Json EncodeBody(const Frame& frame, std::string_view rig)
    {
        auto bones = Json::array();
        for (const auto& bone : frame.bones) {
            Json value{{"local", ScaledTransform(bone.local, bone.scale)}, {"manual", bone.manual}};
            if (bone.root) value["root"] = ScaledTransform(*bone.root, bone.rootScale);
            bones.push_back(std::move(value));
        }
        Json result{{"rig", rig}, {"translation", Point(frame.translation)}, {"bones", std::move(bones)},
            {"twist_phases", frame.twistPhases}};
        if (frame.face) result["face"] = EncodeFaceSnapshot(*frame.face);
        return result;
    }

    template <class Frame, class Rooted>
    bool DecodeBody(const Json& object, Frame& frame, std::string_view rig, std::size_t boneCount, Rooted rooted)
    {
        const auto& bones = Field(object, "bones");
        const auto& twists = Field(object, "twist_phases");
        if (!StringIs(Field(object, "rig"), rig) || !DecodePoint(Field(object, "translation"), frame.translation) ||
            !bones.is_array() || bones.size() != boneCount || !twists.is_array() ||
            twists.size() != frame.twistPhases.size()) return false;
        frame.bones.resize(boneCount);
        for (std::size_t i = 0; i < boneCount; ++i) {
            const auto& item = bones[i];
            auto& bone = frame.bones[i];
            const auto& manual = Field(item, "manual");
            if (!DecodeTransform(Field(item, "local"), bone.local, bone.scale) ||
                !manual.is_boolean() || item.contains("root") != rooted(i)) return false;
            bone.manual = manual.get<bool>();
            if (rooted(i) && !DecodeTransform(item["root"], bone.root.emplace(), bone.rootScale)) return false;
        }
        for (std::size_t i = 0; i < frame.twistPhases.size(); ++i) {
            if (!Number(twists[i], frame.twistPhases[i], -1e7, 1e7)) return false;
        }
        return !object.contains("face") || DecodeFaceSnapshot(object["face"], frame.face.emplace());
    }

    template <class Entry, class Decode>
    std::vector<Entry> ReadFolder(const std::filesystem::path& folder, std::string_view type, std::size_t limit, Decode decode)
    {
        std::vector<Entry> entries;
        const auto same = [&](std::string_view left, std::string_view right) {
            const auto lower = [](unsigned char ch) { return ch >= 'A' && ch <= 'Z' ? ch + 'a' - 'A' : ch; };
            return std::ranges::equal(left, right, {}, lower, lower);
        };
        for (const auto& path : Files(folder)) {
            if (entries.size() >= limit) break;
            std::ifstream input(path, std::ios::binary);
            const auto document = Read(input);
            Entry entry{};
            if (!Header(document, type, entry.name, entry.category) ||
                std::ranges::any_of(entries, [&](const auto& other) { return same(entry.name, other.name); }) ||
                !decode(Field(document, "pose"), entry)) continue;
            if (document.contains("source")) entry.source = document["source"].get<std::string>();
            entry.path = path;
            entries.push_back(std::move(entry));
        }
        return entries;
    }
}
