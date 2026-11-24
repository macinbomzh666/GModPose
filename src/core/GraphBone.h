#pragma once
#include <RE/Skyrim.h>

namespace GModPose
{
    // A graph entry may reference a scene node or a slot in a flattened tree.
    // Scene parents between two graph entries need not have an entry themselves.
    struct BoneRef
    {
        RE::NiAVObject* node{};
        std::int32_t index{-1};
        bool operator==(const BoneRef&) const = default;

        auto* Slot() const
        {
            using SlotType = RE::BSFlattenedBoneTree::BoneEntry;
            if (!node || index < 0) return static_cast<SlotType*>(nullptr);
            auto& tree = static_cast<RE::BSFlattenedBoneTree*>(node)->GetRuntimeData();
            if (!tree.boneEntries || static_cast<std::uint32_t>(index) >= tree.numBones)
                return static_cast<SlotType*>(nullptr);
            return &tree.boneEntries[index];
        }

        BoneRef Resolve() const
        {
            if (index < 0) return *this;
            const auto* slot = Slot();
            if (!slot) return {};
            return slot->node ? BoneRef{slot->node} : *this;
        }

        BoneRef Parent() const
        {
            if (const auto* slot = Slot()) {
                if (slot->parentIndex >= 0) return BoneRef{node, slot->parentIndex}.Resolve();
                return {node};
            }
            return node && index < 0 ? BoneRef{node->parent} : BoneRef{};
        }

        RE::NiTransform* Local() const
        {
            if (auto* slot = Slot()) return &slot->local;
            return node && index < 0 ? &node->local : nullptr;
        }

        const RE::NiTransform* World() const
        {
            if (auto* slot = Slot()) return &slot->world;
            return node && index < 0 ? &node->world : nullptr;
        }

        std::string_view Name() const
        {
            if (const auto* slot = Slot()) {
                if (const auto* name = slot->nodeName.c_str()) return name;
            } else if (node && index < 0) {
                if (const auto* name = node->name.c_str()) return name;
            }
            return {};
        }
    };

    struct BoneTransforms
    {
        RE::NiTransform* local{};
        const RE::NiTransform* world{};
        const RE::NiTransform* parentWorld{};
    };

    inline BoneRef GraphBoneRef(const RE::BShkbAnimationGraph::BoneNodeEntry& entry)
    {
        return BoneRef{entry.node, static_cast<std::int32_t>(entry.unk08)}.Resolve();
    }

    inline BoneTransforms GraphBone(const RE::BShkbAnimationGraph::BoneNodeEntry& entry)
    {
        const auto bone = GraphBoneRef(entry);
        return {bone.Local(), bone.World(), bone.Parent().World()};
    }
}
