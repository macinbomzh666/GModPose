#include <RE/Skyrim.h>
#include <REX/W32/KERNEL32.h>
#include <SKSE/SKSE.h>

#include "HookOriginal.h"
#include "SeatBlock.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <utility>

namespace GModPose::SeatBlock
{
    namespace
    {
        using Occupied = bool(const RE::TESObjectREFR*, std::uint32_t, bool);
        using Claim = bool(RE::TESObjectREFR*, std::uint32_t, RE::Actor*, bool, bool);
        using ReadOwner = RE::ActorHandle*(const RE::TESObjectREFR*, RE::ActorHandle*, std::uint32_t, bool);

        // Skyrim passes this context to its furniture marker iteration.
        struct Selection
        {
            const RE::Actor* requester;
            const RE::TESObjectREFR* furniture;
            std::int32_t result;
            const RE::BSExtraData* reservedMarkers;
        };
        static_assert(offsetof(Selection, result) == 0x10 && offsetof(Selection, reservedMarkers) == 0x18);

        Occupied* originalOccupied{};
        Claim* originalClaim{};
        ReadOwner* readOwner{};
        HookOriginal<void(const RE::TESFurniture*, Selection*)> originalSelect;
        HookOriginal<bool(const RE::TESFurniture*, std::uint32_t)> originalMarkerEnabled;
        thread_local const Selection* currentSelection{};

        // Publish a single furniture/marker pair for queries on AI threads.
        std::atomic_uint64_t blockedMarker{};
        bool installed{};

        std::uint64_t MarkerKey(const RE::TESObjectREFR& furniture, std::uint32_t marker)
        {
            return (static_cast<std::uint64_t>(furniture.GetFormID()) << 32) | marker;
        }

        bool Blocked(const RE::TESObjectREFR* furniture, std::uint32_t marker)
        {
            const auto key = blockedMarker.load();
            return key && furniture && key == MarkerKey(*furniture, marker);
        }

        bool IsOccupied(const RE::TESObjectREFR* furniture, std::uint32_t marker, bool ignoreReservations)
        {
            return Blocked(furniture, marker) || originalOccupied(furniture, marker, ignoreReservations);
        }

        bool ClaimMarker(RE::TESObjectREFR* furniture, std::uint32_t marker, RE::Actor* actor,
            bool reserve, bool replace)
        {
            // AIProcess::ClearFurniture also calls this routine with reserve=false.
            if (reserve && Blocked(furniture, marker) && actor != RE::PlayerCharacter::GetSingleton()) return false;
            return originalClaim(furniture, marker, actor, reserve, replace);
        }

        void SelectMarker(const RE::TESFurniture* furniture, Selection* selection)
        {
            // The native iteration calls MarkerEnabled synchronously on this thread.
            const auto* previousSelection = std::exchange(currentSelection, selection);
            originalSelect(furniture, selection);
            currentSelection = previousSelection;
        }

        bool MarkerEnabled(const RE::TESFurniture* furniture, std::uint32_t marker)
        {
            if (currentSelection && Blocked(currentSelection->furniture, marker) &&
                currentSelection->requester != RE::PlayerCharacter::GetSingleton()) return false;
            return originalMarkerEnabled(furniture, marker);
        }

        // Four register pushes with no address-relative operands, ending at +6.
        // Both verified function entries start with the same complete instructions.
        constexpr std::array<std::uint8_t, 6> nativePrologue{0x40, 0x55, 0x56, 0x57, 0x41, 0x54};
        constexpr std::array<std::uint8_t, 6> absoluteJump{0xFF, 0x25, 0, 0, 0, 0};

        template <class Function>
        void InstallEntry(std::uintptr_t entry, Function* hook, Function*& original)
        {
            auto& trampoline = SKSE::GetTrampoline();
            auto* gateway = static_cast<std::uint8_t*>(trampoline.allocate(
                nativePrologue.size() + absoluteJump.size() + sizeof(entry)));
            const auto resume = entry + nativePrologue.size();
            std::memcpy(gateway, nativePrologue.data(), nativePrologue.size());
            std::memcpy(gateway + nativePrologue.size(), absoluteJump.data(), absoluteJump.size());
            std::memcpy(gateway + nativePrologue.size() + absoluteJump.size(), &resume, sizeof(resume));
            REX::W32::FlushInstructionCache(REX::W32::GetCurrentProcess(), gateway,
                nativePrologue.size() + absoluteJump.size() + sizeof(resume));
            original = reinterpret_cast<Function*>(gateway);
            trampoline.write_branch<6>(entry, hook);
            REX::W32::FlushInstructionCache(REX::W32::GetCurrentProcess(), reinterpret_cast<const void*>(entry),
                nativePrologue.size());
        }

        bool InText(std::uintptr_t address, std::size_t length)
        {
            const auto text = REL::Module::get().segment(REL::Segment::textx);
            return address >= text.address() && length <= text.size() &&
                address - text.address() <= text.size() - length;
        }

        template <std::size_t N>
        bool Matches(std::uintptr_t address, const std::array<std::uint8_t, N>& bytes)
        {
            return InText(address, N) && std::memcmp(reinterpret_cast<const void*>(address), bytes.data(), N) == 0;
        }

        std::uintptr_t CallTarget(std::uintptr_t site)
        {
            if (!InText(site, 5) || *reinterpret_cast<const std::uint8_t*>(site) != 0xE8) return 0;
            std::int32_t displacement;
            std::memcpy(&displacement, reinterpret_cast<const void*>(site + 1), sizeof(displacement));
            return site + 5 + displacement;
        }
    }

    void Install()
    {
        if (installed) return;
        if (!REL::Module::IsAE()) {
            SKSE::log::info("Temporary furniture marker blocking is unavailable on this Skyrim runtime");
            return;
        }
        const auto occupied = REL::ID(20157).address();
        const auto owner = REL::ID(20158).address();
        const auto claim = REL::ID(20159).address();
        const auto select = REL::ID(20162).address() + 0xA4;
        const auto iteration = REL::ID(20283).address();
        const auto enabled = iteration + 0x45;
        // Check the selection context and marker predicate as well as the entry
        // instructions. Call displacements differ even when the ABI is identical.
        constexpr std::array<std::uint8_t, 16> selectionContext{0x84, 0xFF, 0x49, 0x0F, 0x44, 0xFE,
            0x48, 0x89, 0x7C, 0x24, 0x58, 0x48, 0x8D, 0x54, 0x24, 0x40};
        constexpr std::array<std::uint8_t, 13> markerPredicate{0x45, 0x32, 0xC0, 0x83, 0xFA, 0x19, 0x73,
            0x1B, 0x8B, 0x81, 0xE4, 0x00, 0x00};
        const auto testMarker = CallTarget(enabled);
        if (!Matches(occupied, nativePrologue) || !Matches(owner, nativePrologue) || !Matches(claim, nativePrologue) ||
            !Matches(select - selectionContext.size(), selectionContext) || CallTarget(select) != iteration ||
            !Matches(testMarker, markerPredicate)) {
            SKSE::log::warn("Furniture hooks unavailable: Skyrim's marker code differs from the verified build");
            return;
        }

        if (!originalMarkerEnabled.InstallCall(enabled, MarkerEnabled) ||
            !originalSelect.InstallCall(select, SelectMarker)) return;
        InstallEntry(occupied, IsOccupied, originalOccupied);
        InstallEntry(claim, ClaimMarker, originalClaim);
        readOwner = reinterpret_cast<ReadOwner*>(owner);
        installed = true;
        SKSE::log::info("Temporary furniture marker blocking is available");
    }

    void Begin(const RE::TESObjectREFR& furniture, std::uint32_t marker)
    {
        if (installed && furniture.GetFormID() && marker < 25) blockedMarker.store(MarkerKey(furniture, marker));
    }

    void End()
    {
        blockedMarker.store(0);
    }

    bool TakenByOther(const RE::TESObjectREFR& furniture, std::uint32_t marker, RE::Actor& player)
    {
        const auto actor = player.GetHandle();
        if (readOwner) {
            RE::ActorHandle owner;
            readOwner(&furniture, &owner, marker, true);
            return owner && owner != actor;
        }
        const auto* markers = furniture.extraList.GetByType<RE::ExtraUsedMarkers>();
        return markers && std::ranges::any_of(markers->usedMarkers, [&](const auto& entry) {
            return entry.markerID == marker && entry.actorinMarker != actor;
        });
    }
}
