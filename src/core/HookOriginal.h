#pragma once

#include <SKSE/SKSE.h>
#include <atomic>
#include <cstring>
#include <utility>

namespace GModPose
{
    template <class Function>
    class HookOriginal
    {
    public:
        bool InstallVirtual(REL::Relocation<std::uintptr_t>& table, std::size_t index, Function* hook)
        {
            if (target.load()) return true;
            const auto slot = table.address() + index * sizeof(std::uintptr_t);
            const auto address = *reinterpret_cast<const std::uintptr_t*>(slot);
            if (!address) return false;
            // Skyrim can enter the hook from another thread as soon as the slot is patched.
            target.store(reinterpret_cast<Function*>(address));
            table.write_vfunc(index, hook);
            return true;
        }

        bool InstallCall(std::uintptr_t site, Function* hook)
        {
            if (target.load()) return true;
            if (*reinterpret_cast<const std::uint8_t*>(site) != 0xE8) return false;
            std::int32_t offset;
            std::memcpy(&offset, reinterpret_cast<const void*>(site + 1), sizeof(offset));
            target.store(reinterpret_cast<Function*>(site + 5 + offset));
            SKSE::GetTrampoline().write_call<5>(site, hook);
            return true;
        }

        template <class... Args>
        decltype(auto) operator()(Args&&... args) const
        {
            return target.load()(std::forward<Args>(args)...);
        }

    private:
        std::atomic<Function*> target{nullptr};
    };
}
