#pragma once

#include "PresetStorage.h"
#include "PoseHistory.h"
#include "PoseMath.h"
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <type_traits>

namespace GModPose::HistoryStorage
{
    inline bool ReadQuaternion(std::istream& in, Quaternion& q)
    {
        if (!(in >> q.x >> q.y >> q.z >> q.w)) return false;
        const auto normalized = Unit(q);
        if (!normalized) return false;
        q = *normalized;
        return true;
    }

    inline void WriteQuaternion(std::ostream& out, Quaternion q)
    {
        out << ' ' << q.x << ' ' << q.y << ' ' << q.z << ' ' << q.w;
    }

    inline bool SameRotation(Quaternion a, Quaternion b)
    {
        return std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w) >= 1 - 1e-8;
    }

    template <class History, class Read>
    bool Load(const std::filesystem::path& path, const std::string& key, History& history, Read read)
    {
        std::ifstream in(path, std::ios::binary);
        in.imbue(std::locale::classic());
        std::string header;
        std::size_t count{}, cursor{};
        if (!std::getline(in, header) || header != "GModPose history 1 " + key ||
            !(in >> count >> cursor) || count == 0 || count > History::limit || cursor >= count) return false;
        std::remove_cvref_t<decltype(history.States())> states(count);
        for (auto& state : states)
            if (!read(in, state)) return false;
        in >> std::ws;
        return in.eof() && history.Restore(std::move(states), cursor);
    }

    template <class History, class Write>
    bool Save(const std::filesystem::path& path, const std::string& key, History& history, Write write)
    {
        if (!history.Dirty()) return true;
        if (!PresetStorage::Replace(path, [&](std::ostream& out) {
            out << "GModPose history 1 " << key << '\n' << history.States().size() << ' ' << history.Cursor() << '\n'
                << std::setprecision(std::numeric_limits<double>::max_digits10);
            for (const auto& state : history.States()) {
                write(out, state);
                out << '\n';
            }
        })) return false;
        history.MarkSaved();
        return true;
    }
}
