#pragma once

#include <Windows.h>
#include "PosePresetJson.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace GModPose::PresetStorage
{
    inline std::string Fold(std::string_view value)
    {
        std::string key(value);
        for (char& ch : key)
            if (ch >= 'A' && ch <= 'Z') ch += 'a' - 'A';
        return key;
    }

    inline bool ValidName(std::string_view name)
    {
        return PosePresetJson::ValidName(name);
    }

    template <class Entries>
    auto FindName(Entries& entries, std::string_view name)
    {
        const auto key = Fold(name);
        return std::ranges::find_if(entries, [&](const auto& entry) { return Fold(entry.name) == key; });
    }

    template <class Write>
    bool Replace(const std::filesystem::path& file, Write write)
    {
        if (file.empty()) return false;
        // Close the complete write before replacing the existing pose file.
        auto temporary = file;
        temporary += ".tmp";
        {
            std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
            out.imbue(std::locale::classic());
            write(out);
            out.close();
            if (!out) return false;
        }
        if (::MoveFileExW(temporary.c_str(), file.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
            return true;
        std::error_code error;
        std::filesystem::remove(temporary, error);
        return false;
    }

    inline std::filesystem::path Folder(const std::filesystem::path& plugins, const wchar_t* section)
    {
        return plugins / L"GModPose" / L"Presets" / section;
    }

    inline std::string_view TrimName(std::string_view name)
    {
        const auto first = name.find_first_not_of(' ');
        return first == name.npos ? std::string_view{} : name.substr(first, name.find_last_not_of(' ') - first + 1);
    }

    inline std::optional<std::filesystem::path> NewFile(const std::filesystem::path& folder, std::string_view name)
    {
        std::error_code error;
        std::filesystem::create_directories(folder, error);
        if (error) return std::nullopt;
        std::string stem = "pose-";
        for (const unsigned char letter : Fold(name)) {
            if ((letter >= 'a' && letter <= 'z') || (letter >= '0' && letter <= '9')) stem += char(letter);
            else if (stem.back() != '-') stem += '-';
        }
        for (unsigned variant = 0; variant < 10000; ++variant) {
            const auto suffix = variant ? "-" + std::to_string(variant + 1) : std::string{};
            const auto path = folder / (stem + suffix + ".json");
            if (!std::filesystem::exists(path, error)) return error ? std::nullopt : std::optional{path};
        }
        return std::nullopt;
    }

    inline bool WriteJson(const std::filesystem::path& path, const PosePresetJson::Json& document)
    {
        // A legacy file may contain a label in a Windows code page instead of UTF-8.
        // Leave that file available if it cannot be represented in the JSON preset.
        std::string encoded;
        try {
            encoded = document.dump(2);
        } catch (const PosePresetJson::Json::type_error&) {
            return false;
        }
        return Replace(path, [&](std::ostream& out) { out << encoded << '\n'; });
    }

    inline std::filesystem::path SharedFaceFolder(const std::filesystem::path& plugins)
    {
        return plugins / L"PosePresets" / L"Face";
    }

    template <class Entry, class Decode>
    std::vector<Entry> ReadLegacyFolder(const std::filesystem::path& folder, std::string_view header, std::size_t limit, Decode decode)
    {
        std::vector<Entry> entries;
        for (const auto& path : PosePresetJson::Files(folder, ".gmodpose")) {
            std::ifstream input(path, std::ios::binary);
            input.imbue(std::locale::classic());
            std::string format;
            if (!std::getline(input, format)) continue;
            if (!format.empty() && format.back() == '\r') format.pop_back();
            if (format != header) continue;
            Entry entry;
            input >> std::ws;
            if (input.peek() != '"' || !(input >> std::quoted(entry.name)) || !ValidName(entry.name) ||
                FindName(entries, entry.name) != entries.end() || !decode(input, entry)) continue;
            input >> std::ws;
            if (!input.eof()) continue;
            entry.path = path;
            entries.push_back(std::move(entry));
            if (entries.size() >= limit) break;
        }
        return entries;
    }

    inline void ArchiveLegacyFile(const std::filesystem::path& old)
    {
        // Keep the text source after conversion. Move it out of the loadable set.
        for (unsigned index = 0; index < 1000; ++index) {
            auto backup = old;
            backup += index == 0 ? ".bak" : ".bak." + std::to_string(index);
            std::error_code error;
            if (std::filesystem::exists(backup, error)) continue;
            if (!error) ::MoveFileExW(old.c_str(), backup.c_str(), MOVEFILE_WRITE_THROUGH);
            break;
        }
    }

    template <class Entries, class Write>
    void MigrateFiles(const std::filesystem::path& folder, Entries& current, Entries legacy,
        std::size_t limit, Write write)
    {
        for (auto& entry : legacy) {
            if (FindName(current, entry.name) != current.end()) {
                ArchiveLegacyFile(entry.path);
                continue;
            }
            if (current.size() >= limit) break;
            const auto path = NewFile(folder, entry.name);
            if (path && write(*path, entry)) {
                ArchiveLegacyFile(entry.path);
                entry.path = *path;
            }
            current.push_back(std::move(entry));
        }
    }

    template <class Entries, class Write>
    void MigrateLegacy(const std::filesystem::path& folder, const std::filesystem::path& marker,
        Entries& current, Entries legacy, std::size_t limit, Write write)
    {
        // Retry incomplete migrations on the next open. Existing names take
        // precedence over data in the old combined preset file.
        bool complete = true;
        for (auto& entry : legacy) {
            if (FindName(current, entry.name) != current.end()) continue;
            if (current.size() >= limit) {
                complete = false;
                break;
            }
            const auto path = NewFile(folder, entry.name);
            if (path && write(*path, entry)) entry.path = *path;
            else complete = false;
            current.push_back(std::move(entry));
        }
        if (complete && !legacy.empty())
            Replace(marker, [](std::ostream& out) { out << "1\n"; });
    }
}
