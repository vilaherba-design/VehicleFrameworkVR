#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

namespace vfvr::model
{
    // Splits a comma separated list of profile ids ("sedan, pickup") into trimmed, non-empty ids.
    inline std::vector<std::string> splitProfileList(const std::string& list)
    {
        std::vector<std::string> ids;
        std::string cur;
        auto flush = [&] {
            const auto b = cur.find_first_not_of(" \t");
            const auto e = cur.find_last_not_of(" \t");
            if (b != std::string::npos) {
                ids.push_back(cur.substr(b, e - b + 1));
            }
            cur.clear();
        };
        for (const char c : list) {
            if (c == ',') {
                flush();
            } else {
                cur += c;
            }
        }
        flush();
        return ids;
    }

    // The user's profile list with every id of the shipped default list it lacks appended, in the default's order. The user's ids, their order
    // and any id the defaults no longer have are kept (ids compare without case). Used when an older INI is migrated: the user's list is kept,
    // but a vehicle added in the new version must still reach the menu.
    inline std::string mergeProfileLists(const std::string& user, const std::string& defaults)
    {
        auto ids = splitProfileList(user);
        auto lower = [](std::string s) {
            std::transform(s.begin(), s.end(), s.begin(), [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return s;
        };
        for (const auto& d : splitProfileList(defaults)) {
            const bool present = std::any_of(ids.begin(), ids.end(), [&](const std::string& id) { return lower(id) == lower(d); });
            if (!present) {
                ids.push_back(d);
            }
        }
        std::string out;
        for (const auto& id : ids) {
            out += (out.empty() ? "" : ", ") + id;
        }
        return out;
    }
}
