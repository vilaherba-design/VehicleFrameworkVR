#pragma once

// Trunk capacity: which of the items just put in the trunk go back to the player when the trunk holds more weight than it may.
// Pure logic (no game types), shared by the plugin and the offline tests.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace vfvr::model
{
    // Items put in the trunk while it was open, oldest first.
    struct TrunkAddition
    {
        std::uint32_t formId = 0; // base object
        std::int32_t count = 0;
        float unitWeight = 0.0f;
    };

    struct TrunkReturn
    {
        std::uint32_t formId = 0;
        std::int32_t count = 0;
    };

    // Record `count` of `formId` going into (count > 0) or out of (count < 0) the trunk. Taking an item out undoes the latest additions of it first.
    inline void recordTrunkChange(std::vector<TrunkAddition>& added, const std::uint32_t formId, const std::int32_t count, const float unitWeight)
    {
        if (count > 0) {
            added.push_back({ formId, count, unitWeight });
            return;
        }
        std::int32_t left = -count;
        for (auto it = added.rbegin(); it != added.rend() && left > 0; ++it) {
            if (it->formId != formId) {
                continue;
            }
            const std::int32_t n = (std::min)(left, it->count);
            it->count -= n;
            left -= n;
        }
        std::erase_if(added, [](const TrunkAddition& a) { return a.count <= 0; });
    }

    // The fewest of the latest additions that bring `weight` down to `capacity`, newest first. Weightless items never go back. If what was added cannot
    // cover the excess (the capacity was lowered), everything added goes back and the rest stays.
    inline std::vector<TrunkReturn> trunkReturns(const std::vector<TrunkAddition>& added, const float weight, const float capacity)
    {
        std::vector<TrunkReturn> out;
        float excess = weight - capacity;
        for (auto it = added.rbegin(); it != added.rend() && excess > 1e-4f; ++it) {
            if (it->count <= 0 || !(it->unitWeight > 0.0f)) {
                continue;
            }
            const auto needed = static_cast<std::int32_t>(std::ceil(excess / it->unitWeight - 1e-4f));
            const std::int32_t n = std::clamp(needed, 1, it->count);
            out.push_back({ it->formId, n });
            excess -= static_cast<float>(n) * it->unitWeight;
        }
        return out;
    }
}
