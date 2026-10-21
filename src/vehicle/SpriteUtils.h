#pragma once

#include <cmath>
#include <exception>
#include <string>

#include "common/CommonUtils.h"
#include "f4vr/PlayerNodes.h"

namespace vfvr
{
    // Helpers of the sprite effects (the exhaust and the damage smoke): puff references turned toward the player's head.

    // The player's head in the world (the HMD node; the player's position without it). [game]
    inline RE::NiPoint3 hmdPosition()
    {
        const auto* nodes = f4vr::getVRPlayerNodes();
        return nodes && nodes->hmdNode ? nodes->hmdNode->world.translate : RE::PlayerCharacter::GetSingleton()->GetPosition();
    }

    // Turns a smoke sprite (its front is local -Y, x to the right, z up) so that its front faces `viewer`, then rolls it about the line of sight. The
    // engine's rotation matrices hold the local axes in world as rows.
    inline void faceViewer(RE::NiMatrix3& out, const RE::NiPoint3& at, const RE::NiPoint3& viewer, const float roll)
    {
        float fx = viewer.x - at.x;
        float fy = viewer.y - at.y;
        float fz = viewer.z - at.z;
        const float len = std::sqrt(fx * fx + fy * fy + fz * fz);
        if (len < 1e-3f) {
            fx = 0.0f;
            fy = -1.0f;
            fz = 0.0f;
        } else {
            fx /= len;
            fy /= len;
            fz /= len;
        }
        // right = up x toViewer (the horizontal); local +Y points away from the viewer; up' = right x forward
        float rx = -fy;
        float ry = fx;
        const float rl = std::sqrt(rx * rx + ry * ry);
        if (rl < 1e-4f) {
            rx = 1.0f;
            ry = 0.0f;
        } else {
            rx /= rl;
            ry /= rl;
        }
        const float bx = -fx;
        const float by = -fy;
        const float bz = -fz;
        const float ux = ry * bz;
        const float uy = -rx * bz;
        const float uz = rx * by - ry * bx;
        const float c = std::cos(roll);
        const float s = std::sin(roll);
        out.entry[0][0] = c * rx + s * ux;
        out.entry[0][1] = c * ry + s * uy;
        out.entry[0][2] = s * uz;
        out.entry[1][0] = bx;
        out.entry[1][1] = by;
        out.entry[1][2] = bz;
        out.entry[2][0] = -s * rx + c * ux;
        out.entry[2][1] = -s * ry + c * uy;
        out.entry[2][2] = c * uz;
    }

    // "Plugin.esm|0x000A9774" -> the base object. [game]
    inline RE::TESBoundObject* lookupBaseObject(const std::string& spec)
    {
        const auto bar = spec.find('|');
        if (bar == std::string::npos) {
            return nullptr;
        }
        const auto plugin = common::trim(spec.substr(0, bar));
        RE::TESFormID id;
        try {
            id = static_cast<RE::TESFormID>(std::stoul(common::trim(spec.substr(bar + 1)), nullptr, 16) & 0x00FF'FFFF);
        } catch (const std::exception&) {
            return nullptr;
        }
        auto* handler = RE::TESDataHandler::GetSingleton();
        auto* form = handler ? handler->LookupForm(id, plugin) : nullptr;
        return form && form->IsBoundObject() ? static_cast<RE::TESBoundObject*>(form) : nullptr;
    }
}
