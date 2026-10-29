#include "health/AIProbe.h"

#include <format>
#include <string>
#include <utility>
#include <vector>

#include "Config.h"
#include "common/CommonUtils.h"
#include "f4sevr/PapyrusUtils.h"
#include "health/ProbeUtils.h"
#include "f4vr/CollisionLayers.h"
#include "f4vr/PlayerNodes.h"
#include "physics/HavokBridge.h"
#include "vehicle/Vehicle.h"

namespace vfvr
{
    namespace
    {
        constexpr std::uint64_t INTERVAL_MS = 2000;
        constexpr std::uint64_t AFTER_EXIT_MS = 15000;
        constexpr std::size_t MAX_LISTED = 8;
        constexpr std::uint64_t ARE_HOSTILE_ACTORS_NEAR_ID = 1053584; // ProcessLists::AreHostileActorsNear
        constexpr std::uint64_t REQUEST_DETECTION_LEVEL_ID = 943772; // Actor::RequestDetectionLevel
        constexpr std::uint64_t PROCESS_LISTS_ID = 1569706;
        constexpr float EYE_HEIGHT = 115.0f; // game units above an actor's position
        constexpr float CHEST_HEIGHT = 30.0f; // above the seated player's position
        constexpr auto COMBAT_SCRIPT = "VehicleFrameworkVR:Combat";

        // What a line-of-sight ray from `from` to `to` meets first: "clear", the vehicle, or a layer, with how far along. Actors' capsules and
        // ragdolls are passed through (the shooter's own would stop it at once).
        std::string sightTo(RE::bhkWorld* world, const RE::NiPoint3& from, const RE::NiPoint3& to, const Vehicle* vehicle, const bool releasePick)
        {
            const std::uint64_t ignore = f4vr::collisionLayerBit(RE::COL_LAYER::kCharController) | f4vr::collisionLayerBit(RE::COL_LAYER::kBiped) |
                f4vr::collisionLayerBit(RE::COL_LAYER::kBipedNoCC) | f4vr::collisionLayerBit(RE::COL_LAYER::kDeadBip);
            const auto hit = physics::castRay(world, from, to, static_cast<std::uint32_t>(RE::COL_LAYER::kLineOfSight), 0x7FFF'FFFF, ignore, releasePick);
            const float total = from.GetDistance(to) / 70.0f;
            if (!hit.hit) {
                return std::format("clear {:.0f} m", total);
            }
            const bool isVehicle = vehicle && hit.bodyId == vehicle->bodyId();
            return std::format("blocked at {:.1f} of {:.1f} m by {}", total * hit.fraction, total,
                isVehicle ? std::string("the VEHICLE") : std::format("layer '{}' body {}", f4vr::getCollisionLayerName(hit.filterInfo), hit.bodyId));
        }
    }

    namespace
    {
        // A node of the player's third person 3D by name (the bones the AI looks at), or null.
        RE::NiAVObject* findNode(RE::NiAVObject* node, const std::string_view name, const int depth = 0)
        {
            if (!node || depth > 24) {
                return nullptr;
            }
            if (const char* own = node->name.c_str(); own && name == own) {
                return node;
            }
            if (auto* asNode = node->IsNode()) {
                for (const auto& child : asNode->children) {
                    if (auto* found = findNode(child.get(), name, depth + 1)) {
                        return found;
                    }
                }
            }
            return nullptr;
        }

        std::string relative(const RE::NiAVObject* node, const RE::NiPoint3& origin)
        {
            if (!node) {
                return "missing";
            }
            const auto d = (node->world.translate - origin) / 70.0f;
            return std::format("({:.2f}, {:.2f}, {:.2f}){}", d.x, d.y, d.z, (node->flags.flags & 1u) ? " culled" : "");
        }
    }

    AIProbe& AIProbe::get()
    {
        static AIProbe instance;
        return instance;
    }

    void AIProbe::onFrameUpdate(const Vehicle* vehicle, const bool driving, const bool thirdPerson)
    {
        if (!g_config.logAIProbe) {
            return;
        }
        const auto now = common::nowMillis();
        if (driving) {
            _lastDrivingMs = now;
        }
        const bool afterExit = !driving && _lastDrivingMs != 0 && now - _lastDrivingMs < AFTER_EXIT_MS;
        if ((!driving && !afterExit) || now - _lastMs < INTERVAL_MS) {
            return;
        }
        _lastMs = now;
        static const bool available = physics::hasAddressId(ARE_HOSTILE_ACTORS_NEAR_ID) && physics::hasAddressId(REQUEST_DETECTION_LEVEL_ID) &&
            physics::hasAddressId(PROCESS_LISTS_ID);
        if (!available) {
            if (!_warned) {
                _warned = true;
                logger::warn("AIProbe: AreHostileActorsNear / RequestDetectionLevel / ProcessLists are not in the VR address library: probe off");
            }
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* lists = RE::ProcessLists::GetSingleton();
        if (!player || !lists) {
            return;
        }
        RE::BSScrapArray<RE::ActorHandle> hostiles;
        const bool any = lists->AreHostileActorsNear(&hostiles);
        const auto playerPos = player->GetPosition();
        auto* world = physics::getBhkWorld(player);
        const auto* nodes = f4vr::getVRPlayerNodes();
        const RE::NiPoint3 headTarget = nodes && nodes->hmdNode ? nodes->hmdNode->world.translate : playerPos + RE::NiPoint3(0, 0, 100.0f);
        const RE::NiPoint3 chestTarget = playerPos + RE::NiPoint3(0, 0, CHEST_HEIGHT);
        std::string listed;
        std::size_t count = 0;
        std::vector<std::pair<std::uint32_t, RE::NiPoint3>> reported; // form id, eye
        for (const auto& handle : hostiles) {
            auto actor = handle.get();
            if (!actor) {
                continue;
            }
            ++count;
            if (count > MAX_LISTED) {
                continue;
            }
            const auto pos = actor->GetPosition();
            const float meters = playerPos.GetDistance(pos) / 70.0f;
            const int detection = actor->RequestDetectionLevel(player);
            std::string sight;
            if (world) {
                const auto eye = pos + RE::NiPoint3(0, 0, EYE_HEIGHT);
                sight = std::format(", sight to the head node {}, to the chest {}", sightTo(world, eye, headTarget, vehicle, g_config.releasePickWorldReference),
                    sightTo(world, eye, chestTarget, vehicle, g_config.releasePickWorldReference));
            }
            reported.emplace_back(actor->GetFormID(), pos + RE::NiPoint3(0, 0, EYE_HEIGHT));
            listed += std::format("{}{} {:.0f} m detection {}{}", listed.empty() ? "" : "; ", probe::describeRef(actor.get()), meters, detection, sight);
        }
        logger::info("AIProbe: {} ({} person): {} hostile actor{} near{}{}", driving ? "driving" : "on foot after driving", thirdPerson ? "third" : "first", count,
            count == 1 ? "" : "s", any || count > 0 ? ": " : "", listed);
        // Where the bones the AI looks at are (third person 3D: root, head, spine, pelvis), the head node and the actor's 3D root, in metres
        // from the actor's position. Plain node reads: no virtual engine calls (0.1.102's CalculateLOSLocation is gone).
        auto* root = player->Get3D(false);
        const auto hmd = nodes && nodes->hmdNode ? nodes->hmdNode->world.translate : playerPos;
        logger::info("AIProbe: player nodes from the actor's position (m): 3D root {}, Head {}, SPINE2 {}, Pelvis {}; head node ({:.2f}, {:.2f}, {:.2f}); first person 3D {}",
            relative(root, playerPos), relative(findNode(root, "Head"), playerPos), relative(findNode(root, "SPINE2"), playerPos), relative(findNode(root, "Pelvis"), playerPos),
            (hmd.x - playerPos.x) / 70.0f, (hmd.y - playerPos.y) / 70.0f, (hmd.z - playerPos.z) / 70.0f, relative(f4vr::getFirstPersonSkeleton(), playerPos));
        if (world && root) {
            if (const auto* head = findNode(root, "Head")) {
                for (const auto& [id, eye] : reported) {
                    logger::info("AIProbe: sight from {:08X} to the Head bone: {}", id, sightTo(world, eye, head->world.translate, vehicle, g_config.releasePickWorldReference));
                }
            }
        }
        // What the engine says (combat state and target, weapon, HasDetectionLOS, IsDetectedBy): asked through Papyrus, answered in "engine says" lines.
        for (const auto& [id, eye] : reported) {
            F4SEVR::execPapyrusGlobalFunction(COMBAT_SCRIPT, "Report", static_cast<F4SEVR::SInt32>(id));
        }
    }
}
