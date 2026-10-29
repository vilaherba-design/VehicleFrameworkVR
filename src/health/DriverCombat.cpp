#include "health/DriverCombat.h"

#include "Config.h"
#include "common/CommonUtils.h"
#include "f4sevr/PapyrusUtils.h"
#include "health/ProbeUtils.h"
#include "physics/HavokBridge.h"

namespace vfvr
{
    namespace
    {
        constexpr std::uint64_t INTERVAL_MS = 1000;
        constexpr std::uint64_t PER_ACTOR_MS = 5000;
        constexpr float MAX_DISTANCE = 60.0f * 70.0f; // game units
        constexpr auto SCRIPT = "VehicleFrameworkVR:Combat";
        constexpr auto FUNCTION = "Engage";
        constexpr std::uint64_t ARE_HOSTILE_ACTORS_NEAR_ID = 1053584;
        constexpr std::uint64_t REQUEST_DETECTION_LEVEL_ID = 943772;
        constexpr std::uint64_t PROCESS_LISTS_ID = 1569706;
    }

    DriverCombat& DriverCombat::get()
    {
        static DriverCombat instance;
        return instance;
    }

    void DriverCombat::onFrameUpdate(const bool driving)
    {
        if (!driving || !g_config.engageWhileDriving) {
            if (!_engagedMs.empty()) {
                _engagedMs.clear();
            }
            return;
        }
        const auto now = common::nowMillis();
        if (now - _lastMs < INTERVAL_MS) {
            return;
        }
        _lastMs = now;
        static const bool available = physics::hasAddressId(ARE_HOSTILE_ACTORS_NEAR_ID) && physics::hasAddressId(REQUEST_DETECTION_LEVEL_ID) &&
            physics::hasAddressId(PROCESS_LISTS_ID);
        if (!available) {
            if (!_warned) {
                _warned = true;
                logger::warn("DriverCombat: the hostile list, the detection call or the Papyrus VM is not available: enemies are not engaged");
            }
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* lists = RE::ProcessLists::GetSingleton();
        if (!player || !lists) {
            return;
        }
        RE::BSScrapArray<RE::ActorHandle> hostiles;
        lists->AreHostileActorsNear(&hostiles);
        const auto playerPos = player->GetPosition();
        for (const auto& handle : hostiles) {
            auto actor = handle.get();
            if (!actor || playerPos.GetDistance(actor->GetPosition()) > MAX_DISTANCE) {
                continue;
            }
            const auto id = actor->GetFormID();
            if (const auto it = _engagedMs.find(id); it != _engagedMs.end() && now - it->second < PER_ACTOR_MS) {
                continue;
            }
            const int detection = actor->RequestDetectionLevel(player);
            if (detection <= 0) {
                continue;
            }
            _engagedMs[id] = now;
            // The framework's own call into Papyrus (the one its notifications use; CommonLib's DispatchStaticCall crashed in VR), with the
            // actor's form id as a plain int.
            F4SEVR::execPapyrusGlobalFunction(SCRIPT, FUNCTION, static_cast<F4SEVR::SInt32>(id));
            logger::info("DriverCombat: {} (detection {}) told to fight the driver", probe::describeRef(actor.get()), detection);
        }
    }
}
