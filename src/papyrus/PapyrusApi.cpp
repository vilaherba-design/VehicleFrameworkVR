#include "papyrus/PapyrusApi.h"

#include <format>
#include <string>

#include "VehicleFrameworkVR.h"
#include "vehicle/BatteryCharging.h"
#include "vehicle/VehicleTurret.h"

namespace vfvr::papyrus
{
    namespace
    {
        constexpr auto SCRIPT = "VehicleFrameworkVR:Native";

        // Natives run on the Papyrus VM thread: they only queue work for the game thread.

        bool spawnTestVehicleAt(std::monostate, RE::TESObjectREFR* marker)
        {
            return g_vehicleFramework.requestSpawnAtMarker(marker);
        }

        RE::BSFixedString getVersion(std::monostate)
        {
            return RE::BSFixedString(Version::NAME);
        }

        // VehicleFrameworkVR:ChargingStation calls this when the station gets or loses power, loads, unloads or is scrapped.
        void setChargerPowered(std::monostate, RE::TESObjectREFR* station, const bool powered)
        {
            if (station) {
                BatteryCharging::get().setStation(station->GetFormID(), powered);
            }
        }

        // VehicleFrameworkVR:Turret.StartFire calls this with the instance of the fire loop it started (so the plugin can stop it).
        void turretSoundStarted(std::monostate, const std::int32_t instance)
        {
            VehicleTurret::get().onFireSoundStarted(instance);
        }

        // VehicleFrameworkVR:Combat.Report (AIProbe) calls this: what the engine's own Papyrus calls say about an enemy and the player. Only logs.
        void reportCombat(std::monostate, const std::int32_t actorId, const std::int32_t combatState, const std::int32_t targetId, const bool weaponDrawn, const bool hasLos,
            const bool detected, const bool dead, const float distance)
        {
            const auto* player = RE::PlayerCharacter::GetSingleton();
            const bool targetIsPlayer = player && static_cast<std::uint32_t>(targetId) == player->GetFormID();
            static constexpr const char* STATES[] = { "not in combat", "IN COMBAT", "searching" };
            logger::info("AIProbe: engine says {:08X} {:.0f} m: {}, target {}, weapon {}, HasDetectionLOS(player) {}, player IsDetectedBy {}{}", static_cast<std::uint32_t>(actorId),
                distance / 70.0f, combatState >= 0 && combatState <= 2 ? STATES[combatState] : "unknown state",
                targetId == 0 ? std::string("none") : targetIsPlayer ? std::string("THE PLAYER") : std::format("{:08X}", static_cast<std::uint32_t>(targetId)),
                weaponDrawn ? "drawn" : "holstered", hasLos ? "yes" : "NO", detected ? "yes" : "no", dead ? ", dead" : "");
        }
    }

    void registerNatives()
    {
        auto* gameVM = RE::GameVM::GetSingleton();
        if (!gameVM || !gameVM->GetVM().get()) {
            logger::error("Papyrus VM not available: the test grenade will not work");
            return;
        }
        const auto vm = gameVM->GetVM();
        vm->BindNativeMethod(SCRIPT, "SpawnTestVehicleAt", spawnTestVehicleAt, true);
        vm->BindNativeMethod(SCRIPT, "GetVersion", getVersion, true);
        vm->BindNativeMethod(SCRIPT, "SetChargerPowered", setChargerPowered, true);
        vm->BindNativeMethod("VehicleFrameworkVR:Probe", "ReportCombat", reportCombat, true);
        // The compiled scripts call it on Probe ("Native" is a Papyrus keyword, so VehicleFrameworkVR:Native never compiles).
        vm->BindNativeMethod("VehicleFrameworkVR:Probe", "SetChargerPowered", setChargerPowered, true);
        vm->BindNativeMethod("VehicleFrameworkVR:Probe", "TurretSoundStarted", turretSoundStarted, true);
        logger::info("Papyrus natives bound on '{}'", SCRIPT);
    }
}
