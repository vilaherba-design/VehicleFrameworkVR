#include "papyrus/PapyrusApi.h"

#include "VehicleFrameworkVR.h"

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
        logger::info("Papyrus natives bound on '{}'", SCRIPT);
    }
}
