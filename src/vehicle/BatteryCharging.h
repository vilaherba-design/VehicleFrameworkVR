#pragma once

#include <chrono>
#include <mutex>
#include <set>
#include <string>

namespace vfvr
{
    class Vehicle;

    /**
     * The battery, the game side (bBattery). While somebody drives, the battery (model::Battery) drains with the throttle; empty, the engine is
     * dead. It fills while the vehicle stands still within fChargeRadius of a powered charging station: VFVR_ChargingStation in
     * VehicleFrameworkVR_Vehicles.esp, built at a settlement's workshop, whose script (VehicleFrameworkVR:ChargingStation) reports here when
     * it gets or loses power and when it loads or unloads. Emptied on the road, it takes a fusion core (the game's, Fallout4.esm AmmoFusionCore)
     * from the inventory, then the trunk, for fCoreCharge of a full charge. The charge of each profile lives in the co-save next to its health (HealthStore).
     */
    class BatteryCharging
    {
    public:
        static BatteryCharging& get();

        // A charging station has power, or lost it (or unloaded, or was scrapped). [any] from the Papyrus VM.
        void setStation(std::uint32_t formId, bool powered);

        // [game] once per frame. `driving`: the player is at the wheel of `vehicle` (null when there is none).
        void onFrameUpdate(Vehicle* vehicle, bool driving);

        // [game] a vehicle was bound: take over the charge its profile had in the save (a new profile comes full).
        void restore(Vehicle& vehicle) const;

    private:
        BatteryCharging() = default;

        bool nearPoweredStation(const Vehicle& vehicle);
        // An empty battery on the road: one fusion core from the inventory or the trunk for fCoreCharge. False when there is none.
        bool useFusionCore(Vehicle& vehicle);

        mutable std::mutex _mutex;
        std::set<std::uint32_t> _powered; // charging stations with power, loaded

        std::chrono::steady_clock::time_point _last{};
        bool _clockValid = false;
        float _nearTimer = 0.0f;
        float _coreTimer = 0.0f;
        bool _near = false;
        bool _charging = false;
        bool _emptyNotified = false;
        bool _lowNotified = false;
        std::string _profile;
    };
}
