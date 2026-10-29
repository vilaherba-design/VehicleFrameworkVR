#pragma once

#include <cstdint>
#include <map>

namespace vfvr
{
    /**
     * Enemies fight the driver (bEngageWhileDriving). Seated in a vehicle the player is detected (AIProbe showed raiders 8 m away at detection
     * 31 for 20 seconds, and a turret 5 m away at 85) but the AI never starts combat on its own. Once a second while driving, every hostile
     * actor the game counts as near (ProcessLists::AreHostileActorsNear) that detects the player (Actor::RequestDetectionLevel above 0) is
     * handed (by form id) to the Papyrus function VehicleFrameworkVR:Combat.Engage, which calls Actor.StartCombat on the player when it is not fighting
     * them already; at most once every 5 seconds per actor. Nothing runs while not driving. [game]
     */
    class DriverCombat
    {
    public:
        static DriverCombat& get();

        void onFrameUpdate(bool driving);

    private:
        std::uint64_t _lastMs = 0;
        std::map<std::uint32_t, std::uint64_t> _engagedMs; // actor form id -> when it was last handed to Engage
        bool _warned = false;
    };
}
