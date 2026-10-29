#pragma once

#include <cstdint>

namespace vfvr
{
    class Vehicle;

    /**
     * Diagnostics for "the enemies do not shoot the driver" (bLogAIProbe): every 2 seconds while driving, and for 15 seconds after getting out
     * (to compare), the hostile actors the game counts as near the player (ProcessLists::AreHostileActorsNear) with their distance and the
     * detection level each one has of the player (Actor::RequestDetectionLevel: below 0 the player is not detected). Both are engine calls
     * with addresses in the VR address library; the probe stays off if either is missing. For each one, a line-of-sight ray (the AI's own
     * layer) from about its eyes to the player's head node and to the player's chest: clear, or the first thing it meets (the vehicle, or a
     * layer). Then where the player's third person bones (Head, SPINE2, Pelvis) and the head node are, a ray to the Head bone, and what the engine
     * itself says through Papyrus (VehicleFrameworkVR:Combat.Report: combat state and target, weapon drawn, HasDetectionLOS, IsDetectedBy), logged
     * as "AIProbe: engine says" lines. ("AIProbe:" lines.) [game]
     */
    class AIProbe
    {
    public:
        static AIProbe& get();

        void onFrameUpdate(const Vehicle* vehicle, bool driving, bool thirdPerson);

    private:
        std::uint64_t _lastMs = 0;
        std::uint64_t _lastDrivingMs = 0;
        bool _warned = false;
    };
}
