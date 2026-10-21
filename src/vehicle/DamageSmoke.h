#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace vfvr
{
    class Vehicle;

    /**
     * Smoke of a damaged or disabled vehicle: the exhaust's puff sprites (the same base object, so no new assets) rising from the bonnet, a
     * thin wisp while the health is under damage.damagedBelow and a thick column once the vehicle is disabled. The puffs come from the
     * profile's damage.smoke.points, or from the middle of the bonnet (the engine of a two-wheeler). Like the exhaust, a small pool of
     * references is moved and scaled each frame; nothing is attached to the vehicle.
     */
    class DamageSmoke
    {
    public:
        // Every frame: the smoke `vehicle` needs by its condition (none while operational; null or invalid clears the puffs). [game]
        void update(Vehicle* vehicle);

        // Remove the puffs (vehicle release, save load). [game]
        void clear();

    private:
        struct Puff
        {
            RE::ObjectRefHandle ref;
            RE::NiPoint3 pos{};
            RE::NiPoint3 vel{};
            float age = 1.0f;
            float life = 1.0f; // age >= life: not shown
            float roll = 0.0f; // turn of the sprite about the line of sight (rad)
            float spin = 0.0f; // rad/s
            float size = 1.0f;
        };

        float random01();

        std::vector<Puff> _puffs;
        float _accumulator = 0.0f;
        std::size_t _next = 0;
        std::size_t _pointIndex = 0;
        std::uint64_t _clock = 0;
        int _cooldown = 0;
        int _no3DFrames = 0; // frames in a row in which no puff had its 3D (its cell unloaded behind the car)
        std::uint32_t _rng = 0x2545F491u;
    };
}
