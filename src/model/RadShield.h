#pragma once

#include "model/VehicleSpec.h"

namespace vfvr::model
{
    /**
     * Radiation protection of one vehicle (pure C++). While the occupant is covered and reserve is left, the vehicle lends its `resistance`
     * points of radiation resistance to the player (the game applies its own curve: there is no second pool of rads); what the resistance
     * blocks uses the reserve up. The glue owns the clock, the player's actor value and the ambient reading; this class does the arithmetic.
     *
     * The game's resistance curve (the same as damage resistance), for an intensity r against R points:
     *   received = r * 0.5 * (r / R)^0.365          for R >= r
     *   received = r * (1 - 0.5 * (R / r)^0.365)    for R < r
     */
    class RadShield
    {
    public:
        RadShield() = default;
        explicit RadShield(const RadiationSpec& spec);

        const RadiationSpec& spec() const
        {
            return _spec;
        }

        float capacity() const
        {
            return _spec.capacity;
        }

        float remaining() const
        {
            return _remaining;
        }

        // 0..1 of the capacity left; 0 for a vehicle without protection.
        float share() const;

        // The protection acts: the vehicle has some and reserve is left.
        bool active() const
        {
            return _spec.resistance > 0.0f && _remaining > 0.0f;
        }

        // Resistance points to lend the occupant right now (0 when not active).
        float resistance() const
        {
            return active() ? _spec.resistance : 0.0f;
        }

        // Share (0..1) of an ambient intensity that gets through `resistance` points by the game's curve.
        static float receivedShare(float ambient, float resistance);

        // 1 - receivedShare: what the protection blocks.
        static float efficacy(float ambient, float resistance)
        {
            return 1.0f - receivedShare(ambient, resistance);
        }

        // The ambient intensity that leaves `received` rads/s through `resistance` points (the curve inverted).
        static float ambientFromReceived(float received, float resistance);

        // One tick of `dt` seconds under `ambient` rads/s: what the resistance blocks of it (by the curve) uses the reserve up (above the drain
        // threshold) and the recovery per second comes back. Returns the reserve used.
        float tick(float ambient, float dt);

        // One tick of `dt` seconds with `blocked` rads/s kept off the occupant (measured by the glue instead of taken from the curve): above
        // the drain threshold it uses the reserve up (drainPerRad per rad), and the recovery per second comes back. Returns the reserve used.
        float drain(float blocked, float dt);

        // Give back `share` of the capacity (1 = all of it).
        void repair(float share);

        // Put back a remembered reserve (save game), clamped to 0..capacity.
        void setRemaining(float remaining);

    private:
        RadiationSpec _spec;
        float _remaining = 0.0f;
    };
}
