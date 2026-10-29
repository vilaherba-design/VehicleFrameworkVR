#pragma once

#include "model/VehicleSpec.h"

namespace vfvr::model
{
    /**
     * A vehicle's battery (pure C++): its charge, 0..1. The glue owns the clock, the throttle and whether a powered charging station is near;
     * this class does the arithmetic.
     */
    class Battery
    {
    public:
        Battery() = default;
        explicit Battery(const BatterySpec& spec);

        const BatterySpec& spec() const
        {
            return _spec;
        }

        // 0..1 (a new vehicle comes full).
        float charge() const
        {
            return _charge;
        }

        // The vehicle needs charge and has none left: the engine is dead.
        bool empty() const
        {
            return _spec.enabled && _charge <= 0.0f;
        }

        bool full() const
        {
            return _charge >= 1.0f;
        }

        // Share of the battery used per second at `throttle` (0..1): 1/minutesIdle per minute at 0, 1/minutesFullThrottle per minute at 1.
        static float drainPerSecond(const BatterySpec& spec, float throttle);

        // `seconds` of driving at `throttle`, times `scale` (fBatteryScale). True when this call emptied it.
        bool drain(float seconds, float throttle, float scale);

        // `seconds` at a powered charging station, times `scale` (fChargeScale). True when this call filled it.
        bool recharge(float seconds, float scale);

        // A fusion core put in on the road (fCoreCharge): `share` of a full battery added at once, up to full.
        void addShare(float share);

        // The charge of a fusion core in an inventory, 0..1, from its stack's health extra data: none = a full core; the game keeps 0..1, a
        // value over 1 is read as a percentage.
        static float coreCharge(bool hasHealth, float health);

        // The save game's charge.
        void restore(float charge);

    private:
        BatterySpec _spec;
        float _charge = 1.0f;
    };
}
