#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "model/VehicleHealth.h"

namespace vfvr
{
    class Vehicle;

    /**
     * The engine's hit events (TESHitEvent) turned into vehicle damage. A hit on the covered driver or on the hull is copied on the engine's
     * thread (ids, numbers and names: never pointers) and applied on the game thread to the vehicle's VehicleHealth with the design's formula.
     * The driver takes nothing: while covered the OccupantShield's perk multiplies the player's incoming damage by zero, so the vehicle's
     * damage is the only damage. One projectile hits once (the driver or the hull); an explosion reaches both, and VehicleHealth counts it once.
     *
     * How the events arrive: TESHitEvent::GetEventSource is not in the VR address library, so a sink cannot be registered with the source the
     * usual way. The player itself is a BSTEventSink<TESHitEvent> (one of PlayerCharacter's bases) and the engine tells it of every hit in the
     * game, so this plugin hooks the ProcessEvent slot of that base's vtable: the sub-object is found by walking the player object's MSVC RTTI
     * (the complete object locator of a vtable names the class and the sub-object's offset, and the class hierarchy names the base at that
     * offset), the engine's function is kept and called after ours. No address id is needed.
     *
     * With bLogHitProbe every hit on the player, by the player or on the vehicle is logged ("HitProbe:" lines).
     */
    class HitSink
    {
    public:
        static HitSink& get();

        // Hook the player's hit event sink once the player exists; later calls do nothing (a failure is logged once). [game]
        void ensureHooked();

        bool hooked() const
        {
            return _hooked;
        }

        // The vehicle whose hits count (0 = none). [game]
        void setWatched(const std::uint32_t vehicleFormId)
        {
            _watched.store(vehicleFormId, std::memory_order_relaxed);
        }

        // Apply the hits queued since the last frame to `vehicle` (null: they are dropped). `seated`: the player sits in it. [game]
        void onFrameUpdate(Vehicle* vehicle, bool seated);

        // Every hit event the engine hands the player's sink, before the player's own handler sees it. [any]
        void onHit(const RE::TESHitEvent& event);

    private:
        struct Record
        {
            bool onPlayer = false;
            bool onVehicle = false;
            bool byPlayer = false;
            model::HitKind kind = model::HitKind::Unknown;
            float baseDamage = 0.0f; // the weapon's or explosion's base damage (0 = unknown)
            float engineDamage = 0.0f; // what the engine computed for the hit (with hit data only; 0 for the covered driver)
            int attackerLevel = 0;
            std::uint32_t causeId = 0;
            std::uint32_t sourceId = 0;
            std::string cause; // for the log
        };

        std::atomic<std::uint32_t> _watched{ 0 };
        std::atomic<std::uint32_t> _count{ 0 };
        bool _hooked = false;
        bool _hookFailed = false;
        std::mutex _mutex;
        std::vector<Record> _queue;
    };
}
