#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "model/VehicleSpec.h"

namespace vfvr::model
{
    enum class HitKind : std::uint8_t
    {
        Ballistic,
        Explosion,
        Melee,
        Unknown,
    };

    enum class VehicleCondition : std::uint8_t
    {
        Operational,
        Damaged, // health under damage.damagedBelow: a thin smoke, drives as ever
        Disabled, // health 0: no throttle, damage smoke, stays so until repaired
    };

    // Global scaling of the damage the enemies do to vehicles (INI).
    struct DamageScaling
    {
        float playerLevelFactor = 0.02f; // per player level above 1
        float playerLevelCap = 3.0f;
        float attackerLevelFactor = 0.01f; // per attacker level above 1
        float attackerLevelCap = 2.0f;
        float scale = 1.0f; // multiplies every hit
    };

    struct HitInput
    {
        HitKind kind = HitKind::Unknown;
        float baseDamage = 0.0f; // the attack's base damage; <= 0 = unknown (the spec's unknownAttackDamage stands in)
        int playerLevel = 1;
        int attackerLevel = 1; // <= 0 = unknown (counts as 1)
    };

    /**
     * Health of one vehicle (pure C++): the damage formulas of the design, the damaged and disabled conditions and the repair. The vehicle
     * and the game glue own the clock and the events; this class does the arithmetic and keeps the numbers.
     *
     *   hit damage = base * kind multiplier * level factor * 100 / (100 + kind resistance) * scale
     *   level factor = min(1 + playerLevelFactor * (playerLevel - 1), playerLevelCap) * min(1 + attackerLevelFactor * (attackerLevel - 1), attackerLevelCap)
     *   collision damage = min(factor * max(0, deltaV - minDeltaV)^exponent * 100 / (100 + collision resistance), maxShare * maxHealth)
     */
    class VehicleHealth
    {
    public:
        VehicleHealth() = default;
        explicit VehicleHealth(const DamageSpec& spec);

        const DamageSpec& spec() const
        {
            return _spec;
        }

        float health() const
        {
            return _health;
        }

        float maxHealth() const
        {
            return _spec.maxHealth;
        }

        // 0..1
        float share() const;
        VehicleCondition condition() const;

        bool disabled() const
        {
            return _health <= 0.0f;
        }

        static float levelFactor(int playerLevel, int attackerLevel, const DamageScaling& scaling);
        static float hitDamage(const DamageSpec& spec, const HitInput& hit, const DamageScaling& scaling);
        static float impactDamage(const DamageSpec& spec, float deltaV);

        // Take a hit; returns the damage taken (0 when nothing happened).
        float applyHit(const HitInput& hit, const DamageScaling& scaling);

        // Take a collision with a velocity change of `deltaV` (m/s) at `nowSeconds` (any steady clock). Within the cooldown of the previous
        // damaging one it is the same collision and does nothing. Returns the damage taken.
        float applyImpact(float deltaV, double nowSeconds);

        // An explosion reaches the vehicle twice (the driver and the hull): a second event with the same cause and source within
        // `windowSeconds` is the same explosion. Returns true the first time only.
        bool noteExplosion(std::uint32_t causeId, std::uint32_t sourceId, double nowSeconds, double windowSeconds = 0.15);

        // Give back `share` of the maximum health (1 = all of it). A disabled vehicle with health again is operational.
        void repair(float share);

        // Put back a remembered health (save game), clamped to 0..max.
        void setHealth(float health);

    private:
        DamageSpec _spec;
        float _health = 400.0f;
        double _lastImpactSeconds = -1e9;

        struct Explosion
        {
            std::uint32_t cause = 0;
            std::uint32_t source = 0;
            double seconds = -1e9;
        };
        std::array<Explosion, 8> _explosions{};
        std::size_t _nextExplosion = 0;
    };
}
