#include "model/VehicleHealth.h"

#include <algorithm>
#include <cmath>

namespace vfvr::model
{
    namespace
    {
        float resistanceShare(const float resistance)
        {
            return 100.0f / (100.0f + (std::max)(resistance, 0.0f));
        }

        float clampHealth(const float health, const float maxHealth)
        {
            return std::clamp(health, 0.0f, (std::max)(maxHealth, 0.0f));
        }
    }

    VehicleHealth::VehicleHealth(const DamageSpec& spec)
        : _spec(spec)
        , _health((std::max)(spec.maxHealth, 0.0f))
    {}

    float VehicleHealth::share() const
    {
        return _spec.maxHealth > 0.0f ? std::clamp(_health / _spec.maxHealth, 0.0f, 1.0f) : 0.0f;
    }

    VehicleCondition VehicleHealth::condition() const
    {
        if (_health <= 0.0f) {
            return VehicleCondition::Disabled;
        }
        return share() < _spec.damagedBelow ? VehicleCondition::Damaged : VehicleCondition::Operational;
    }

    float VehicleHealth::levelFactor(const int playerLevel, const int attackerLevel, const DamageScaling& scaling)
    {
        const auto player = static_cast<float>((std::max)(playerLevel, 1) - 1);
        const auto attacker = static_cast<float>((std::max)(attackerLevel, 1) - 1);
        const float playerFactor = (std::min)(1.0f + scaling.playerLevelFactor * player, (std::max)(scaling.playerLevelCap, 1.0f));
        const float attackerFactor = (std::min)(1.0f + scaling.attackerLevelFactor * attacker, (std::max)(scaling.attackerLevelCap, 1.0f));
        return playerFactor * attackerFactor;
    }

    float VehicleHealth::hitDamage(const DamageSpec& spec, const HitInput& hit, const DamageScaling& scaling)
    {
        const float base = hit.baseDamage > 0.0f ? hit.baseDamage : (std::max)(spec.unknownAttackDamage, 0.0f);
        float multiplier = 1.0f;
        float resistance = 0.0f;
        switch (hit.kind) {
        case HitKind::Ballistic:
            multiplier = spec.ballisticMultiplier;
            resistance = spec.ballisticResistance;
            break;
        case HitKind::Explosion:
            multiplier = spec.explosionMultiplier;
            resistance = spec.explosionResistance;
            break;
        case HitKind::Melee:
            multiplier = spec.meleeMultiplier;
            resistance = spec.meleeResistance;
            break;
        case HitKind::Unknown:
            multiplier = spec.ballisticMultiplier;
            resistance = spec.ballisticResistance;
            break;
        }
        const float damage = base * (std::max)(multiplier, 0.0f) * levelFactor(hit.playerLevel, hit.attackerLevel, scaling) * resistanceShare(resistance) *
            (std::max)(scaling.scale, 0.0f);
        return (std::max)(damage, 0.0f);
    }

    float VehicleHealth::impactDamage(const DamageSpec& spec, const float deltaV)
    {
        const float over = deltaV - spec.collisionMinDeltaV;
        if (over <= 0.0f || spec.collisionFactor <= 0.0f) {
            return 0.0f;
        }
        const float raw = spec.collisionFactor * std::pow(over, (std::max)(spec.collisionExponent, 0.0f)) * resistanceShare(spec.collisionResistance);
        const float cap = (std::max)(spec.collisionMaxShare, 0.0f) * (std::max)(spec.maxHealth, 0.0f);
        return (std::min)(raw, cap);
    }

    float VehicleHealth::applyHit(const HitInput& hit, const DamageScaling& scaling)
    {
        if (_health <= 0.0f) {
            return 0.0f;
        }
        const float damage = (std::min)(hitDamage(_spec, hit, scaling), _health);
        _health = clampHealth(_health - damage, _spec.maxHealth);
        return damage;
    }

    float VehicleHealth::applyImpact(const float deltaV, const double nowSeconds)
    {
        if (_health <= 0.0f) {
            return 0.0f;
        }
        if (nowSeconds - _lastImpactSeconds < static_cast<double>(_spec.collisionCooldownSeconds)) {
            return 0.0f;
        }
        const float damage = (std::min)(impactDamage(_spec, deltaV), _health);
        if (damage <= 0.0f) {
            return 0.0f;
        }
        _lastImpactSeconds = nowSeconds;
        _health = clampHealth(_health - damage, _spec.maxHealth);
        return damage;
    }

    bool VehicleHealth::noteExplosion(const std::uint32_t causeId, const std::uint32_t sourceId, const double nowSeconds, const double windowSeconds)
    {
        for (const auto& e : _explosions) {
            if (e.cause == causeId && e.source == sourceId && nowSeconds - e.seconds <= windowSeconds && nowSeconds >= e.seconds) {
                return false;
            }
        }
        _explosions[_nextExplosion] = Explosion{ causeId, sourceId, nowSeconds };
        _nextExplosion = (_nextExplosion + 1) % _explosions.size();
        return true;
    }

    void VehicleHealth::repair(const float share)
    {
        _health = clampHealth(_health + (std::max)(share, 0.0f) * _spec.maxHealth, _spec.maxHealth);
    }

    void VehicleHealth::setHealth(const float health)
    {
        _health = clampHealth(health, _spec.maxHealth);
    }
}
