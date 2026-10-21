#include "health/VitalsProbe.h"

#include <format>

#include "Config.h"
#include "common/CommonUtils.h"
#include "health/OccupantShield.h"
#include "physics/HavokBridge.h"

namespace vfvr
{
    namespace
    {
        constexpr std::uint64_t ACTOR_GET_LEVEL_ID = 661617;
    }

    VitalsProbe& VitalsProbe::get()
    {
        static VitalsProbe instance;
        return instance;
    }

    void VitalsProbe::onFrameUpdate(const bool driving)
    {
        if (!g_config.logVitals) {
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        const auto* avs = RE::ActorValue::GetSingleton();
        if (!player || !avs || !avs->health || !avs->rads || !avs->radExposureResistance) {
            return;
        }
        using Modifier = RE::ACTOR_VALUE_MODIFIER;
        const float health = player->GetActorValue(*avs->health);
        const float healthPermanent = player->GetPermanentActorValue(*avs->health);
        const float healthDamage = player->GetModifier(Modifier::kDamage, *avs->health);
        const float actionPoints = avs->actionPoints ? player->GetActorValue(*avs->actionPoints) : -1.0f;
        const float rads = player->GetActorValue(*avs->rads);
        const float radsBase = player->GetBaseActorValue(*avs->rads);
        const float radsPermanent = player->GetModifier(Modifier::kPermanent, *avs->rads);
        const float radsTemporary = player->GetModifier(Modifier::kTemporary, *avs->rads);
        const float radsDamage = player->GetModifier(Modifier::kDamage, *avs->rads);
        const float radResistance = player->GetActorValue(*avs->radExposureResistance);
        const float radResistanceBase = player->GetBaseActorValue(*avs->radExposureResistance);
        const float radResistanceTemporary = player->GetModifier(Modifier::kTemporary, *avs->radExposureResistance);
        const float radHealthMax = avs->radHealthMax ? player->GetActorValue(*avs->radHealthMax) : -1.0f;

        // What decides an immediate line: the values that matter, rounded so a slow rad tick does not log every frame.
        const auto state = std::format("health {:.0f} rads {:.0f} radRes {:.0f} shield {} driving {}", health, rads, radResistance, OccupantShield::get().active(), driving);
        const auto now = common::nowMillis();
        const std::uint64_t interval = driving ? 2000 : 10000;
        const bool changed = state != _lastState;
        if (!changed && now - _lastLogMs < interval) {
            return;
        }
        if (changed && now - _lastLogMs < 500) {
            return;
        }
        _lastLogMs = now;
        _lastState = state;
        static const bool hasLevel = physics::hasAddressId(ACTOR_GET_LEVEL_ID);
        const int level = hasLevel ? static_cast<int>(player->GetLevel()) : -1;
        logger::info("Vitals{}: health {:.1f} (permanent {:.1f}, damage {:.1f}), action points {:.1f}, rads {:.1f} (base {:.1f}, permanent {:.1f}, temporary {:.1f}, damage {:.1f}), "
                     "rad resistance {:.1f} (base {:.1f}, temporary {:.1f}), rad health max {:.1f}, level {}, driving {}, shield {}; {}",
            changed ? " CHANGED" : "",
            health,
            healthPermanent,
            healthDamage,
            actionPoints,
            rads,
            radsBase,
            radsPermanent,
            radsTemporary,
            radsDamage,
            radResistance,
            radResistanceBase,
            radResistanceTemporary,
            radHealthMax,
            level,
            driving,
            OccupantShield::get().active() ? "on" : "off",
            OccupantShield::describePlayer());
    }
}
