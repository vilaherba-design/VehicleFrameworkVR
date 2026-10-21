#pragma once

#include <algorithm>

#include "vehicle/HudState.h"

namespace vfvr::health
{
    // The player's real health, rads and action points for the HUD bars, as the game's own HUD shows them: health over its full maximum
    // (permanent plus temporary; the rads take their share of that maximum, rads over the lethal 1000, and the game paints it red from the
    // right of the health bar), action points over their maximum. Nothing is computed twice: the values are the game's own actor values. [game]
    inline void fillPlayerVitals(HudState& state)
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        const auto* avs = RE::ActorValue::GetSingleton();
        if (!player || !avs || !avs->health || !avs->rads) {
            return;
        }
        using Modifier = RE::ACTOR_VALUE_MODIFIER;
        const auto fullMax = [&](const RE::ActorValueInfo& info) {
            return (std::max)(player->GetPermanentActorValue(info) + player->GetModifier(Modifier::kTemporary, info), 1.0f);
        };
        const float health = player->GetActorValue(*avs->health);
        state.playerHealthPoints = (std::max)(health, 0.0f);
        state.playerHealth = std::clamp(health / fullMax(*avs->health), 0.0f, 1.0f);
        const float rads = player->GetActorValue(*avs->rads);
        state.playerRadsPoints = (std::max)(rads, 0.0f);
        state.playerRads = std::clamp(rads / 1000.0f, 0.0f, 1.0f);
        if (avs->actionPoints) {
            const float actionPoints = player->GetActorValue(*avs->actionPoints);
            state.playerActionPointsValue = (std::max)(actionPoints, 0.0f);
            state.playerActionPoints = std::clamp(actionPoints / fullMax(*avs->actionPoints), 0.0f, 1.0f);
        }
    }
}
