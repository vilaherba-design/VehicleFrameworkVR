#include "health/RadProtection.h"

#include <algorithm>
#include <format>

#include "common/CommonUtils.h"
#include "f4vr/F4VRUtils.h"
#include "model/RadShield.h"
#include "vehicle/Vehicle.h"

namespace vfvr
{
    namespace
    {
        constexpr std::uint64_t TICK_MS = 250;
        constexpr std::uint64_t LOG_MS = 2000;
    }

    RadProtection& RadProtection::get()
    {
        static RadProtection instance;
        return instance;
    }

    bool RadProtection::modify(const float delta)
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        const auto* avs = RE::ActorValue::GetSingleton();
        if (!player || !avs || !avs->radExposureResistance || delta == 0.0f) {
            return false;
        }
        player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *avs->radExposureResistance, delta);
        return true;
    }

    void RadProtection::onFrameUpdate(Vehicle* vehicle, const bool seated)
    {
        if (_undoPending != 0.0f && modify(-_undoPending)) {
            logger::info("RadProtection: the save had {:.0f} points of a vehicle's radiation resistance on the player; taken off", _undoPending);
            _undoPending = 0.0f;
        }
        const bool valid = vehicle && vehicle->isValid();
        const bool wanted = valid && seated && vehicle->radShield().active();
        const float resistance = wanted ? vehicle->radShield().resistance() : 0.0f;
        if (_applied != 0.0f && (!wanted || resistance != _applied)) {
            clear(!valid ? "no vehicle" : !seated ? "left the seat" : !wanted ? "shielding depleted" : "resistance changed");
        }
        if (wanted && _applied == 0.0f && modify(resistance)) {
            _applied = resistance;
            _hasLastRads = false;
            _lastTickMs = common::nowMillis();
            logger::info("RadProtection: ON, {:.0f} points of radiation resistance from {} (reserve {:.0f} / {:.0f})", resistance, vehicle->spec().displayName,
                vehicle->radShield().remaining(), vehicle->radShield().capacity());
        }
        if (_applied == 0.0f) {
            return;
        }
        const auto now = common::nowMillis();
        if (now - _lastTickMs < TICK_MS) {
            return;
        }
        const float dt = static_cast<float>(now - _lastTickMs) * 0.001f;
        _lastTickMs = now;
        tick(*vehicle, dt);
    }

    void RadProtection::clear(const char* reason)
    {
        if (_applied == 0.0f) {
            return;
        }
        modify(-_applied);
        logger::info("RadProtection: OFF ({}), {:.0f} points taken off the player", reason, _applied);
        _applied = 0.0f;
        _hasLastRads = false;
        _ambientEstimate = 0.0f;
    }

    void RadProtection::tick(Vehicle& vehicle, const float dt)
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        const auto* avs = RE::ActorValue::GetSingleton();
        if (!player || !avs || !avs->rads || !avs->radExposureResistance || dt <= 0.0f) {
            return;
        }
        // What the player got since the last tick (RadAway lowers the value: that is not radiation).
        const float rads = player->GetActorValue(*avs->rads);
        const float received = _hasLastRads ? (std::max)(rads - _lastRads, 0.0f) / dt : 0.0f;
        _lastRads = rads;
        _hasLastRads = true;

        // The ambient intensity from what got through the player's whole resistance; what the vehicle keeps off is what the player would get
        // with their own resistance alone, less what they get now.
        const float total = (std::max)(player->GetActorValue(*avs->radExposureResistance), 0.0f);
        const float own = (std::max)(total - _applied, 0.0f);
        const float ambient = model::RadShield::ambientFromReceived(received, total);
        _ambientEstimate = ambient;
        const float blocked = ambient > 0.0f ? ambient * (model::RadShield::receivedShare(ambient, own) - model::RadShield::receivedShare(ambient, total)) : 0.0f;
        auto& shield = vehicle.radShield();
        const float used = shield.drain((std::max)(blocked, 0.0f), dt);

        const auto now = common::nowMillis();
        if ((used > 0.0f || ambient > 0.0f) && now - _lastLogMs >= LOG_MS) {
            _lastLogMs = now;
            logger::info("RadProtection: ambient ~{:.1f} rads/s, player gets {:.2f} rads/s with {:.0f} points ({:.0f} own), vehicle keeps off {:.1f} rads/s, reserve {:.0f} / {:.0f}",
                ambient, received, total, own, blocked, shield.remaining(), shield.capacity());
        }
        if (!shield.active()) {
            logger::info("RadProtection: the reserve of {} ran out; the player gets the game's normal radiation from here", vehicle.spec().displayName);
            f4vr::showNotification(std::format("{}: radiation shielding depleted", vehicle.spec().displayName));
            // The modifier comes off on the next frame (the shield is no longer active).
        }
    }
}
