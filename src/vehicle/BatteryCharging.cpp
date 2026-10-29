#include "vehicle/BatteryCharging.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <vector>

#include "Config.h"
#include "f4vr/F4VRUtils.h"
#include "health/HealthStore.h"
#include "physics/HavokBridge.h"
#include "vehicle/Trunk.h"
#include "vehicle/Vehicle.h"

namespace vfvr
{
    namespace
    {
        constexpr float LOW_SHARE = 0.15f;
        constexpr float NEAR_INTERVAL = 0.25f; // s between looks for a powered station around the vehicle
        constexpr float STILL_SPEED = 0.5f; // m/s under which the vehicle stands still and can charge
        constexpr float CORE_INTERVAL = 1.0f; // s between looks for a fusion core while the battery is empty
        constexpr auto GAME_FILE = "Fallout4.esm";
        constexpr RE::TESFormID FUSION_CORE_ID = 0x075FE4; // AmmoFusionCore

        RE::TESBoundObject* fusionCore()
        {
            static RE::TESBoundObject* core = [] {
                auto* handler = RE::TESDataHandler::GetSingleton();
                auto* form = handler ? handler->LookupForm(FUSION_CORE_ID, GAME_FILE) : nullptr;
                auto* found = form ? form->As<RE::TESBoundObject>() : nullptr;
                if (!found) {
                    logger::error("Battery: the fusion core ({:06X} in {}) was not found: no core can charge a battery", FUSION_CORE_ID, GAME_FILE);
                }
                return found;
            }();
            return core;
        }

        // The most charged fusion core stack in `ref`'s inventory: its index in the item's stack chain and its charge. False when there is none.
        bool bestCore(RE::TESObjectREFR* ref, RE::TESBoundObject* core, std::uint32_t& index, float& charge, float& rawHealth, bool& hasHealth)
        {
            if (!ref || !core || !ref->inventoryList) {
                return false;
            }
            bool found = false;
            const RE::BSAutoReadLock lock{ ref->inventoryList->rwLock };
            for (auto& item : ref->inventoryList->data) {
                if (item.object != core) {
                    continue;
                }
                std::uint32_t i = 0;
                for (auto* stack = item.stackData.get(); stack; stack = stack->nextStack.get(), ++i) {
                    if (stack->GetCount() == 0) {
                        continue;
                    }
                    const auto* health = stack->extra ? stack->extra->GetByType<RE::ExtraHealth>() : nullptr;
                    const float c = model::Battery::coreCharge(health != nullptr, health ? health->health : 0.0f);
                    if (!found || c > charge) {
                        found = true;
                        index = i;
                        charge = c;
                        hasHealth = health != nullptr;
                        rawHealth = health ? health->health : 0.0f;
                    }
                }
                break;
            }
            return found;
        }
    }

    BatteryCharging& BatteryCharging::get()
    {
        static BatteryCharging instance;
        return instance;
    }

    void BatteryCharging::setStation(const std::uint32_t formId, const bool powered)
    {
        bool changed;
        {
            const std::scoped_lock lock(_mutex);
            changed = powered ? _powered.insert(formId).second : _powered.erase(formId) > 0;
        }
        if (changed) {
            logger::info("Battery: charging station {:08X} {}", formId, powered ? "has power" : "has no power (or unloaded)");
        }
    }

    void BatteryCharging::restore(Vehicle& vehicle) const
    {
        if (!g_config.rememberVehicleState) {
            return;
        }
        if (const auto saved = HealthStore::get().lookupBattery(vehicle.spec().id)) {
            vehicle.battery().restore(*saved);
            logger::info("Battery: {} takes over its remembered charge: {:.0f}%", vehicle.spec().displayName, *saved * 100.0f);
        }
    }

    bool BatteryCharging::nearPoweredStation(const Vehicle& vehicle)
    {
        std::vector<std::uint32_t> stations;
        {
            const std::scoped_lock lock(_mutex);
            stations.assign(_powered.begin(), _powered.end());
        }
        if (stations.empty()) {
            return false;
        }
        const auto car = physics::toGame(vehicle.poseFromBody().pos);
        const float radius = g_config.chargeRadius;
        for (const auto id : stations) {
            auto* station = RE::TESForm::GetFormByID<RE::TESObjectREFR>(id);
            if (!station || station->IsDeleted() || station->IsDisabled() || !station->Get3D()) {
                continue;
            }
            const auto& p = station->data.location;
            const float dx = p.x - car.x;
            const float dy = p.y - car.y;
            const float dz = p.z - car.z;
            if (dx * dx + dy * dy + dz * dz <= radius * radius) {
                return true;
            }
        }
        return false;
    }

    bool BatteryCharging::useFusionCore(Vehicle& vehicle)
    {
        auto* core = fusionCore();
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!core || !player || g_config.coreCharge <= 0.0f) {
            return false;
        }
        const auto& spec = vehicle.spec();
        RE::TESObjectREFR* sources[2] = { player, Trunk::get().existingContainer(spec.id) };
        const char* sourceNames[2] = { "the inventory", "the trunk" };
        for (int s = 0; s < 2; ++s) {
            std::uint32_t index = 0;
            float charge = 0.0f;
            float raw = 0.0f;
            bool hasHealth = false;
            if (!bestCore(sources[s], core, index, charge, raw, hasHealth)) {
                continue;
            }
            if (charge <= 0.0f) {
                logger::info("Battery: the best fusion core in {} is spent (health {:.3f})", sourceNames[s], raw);
                continue;
            }
            RE::TESObjectREFR::RemoveItemData data(core, 1);
            data.stackData.push_back(index);
            sources[s]->RemoveItem(data);
            vehicle.battery().addShare(g_config.coreCharge);
            logger::info("Battery: {} takes a fusion core from {} (stack {}, {}): battery {:.0f}%", spec.displayName, sourceNames[s], index,
                hasHealth ? std::format("health {:.3f}", raw) : std::string("no health data"), vehicle.battery().charge() * 100.0f);
            f4vr::showNotification(std::format("{}: a fusion core from {} charged the battery to {:.0f}%", spec.displayName, sourceNames[s],
                vehicle.battery().charge() * 100.0f));
            return true;
        }
        return false;
    }

    void BatteryCharging::onFrameUpdate(Vehicle* vehicle, const bool driving)
    {
        const auto now = std::chrono::steady_clock::now();
        const float dt = _clockValid ? std::clamp(std::chrono::duration<float>(now - _last).count(), 0.0f, 0.1f) : 0.0f;
        _last = now;
        _clockValid = true;
        if (!g_config.battery || !vehicle || !vehicle->isValid()) {
            _charging = false;
            return;
        }
        auto& battery = vehicle->battery();
        const auto& spec = vehicle->spec();
        if (spec.id != _profile) {
            _profile = spec.id;
            _emptyNotified = battery.empty();
            _lowNotified = battery.charge() < LOW_SHARE;
            _charging = false;
            _nearTimer = 0.0f;
        }
        if (!battery.spec().enabled) {
            vehicle->setCharging(false);
            return;
        }

        // Driving uses it up.
        if (driving && !vehicle->health().disabled()) {
            const float before = battery.charge();
            if (battery.drain(dt, vehicle->telemetry().throttle, g_config.batteryScale)) {
                logger::info("Battery: {} is empty", spec.displayName);
            }
            if (!_lowNotified && before >= LOW_SHARE && battery.charge() < LOW_SHARE && !battery.empty()) {
                _lowNotified = true;
                f4vr::showNotification(std::format("{}: battery at {:.0f}%", spec.displayName, battery.charge() * 100.0f));
            }
        }

        // Empty on the road: a fusion core, if one is carried.
        if (driving && battery.empty()) {
            _coreTimer -= dt;
            if (_coreTimer <= 0.0f) {
                _coreTimer = CORE_INTERVAL;
                if (useFusionCore(*vehicle)) {
                    _emptyNotified = false;
                } else if (!_emptyNotified) {
                    _emptyNotified = true;
                    f4vr::showNotification(std::format("{}: battery empty. Charge it at a charging station or carry a fusion core", spec.displayName));
                }
            }
        } else {
            _coreTimer = 0.0f;
        }

        // Standing still by a powered station fills it, with or without the driver in the seat.
        _nearTimer -= dt;
        if (_nearTimer <= 0.0f) {
            _nearTimer = NEAR_INTERVAL;
            _near = nearPoweredStation(*vehicle);
        }
        const bool canCharge = _near && std::abs(vehicle->speed()) < STILL_SPEED && !battery.full();
        if (canCharge) {
            if (!_charging) {
                logger::info("Battery: {} starts charging at {:.0f}%", spec.displayName, battery.charge() * 100.0f);
                f4vr::showNotification(std::format("{}: charging ({:.0f}%)", spec.displayName, battery.charge() * 100.0f));
            }
            _charging = true;
            if (battery.recharge(dt, g_config.chargeScale)) {
                logger::info("Battery: {} is fully charged", spec.displayName);
                f4vr::showNotification(std::format("{}: battery full", spec.displayName));
            }
            if (battery.charge() >= LOW_SHARE) {
                _lowNotified = false;
            }
            _emptyNotified = battery.empty();
        } else if (_charging) {
            _charging = false;
            logger::info("Battery: {} stops charging at {:.0f}%", spec.displayName, battery.charge() * 100.0f);
        }
        vehicle->setCharging(_charging);

        if (g_config.rememberVehicleState) {
            HealthStore::get().rememberBattery(spec.id, battery.charge());
        }
    }
}
