#include "health/HitSink.h"

#include <algorithm>
#include <chrono>
#include <format>
#include <optional>
#include <string_view>
#include <utility>

#include "Config.h"
#include "health/OccupantShield.h"
#include "health/ProbeUtils.h"
#include "physics/HavokBridge.h"
#include "vehicle/Vehicle.h"

namespace vfvr
{
    namespace
    {
        constexpr std::uint32_t PLAYER_FORM_ID = 0x14;
        constexpr std::size_t MAX_QUEUE = 64; // hits waiting for the next frame; a flood beyond that is dropped
        constexpr std::size_t PROCESS_EVENT_SLOT = 1; // BSTEventSink's vtable: 0 the destructor, 1 ProcessEvent
        constexpr std::size_t MAX_SUBOBJECT_OFFSET = 0x800; // the player's sinks lie well inside the object (0x4B0 in the headers)
        constexpr std::string_view PLAYER_TYPE = ".?AVPlayerCharacter@@";
        constexpr std::string_view SINK_TYPE = ".?AV?$BSTEventSink@VTESHitEvent@@@@";
        constexpr double SAME_ATTACK_SECONDS = 0.3; // an effect hit this close to its attacker's weapon hit belongs to it
        constexpr float DRIVER_COVER_FRACTION = 0.1f; // what the cover perk lets through of the driver's incoming damage (tools/make_vehicle_esp.py)

        using ProcessEventFn = RE::BSEventNotifyControl (*)(RE::BSTEventSink<RE::TESHitEvent>*, const RE::TESHitEvent&, RE::BSTEventSource<RE::TESHitEvent>*);
        ProcessEventFn g_originalProcessEvent = nullptr;

        RE::BSEventNotifyControl hookedProcessEvent(RE::BSTEventSink<RE::TESHitEvent>* self, const RE::TESHitEvent& event, RE::BSTEventSource<RE::TESHitEvent>* source)
        {
            HitSink::get().onHit(event);
            return g_originalProcessEvent ? g_originalProcessEvent(self, event, source) : RE::BSEventNotifyControl::kContinue;
        }

        bool within(const std::uintptr_t address, const REL::Segment& segment)
        {
            return segment.size() != 0 && address >= segment.address() && address < segment.address() + segment.size();
        }

        // The mangled name of a type descriptor that lies in the game module (type_info objects live in .data), or null.
        const char* typeName(const RE::RTTI::RVA<RE::RTTI::TypeDescriptor>& rva, const REL::Segment& rdata, const REL::Segment& data)
        {
            if (!rva) {
                return nullptr;
            }
            const auto* type = rva.get();
            const auto address = reinterpret_cast<std::uintptr_t>(type);
            return type && (within(address, data) || within(address, rdata)) ? type->mangled_name() : nullptr;
        }

        struct SinkVTable
        {
            std::uintptr_t* slots = nullptr;
            std::size_t offset = 0;
        };

        // The vtable of the player's BSTEventSink<TESHitEvent> sub-object, by the object's own RTTI: a vtable pointer in the object whose
        // complete object locator names PlayerCharacter at that very offset, and whose class hierarchy has the hit sink base at that offset.
        std::optional<SinkVTable> findHitSinkVTable(const RE::PlayerCharacter& player)
        {
            const auto& module = REL::Module::get();
            const auto text = module.segment(REL::Segment::text);
            const auto rdata = module.segment(REL::Segment::rdata);
            const auto data = module.segment(REL::Segment::data);
            const auto object = reinterpret_cast<std::uintptr_t>(&player);
            for (std::size_t offset = 8; offset < MAX_SUBOBJECT_OFFSET; offset += 8) {
                auto* slots = *reinterpret_cast<std::uintptr_t* const*>(object + offset);
                const auto address = reinterpret_cast<std::uintptr_t>(slots);
                if (!within(address, rdata) || !within(address - sizeof(std::uintptr_t), rdata) || !within(slots[-1], rdata)) {
                    continue;
                }
                const auto* locator = reinterpret_cast<const RE::RTTI::CompleteObjectLocator*>(slots[-1]);
                if (locator->signature.underlying() != 1 || locator->offset != offset) {
                    continue;
                }
                const char* name = typeName(locator->typeDescriptor, rdata, data);
                if (!name || std::string_view(name) != PLAYER_TYPE) {
                    continue;
                }
                const auto* hierarchy = locator->classDescriptor ? locator->classDescriptor.get() : nullptr;
                if (!hierarchy || !within(reinterpret_cast<std::uintptr_t>(hierarchy), rdata) || !hierarchy->baseClassArray) {
                    continue;
                }
                // The array holds image-relative addresses of the base class descriptors.
                const auto* bases = reinterpret_cast<const RE::RTTI::RVA<RE::RTTI::BaseClassDescriptor>*>(hierarchy->baseClassArray.get());
                if (!within(reinterpret_cast<std::uintptr_t>(bases), rdata)) {
                    continue;
                }
                for (std::uint32_t i = 0; i < hierarchy->numBaseClasses && i < 256; ++i) {
                    const auto* base = bases[i] ? bases[i].get() : nullptr;
                    if (!base || !within(reinterpret_cast<std::uintptr_t>(base), rdata) || base->pmd.mDisp != static_cast<std::int32_t>(offset) || base->pmd.pDisp != -1) {
                        continue;
                    }
                    const char* baseName = typeName(base->typeDescriptor, rdata, data);
                    if (baseName && std::string_view(baseName) == SINK_TYPE && within(slots[PROCESS_EVENT_SLOT], text)) {
                        return SinkVTable{ slots, offset };
                    }
                }
            }
            return std::nullopt;
        }

        const char* kindName(const model::HitKind kind)
        {
            switch (kind) {
            case model::HitKind::Ballistic:
                return "ballistic";
            case model::HitKind::Explosion:
                return "explosion";
            case model::HitKind::Melee:
                return "melee";
            default:
                return "unknown";
            }
        }

        // The kind and base damage of the attack from the form the event names as its source: the explosion (its damage trumps the launcher's
        // number), or the weapon (a hit on the hull carries no hit data; a creature's attack names its unarmed weapon). Leaves what it
        // cannot tell. [any]
        void classifyBySource(const std::uint32_t sourceFormId, model::HitKind& kind, float& baseDamage)
        {
            const auto* source = sourceFormId != 0 ? RE::TESForm::GetFormByID(sourceFormId) : nullptr;
            if (!source) {
                return;
            }
            if (source->GetFormType() == RE::ENUM_FORM_ID::kEXPL) {
                kind = model::HitKind::Explosion;
                baseDamage = static_cast<const RE::BGSExplosion*>(source)->data.damage;
            } else if (source->GetFormType() == RE::ENUM_FORM_ID::kWEAP) {
                const auto* weapon = static_cast<const RE::TESObjectWEAP*>(source);
                if (kind == model::HitKind::Unknown) {
                    kind = weapon->IsMeleeWeapon() ? model::HitKind::Melee : model::HitKind::Ballistic;
                }
                if (baseDamage <= 0.0f) {
                    baseDamage = static_cast<float>(weapon->weaponData.attackDamage);
                }
            }
        }

        double nowSeconds()
        {
            return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }
    }

    HitSink& HitSink::get()
    {
        static HitSink instance;
        return instance;
    }

    void HitSink::ensureHooked()
    {
        if (_hooked || _hookFailed) {
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            return;
        }
        const auto found = findHitSinkVTable(*player);
        if (!found) {
            _hookFailed = true;
            logger::error("HitSink: the player's hit event sink (BSTEventSink<TESHitEvent>) was not found in the player's RTTI: hits cannot damage the vehicle");
            return;
        }
        // Which of the player's vtables it is, by the address library: a cross-check for the log, not a requirement.
        int index = -1;
        for (std::size_t i = 0; i < RE::VTABLE::PlayerCharacter.size(); ++i) {
            const auto& id = RE::VTABLE::PlayerCharacter[i];
            if (physics::hasAddressId(id.id()) && id.address() == reinterpret_cast<std::uintptr_t>(found->slots)) {
                index = static_cast<int>(i);
                break;
            }
        }
        g_originalProcessEvent = reinterpret_cast<ProcessEventFn>(found->slots[PROCESS_EVENT_SLOT]);
        REL::safe_write(reinterpret_cast<std::uintptr_t>(found->slots + PROCESS_EVENT_SLOT), reinterpret_cast<std::uintptr_t>(&hookedProcessEvent));
        _hooked = true;
        const auto base = REL::Module::get().base();
        logger::info("HitSink: hooked the player's hit event sink: sub-object at +0x{:X}, vtable at module +0x{:X}{}, the engine's handler at +0x{:X}",
            found->offset,
            reinterpret_cast<std::uintptr_t>(found->slots) - base,
            index >= 0 ? std::format(" (PlayerCharacter vtable {} of the address library)", index) : " (not one the address library lists)",
            reinterpret_cast<std::uintptr_t>(g_originalProcessEvent) - base);
    }

    void HitSink::onHit(const RE::TESHitEvent& event)
    {
        auto* target = event.target.get();
        auto* cause = event.cause.get();
        const auto targetId = target ? target->GetFormID() : 0u;
        const auto causeId = cause ? cause->GetFormID() : 0u;
        const auto watched = _watched.load(std::memory_order_relaxed);
        Record r;
        r.onPlayer = targetId == PLAYER_FORM_ID;
        r.onVehicle = watched != 0 && targetId == watched;
        r.byPlayer = causeId == PLAYER_FORM_ID;
        if (!r.onPlayer && !r.onVehicle) {
            if (g_config.logHitProbe) {
                std::string where;
                if (event.usesHitData) {
                    const auto& at = event.hitData.impactData.location;
                    where = std::format(" at ({:.0f}, {:.0f}, {:.0f})", at.x, at.y, at.z);
                }
                if (r.byPlayer) {
                    logger::info("HitProbe: the player hit {} with {}{}", probe::describeRef(target), probe::describeFormId(event.sourceFormID), where);
                } else {
                    // Where everybody else's hits land: a shot at the driver that misses hits the world (0.1.106: in third person none hit the player).
                    logger::info("HitProbe: other hit: {} hit by {} with {}, projectile {}{}", probe::describeRef(target), probe::describeRef(cause),
                        probe::describeFormId(event.sourceFormID), probe::describeFormId(event.projectileFormID), where);
                }
            }
            return;
        }
        const auto n = _count.fetch_add(1, std::memory_order_relaxed) + 1;
        r.causeId = causeId;
        r.sourceId = event.sourceFormID;
        r.attackerLevel = probe::levelOf(cause);

        std::string details;
        if (event.usesHitData) {
            const auto& d = event.hitData;
            r.engineDamage = d.totalDamage;
            if (d.flags.any(RE::HitData::Flag::kExplosion)) {
                r.kind = model::HitKind::Explosion;
            } else if (d.flags.any(RE::HitData::Flag::kMeleeAttack) || d.flags.any(RE::HitData::Flag::kBash)) {
                r.kind = model::HitKind::Melee;
            }
            const auto* weaponForm = d.weapon.object;
            if (weaponForm && weaponForm->GetFormType() == RE::ENUM_FORM_ID::kWEAP) {
                const auto* weapon = static_cast<const RE::TESObjectWEAP*>(weaponForm);
                if (r.kind == model::HitKind::Unknown) {
                    r.kind = weapon->IsMeleeWeapon() ? model::HitKind::Melee : model::HitKind::Ballistic;
                }
                // The instance carries the mods' damage; without one the weapon's own number stands.
                const auto* instance = static_cast<const RE::TESObjectWEAP::InstanceData*>(d.weapon.instanceData.get());
                r.baseDamage = static_cast<float>(instance ? instance->attackDamage : weapon->weaponData.attackDamage);
            }
            if (g_config.logHitProbe) {
                details = std::format("; hit data: weapon {} ammo {} health {:.1f} total {:.1f} physical {:.1f} resisted {:.1f} flags 0x{:X} at ({:.0f}, {:.0f}, {:.0f})",
                    probe::describeForm(weaponForm),
                    probe::describeForm(d.ammo),
                    d.healthDamage,
                    d.totalDamage,
                    d.physicalDamage,
                    d.resistedPhysicalDamage,
                    d.flags.underlying(),
                    d.impactData.location.x,
                    d.impactData.location.y,
                    d.impactData.location.z);
            }
        }
        if (r.kind == model::HitKind::Explosion || r.kind == model::HitKind::Unknown || r.baseDamage <= 0.0f) {
            classifyBySource(event.sourceFormID, r.kind, r.baseDamage);
        }
        r.cause = probe::describeRef(cause);
        if (r.sourceId != 0) {
            r.cause += " with " + probe::describeFormId(r.sourceId);
        }
        if (g_config.logHitProbe) {
            const char* material = event.material.c_str();
            logger::info("HitProbe #{}: {} {} hit by {} (level {}), {} base {:.0f}, projectile {}, material '{}'{}",
                n,
                r.onPlayer ? "PLAYER" : "VEHICLE",
                probe::describeRef(target),
                r.cause,
                r.attackerLevel,
                kindName(r.kind),
                r.baseDamage,
                probe::describeFormId(event.projectileFormID),
                material ? material : "",
                details);
        }
        {
            const std::scoped_lock lock(_mutex);
            if (_queue.size() < MAX_QUEUE) {
                _queue.push_back(std::move(r));
            }
        }
    }

    void HitSink::onFrameUpdate(Vehicle* vehicle, const bool seated)
    {
        std::vector<Record> batch;
        {
            const std::scoped_lock lock(_mutex);
            batch.swap(_queue);
        }
        if (!vehicle || !vehicle->isValid() || !g_config.vehicleHealth) {
            _healthWatched = false;
            return;
        }
        const auto& spec = vehicle->spec().damage;
        auto& health = vehicle->health();
        const model::DamageScaling scaling{
            .playerLevelFactor = g_config.playerLevelFactor,
            .playerLevelCap = g_config.playerLevelCap,
            .attackerLevelFactor = g_config.attackerLevelFactor,
            .attackerLevelCap = g_config.attackerLevelCap,
            .scale = g_config.vehicleDamageScale,
        };
        const int playerLevel = probe::levelOf(RE::PlayerCharacter::GetSingleton());
        const double now = nowSeconds();
        // The attackers with a weapon or explosion hit in this batch: the effect event of the same attack may come before or after it.
        const auto hasAttackInBatch = [&batch](const std::uint32_t cause) {
            return std::ranges::any_of(batch, [cause](const Record& o) { return o.causeId == cause && o.kind != model::HitKind::Unknown; });
        };
        for (const auto& r : batch) {
            // Who the hit was for: the covered driver (the vehicle takes it) or the hull itself.
            const bool coveredHit = r.onPlayer && seated && spec.occupantProtected;
            const bool hullHit = r.onVehicle && (seated || spec.hitsWhileParked);
            if (!coveredHit && !hullHit) {
                if (g_config.logHitProbe) {
                    logger::info("VehicleHealth: hit on {} by {} not counted ({})", r.onPlayer ? "the player" : "the hull", r.cause,
                        r.onPlayer ? (seated ? "the driver is not covered in this vehicle" : "nobody is driving") : "parked, and hits while parked do not count");
                }
                continue;
            }
            if (r.byPlayer && !spec.selfDamage) {
                if (g_config.logHitProbe) {
                    logger::info("VehicleHealth: the player's own hit on the hull not counted (selfDamage off)");
                }
                continue;
            }
            if (r.kind == model::HitKind::Explosion && !health.noteExplosion(r.causeId, r.sourceId, now)) {
                if (g_config.logHitProbe) {
                    logger::info("VehicleHealth: the same explosion reached {} too: counted once", coveredHit ? "the driver" : "the hull");
                }
                continue;
            }
            // An effect without damage of its own that came with an attacker's weapon hit (a bloodbug's infection with its bite) is the same
            // attack: the bite already counted.
            if (r.kind == model::HitKind::Unknown && r.baseDamage <= 0.0f && r.causeId != 0 &&
                (hasAttackInBatch(r.causeId) || (r.causeId == _lastAttackCause && now - _lastAttackSeconds <= SAME_ATTACK_SECONDS))) {
                if (g_config.logHitProbe) {
                    logger::info("VehicleHealth: {} came with that attacker's weapon hit: counted once", r.cause);
                }
                continue;
            }
            if (r.kind != model::HitKind::Unknown) {
                _lastAttackCause = r.causeId;
                _lastAttackSeconds = now;
            }
            // The engine's number is zero for the covered driver (the perk multiplies it by zero): then the base damage stands in any mode.
            const float base = g_config.vehicleDamageBase == 1 && r.engineDamage > 0.0f ? r.engineDamage : r.baseDamage;
            const model::HitInput hit{ .kind = r.kind, .baseDamage = base, .playerLevel = playerLevel, .attackerLevel = r.attackerLevel };
            const bool wasDisabled = health.disabled();
            const float taken = health.applyHit(hit, scaling);
            if (coveredHit) {
                _lastCoveredHitSeconds = now;
            }
            logger::info("VehicleHealth: {} hit on {} by {} (level {}), base {:.0f} -> {:.1f} damage: health {:.0f} / {:.0f}{}",
                kindName(r.kind),
                coveredHit ? "the driver" : "the hull",
                r.cause,
                r.attackerLevel,
                base,
                taken,
                health.health(),
                health.maxHealth(),
                !wasDisabled && health.disabled() ? " - DISABLED" : "");
        }
        watchDriverHealth(*vehicle, seated, scaling, playerLevel, now);
    }

    // [game] The covered driver's health: what the perk let through of a hit is given back, and the hit is charged to the vehicle when no hit
    // event brought it (the engine raises none for a projectile on the seated player). Nothing while dead: the game reloads.
    void HitSink::watchDriverHealth(Vehicle& vehicle, const bool seated, const model::DamageScaling& scaling, const int playerLevel, const double now)
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        const auto* avs = RE::ActorValue::GetSingleton();
        const bool covered = seated && vehicle.spec().damage.occupantProtected && OccupantShield::get().active();
        if (!covered || !player || !avs || !avs->health) {
            _healthWatched = false;
            return;
        }
        const float damage = player->GetModifier(RE::ACTOR_VALUE_MODIFIER::kDamage, *avs->health); // <= 0
        if (!_healthWatched) {
            _healthWatched = true;
            _healthDamageSeen = damage;
            return;
        }
        const float lost = _healthDamageSeen - damage;
        _healthDamageSeen = damage;
        if (lost <= 0.05f || player->GetActorValue(*avs->health) <= 0.0f) {
            return;
        }
        player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kDamage, *avs->health, lost);
        _healthDamageSeen = player->GetModifier(RE::ACTOR_VALUE_MODIFIER::kDamage, *avs->health);
        if (now - _lastCoveredHitSeconds <= SAME_ATTACK_SECONDS) {
            logger::info("VehicleHealth: the driver lost {:.1f} health to the hit just counted: given back", lost);
            return;
        }
        const float base = lost / DRIVER_COVER_FRACTION;
        const model::HitInput hit{ .kind = model::HitKind::Ballistic, .baseDamage = base, .playerLevel = playerLevel, .attackerLevel = playerLevel };
        auto& health = vehicle.health();
        const bool wasDisabled = health.disabled();
        const float taken = health.applyHit(hit, scaling);
        logger::info("VehicleHealth: hit on the driver without a hit event ({:.1f} health lost and given back), base {:.0f} -> {:.1f} damage: health {:.0f} / {:.0f}{}", lost,
            base, taken, health.health(), health.maxHealth(), !wasDisabled && health.disabled() ? " - DISABLED" : "");
    }
}
