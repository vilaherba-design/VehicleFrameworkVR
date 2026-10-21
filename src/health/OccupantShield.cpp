#include "health/OccupantShield.h"

#include <format>

#include "health/HealthStore.h"
#include "physics/HavokBridge.h"

namespace vfvr
{
    namespace
    {
        constexpr auto PLUGIN_FILE = "VehicleFrameworkVR_Vehicles.esp";
        constexpr RE::TESFormID GLOBAL_ID = 0x806; // VFVR_DriverCovered (tools/make_vehicle_esp.py)
        constexpr RE::TESFormID PERK_ID = 0x807; // VFVR_DriverCover
        constexpr std::uint64_t ADD_PERK_ID = 187096; // Actor::AddPerk
        constexpr std::uint64_t GET_PERK_RANK_ID = 1368313; // Actor::GetPerkRank

        template <class T>
        T* lookupForm(const RE::TESFormID id, const RE::ENUM_FORM_ID type)
        {
            auto* handler = RE::TESDataHandler::GetSingleton();
            auto* form = handler ? handler->LookupForm(id, PLUGIN_FILE) : nullptr;
            return form && form->GetFormType() == type ? static_cast<T*>(form) : nullptr;
        }

        RE::TESNPC* playerBase()
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            return player ? player->GetNPC() : nullptr;
        }

        bool hasPerkApi()
        {
            static const bool has = physics::hasAddressId(ADD_PERK_ID) && physics::hasAddressId(GET_PERK_RANK_ID);
            return has;
        }
    }

    OccupantShield& OccupantShield::get()
    {
        static OccupantShield instance;
        return instance;
    }

    bool OccupantShield::resolveForms()
    {
        if (_formsResolved) {
            return !_formsMissing;
        }
        _formsResolved = true;
        _global = lookupForm<RE::TESGlobal>(GLOBAL_ID, RE::ENUM_FORM_ID::kGLOB);
        _perk = lookupForm<RE::BGSPerk>(PERK_ID, RE::ENUM_FORM_ID::kPERK);
        _formsMissing = !_global || !_perk;
        if (_formsMissing) {
            logger::error("OccupantShield: {} has no global 0x{:X} / perk 0x{:X} (is the plugin enabled, and from this build?): the driver is not protected",
                PLUGIN_FILE, GLOBAL_ID, PERK_ID);
        }
        return !_formsMissing;
    }

    void OccupantShield::update(const bool covered, const int mode)
    {
        const auto wanted = covered ? static_cast<Mode>(mode) : Mode::None;
        if (wanted == Mode::None) {
            if (_active) {
                clear("left the seat");
            }
            return;
        }
        if (wanted != Mode::DriverCoverPerk) {
            if (_warnedMode != mode) {
                _warnedMode = mode;
                logger::warn("OccupantShield: unknown mode {} (iOccupantProtection), the player is not protected", mode);
            }
            return;
        }
        if (_active) {
            return;
        }
        if (!resolveForms()) {
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            return;
        }
        if (!hasPerkApi()) {
            if (!_apiWarned) {
                _apiWarned = true;
                logger::error("OccupantShield: Actor::AddPerk / GetPerkRank (address ids {}, {}) are not in the VR address library: the driver is not protected", ADD_PERK_ID,
                    GET_PERK_RANK_ID);
            }
            return;
        }
        // The perk once per save: the save game keeps it, and the co-save remembers that it was given (the rank the engine reports is logged,
        // not relied on). The entries are inert while the global is 0, so a player with the perk and no vehicle loses nothing.
        if (!_perkChecked) {
            _perkChecked = true;
            const int before = player->GetPerkRank(_perk);
            if (!HealthStore::get().driverPerkGiven()) {
                player->AddPerk(_perk, 0);
                HealthStore::get().markDriverPerkGiven();
                logger::info("OccupantShield: perk VFVR_DriverCover given to the player (rank {} -> {})", before, player->GetPerkRank(_perk));
            } else {
                logger::info("OccupantShield: perk VFVR_DriverCover already given in this save (rank {})", before);
            }
        }
        _global->value = 1.0f;
        _active = true;
        _mode = wanted;
        logger::info("OccupantShield: ON (perk VFVR_DriverCover, global VFVR_DriverCovered = 1); {}", describePlayer());
    }

    void OccupantShield::clear(const char* reason)
    {
        if (!_active) {
            return;
        }
        if (_global) {
            _global->value = 0.0f;
        }
        _active = false;
        _mode = Mode::None;
        logger::info("OccupantShield: OFF ({}); {}", reason, describePlayer());
    }

    void OccupantShield::onSessionLoaded()
    {
        _active = false;
        _mode = Mode::None;
        _formsResolved = false;
        _formsMissing = false;
        _perkChecked = false;
        _global = nullptr;
        _perk = nullptr;
        if (resolveForms() && _global->value != 0.0f) {
            _global->value = 0.0f;
            logger::info("OccupantShield: the global VFVR_DriverCovered was {} in the save (saved while covered), put back to 0", _global->value);
        }
        // The Invulnerable flag of the earlier builds (it never protected the player): off if a save or a crash left it on.
        if (auto* npc = playerBase(); npc && npc->actorData.actorBaseFlags.all(RE::ACTOR_BASE_DATA::Flag::kInvulnerable)) {
            npc->actorData.actorBaseFlags.reset(RE::ACTOR_BASE_DATA::Flag::kInvulnerable);
            logger::warn("OccupantShield: the Invulnerable flag of an earlier build was on the player's base NPC, taken off");
        }
    }

    std::string OccupantShield::describePlayer()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            return "player: none";
        }
        auto& self = get();
        static const bool hasGodMode = physics::hasAddressId(1032309); // PlayerCharacter::IsGodMode
        const std::string rank = self._perk && hasPerkApi() ? std::format("{}", player->GetPerkRank(self._perk)) : "n/a";
        const std::string global = self._global ? std::format("{:.0f}", self._global->value) : "n/a";
        return std::format("perk VFVR_DriverCover rank {}, global VFVR_DriverCovered {}, god mode {}", rank, global, hasGodMode ? (player->IsGodMode() ? "on" : "off") : "unknown");
    }
}
