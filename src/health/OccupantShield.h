#pragma once

#include <string>

namespace vfvr
{
    /**
     * Protection of the player in the driver's seat: the engine must not take the player's health for the hits that reach the seat; those
     * hits are the vehicle's (see HitSink and VehicleHealth).
     *
     * Mode 1 uses the game's own perk system: the plugin's ESP carries the hidden perk VFVR_DriverCover, whose entries take away the incoming
     * weapon and explosion damage, the limb (crippling) damage, the falling damage and the magnitude of incoming poison, acid, fire, cryo,
     * electrical and energy effects while the global VFVR_DriverCovered is 1 (a GetGlobalValue condition on the perk owner; the effects by
     * their DamageType keyword, the way the game's ImmuneToPoison does it; see tools/make_vehicle_esp.py). The perk is given to the player once per save (the save keeps it; its entries are inert while the global is 0) and
     * the global is written in memory every time the cover changes: 1 while covered, 0 otherwise, on leaving the seat and after a load.
     * The hits still land, the hit events still fire and the enemies' AI sees nothing different. Radiation is not in the perk (the cabin's
     * RadProtection handles the rads), nor are effects without one of those keywords, nor anything already on the player before getting in.
     *
     * The Invulnerable flag of earlier builds does not stop the player's damage in Fallout 4 VR (seen in a test log) and is no longer used; a
     * flag left on the player by one of those builds is taken off after a load.
     */
    class OccupantShield
    {
    public:
        enum class Mode : int
        {
            None = 0,
            DriverCoverPerk = 1,
        };

        static OccupantShield& get();

        // Keep the protection in step with the seat: on while `covered` (in the `mode` of iOccupantProtection), off otherwise. Every frame. [game]
        void update(bool covered, int mode);

        // Take the protection off whatever the state (seat exit, vehicle released). [game]
        void clear(const char* reason);

        // After a save load: forms are looked up again, the global is put back to 0 (a save made while covered keeps 1), the perk is checked
        // again, and a stray Invulnerable flag is taken off. [game]
        void onSessionLoaded();

        bool active() const
        {
            return _active;
        }

        // The protection state of the player, for the log. [game]
        static std::string describePlayer();

    private:
        bool resolveForms();

        bool _active = false;
        Mode _mode = Mode::None;
        int _warnedMode = -1;
        bool _formsResolved = false;
        bool _formsMissing = false;
        bool _perkChecked = false;
        bool _apiWarned = false;
        RE::TESGlobal* _global = nullptr;
        RE::BGSPerk* _perk = nullptr;
    };
}
