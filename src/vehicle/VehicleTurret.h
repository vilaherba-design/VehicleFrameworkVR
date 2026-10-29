#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "model/Turret.h"
#include "vehicle/HudState.h"

namespace vfvr
{
    class Vehicle;

    /**
     * Vehicle turrets (bTurrets). A turret type is a JSON file in Turrets\ (model::TurretSpec): its mesh, the two nodes that turn (TurretYaw about
     * its z, TurretPitch about its x), how fast they turn, and the weapon whose projectile it fires. A vehicle carries one turret on its roof mount
     * (the profile's "turrets" block, or one placed on top of the model above the seat). The mesh is loaded once and hung under the vehicle's model
     * root, so it rides with the car without being moved by us; only the two joints' local rotations are written each frame.
     *
     * The driver aims with the head: the turret turns, at its own speed, towards the point the line of sight meets (fTurretAimDistance when it
     * meets nothing) and fires while sTurretFireBinding is held. Each shot is the engine's own projectile launch (Projectile::Launch, address
     * library id 1452334) with the player as the shooter and the turret's weapon and ammo, so the game counts the hits, the kills and the combat as
     * the player's. Each shot takes ammoPerShot of the turret's ammo from the inventory, then from the vehicle's trunk.
     *
     * A turret is made as a kit (a MISC item) at a settlement's chemistry station and fitted from the vehicle's menu; the kit comes back when it
     * is taken off. The turret menu after a vehicle is spawned (bTurretMenuAfterSpawn) fits any type without a kit, like the wheels; such a turret
     * gives no kit back. Which turret each profile carries lives in the co-save (HealthStore; a free one is stored as "*<id>"). Nothing goes
     * through Papyrus per frame: an automatic weapon's fire sound is a loop started and stopped by one Papyrus call each per burst, a single-shot
     * weapon's sound one call per shot (VehicleFrameworkVR:Turret). A turret with several muzzles fires from them in turn.
     */
    class VehicleTurret
    {
    public:
        static VehicleTurret& get();

        // Read every Turrets\*.json (again). [game]
        void loadCatalog(const std::string& folder);
        const std::vector<model::TurretSpec>& catalog() const
        {
            return _catalog;
        }

        // [game] once per frame. `driving`: the player is at the wheel of `vehicle` (null when there is none).
        void onFrameUpdate(Vehicle* vehicle, bool driving);

        // A vehicle was bound: fit the turret its profile carries in the save (or bTurretFreeKit's). [game]
        void restore(Vehicle& vehicle);
        // The vehicle is going away: take the mesh off its model. [game]
        void release();

        // Put the turret `turretId` on the vehicle, taking its kit from the inventory or the trunk (unless bTurretFreeKit or !useKit: the
        // spawn menu's free choice). Shows a notification and returns false when it cannot. [game]
        bool fit(Vehicle& vehicle, const std::string& turretId, bool useKit = true);
        // Take the turret off and give its kit back (a turret fitted without a kit gives none). [game]
        void remove(Vehicle& vehicle);
        // The turret fitted now ("" = none).
        std::string fittedId() const;
        // Turret types whose kit the player has (or all with bTurretFreeKit), for the menu. [game]
        std::vector<const model::TurretSpec*> availableKits(Vehicle& vehicle) const;

        // The fire loop Turret.StartFire started (its instance). [any] from the Papyrus VM.
        void onFireSoundStarted(std::int32_t instance);

        // The weapon icon, its ammo and the sight. [game]
        void fillHud(HudState& state) const;

    private:
        VehicleTurret() = default;

        const model::TurretSpec* find(const std::string& id) const;
        bool attach(Vehicle& vehicle);
        void detach();
        model::TurretMountSpec mountFor(const Vehicle& vehicle) const;
        void aimAndFire(Vehicle& vehicle, bool driving, float dt);
        void writeJoints();
        void launch(Vehicle& vehicle, const model::Vec3& muzzle, const model::Vec3& dir);
        int ammoCount(Vehicle& vehicle) const;
        bool takeAmmo(Vehicle& vehicle, int count);
        void startFireSound();
        void playShotSound();
        void stopFireSound();

        std::vector<model::TurretSpec> _catalog;
        std::string _catalogFolder;

        // The fitted turret.
        std::optional<model::TurretSpec> _spec;
        model::TurretMountSpec _mount;
        model::TurretAim _aim;
        model::FireControl _fire;
        std::string _profile; // the vehicle profile it is fitted to
        bool _free = false; // fitted without a kit (the spawn menu): taking it off gives no kit
        RE::NiPointer<RE::NiNode> _mesh;
        RE::NiAVObject* _parent = nullptr; // the model root it hangs under
        RE::NiPointer<RE::NiAVObject> _yawNode;
        RE::NiPointer<RE::NiAVObject> _pitchNode;
        std::vector<RE::NiPointer<RE::NiAVObject>> _muzzleNodes;
        std::size_t _nextMuzzle = 0; // the muzzle the next shot leaves from
        RE::NiMatrix3 _yawRest{};
        RE::NiMatrix3 _pitchRest{};
        bool _attachFailed = false;

        // Fire.
        bool _firing = false;
        int _burstShots = 0;
        int _ammo = 0; // what the HUD shows: inventory + trunk
        float _ammoTimer = 0.0f;
        bool _noAmmoNotified = false;
        bool _launchWarned = false;
        std::mt19937 _rng{ 0x5EED };
        // The fire loop: asked for while shots leave; its instance comes back from the VM a little later.
        std::atomic<std::int32_t> _soundInstance{ 0 };
        std::atomic<bool> _soundWanted{ false };
        bool _soundOn = false;

        // Sight: where the barrel points (world, meters).
        bool _sightValid = false;
        model::Vec3 _sight;

        std::chrono::steady_clock::time_point _last{};
        bool _clockValid = false;
    };
}
