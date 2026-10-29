#pragma once

#include <string>
#include <vector>

#include "model/VMath.h"

namespace vfvr::model
{
    // A turret type (Turrets\<id>.json): its model, the nodes that turn, how far and how fast they turn, and what it fires. A vehicle only says
    // where turrets can be mounted (TurretMountSpec); which turret sits on a mount is the player's choice (a kit made at a settlement).
    struct TurretSpec
    {
        std::string id;
        std::string displayName;
        std::string mesh; // NIF under Meshes\, loaded and attached to the vehicle's node at the mount
        // Nodes in the mesh: yawNode turns about its z, pitchNode (a child of yawNode) about its x. muzzleNodes are where the shots leave
        // (y = along the barrel), taken in turn shot after shot ("muzzle": a name or a list of names); without any, muzzleOffset (game units, in
        // the pitch node's frame) is used.
        std::string yawNode;
        std::string pitchNode;
        std::vector<std::string> muzzleNodes;
        Vec3 muzzleOffset{ 0.0f, 0.8f, 0.0f }; // m
        float meshScale = 1.0f;
        Vec3 meshOffset{}; // m, in the mount's frame: where the mesh's origin goes from the mount point (a mesh whose base is above its origin)
        // Turning, degrees relative to the mount's forward. Every turret turns 360 degrees and -30..+90 up and down (not read from the JSON: the
        // same for all); yawMin <= -180 and yawMax >= 180 = turns all the way round.
        float yawMinDeg = -180.0f;
        float yawMaxDeg = 180.0f;
        float pitchMinDeg = -30.0f;
        float pitchMaxDeg = 90.0f;
        float yawSpeedDeg = 120.0f; // deg/s
        float pitchSpeedDeg = 90.0f;
        // Shots only leave while the barrel is within this angle of the aim (0 = always, wherever it points).
        float fireConeDeg = 0.0f;
        // What it fires: a weapon form (its projectile, damage and sounds) and the ammo used per shot from the player's inventory, then the
        // vehicle's trunk ("" = no ammo needed). projectileForm overrides the weapon's projectile. The damage is
        // the weapon's own.
        std::string weaponForm;
        std::string projectileForm;
        std::string ammoForm;
        int ammoPerShot = 1;
        float roundsPerMinute = 600.0f;
        float spreadDeg = 1.5f; // half angle of the cone the shots scatter in
        std::string fireSound; // a sound descriptor form; "" = the weapon's own
        // The fire sound is a loop played while the trigger is held (an automatic weapon's, such as the vanilla turret's 420 rpm loop), or, with
        // false, a single shot played for every shot (a cannon, a rocket).
        bool fireSoundLoop = true;
        std::string kitForm; // the MISC item made at a settlement workbench that puts this turret on a vehicle
    };

    // Where a vehicle carries a turret.
    struct TurretMountSpec
    {
        std::string id = "roof";
        std::string node; // the vehicle mesh node the turret hangs under ("" = the root)
        Vec3 position{ 0.0f, 0.0f, 1.6f }; // vehicle frame, m
        float yawDeg = 0.0f; // the mount's forward, degrees clockwise from the vehicle's forward
        std::string seat = "driver"; // who aims it: "driver" (head) or "passenger" (a companion, later)
        std::string defaultTurret; // a turret id mounted when the vehicle first appears ("" = none until a kit is fitted)
    };

    TurretSpec parseTurretSpec(const std::string& jsonText);
    TurretSpec loadTurretSpecFile(const std::string& path);

    /**
     * How a turret's two joints follow an aim direction (pure C++). Angles are radians in the mount's frame: yaw 0 = the mount's forward
     * (+y), positive = clockwise from above, like headings; pitch positive = up. The joints move at most yawSpeed / pitchSpeed per second, take
     * the short way round on a free turret and stop at the limits of a limited one.
     */
    class TurretAim
    {
    public:
        TurretAim() = default;
        explicit TurretAim(const TurretSpec& spec);

        float yaw() const
        {
            return _yaw;
        }

        float pitch() const
        {
            return _pitch;
        }

        bool freeYaw() const
        {
            return _freeYaw;
        }

        // The yaw/pitch that point along `dir` (mount frame; need not be unit), clamped to the limits.
        void anglesFor(const Vec3& dir, float& yaw, float& pitch) const;

        // Turn towards `dir` for `dt` seconds. Returns true when the barrel is now within the fire cone of the aim (always true with no cone).
        bool update(const Vec3& dir, float dt);

        // Back to the rest position (forward, level) at the joints' speed: used when nobody aims it.
        void relax(float dt);

        // The barrel's direction in the mount frame.
        Vec3 barrel() const;

        // Angle between the barrel and `dir`, radians.
        float errorTo(const Vec3& dir) const;

        void snap(float yaw, float pitch);

    private:
        void stepTowards(float yaw, float pitch, float dt);

        float _yaw = 0.0f;
        float _pitch = 0.0f;
        float _yawMin = -PI;
        float _yawMax = PI;
        float _pitchMin = -0.17f;
        float _pitchMax = 0.78f;
        float _yawSpeed = 2.0f;
        float _pitchSpeed = 1.5f;
        float _fireCone = 0.0f;
        bool _freeYaw = true;
    };

    // Rate of fire: how many shots leave this frame while the trigger is held. A held trigger fires at once, then every 60/rpm s; letting go
    // keeps the cooldown, so tapping does not fire faster than holding.
    class FireControl
    {
    public:
        FireControl() = default;
        explicit FireControl(float roundsPerMinute);

        int update(bool held, float dt);

    private:
        float _interval = 0.1f;
        float _cooldown = 0.0f;
    };

    // A shot's direction: `barrel` turned off axis by up to `spreadRad`, from two uniform numbers in [0,1) (the glue's random generator).
    Vec3 scatter(const Vec3& barrel, float spreadRad, float u1, float u2);

    // The launch angles the engine wants for a direction in the game's world frame (x east, y north, z up): heading (zAngle, clockwise
    // from north) and pitch (xAngle, positive = DOWN, as the engine's actors use).
    void launchAngles(const Vec3& dir, float& zAngle, float& xAngle);
}
