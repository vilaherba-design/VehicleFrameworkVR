#pragma once

#include "ConfigBase.h"
#include "Resources.h"
#include "common/CommonUtils.h"
#include "vrcf/VRControllersManager.h"

namespace vfvr
{
    static const auto BASE_MOD_PATH = BASE_PATH + "\\" + std::string(Version::PROJECT);
    static const auto INI_PATH = BASE_MOD_PATH + "\\" + std::string(Version::PROJECT) + ".ini";

    // Where the seat (and so the camera) takes the vehicle's pose from.
    enum class SeatPoseSource : std::uint8_t
    {
        Node = 0, // the carrier node's world transform, as the engine last synced it (it stopped following the body in one of the test sessions)
        HavokBody = 1, // the Havok body transform: the newest physics state, the same one the model is drawn from
    };

    // Where in the frame the driver is put in the seat.
    enum class CameraUpdatePoint : std::uint8_t
    {
        ModFrame = 0, // this plugin's own frame update
        FrikFrameBegin = 1, // FRIK's FrameBegin phase, right before FRIK places the body (falls back to ModFrame without FRIK)
    };

    // Which engine call applies the wheel impulses.
    enum class ImpulseApi : std::uint8_t
    {
        HknpWorld = 0, // hknpWorld::ApplyBodyImpulseAt
        CollisionObject = 1, // bhkNPCollisionObject::ApplyPointImpulseAt (Bethesda wrapper)
    };

    class Config : public ConfigBase
    {
    public:
        explicit Config()
            : ConfigBase(Version::PROJECT, INI_PATH, IDR_CONFIG_INI)
        {}

        // [VehicleFrameworkVR] - vehicles
        std::string vehiclesFolder = "Data\\F4SE\\Plugins\\VehicleFrameworkVR\\Vehicles";
        std::string testVehicleProfile = "sedan";
        float spawnDistance = 6.0f; // m in front of the player
        bool simpleMode = false;
        ImpulseApi impulseApi = ImpulseApi::HknpWorld;
        std::string wheelRayLayer = "charController";
        // Layer of the vehicle body. ROCK's physical hands collide with most layers, so the hull around the cabin would push the arms; "" keeps the original layer.
        std::string vehicleBodyLayer = "charController";
        // Layer of the body while parked (empty = the layer the engine gave it). Lets the player's collision with a parked car be tried per layer.
        std::string parkedBodyLayer;
        std::uint64_t wheelRayIgnoreMask = 0;
        bool releasePickWorldReference = true;
        // Wheel set the vehicle starts with ("stock", "offroad", "beadlock"), and whether a menu asks for it each time you get in.
        std::string wheelSet = "stock";
        bool wheelMenuOnEnter = false;
        bool wheelMenuAfterSpawn = true;
        std::string vehicleProfiles = "sedan, pickup, rockboy, rusianmother";
        bool exhaustSmoke = true; // smoke puffs from the tailpipe while driving
        bool engineSound = true; // synthesised engine hum while driving
        float engineVolume = 0.6f; // 0..1

        // Driving and camera
        float enterDistance = 2.5f; // m from the head to the driver's seat
        // Left stick pulled down past this share of its travel holds the handbrake (0 = off).
        float handbrakeStickThreshold = 0.7f;
        // Which stick pulls the handbrake: the right one (the primary hand's) or the left one (the steering stick).
        vrcf::Hand handbrakeStick = vrcf::Hand::Primary;
        // Seconds the camera takes to glide from where the player stands into the seat on getting in (0 = jump straight in).
        float enterGlideSeconds = 0.4f;
        float deadzone = 0.08f;
        float responseCurve = 1.4f; // exponent applied to triggers and stick after the deadzone
        SeatPoseSource seatPoseSource = SeatPoseSource::HavokBody;
        CameraUpdatePoint cameraUpdatePoint = CameraUpdatePoint::FrikFrameBegin;
        float roomYawSign = 1.0f;
        bool autoDetectYawSign = true;
        float verticalFilterHz = 4.0f; // low-pass on the seat height; 0 disables
        float verticalFilterMax = 0.04f; // m, the filtered seat never strays further than this from the real one
        bool disablePlayerCollision = true;
        // While driving, take the gravity off the player's character controller and zero its velocity (the engine otherwise sinks the player each frame).
        bool playerNoGravity = true;
        // Draw the vehicle model where the engine really has the camera: the engine moves it only on its next update, one frame after the seat placed it.
        bool lockModelToCamera = true;
        int modelLagFrames = 1;
    float cabinTiltScale = 0.4f;
    bool renderProbe = false;
    bool drawBodyNode = true;
        // Keep the model's previous-frame transforms (the temporal upscaler's motion vectors are built from them) in step with the hand-moved model.
        bool modelPreviousWorld = true;
        std::string driverCollisionLayer = "nonCollidable";
        bool blockFrikSmoothMovement = true;

        // Bindings
        vrcf::InputBinding spawnTestVehicleBinding;
        vrcf::InputBinding enterBinding;
        vrcf::InputBinding trunkBinding;
        vrcf::InputBinding exitBinding;
        vrcf::InputBinding recenterBinding;
        vrcf::InputBinding thirdPersonBinding;
        vrcf::InputBinding lightsBinding;
        float thirdPersonDistance = 11.0f; // m behind the seat
        float thirdPersonHeight = 3.5f; // m above the seat
        float thirdPersonOrbitSpeed = 120.0f; // deg/s at full right-stick deflection (0 disables the orbit)
        bool thirdPersonOrbitInvert = true; // the chase camera orbits the other way round for the same stick direction
        // Third person: the player's body stays in the driver's seat (moved there from the chase camera once FRIK has built it) instead of hidden.
        bool thirdPersonBodyInSeat = true;
        // Its legs: "pose" (bent into a seated pose: thighs fSeatThighAngleDeg from straight down toward the front, shins fSeatKneeAngleDeg back
        // down at the knee, knees fSeatKneeSpreadDeg apart), "hide" (collapsed into the hips), "none" (as FRIK stands them).
        std::string thirdPersonLegs = "pose";
        float seatThighAngleDeg = 80.0f;
        float seatKneeAngleDeg = 90.0f;
        float seatKneeSpreadDeg = 8.0f;
        // The driver does not shoot: the weapon is holstered while driving (drawn again on getting out if it was out on getting in).
        bool holsterWeaponWhileDriving = true;
        float thirdPersonRecenterDelay = 3.0f; // s after releasing the stick before the camera swings back behind the car (0 = stay)
        // The chase camera stops short of walls, rocks and the ground between it and the car: this far in front of what it hits (m), never closer
        // to its pivot above the seat than fCameraMinDistance.
        bool cameraCollision = true;
        float cameraCollisionMargin = 0.3f;
        float cameraMinDistance = 2.0f;
        // Switching between first and third person slides the camera between the seat and the chase spot over this many seconds (0 = cut over).
        float cameraSwitchSeconds = 0.6f;

        // Driving HUD: speedometer with the action and weapon icons, fHudDistance m in front of the eyes and fHudDownAngle deg below the line of sight.
        bool hud = true;
        float hudDistance = 1.0f;
        float hudDownAngle = 25.0f;
        float hudScale = 1.0f;
        float hudFollowSeconds = 0.2f;
        // Behind the car (third person) the panel sits lower, out of the middle of the view.
        float hudDistanceThirdPerson = 1.0f;
        float hudDownAngleThirdPerson = 40.0f;

        // Smooth rendering: draw the car (and the seat camera) at a steady time slightly behind the physics instead of at the
        // raw, irregularly stepped state of each frame.
        bool renderInterpolation = true;

        // Extra height of the seat camera over every vehicle's driverEye, in centimetres.
        float eyeLiftCm = 15.0f;

        // Multiplies every vehicle's engine torque curve.
        float torqueScale = 1.3f;
        float renderDelayMs = 12.0f;

        // Self-righting: seconds the car must stay on its side or roof before it rolls itself back onto its wheels (0 = off).
        float rightAfterSeconds = 3.0f;
        // A car that falls through the world (ground found above it) more than this many meters below where it last rested on its wheels is put back
        // there (0 = only log it).
        float fallRecoverMeters = 3.0f;

        // Diagnostics
        bool telemetry = true;
        float telemetryMaxMinutes = 10.0f;
        bool debugDraw = false;
        bool costReport = true;
        // Log the player's body (3D shown or hidden, its draw bound, FRIK's skeleton, vehicle / seat / zones) every 2 s and on every change.
        bool logPlayerBody = true;
        // Two-wheelers: a CSV row per physics step (VehicleFrameworkVR_moto.csv) and log lines for the front wheel in the air, a big lean or a fast yaw.
        bool logMotoDiag = true;
        // A physics step whose velocity change the model cannot explain (m/s) is a collision of the hull: it is logged with what the
        // world holds around the car (0 = off).
        float impactLogMps = 2.0f;

        // What protects the player in the driver's seat from the enemies' fire: 0 = nothing, 1 = the game's own perk VFVR_DriverCover (from the
        // plugin's ESP), whose entries multiply the incoming weapon and explosion damage by zero while the global VFVR_DriverCovered is 1 (hits
        // still land, and become vehicle damage, but take no health).
        int occupantProtection = 1;
        // Log every hit on the player, by the player or on the vehicle ("HitProbe:" lines).
        bool logHitProbe = true;
        // Log the player's vitals (health, action points, rads, rad resistance, the cover perk, god mode) every 2 s while driving, 10 s otherwise,
        // and on change ("Vitals:" lines).
        bool logVitals = true;

        // Health system. bVehicleHealth switches all of it: hits and collisions damage the vehicle, the covered driver takes none of the enemies'
        // fire, the cabin's radiation shielding, the disabled state, the repair and the HUD bars (the vehicle's health and shielding, the
        // player's health with the rads, and action points).
        bool vehicleHealth = true;
        float vehicleDamageScale = 1.0f; // multiplies every hit
        // Level scaling of the hits (see VehicleHealth): min(1 + fPlayerLevelFactor * (player level - 1), fPlayerLevelCap), the same for the attacker.
        float playerLevelFactor = 0.02f;
        float playerLevelCap = 3.0f;
        float attackerLevelFactor = 0.01f;
        float attackerLevelCap = 2.0f;
        // What a hit's base damage is: 0 = the weapon's or explosion's own number, 1 = what the engine computed for the hit (perks and resistances in).
        int vehicleDamageBase = 0;
        bool rememberVehicleState = true; // the save game keeps each profile's health and radiation reserve (a vehicle spawned again takes them over)
        bool damageSmoke = true;
        vrcf::InputBinding repairBinding;

    protected:
        virtual void loadIniConfigInternal(const CSimpleIniA& ini) override;
    };

    // Global singleton for easy access
    inline Config g_config;
}
