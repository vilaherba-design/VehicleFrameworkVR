#pragma once

// Data that defines one vehicle profile. Loaded from JSON (VehicleSpecJson.cpp); every field has the
// default of the reference medium sedan, so a JSON only needs the values it changes.
// Units: SI (m, kg, s, N, N*m, rad/s). Positions are in the vehicle frame: the reference's own node
// frame, x = right, y = forward, z = up, origin at the model origin.

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "model/VMath.h"

namespace vfvr::model
{
    struct WheelSpec
    {
        std::string name;
        // Hub position in the vehicle frame with the suspension fully extended (meters).
        Vec3 hub;
        bool steer = false;
        bool drive = false;
        bool handbrake = false;
        // +1 for wheels on the right side, -1 on the left (used by anti-roll); derived from hub.x when 0.
        float side = 0.0f;
        // Tyre radius of this wheel (m); 0 = the profile's `tyre.radius`. A motorcycle's front tyre is taller than the rear one.
        float radius = 0.0f;
    };

    struct SuspensionSpec
    {
        float restLength = 0.25f; // travel from full extension to full compression (m)
        float springRate = 31000.0f; // N/m
        float damperBump = 2400.0f; // N*s/m while compressing
        float damperRebound = 2900.0f; // N*s/m while extending
        float maxForce = 30000.0f; // N, keeps a hard landing from launching the car
        // Anti-roll bars, N/m of left-right compression difference. A stiffer front than rear moves load
        // transfer to the front in corners, so the front tyres reach their limit first (stable understeer).
        float antiRollFront = 14000.0f;
        float antiRollRear = 4000.0f;
        // Where the tyres' horizontal (driving, braking, cornering) forces act on the body: 0 = at the contact patch (physically exact, but a high
        // centre of mass then rolls and pitches a lot), 1 = at the height of the centre of mass (no body roll or pitch from tyre forces).
        // A value between the two is the usual arcade compromise: the car leans a little but does not tip over in a corner.
        float frictionLift = 0.0f;
        // Bump stop: extra force in the last part of the travel, so a hard bump at speed does not bottom the suspension out. It starts at `bumpStopStart`
        // (share of restLength) and, by default, is hydraulic: while compressing only, a damper that grows with the depth into the stop
        // (bumpStopDamping * compression speed * e / the remaining travel, at most twice maxForce). It soaks the hit up and never pushes the body back
        // up by itself, so it cannot bounce or launch the car. An optional progressive spring (bumpStopRate * e^2 / the remaining travel, at most half of
        // maxForce) can be added; it gives its energy back on the rebound. Below the start nothing changes.
        // bumpStopDamping < 0 = automatic (24 x damperBump), 0 = no bump stop damper; bumpStopRate 0 = no spring part (the default).
        float bumpStopStart = 0.75f;
        float bumpStopRate = 0.0f; // N/m
        float bumpStopDamping = -1.0f; // N*s/m

        // The bump stop's force at a compression (m) and compression speed (m/s, > 0 = compressing).
        float bumpStopForce(const float compression, const float compressionRate) const
        {
            const float rate = (std::max)(bumpStopRate, 0.0f);
            const float damping = bumpStopDamping < 0.0f ? 24.0f * damperBump : bumpStopDamping;
            const float start = bumpStopStart * restLength;
            const float span = restLength - start;
            if ((rate <= 0.0f && damping <= 0.0f) || span <= 1e-4f || compression <= start) {
                return 0.0f;
            }
            const float share = (compression - start) / span; // 0 at the start of the stop, 1 at full compression
            // The spring part gives its energy back on the rebound, so it stays small (half of maxForce); the damper part only takes energy out (up to twice maxForce).
            const float spring = (std::min)(rate * span * share * share, 0.5f * maxForce);
            const float damper = compressionRate > 0.0f ? (std::min)(damping * compressionRate * share, 2.0f * maxForce) : 0.0f;
            return spring + damper;
        }
    };

    struct TyreSpec
    {
        float radius = 0.33f; // m
        float wheelInertia = 1.2f; // kg*m^2 per wheel (with brake disc and half shaft)
        float gripAsphalt = 0.95f; // peak friction coefficient on the reference surface
        // Simplified Pacejka "magic formula" shape: F = mu * Fz * sin(C * atan(B * slip))
        float longB = 10.0f;
        float longC = 1.65f;
        float latB = 9.0f;
        float latC = 1.45f;
        float rollingResistance = 0.015f; // fraction of Fz
        // Below this speed the slip model blends into a simple velocity-damping model (m/s).
        float lowSpeedBlend = 2.5f;
        // Slip denominators never go below this speed, which keeps the explicit solver stable (m/s).
        float minSlipSpeed = 2.5f;
    };

    // How the engine sounds (the plugin synthesises it, see audio/EngineSound.h; "cylinders" below sets the firing pitch). Every field is optional in the
    // profile's "engine.sound" block.
    struct EngineSoundSpec
    {
        float gain = 1.0f; // this vehicle's loudness, relative (0..2)
        float pitch = 1.0f; // scales the firing frequency (0.5..2)
        float oddDecay = 0.78f; // harmonic roll-off of the odd harmonics (0.3..0.95): lower is rounder, higher is buzzier
        float evenDecay = 0.62f; // the same for the even ones
        float rumble = 0.0f; // 0..1, the half-order chug of a big, slow engine
        float lumpiness = 0.2f; // 0..1, how uneven the firing is (a V8's burble, a single's thump)
        float noise = 1.0f; // 0..2, intake and exhaust rush
        float brightness = 1.0f; // 0.3..2, tone: low-pass cutoff scale
        float whine = 0.0f; // 0..1, supercharger or turbo whine
        float whineOrder = 12.0f; // whine frequency = rpm / 60 * whineOrder
    };

    struct EngineSpec
    {
        // Torque curve: (rpm, N*m) points, linearly interpolated, flat outside the range.
        std::vector<std::pair<float, float>> torqueCurve{ { 1000.0f, 150.0f }, { 2500.0f, 200.0f }, { 4500.0f, 205.0f }, { 6000.0f, 160.0f } };
        int cylinders = 4; // shapes the pitch of the engine sound only
        float idleRpm = 900.0f;
        float maxRpm = 6200.0f;
        std::vector<float> gears{ 3.4f, 2.0f, 1.35f, 1.0f, 0.8f };
        float reverseRatio = 3.2f;
        float finalDrive = 3.7f;
        float efficiency = 0.85f;
        float shiftUpRpm = 5600.0f;
        float shiftDownRpm = 2200.0f;
        float shiftTime = 0.25f; // seconds without drive torque while shifting
        float engineBrake = 25.0f; // N*m at the crank when off throttle
        float tractionControlSlip = 0.14f; // drive torque fades out above this wheel slip ratio (0 = off)
        EngineSoundSpec sound;
    };

    struct BrakeSpec
    {
        float maxTorque = 1100.0f; // N*m per wheel at full pedal
        float frontBias = 0.65f;
        float handbrakeTorque = 1600.0f; // N*m per handbrake wheel
        // Below this speed (m/s), with no throttle, wheels lock to the ground so the car does not creep.
        float holdSpeed = 0.35f;
        // Anti-lock: release the brake on a wheel whose slip passes this ratio (0 disables ABS).
        float absSlip = 0.12f;
    };

    struct SteeringSpec
    {
        float maxAngleDeg = 34.0f;
        // Steering angle shrinks with speed: at `reduceAtSpeed` (m/s) only `reducedFactor` of max remains.
        float reduceAtSpeed = 25.0f;
        float reducedFactor = 0.22f;
        float speedDegPerSec = 220.0f; // how fast the wheels turn toward the stick
        // When > 0 the speed sensitivity is physical instead: the stick's full range is the angle that would give this lateral acceleration (in g) at the
        // current speed, atan(wheelbase * g * maxLateralG / v^2), capped by maxAngleDeg. A bike needs it: a few degrees of steering at 60 km/h is already a
        // hard corner, and the stick must never ask for more than the tyres can give.
        float maxLateralG = 0.0f;
        // When > 0 the steering backs off by itself as the vehicle's measured lateral acceleration (speed * yaw rate, in g) approaches this value: a
        // two-wheeler that is already turning as hard as its tyres allow must not be asked for more, or the stick only spins it out.
        float yawLimitG = 0.0f;
        // What is DRAWN turning (the physics only needs the angle): a motorcycle's front wheel, fork and handlebar turn about the raked steering axis
        // through `pivot` (vehicle frame, m), not about the vertical through the hub. The axis points up (and back by the rake angle); a zero axis
        // keeps the plain behaviour: each steered wheel turns about the vertical through its own hub. Nodes of the model named "Steer..." turn
        // with the wheel.
        Vec3 pivot;
        Vec3 axis;
    };

    // Roll stabiliser of a two-wheeler. A motorcycle has no side support: it stands and leans into a corner because its rider balances it. The
    // model plays the rider: it holds the bike upright at a standstill and at the lean angle a steady corner needs when it moves (the physical
    // angle atan(a / g) for the lateral acceleration `a` it measures), with a spring-damper on the roll angle. That acts as a pure torque (two
    // opposite impulses along the bike's right axis above and below the centre of mass, so it never pushes the bike sideways) and it cancels the
    // roll torque of the tyre and suspension forces first, so the lean follows the spring-damper whatever the wheels do. Off for every vehicle
    // that stands on its own.
    struct LeanSpec
    {
        bool enabled = false;
        float maxLeanDeg = 40.0f; // the largest lean it aims for (a VR driver sees the room tilt with it: keep it modest)
        float follow = 1.0f; // share of the physical lean angle it takes (1 = like a real bike, less = a calmer bike)
        float frequencyHz = 2.5f; // natural frequency of the roll response
        float damping = 0.85f; // its damping ratio
        float uprightBelow = 1.5f; // m/s: below this speed it stands upright; the lean is faded in up to twice this speed
        float armLength = 0.5f; // m: lever arm of the couple above and below the centre of mass
        float rollInertia = 0.0f; // kg*m^2 about the forward axis; 0 = from the collision box (uniform density)
        float maxTorque = 0.0f; // N*m; 0 = 1.5 * mass * g (a 1 m lever)
        float fallDeg = 65.0f; // beyond this tilt the bike is down and the stabiliser lets go
        float targetSeconds = 0.08f; // lag of the lean target behind the turn rate (s): longer = the lean builds more smoothly with the steering
        // Throttle in a corner: the drive torque is cut by this share at maxLeanDeg, ramping in from throttleCutStartDeg of lean (0 = never cut), so
        // full throttle at full lean does not spin the rear wheel out from under the bike.
        float throttleCutAtMax = 0.0f;
        float throttleCutStartDeg = 15.0f;
        // Pitch: a two-wheeler has nothing that keeps it from pitching onto its front wheel. pitchDamping (1/s) brakes the pitch rate on the ground
        // and in the air; airLevelHz (> 0) also levels it in the air with the ground it left, so it lands on both wheels. The torque is capped at
        // pitchMaxTorque (N*m; 0 = 0.6 * mass * g, a 0.6 m lever), so a hard hit can still put it over. 0 = off (the default).
        float pitchDamping = 0.0f;
        float airLevelHz = 0.0f;
        float pitchMaxTorque = 0.0f;
        float pitchInertia = 0.0f; // kg*m^2 about the right axis; 0 = from the collision box (uniform density)
    };

    struct AeroSpec
    {
        float dragCoefficient = 0.42f; // F = c * v^2 (N per (m/s)^2)
        float topSpeedKmh = 120.0f; // drive torque fades to zero approaching this speed
    };

    // The collision box chassis, in the vehicle frame. Its lower front and rear edges are cut into ramps so a kerb or a ditch edge slides
    // the car up instead of stopping it, and its lower sides are pulled in a little.
    struct HullSpec
    {
        bool explicitBox = false; // false: the object's bounds raised by hullClearance
        Vec3 min; // m
        Vec3 max; // m
        float rampLength = 0.9f; // how far the bottom edge is pulled in at both ends (m)
        float rampHeight = 0.35f; // height at which the ramp reaches the full length (m)
        float sideInset = 0.12f; // how far the bottom edge is pulled in at the sides (m)
    };

    // A choice for one slot of the vehicle (today: the wheel set). `patchJson` is a JSON merge patch (RFC 7386) applied to the profile document
    // before it is parsed, so a part can change anything a profile can: tyre size, suspension, mass, drive wheels, hub height, visuals.
    struct PartSpec
    {
        std::string slot; // "wheels"
        std::string id; // "stock", "offroad", "beadlock"
        std::string name; // shown in the menu
        std::string patchJson;
    };

    // Health of the vehicle: what it takes from the enemies' fire and from collisions, when it counts as damaged or disabled, and how it is
    // repaired. Every key is optional; the defaults are the sedan's. See VehicleHealth for the formulas.
    struct DamageSpec
    {
        float maxHealth = 400.0f;
        // Resistance per kind of hit, in the game's own terms: 100 halves the damage, 300 leaves a quarter.
        float ballisticResistance = 0.0f;
        float explosionResistance = 0.0f;
        float meleeResistance = 0.0f;
        float collisionResistance = 0.0f;
        // Multiplier per kind of hit on the attack's base damage.
        float ballisticMultiplier = 1.0f;
        float explosionMultiplier = 1.5f;
        float meleeMultiplier = 0.6f;
        float unknownAttackDamage = 10.0f; // base damage of a hit whose source gives none (a creature without a weapon)
        bool occupantProtected = true; // the driver is covered: hits on the driver are the vehicle's (false on a motorcycle)
        bool hitsWhileParked = true; // hits on the hull with nobody inside count
        bool selfDamage = true; // the player's own fire counts
        // Collisions: damage = min(factor * max(0, deltaV - minDeltaV)^exponent * 100 / (100 + collisionResistance), maxShare * maxHealth), at most
        // once per cooldownSeconds.
        float collisionMinDeltaV = 3.0f; // m/s
        float collisionFactor = 4.0f;
        float collisionExponent = 1.5f;
        float collisionMaxShare = 0.15f;
        float collisionCooldownSeconds = 0.5f;
        // Under this share of maxHealth the vehicle is damaged (a thin smoke); at zero it is disabled.
        float damagedBelow = 0.35f;
        std::string onZeroHealth = "disable"; // what zero health does; only "disable" today
        std::vector<Vec3> smokePoints; // where the damage smoke comes from (vehicle frame, meters); empty = the middle of the bonnet
        // Repair: holding the repair binding at a door handle of the parked vehicle for repairSeconds gives back repairHealthShare of the health
        // and all the radiation protection. repairItem is reserved (no cost today).
        float repairSeconds = 3.0f;
        float repairHealthShare = 1.0f;
        std::string repairItem;
        Vec3 repairPoint{}; // where the repair is done (vehicle frame, meters); without one (hasRepairPoint false) the front of the bonnet
        bool hasRepairPoint = false;
    };

    // Radiation protection of the cabin: `resistance` points of radiation resistance (a hazmat suit has 1000) are lent to the occupant while
    // the protection lasts; the protection is a reserve of `capacity` rads that the blocked radiation uses up. 0 = no protection (the default).
    struct RadiationSpec
    {
        float resistance = 0.0f;
        float capacity = 0.0f;
        float drainThreshold = 0.5f; // rads/s blocked under which nothing is used up
        float drainPerRad = 1.0f; // reserve used per rad blocked
        float recoverPerSecond = 0.0f; // reserve that comes back on its own (0 = only with a repair)
    };

    struct VehicleSpec
    {
        std::string id = "sedan";
        std::string displayName = "Sedan";
        std::string baseForm; // "Plugin.esm|0xFormID" of the base object this profile drives (its model)
        // Used when the plugin that carries `baseForm` is not loaded (the vanilla object the mod's own one is derived from).
        std::string fallbackBaseForm;
        // Physics carrier: "Plugin.esm|0xFormID" of a small dynamic object that carries the Havok body when the
        // model's own collision cannot be dynamic (vanilla cars: static mesh). Empty = the model's own body.
        std::string carrierForm;
        // Which model axis points forward: "+y" (default), "-y", "+x" or "-x". The vehicle frame is always
        // x = right, y = forward, z = up; the model is turned to match.
        std::string modelForward = "+y";
        float mass = 1400.0f; // kg
        // Collision box chassis (carrier mode): its bottom sits this far above the model bounds' bottom. High enough
        // to clear kerbs, because the wheels, not the hull, must meet them (m).
        float hullClearance = 0.35f;
        // Without a carrier: replace the model's own collision shape by the box chassis (the model's body must be simulated).
        bool boxHull = false;
        HullSpec hull;
        // Wheel meshes follow the physics wheel centre (hub + suspension compression). A tyre may rise this far above the place it has
        // in the mesh (m) before it would poke through the bodywork: the mesh wheel arches leave almost no room above the tyres.
        float wheelMaxAboveMesh = 0.0f;
        float wheelVisualLift = 0.0f; ///< m added to every drawn wheel (the tyres look sunk into the ground on some models)
        // Which wheel mesh is shown: shapes named Wheel_<corner>_<set> in the model ("stock", "offroad", "beadlock").
        std::string wheelSet = "stock";
        std::vector<WheelSpec> wheels;
        SuspensionSpec suspension;
        TyreSpec tyre;
        EngineSpec engine;
        BrakeSpec brakes;
        SteeringSpec steering;
        AeroSpec aero;
        LeanSpec lean;
        DamageSpec damage;
        RadiationSpec radiation;

        // Seat: where the driver's eyes go, in the vehicle frame (meters).
        Vec3 driverEye{ -0.37f, -0.15f, 1.12f };
        // Share of a motorcycle's lean the seat camera follows (1 = all of it, 0 = the camera stays level): the rider's head swings less than the bike.
        float seatLeanFollow = 1.0f;
        // Headlights: a spot light base object ("Plugin.esm|0xFormID"), placed at each point (vehicle frame, meters) and aimed
        // forward, pitched down by `lightAimDeg` degrees from level... see Vehicle::placeLight.
        // Exhaust: tailpipe points (vehicle frame, meters) where smoke puffs come out, and the puff object ("Plugin.esp|0xFormID").
        std::vector<Vec3> exhaustPoints;
        std::string exhaustForm = "VehicleFrameworkVR_Vehicles.esp|0x000802";
        std::string lightForm;
        std::vector<Vec3> lightPoints;
        float lightPitchDownDeg = 4.0f;
        float lightYawDeg = 90.0f; // turns the light's own shining axis (+X) onto the vehicle's forward; use -90 if they shine backwards

        // Where the player is put when leaving, in the vehicle frame (meters).
        Vec3 exitPoint{ -1.6f, 0.0f, 0.1f };
        // Door handles where the player gets in (vehicle frame, meters), and how close the hand must be to one (m). Empty = one on the driver's side, see
        // doorHandlePoints().
        std::vector<Vec3> doorHandles;
        float interactRadius = 0.9f;

        // Trunk (saddlebags on a motorcycle): opened with the hand within interactRadius of `trunkPoint` (vehicle frame, meters), holding at most
        // `trunkCapacity` of carry weight. Without a "trunk" section the vehicle has none.
        bool hasTrunk = false;
        Vec3 trunkPoint{};
        float trunkCapacity = 0.0f;

        // Parts the player can choose between, and the choice this spec was built with (slot -> part id).
        std::vector<PartSpec> parts;
        std::map<std::string, std::string> selection;

        // Auto wheel layout, used when `wheels` is empty: wheels are placed from the object bounds.
        std::string driveLayout = "rwd"; // rwd | fwd | awd
        float autoWheelInsetX = 0.18f; // m in from the bounds side
        float autoWheelbaseFactor = 0.58f; // half wheelbase as a fraction of half the bounds length
        // Hub height above the bounds bottom (m). < 0 = automatic: low enough that, with the static load on
        // the springs, the body's lowest point rides `autoGroundClearance` above the ground. Vanilla cars
        // have collision down to their tyres, so the wheels must hold the hull off the ground.
        float autoHubHeight = -1.0f;
        float autoGroundClearance = 0.04f;

        // Steepest face a wheel can stand on (degrees from level). Steeper ones (walls, cliff sides) give the wheel no ground. 78.46 deg is a ground
        // normal with z 0.2, what every version up to 0.1.67 used.
        float maxGroundSlopeDeg = 78.46304f;

        // Fills `wheels` from object bounds (vehicle frame, meters) when the profile gave none.
        // Returns true when it generated them.
        bool ensureWheels(const Vec3& boundsMin, const Vec3& boundsMax);

        // The door handles to get in at: `doorHandles`, or without them one on the driver's side (the side of the exit point), between the eyes and the hull bottom.
        std::vector<Vec3> doorHandlePoints() const;

        // Moment of inertia about the forward axis (kg*m^2): the profile's `lean.rollInertia`, else the collision box's (uniform density).
        float rollInertia() const;
        // Moment of inertia about the right axis (pitch): the profile's `lean.pitchInertia`, else the collision box's.
        float pitchInertia() const;
    };

    // Parse a profile from JSON text. Positions in the JSON are in game units (what NifSkope and the
    // Creation Kit show), everything else in SI units. `selection` (slot -> part id) merges those parts' patches into the
    // document first. Throws std::runtime_error with a readable message.
    VehicleSpec parseVehicleSpec(const std::string& jsonText, const std::map<std::string, std::string>& selection = {});

    // Load and parse a profile file.
    VehicleSpec loadVehicleSpecFile(const std::string& path, const std::map<std::string, std::string>& selection = {});

    // The part of `spec` in `slot` with this id, or null.
    const PartSpec* findPart(const VehicleSpec& spec, const std::string& slot, const std::string& id);
}
