#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "model/RadShield.h"
#include "model/VehicleHealth.h"
#include "model/VehicleModel.h"
#include "physics/HavokBridge.h"
#include "vehicle/HudState.h"

namespace vfvr
{
    class CsvLog;

    /**
     * One drivable vehicle: a world reference whose Havok body is switched to dynamic and driven by the
     * vehicle model with continuous forces.
     *
     * Split by thread:
     * - Game thread (onFrameUpdate): validates the body, casts the wheel rays, publishes ground contacts and
     *   driver input, keeps the body awake, re-registers the physics step listener, debug draw.
     * - Physics step (onPhysicsStep, through the listener): reads the body state, runs the model, applies the
     *   impulses. Nothing per frame goes through Papyrus.
     */
    class Vehicle
    {
    public:
        // Bind a loaded reference to a profile and make its body dynamic. Null with `error` set on failure;
        // `retryable` says whether trying again on a later frame can help (the body may not be in the world yet).
        // `ref` carries the physics body; `visual` (optional) is the model shown in its place, used when the
        // model's own collision cannot be dynamic (profile "carrierForm").
        static std::unique_ptr<Vehicle> create(RE::TESObjectREFR* ref, RE::TESObjectREFR* visual, model::VehicleSpec spec, std::string& error, bool& retryable);
        ~Vehicle();

        Vehicle(const Vehicle&) = delete;
        Vehicle& operator=(const Vehicle&) = delete;

        RE::TESObjectREFR* ref() const;

        const model::VehicleSpec& spec() const
        {
            return _spec;
        }

        const physics::BodyHandle& bodyHandle() const
        {
            return _body;
        }

        std::uint32_t bodyId() const
        {
            return _body.bodyId.value;
        }

        // False once the reference or its 3D/body is gone (cell unloaded, deleted). [game]
        bool isValid() const
        {
            return _valid.load(std::memory_order_acquire);
        }

        // [game] once per frame. `input` is what the driver does this frame (zero when nobody drives).
        // `motoLog`: the two-wheeler diagnostics CSV (bLogMotoDiag), written only for profiles with a "lean" block.
        void onFrameUpdate(const model::DriverInput& input, bool occupied, CsvLog* physicsLog, CsvLog* motoLog = nullptr);

        // World pose of the vehicle frame in meters, from the drawn node or from the Havok body. [game]
        model::Pose poseFromNode() const;
        model::Pose poseFromBody() const;
        model::Pose poseForSeat() const;
        // How far the engine moved the model root between our two writes (cm; z only in the second): the drawn model against what we wrote.
        float visualErrCm() const { return _visualErrCm; }
        float visualErrDzCm() const { return _visualErrDzCm; }
        // How far the engine's previous-frame transform of the model root was from its current one when we last moved the model (cm).
        float visualPrevWorldErrCm() const { return _prevWorldErrCm; }
        // Start a frame for the seat camera: sample the pose the model and the eyes come from this frame (the seat reads it with poseForSeat) and tell the
        // frame update to leave the model alone while that keeps happening. The seat then draws the model itself with placeVisualAt. [game]
        void beginCameraFrame();
        // Draw the model at `pose` (the vehicle frame, as poseForSeat gives it). [game]
        void placeVisualAt(const model::Pose& pose);
        // A car without a separate model: write the reference's own node at the pose (the engine syncs it from the Havok body once per frame, before the seat's FrameBegin).
        void placeBodyNodeAt(const model::Pose& pose);
        // The pose the model was last drawn at (the headlights follow it).
        model::Pose drawnPose() const;
        // The model reference is a plain static: it only stays loaded in the cell it was created in. The mod re-creates it in the
        // carrier's cell when the two differ.
        bool visualCellMismatch() const;
        // Put a headlight reference (spawned by the mod) on light point `index`, aimed forward. False while its 3D is not loaded. [game]
        bool placeLight(RE::TESObjectREFR& light, std::size_t index) const;
        RE::ObjectRefHandle visualHandle() const { return _visualHandle; }
        RE::ObjectRefHandle swapVisual(const RE::ObjectRefHandle& fresh, RE::NiAVObject* freshRoot);

        // Forward speed in m/s from the last physics step.
        float speed() const;
        // What the driving HUD shows for this vehicle. [game]
        HudState hudState() const;
        // Last published telemetry (rpm, throttle, gear...), for the engine sound. [game]
        model::VehicleTelemetry telemetry() const;

        // Health and radiation shielding (bVehicleHealth): the hit sink, the collisions, the repair, the save and the HUD read and write them, all
        // on the game thread. A disabled vehicle (health 0) takes no throttle and holds the handbrake until it is repaired. [game]
        model::VehicleHealth& health() { return _health; }
        const model::VehicleHealth& health() const { return _health; }
        model::RadShield& radShield() { return _radShield; }
        const model::RadShield& radShield() const { return _radShield; }
        // Where the damage smoke comes from (vehicle frame, meters): the profile's damage.smoke.points, or the middle of the bonnet (the engine
        // of a two-wheeler). [game]
        std::vector<model::Vec3> smokePoints() const;
        // Where the repair is done (vehicle frame, meters): the profile's damage.repair.point, or the front of the bonnet. [game]
        model::Vec3 repairPoint() const;

        // Swap the profile in place (another part selection: wheels, tyres, mass, suspension...). The car should be at rest. The hull stays
        // as it was built; wheel positions, tyre size, suspension, drive layout and the visible wheel meshes follow the new spec. [game]
        void applySpec(model::VehicleSpec spec);

        void setSimpleMode(bool simple);

        // Called by the step listener. [step]
        void onPhysicsStep(float deltaSeconds);

        // Time spent in physics step callbacks since the last call (nanoseconds), for the cost report.
        static std::uint64_t consumeStepNanos();

    private:
        Vehicle() = default;

        bool validate();
        void syncVisual();
        void animateWheels();
        void scanWheelNodes();
        void logGeometryState(const char* why);
        void applyBodyLayer(bool occupied);
        void castWheelRays(const model::Pose& frame);
        void drawDebug(const model::Pose& frame) const;
        void applyWheelSetVisibility();
        void probeImpact();
        // The hull's velocity change (m/s) from which a step counts as a collision: the lower of the log and the damage thresholds. [step]
        float impactThreshold() const;

        RE::ObjectRefHandle _handle;
        RE::NiAVObject* _root = nullptr;
        RE::ObjectRefHandle _visualHandle;
        RE::NiAVObject* _visualRoot = nullptr;
        int _visualMissing = 0;
        // Render interpolation: pose samples on the physics clock, written by the step listener and the frame update.
        struct PoseSample
        {
            double t = 0.0;
            model::Pose pose;
        };
        model::Pose renderPose();
        // Flipped-car recovery: timer in the frame update, the impulses go out in the next physics step.
        float _flippedTime = 0.0f;
        std::atomic<bool> _rightPending{ false };
        // Falling out of the world: the last pose the car rested on its wheels at (body frame, step thread, under _mutex), the time it has rested
        // there, and a recovery asked by the game thread for the next step.
        model::Pose _lastSupportedBody{};
        bool _hasSupported = false;
        float _supportedTime = 0.0f; // game thread: how long the car has rested on two or more wheels, upright
        std::atomic<bool> _recoverPending{ false };
        float _fallCheckCooldown = 0.0f;
        void checkFallThrough(const model::Pose& frame, float riseMps);
        float _rightCooldown = 0.0f;
        std::chrono::steady_clock::time_point _flipClock{};
        void updateRighting(const model::Pose& frame);
        // `spin` and `lift` scale the roll and the lift of one righting (1 = the single kick used on its side).
        void applyRighting(const physics::BodyStateMeters& state, float spin = 1.0f, float lift = 1.0f);
        int _rightingStepsLeft = 0; // step thread: an upside-down vehicle is rolled over across this many more steps
        void pushSample(double t, const model::Pose& pose); // _renderMutex held
        std::mutex _renderMutex;
        std::vector<PoseSample> _samples;
        double _simClock = 0.0; // sum of the physics step times seen so far
        double _clockOffset = 0.0; // smoothed (physics clock - wall clock)
        double _lastWall = 0.0;
        bool _clockValid = false;
        model::Pose _syncedPose{};
        bool _hasSyncedPose = false;
        RE::NiPoint3 _lastWrittenWorld{};
        bool _hasWrittenWorld = false;
        float _visualErrCm = 0.0f;
        float _visualErrDzCm = 0.0f;
        float _prevWorldErrCm = 0.0f;
        model::Pose _drawnPose{};
        bool _hasDrawnPose = false;
        std::chrono::steady_clock::time_point _cameraSync{}; // last beginCameraFrame()
        float _lastFrameZ = 0.0f;
        float _launchLogCooldown = 0.0f;
        float _bottomedLogCooldown = 0.0f;
        struct WheelNode
        {
            RE::NiPointer<RE::NiAVObject> node;
            RE::NiMatrix3 rest; // local rotation as found
            RE::NiPoint3 restPos; // local translation as found (the hub at static ride height)
            std::size_t wheel = 0; // index into the spec wheels
            float angle = 0.0f; // accumulated spin (rad)
            std::vector<RE::NiPointer<RE::NiAVObject>> shapes; // every Wheel_<corner>_<set> shape of the node, attached or not
        };
        std::vector<WheelNode> _wheelNodes;
        // Nodes named "Steer..." (a motorcycle's fork, handlebar and headlight) turn with the steering about the profile's steering axis.
        struct SteerNode
        {
            RE::NiPointer<RE::NiAVObject> node;
            RE::NiMatrix3 rest; // local rotation as found
            RE::NiPoint3 restPos; // local translation as found
        };
        std::vector<SteerNode> _steerNodes;
        RE::NiAVObject* _wheelScanRoot = nullptr;
        std::string _wheelSetShown; // the wheel mesh set the shapes are currently showing
        bool _wheelSetMissingLogged = false;
        std::chrono::steady_clock::time_point _wheelClock{};
        float _visualDiagTimer = 0.0f;
        bool _modelIsRoot = false; // no separate model: the reference's own 3D is the car and the seat writes its node itself
        int _watchCounter = 0; // frames since the last "Vehicle watch" log line
        int _watchLines = 0;
        float _geometryDiagTimer = 0.0f;
        int _geometryDiagCount = 0;
        model::Vec3 _boundsMin; // model bounds in the vehicle frame, for the automatic wheel layout
        model::Vec3 _boundsMax;
        model::Quat _frameFix; // body frame -> vehicle frame when the model's own body is driven (no carrier)
        model::Quat _modelRot; // model axes -> vehicle axes
        physics::BodyHandle _body;
        model::VehicleSpec _spec;
        model::VehicleHealth _health;
        model::RadShield _radShield;
        std::atomic<bool> _valid{ false };
        std::uint32_t _rayFilter = 0;
        std::string _layerName; // the sParked/sVehicleBodyLayer text last parsed
        std::optional<RE::COL_LAYER> _layerParsed;
        RE::COL_LAYER _appliedLayer = RE::COL_LAYER::kNonCollidable;
        bool _hasAppliedLayer = false;
        RE::COL_LAYER _parkedBodyLayer = RE::COL_LAYER::kNonCollidable; // the body's layer as the engine made it: what it keeps while nobody drives
        bool _bodyOccupied = false;

        // Shared between the game thread and the physics step.
        mutable std::mutex _mutex;
        model::VehicleModel _model;
        model::DriverInput _input;
        std::array<model::WheelContact, model::MAX_WHEELS> _contacts{};
        bool _simpleMode = false;
        model::VehicleTelemetry _lastTelemetry;
        CsvLog* _physicsLog = nullptr;
        CsvLog* _motoLog = nullptr;
        // Two-wheeler events for the log (step thread): front wheel in the air, a big lean, a fast yaw.
        void logMotoStep(CsvLog* motoLog, const model::Pose& frame, const physics::BodyStateMeters& state, const std::array<model::WheelContact, model::MAX_WHEELS>& contacts,
            float deltaSeconds);
        float _motoFrontAir = 0.0f;
        bool _motoFrontAirLogged = false;
        bool _motoLeanLogged = false;
        float _motoYawCooldown = 0.0f;
        std::uint64_t _frame = 0;
        std::vector<model::AppliedImpulse> _impulses; // step-thread scratch

        // Impact diagnostic: a step whose velocity change is not explained by the tyre forces, gravity and drag is a collision of the hull.
        // The step thread detects it (and the CSV column), the game thread probes the world around the car and logs what is there.
        struct ImpactEvent
        {
            bool pending = false;
            model::Vec3 residual; // m/s, world
            model::Vec3 com;
            model::Pose frame;
            float speed = 0.0f; // forward, m/s
        };
        std::mutex _impactMutex;
        ImpactEvent _impact;
        bool _hasPrevStep = false;
        model::Vec3 _prevVel;
        model::Vec3 _prevImpulse; // sum of the impulses applied in the previous callback (N*s)
        float _prevDt = 0.0f;
        float _impactResidual = 0.0f; // |residual| of the latest step (CSV)
        float _impactResidualZ = 0.0f;
        float _settleTimer = 0.0f;
        float _settleLog = 0.0f;
        std::vector<std::uint32_t> _mutedBodies; // foreign bodies the hull no longer collides with
        float _impactCooldown = 0.0f;

        // Game-thread diagnostics.
        std::uint64_t _lastStepCount = 0;
        int _framesWithoutStep = 0;
        float _diagTimer = 0.0f;
    };
}
