#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>

#include "Config.h"
#include "audio/EngineSound.h"
#include "ModBase.h"
#include "f4vr/WandActivationSphere.h"
#include "player/DriverSeat.h"
#include "telemetry/CsvLog.h"
#include "vehicle/DamageSmoke.h"
#include "vehicle/Vehicle.h"

namespace vfvr
{
    class VehicleFrameworkVR : public ModBase
    {
    public:
        VehicleFrameworkVR()
            : ModBase(withRenderingPreload(Settings(Version::PROJECT, Version::NAME, &g_config, 256, true)))
        {}

        // Papyrus (VM thread): spawn the test vehicle at a marker, done on the next game frame.
        bool requestSpawnAtMarker(RE::TESObjectREFR* marker);

        // FRIK FrameBegin / FrameEnd phase callbacks (game thread).
        void onFrikFrameBegin();
        void onFrikFrameEnd();
        void onFrikAfterWorldFinal();

    protected:
        virtual void onModLoaded(const F4SE::LoadInterface* f4SE) override;
        virtual void onGameLoaded() override;
        virtual void onGameSessionLoaded() override;
        virtual void onFrameUpdate() override;

    private:
        static Settings withRenderingPreload(Settings settings)
        {
            settings.preloadRendering = true; // the HUD and the activation icons draw on the overlay: no first-draw stall
            return settings;
        }

        struct CostMeter
        {
            std::atomic<std::uint64_t> gameNanos{ 0 };
            std::atomic<std::uint64_t> stepNanos{ 0 };
            std::atomic<std::uint64_t> maxFrameNanos{ 0 };
            std::uint64_t frames = 0;
            std::uint64_t lastReportMs = 0;
        };

        void initFrikApi();
    void updateBodyHiding();
        // Diagnostic: the player's body state, every 2 s and on change (bLogPlayerBody). [game]
        void logPlayerBody();
        std::string _lastBodyState;
        std::uint64_t _lastBodyLogMs = 0;
        bool _zonesActive = false; // a get-in or trunk zone was live this frame
        // After the trunk menu closes and after getting in or out, the zones wait until A is released and a second has passed: the press that closed
        // the menu (or left the seat) must not open the trunk or get in again.
        bool _zonesWaitRelease = false;
        std::uint64_t _zonesBlockedUntilMs = 0;
        bool _trunkWasBusy = false;
        bool _wasDriving = false;
        void openTelemetry();
        void handleSpawnRequests();
        void spawnTestVehicle(RE::NiPoint3 groundPoint, float heading);
        void bindPendingVehicle();
        void releaseVehicle(bool deleteReference, const std::string& reason);
        void handleSeatBindings();
        // Getting in: an activation zone on each door handle, the enter binding pressed with the hand inside one. [game]
        void updateDoorZones();
        std::vector<std::unique_ptr<f4vr::WandActivationSphere>> _doorZones;
        f4vr::WandActivationSphere _trunkZone{ "VFVR_Trunk" };
        model::DriverInput readDriverInput() const;
        std::optional<model::VehicleSpec> loadProfile(const std::string& name, const std::map<std::string, std::string>& selection = {}, bool quiet = false) const;
        // Wheel sets: the menu on entering, the INI setting, and applying a choice to the vehicle. [game]
        void showWheelMenu();
        void selectWheelSet(const std::string& id);
        void updateWheelSet();
        // Menus: vehicle choice (hold B, before anything spawns), then the wheel choice for the new vehicle. [game]
        enum class MenuKind { None, Vehicle, Wheels };
        bool openMenu(const std::string& body, const std::vector<std::string>& labels);
        void beginSpawn(RE::NiPoint3 groundPoint, float heading);
        void updateMenus();
        MenuKind _menuKind = MenuKind::None;
        struct SpawnChoice
        {
            RE::NiPoint3 point;
            float heading = 0.0f;
        };
        std::optional<SpawnChoice> _spawnChoice;
        std::vector<std::string> _vehicleIds;
        std::string _chosenProfile;
        bool _wheelMenuPending = false; // show the wheel menu once the vehicle just chosen is ready
        std::string _wheelSet; // the wheel set chosen this session ("" = the INI's)
        std::string _lastConfigWheelSet;
        std::string _profileName;
        std::atomic<int> _menuChoice{ -1 };
        std::atomic<bool> _menuOpen{ false };
        bool _menuInputReleased = false; // the seat gave the controllers back to the game while the menu is open
        std::uint64_t _menuShownFrame = 0;
        std::vector<std::string> _menuOptionIds;
        void addFrameCost(std::uint64_t nanos);

        std::unique_ptr<Vehicle> _vehicle;
        DriverSeat _seat;
        CsvLog _physicsLog;
        CsvLog _cameraLog;
        CsvLog _motoLog; // two-wheeler diagnostics (bLogMotoDiag)

        // A spawned reference waiting for its 3D and body before it can become a vehicle.
        RE::ObjectRefHandle _pendingRef;
        std::optional<model::VehicleSpec> _pendingSpec;
        int _pendingFrames = 0;
        RE::ObjectRefHandle _spawnedRef;
        RE::ObjectRefHandle _pendingVisual; // model shown on a carrier, when the profile uses one
        RE::ObjectRefHandle _spawnedVisual;
        RE::ObjectRefHandle _freshVisual; // replacement model being created in the carrier's cell
        int _freshFrames = 0;
        int _respawnCooldown = 0;
        void keepVisualInCarrierCell();
        // Headlights: spot light references that follow the vehicle. [game]
        void toggleLights();
        void updateLights();
        void removeLights();
        std::vector<RE::ObjectRefHandle> _lightRefs;
        bool _lightsOn = false;
        // Exhaust smoke: a small pool of puff references (flat soft cloud sprites turned toward the player's head) that drift up and swell behind the
        // tailpipe. [game]
        void updateExhaust(bool emitting, float throttle);
        void removeExhaust();
        struct Puff
        {
            RE::ObjectRefHandle ref;
            RE::NiPoint3 pos{};
            RE::NiPoint3 vel{};
            float age = 1.0f;
            float life = 1.0f; // age >= life: not shown
            float roll = 0.0f; // turn of the sprite about the line of sight (rad)
            float spin = 0.0f; // rad/s
            float size = 1.0f; // this puff's size relative to the others
        };
        std::vector<Puff> _puffs;
        float _puffAccumulator = 0.0f;
        std::size_t _puffNext = 0;
        std::uint64_t _puffClock = 0;
        int _puffCooldown = 0;
        int _puffNo3DFrames = 0; // frames in a row in which no puff had its 3D (its cell unloaded behind the car)
        std::uint32_t _puffRng = 0x9E3779B9u;
        EngineSound _engineSound; // synthesised engine hum, played while driving
        // Health system: the hits, the driver's cover, the radiation shielding and the smoke each frame; the condition changes; the repair at the
        // repair zone; the remembered state of a profile. [game]
        void updateHealth();
        void restoreVehicleState();
        bool advanceRepair();
        DamageSmoke _damageSmoke;
        f4vr::WandActivationSphere _repairZone{ "VFVR_Repair" };
        float _repairHeld = 0.0f; // seconds the repair binding has been held at the repair point
        std::uint64_t _repairLastMs = 0; // 0 = no repair in progress
        std::uint64_t _repairFrame = 0; // the last frame the hold advanced
        bool _conditionKnown = false;
        model::VehicleCondition _lastCondition = model::VehicleCondition::Operational;
        int _lightCooldown = 0;

        // Spawn requests from Papyrus.
        std::mutex _requestMutex;
        std::optional<RE::NiPoint3> _requestedPoint;

        bool _frikReady = false;
        bool _frikScope = false;
        bool _bodyHidden = false;
        int _bodyBoundFrames = 0; // frames left of keeping the player body's culling spheres on the body
        bool _frikFrameCallback = false;
        bool _frikEndCallback = false;
        bool _frikFinalCallback = false;
        std::uint64_t _frame = 0;
        std::uint64_t _frikLastFrame = 0;
        bool _lastSimpleMode = false;
        int _flushCountdown = 0;

        CostMeter _cost;
    };

    // The ONE global to rule them ALL
    inline VehicleFrameworkVR g_vehicleFramework;
}
