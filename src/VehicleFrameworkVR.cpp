#include "VehicleFrameworkVR.h"

#include <chrono>
#include <filesystem>
#include <format>

#include "api/FRIKApiV2.h"
#include "common/MatrixUtils.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "health/HealthStore.h"
#include "health/HitSink.h"
#include "health/OccupantShield.h"
#include "health/RadProtection.h"
#include "health/VitalsProbe.h"
#include "papyrus/PapyrusApi.h"
#include "perf/Perf.h"
#include "physics/HavokBridge.h"
#include "player/RenderProbe.h"
#include "vehicle/SpriteUtils.h"
#include "vehicle/Trunk.h"
#include "vrcf/VRControllersManager.h"

using namespace vfvr;

// This is the entry point to the mod.
extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Query(const F4SE::QueryInterface* a_f4se, F4SE::PluginInfo* a_info)
{
    return g_mod->onF4SEPluginQuery(a_f4se, a_info);
}

// This is the entry point to the mod.
extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Load(const F4SE::LoadInterface* a_f4se)
{
    return g_mod->onF4SEPluginLoad(a_f4se);
}

namespace vfvr
{
    namespace
    {
        constexpr float MIN_SPAWN_DISTANCE = 3.5f; // m between the player and the spawned vehicle
        constexpr int PENDING_BIND_FRAMES = 270; // ~3 s for the spawned reference to load its 3D and body
        constexpr float SPAWN_HEIGHT = 0.5f; // m above the ground (the physics settles it)
        constexpr float TRUNK_MAX_SPEED = 0.5f; // m/s, the trunk opens only with the vehicle (nearly) stopped
        constexpr float TRUNK_HIDE_DEPTH = 300.0f; // game units under the vehicle where a new trunk container is placed

        std::uint64_t nowNanos()
        {
            return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
        }

        // Dead zone and response curve for a 0..1 axis.
        float shapeAxis(const float raw)
        {
            const float v = std::clamp(std::abs(raw), 0.0f, 1.0f);
            if (v <= g_config.deadzone) {
                return 0.0f;
            }
            const float t = (v - g_config.deadzone) / (1.0f - g_config.deadzone);
            return std::copysign(std::pow(t, g_config.responseCurve), raw);
        }

        // Heading of the HMD in the world (game convention).
        float hmdHeading()
        {
            const auto* nodes = f4vr::getVRPlayerNodes();
            if (!nodes || !nodes->hmdNode) {
                return 0.0f;
            }
            const auto forward = nodes->hmdNode->world.rotate.Transpose() * RE::NiPoint3(0, 1, 0);
            return model::headingOf({ forward.x, forward.y, forward.z });
        }

        // The profile's engine as the sound synthesis wants it.
        EngineVoice engineVoiceOf(const model::EngineSpec& engine)
        {
            EngineVoice v;
            v.cylinders = engine.cylinders;
            v.gain = engine.sound.gain;
            v.pitch = engine.sound.pitch;
            v.oddDecay = engine.sound.oddDecay;
            v.evenDecay = engine.sound.evenDecay;
            v.rumble = engine.sound.rumble;
            v.lumpiness = engine.sound.lumpiness;
            v.noise = engine.sound.noise;
            v.brightness = engine.sound.brightness;
            v.whine = engine.sound.whine;
            v.whineOrder = engine.sound.whineOrder;
            return v;
        }

        void FRIK_CALL frikFrameBeginThunk(std::uint32_t, void*) noexcept
        {
            try {
                g_vehicleFramework.onFrikFrameBegin();
            } catch (const std::exception& e) {
                logger::error("FRIK frame callback failed: {}", e.what());
            } catch (...) {
                logger::error("FRIK frame callback failed");
            }
        }

        void FRIK_CALL frikAfterWorldFinalThunk(std::uint32_t, void*) noexcept
        {
            try {
                g_vehicleFramework.onFrikAfterWorldFinal();
            } catch (const std::exception& e) {
                logger::error("FRIK after-world-final callback failed: {}", e.what());
            } catch (...) {
                logger::error("FRIK after-world-final callback failed");
            }
        }

        void FRIK_CALL frikFrameEndThunk(std::uint32_t, void*) noexcept
        {
            try {
                g_vehicleFramework.onFrikFrameEnd();
            } catch (const std::exception& e) {
                logger::error("FRIK frame end callback failed: {}", e.what());
            } catch (...) {
                logger::error("FRIK frame end callback failed");
            }
        }
    }

    void VehicleFrameworkVR::onModLoaded(const F4SE::LoadInterface*)
    {
        logger::info("{} v{} - vehicle framework for Fallout 4 VR (phase 1 prototype)", Version::PROJECT, Version::NAME);
        Trunk::get().registerSerialization();
    }

    void VehicleFrameworkVR::onGameLoaded()
    {
        physics::logEngineAddresses();
        papyrus::registerNatives();
        Trunk::get().registerEvents();
        initFrikApi();
        openTelemetry();
    }

    void VehicleFrameworkVR::onGameSessionLoaded()
    {
        // A save load replaces the world: drop the vehicle and the seat state without touching the game.
        _seat.reset();
        OccupantShield::get().onSessionLoaded();
        RadProtection::get().onSessionLoaded();
        HitSink::get().setWatched(0);
        _damageSmoke.clear();
        _repairHeld = 0.0f;
        _repairLastMs = 0;
        _conditionKnown = false;
        _vehicle.reset();
        _pendingRef = {};
        _pendingSpec.reset();
        _spawnedRef = {};
        physics::StepListener::get().clearCallback();
    }

    void VehicleFrameworkVR::initFrikApi()
    {
        using frik::api::FRIKApiV2;
        // v2.1 has blockFeature; frame callbacks need v2.3, checked separately.
        const int err = FRIKApiV2::initialize(1);
        if (err != 0) {
            logger::warn("FRIK API v2 not available (error {}): Smooth Movement is not blocked and the camera uses this plugin's frame update", err);
            return;
        }
        _frikReady = true;
        const auto version = FRIKApiV2::inst->getVersion();
        logger::info("FRIK API v2 (v{}) ready", version);
        if (version >= 2) {
            // Publishing "looking through a scope" makes FRIK hide the player body; used for the third person camera.
            _frikScope = FRIKApiV2::inst->setScopeProvider("VehicleFrameworkVR", static_cast<std::uint32_t>(FRIKApiV2::ScopeCapability::PublishesLookingThrough));
            logger::info("FRIK scope provider {}", _frikScope ? "registered (body hiding in third person)" : "NOT registered");
        }
        if (version >= 3) {
            _frikFrameCallback =
                FRIKApiV2::inst->registerFrameCallback("VehicleFrameworkVR", static_cast<std::uint32_t>(FRIKApiV2::FramePhase::FrameBegin), &frikFrameBeginThunk, nullptr, 100);
            logger::info("FRIK FrameBegin callback {}", _frikFrameCallback ? "registered" : "NOT registered");
            // FrameEnd: what the engine did with the head node since the seat placed the player (completes the camera CSV row).
            _frikEndCallback =
                FRIKApiV2::inst->registerFrameCallback("VehicleFrameworkVR", static_cast<std::uint32_t>(FRIKApiV2::FramePhase::FrameEnd), &frikFrameEndThunk, nullptr, 100);
            logger::info("FRIK FrameEnd callback {}", _frikEndCallback ? "registered" : "NOT registered");
            // AfterWorldFinal: every bone is final; third person moves the body from the chase camera into the seat (bThirdPersonBodyInSeat).
            _frikFinalCallback = FRIKApiV2::inst->registerFrameCallback("VehicleFrameworkVR", static_cast<std::uint32_t>(FRIKApiV2::FramePhase::AfterWorldFinal),
                &frikAfterWorldFinalThunk, nullptr, 100);
            logger::info("FRIK AfterWorldFinal callback {}", _frikFinalCallback ? "registered" : "NOT registered");
        }
    }

    void VehicleFrameworkVR::updateBodyHiding()
    {
        // The player is carried by SetPosition every frame while driving, and the skeleton's culling spheres do not follow: afterwards the renderer
        // skips the body (the FRIK body did not show after getting out). Keep the spheres on the nodes while driving and for a few seconds after.
        if (_seat.isDriving()) {
            _bodyBoundFrames = 360;
        }
        if (_bodyBoundFrames > 0) {
            --_bodyBoundFrames;
            if (auto* player = RE::PlayerCharacter::GetSingleton()) {
                std::function<void(RE::NiAVObject*, int)> keep = [&](RE::NiAVObject* node, const int depth) {
                    auto* asNode = node ? node->IsNode() : nullptr;
                    if (!asNode || depth > 10) {
                        return;
                    }
                    node->worldBound.center = node->world.translate;
                    if (node->worldBound.fRadius < 150.0f) {
                        node->worldBound.fRadius = 150.0f;
                    }
                    for (const auto& child : asNode->children) {
                        keep(child.get(), depth + 1);
                    }
                };
                keep(player->Get3D(false), 0);
                keep(player->Get3D(true), 0);
            }
        }
        if (!_frikScope) {
            return;
        }
        // Third person hides the body, unless it is shown in the seat (bThirdPersonBodyInSeat, FRIK's AfterWorldFinal phase available).
        const bool inSeat = g_config.thirdPersonBodyInSeat && _frikFinalCallback;
        const bool hide = _vehicle && _seat.isDriving() && _seat.isThirdPerson() && !inSeat;
        if (hide != _bodyHidden) {
            _bodyHidden = hide;
            frik::api::FRIKApiV2::inst->setLookingThroughScope("VehicleFrameworkVR", hide);
            logger::info("FRIK body hidden: {}", hide);
        }
    }

    void VehicleFrameworkVR::logPlayerBody()
    {
        if (!g_config.logPlayerBody) {
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            return;
        }
        // One 3D root: present, hidden (app culled), and the radius the renderer culls it by (a tiny or far-off bound hides the body too).
        const auto describe = [](RE::NiAVObject* root, float& radius, float& boundOffset) {
            radius = 0.0f;
            boundOffset = 0.0f;
            if (!root) {
                return std::string("none");
            }
            radius = root->worldBound.fRadius;
            const auto d = root->worldBound.center - root->world.translate;
            boundOffset = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            return std::string(root->GetAppCulled() ? "hidden" : "shown");
        };
        float thirdRadius = 0.0f;
        float thirdOffset = 0.0f;
        float firstRadius = 0.0f;
        float firstOffset = 0.0f;
        const auto third = describe(player->Get3D(false), thirdRadius, thirdOffset);
        const auto first = describe(player->Get3D(true), firstRadius, firstOffset);
        using frik::api::FRIKApiV2;
        const auto* frik = FRIKApiV2::inst;
        const bool skeletonReady = frik && frik->isSkeletonReady && frik->isSkeletonReady();
        const std::uint32_t generation = frik && frik->getSkeletonGeneration ? frik->getSkeletonGeneration() : 0;
        const bool vehicle = _vehicle && _vehicle->isValid();

        // What decides a log line: anything but the exact radius, which moves every frame (only whether it is too small to see the body by).
        const auto state = std::format("3rd {} (bound {}) 1st {} (bound {}), FRIK skeleton {} gen {}, vehicle {}, driving {}, third person {}, body hidden by us {}, zones {}, body in seat {}",
            third,
            thirdRadius < 50.0f ? "tiny" : "ok",
            first,
            firstRadius < 50.0f ? "tiny" : "ok",
            skeletonReady ? "ready" : "not ready",
            generation,
            vehicle ? _vehicle->spec().id : "none",
            _seat.isDriving(),
            _seat.isThirdPerson(),
            _bodyHidden,
            _zonesActive ? "active" : "off",
            _seat.bodyInSeat());
        const auto now = common::nowMillis();
        const bool changed = state != _lastBodyState;
        if (!changed && now - _lastBodyLogMs < 2000) {
            return;
        }
        _lastBodyLogMs = now;
        logger::info("Player body{}: {}; 3rd bound r {:.0f} off {:.0f}, 1st bound r {:.0f} off {:.0f} (game units)", changed ? " CHANGED" : "", state, thirdRadius,
            thirdOffset, firstRadius, firstOffset);
        _lastBodyState = state;
    }

    void VehicleFrameworkVR::openTelemetry()
    {
        if (!g_config.telemetry) {
            return;
        }
        const auto dir = common::getRelativePathInDocuments(R"(\My Games\Fallout4VR\F4SE)");
        const auto maxRows = static_cast<std::size_t>(g_config.telemetryMaxMinutes * 60.0f * 90.0f);
        std::string header = "frame,dt,speed_kmh,rpm,gear,throttle,brake,steer_deg,wheels_down,yaw_rate,vel_z,up_z";
        for (const auto* w : { "w0", "w1", "w2", "w3" }) {
            header += std::format(",{0}_down,{0}_hold,{0}_comp,{0}_spin,{0}_slip,{0}_slip_deg,{0}_fz,{0}_fx,{0}_fy,{0}_grip", w);
        }
        header += ",impact_dv,impact_z,lean_deg,lean_target_deg,handbrake";
        render_probe::setDirectory(dir);
        if (g_config.logMotoDiag) {
            std::string moto = "frame,t_ms,dt,speed_kmh,throttle,brake,handbrake,steer_in,steer_deg,lean_deg,lean_target_deg,stab_torque,stab_saturated,roll_rate_dps,"
                               "pitch_rate_dps,yaw_rate_dps,up_z,pos_x,pos_y,pos_z,vel_z,wheels_down";
            for (const auto* w : { "front", "rear" }) {
                moto += std::format(",{0}_down,{0}_comp,{0}_nx,{0}_ny,{0}_nz,{0}_layer,{0}_slip,{0}_slip_deg,{0}_fx,{0}_fy,{0}_fz,{0}_grip", w);
            }
            moto += ",impact_dv";
            _motoLog.open(dir + "\\VehicleFrameworkVR_moto.csv", moto, maxRows);
        }
        _physicsLog.open(dir + "\\VehicleFrameworkVR_physics.csv", header, maxRows);
        _cameraLog.open(dir + "\\VehicleFrameworkVR_camera.csv",
            "frame,dt,speed_kmh,head_seat_cm,anchor_error_cm,room_yaw_error_deg,vertical_filter_cm,yaw_sign,hmd_heading_deg,commanded_deg,yaw_deg,seat_step_cm,expected_step_cm,seat_dz_cm,pitch_deg,roll_deg,view,node_body_cm,node_lead_cm,actor_drift_cm,actor_dz_cm,vis_err_cm,vis_dz_cm,anchor_fwd_cm,anchor_side_cm,anchor_up_cm,t_ms,lock,draw_lag_cm,draw_lag_fwd_cm,prev_world_err_cm,ask_fwd_cm,cam_b_x,cam_b_y,cam_b_z,hmd_x,hmd_y,hmd_z,eye_x,eye_y,eye_z,end_hmd_fwd_cm,end_hmd_side_cm,end_hmd_up_cm,cam_e_x,cam_e_y,cam_e_z",
            maxRows);
    }

    bool VehicleFrameworkVR::requestSpawnAtMarker(RE::TESObjectREFR* marker)
    {
        if (!marker) {
            return false;
        }
        // Read the position now: the script deletes its marker right after the call.
        std::scoped_lock lock(_requestMutex);
        _requestedPoint = marker->GetPosition();
        return true;
    }

    void VehicleFrameworkVR::onFrikFrameBegin()
    {
        _frikLastFrame = _frame;
        _seat.weaponCheckAtFrameBegin();
        const auto* player = RE::PlayerCharacter::GetSingleton();
        if (g_config.cameraUpdatePoint == CameraUpdatePoint::FrikFrameBegin && player && player->loadedData && _seat.isDriving() && _vehicle && _vehicle->isValid()) {
            // One pose for the frame, sampled first; the seat then places the eyes and draws the model from it.
            _vehicle->beginCameraFrame();
            _seat.updateCamera(*_vehicle, _cameraLog.isOpen() ? &_cameraLog : nullptr, true);
        }
    }

    void VehicleFrameworkVR::onFrikAfterWorldFinal()
    {
        if (_vehicle && _vehicle->isValid() && _seat.isDriving()) {
            _seat.placeBodyInSeat(*_vehicle);
        }
    }

    void VehicleFrameworkVR::onFrikFrameEnd()
    {
        _seat.onFrameEnd(_cameraLog.isOpen() ? &_cameraLog : nullptr);
    }

    std::optional<model::VehicleSpec> VehicleFrameworkVR::loadProfile(const std::string& name, const std::map<std::string, std::string>& selection, const bool quiet) const
    {
        const auto path = g_config.vehiclesFolder + "\\" + name + ".json";
        try {
            auto spec = model::loadVehicleSpecFile(path, selection);
            logger::info("Profile '{}' loaded from '{}'{}", spec.id, path, selection.empty() ? "" : std::format(" with wheels '{}'", selection.contains("wheels") ? selection.at("wheels") : ""));
            return spec;
        } catch (const std::exception& e) {
            logger::error("Profile '{}' failed: {}", path, e.what());
            if (!quiet) {
                f4vr::showNotification(std::format("Vehicle profile error: {}", name));
            }
            return std::nullopt;
        }
    }

    void VehicleFrameworkVR::releaseVehicle(const bool deleteReference, const std::string& reason)
    {
        if (_seat.isDriving()) {
            _seat.exit(_vehicle.get(), reason);
        }
        _vehicle.reset();
        RadProtection::get().clear(reason.c_str());
        HitSink::get().setWatched(0);
        _damageSmoke.clear();
        _repairHeld = 0.0f;
        _repairLastMs = 0;
        _conditionKnown = false;
        _pendingRef = {};
        removeLights();
        _lightsOn = false;
        _pendingVisual = {};
        _freshVisual = {};
        _freshFrames = 0;
        _pendingSpec.reset();
        if (deleteReference) {
            for (auto* handle : { &_spawnedRef, &_spawnedVisual, &_freshVisual }) {
                if (auto* old = handle->get().get()) {
                    old->Disable();
                    old->SetDelete(true);
                    logger::info("Previous test vehicle reference removed");
                }
                *handle = {};
            }
        }
    }

    void VehicleFrameworkVR::spawnTestVehicle(RE::NiPoint3 groundPoint, const float heading)
    {
        if (_wheelSet.empty()) {
            _wheelSet = g_config.wheelSet;
        }
        _lastConfigWheelSet = g_config.wheelSet;
        _profileName = _chosenProfile.empty() ? g_config.testVehicleProfile : _chosenProfile;
        auto profile = loadProfile(_profileName, { { "wheels", _wheelSet } }, true);
        if (!profile) {
            logger::warn("Wheel set '{}' is not in the profile: using the profile as it is", _wheelSet);
            profile = loadProfile(_profileName);
        }
        if (!profile) {
            return;
        }
        auto* base = lookupBaseObject(profile->baseForm);
        if (!base && !profile->fallbackBaseForm.empty()) {
            base = lookupBaseObject(profile->fallbackBaseForm);
            if (base) {
                logger::warn("Base object '{}' not found (is its plugin enabled?): using '{}'", profile->baseForm, profile->fallbackBaseForm);
                f4vr::showNotification("VehicleFrameworkVR_Vehicles.esp is not enabled: using the vanilla wrecked sedan");
            }
        }
        if (!base) {
            logger::error("Base object '{}' of profile '{}' not found", profile->baseForm, profile->id);
            f4vr::showNotification("Vehicle base object not found (enable VehicleFrameworkVR_Vehicles.esp)");
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* cell = player->GetParentCell();
        auto* handler = RE::TESDataHandler::GetSingleton();
        if (!cell || !handler) {
            return;
        }

        RE::TESBoundObject* carrier = nullptr;
        if (!profile->carrierForm.empty()) {
            carrier = lookupBaseObject(profile->carrierForm);
            if (!carrier) {
                logger::error("Carrier object '{}' of profile '{}' not found", profile->carrierForm, profile->id);
                f4vr::showNotification("Vehicle carrier object not found");
                return;
            }
        }

        releaseVehicle(true, "a new test vehicle was spawned");

        // Heading of the model's own forward axis inside the model (the reference angle turns the model's +Y).
        float modelYaw = 0.0f;
        if (profile->modelForward == "+x") {
            modelYaw = model::PI * 0.5f;
        } else if (profile->modelForward == "-x") {
            modelYaw = -model::PI * 0.5f;
        } else if (profile->modelForward == "-y") {
            modelYaw = model::PI;
        }

        // Lift the model so its lowest point is SPAWN_HEIGHT above the ground; the physics sets it down.
        const auto location = groundPoint + RE::NiPoint3(0, 0, SPAWN_HEIGHT * model::HAVOK_TO_GAME - static_cast<float>(base->boundData.boundMin.z));
        const auto spawn = [&](RE::TESBoundObject* object, const float angle) {
            RE::NEW_REFR_DATA data;
            data.location = location;
            data.direction = RE::NiPoint3(0, 0, angle);
            data.object = object;
            data.interior = cell->IsInterior() ? cell : nullptr;
            data.world = cell->IsExterior() ? cell->worldSpace : nullptr;
            data.clearStillLoadingFlag = true;
            data.initializeScripts = true;
            return handler->CreateReferenceAtLocation(data);
        };

        const auto visual = spawn(base, heading - modelYaw);
        if (visual.get() == nullptr) {
            logger::error("Could not create the test vehicle reference");
            return;
        }
        if (carrier) {
            const auto body = spawn(carrier, heading);
            if (body.get() == nullptr) {
                logger::error("Could not create the vehicle carrier reference");
                return;
            }
            _spawnedRef = body;
            _pendingRef = body;
            _spawnedVisual = visual;
            _pendingVisual = visual;
        } else {
            _spawnedRef = visual;
            _pendingRef = visual;
        }
        _pendingSpec = std::move(profile);
        _pendingFrames = 0;
        logger::info("Test vehicle spawned at ({:.0f}, {:.0f}, {:.0f}), heading {:.0f} deg{}",
            location.x,
            location.y,
            location.z,
            heading * 180.0f / model::PI,
            carrier ? " (model on a carrier)" : "");
    }

    void VehicleFrameworkVR::handleSpawnRequests()
    {
        // From the test grenade's marker (Papyrus).
        std::optional<RE::NiPoint3> target;
        {
            std::scoped_lock lock(_requestMutex);
            target = std::exchange(_requestedPoint, std::nullopt);
        }

        // Development shortcut: in front of the player.
        if (!target && !_seat.isDriving() && !_menuOpen && g_config.spawnTestVehicleBinding.isEnabled() && vrcf::VRControllers.check(g_config.spawnTestVehicleBinding)) {
            const float h = hmdHeading();
            const float d = g_config.spawnDistance * model::HAVOK_TO_GAME;
            target = RE::PlayerCharacter::GetSingleton()->GetPosition() + RE::NiPoint3(std::sin(h) * d, std::cos(h) * d, 0.0f);
        }
        if (!target) {
            return;
        }

        // Facing along the line from the player to the point, kept at least MIN_SPAWN_DISTANCE away.
        const auto playerPos = RE::PlayerCharacter::GetSingleton()->GetPosition();
        RE::NiPoint3 flat = *target - playerPos;
        flat.z = 0.0f;
        const float distance = flat.Length();
        const float heading = distance > 1.0f ? std::atan2(flat.x, flat.y) : hmdHeading();
        const float minDistance = MIN_SPAWN_DISTANCE * model::HAVOK_TO_GAME;
        if (distance < minDistance) {
            *target = playerPos + RE::NiPoint3(std::sin(heading) * minDistance, std::cos(heading) * minDistance, target->z - playerPos.z);
        }

        // Find the ground under the point.
        auto* world = physics::getBhkWorld(RE::PlayerCharacter::GetSingleton());
        const auto hit = physics::castRay(world,
            *target + RE::NiPoint3(0, 0, 150.0f),
            *target - RE::NiPoint3(0, 0, 400.0f),
            static_cast<std::uint32_t>(RE::COL_LAYER::kCharController),
            0x7FFF'FFFF,
            g_config.wheelRayIgnoreMask,
            g_config.releasePickWorldReference);
        beginSpawn(hit.hit ? hit.point : *target, heading);
    }

    void VehicleFrameworkVR::removeLights()
    {
        for (auto& handle : _lightRefs) {
            if (auto* ref = handle.get().get()) {
                ref->Disable();
                ref->SetDelete(true);
            }
        }
        _lightRefs.clear();
    }

    void VehicleFrameworkVR::toggleLights()
    {
        if (!_vehicle || !_vehicle->isValid()) {
            return;
        }
        if (_vehicle->spec().lightForm.empty() || _vehicle->spec().lightPoints.empty()) {
            f4vr::showNotification("This vehicle has no lights");
            return;
        }
        _lightsOn = !_lightsOn;
        if (!_lightsOn) {
            removeLights();
        }
        _lightCooldown = 0;
        f4vr::showNotification(_lightsOn ? "Lights on" : "Lights off");
    }

    void VehicleFrameworkVR::updateLights()
    {
        if (!_lightsOn) {
            return;
        }
        if (!_vehicle || !_vehicle->isValid()) {
            removeLights();
            _lightsOn = false;
            return;
        }
        if (_lightCooldown > 0) {
            --_lightCooldown;
        }
        auto* carrier = _vehicle->ref();
        auto* cell = carrier ? carrier->GetParentCell() : nullptr;
        auto* handler = RE::TESDataHandler::GetSingleton();
        const auto& spec = _vehicle->spec();
        if (!cell || !handler) {
            return;
        }
        // (Re)create the lights when there are none or one sits in another cell than the vehicle.
        bool recreate = _lightRefs.size() != spec.lightPoints.size();
        for (auto& handle : _lightRefs) {
            auto* ref = handle.get().get();
            if (!ref || ref->IsDeleted() || ref->GetParentCell() != cell) {
                recreate = true;
            }
        }
        if (recreate && _lightCooldown == 0) {
            removeLights();
            auto* base = lookupBaseObject(spec.lightForm);
            if (!base) {
                logger::error("Light object '{}' not found", spec.lightForm);
                _lightsOn = false;
                return;
            }
            const auto pose = _vehicle->poseFromBody();
            for (const auto& point : spec.lightPoints) {
                RE::NEW_REFR_DATA data;
                data.location = physics::toGame(pose.toWorld(point));
                data.direction = RE::NiPoint3(0, 0, 0);
                data.object = base;
                data.interior = cell->IsInterior() ? cell : nullptr;
                data.world = cell->IsExterior() ? cell->worldSpace : nullptr;
                data.clearStillLoadingFlag = true;
                data.initializeScripts = true;
                _lightRefs.push_back(handler->CreateReferenceAtLocation(data));
            }
            _lightCooldown = 20;
            logger::info("Headlights created ({})", _lightRefs.size());
        }
        for (std::size_t i = 0; i < _lightRefs.size(); ++i) {
            if (auto* ref = _lightRefs[i].get().get()) {
                _vehicle->placeLight(*ref, i);
            }
        }
    }

    void VehicleFrameworkVR::removeExhaust()
    {
        for (auto& puff : _puffs) {
            if (auto* ref = puff.ref.get().get()) {
                ref->Disable();
                ref->SetDelete(true);
            }
        }
        _puffs.clear();
        _puffAccumulator = 0.0f;
    }

    void VehicleFrameworkVR::updateExhaust(const bool emitting, const float throttle)
    {
        constexpr std::size_t POOL = 24;
        const auto now = nowNanos();
        const float dt = _puffClock ? (std::min)(static_cast<float>(now - _puffClock) * 1e-9f, 0.1f) : 0.0f;
        _puffClock = now;
        if (!_vehicle || !_vehicle->isValid() || _vehicle->spec().exhaustPoints.empty() || !g_config.exhaustSmoke) {
            if (!_puffs.empty()) {
                removeExhaust();
            }
            return;
        }
        const auto& spec = _vehicle->spec();
        auto* carrier = _vehicle->ref();
        auto* cell = carrier ? carrier->GetParentCell() : nullptr;
        auto* handler = RE::TESDataHandler::GetSingleton();
        if (!cell || !handler) {
            return;
        }
        if (_puffCooldown > 0) {
            --_puffCooldown;
        }
        // Outdoors the car crosses a cell border every ~58 m and the puffs keep rendering from the neighbouring loaded cell: only another
        // worldspace, an interior, or a puff that is gone needs a new pool (recreating 24 references at every border, or every 20 frames on one, hitched).
        const auto sameSpace = [cell](const RE::TESObjectCELL* other) {
            if (!other) {
                return false;
            }
            if (cell->IsInterior() || other->IsInterior()) {
                return other == cell;
            }
            return other->worldSpace == cell->worldSpace;
        };
        bool recreate = _puffs.size() != POOL;
        bool any3D = false;
        for (auto& puff : _puffs) {
            auto* ref = puff.ref.get().get();
            if (!ref || ref->IsDeleted() || !sameSpace(ref->GetParentCell())) {
                recreate = true;
            } else if (ref->Get3D()) {
                any3D = true;
            }
        }
        _puffNo3DFrames = any3D || _puffs.empty() ? 0 : _puffNo3DFrames + 1;
        if (_puffNo3DFrames > 60) {
            recreate = true; // the cell they were made in unloaded behind the car
        }
        if (recreate) {
            if (!emitting || _puffCooldown > 0) {
                return;
            }
            removeExhaust();
            auto* base = lookupBaseObject(spec.exhaustForm);
            if (!base) {
                logger::error("Exhaust puff object '{}' not found (is VehicleFrameworkVR_Vehicles.esp enabled?)", spec.exhaustForm);
                _puffCooldown = 600;
                return;
            }
            const auto pose = _vehicle->poseFromBody();
            for (std::size_t i = 0; i < POOL; ++i) {
                RE::NEW_REFR_DATA data;
                data.location = physics::toGame(pose.toWorld(spec.exhaustPoints.front()));
                data.direction = RE::NiPoint3(0, 0, 0);
                data.object = base;
                data.interior = cell->IsInterior() ? cell : nullptr;
                data.world = cell->IsExterior() ? cell->worldSpace : nullptr;
                data.clearStillLoadingFlag = true;
                data.initializeScripts = false;
                Puff puff;
                puff.ref = handler->CreateReferenceAtLocation(data);
                puff.pos = data.location;
                _puffs.push_back(puff);
            }
            _puffCooldown = 20;
            _puffNo3DFrames = 0;
            logger::info("Exhaust smoke created ({} puffs)", _puffs.size());
        }

        // Cheap random numbers in 0..1 (the smoke only needs variety).
        const auto random01 = [this]() {
            _puffRng = _puffRng * 1664525u + 1013904223u;
            return static_cast<float>(_puffRng >> 8) / static_cast<float>(1u << 24);
        };

        // Emit: a lazy trickle at idle, thicker puffs under load.
        const auto pose = _vehicle->poseFromBody();
        const auto forward = pose.rot.axisY();
        if (emitting) {
            _puffAccumulator += dt * (2.2f + 8.0f * throttle) * static_cast<float>(spec.exhaustPoints.size());
        }
        std::size_t pointIndex = 0;
        while (_puffAccumulator >= 1.0f) {
            _puffAccumulator -= 1.0f;
            auto& puff = _puffs[_puffNext];
            _puffNext = (_puffNext + 1) % _puffs.size();
            const auto& point = spec.exhaustPoints[pointIndex++ % spec.exhaustPoints.size()];
            puff.pos = physics::toGame(pose.toWorld(point));
            // Out of the pipe backwards, then up with the hot air and a little sideways wander; the puff stays where it was left.
            const float back = 20.0f + 28.0f * throttle;
            puff.vel = RE::NiPoint3(-forward.x * back + (random01() - 0.5f) * 10.0f, -forward.y * back + (random01() - 0.5f) * 10.0f, 8.0f + 10.0f * throttle + random01() * 6.0f);
            puff.age = 0.0f;
            puff.life = 1.5f + 0.7f * throttle + 0.3f * random01();
            puff.roll = random01() * 6.2831853f;
            puff.spin = (random01() - 0.5f) * 1.0f;
            puff.size = 0.85f + 0.35f * random01();
        }

        const auto head = hmdPosition();
        for (auto& puff : _puffs) {
            auto* ref = puff.ref.get().get();
            if (!ref || ref->IsDeleted()) {
                continue;
            }
            float scale = 0.001f;
            if (puff.age < puff.life) {
                puff.age += dt;
                puff.roll += puff.spin * dt;
                puff.vel.z += 6.0f * dt;
                const float drag = std::pow(0.45f, dt);
                puff.vel.x *= drag;
                puff.vel.y *= drag;
                puff.vel.z *= std::pow(0.8f, dt);
                puff.pos.x += puff.vel.x * dt;
                puff.pos.y += puff.vel.y * dt;
                puff.pos.z += puff.vel.z * dt;
                const float t = (std::min)(puff.age / puff.life, 1.0f);
                // The sprite's half size is 3 game units: a puff starts as a wisp at the pipe, swells quickly to ~25 cm across and then dissolves (it can
                // only be scaled, not faded, so the last stretch shrinks it).
                const float grow = 0.7f + 9.5f * std::sqrt(t);
                const float fade = t > 0.6f ? std::pow((std::max)(1.0f - (t - 0.6f) / 0.4f, 0.0f), 1.3f) : 1.0f;
                scale = (std::max)(grow * fade * puff.size * (1.0f + 0.3f * throttle), 0.001f);
            }
            ref->data.location.x = puff.pos.x;
            ref->data.location.y = puff.pos.y;
            ref->data.location.z = puff.pos.z;
            ref->data.angle.x = 0.0f;
            ref->data.angle.y = 0.0f;
            ref->data.angle.z = 0.0f;
            ref->Update3DPosition(true);
            if (auto* root = ref->Get3D()) {
                root->local.scale = scale;
                faceViewer(root->local.rotate, puff.pos, head, puff.roll);
                f4vr::updateDown(root, true);
            }
        }
    }

    void VehicleFrameworkVR::keepVisualInCarrierCell()
    {
        if (!_vehicle || !_vehicle->isValid()) {
            return;
        }
        if (_respawnCooldown > 0) {
            --_respawnCooldown;
        }
        auto* carrier = _vehicle->ref();
        auto* cell = carrier ? carrier->GetParentCell() : nullptr;
        if (!cell) {
            return;
        }
        if (auto fresh = _freshVisual.get()) {
            // Waiting for the replacement model to load, then hand it over and remove the old one.
            if (auto* root = fresh->Get3D()) {
                const auto old = _vehicle->swapVisual(_freshVisual, root);
                _spawnedVisual = _freshVisual;
                _freshVisual = {};
                if (auto* oldRef = old.get().get()) {
                    oldRef->Disable();
                    oldRef->SetDelete(true);
                }
                logger::info("Vehicle model re-created in the carrier's cell");
            } else if (++_freshFrames > 180) {
                logger::warn("Replacement vehicle model never loaded");
                fresh->Disable();
                fresh->SetDelete(true);
                _freshVisual = {};
                _respawnCooldown = 90;
            }
            return;
        }
        if (_respawnCooldown > 0 || !_vehicle->visualCellMismatch()) {
            return;
        }
        auto* handler = RE::TESDataHandler::GetSingleton();
        auto old = _vehicle->visualHandle().get();
        if (!handler || !old || !old->GetObjectReference()) {
            return;
        }
        RE::NEW_REFR_DATA data;
        data.location = physics::toGame(_vehicle->poseFromBody().pos);
        data.direction = RE::NiPoint3(0, 0, 0);
        data.object = old->GetObjectReference();
        data.interior = cell->IsInterior() ? cell : nullptr;
        data.world = cell->IsExterior() ? cell->worldSpace : nullptr;
        data.clearStillLoadingFlag = true;
        data.initializeScripts = true;
        const auto handle = handler->CreateReferenceAtLocation(data);
        if (!handle.get()) {
            _respawnCooldown = 90;
            return;
        }
        _freshVisual = handle;
        _freshFrames = 0;
        logger::info("Vehicle model is in another cell than the carrier: creating it again where the carrier is");
    }

    void VehicleFrameworkVR::bindPendingVehicle()
    {
        if (!_pendingSpec) {
            return;
        }
        auto ref = _pendingRef.get();
        if (!ref) {
            logger::warn("Spawned test vehicle disappeared before it loaded");
            _pendingSpec.reset();
            return;
        }
        auto visual = _pendingVisual.get();
        if (!ref->Get3D() || (visual && !visual->Get3D())) {
            if (++_pendingFrames > PENDING_BIND_FRAMES) {
                logger::error("Test vehicle 3D never loaded");
                _pendingSpec.reset();
            }
            return;
        }
        std::string error;
        bool retryable = true;
        _vehicle = Vehicle::create(ref.get(), visual.get(), *_pendingSpec, error, retryable);
        if (_vehicle) {
            _pendingSpec.reset();
            _lastSimpleMode = g_config.simpleMode;
            restoreVehicleState();
            f4vr::showNotification("Test vehicle ready");
            return;
        }
        if (!retryable || ++_pendingFrames > PENDING_BIND_FRAMES) {
            logger::error("Test vehicle could not be made drivable: {}", error);
            f4vr::showNotification("Test vehicle could not be made drivable (see log)");
            _pendingSpec.reset();
        } else if (_pendingFrames % 30 == 1) {
            logger::info("Waiting for the test vehicle body: {}", error);
        }
    }

    model::DriverInput VehicleFrameworkVR::readDriverInput() const
    {
        using vrcf::Axis;
        using vrcf::Hand;
        model::DriverInput in;
        in.throttle = shapeAxis(vrcf::VRControllers.getAxisValue(Hand::Primary, Axis::Trigger).x);
        in.brake = shapeAxis(vrcf::VRControllers.getAxisValue(Hand::Offhand, Axis::Trigger).x);
        const auto stick = vrcf::VRControllers.getThumbstickValue(Hand::Offhand);
        in.steer = shapeAxis(stick.x);
        const auto handbrakeStick = g_config.handbrakeStick == Hand::Offhand ? stick : vrcf::VRControllers.getThumbstickValue(g_config.handbrakeStick);
        in.handbrake = g_config.handbrakeStickThreshold > 0.0f && handbrakeStick.y <= -g_config.handbrakeStickThreshold;
        return in;
    }

    void VehicleFrameworkVR::updateDoorZones()
    {
        // Each zone keys its own input suppression, so every handle needs a key of its own.
        static constexpr std::array<const char*, 8> ZONE_KEYS{ "VFVR_DoorHandle0", "VFVR_DoorHandle1", "VFVR_DoorHandle2", "VFVR_DoorHandle3", "VFVR_DoorHandle4",
            "VFVR_DoorHandle5", "VFVR_DoorHandle6", "VFVR_DoorHandle7" };
        static const f4vr::ActivationIconStyle ICON{ .size = 8.0f };

        const bool trunkBusy = Trunk::get().isBusy();
        if ((_trunkWasBusy && !trunkBusy) || _seat.isDriving() != _wasDriving) {
            _zonesWaitRelease = true;
        }
        _trunkWasBusy = trunkBusy;
        _wasDriving = _seat.isDriving();
        const auto nowMs = common::nowMillis();
        if (_zonesWaitRelease) {
            const auto held = [](const vrcf::InputBinding& b) {
                return b.isEnabled() && b.type != vrcf::ActivationType::AxisDirection && vrcf::VRControllers.isPressHeldDown(b.hand, b.button);
            };
            if (!held(g_config.enterBinding) && !held(g_config.trunkBinding)) {
                _zonesWaitRelease = false;
                _zonesBlockedUntilMs = nowMs + 1000;
            }
        }
        const bool zonesBlocked = _zonesWaitRelease || nowMs < _zonesBlockedUntilMs;

        const bool available = _vehicle && _vehicle->isValid() && !_seat.isDriving() && !_menuOpen && !trunkBusy && !zonesBlocked;
        auto* root = available ? _vehicle->ref()->Get3D() : nullptr;
        auto* node = root ? root->IsNode() : nullptr;
        const auto handles = node && g_config.enterBinding.isEnabled() ? _vehicle->spec().doorHandlePoints() : std::vector<model::Vec3>{};
        const std::size_t count = (std::min)(handles.size(), ZONE_KEYS.size());
        while (_doorZones.size() < count) {
            _doorZones.push_back(std::make_unique<f4vr::WandActivationSphere>(ZONE_KEYS[_doorZones.size()]));
        }

        // The zones are measured off the vehicle's node; the handles are in the vehicle frame, which the node may be turned against (modelForward),
        // so each handle goes to the world and back into the node's frame. The zone's radius is its scale * the node's scale * the sphere's 0.5.
        const model::Pose pose = node ? _vehicle->poseFromBody() : model::Pose{};
        const float nodeScale = node && node->world.scale > 0.0001f ? node->world.scale : 1.0f;
        const float zoneScale = node ? _vehicle->spec().interactRadius * model::HAVOK_TO_GAME / (0.5f * nodeScale) : 1.0f;

        // The trunk, with the vehicle stopped, and the repair point of a damaged vehicle. Their spheres can overlap a door handle's (the motorcycle's
        // saddlebags sit right behind the seat; a short bonnet is near the front door), so the zone whose point is nearest the hand takes the press.
        const bool trunkAvailable = node && _vehicle->spec().hasTrunk && g_config.trunkBinding.isEnabled() && std::abs(_vehicle->speed()) < TRUNK_MAX_SPEED;
        const model::Vec3 trunkWorld = trunkAvailable ? pose.toWorld(_vehicle->spec().trunkPoint) : model::Vec3{};
        const bool needsRepair = node && g_config.vehicleHealth &&
            (_vehicle->health().health() < _vehicle->health().maxHealth() - 0.5f || _vehicle->radShield().remaining() < _vehicle->radShield().capacity() - 0.5f);
        const bool repairAvailable = needsRepair && g_config.repairBinding.isEnabled() && std::abs(_vehicle->speed()) < TRUNK_MAX_SPEED;
        const model::Vec3 repairWorld = repairAvailable ? pose.toWorld(_vehicle->repairPoint()) : model::Vec3{};
        enum class Zone
        {
            Doors,
            Trunk,
            Repair
        };
        Zone preferred = count > 0 ? Zone::Doors : trunkAvailable ? Zone::Trunk : Zone::Repair;
        if (const auto* nodes = f4vr::getVRPlayerNodes(); node && nodes && nodes->primaryWandNode) {
            const model::Vec3 hand = physics::toMeters(nodes->primaryWandNode->world.translate);
            float nearest = 1e9f;
            const auto consider = [&](const bool available, const model::Vec3& point, const Zone zone) {
                if (!available) {
                    return;
                }
                const float distance = model::length(hand - point);
                if (distance < nearest) {
                    nearest = distance;
                    preferred = zone;
                }
            };
            for (std::size_t i = 0; i < count; ++i) {
                consider(true, pose.toWorld(handles[i]), Zone::Doors);
            }
            consider(trunkAvailable, trunkWorld, Zone::Trunk);
            consider(repairAvailable, repairWorld, Zone::Repair);
        }
        const bool preferTrunk = trunkAvailable && preferred == Zone::Trunk;
        const bool preferRepair = repairAvailable && preferred == Zone::Repair;
        _zonesActive = preferTrunk || preferRepair || (count > 0 && preferred == Zone::Doors);
        RE::NiTransform trunkZone{};
        if (trunkAvailable) {
            trunkZone.translate = common::MatrixUtils::worldToLocalPoint(node->world, physics::toGame(trunkWorld));
            trunkZone.scale = zoneScale;
        }
        _trunkZone.onFrameUpdate(
            f4vr::WandActivationSphere::Frame{
                .enabled = preferTrunk,
                .node = node,
                .zone = trunkZone,
                .bindings = { { g_config.trunkBinding, vrcf::HapticPattern::DoubleClick } },
                .entryHaptic = vrcf::HapticPattern::Tick,
                .showIcon = f4vr::ActivationSphereVisibility::WhenInside,
                .iconStyle = &ICON,
            },
            [&](const vrcf::InputBinding&) {
                const auto& spec = _vehicle->spec();
                const auto hidden = physics::toGame(pose.pos) - RE::NiPoint3(0, 0, TRUNK_HIDE_DEPTH);
                return Trunk::get().open(spec.id, spec.displayName, spec.trunkCapacity, hidden);
            });

        bool entered = false;
        for (std::size_t i = 0; i < _doorZones.size(); ++i) {
            RE::NiTransform zone{};
            if (i < count) {
                zone.translate = common::MatrixUtils::worldToLocalPoint(node->world, physics::toGame(pose.toWorld(handles[i])));
                zone.scale = zoneScale;
            }
            _doorZones[i]->onFrameUpdate(
                f4vr::WandActivationSphere::Frame{
                    .enabled = i < count && !entered && preferred == Zone::Doors,
                    .node = node,
                    .zone = zone,
                    .bindings = { { g_config.enterBinding, vrcf::HapticPattern::DoubleClick } },
                    .entryHaptic = vrcf::HapticPattern::Tick,
                    .showIcon = f4vr::ActivationSphereVisibility::WhenInside,
                    .iconStyle = &ICON,
                },
                [&](const vrcf::InputBinding&) {
                    if (!_seat.canEnter(*_vehicle)) {
                        return false;
                    }
                    _seat.enter(*_vehicle);
                    entered = true;
                    return true;
                });
        }

        // The repair: the repair binding held at the repair point for damage.repair.seconds (advanceRepair counts the hold).
        RE::NiTransform repairZone{};
        if (repairAvailable) {
            repairZone.translate = common::MatrixUtils::worldToLocalPoint(node->world, physics::toGame(repairWorld));
            repairZone.scale = zoneScale;
        }
        _repairZone.onFrameUpdate(
            f4vr::WandActivationSphere::Frame{
                .enabled = preferRepair,
                .node = node,
                .zone = repairZone,
                .bindings = { { g_config.repairBinding, vrcf::HapticPattern::DoubleClick } },
                .entryHaptic = vrcf::HapticPattern::Tick,
                .showIcon = f4vr::ActivationSphereVisibility::WhenInside,
                .iconStyle = &ICON,
            },
            [&](const vrcf::InputBinding&) { return advanceRepair(); });
    }

    void VehicleFrameworkVR::handleSeatBindings()
    {
        updateDoorZones();
        if (!_vehicle || !_vehicle->isValid() || !_seat.isDriving()) {
            return;
        }
        if (g_config.exitBinding.isEnabled() && vrcf::VRControllers.check(g_config.exitBinding)) {
            _seat.exit(_vehicle.get(), "exit button");
        } else if (_menuOpen) {
            return; // the menu uses the same buttons (thumbstick to move, click to choose)
        } else if (g_config.lightsBinding.isEnabled() && vrcf::VRControllers.check(g_config.lightsBinding)) {
            toggleLights();
        } else if (g_config.recenterBinding.isEnabled() && vrcf::VRControllers.check(g_config.recenterBinding)) {
            _seat.recenter(*_vehicle);
        } else if (g_config.thirdPersonBinding.isEnabled() && vrcf::VRControllers.check(g_config.thirdPersonBinding)) {
            _seat.toggleThirdPerson();
        }
    }

    namespace
    {
        // The engine calls this when a button of the message box is pressed (UI thread / game thread): only record the choice, the
        // frame update applies it.
        class WheelMenuCallback : public RE::IMessageBoxCallback
        {
        public:
            explicit WheelMenuCallback(std::atomic<int>* choice, std::atomic<bool>* open)
                : _choice(choice), _open(open)
            {}

            void operator()(const std::uint8_t buttonIndex) override
            {
                _choice->store(static_cast<int>(buttonIndex));
                _open->store(false);
            }

        private:
            std::atomic<int>* _choice;
            std::atomic<bool>* _open;
        };
    }

    void VehicleFrameworkVR::showWheelMenu()
    {
        if (!_vehicle || !_vehicle->isValid() || _menuOpen) {
            return;
        }
        const auto& spec = _vehicle->spec();
        std::vector<const model::PartSpec*> options;
        for (const auto& part : spec.parts) {
            if (part.slot == "wheels" && options.size() < 4) {
                options.push_back(&part);
            }
        }
        if (options.size() < 2) {
            return;
        }
        std::string current = _wheelSet;
        _menuOptionIds.clear();
        std::vector<std::string> labels;
        for (std::size_t i = 0; i < options.size(); ++i) {
            _menuOptionIds.push_back(options[i]->id);
            labels.push_back(options[i]->name);
            if (spec.selection.contains("wheels") ? spec.selection.at("wheels") == options[i]->id : false) {
                current = options[i]->name;
                labels[i] += " (fitted)";
            }
        }
        if (!openMenu(std::format("Wheels fitted: {}.\nChoose the wheels for this vehicle.", current), labels)) {
            f4vr::showNotification(std::format("Wheels: {} (change sWheelSet in the INI)", _wheelSet));
            return;
        }
        _menuKind = MenuKind::Wheels;
        logger::info("Wheel menu shown ({} options)", options.size());
    }

    bool VehicleFrameworkVR::openMenu(const std::string& body, const std::vector<std::string>& labels)
    {
        if (_menuOpen || labels.size() < 2) {
            return false;
        }
        // The message box code is looked up by address library ID: a missing ID ends the game on first use, so check first.
        auto* manager = physics::hasAddressId(959572) && physics::hasAddressId(89563) ? RE::MessageMenuManager::GetSingleton() : nullptr;
        if (!manager) {
            logger::warn("Menu: the message box is not available in this game version");
            return false;
        }
        auto* callback = new WheelMenuCallback(&_menuChoice, &_menuOpen);
        _menuChoice = -1;
        _menuOpen = true;
        _menuShownFrame = _frame;
        manager->Create("Vehicle Framework VR",
            body.c_str(),
            callback,
            RE::WARNING_TYPES::kInGameMessage,
            labels[0].c_str(),
            labels[1].c_str(),
            labels.size() > 2 ? labels[2].c_str() : nullptr,
            labels.size() > 3 ? labels[3].c_str() : nullptr,
            true);
        // The seat suppresses every controller input while driving: the menu needs them (pointer and buttons) until it closes.
        _seat.setGameInputSuppressed(false);
        _menuInputReleased = true;
        return true;
    }

    void VehicleFrameworkVR::beginSpawn(RE::NiPoint3 groundPoint, const float heading)
    {
        // Every profile listed in sVehicleProfiles that loads, with its display name.
        _vehicleIds.clear();
        std::vector<std::string> labels;
        std::string list = g_config.vehicleProfiles;
        std::size_t pos = 0;
        while (pos <= list.size() && _vehicleIds.size() < 4) {
            auto end = list.find(',', pos);
            if (end == std::string::npos) {
                end = list.size();
            }
            std::string id = list.substr(pos, end - pos);
            pos = end + 1;
            const auto b = id.find_first_not_of(" \t");
            const auto e = id.find_last_not_of(" \t");
            if (b == std::string::npos) {
                continue;
            }
            id = id.substr(b, e - b + 1);
            if (const auto spec = loadProfile(id, {}, true)) {
                _vehicleIds.push_back(id);
                labels.push_back(spec->displayName);
            }
        }
        _chosenProfile.clear();
        if (_vehicleIds.size() < 2) {
            if (!_vehicleIds.empty()) {
                _chosenProfile = _vehicleIds[0];
            }
            _wheelMenuPending = g_config.wheelMenuAfterSpawn;
            spawnTestVehicle(groundPoint, heading);
            return;
        }
        if (!openMenu("Choose the vehicle to spawn.", labels)) {
            spawnTestVehicle(groundPoint, heading);
            return;
        }
        _menuKind = MenuKind::Vehicle;
        _spawnChoice = SpawnChoice{ groundPoint, heading };
        logger::info("Vehicle menu shown ({} options)", labels.size());
    }

    void VehicleFrameworkVR::updateMenus()
    {
        const int choice = _menuChoice.exchange(-1);
        if (choice >= 0) {
            const auto kind = std::exchange(_menuKind, MenuKind::None);
            if (kind == MenuKind::Vehicle) {
                if (_spawnChoice && static_cast<std::size_t>(choice) < _vehicleIds.size()) {
                    _chosenProfile = _vehicleIds[static_cast<std::size_t>(choice)];
                    _wheelSet.clear();
                    _wheelMenuPending = g_config.wheelMenuAfterSpawn;
                    logger::info("Vehicle chosen: '{}'", _chosenProfile);
                    spawnTestVehicle(_spawnChoice->point, _spawnChoice->heading);
                } else {
                    logger::info("Vehicle menu closed without a choice (button {})", choice);
                }
                _spawnChoice.reset();
            } else if (kind == MenuKind::Wheels) {
                if (static_cast<std::size_t>(choice) < _menuOptionIds.size()) {
                    selectWheelSet(_menuOptionIds[static_cast<std::size_t>(choice)]);
                } else {
                    logger::info("Wheel menu closed without a choice (button {})", choice);
                }
            }
        }
        // A menu that never showed (or closed without calling back) must not leave the driver without controls.
        if (_menuOpen && _frame - _menuShownFrame > 45) {
            const auto* ui = RE::UI::GetSingleton();
            const bool shown = ui && ui->GetMenuOpen("MessageBoxMenu");
            if (!shown || _frame - _menuShownFrame > 90 * 120) {
                logger::info("Menu: no message box on screen, controls back");
                _menuOpen = false;
                _menuKind = MenuKind::None;
                _spawnChoice.reset();
            }
        }
        // The menu is gone: the driving controls are suppressed for the game again.
        if (_menuInputReleased && !_menuOpen) {
            _menuInputReleased = false;
            _seat.setGameInputSuppressed(true);
        }
        // The wheel menu of the vehicle just chosen, once it is ready.
        if (_wheelMenuPending && !_menuOpen) {
            if (_vehicle && _vehicle->isValid()) {
                _wheelMenuPending = false;
                showWheelMenu();
            } else if (!_pendingSpec) {
                _wheelMenuPending = false;
            }
        }
    }

    void VehicleFrameworkVR::selectWheelSet(const std::string& id)
    {
        _wheelSet = id;
        if (!_vehicle || !_vehicle->isValid()) {
            return;
        }
        const auto& current = _vehicle->spec().selection;
        if (current.contains("wheels") && current.at("wheels") == id) {
            return;
        }
        if (std::abs(_vehicle->speed()) > 1.5f) {
            f4vr::showNotification("Stop the vehicle to change the wheels");
            return;
        }
        auto spec = loadProfile(_profileName.empty() ? g_config.testVehicleProfile : _profileName, { { "wheels", id } });
        if (!spec) {
            return;
        }
        const auto* part = model::findPart(*spec, "wheels", id);
        const std::string name = part ? part->name : id;
        _vehicle->applySpec(std::move(*spec));
        f4vr::showNotification(std::format("Wheels: {}", name));
    }

    void VehicleFrameworkVR::updateWheelSet()
    {
        // A change of sWheelSet in the INI (live reload) applies to the vehicle that is out.
        if (g_config.wheelSet != _lastConfigWheelSet) {
            _lastConfigWheelSet = g_config.wheelSet;
            selectWheelSet(g_config.wheelSet);
        }
    }

    void VehicleFrameworkVR::updateHealth()
    {
        if (g_config.vehicleHealth) {
            HitSink::get().ensureHooked(); // once the player exists; nothing after the first time
        }
        const bool valid = _vehicle && _vehicle->isValid();
        const bool seated = valid && _seat.isDriving();
        // The driver's cover follows the seat (in vehicles whose cabin covers the driver); the hits, the shielding and the smoke follow the vehicle.
        const bool covered = seated && g_config.vehicleHealth && _vehicle->spec().damage.occupantProtected;
        OccupantShield::get().update(covered, g_config.occupantProtection);
        Vehicle* vehicle = valid ? _vehicle.get() : nullptr;
        auto* vehicleRef = valid ? _vehicle->ref() : nullptr;
        HitSink::get().setWatched(vehicleRef ? vehicleRef->GetFormID() : 0u);
        HitSink::get().onFrameUpdate(vehicle, seated);
        RadProtection::get().onFrameUpdate(g_config.vehicleHealth ? vehicle : nullptr, seated);
        _damageSmoke.update(vehicle);
        VitalsProbe::get().onFrameUpdate(seated);
        if (!valid) {
            _conditionKnown = false;
            return;
        }
        // The condition changes: a log line and a notification.
        const auto condition = _vehicle->health().condition();
        if (!_conditionKnown) {
            _conditionKnown = true;
            _lastCondition = condition;
        } else if (condition != _lastCondition) {
            const auto& name = _vehicle->spec().displayName;
            const char* word = condition == model::VehicleCondition::Disabled ? "DISABLED" : condition == model::VehicleCondition::Damaged ? "damaged" : "operational";
            switch (condition) {
            case model::VehicleCondition::Disabled:
                f4vr::showNotification(std::format("{} is DISABLED: get out and repair it at the bonnet", name));
                break;
            case model::VehicleCondition::Damaged:
                f4vr::showNotification(std::format("{} is damaged", name));
                break;
            default:
                f4vr::showNotification(std::format("{} is running again", name));
                break;
            }
            logger::info("VehicleHealth: {} is now {} (health {:.0f} / {:.0f})", name, word, _vehicle->health().health(), _vehicle->health().maxHealth());
            _lastCondition = condition;
        }
        // The save game remembers the vehicle's state by profile.
        if (g_config.rememberVehicleState) {
            HealthStore::get().remember(_vehicle->spec().id, _vehicle->health().health(), _vehicle->radShield().remaining());
        }
        // A repair hold that stopped (the hand left the point or the button was released).
        if (_repairLastMs != 0 && _frame - _repairFrame > 15) {
            logger::info("VehicleHealth: repair of {} interrupted after {:.1f} s", _vehicle->spec().displayName, _repairHeld);
            _repairHeld = 0.0f;
            _repairLastMs = 0;
        }
    }

    void VehicleFrameworkVR::restoreVehicleState()
    {
        if (!g_config.rememberVehicleState || !_vehicle) {
            return;
        }
        const auto remembered = HealthStore::get().lookup(_vehicle->spec().id);
        if (!remembered) {
            return;
        }
        if (remembered->health >= 0.0f) {
            _vehicle->health().setHealth(remembered->health);
        }
        if (remembered->shield >= 0.0f) {
            _vehicle->radShield().setRemaining(remembered->shield);
        }
        _conditionKnown = false;
        logger::info("VehicleHealth: {} takes over its remembered state: health {:.0f} / {:.0f}, radiation reserve {:.0f} / {:.0f}{}",
            _vehicle->spec().displayName,
            _vehicle->health().health(),
            _vehicle->health().maxHealth(),
            _vehicle->radShield().remaining(),
            _vehicle->radShield().capacity(),
            _vehicle->health().disabled() ? " - DISABLED" : "");
    }

    bool VehicleFrameworkVR::advanceRepair()
    {
        if (!_vehicle || !_vehicle->isValid()) {
            return false;
        }
        const auto& spec = _vehicle->spec();
        const auto nowMs = common::nowMillis();
        const bool starting = _repairLastMs == 0;
        const float dt = starting ? 0.0f : (std::min)(static_cast<float>(nowMs - _repairLastMs) * 0.001f, 0.1f);
        if (starting) {
            _repairHeld = 0.0f;
            logger::info("VehicleHealth: repairing {} (hold for {:.1f} s)", spec.displayName, spec.damage.repairSeconds);
            f4vr::showNotification(std::format("Repairing {}: hold for {:.0f} s", spec.displayName, spec.damage.repairSeconds));
        }
        _repairHeld += dt;
        _repairLastMs = nowMs;
        _repairFrame = _frame;
        if (_repairHeld < (std::max)(spec.damage.repairSeconds, 0.1f)) {
            return false;
        }
        _vehicle->health().repair(spec.damage.repairHealthShare);
        _vehicle->radShield().repair(1.0f);
        _repairHeld = 0.0f;
        _repairLastMs = 0;
        logger::info("VehicleHealth: {} repaired: health {:.0f} / {:.0f}, radiation reserve {:.0f} / {:.0f}",
            spec.displayName,
            _vehicle->health().health(),
            _vehicle->health().maxHealth(),
            _vehicle->radShield().remaining(),
            _vehicle->radShield().capacity());
        f4vr::showNotification(std::format("{} repaired", spec.displayName));
        return true;
    }

    void VehicleFrameworkVR::onFrameUpdate()
    {
        const auto start = nowNanos();
        F4CF_PERF_SCOPE("VehicleFrameworkVR");
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player || !player->loadedData) {
            logger::sample(3000, "no player data - noop");
            return;
        }
        ++_frame;

        updateMenus();
        handleSpawnRequests();
        bindPendingVehicle();

        if (_vehicle) {
            if (_lastSimpleMode != g_config.simpleMode) {
                _lastSimpleMode = g_config.simpleMode;
                _vehicle->setSimpleMode(_lastSimpleMode);
            }
            updateWheelSet();
            // Nobody at the wheel: the handbrake stays on, so a car left at speed brakes to a stop instead of rolling away.
            model::DriverInput parkedInput{};
            parkedInput.handbrake = true;
            const auto input = _seat.isDriving() ? (_menuOpen ? model::DriverInput{} : readDriverInput()) : parkedInput;
            _vehicle->onFrameUpdate(input, _seat.isDriving(), _physicsLog.isOpen() ? &_physicsLog : nullptr, _motoLog.isOpen() ? &_motoLog : nullptr);
            if (!_vehicle->isValid()) {
                releaseVehicle(false, "vehicle unloaded");
            }
        }

        if (g_config.engineSound) {
            _engineSound.start();
            if (_vehicle && _vehicle->isValid() && _seat.isDriving() && !_vehicle->health().disabled()) {
                const auto tel = _vehicle->telemetry();
                _engineSound.setState(true, tel.engineRpm, tel.throttle, g_config.engineVolume, engineVoiceOf(_vehicle->spec().engine));
            } else {
                _engineSound.setState(false, 900.0f, 0.0f, g_config.engineVolume, EngineVoice{});
            }
        }

        {
            // A disabled vehicle's engine is dead: no hum, no exhaust (the damage smoke takes over).
            const bool drivingNow = _vehicle && _vehicle->isValid() && _seat.isDriving() && !_vehicle->health().disabled();
            const auto tel = drivingNow ? _vehicle->telemetry() : model::VehicleTelemetry{};
            updateExhaust(drivingNow, tel.throttle);
        }
        keepVisualInCarrierCell();
        updateLights();
        Trunk::get().onFrameUpdate();
        handleSeatBindings();
        updateHealth();
        updateBodyHiding();
        logPlayerBody();
        if (_seat.isDriving()) {
            const auto stick = vrcf::VRControllers.getThumbstickValue(vrcf::Hand::Primary);
            // The menu is moved with this same stick: the chase camera must not follow it. Nor while the stick pulls the handbrake (down, with
            // some sideways wobble).
            const bool handbrakeHeld = g_config.handbrakeStick == vrcf::Hand::Primary && g_config.handbrakeStickThreshold > 0.0f &&
                stick.y <= -g_config.handbrakeStickThreshold;
            const bool still = _menuOpen || handbrakeHeld;
            _seat.setCameraStick(still ? 0.0f : stick.x, still ? 0.0f : stick.y);
            if (_menuOpen) {
                _seat.resetOrbit();
            }
        }

        if (_vehicle && _seat.isDriving()) {
            // FRIK's FrameBegin places the driver when it is in use and arriving; otherwise do it here.
            const bool frikPlaces = g_config.cameraUpdatePoint == CameraUpdatePoint::FrikFrameBegin && _frikFrameCallback && _frame - _frikLastFrame <= 2;
            if (!frikPlaces) {
                _seat.updateCamera(*_vehicle, _cameraLog.isOpen() ? &_cameraLog : nullptr, false);
            }
        }
        if (_vehicle && g_config.debugDraw) {
            _seat.drawDebug(*_vehicle);
        }

        if (--_flushCountdown <= 0) {
            _flushCountdown = 45;
            _physicsLog.flush();
            _motoLog.flush();
            _cameraLog.flush();
        }
        addFrameCost(nowNanos() - start);
    }

    void VehicleFrameworkVR::addFrameCost(const std::uint64_t nanos)
    {
        if (!g_config.costReport) {
            return;
        }
        _cost.gameNanos += nanos;
        _cost.maxFrameNanos = (std::max)(_cost.maxFrameNanos.load(), nanos);
        ++_cost.frames;
        const auto nowMs = common::nowMillis();
        if (_cost.lastReportMs == 0) {
            _cost.lastReportMs = nowMs;
        }
        if (nowMs - _cost.lastReportMs < 5000 || _cost.frames == 0) {
            return;
        }
        const auto steps = Vehicle::consumeStepNanos();
        const double frames = static_cast<double>(_cost.frames);
        const double gameMs = static_cast<double>(_cost.gameNanos.exchange(0)) / frames / 1e6;
        const double stepMs = static_cast<double>(steps) / frames / 1e6;
        if (_vehicle) {
            logger::info("Cost per frame: {:.3f} ms total (game thread {:.3f} ms, physics step {:.3f} ms), worst game-thread frame {:.3f} ms, over {} frames - target < 0.2 ms",
                gameMs + stepMs,
                gameMs,
                stepMs,
                static_cast<double>(_cost.maxFrameNanos.exchange(0)) / 1e6,
                _cost.frames);
        } else {
            _cost.maxFrameNanos = 0;
        }
        _cost.frames = 0;
        _cost.lastReportMs = nowMs;
    }
}
