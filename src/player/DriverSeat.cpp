#include "player/DriverSeat.h"

#include "health/PlayerVitals.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <format>

#include "Config.h"
#include "api/FRIKApiV2.h"
#include "debug/DebugDraw.h"
#include "f4vr/CollisionLayers.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "f4vr/BSFlattenedBoneTree.h"
#include "f4vr/PlayerRotation.h"
#include "model/CameraArm.h"
#include "perf/Perf.h"
#include "render/RenderUtils.h"
#include "physics/HavokBridge.h"
#include "player/RenderProbe.h"
#include "telemetry/CsvLog.h"
#include "vehicle/Vehicle.h"
#include "vrcf/VRControllersSuppressor.h"

namespace vfvr
{
    namespace
    {
        constexpr auto SUPPRESS_KEY = "VehicleFrameworkVR.drive";
        constexpr auto FRIK_TAG = "VehicleFrameworkVR";

        // The seat eye of a vehicle, raised by the global INI offset (the same for every vehicle).
        model::Vec3 eyeLocal(const Vehicle& vehicle)
        {
            auto eye = vehicle.spec().driverEye;
            eye.z += g_config.eyeLiftCm * 0.01f;
            return eye;
        }

        // Rotate a vector about world Z by a heading (game convention: clockwise from +Y).
        model::Vec3 rotH(const model::Vec3& v, const float heading)
        {
            const float c = std::cos(heading);
            const float s = std::sin(heading);
            return { v.x * c + v.y * s, -v.x * s + v.y * c, v.z };
        }

        float headingOfNode(const RE::NiAVObject* node)
        {
            const auto forward = node->world.rotate.Transpose() * RE::NiPoint3(0, 1, 0);
            return model::headingOf({ forward.x, forward.y, forward.z });
        }

        /*
         * The player's character controller. FO4VR keeps it at MiddleHighProcessData + 0x3E8; CommonLibF4VR
         * declares the member at the flat game's 0x3E0, which reads a wrong pointer in VR. Path verified by
         * ROCK (CharacterControllerRuntime.h): Actor+0x300 currentProcess -> +0x08 middleHigh -> +0x3E8.
         */
        RE::bhkCharacterController* playerCharController()
        {
            const auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player) {
                return nullptr;
            }
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(player);
            const auto* process = *reinterpret_cast<const std::uint8_t* const*>(bytes + 0x300);
            if (!process) {
                return nullptr;
            }
            const auto* middleHigh = *reinterpret_cast<const std::uint8_t* const*>(process + 0x08);
            if (!middleHigh) {
                return nullptr;
            }
            return *reinterpret_cast<RE::bhkCharacterController* const*>(middleHigh + 0x3E8);
        }

        /*
         * Virtual calls on the character controller are only made when its vtable matches CommonLib's layout:
         * slot 0x34 must be SetPositionImpl at RVA 0x1E4DF60 (FO4VR 1.2.72, per ROCK). A wrong slot would call
         * an arbitrary function.
         */
        bool charControllerVtableVerified(const RE::bhkCharacterController* controller)
        {
            static int verified = -1;
            if (verified < 0 && controller) {
                const auto* vtable = *reinterpret_cast<const std::uintptr_t* const*>(controller);
                const auto rva = vtable[0x34] - REL::Module::get().base();
                verified = rva == 0x1E4DF60 ? 1 : 0;
                logger::info("Character controller vtable {} (slot 0x34 RVA 0x{:X})", verified ? "verified" : "NOT verified - velocity reset and collision change disabled", rva);
            }
            return verified == 1;
        }

        /*
         * The virtual call above is off on this game build, and the engine then keeps pulling the seated player down (~0.67 m/s, so ~1 cm
         * per frame, more on long frames) between our placements: the camera and hands bob against the cabin. These are plain data members
         * of the controller whose layout the CCDUMP log confirmed (outVelocity 0x160, initialVelocity 0x170, gravity 0x330, actor
         * height 0x348, radius 0x370, height 0x374). Written only when those values look as expected.
         */
        constexpr std::size_t CC_OUT_VELOCITY = 0x160;
        constexpr std::size_t CC_INITIAL_VELOCITY = 0x170;
        constexpr std::size_t CC_GRAVITY = 0x330;
        constexpr std::size_t CC_ACTOR_HEIGHT = 0x348;
        constexpr std::size_t CC_RADIUS = 0x370;
        constexpr std::size_t CC_HEIGHT = 0x374;

        float readCcFloat(const std::uint8_t* bytes, const std::size_t off)
        {
            float v;
            std::memcpy(&v, bytes + off, sizeof(v));
            return v;
        }

        bool charControllerLayoutPlausible(const std::uint8_t* bytes)
        {
            const auto inRange = [&](const std::size_t off, const float lo, const float hi) {
                const float v = readCcFloat(bytes, off);
                return std::isfinite(v) && v >= lo && v <= hi;
            };
            return inRange(CC_ACTOR_HEIGHT, 0.8f, 3.5f) && inRange(CC_HEIGHT, 0.8f, 3.5f) && inRange(CC_RADIUS, 0.05f, 1.0f) && inRange(CC_GRAVITY, 0.0f, 20.0f);
        }

        float g_savedGravity = -1.0f;

        void setPlayerGravity(const bool driving)
        {
            auto* controller = playerCharController();
            if (!controller) {
                return;
            }
            auto* bytes = reinterpret_cast<std::uint8_t*>(controller);
            if (!charControllerLayoutPlausible(bytes)) {
                logger::sample(5000, "Player controller layout does not look as expected: gravity left alone");
                return;
            }
            const float zero = 0.0f;
            if (driving) {
                const float current = readCcFloat(bytes, CC_GRAVITY);
                if (g_savedGravity < 0.0f && current > 0.01f) {
                    g_savedGravity = current;
                    std::memcpy(bytes + CC_GRAVITY, &zero, sizeof(zero));
                    logger::info("Player controller: gravity {:.3f} -> 0 while driving", current);
                }
            } else if (g_savedGravity >= 0.0f) {
                std::memcpy(bytes + CC_GRAVITY, &g_savedGravity, sizeof(g_savedGravity));
                logger::info("Player controller: gravity back to {:.3f}", g_savedGravity);
                g_savedGravity = -1.0f;
            }
        }

        void zeroPlayerVelocity()
        {
            auto* controller = playerCharController();
            if (!controller) {
                return;
            }
            if (charControllerVtableVerified(controller)) {
                const RE::hkVector4f zero;
                controller->SetLinearVelocityImpl(zero);
                return;
            }
            auto* bytes = reinterpret_cast<std::uint8_t*>(controller);
            if (g_config.playerNoGravity && charControllerLayoutPlausible(bytes)) {
                const float zero = 0.0f;
                for (std::size_t i = 0; i < 4; ++i) {
                    std::memcpy(bytes + CC_OUT_VELOCITY + i * 4, &zero, sizeof(zero));
                    std::memcpy(bytes + CC_INITIAL_VELOCITY + i * 4, &zero, sizeof(zero));
                }
            }
        }

        // Diagnostic: the character controller's floats (offset: value), to find which member drags the seated player down each frame.
        // Read only; the VR layout differs from the header's, so nothing is written.
        void dumpControllerFloats(const char* tag)
        {
            const auto* controller = playerCharController();
            if (!controller) {
                return;
            }
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(controller);
            std::string out;
            for (std::size_t off = 0x140; off < 0x450; off += 4) {
                float v;
                std::memcpy(&v, bytes + off, sizeof(v));
                if (std::isfinite(v) && std::abs(v) > 1.0e-4f && std::abs(v) < 1.0e5f) {
                    out += std::format(" {:X}:{:.4g}", off, v);
                }
            }
            logger::info("CCDUMP {}:{}", tag, out);
        }

        // Wall clock in milliseconds since the first call, for lining the camera CSV up with the other logs.
        double wallMillis()
        {
            static const auto start = std::chrono::steady_clock::now();
            return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        }

        // The eye position the engine rendered its last frame with: the middle of the two per-eye offsets of its camera constants (metres), or NaN.
        model::Vec3 renderCameraEye()
        {
            f4cf::render::StereoCameraConstants c{};
            if (!f4cf::render::getStereoCameraConstants(c)) {
                const float nan = std::nanf("");
                return { nan, nan, nan };
            }
            const RE::NiPoint3 mid((c.posAdjust[0].x + c.posAdjust[1].x) * 0.5f, (c.posAdjust[0].y + c.posAdjust[1].y) * 0.5f, (c.posAdjust[0].z + c.posAdjust[1].z) * 0.5f);
            return physics::toMeters(mid);
        }

        // The cabin's pitch and roll, scaled down around the eye point: the head stays level with the world (the room only follows the heading), so on
        // bumps the whole cabin would otherwise rock in front of the eyes. Small tilts are scaled by `scale`; beyond ~35 deg (ramps, flips) the true
        // pose is used, so a rolling car still rolls.
        model::Pose attenuateTilt(const model::Pose& pose, const model::Vec3& eyeLocalPos, const float scale)
        {
            const float heading = model::headingOf(pose.rot.axisY());
            const model::Quat levelRot = model::Quat::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -heading);
            model::Quat tilt = (levelRot.conjugate() * pose.rot).normalized();
            if (tilt.w < 0.0f) {
                tilt = { -tilt.x, -tilt.y, -tilt.z, -tilt.w };
            }
            const float angle = 2.0f * std::acos(std::clamp(tilt.w, -1.0f, 1.0f));
            const float sinHalf = std::sqrt((std::max)(0.0f, 1.0f - tilt.w * tilt.w));
            if (!(angle > 1e-4f) || !(sinHalf > 1e-6f)) {
                return pose;
            }
            const float f = std::clamp((angle * 180.0f / model::PI - 10.0f) / 25.0f, 0.0f, 1.0f);
            const float effective = scale + (1.0f - scale) * f * f * (3.0f - 2.0f * f);
            const model::Vec3 axis{ tilt.x / sinHalf, tilt.y / sinHalf, tilt.z / sinHalf };
            model::Pose out;
            out.rot = (levelRot * model::Quat::fromAxisAngle(axis, angle * effective)).normalized();
            out.pos = pose.toWorld(eyeLocalPos) - out.rot.rotate(eyeLocalPos);
            return out;
        }

        float elapsedSeconds()
        {
            static auto last = std::chrono::steady_clock::now();
            const auto now = std::chrono::steady_clock::now();
            const float dt = std::chrono::duration<float>(now - last).count();
            last = now;
            return std::clamp(dt, 0.0f, 0.1f);
        }
    }

    bool DriverSeat::readRoom(RoomState& out) const
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        const auto* nodes = f4vr::getVRPlayerNodes();
        if (!player || !nodes || !nodes->roomNode || !nodes->hmdNode) {
            return false;
        }
        const auto* room = nodes->roomNode;
        out.origin = physics::toMeters(room->world.translate);
        out.heading = _headingTracked ? _lastTargetRoomHeading : headingOfNode(room);
        out.hmdHeading = headingOfNode(nodes->hmdNode);
        out.scale = room->world.scale > 0.01f ? room->world.scale : 1.0f;
        out.hmdWorld = physics::toMeters(nodes->hmdNode->world.translate);
        out.hmdLocal = rotH(out.hmdWorld - out.origin, -out.heading) / out.scale;
        out.hmdLocalHeading = model::wrapAngle(out.hmdHeading - out.heading);
        out.actorWorld = physics::toMeters(player->GetPosition());
        out.actorLocal = rotH(out.actorWorld - out.origin, -out.heading) / out.scale;
        return true;
    }

    model::Pose DriverSeat::vehiclePose(const Vehicle& vehicle) const
    {
        const auto pose = g_config.seatPoseSource == SeatPoseSource::HavokBody ? vehicle.poseForSeat() : vehicle.poseFromNode();
        return model::scaleRoll(pose, vehicle.spec().seatLeanFollow); // a motorcycle: the camera follows only part of the lean
    }

    model::Vec3 DriverSeat::seatEyeWorld(const Vehicle& vehicle) const
    {
        return vehiclePose(vehicle).toWorld(eyeLocal(vehicle));
    }

    float DriverSeat::headToSeatMeters(const Vehicle& vehicle) const
    {
        RoomState room;
        if (!readRoom(room)) {
            return 1e9f;
        }
        return model::length(room.hmdWorld - seatEyeWorld(vehicle));
    }

    bool DriverSeat::canEnter(const Vehicle& vehicle) const
    {
        return !_driving && vehicle.isValid() && headToSeatMeters(vehicle) <= g_config.enterDistance;
    }

    void DriverSeat::calibrate(const Vehicle& vehicle)
    {
        RoomState room;
        if (!readRoom(room)) {
            return;
        }
        const float carHeading = model::headingOf(vehiclePose(vehicle).rot.axisY());
        _hmdLocal0 = room.hmdLocal;
        // Turn the room so that where the head looks now becomes the vehicle's forward.
        _headingOffset = -room.hmdLocalHeading;
        _yaw0 = f4vr::PlayerRotation::getYaw();
        _roomHeading0 = room.heading;
        _lastTargetRoomHeading = room.heading;
        _hmdHeading0 = room.hmdHeading;
        _headingTracked = true;
        _yawSettle = 4;
        _yawCheckFrames = 0;
        _yawConfirmFrames = 0;
        _filterReady = false;
        _hasLastPlaced = false;
        _hasPrevRaw = false;
        _rawHistCount = 0;
        logger::info("Seat: calibrated, head in room ({:.2f}, {:.2f}, {:.2f}) m, head yaw in room {:.1f} deg, room scale {:.3f}, vehicle heading {:.1f} deg",
            room.hmdLocal.x,
            room.hmdLocal.y,
            room.hmdLocal.z,
            room.hmdLocalHeading * 180.0f / model::PI,
            room.scale,
            carHeading * 180.0f / model::PI);
    }

    void DriverSeat::enter(Vehicle& vehicle)
    {
        if (_driving) {
            return;
        }
        _thirdPerson = false;
        _viewBlend = 0.0f;
        _orbitYaw = 0.0f;
        _orbitIdle = 0.0f;
        _driving = true;
        _yawSign = g_config.roomYawSign;
        _yawSignChecked = !g_config.autoDetectYawSign;
        _yawCheckFrames = 0;
        _yawFlips = 0;
        _headingTracked = false;
        _lockSuspended = false;
        _followDecided = false;
        _followSamples = 0;
        _followMovedCm = 0.0f;
        _followAskedCm = 0.0f;
        _hasPrevRaw = false;
        _rawHistCount = 0;
        _hasLastSeatRaw = false; // the first seat step is measured from this drive, not from where the last one ended
        _hasLastActor = false;

        vrcf::VRControllersSuppress.suppressAll(SUPPRESS_KEY);
        f4vr::PlayerRotation::cancel();
        if (auto* player = RE::PlayerCharacter::GetSingleton(); player && g_config.holsterWeaponWhileDriving) {
            _weaponWasDrawn = player->GetWeaponMagicDrawn();
            _lastHolsterMs = 0;
            keepWeaponHolstered();
        }

        using frik::api::FRIKApiV2;
        if (g_config.blockFrikSmoothMovement && FRIKApiV2::inst) {
            _frikBlocked = FRIKApiV2::inst->blockFeature(FRIK_TAG, FRIKApiV2::Feature::SmoothMovement, true);
        }
        setPlayerCollision(true);
        setPlayerBodyLayer(true, &vehicle);
        if (g_config.playerNoGravity) {
            setPlayerGravity(true);
        }
        // Where the head is now, before anything moves the player: the camera glides from there into the seat.
        RoomState startRoom;
        _glideActive = g_config.enterGlideSeconds > 0.0f && readRoom(startRoom);
        if (_glideActive) {
            _glideFromLocal = vehiclePose(vehicle).toLocal(startRoom.hmdWorld);
            _glideStart = std::chrono::steady_clock::now();
        }
        calibrate(vehicle);
        zeroPlayerVelocity();
        logger::info("Seat: entered '{}' (FRIK smooth movement {}, collision {})",
            vehicle.spec().displayName,
            _frikBlocked ? "blocked" : "untouched",
            _collisionChanged ? "off" : "unchanged");
    }

    void DriverSeat::setGameInputSuppressed(const bool suppressed)
    {
        if (!_driving) {
            return;
        }
        if (suppressed) {
            vrcf::VRControllersSuppress.suppressAll(SUPPRESS_KEY);
        } else {
            vrcf::VRControllersSuppress.release(SUPPRESS_KEY);
        }
    }

    void DriverSeat::exit(Vehicle* vehicle, const std::string& reason)
    {
        if (!_driving) {
            return;
        }
        _driving = false;
        _headingTracked = false;
        _glideActive = false;
        _thirdPerson = false; // the next drive starts in the seat (enter() says so too), and the state no longer claims a chase camera
        _viewBlend = 0.0f;
        _bodyMoveValid = false;
        // The weapon comes out again only if the player had it out when getting in.
        if (g_config.holsterWeaponWhileDriving && _weaponWasDrawn) {
            if (auto* player = RE::PlayerCharacter::GetSingleton()) {
                player->DrawWeaponMagicHands(true);
                logger::info("Seat: weapon drawn again (it was out when the player got in)");
            }
        }
        _weaponWasDrawn = false;
        _orbitYaw = 0.0f;
        _hud.update(false, false, {}, {}, 0.0f, {});

        if (vehicle && vehicle->isValid()) {
            // Exit point, dropped onto whatever is below it.
            const auto pose = vehiclePose(*vehicle);
            const model::Vec3 exitWorld = pose.toWorld(vehicle->spec().exitPoint);
            auto* bhkWorld = physics::getBhkWorld(vehicle->ref());
            const auto from = physics::toGame(exitWorld + model::Vec3{ 0, 0, 1.5f });
            const auto to = physics::toGame(exitWorld - model::Vec3{ 0, 0, 3.0f });
            const auto layer = static_cast<std::uint32_t>(RE::COL_LAYER::kCharController);
            const auto hit = physics::castRay(bhkWorld, from, to, layer, vehicle->bodyId(), g_config.wheelRayIgnoreMask, g_config.releasePickWorldReference);
            RE::NiPoint3 target = hit.hit ? hit.point + RE::NiPoint3(0, 0, 4.0f) : physics::toGame(exitWorld);
            if (auto* player = RE::PlayerCharacter::GetSingleton()) {
                player->SetPosition(target, true);
            }
        }

        setPlayerBodyLayer(false, nullptr);
        render_probe::tick(false);
        setPlayerCollision(false);
        setPlayerGravity(false);
        zeroPlayerVelocity();
        using frik::api::FRIKApiV2;
        if (_frikBlocked && FRIKApiV2::inst) {
            FRIKApiV2::inst->blockFeature(FRIK_TAG, FRIKApiV2::Feature::SmoothMovement, false);
        }
        _frikBlocked = false;
        vrcf::VRControllersSuppress.release(SUPPRESS_KEY);
        logger::info("Seat: left the vehicle ({})", reason);
    }

    void DriverSeat::reset()
    {
        if (_driving) {
            vrcf::VRControllersSuppress.release(SUPPRESS_KEY);
            using frik::api::FRIKApiV2;
            if (_frikBlocked && FRIKApiV2::inst) {
                FRIKApiV2::inst->blockFeature(FRIK_TAG, FRIKApiV2::Feature::SmoothMovement, false);
            }
        }
        _driving = false;
        _headingTracked = false;
        _glideActive = false;
        _bodyMoveValid = false;
        _weaponWasDrawn = false; // after a load the game set the weapon state itself
        _legs3rd = {};
        _legs1st = {};
        _hud.update(false, false, {}, {}, 0.0f, {});
        _frikBlocked = false;
        _collisionChanged = false;
        _playerBodyChanged = false;
        _playerBodyTries = 0;
        _playerBody = {};
        _filterReady = false;
    }

    void DriverSeat::recenter(const Vehicle& vehicle)
    {
        if (_driving) {
            calibrate(vehicle);
            f4vr::showNotification("Seat re-centered");
        }
    }

    void DriverSeat::setPlayerCollision(const bool driving)
    {
        if (!g_config.disablePlayerCollision) {
            return;
        }
        auto* controller = playerCharController();
        if (!controller || !charControllerVtableVerified(controller)) {
            return;
        }
        if (driving && !_collisionChanged) {
            const auto layer = f4vr::parseCollisionLayer(g_config.driverCollisionLayer).value_or(RE::COL_LAYER::kNonCollidable);
            _savedCollisionFilter = controller->GetCollisionFilterInfo();
            RE::CFilter filter;
            filter.filter = (_savedCollisionFilter & ~0x7Fu) | static_cast<std::uint32_t>(layer);
            controller->SetCollisionFilterInfo(filter);
            _collisionChanged = true;
            logger::info("Seat: player collision layer {} -> {}", f4vr::getCollisionLayerName(_savedCollisionFilter), f4vr::getCollisionLayerName(filter.filter));
        } else if (!driving && _collisionChanged) {
            RE::CFilter filter;
            filter.filter = _savedCollisionFilter;
            controller->SetCollisionFilterInfo(filter);
            _collisionChanged = false;
        }
    }

    /*
     * With the controller's virtual calls off (see charControllerVtableVerified), its capsule kept colliding with the ground while driving. The seated
     * player's feet are below the road, so the engine pushed the actor out every frame: the ~1 cm sink with jitter that shakes the world against the cabin
     * (the tremor vanishes when the car is flipped or airborne, and with the console's tcl). The controller's own body index is not readable (0.1.49 found
     * it 0), so the capsule is found the way the vehicle finds foreign bodies: rays towards the actor from around it. The one body on the charController
     * layer that the rays hit within the capsule's radius of the actor is the player's; it goes to sDriverCollisionLayer, like the vehicle's body does.
     */
    void DriverSeat::setPlayerBodyLayer(const bool driving, const Vehicle* vehicle)
    {
        if (!g_config.disablePlayerCollision) {
            return;
        }
        if (driving && !_playerBodyChanged && vehicle) {
            auto* player = RE::PlayerCharacter::GetSingleton();
            physics::BodyHandle handle = vehicle->bodyHandle();
            if (!player || !handle.bhkWorld || !handle.world) {
                return;
            }
            ++_playerBodyTries;
            const auto actor = physics::toMeters(player->GetPosition());
            constexpr float PROBE_RADIUS = 0.6f; // m, the capsule is ~0.26 m wide
            constexpr float PROBE_START = 2.5f;
            const auto rayLayer = static_cast<std::uint32_t>(RE::COL_LAYER::kCharController);
            std::uint32_t bestBody = 0x7FFF'FFFF;
            std::uint32_t bestFilter = 0;
            float bestDist = 1e9f;
            int hits = 0;
            for (const float height : { 0.5f, 0.9f, 1.3f }) {
                for (int k = 0; k < 12; ++k) {
                    const float a = static_cast<float>(k) * (2.0f * model::PI / 12.0f);
                    const model::Vec3 dir{ std::cos(a), std::sin(a), 0.0f };
                    const model::Vec3 target = actor + model::Vec3{ 0.0f, 0.0f, height };
                    const auto hit = physics::castRay(handle.bhkWorld, physics::toGame(target + dir * PROBE_START), physics::toGame(target), rayLayer, vehicle->bodyId(), 0,
                        g_config.releasePickWorldReference);
                    if (!hit.hit || (hit.filterInfo & 0x7Fu) != static_cast<std::uint32_t>(RE::COL_LAYER::kCharController)) {
                        continue;
                    }
                    const auto point = physics::toMeters(hit.point);
                    const float dist = std::hypot(point.x - actor.x, point.y - actor.y);
                    if (dist < PROBE_RADIUS) {
                        ++hits;
                        if (dist < bestDist) {
                            bestDist = dist;
                            bestBody = hit.bodyId;
                            bestFilter = hit.filterInfo;
                        }
                    }
                }
            }
            if (bestBody == 0x7FFF'FFFF) {
                if (_playerBodyTries == 1 || _playerBodyTries % 30 == 0) {
                    logger::warn("Seat: player body not found by rays around the actor (try {})", _playerBodyTries);
                }
                return;
            }
            handle.collisionObject = reinterpret_cast<RE::bhkNPCollisionObject*>(playerCharController());
            handle.node = nullptr;
            handle.bodyId.value = bestBody;
            if (!handle.isValid()) {
                handle.collisionObject = vehicle->bodyHandle().collisionObject; // only needed to make the handle valid for the layer call
            }
            const auto layer = f4vr::parseCollisionLayer(g_config.driverCollisionLayer).value_or(RE::COL_LAYER::kNonCollidable);
            physics::setBodyLayer(handle, layer);
            _playerBody = handle;
            _playerBodyFilter = bestFilter;
            _playerBodyChanged = true;
            logger::info("Seat: player body {} found ({} ray hits, {:.2f} m from the actor): layer {} -> {} (filter 0x{:08X} -> 0x{:08X})", bestBody, hits, bestDist,
                f4vr::getCollisionLayerName(bestFilter), f4vr::getCollisionLayerName(static_cast<std::uint32_t>(layer)), bestFilter, physics::getFilterInfo(handle));
        } else if (!driving) {
            _playerBodyTries = 0;
            if (_playerBodyChanged) {
                _playerBodyChanged = false;
                // The filter only if the body still is the one we changed (a cell change rebuilds the controller with the engine's own layer).
                if (physics::getBodyLayer(_playerBody) == f4vr::parseCollisionLayer(g_config.driverCollisionLayer).value_or(RE::COL_LAYER::kNonCollidable)) {
                    physics::setBodyLayer(_playerBody, static_cast<RE::COL_LAYER>(_playerBodyFilter & 0x7Fu));
                    logger::info("Seat: player body layer restored to {}", f4vr::getCollisionLayerName(_playerBodyFilter));
                }
                _playerBody = {};
            }
        }
    }

    void DriverSeat::applyRoomYaw(const float targetRoomHeading, const RoomState& room)
    {
        // The engine's VR yaw and the heading it produces may turn in opposite senses. The room node does not
        // show the VR yaw, but the head (HMD) node does: judge the previous command by where the head points now.
        // Intended: the head heading follows `commanded` turn since calibration; mirrored: it went the other way.
        if (!_yawSignChecked && _yawFlips < 2) {
            const float commanded = model::wrapAngle(_lastTargetRoomHeading - _roomHeading0);
            if (_yawSettle > 0) {
                --_yawSettle;
            } else if (std::abs(commanded) > 30.0f * model::PI / 180.0f) {
                const float errIntended = std::abs(model::wrapAngle(room.hmdHeading - (_hmdHeading0 + commanded)));
                const float errMirrored = std::abs(model::wrapAngle(room.hmdHeading - (_hmdHeading0 - commanded)));
                if (errMirrored + 10.0f * model::PI / 180.0f < errIntended) {
                    _yawConfirmFrames = 0;
                    if (++_yawCheckFrames >= 6) {
                        _yawSign = -_yawSign;
                        _yawCheckFrames = 0;
                        _yawSettle = 6;
                        ++_yawFlips;
                        logger::warn("Seat: room yaw direction was reversed - now using fRoomYawSign = {}", _yawSign);
                    }
                } else if (errIntended + 10.0f * model::PI / 180.0f < errMirrored) {
                    _yawCheckFrames = 0;
                    if (++_yawConfirmFrames >= 20) {
                        _yawSignChecked = true;
                        logger::info("Seat: room yaw direction confirmed (fRoomYawSign = {})", _yawSign);
                    }
                }
            }
        }
        f4vr::PlayerRotation::setYaw(_yaw0 + _yawSign * model::wrapAngle(targetRoomHeading - _roomHeading0));
        _lastTargetRoomHeading = targetRoomHeading;
    }

    namespace
    {
        // One transform of the player's 3D, moved rigidly: the position and the bone's axes (this codebase keeps world rotations transposed: row i
        // of `rotate` is where local axis i points).
        void moveTransform(RE::NiTransform& t, const model::Pose& move)
        {
            t.translate = physics::toGame(move.toWorld(physics::toMeters(t.translate)));
            for (int i = 0; i < 3; ++i) {
                const auto axis = move.rot.rotate({ t.rotate.entry[i][0], t.rotate.entry[i][1], t.rotate.entry[i][2] });
                t.rotate.entry[i][0] = axis.x;
                t.rotate.entry[i][1] = axis.y;
                t.rotate.entry[i][2] = axis.z;
            }
        }

        // Every object of a 3D subtree, with its bound (the renderer culls by it).
        void moveTree(RE::NiAVObject* node, const model::Pose& move, const int depth)
        {
            if (!node || depth > 64) {
                return;
            }
            moveTransform(node->world, move);
            node->worldBound.center = physics::toGame(move.toWorld(physics::toMeters(node->worldBound.center)));
            if (auto* asNode = node->IsNode()) {
                for (const auto& child : asNode->children) {
                    moveTree(child.get(), move, depth + 1);
                }
            }
        }

        bool validTree(const f4vr::BSFlattenedBoneTree* tree)
        {
            return tree && tree->transforms && tree->numTransforms > 0 && tree->numTransforms <= 1024;
        }

        // The flattened bone array the skin is drawn from: all of it, or a set of bones.
        void moveBones(f4vr::BSFlattenedBoneTree* tree, const model::Pose& move)
        {
            if (!validTree(tree)) {
                return;
            }
            for (int i = 0; i < tree->numTransforms; ++i) {
                moveTransform(tree->transforms[i].world, move);
            }
        }

        void moveBoneSet(f4vr::BSFlattenedBoneTree* tree, const std::vector<int>& bones, const model::Pose& move)
        {
            for (const int i : bones) {
                moveTransform(tree->transforms[i].world, move);
            }
        }

        model::Vec3 bonePosition(const f4vr::BSFlattenedBoneTree* tree, const int i)
        {
            return physics::toMeters(tree->transforms[i].world.translate);
        }

        int findBone(const f4vr::BSFlattenedBoneTree* tree, const char* name)
        {
            for (int i = 0; i < tree->numTransforms; ++i) {
                const char* n = tree->transforms[i].name.c_str();
                if (n && _stricmp(n, name) == 0) {
                    return i;
                }
            }
            return -1;
        }

        // The bone and every bone under it in the array (each bone knows its parent's index).
        std::vector<int> bonesBelow(const f4vr::BSFlattenedBoneTree* tree, const int top)
        {
            std::vector<int> out;
            if (top < 0) {
                return out;
            }
            for (int i = 0; i < tree->numTransforms; ++i) {
                int k = i;
                for (int steps = 0; k >= 0 && k < tree->numTransforms && steps < 64; ++steps) {
                    if (k == top) {
                        out.push_back(i);
                        break;
                    }
                    k = tree->transforms[k].parPos;
                }
            }
            return out;
        }
    }

    void DriverSeat::keepWeaponHolstered()
    {
        // [game] The driver does not shoot: the weapon goes away with the game's own holstering (as the player does it), so it shows neither in the
        // hands nor at the chase camera. If something draws it during the drive, it is put away again (at most twice a second). The shooting itself
        // is already blocked: the seat keeps every controller input from the game while driving.
        if (!g_config.holsterWeaponWhileDriving) {
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player || !player->GetWeaponMagicDrawn()) {
            return;
        }
        const auto now = common::nowMillis();
        if (now - _lastHolsterMs < 500) {
            return;
        }
        _lastHolsterMs = now;
        player->DrawWeaponMagicHands(false);
        logger::info("Seat: weapon put away for the drive");
    }

    void DriverSeat::weaponCheckAtFrameBegin()
    {
        // [game] Did FRIK or the engine put the weapon back after placeBodyInSeat moved it (the weapon showed at the chase camera in the 0.1.75 test)?
        if (!_weaponCheckPending) {
            return;
        }
        _weaponCheckPending = false;
        if (const auto* weapon = f4vr::getWeaponNode()) {
            const auto now = weapon->world.translate;
            const auto d = now - _weaponAfterMove;
            logger::info("Seat: weapon node at FrameBegin ({:.0f}, {:.0f}, {:.0f}), {:.0f} game units from where the seat left it", now.x, now.y, now.z,
                std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z));
        }
    }

    void DriverSeat::placeBodyInSeat(const Vehicle& vehicle)
    {
        // [game] FRIK's AfterWorldFinal: every bone of this frame is final, and the body hangs under the head at the chase camera.
        //  1. One rigid move T = seatEye * inverse(head) takes the head frame FRIK built (the head, the body root's heading) to the seat's eye point
        //     with the drawn vehicle's full rotation: the body faces the car's front, leans, climbs and rolls with it.
        //  2. The standing legs are bent into a seated pose (sThirdPersonLegs): each thigh about its hip, then each shin about its knee.
        //  3. The first-person weapon node is hidden while the body sits (bThirdPersonHideWeapon): it is not part of the body that moves.
        // Both skeletons move, with the third person bone array the skin is drawn from and every object's bound. Nothing is parented; the next frame
        // FRIK builds the body from scratch.
        if (!_driving || !_bodyMoveValid) {
            return;
        }
        F4CF_PERF_SCOPE("body in seat");
        const auto started = std::chrono::steady_clock::now();
        auto* player = RE::PlayerCharacter::GetSingleton();
        const auto* nodes = f4vr::getVRPlayerNodes();
        auto* root = f4vr::getRootNode();
        if (!player || !nodes || !nodes->hmdNode || !root) {
            return;
        }
        const model::Pose drawn = vehicle.drawnPose();
        const model::Pose seatEye{ drawn.toWorld(eyeLocal(vehicle)), drawn.rot };
        const float bodyHeading = headingOfNode(root);
        const model::Pose move = model::seatBodyTransform(seatEye, physics::toMeters(nodes->hmdNode->world.translate), bodyHeading);

        auto* tree3rd = f4vr::getFlattenedBoneTree();
        auto* tree1st = f4vr::getFirstPersonBoneTree();
        moveTree(root, move, 0);
        moveBones(tree3rd, move);
        if (auto* firstPerson = f4vr::getFirstPersonSkeleton()) {
            moveTree(firstPerson, move, 0);
            moveBones(tree1st, move);
        }

        // Seated legs.
        const auto& legsMode = g_config.thirdPersonLegs;
        if (legsMode != "none") {
            const auto bendLegs = [&](f4vr::BSFlattenedBoneTree* tree, SkeletonLegs& cache) {
                if (!validTree(tree)) {
                    return;
                }
                if (cache.tree != tree || cache.count != tree->numTransforms) {
                    cache = {};
                    cache.tree = tree;
                    cache.count = tree->numTransforms;
                    const char* names[2][3] = { { "LLeg_Thigh", "LLeg_Calf", "LLeg_Foot" }, { "RLeg_Thigh", "RLeg_Calf", "RLeg_Foot" } };
                    for (int side = 0; side < 2; ++side) {
                        auto& leg = cache.legs[side];
                        leg.thigh = findBone(tree, names[side][0]);
                        leg.calf = findBone(tree, names[side][1]);
                        leg.foot = findBone(tree, names[side][2]);
                        leg.belowThigh = bonesBelow(tree, leg.thigh);
                        leg.belowCalf = bonesBelow(tree, leg.calf);
                    }
                    logger::info("Seat: leg bones in a skeleton of {} bones: left {}/{}/{}, right {}/{}/{}", cache.count, cache.legs[0].thigh, cache.legs[0].calf,
                        cache.legs[0].foot, cache.legs[1].thigh, cache.legs[1].calf, cache.legs[1].foot);
                }
                for (int side = 0; side < 2; ++side) {
                    const auto& leg = cache.legs[side];
                    if (leg.thigh < 0 || leg.calf < 0 || leg.foot < 0) {
                        continue;
                    }
                    const auto* thighNode = tree->transforms[leg.thigh].refNode;
                    const auto* calfNode = tree->transforms[leg.calf].refNode;
                    if (legsMode == "hide") {
                        // Fallback: the legs collapsed into the hips.
                        for (const int i : leg.belowThigh) {
                            tree->transforms[i].world.scale = 0.0001f;
                        }
                        continue;
                    }
                    const float sideSign = side == 0 ? -1.0f : 1.0f;
                    const model::Vec3 hip = bonePosition(tree, leg.thigh);
                    const model::Vec3 thighTo = drawn.rot.rotate(model::seatedThighLocal(g_config.seatThighAngleDeg, g_config.seatKneeSpreadDeg, sideSign));
                    const model::Pose bendHip = model::rotateAbout(hip, model::rotationBetween(bonePosition(tree, leg.calf) - hip, thighTo));
                    moveBoneSet(tree, leg.belowThigh, bendHip);
                    if (thighNode) {
                        moveTree(const_cast<RE::NiNode*>(thighNode), bendHip, 0);
                    }
                    const model::Vec3 knee = bonePosition(tree, leg.calf);
                    const model::Vec3 shinTo = drawn.rot.rotate(model::seatedShinLocal(g_config.seatThighAngleDeg, g_config.seatKneeAngleDeg, g_config.seatKneeSpreadDeg, sideSign));
                    const model::Pose bendKnee = model::rotateAbout(knee, model::rotationBetween(bonePosition(tree, leg.foot) - knee, shinTo));
                    moveBoneSet(tree, leg.belowCalf, bendKnee);
                    if (calfNode) {
                        moveTree(const_cast<RE::NiNode*>(calfNode), bendKnee, 0);
                    }
                }
            };
            bendLegs(tree3rd, _legs3rd);
            bendLegs(tree1st, _legs1st);
        }

        // The weapon node: where it is, for the FrameBegin check (the weapon itself is holstered for the drive, keepWeaponHolstered).
        auto* weapon = f4vr::getWeaponNode();
        const auto nowMs = common::nowMillis();
        if (g_config.logPlayerBody && weapon && nowMs - _lastWeaponLogMs > 2000) {
            _lastWeaponLogMs = nowMs;
            const auto w = weapon->world.translate;
            const auto* child = weapon->IsNode() && !weapon->IsNode()->children.empty() ? weapon->IsNode()->children[0].get() : nullptr;
            const auto* weapon3rd = f4vr::findNode(root, "WEAPON");
            const int handBone = validTree(tree3rd) ? findBone(tree3rd, "RArm_Hand") : -1;
            const auto* hand = handBone >= 0 ? &tree3rd->transforms[handBone] : nullptr;
            const auto head = physics::toGame(seatEye.pos);
            logger::info("Seat: weapon node ({:.0f}, {:.0f}, {:.0f}) parent '{}', first child '{}'; 3rd person WEAPON {}; right hand {}; seat eye ({:.0f}, {:.0f}, {:.0f})", w.x, w.y, w.z,
                weapon->parent ? weapon->parent->name.c_str() : "-", child ? child->name.c_str() : "-",
                weapon3rd ? std::format("({:.0f}, {:.0f}, {:.0f})", weapon3rd->world.translate.x, weapon3rd->world.translate.y, weapon3rd->world.translate.z) : std::string("none"),
                hand ? std::format("({:.0f}, {:.0f}, {:.0f})", hand->world.translate.x, hand->world.translate.y, hand->world.translate.z) : std::string("none"), head.x, head.y, head.z);
            _weaponAfterMove = w;
            _weaponCheckPending = true;
        }

        const float tookMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started).count();
        logger::sample(5000, "Seat: body moved into the seat from the chase camera ({:.1f} m), legs '{}', took {:.3f} ms", model::length(seatEye.pos - physics::toMeters(nodes->hmdNode->world.translate)),
            legsMode, tookMs);
    }

    void DriverSeat::toggleThirdPerson()
    {
        // The camera then slides between the seat and the chase spot over fCameraSwitchSeconds (updateCamera).
        _thirdPerson = !_thirdPerson;
        if (_thirdPerson) {
            logger::info("Camera: chase camera, collision {} (margin {:.2f} m, at least {:.1f} m)", g_config.cameraCollision ? "on" : "off", g_config.cameraCollisionMargin,
                g_config.cameraMinDistance);
        }
        _filterReady = false;
        _hasLastPlaced = false;
        _hasPrevRaw = false;
        _rawHistCount = 0;
        _orbitYaw = 0.0f;
        _orbitIdle = 0.0f;
        f4vr::showNotification(_thirdPerson ? "Third person camera" : "First person camera");
    }

    model::Vec3 DriverSeat::chaseCameraEye(const Vehicle& vehicle, const model::Vec3& seatEye, const float heading, const float dt)
    {
        // [game] Behind and above the seat. A ray from a pivot just above the seat toward that spot finds walls, rocks and the ground (the car,
        // the player and actors are skipped); the camera stops short of the hit. One ray per frame, game thread only.
        const model::Vec3 wanted = seatEye + rotH(model::Vec3{ 0.0f, -g_config.thirdPersonDistance, 0.0f }, heading) + model::Vec3{ 0.0f, 0.0f, g_config.thirdPersonHeight };
        const model::Vec3 pivot = seatEye + model::Vec3{ 0.0f, 0.0f, 0.5f };
        const model::Vec3 arm = wanted - pivot;
        const float length = model::length(arm);
        if (!g_config.cameraCollision || length < 0.01f) {
            _cameraArmReady = false;
            return wanted;
        }
        const model::Vec3 dir = arm * (1.0f / length);
        float hitDistance = -1.0f;
        if (auto* bhkWorld = physics::getBhkWorld(vehicle.ref()); bhkWorld && length > model::CAMERA_RAY_START) {
            const auto layer = static_cast<std::uint32_t>(RE::COL_LAYER::kCharController);
            const auto hit = physics::castRay(bhkWorld, physics::toGame(pivot + dir * model::CAMERA_RAY_START), physics::toGame(wanted), layer, vehicle.bodyId(),
                model::cameraIgnoreMask(g_config.wheelRayIgnoreMask), g_config.releasePickWorldReference);
            if (hit.hit) {
                hitDistance = model::length(physics::toMeters(hit.point) - pivot);
                logger::sample(2000, "Camera: something between the camera and the car: layer '{}', body {}, {:.1f} m from the pivot; arm {:.1f} -> {:.1f} m",
                    f4vr::getCollisionLayerName(hit.filterInfo & 0x7Fu), hit.bodyId, hitDistance, length,
                    model::cameraArmTarget(length, hitDistance, g_config.cameraCollisionMargin, g_config.cameraMinDistance));
            }
        } else {
            logger::sample(5000, "Camera: no Havok world for the vehicle, the chase camera cannot test for walls");
        }
        const float target = model::cameraArmTarget(length, hitDistance, g_config.cameraCollisionMargin, g_config.cameraMinDistance);
        if (!_cameraArmReady) {
            _cameraArm = target;
            _cameraArmReady = true;
        } else {
            _cameraArm = model::smoothCameraArm(_cameraArm, target, dt, 0.05f, 0.6f);
        }
        return pivot + dir * _cameraArm;
    }

    void DriverSeat::flushPendingRow(CsvLog* cameraLog)
    {
        if (!_beginPending) {
            return;
        }
        _beginPending = false;
        if (cameraLog && !_pendingRow.empty()) {
            // FRIK's FrameEnd did not complete this row (no FRIK, or it did not arrive): the columns it fills are unknown.
            cameraLog->add(_pendingRow + ",nan,nan,nan,nan,nan,nan");
        }
        _pendingRow.clear();
    }

    void DriverSeat::updateCamera(Vehicle& vehicle, CsvLog* cameraLog, const bool placeModel)
    {
        F4CF_PERF_SCOPE("seat camera");
        flushPendingRow(cameraLog);
        if (!_driving || !vehicle.isValid()) {
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        RoomState room;
        if (!player || !readRoom(room)) {
            if (placeModel) {
                vehicle.placeVisualAt(vehicle.poseForSeat());
            }
            return;
        }
        ++_frame;
        keepWeaponHolstered();
        render_probe::tick(g_config.renderProbe);
        render_probe::noteFrameBegin(_frame);
        const float dt = elapsedSeconds();
        const bool hadLastPlaced = _hasLastPlaced;
        const model::Pose rawPose = vehicle.poseForSeat(); // the model's own pose this frame
        const auto pose = vehiclePose(vehicle);
        model::Vec3 seatEye = pose.toWorld(eyeLocal(vehicle));

        // Getting in: the eyes slide from where the head stood into the seat (smoothstep), instead of jumping there.
        if (_glideActive) {
            const float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - _glideStart).count();
            const float s = model::smoothstep(0.0f, g_config.enterGlideSeconds, elapsed);
            if (s >= 1.0f || g_config.enterGlideSeconds <= 0.0f) {
                _glideActive = false;
            } else {
                const model::Vec3 seatLocal = eyeLocal(vehicle);
                seatEye = pose.toWorld(_glideFromLocal + (seatLocal - _glideFromLocal) * s);
            }
        }

        // Smoothness diagnostics: how far the seat really moved this frame against what the speed predicts.
        float seatStepCm = 0.0f;
        float seatDzCm = 0.0f;
        if (_hasLastSeatRaw) {
            const model::Vec3 delta = seatEye - _lastSeatRaw;
            seatStepCm = model::length(delta) * 100.0f;
            seatDzCm = delta.z * 100.0f;
        }
        _lastSeatRaw = seatEye;
        _hasLastSeatRaw = true;
        const float expectedStepCm = std::abs(vehicle.speed()) * dt * 100.0f;
        const model::Vec3 fwd = pose.rot.axisY();
        const float pitchDeg = std::asin(std::clamp(fwd.z, -1.0f, 1.0f)) * 180.0f / model::PI;
        const float rollDeg = std::asin(std::clamp(pose.rot.axisX().z, -1.0f, 1.0f)) * 180.0f / model::PI;

        // Does the carrier node still follow the Havok body? In one test session it stopped (the seat read it and stood still while the car drove
        // 16 m away). Logged either way, since the seat now reads the body.
        const model::Pose bodyPose = vehicle.poseFromBody();
        const model::Pose nodePose = vehicle.poseFromNode();
        const model::Vec3 nodeOff = nodePose.pos - bodyPose.pos;
        const float nodeBodyCm = model::length(nodeOff) * 100.0f;
        const float nodeLeadCm = model::dot(nodeOff, bodyPose.rot.axisY()) * 100.0f;
        if (nodeBodyCm > 30.0f && std::abs(vehicle.speed()) > 2.0f) {
            logger::sample(3000, "Seat: the carrier node is {:.0f} cm from the Havok body at {:.0f} km/h (the node is not following the body; seat source {})", nodeBodyCm, vehicle.speed() * 3.6f,
                g_config.seatPoseSource == SeatPoseSource::HavokBody ? "Havok body" : "node");
        }

        // Measured before moving anything: how far the head is from where the previous frame's placement said it should be (the placement is the
        // only thing the engine can get wrong; the vehicle's own motion since then is not an error).
        const model::Vec3 lean = rotH((room.hmdLocal - _hmdLocal0) * room.scale, room.heading);
        const float headSeatRaw = model::length(room.hmdWorld - seatEye);
        const float anchorError = model::length(room.hmdWorld - (_hasLastPlaced ? _lastPlacedEye : seatEye) - lean);
        // The same error along the vehicle's own axes (of the pose it was placed with): is the head sliding forward/back or up/down in the cabin?
        const model::Vec3 anchorDelta = room.hmdWorld - (_hasLastPlaced ? _lastPlacedEye : seatEye) - lean;
        const model::Pose anchorFrame{ model::Vec3{}, _hasLastPlaced ? _lastPlacedRot : pose.rot };
        const float anchorFwdCm = model::dot(anchorDelta, anchorFrame.rot.axisY()) * 100.0f;
        const float anchorSideCm = model::dot(anchorDelta, anchorFrame.rot.axisX()) * 100.0f;
        const float anchorUpCm = model::dot(anchorDelta, anchorFrame.rot.axisZ()) * 100.0f;
        const float roomYawError = model::wrapAngle(room.heading - _lastTargetRoomHeading);

        // Comfort: filter small vertical shakes of the seat, never further than the configured limit.
        float filterOffset = 0.0f;
        if (g_config.verticalFilterHz > 0.0f && _thirdPerson) {
            if (!_filterReady) {
                _filteredSeatZ = seatEye.z;
                _filterReady = true;
            }
            const float alpha = 1.0f - std::exp(-2.0f * model::PI * g_config.verticalFilterHz * dt);
            _filteredSeatZ += (seatEye.z - _filteredSeatZ) * alpha;
            _filteredSeatZ = std::clamp(_filteredSeatZ, seatEye.z - g_config.verticalFilterMax, seatEye.z + g_config.verticalFilterMax);
            filterOffset = _filteredSeatZ - seatEye.z;
            seatEye.z = _filteredSeatZ;
        }

        if (_thirdPerson) {
            // Right stick left/right: orbit around the car. Released, the camera swings back behind the car.
            const float dz = 0.15f;
            const float sx = std::abs(_stickX) > dz ? _stickX : 0.0f;
            if (sx != 0.0f) {
                _orbitIdle = 0.0f;
                const float direction = g_config.thirdPersonOrbitInvert ? -1.0f : 1.0f;
                _orbitYaw = model::wrapAngle(_orbitYaw + direction * sx * g_config.thirdPersonOrbitSpeed * (model::PI / 180.0f) * dt);
            } else {
                _orbitIdle += dt;
                if (g_config.thirdPersonRecenterDelay > 0.0f && _orbitIdle > g_config.thirdPersonRecenterDelay) {
                    _orbitYaw -= _orbitYaw * (1.0f - std::exp(-3.0f * dt));
                }
            }
        }
        // Chase camera: same room handling, but the head sits behind and above the vehicle (level with the horizon). Switching view slides the eyes
        // between the seat and that spot (smoothstep over fCameraSwitchSeconds), both ways.
        const float switchSeconds = g_config.cameraSwitchSeconds;
        const float viewTarget = _thirdPerson ? 1.0f : 0.0f;
        if (switchSeconds <= 0.0f) {
            _viewBlend = viewTarget;
        } else if (_viewBlend < viewTarget) {
            _viewBlend = (std::min)(viewTarget, _viewBlend + dt / switchSeconds);
        } else {
            _viewBlend = (std::max)(viewTarget, _viewBlend - dt / switchSeconds);
        }
        if (_viewBlend > 0.0f) {
            const float h = model::headingOf(pose.rot.axisY()) + _orbitYaw;
            const model::Vec3 chase = chaseCameraEye(vehicle, seatEye, h, dt);
            seatEye = seatEye + (chase - seatEye) * model::smoothstep(0.0f, 1.0f, _viewBlend);
        } else {
            _cameraArmReady = false;
        }
        _bodyMoveValid = g_config.thirdPersonBodyInSeat && _viewBlend > 0.0f;

        // Heading only: the room follows the vehicle's yaw; pitch and roll are left out (level horizon).
        const float carHeading = model::headingOf(pose.rot.axisY());
        const float targetRoomHeading = model::wrapAngle(carHeading + _headingOffset + (_thirdPerson ? _orbitYaw : 0.0f));
        applyRoomYaw(targetRoomHeading, room);

        const model::Vec3 roomOrigin = seatEye - rotH(_hmdLocal0 * room.scale, targetRoomHeading);
        const model::Vec3 actorTarget = roomOrigin + rotH(room.actorLocal * room.scale, targetRoomHeading);
        {
            static int dumpFrame = 0;
            ++dumpFrame;
            if (dumpFrame == 120 || dumpFrame == 121 || dumpFrame == 300 || dumpFrame == 301) {
                dumpControllerFloats(dumpFrame < 200 ? "a" : "b");
            }
        }
        float actorDriftCm = 0.0f;
        float actorDzCm = 0.0f;
        if (_hasLastActor) {
            const auto drift = room.actorWorld - _lastActorTarget;
            actorDriftCm = model::length(drift) * 100.0f;
            actorDzCm = drift.z * 100.0f;
        }
        _lastActorTarget = actorTarget;
        _hasLastActor = true;

        // The engine moves the camera only on its next update. Where it has the head right now is where the previous frame's placement put it, plus whatever its own
        // character controller moved since (anchorDelta). The model is drawn at that previous pose of the vehicle, shifted by the same, so cabin and eyes stay one rigid
        // thing. Drawn at this frame's pose instead, the cabin runs one frame of driving ahead of the eyes: 30 cm at 60 km/h, and varying with every frame time.
        model::Pose drawPose = rawPose;
        bool locked = false;
        if (placeModel) {
            const int lag = g_config.modelLagFrames;
            const bool lockOn = g_config.lockModelToCamera && lag > 0 && !_lockSuspended && g_config.cameraUpdatePoint == CameraUpdatePoint::FrikFrameBegin;
            if (lockOn && hadLastPlaced && _rawHistCount >= lag && model::length(anchorDelta) < 0.6f) {
                const auto& past = _rawHist[static_cast<std::size_t>(lag - 1)];
                drawPose.pos = past.pos + anchorDelta;
                drawPose.rot = past.rot;
                locked = true;
            }
            const bool damp = !_thirdPerson && g_config.cabinTiltScale < 0.999f;
            vehicle.placeVisualAt(damp ? attenuateTilt(drawPose, eyeLocal(vehicle), g_config.cabinTiltScale) : drawPose);
        }
        for (std::size_t i = _rawHist.size() - 1; i > 0; --i) {
            _rawHist[i] = _rawHist[i - 1];
        }
        _rawHist[0] = rawPose;
        _rawHistCount = std::min<int>(_rawHistCount + 1, static_cast<int>(_rawHist.size()));
        _prevRawPose = rawPose;
        _hasPrevRaw = true;
        const model::Vec3 drawLag = drawPose.pos - rawPose.pos;
        const float drawLagCm = model::length(drawLag) * 100.0f;
        const float drawLagFwdCm = model::dot(drawLag, rawPose.rot.axisY()) * 100.0f;
        const float askFwdCm = model::dot(seatEye - (hadLastPlaced ? _lastPlacedEye : seatEye), pose.rot.axisY()) * 100.0f;

        const model::Vec3 camBegin = renderCameraEye();
        player->SetPosition(physics::toGame(actorTarget), true);
        zeroPlayerVelocity();
        // The capsule is easiest to find once the actor sits in the seat; tried for the first frames only.
        if (!_playerBodyChanged && _playerBodyTries > 0 && _playerBodyTries < 90) {
            setPlayerBodyLayer(true, &vehicle);
        }
        _lastPlacedEye = seatEye;
        _lastPlacedRot = pose.rot;
        _hasLastPlaced = true;
        // The eyes land on the seat plus the player's own lean, turned with the room: that is where the HUD goes in front of.
        HudState hud = vehicle.hudState();
        if (hud.healthShown) {
            health::fillPlayerVitals(hud);
        }
        _hud.update(true, _thirdPerson, pose, seatEye + lean, model::wrapAngle(targetRoomHeading + room.hmdLocalHeading), hud);

        // What FRIK's FrameEnd needs to tell whether the head node moved within this frame.
        _beginHmd = room.hmdWorld;
        _beginRot = pose.rot;
        _beginAskFwdCm = askFwdCm;
        _beginLockEligible = placeModel && hadLastPlaced && g_config.cameraUpdatePoint == CameraUpdatePoint::FrikFrameBegin;
        _beginPending = true;
        _pendingRow.clear();

        if (cameraLog) {
            _pendingRow = std::format("{},{:.5f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.1f},{:.1f},{:.1f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{},{:.1f},{:.1f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.3f},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                _frame,
                dt,
                vehicle.speed() * 3.6f,
                headSeatRaw * 100.0f,
                anchorError * 100.0f,
                roomYawError * 180.0f / model::PI,
                filterOffset * 100.0f,
                _yawSign,
                room.hmdHeading * 180.0f / model::PI,
                model::wrapAngle(targetRoomHeading - _roomHeading0) * 180.0f / model::PI,
                f4vr::PlayerRotation::getYaw() * 180.0f / model::PI,
                seatStepCm,
                expectedStepCm,
                seatDzCm,
                pitchDeg,
                rollDeg,
                _thirdPerson ? 1 : 0,
                nodeBodyCm,
                nodeLeadCm,
                actorDriftCm,
                actorDzCm,
                vehicle.visualErrCm(),
                vehicle.visualErrDzCm(),
                anchorFwdCm,
                anchorSideCm,
                anchorUpCm,
                wallMillis(),
                locked ? 1 : 0,
                drawLagCm,
                drawLagFwdCm,
                vehicle.visualPrevWorldErrCm(),
                askFwdCm,
                camBegin.x, camBegin.y, camBegin.z,
                room.hmdWorld.x, room.hmdWorld.y, room.hmdWorld.z,
                seatEye.x, seatEye.y, seatEye.z);
        }
        logger::sample(2000,
            "Seat: {:.0f} km/h, head-seat anchor error {:.1f} cm (raw {:.1f} cm), room yaw error {:.1f} deg",
            vehicle.speed() * 3.6f,
            anchorError * 100.0f,
            headSeatRaw * 100.0f,
            roomYawError * 180.0f / model::PI);
    }

    void DriverSeat::onFrameEnd(CsvLog* cameraLog)
    {
        render_probe::noteFrameEnd(_frame);
        if (!_beginPending) {
            return;
        }
        _beginPending = false;
        float fwdCm = std::nanf("");
        float sideCm = fwdCm;
        float upCm = fwdCm;
        const auto* nodes = f4vr::getVRPlayerNodes();
        if (nodes && nodes->hmdNode) {
            // Along the vehicle's axes: how far the engine's head node moved between our placement and the end of FRIK's frame.
            const model::Vec3 moved = physics::toMeters(nodes->hmdNode->world.translate) - _beginHmd;
            fwdCm = model::dot(moved, _beginRot.axisY()) * 100.0f;
            sideCm = model::dot(moved, _beginRot.axisX()) * 100.0f;
            upCm = model::dot(moved, _beginRot.axisZ()) * 100.0f;
            if (_beginLockEligible && !_followDecided && std::abs(_beginAskFwdCm) > 3.0f) {
                _followMovedCm += fwdCm * (_beginAskFwdCm > 0.0f ? 1.0f : -1.0f);
                _followAskedCm += std::abs(_beginAskFwdCm);
                if (++_followSamples >= 30) {
                    _followDecided = true;
                    const float ratio = _followMovedCm / _followAskedCm;
                    if (ratio > 0.6f) {
                        _lockSuspended = true;
                        logger::warn("Seat: the engine moves the head node within the frame the player is placed in ({:.0f} % of the movement asked, {} frames): the model is not held back to the camera",
                            ratio * 100.0f,
                            _followSamples);
                    } else if (ratio < 0.3f) {
                        logger::info("Seat: the engine moves the head node only on its next update ({:.0f} % of the movement asked within the frame, {} frames): the model is drawn where the camera really is",
                            ratio * 100.0f,
                            _followSamples);
                    } else {
                        logger::warn("Seat: the head node followed {:.0f} % of the movement asked within the frame ({} frames): unclear, the model stays held back to the camera", ratio * 100.0f, _followSamples);
                    }
                }
            }
        }
        if (cameraLog && !_pendingRow.empty()) {
            const model::Vec3 camEnd = renderCameraEye();
            cameraLog->add(std::format("{},{:.2f},{:.2f},{:.2f},{:.4f},{:.4f},{:.4f}", _pendingRow, fwdCm, sideCm, upCm, camEnd.x, camEnd.y, camEnd.z));
        }
        _pendingRow.clear();
    }

    void DriverSeat::drawDebug(const Vehicle& vehicle) const
    {
        auto& dd = debug::dd();
        auto channel = dd.channelScope("seat");
        const auto pose = vehiclePose(vehicle);
        dd.sphere(physics::toGame(pose.toWorld(eyeLocal(vehicle))), 4.0f, false, _driving ? debug::colors::Green : debug::colors::Cyan);
        dd.sphere(physics::toGame(pose.toWorld(vehicle.spec().exitPoint)), 6.0f, false, debug::colors::Magenta);
        dd.arrow(physics::toGame(pose.pos), physics::toNi(pose.rot.axisY()), 60.0f, debug::colors::White);
    }
}
