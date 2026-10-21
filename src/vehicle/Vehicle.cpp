#include "vehicle/Vehicle.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <format>
#include <functional>
#include <map>

#include "Config.h"
#include "common/MatrixUtils.h"
#include "debug/DebugDraw.h"
#include "f4vr/CollisionLayers.h"
#include "f4vr/F4VRUtils.h"
#include "perf/Perf.h"
#include "player/RenderProbe.h"
#include "telemetry/CsvLog.h"

namespace vfvr
{
    namespace
    {
        // Wheel columns of the telemetry CSV (a two-wheeler pads the unused ones with zeros).
        constexpr std::size_t CSV_WHEELS = 4;
        // Righting a vehicle on its roof: spread over this many physics steps, with this much more roll and lift in all than the single kick.
        constexpr int RIGHTING_STEPS = 12;
        constexpr float UPSIDE_DOWN_SPIN = 2.2f;
        constexpr float UPSIDE_DOWN_LIFT = 1.6f;

        // Held by the step thunk for the whole callback and by the vehicle while it detaches, so a vehicle is
        // never destroyed under a running physics step.
        std::mutex g_stepGate;
        Vehicle* g_stepTarget = nullptr;
        std::atomic<std::uint64_t> g_stepNanos{ 0 };

        void stepThunk(void*, const float deltaSeconds)
        {
            const auto start = std::chrono::steady_clock::now();
            {
                std::scoped_lock gate(g_stepGate);
                if (g_stepTarget) {
                    g_stepTarget->onPhysicsStep(deltaSeconds);
                }
            }
            g_stepNanos.fetch_add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count()),
                std::memory_order_relaxed);
        }

        RE::NiPoint3 gameVec(const model::Vec3& metersVec)
        {
            return physics::toGame(metersVec);
        }

        model::Quat modelRotation(const std::string& forward)
        {
            // Rotation about Z that takes the model's forward axis onto +Y.
            if (forward == "+x") {
                return model::Quat::fromAxisAngle({ 0, 0, 1 }, model::PI * 0.5f);
            }
            if (forward == "-x") {
                return model::Quat::fromAxisAngle({ 0, 0, 1 }, -model::PI * 0.5f);
            }
            if (forward == "-y") {
                return model::Quat::fromAxisAngle({ 0, 0, 1 }, model::PI);
            }
            return {};
        }

        void rotatedBounds(const model::Vec3& lo, const model::Vec3& hi, const model::Quat& rot, model::Vec3& outLo, model::Vec3& outHi)
        {
            outLo = { 1e9f, 1e9f, 1e9f };
            outHi = { -1e9f, -1e9f, -1e9f };
            for (int i = 0; i < 8; ++i) {
                const auto p = rot.rotate({ (i & 1) ? hi.x : lo.x, (i & 2) ? hi.y : lo.y, (i & 4) ? hi.z : lo.z });
                outLo = { (std::min)(outLo.x, p.x), (std::min)(outLo.y, p.y), (std::min)(outLo.z, p.z) };
                outHi = { (std::max)(outHi.x, p.x), (std::max)(outHi.y, p.y), (std::max)(outHi.z, p.z) };
            }
        }

        // Engine rotation matrix for a world pose: rows are the local axes in world (rotations are stored transposed).
        RE::NiTransform toNiTransform(const model::Pose& pose)
        {
            RE::NiTransform t;
            const auto x = pose.rot.axisX();
            const auto y = pose.rot.axisY();
            const auto z = pose.rot.axisZ();
            t.rotate.entry[0][0] = x.x;
            t.rotate.entry[0][1] = x.y;
            t.rotate.entry[0][2] = x.z;
            t.rotate.entry[1][0] = y.x;
            t.rotate.entry[1][1] = y.y;
            t.rotate.entry[1][2] = y.z;
            t.rotate.entry[2][0] = z.x;
            t.rotate.entry[2][1] = z.y;
            t.rotate.entry[2][2] = z.z;
            t.translate = physics::toGame(pose.pos);
            t.scale = 1.0f;
            return t;
        }

        // The engine keeps a node's previous-frame transform (the temporal upscaler builds its motion vectors from it) only for nodes it updates itself;
        // the model is moved by hand, so its previous transform is brought up to date here, before the next move.
        void copyPreviousWorld(RE::NiAVObject* node, const int depth)
        {
            if (!node || depth > 24) {
                return;
            }
            node->previousWorld = node->world;
            if (const auto ni = node->IsNode()) {
                for (const auto& child : ni->children) {
                    copyPreviousWorld(child.get(), depth + 1);
                }
            }
        }

        std::string poseDifference(const model::Pose& a, const model::Pose& b)
        {
            const float dist = model::length(a.pos - b.pos) * 100.0f;
            const float cosAngle = std::clamp(model::dot(a.rot.axisY(), b.rot.axisY()), -1.0f, 1.0f);
            const float cosUp = std::clamp(model::dot(a.rot.axisZ(), b.rot.axisZ()), -1.0f, 1.0f);
            return std::format("{:.1f} cm, forward {:.1f} deg, up {:.1f} deg", dist, std::acos(cosAngle) * 180.0f / model::PI, std::acos(cosUp) * 180.0f / model::PI);
        }
    }

    namespace
    {
        // Menus that pause the game, and with it the physics: no step callback while one is open is not a fault.
        bool gamePausedByMenu()
        {
            static constexpr const char* PAUSING_MENUS[] = { "MessageBoxMenu", "ContainerMenu", "PipboyMenu", "PauseMenu", "LoadingMenu", "BarterMenu",
                "LevelUpMenu", "LooksMenu", "ExamineMenu", "CookingMenu", "TerminalMenu", "FavoritesMenu", "BookMenu", "SleepWaitMenu", "Console" };
            const auto* ui = RE::UI::GetSingleton();
            if (!ui) {
                return false;
            }
            for (const auto* name : PAUSING_MENUS) {
                if (ui->GetMenuOpen(name)) {
                    return true;
                }
            }
            return false;
        }

        void scaleTorque(model::VehicleSpec& spec)
        {
            for (auto& point : spec.engine.torqueCurve) {
                point.second *= g_config.torqueScale;
            }
        }
    }

    std::unique_ptr<Vehicle> Vehicle::create(RE::TESObjectREFR* ref, RE::TESObjectREFR* visual, model::VehicleSpec spec, std::string& error, bool& retryable)
    {
        retryable = true;
        if (!ref) {
            error = "no reference";
            retryable = false;
            return nullptr;
        }
        auto* root = ref->Get3D();
        if (!root) {
            error = "the reference has no 3D loaded";
            return nullptr;
        }

        if (visual && !visual->Get3D()) {
            error = "the vehicle model has no 3D loaded";
            return nullptr;
        }

        std::unique_ptr<Vehicle> v(new Vehicle());
        v->_handle = ref->GetHandle();
        v->_root = root;
        v->_modelRot = modelRotation(spec.modelForward);
        if (!visual) {
            // The reference's own model carries the body: the vehicle frame is the body frame turned by the model axis rotation.
            v->_frameFix = v->_modelRot.conjugate();
            v->_modelIsRoot = true;
        }

        std::string diag;
        auto body = physics::findBody(root, diag);
        if (!body) {
            error = diag;
            return nullptr;
        }
        v->_body = *body;
        v->_body.bhkWorld = physics::getBhkWorld(ref);
        if (physics::getHknpWorld(v->_body.bhkWorld) != v->_body.world) {
            logger::warn("Vehicle: the body's world differs from the cell's bhkWorld+0x60 world ({} vs {})",
                static_cast<const void*>(v->_body.world),
                static_cast<const void*>(physics::getHknpWorld(v->_body.bhkWorld)));
        }
        const auto bodyDiag = std::format("{}; shape type {}, motion type {}", diag, physics::bodyShapeType(v->_body), physics::bodyMotionPreset(v->_body));

        // Model bounds in the vehicle frame (turned by modelForward).
        auto* boundsRef = visual ? visual : ref;
        model::Vec3 boundsMin, boundsMax;
        rotatedBounds(physics::toMeters(boundsRef->GetBoundMin()), physics::toMeters(boundsRef->GetBoundMax()), v->_modelRot, boundsMin, boundsMax);

        // Box chassis for meshes that cannot be dynamic (and always on a carrier). With an explicit hull in the profile that is the box
        // (vehicle frame, turned into the body's own frame); otherwise the model bounds, with the bottom raised so the box sits above the
        // wheels' contact patches instead of dragging on the ground. The body's own frame is the carrier's (= vehicle frame) or, without a
        // carrier, the model's.
        physics::HullShape hullShape;
        hullShape.rampLength = spec.hull.rampLength;
        hullShape.rampHeight = spec.hull.rampHeight;
        hullShape.sideInset = spec.hull.sideInset;
        model::Vec3 boxMin;
        model::Vec3 boxMax;
        if (spec.hull.explicitBox) {
            if (visual) {
                boxMin = spec.hull.min;
                boxMax = spec.hull.max;
            } else {
                rotatedBounds(spec.hull.min, spec.hull.max, v->_modelRot.conjugate(), boxMin, boxMax);
                hullShape.lengthAxis = (spec.modelForward == "+x" || spec.modelForward == "-x") ? 0 : 1;
            }
        } else {
            boxMin = visual ? boundsMin : physics::toMeters(boundsRef->GetBoundMin());
            boxMax = visual ? boundsMax : physics::toMeters(boundsRef->GetBoundMax());
            boxMin.z += spec.hullClearance;
            hullShape.lengthAxis = (!visual && (spec.modelForward == "+x" || spec.modelForward == "-x")) ? 0 : 1;
        }
        if (!physics::makeDynamic(v->_body, spec.mass, boxMin, boxMax, visual != nullptr || spec.boxHull, hullShape, diag)) {
            error = bodyDiag + "; " + diag;
            retryable = false;
            return nullptr;
        }
        logger::info("Vehicle: {}", bodyDiag);
        logger::info("Vehicle: {}", diag);
        // Parked, the body keeps the layer the engine gave it, so the player and everything else collide with it. It moves to
        // sVehicleBodyLayer only while somebody drives (applyBodyLayer).
        v->_parkedBodyLayer = physics::getBodyLayer(v->_body);
        logger::info("Vehicle: body collision layer '{}' while parked, '{}' while driven", f4vr::getCollisionLayerName(static_cast<std::uint32_t>(v->_parkedBodyLayer)),
            g_config.vehicleBodyLayer.empty() ? "unchanged" : g_config.vehicleBodyLayer);

        // The visual model: its static collision is switched off and it is moved onto the carrier every frame;
        // the carrier's own model is hidden.
        if (visual) {
            v->_visualHandle = visual->GetHandle();
            v->_visualRoot = visual->Get3D();
            std::string visualDiag;
            if (const auto visualBody = physics::findBody(v->_visualRoot, visualDiag)) {
                physics::setBodyLayer(*visualBody, RE::COL_LAYER::kNonCollidable);
                logger::info("Vehicle: model collision ({}) set to nonCollidable, simulated: {}", visualDiag, physics::bodyIsDynamic(*visualBody));
                if (physics::bodyIsDynamic(*visualBody)) {
                    // A simulated model body would fall away and the engine would keep dragging the model node after it, against our placement.
                    auto keyed = *visualBody;
                    const bool ok = physics::makeKeyframed(keyed);
                    logger::info("Vehicle: model body made keyframed: {} (motion type now {})", ok ? "yes" : "NO", physics::bodyMotionPreset(keyed));
                }
            }
            f4vr::setNodeVisibility(root, false);
            v->syncVisual();
            logger::info("Vehicle: model carried by a dynamic carrier, model forward '{}'", spec.modelForward);
        }

        // Wheels from the object bounds when the profile has none.
        v->_boundsMin = boundsMin;
        v->_boundsMax = boundsMax;
        const bool generated = spec.ensureWheels(boundsMin, boundsMax);
        const auto gMin = gameVec(boundsMin);
        const auto gMax = gameVec(boundsMax);
        logger::info("Vehicle: bounds in the vehicle frame (game units) min ({:.0f}, {:.0f}, {:.0f}) max ({:.0f}, {:.0f}, {:.0f})", gMin.x, gMin.y, gMin.z, gMax.x, gMax.y, gMax.z);
        for (const auto& w : spec.wheels) {
            const auto hub = gameVec(w.hub);
            logger::info("Vehicle: wheel layout {} hub [{:.0f}, {:.0f}, {:.0f}] steer={} drive={} handbrake={}{}",
                w.name,
                hub.x,
                hub.y,
                hub.z,
                w.steer,
                w.drive,
                w.handbrake,
                generated ? " (auto)" : "");
        }

        // Where does the Havok body frame sit relative to the node we draw? The seat can use either.
        logger::info("Vehicle: node vs Havok body frame: {}", poseDifference(v->poseFromNode(), v->poseFromBody()));

        const auto layer = f4vr::parseCollisionLayer(g_config.wheelRayLayer);
        v->_rayFilter = static_cast<std::uint32_t>(layer.value_or(RE::COL_LAYER::kCharController));
        logger::info("Vehicle: wheel rays on layer '{}'", f4vr::getCollisionLayerName(v->_rayFilter));

        scaleTorque(spec);
        v->_spec = spec;
        v->_health = model::VehicleHealth(spec.damage);
        v->_radShield = model::RadShield(spec.radiation);
        v->_model.init(spec);
        v->_simpleMode = g_config.simpleMode;
        v->_model.setSimpleMode(v->_simpleMode);
        v->_valid = true;

        {
            std::scoped_lock gate(g_stepGate);
            g_stepTarget = v.get();
        }
        physics::StepListener::get().setCallback(&stepThunk, nullptr);
        v->_lastStepCount = physics::StepListener::get().stepCount();
        return v;
    }

    std::uint64_t Vehicle::consumeStepNanos()
    {
        return g_stepNanos.exchange(0, std::memory_order_relaxed);
    }

    Vehicle::~Vehicle()
    {
        std::scoped_lock gate(g_stepGate);
        if (g_stepTarget == this) {
            g_stepTarget = nullptr;
        }
    }

    RE::TESObjectREFR* Vehicle::ref() const
    {
        return _handle.get().get();
    }

    model::Pose Vehicle::poseFromNode() const
    {
        if (!_root) {
            return {};
        }
        auto pose = physics::nodePoseMeters(_root);
        pose.rot = pose.rot * _frameFix;
        return pose;
    }

    void Vehicle::updateRighting(const model::Pose& frame)
    {
        // [game] Lying on the side or the roof (up.z < 0.35) and nearly still for fRightAfterSeconds: ask the next step to roll it back.
        const auto now = std::chrono::steady_clock::now();
        const float dt = _flipClock.time_since_epoch().count() == 0 ? 0.0f : std::clamp(std::chrono::duration<float>(now - _flipClock).count(), 0.0f, 0.1f);
        _flipClock = now;
        _rightCooldown = (std::max)(0.0f, _rightCooldown - dt);
        if (g_config.rightAfterSeconds <= 0.0f) {
            _flippedTime = 0.0f;
            return;
        }
        const bool flipped = frame.rot.axisZ().z < 0.35f && std::abs(speed()) < 4.0f;
        _flippedTime = flipped ? _flippedTime + dt : 0.0f;
        if (_flippedTime >= g_config.rightAfterSeconds && _rightCooldown <= 0.0f) {
            _flippedTime = 0.0f;
            _rightCooldown = 2.5f; // if it is still flipped afterwards, try again
            _rightPending = true;
            logger::info("Vehicle: on its side/roof for {:.0f} s, rolling it back onto its wheels", g_config.rightAfterSeconds);
        }
    }

    void Vehicle::checkFallThrough(const model::Pose& frame, const float riseMps)
    {
        // [game] Falling fast with no wheel on the ground: is there ground above the car? A ray from 40 m above it down to it meets the top of the
        // terrain if the car is under it. Then put it back where it last rested on its wheels (the next step does it), if it fell more than
        // fFallRecoverMeters below that.
        _fallCheckCooldown = (std::max)(0.0f, _fallCheckCooldown - 1.0f / 90.0f);
        if (riseMps > -8.0f || _lastTelemetry.groundedWheels > 0 || _fallCheckCooldown > 0.0f) {
            return;
        }
        _fallCheckCooldown = 0.5f;
        const model::Vec3 above = frame.pos + model::Vec3{ 0.0f, 0.0f, 40.0f };
        const auto hit = physics::castRay(_body.bhkWorld, gameVec(above), gameVec(frame.pos), _rayFilter, _body.bodyId.value, g_config.wheelRayIgnoreMask,
            g_config.releasePickWorldReference);
        if (!hit.hit) {
            return; // nothing above: a jump or a cliff, not a fall through the world
        }
        float drop = 0.0f;
        bool known = false;
        {
            std::scoped_lock lock(_mutex);
            if (_hasSupported) {
                drop = _lastSupportedBody.pos.z - frame.pos.z;
                known = true;
            }
        }
        const auto p = physics::toGame(frame.pos);
        const float groundAbove = model::length(physics::toMeters(hit.point) - frame.pos);
        const auto layer = static_cast<std::uint32_t>(physics::getBodyLayer(_body));
        logger::warn("Vehicle: falling through the world at {:.1f} m/s, ground {:.1f} m above it (layer '{}', body {}), game position ({:.0f}, {:.0f}, {:.0f}), up.z {:.2f}, "
                     "body layer '{}', {:.1f} m below where it last rested on its wheels",
            -riseMps, groundAbove, f4vr::getCollisionLayerName(hit.filterInfo & 0x7Fu), hit.bodyId, p.x, p.y, p.z, frame.rot.axisZ().z, f4vr::getCollisionLayerName(layer),
            known ? drop : 0.0f);
        if (known && drop > g_config.fallRecoverMeters) {
            _recoverPending = true;
        }
    }

    void Vehicle::applyRighting(const physics::BodyStateMeters& state, const float spin, const float lift)
    {
        // [step] A couple of equal and opposite vertical impulses rolls the car about the horizontal axis that takes its up vector
        // to the world up vector; a small lift on top clears the wheels off the ground. The body is never teleported.
        auto frame = state.frame;
        frame.rot = frame.rot * _frameFix;
        const model::Vec3 worldUp{ 0.0f, 0.0f, 1.0f };
        const model::Vec3 carUp = frame.rot.axisZ();
        model::Vec3 axis = model::cross(carUp, worldUp);
        float axisLen = model::length(axis);
        if (axisLen < 0.05f) {
            axis = frame.rot.axisY(); // upside down: roll about the car's own long axis
            axisLen = 1.0f;
        }
        axis = axis * (1.0f / axisLen);
        const float angle = std::acos(std::clamp(carUp.z, -1.0f, 1.0f));
        const model::Vec3 e = model::cross(axis, worldUp);
        const model::Vec3 torqueDir = model::cross(e, worldUp);
        const float sign = model::dot(torqueDir, axis) >= 0.0f ? 1.0f : -1.0f;

        const float mass = _spec.mass;
        const float inertia = _spec.lean.enabled ? _spec.rollInertia() : mass * 1.5f; // rough roll inertia, kg m^2 (a motorcycle's is a fraction of a car's)
        const float omega = (std::min)(angle, model::PI) / 0.8f; // finish the roll in about 0.8 s
        const float arm = 1.0f;
        const float j = inertia * omega * spin / (2.0f * arm);
        const bool viaCollisionObject = g_config.impulseApi == ImpulseApi::CollisionObject;
        const model::Vec3 com = state.com;
        physics::applyImpulseAt(_body, worldUp * (sign * j), com + e * arm, viaCollisionObject);
        physics::applyImpulseAt(_body, worldUp * (-sign * j), com - e * arm, viaCollisionObject);
        physics::applyImpulseAt(_body, worldUp * (mass * 2.5f * lift), com, viaCollisionObject);
    }

    void Vehicle::pushSample(const double t, const model::Pose& pose)
    {
        if (!_samples.empty()) {
            const auto& last = _samples.back();
            if (t <= last.t) {
                return;
            }
            if (model::length(pose.pos - last.pose.pos) > 5.0f) {
                _samples.clear(); // teleport (respawn, load): never blend across it
                _clockValid = false;
            }
        }
        _samples.push_back({ t, pose });
        if (_samples.size() > 48) {
            _samples.erase(_samples.begin(), _samples.begin() + 16);
        }
    }

    model::Pose Vehicle::renderPose()
    {
        const auto now = poseFromBody();
        if (!g_config.renderInterpolation) {
            return now;
        }
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        std::scoped_lock lock(_renderMutex);
        pushSample(_simClock, now);
        if (_samples.size() < 2) {
            return now;
        }
        const double sample = _simClock - wall;
        if (!_clockValid) {
            _clockOffset = sample;
            _clockValid = true;
        } else {
            const double elapsed = std::clamp(wall - _lastWall, 0.0, 0.25);
            _clockOffset += (sample - _clockOffset) * (1.0 - std::exp(-elapsed / 0.5));
        }
        _lastWall = wall;

        const double target = std::clamp(wall + _clockOffset - static_cast<double>(g_config.renderDelayMs) * 0.001, _samples.front().t, _samples.back().t);
        std::size_t hi = 1;
        while (hi + 1 < _samples.size() && _samples[hi].t < target) {
            ++hi;
        }
        const auto& a = _samples[hi - 1];
        const auto& b = _samples[hi];
        const double span = b.t - a.t;
        const float k = span > 1e-6 ? static_cast<float>(std::clamp((target - a.t) / span, 0.0, 1.0)) : 1.0f;
        model::Pose out;
        out.pos = a.pose.pos + (b.pose.pos - a.pose.pos) * k;
        model::Quat qb = b.pose.rot;
        const model::Quat& qa = a.pose.rot;
        if (qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w < 0.0f) {
            qb = { -qb.x, -qb.y, -qb.z, -qb.w };
        }
        model::Quat q{ qa.x + (qb.x - qa.x) * k, qa.y + (qb.y - qa.y) * k, qa.z + (qb.z - qa.z) * k, qa.w + (qb.w - qa.w) * k };
        const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        out.rot = n > 1e-6f ? model::Quat{ q.x / n, q.y / n, q.z / n, q.w / n } : b.pose.rot;
        // Samples older than the target are no longer needed (keep one before it).
        if (hi > 2) {
            _samples.erase(_samples.begin(), _samples.begin() + static_cast<std::ptrdiff_t>(hi - 2));
        }
        return out;
    }

    model::Pose Vehicle::poseForSeat() const
    {
        // The pose the visual model was last drawn at: the seat camera must use the very same one, otherwise the
        // cabin jitters against the eyes whenever the physics thread steps between the two reads.
        return _hasSyncedPose ? _syncedPose : poseFromBody();
    }

    model::Pose Vehicle::poseFromBody() const
    {
        const auto state = physics::readBodyState(_body, false);
        if (!state.valid) {
            return poseFromNode();
        }
        auto pose = state.frame;
        pose.rot = pose.rot * _frameFix;
        return pose;
    }

    model::VehicleTelemetry Vehicle::telemetry() const
    {
        std::scoped_lock lock(_mutex);
        return _lastTelemetry;
    }

    float Vehicle::speed() const
    {
        std::scoped_lock lock(_mutex);
        return _lastTelemetry.speed;
    }

    HudState Vehicle::hudState() const
    {
        HudState state;
        state.speedKmh = std::abs(speed()) * 3.6f;
        state.healthShown = g_config.vehicleHealth;
        state.vehicleHealth = _health.share();
        state.vehicleDamaged = _health.condition() == model::VehicleCondition::Damaged;
        state.vehicleDisabled = _health.disabled();
        state.hasShield = _radShield.spec().resistance > 0.0f && _radShield.capacity() > 0.0f;
        state.shield = _radShield.share();
        state.shieldActive = _radShield.active();
        return state;
    }

    void Vehicle::setSimpleMode(const bool simple)
    {
        std::scoped_lock lock(_mutex);
        if (_simpleMode != simple) {
            _simpleMode = simple;
            _model.setSimpleMode(simple);
            logger::info("Vehicle: simple diagnostic physics {}", simple ? "ON" : "OFF");
        }
    }

    bool Vehicle::validate()
    {
        auto* ref = this->ref();
        if (!ref || ref->IsDeleted() || ref->Get3D() != _root || !_root) {
            return false;
        }
        if (!_body.node || _body.node->collisionObject.get() != _body.collisionObject) {
            return false;
        }
        if (physics::getBhkWorld(ref) != _body.bhkWorld) {
            return false;
        }
        if (_visualRoot) {
            auto* visual = _visualHandle.get().get();
            if (!visual || visual->IsDeleted()) {
                logger::info("Vehicle: validate failed: model reference is gone");
                return false;
            }
            if (visual->Get3D() != _visualRoot) {
                // The engine rebuilt (or unloaded) the model's 3D, e.g. after a cell change. Follow it instead of giving up.
                auto* now = visual->Get3D();
                logger::info("Vehicle: model 3D changed {} -> {}", static_cast<const void*>(_visualRoot), static_cast<const void*>(now));
                if (!now) {
                    if (++_visualMissing > 180) {
                        logger::info("Vehicle: validate failed: model 3D missing for 2 s");
                        return false;
                    }
                } else {
                    _visualRoot = now;
                    _visualMissing = 0;
                }
            } else {
                _visualMissing = 0;
            }
        }
        return physics::bodyIsDynamic(_body);
    }

    bool Vehicle::visualCellMismatch() const
    {
        const auto* visual = _visualHandle.get().get();
        const auto* carrier = ref();
        return _visualRoot && visual && carrier && visual->GetParentCell() != carrier->GetParentCell();
    }

    void Vehicle::scanWheelNodes()
    {
        auto previous = std::move(_wheelNodes); // the shapes of a set that is not fitted are detached: keep hold of them across rescans
        _wheelNodes.clear();
        _steerNodes.clear();
        // With a separate model (carrier) the wheels are part of the model, not of the carrier reference.
        auto* const scanRoot = _visualRoot ? _visualRoot : _root;
        _wheelScanRoot = scanRoot;
        if (!scanRoot || _spec.wheels.empty()) {
            return;
        }
        const auto frame = poseFromBody();
        std::string tree;
        std::function<void(RE::NiAVObject*, int)> walk = [&](RE::NiAVObject* node, const int depth) {
            if (!node || depth > 6) {
                return;
            }
            std::string lower = node->name.c_str() ? node->name.c_str() : "";
            for (auto& c : lower) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            const auto local = frame.toLocal(physics::toMeters(node->world.translate));
            tree += std::format("\n  {}{} ({:.2f}, {:.2f}, {:.2f}) m", std::string(static_cast<std::size_t>(depth) * 2, ' '), node->name.c_str() ? node->name.c_str() : "?", local.x, local.y, local.z);
            if (!node->IsNode()) {
                // A shape (leaf): log where its geometry sits in the car frame, to tell separate wheels from the bodywork.
                const auto c = frame.toLocal(physics::toMeters(node->worldBound.center));
                tree += std::format("  [shape bound centre ({:.2f}, {:.2f}, {:.2f}) m, radius {:.2f} m]", c.x, c.y, c.z, node->worldBound.fRadius / 70.0f);
            }
            if (depth > 0 && lower.starts_with("steer")) {
                SteerNode s;
                s.node.reset(node);
                s.rest = node->local.rotate;
                s.restPos = node->local.translate;
                _steerNodes.push_back(std::move(s));
                tree += "  -> steering assembly";
                return; // everything below it turns with it
            }
            const bool isWheel = depth > 0 && (lower.contains("wheel") || lower.contains("tire") || lower.contains("tyre") || lower.contains("rim"));
            if (isWheel) {
                // Closest spec wheel by horizontal position.
                std::size_t best = 0;
                float bestDist = 1e9f;
                for (std::size_t i = 0; i < _spec.wheels.size(); ++i) {
                    const auto d = _spec.wheels[i].hub - local;
                    const float dist = std::sqrt(d.x * d.x + d.y * d.y);
                    if (dist < bestDist) {
                        bestDist = dist;
                        best = i;
                    }
                }
                if (bestDist < 0.6f) {
                    WheelNode w;
                    w.node.reset(node);
                    w.rest = node->local.rotate;
                    w.restPos = node->local.translate;
                    w.wheel = best;
                    for (auto& old : previous) {
                        if (old.node.get() == node) {
                            w.shapes = old.shapes;
                        }
                    }
                    _wheelNodes.push_back(std::move(w));
                    tree += std::format("  -> wheel {} ({:.2f} m)", best, bestDist);
                    return; // do not animate children of a wheel node a second time
                }
            }
            if (auto* asNode = node->IsNode()) {
                for (const auto& child : asNode->children) {
                    walk(child.get(), depth + 1);
                }
            }
        };
        walk(scanRoot, 0);
        logger::info("Vehicle: wheel nodes found: {}, steering assembly nodes: {}. Node tree:{}", _wheelNodes.size(), _steerNodes.size(), tree);
    }

    void Vehicle::logGeometryState(const char* why)
    {
        // [game] Diagnostic: what the engine holds for every shape of the model (visibility, bounds, triangle counts and the words that
        // follow them, where a BSMeshLODTriShape keeps its LOD sizes), and the state of each wheel node.
        if (!_root) {
            return;
        }
        const auto frame = poseFromBody();
        float distance = -1.0f;
        if (const auto* player = RE::PlayerCharacter::GetSingleton()) {
            const auto car = physics::toGame(frame.pos);
            const float dx = player->data.location.x - car.x;
            const float dy = player->data.location.y - car.y;
            const float dz = player->data.location.z - car.z;
            distance = std::sqrt(dx * dx + dy * dy + dz * dz) / 70.0f;
        }
        std::string out = std::format("Vehicle geometry [{}], player {:.1f} m from the car:", why, distance);
        // How many shapes use each shader property: the engine builds a property's render passes for one geometry, so a shared one
        // draws only one of its shapes.
        std::map<const void*, int> propertyUsers;
        std::function<void(RE::NiAVObject*, int)> countUsers = [&](RE::NiAVObject* node, const int depth) {
            if (!node || depth > 6) {
                return;
            }
            if (auto* tri = node->IsTriShape()) {
                ++propertyUsers[tri->GetRuntimeData().properties[0].get()];
            }
            if (auto* asNode = node->IsNode()) {
                for (const auto& child : asNode->children) {
                    countUsers(child.get(), depth + 1);
                }
            }
        };
        countUsers(_visualRoot ? _visualRoot : _root, 0);
        std::function<void(RE::NiAVObject*, int)> walk = [&](RE::NiAVObject* node, const int depth) {
            if (!node || depth > 6) {
                return;
            }
            const char* name = node->name.c_str() ? node->name.c_str() : "?";
            if (auto* tri = node->IsTriShape()) {
                const auto c = frame.toLocal(physics::toMeters(node->worldBound.center));
                const auto& rt = tri->GetTriShapeRuntimeData();
                std::uint32_t words[6]{};
                std::memcpy(words, reinterpret_cast<const std::uint8_t*>(tri) + 0x1A8, sizeof(words));
                const auto& geo = tri->GetRuntimeData();
                out += std::format("\n  {} culled={} flags={:#x} bound ({:.2f}, {:.2f}, {:.2f}) r {:.2f} m tris {} verts {} after [{} {} {} {} {} {}] shader {} (used by {}) renderer data {} registered {} type {}",
                    name,
                    node->GetAppCulled(),
                    node->GetFlags(),
                    c.x,
                    c.y,
                    c.z,
                    node->worldBound.fRadius / 70.0f,
                    rt.numTriangles,
                    rt.numVertices,
                    words[0],
                    words[1],
                    words[2],
                    words[3],
                    words[4],
                    words[5],
                    static_cast<const void*>(geo.properties[0].get()),
                    propertyUsers[geo.properties[0].get()],
                    geo.rendererData ? "yes" : "NO",
                    geo.registered,
                    geo.type);
            }
            if (auto* asNode = node->IsNode()) {
                for (const auto& child : asNode->children) {
                    walk(child.get(), depth + 1);
                }
            }
        };
        walk(_visualRoot ? _visualRoot : _root, 0);
        for (const auto& w : _wheelNodes) {
            if (w.node) {
                const auto local = frame.toLocal(physics::toMeters(w.node->world.translate));
                const auto& r = w.node->local.rotate.entry;
                const float det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
                out += std::format("\n  wheel node {} culled={} rotation det {:.3f} local ({:.1f}, {:.1f}, {:.1f}) scale {:.2f} world in car frame ({:.2f}, {:.2f}, {:.2f}) m",
                    w.node->name.c_str() ? w.node->name.c_str() : "?",
                    w.node->GetAppCulled(),
                    det,
                    w.node->local.translate.x,
                    w.node->local.translate.y,
                    w.node->local.translate.z,
                    w.node->local.scale,
                    local.x,
                    local.y,
                    local.z);
            }
        }
        logger::info("{}", out);
    }

    void Vehicle::applyWheelSetVisibility()
    {
        // [game] Every wheel node holds one shape per wheel set, named Wheel_<corner>_<set>. The shapes of the chosen set are attached and visible; the
        // others are detached from the node (the hidden flag alone did not take effect on screen). A model without shapes for the set (an older
        // mesh) is left alone: removing every wheel would be worse than showing the wrong ones.
        const std::string suffix = "_" + _spec.wheelSet;
        for (auto& w : _wheelNodes) {
            auto* node = w.node ? w.node->IsNode() : nullptr;
            if (!node) {
                continue;
            }
            if (w.shapes.empty()) {
                for (const auto& child : node->children) {
                    const std::string name = child && child->name.c_str() ? child->name.c_str() : "";
                    if (name.find("Wheel_") == 0) {
                        w.shapes.emplace_back(child.get());
                    }
                }
            }
            bool any = false;
            for (const auto& shape : w.shapes) {
                const std::string name = shape->name.c_str() ? shape->name.c_str() : "";
                any = any || name.ends_with(suffix);
            }
            if (!any) {
                if (!_wheelSetMissingLogged) {
                    _wheelSetMissingLogged = true;
                    logger::warn("Vehicle: the model has no '{}' wheel shapes (looked for names ending in '{}'); the wheels are left as they are", _spec.wheelSet, suffix);
                }
                continue;
            }
            for (auto& shape : w.shapes) {
                const std::string name = shape->name.c_str() ? shape->name.c_str() : "";
                const bool want = name.ends_with(suffix);
                const bool attached = shape->parent == node;
                f4vr::setNodeVisibility(shape.get(), want);
                if (want && !attached) {
                    node->AttachChild(shape.get(), true);
                } else if (!want && attached) {
                    RE::NiPointer<RE::NiAVObject> out;
                    node->DetachChild(shape.get(), out);
                }
            }
        }
        _wheelSetShown = _spec.wheelSet;
        logger::info("Vehicle: wheel set '{}' shown", _spec.wheelSet);
    }

    void Vehicle::applySpec(model::VehicleSpec spec)
    {
        // [game] The hull is not rebuilt (it is the same body); everything the model computes from the spec is.
        spec.ensureWheels(_boundsMin, _boundsMax);
        scaleTorque(spec);
        const float oldMass = _spec.mass;
        {
            std::scoped_lock lock(_mutex);
            _spec = std::move(spec);
            // The health and the reserve carry over (another part selection does not repair the car); their limits follow the new spec.
            const float keptHealth = _health.health();
            const float keptReserve = _radShield.remaining();
            _health = model::VehicleHealth(_spec.damage);
            _health.setHealth(keptHealth);
            _radShield = model::RadShield(_spec.radiation);
            _radShield.setRemaining(keptReserve);
            _model.init(_spec);
            _model.setSimpleMode(_simpleMode);
            _lastTelemetry = {};
            _contacts = {};
        }
        if (_spec.mass != oldMass) {
            physics::setBodyMass(_body, _spec.mass);
        }
        physics::activate(_body);
        _settleTimer = 4.0f; // keep the body awake while it settles to the new ride height
        _settleLog = 0.0f;
        _wheelScanRoot = nullptr; // rescan: spin angles restart and the wheel shapes of the new set are shown
        _hasPrevStep = false;
        logger::info("Vehicle: spec changed: tyre radius {:.3f} m, mass {:.0f} kg, wheel set '{}', hubs z {:.3f} m", _spec.tyre.radius, _spec.mass, _spec.wheelSet,
            _spec.wheels.empty() ? 0.0f : _spec.wheels[0].hub.z);
    }

    void Vehicle::animateWheels()
    {
        if (_wheelScanRoot != (_visualRoot ? _visualRoot : _root)) {
            scanWheelNodes();
            _wheelClock = std::chrono::steady_clock::now();
            _geometryDiagTimer = 0.0f;
            _geometryDiagCount = 0;
            logGeometryState("spawn");
            applyWheelSetVisibility();
        }
        if (_wheelNodes.empty()) {
            return;
        }
        if (_wheelSetShown != _spec.wheelSet) {
            applyWheelSetVisibility();
        }
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::clamp(std::chrono::duration<float>(now - _wheelClock).count(), 0.0f, 0.1f);
        _wheelClock = now;
        _geometryDiagTimer += dt;
        if (_geometryDiagTimer >= 3.0f && _geometryDiagCount < 12) {
            _geometryDiagTimer = 0.0f;
            ++_geometryDiagCount;
            logGeometryState("periodic");
        }

        model::VehicleTelemetry tel;
        {
            std::scoped_lock lock(_mutex);
            tel = _lastTelemetry;
        }
        // Extra rotation is built in the vehicle frame (X right, Y forward, Z up) and carried into the model's own frame, which the wheel nodes live in.
        const auto toModel = [&](const model::Quat& q) { return _modelRot.conjugate() * q * _modelRot; };
        // Rotations are stored transposed, so "extra after rest" is rest * extra in matrix order.
        const auto composeRotation = [&](const RE::NiMatrix3& rest, const model::Quat& extraModel) {
            const auto e = toNiTransform(model::Pose{ { 0, 0, 0 }, extraModel }).rotate;
            RE::NiMatrix3 out = rest; // start from a valid matrix: any padding column is carried over
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    out.entry[r][c] = rest.entry[r][0] * e.entry[0][c] + rest.entry[r][1] * e.entry[1][c] + rest.entry[r][2] * e.entry[2][c];
                }
            }
            return out;
        };
        // The steering turn: about the vertical through each steered wheel's own hub, or, with a steering axis in the profile (a motorcycle), about
        // that raked axis through the pivot, which also swings the hub of the front wheel and carries the "Steer..." nodes round with it.
        const bool axisSteer = model::length(_spec.steering.axis) > 0.5f;
        const model::Quat steerVehicle = model::Quat::fromAxisAngle(axisSteer ? _spec.steering.axis : model::Vec3{ 0, 0, 1 }, -tel.steerAngle);
        const model::Quat steerModel = toModel(steerVehicle);
        const model::Vec3 pivotGame = _modelRot.rotateInverse(_spec.steering.pivot) * model::HAVOK_TO_GAME;
        const auto aboutPivot = [&](const model::Vec3& position) { return steerModel.rotate(position - pivotGame) + pivotGame; };
        // A world update moves the transforms, but a node's own bound is only recomputed by a full update pass: left alone it keeps the place and size
        // it had when the model loaded, and the renderer culls the node when that spot is off screen. Keep the bound on the node and its shapes.
        const auto keepBound = [](RE::NiAVObject* object, const float radius) {
            object->worldBound.center = object->world.translate;
            object->worldBound.fRadius = radius;
        };
        for (auto& w : _wheelNodes) {
            if (!w.node || w.wheel >= _spec.wheels.size()) {
                continue;
            }
            w.angle = std::fmod(w.angle + tel.wheels[w.wheel].spin * dt, 2.0f * model::PI);
            auto extra = model::Quat::fromAxisAngle({ 1, 0, 0 }, -w.angle);
            const bool steered = _spec.wheels[w.wheel].steer;
            if (steered) {
                extra = steerVehicle * extra;
            }
            w.node->local.rotate = composeRotation(w.rest, toModel(extra));
            // The wheel mesh goes where the physics wheel is: the hub (centre at full droop) plus the compression. The mesh has it at
            // restPos; the difference is vertical, turned from the vehicle frame into the model's. A tyre never rises above its mesh
            // position by more than wheelMaxAboveMesh, or it would come out through the fender (the arches leave no room).
            const auto restVehicle = _modelRot.rotate({ w.restPos.x, w.restPos.y, w.restPos.z });
            const float wantedZ = (_spec.wheels[w.wheel].hub.z + tel.wheels[w.wheel].compression + _spec.wheelVisualLift) * model::HAVOK_TO_GAME;
            const float lift = (std::min)(wantedZ - restVehicle.z, _spec.wheelMaxAboveMesh * model::HAVOK_TO_GAME);
            const auto up = _modelRot.rotateInverse({ 0.0f, 0.0f, lift });
            model::Vec3 position{ w.restPos.x + up.x, w.restPos.y + up.y, w.restPos.z + up.z };
            if (steered && axisSteer) {
                position = aboutPivot(position);
            }
            w.node->local.translate = { position.x, position.y, position.z };
            f4vr::updateDown(w.node.get(), true);
            // Radius 45 units > the 29 of the tyre.
            keepBound(w.node.get(), 45.0f);
            if (const auto* asNode = w.node->IsNode()) {
                for (const auto& child : asNode->children) {
                    if (child) {
                        keepBound(child.get(), 45.0f);
                    }
                }
            }
        }
        for (auto& s : _steerNodes) {
            if (!s.node) {
                continue;
            }
            s.node->local.rotate = composeRotation(s.rest, steerModel);
            const auto position = aboutPivot({ s.restPos.x, s.restPos.y, s.restPos.z });
            s.node->local.translate = { position.x, position.y, position.z };
            f4vr::updateDown(s.node.get(), true);
            // The assembly's shapes are modelled around the vehicle origin and cover the front half of the bike: a sphere that spans all of it.
            keepBound(s.node.get(), 130.0f);
            if (const auto* asNode = s.node->IsNode()) {
                for (const auto& child : asNode->children) {
                    if (child) {
                        keepBound(child.get(), 130.0f);
                    }
                }
            }
        }
    }

    bool Vehicle::placeLight(RE::TESObjectREFR& light, const std::size_t index) const
    {
        if (index >= _spec.lightPoints.size()) {
            return false;
        }
        // Node writes do not move the engine light, so move the reference itself: location + yaw, then refresh its 3D.
        // The light shines along its local +X; lightYawDeg turns that onto the vehicle's forward.
        const float deg = model::PI / 180.0f;
        const auto pose = drawnPose();
        const float heading = model::headingOf(pose.rot.axisY());
        const auto loc = physics::toGame(pose.toWorld(_spec.lightPoints[index]));
        light.data.location.x = loc.x;
        light.data.location.y = loc.y;
        light.data.location.z = loc.z;
        light.data.angle.x = 0.0f;
        light.data.angle.y = 0.0f;
        light.data.angle.z = heading - _spec.lightYawDeg * deg;
        light.Update3DPosition(true);
        return true;
    }

    RE::ObjectRefHandle Vehicle::swapVisual(const RE::ObjectRefHandle& fresh, RE::NiAVObject* freshRoot)
    {
        const auto old = _visualHandle;
        _visualHandle = fresh;
        _visualRoot = freshRoot;
        _visualMissing = 0;
        // The new model is a plain static with its own collision, sitting right where the car is: switch it off like the first one,
        // or the car crashes into its own copy.
        std::string diag;
        if (const auto body = physics::findBody(freshRoot, diag)) {
            physics::setBodyLayer(*body, RE::COL_LAYER::kNonCollidable);
        } else {
            logger::warn("Vehicle: replacement model has no collision body to switch off ({})", diag);
        }
        return old;
    }

    model::Pose Vehicle::drawnPose() const
    {
        return _hasDrawnPose ? _drawnPose : poseForSeat();
    }

    void Vehicle::beginCameraFrame()
    {
        _cameraSync = std::chrono::steady_clock::now();
        _syncedPose = renderPose();
        _hasSyncedPose = true;
    }

    void Vehicle::syncVisual()
    {
        if (!_visualRoot && !(_modelIsRoot && _root)) {
            return;
        }
        _syncedPose = renderPose();
        _hasSyncedPose = true;
        placeVisualAt(_syncedPose);
    }

    void Vehicle::placeBodyNodeAt(const model::Pose& pose)
    {
        // Runs from FRIK's FrameBegin, which also fires while the plugin's own frame is skipped (loading screens): only write a node the reference still owns.
        auto* carrier = ref();
        if (!carrier || carrier->IsDeleted() || carrier->Get3D() != _root) {
            return;
        }
        // The pose the seat asks for (tilt attenuated, locked to the camera) is drawn by writing the node the engine just synced from the body; the body itself
        // (collision, wheels, physics) is untouched.
        _drawnPose = pose;
        _hasDrawnPose = true;
        const model::Pose modelPose{ pose.pos, pose.rot * _modelRot };
        const auto world = toNiTransform(modelPose);
        _root->local = _root->parent ? common::MatrixUtils::worldToLocalTransform(_root->parent->world, world) : world;
        f4vr::updateDown(_root, true);
        _lastWrittenWorld = _root->world.translate;
        _hasWrittenWorld = true;
        render_probe::noteWritten(_root, _lastWrittenWorld.x, _lastWrittenWorld.y, _lastWrittenWorld.z);
    }

    void Vehicle::placeVisualAt(const model::Pose& pose)
    {
        render_probe::noteCall(_visualRoot != nullptr, _root);
        if (!_visualRoot) {
            if (_modelIsRoot && _root) {
                if (g_config.drawBodyNode) {
                    placeBodyNodeAt(pose);
                } else {
                    _hasDrawnPose = false;
                }
            }
            return;
        }
        {
            auto* visual = _visualHandle.get().get();
            if (!visual || visual->IsDeleted() || visual->Get3D() != _visualRoot) {
                return; // gone or rebuilt: validate() follows it on the next frame
            }
        }
        // Model pose = carrier body pose turned by the model axis rotation, written as the model root's local
        // transform under its (static) parent.
        if (_hasWrittenWorld) {
            const auto& now = _visualRoot->world.translate;
            const float dx = now.x - _lastWrittenWorld.x;
            const float dy = now.y - _lastWrittenWorld.y;
            const float dz = now.z - _lastWrittenWorld.z;
            _visualErrCm = std::sqrt(dx * dx + dy * dy + dz * dz) / 0.7f;
            _visualErrDzCm = dz / 0.7f;
        }
        {
            // Is the engine keeping the previous-frame transform of the model in step? (a stale one would feed the upscaler wrong motion vectors)
            const auto& prev = _visualRoot->previousWorld.translate;
            const auto& cur = _visualRoot->world.translate;
            const float dx = prev.x - cur.x;
            const float dy = prev.y - cur.y;
            const float dz = prev.z - cur.z;
            _prevWorldErrCm = std::sqrt(dx * dx + dy * dy + dz * dz) / 0.7f;
        }
        if (g_config.modelPreviousWorld) {
            copyPreviousWorld(_visualRoot, 0);
        }
        _drawnPose = pose;
        _hasDrawnPose = true;
        const model::Pose modelPose{ pose.pos, pose.rot * _modelRot };
        const auto world = toNiTransform(modelPose);
        _visualRoot->local = _visualRoot->parent ? common::MatrixUtils::worldToLocalTransform(_visualRoot->parent->world, world) : world;
        f4vr::updateDown(_visualRoot, true);
        {
            // The bound of the model's node sphere drifted metres away from the car while driving (log: "bound centre-to-root" up to 12 m for a 4 m radius), and the
            // renderer culls a subtree whose bound is off screen: the car vanished. Keep every inner node's sphere on the node, 85 m wide (the shapes keep their own bounds).
            constexpr float NODE_BOUND_RADIUS = 6000.0f; // the engine fades a node out by distance from its bound radius: 600 units made the car vanish a few metres away
            std::function<void(RE::NiAVObject*, int)> keep = [&](RE::NiAVObject* node, const int depth) {
                auto* asNode = node ? node->IsNode() : nullptr;
                if (!asNode || depth > 6) {
                    return;
                }
                node->worldBound.center = node->world.translate;
                if (node->worldBound.fRadius < NODE_BOUND_RADIUS) {
                    node->worldBound.fRadius = NODE_BOUND_RADIUS;
                }
                for (const auto& child : asNode->children) {
                    keep(child.get(), depth + 1);
                }
            };
            keep(_visualRoot, 0);
        }
        _lastWrittenWorld = _visualRoot->world.translate;
        _hasWrittenWorld = true;
        render_probe::noteWritten(_visualRoot, _lastWrittenWorld.x, _lastWrittenWorld.y, _lastWrittenWorld.z);
        if (auto* visual = _visualHandle.get().get()) {
            visual->data.location.x = world.translate.x;
            visual->data.location.y = world.translate.y;
            visual->data.location.z = world.translate.z;
        }

        // The model's ancestors in the scene graph keep the bound they had when the model was spawned, and the
        // renderer skips a subtree whose parent bound is off screen: the model vanished once the car had driven
        // ~30 m. Only ever grow those bounds (plain data, nothing is recomputed) so they keep covering the model.
        const float modelRadius = _visualRoot->worldBound.fRadius;
        int depth = 0;
        for (auto* p = _visualRoot->parent; p && depth < 12; p = p->parent, ++depth) {
            const float dx = p->worldBound.center.x - world.translate.x;
            const float dy = p->worldBound.center.y - world.translate.y;
            const float dz = p->worldBound.center.z - world.translate.z;
            const float need = std::sqrt(dx * dx + dy * dy + dz * dz) + modelRadius;
            if (need > p->worldBound.fRadius && need < 1.0e6f) {
                p->worldBound.fRadius = need;
            }
        }

        // What the engine says about the model, twice a second (at most 600 lines): is it drawn at all, and where.
        if (++_watchCounter >= 45 && _watchLines < 600) {
            _watchCounter = 0;
            ++_watchLines;
            int shapes = 0;
            int culledShapes = 0;
            std::function<void(RE::NiAVObject*, int)> count = [&](RE::NiAVObject* node, const int depth) {
                if (!node || depth > 6) {
                    return;
                }
                if (node->IsTriShape()) {
                    ++shapes;
                    if (node->GetAppCulled()) {
                        ++culledShapes;
                    }
                }
                if (auto* asNode = node->IsNode()) {
                    for (const auto& child : asNode->children) {
                        count(child.get(), depth + 1);
                    }
                }
            };
            count(_visualRoot, 0);
            auto* visual = _visualHandle.get().get();
            const auto* player = RE::PlayerCharacter::GetSingleton();
            const auto* player3d = player ? player->Get3D() : nullptr;
            const auto& wb = _visualRoot->worldBound;
            const auto& wt = _visualRoot->world.translate;
            const auto body = poseFromBody();
            const auto bodyGame = physics::toGame(body.pos);
            logger::info("Vehicle watch: root culled {} flags 0x{:X}, shapes {} ({} culled), ref {} disabled {} deleted {} 3D-same {}, bound centre-to-root {:.0f} cm r {:.0f}, root-to-body {:.0f} cm, player {:.1f} m, player 3D {} culled {} scale {:.2f}, cell visual {} carrier {}",
                _visualRoot->GetAppCulled(),
                static_cast<std::uint64_t>(_visualRoot->flags.flags),
                shapes,
                culledShapes,
                visual ? "ok" : "NONE",
                visual ? visual->IsDisabled() : false,
                visual ? visual->IsDeleted() : false,
                visual ? visual->Get3D() == _visualRoot : false,
                std::sqrt((wb.center.x - wt.x) * (wb.center.x - wt.x) + (wb.center.y - wt.y) * (wb.center.y - wt.y) + (wb.center.z - wt.z) * (wb.center.z - wt.z)) / 0.7f,
                wb.fRadius / 0.7f,
                std::sqrt((bodyGame.x - wt.x) * (bodyGame.x - wt.x) + (bodyGame.y - wt.y) * (bodyGame.y - wt.y) + (bodyGame.z - wt.z) * (bodyGame.z - wt.z)) / 0.7f,
                player ? std::sqrt((player->data.location.x - wt.x) * (player->data.location.x - wt.x) + (player->data.location.y - wt.y) * (player->data.location.y - wt.y) + (player->data.location.z - wt.z) * (player->data.location.z - wt.z)) / 70.0f : -1.0f,
                player3d ? "ok" : "NONE",
                player3d ? player3d->GetAppCulled() : false,
                player3d ? player3d->local.scale : 0.0f,
                static_cast<const void*>(visual ? visual->GetParentCell() : nullptr),
                static_cast<const void*>(ref() ? ref()->GetParentCell() : nullptr));
        }

        // Evidence for the next log: what the model hangs under, and whether it is hidden.
        _visualDiagTimer += 1.0f / 90.0f;
        if (_visualDiagTimer > 2.0f) {
            _visualDiagTimer = 0.0f;
            const auto* visual = _visualHandle.get().get();
            const auto* carrier = ref();
            std::string chain;
            int d2 = 0;
            for (const auto* p = _visualRoot->parent; p && d2 < 6; p = p->parent, ++d2) {
                chain += std::format(" <- '{}' r={:.0f}", p->GetName(), p->worldBound.fRadius);
            }
            logger::info("Vehicle model: root flags 0x{:X}, radius {:.0f}, cell {} (carrier cell {}),{}",
                static_cast<std::uint64_t>(_visualRoot->flags.flags),
                modelRadius,
                static_cast<const void*>(visual ? visual->GetParentCell() : nullptr),
                static_cast<const void*>(carrier ? carrier->GetParentCell() : nullptr),
                chain);
        }
    }

    void Vehicle::applyBodyLayer(const bool occupied)
    {
        // [game] The hull covers the cabin, so while somebody drives it goes on a layer the ROCK hands and the player's own
        // character controller do not collide with. Parked, it goes back to the engine's layer (or sParkedBodyLayer) and the player bumps into it.
        // Re-checked every frame because the INI is reloaded live.
        auto layer = _parkedBodyLayer;
        // sParkedBodyLayer only while the car rests on its wheels: on its side or in the air only the hull holds it up, and that layer may not meet the
        // terrain or the statics (a Rockboy left on its side on a rock fell through the world, again and again). Then the engine's own layer.
        const auto up = poseFromBody().rot.axisZ();
        const bool supported = _lastTelemetry.groundedWheels >= 2 && up.z > 0.7f;
        _supportedTime = supported ? _supportedTime + 1.0f / 90.0f : 0.0f;
        const bool parkedLayerSafe = _supportedTime > 0.5f;
        const auto& wanted = occupied ? g_config.vehicleBodyLayer : (parkedLayerSafe ? g_config.parkedBodyLayer : std::string());
        if (!wanted.empty()) {
            if (wanted != _layerName) {
                _layerName = wanted;
                _layerParsed = f4vr::parseCollisionLayer(wanted);
                if (!_layerParsed) {
                    logger::warn("Vehicle: unknown body layer '{}', layer left as it was", wanted);
                }
            }
            if (_layerParsed) {
                layer = *_layerParsed;
            }
        }
        if (occupied == _bodyOccupied && _hasAppliedLayer && layer == _appliedLayer) {
            return;
        }
        _bodyOccupied = occupied;
        _appliedLayer = layer;
        _hasAppliedLayer = true;
        const auto before = physics::getFilterInfo(_body);
        physics::setBodyLayer(_body, layer);
        logger::info("Vehicle: body collision layer now '{}' ({}); filter info 0x{:08X} -> 0x{:08X}", f4vr::getCollisionLayerName(static_cast<std::uint32_t>(layer)),
            occupied ? "driven" : "parked", before, physics::getFilterInfo(_body));
    }

    void Vehicle::onFrameUpdate(const model::DriverInput& input, const bool occupied, CsvLog* physicsLog, CsvLog* motoLog)
    {
        F4CF_PERF_SCOPE("vehicle frame");
        if (!_valid) {
            return;
        }
        if (!validate()) {
            logger::info("Vehicle: reference, 3D or body is gone; vehicle released");
            _valid = false;
            std::scoped_lock gate(g_stepGate);
            if (g_stepTarget == this) {
                g_stepTarget = nullptr;
            }
            return;
        }

        applyBodyLayer(occupied);
        // While the seat camera syncs the model itself (right before it places the driver), leave it alone: a second write with a slightly later
        // pose would put the model and the eyes on different poses.
        const bool cameraSyncs = _cameraSync.time_since_epoch().count() != 0 && std::chrono::steady_clock::now() - _cameraSync < std::chrono::milliseconds(25);
        if (!cameraSyncs) {
            syncVisual();
        }
        animateWheels();
        const auto frame = poseFromBody();
        float previousFrameZ = frame.pos.z;
        {
            // Diagnostic: the car rising fast while barely moving along the road means something threw it up. Log where.
            const float rise = (frame.pos.z - _lastFrameZ) * 90.0f;
            const float previousZ = _lastFrameZ;
            previousFrameZ = previousZ != 0.0f ? previousZ : frame.pos.z;
            _lastFrameZ = frame.pos.z;
            _launchLogCooldown = (std::max)(0.0f, _launchLogCooldown - 1.0f / 90.0f);
            if (previousZ != 0.0f && rise > 2.5f && rise < 50.0f && std::abs(speed()) < 3.0f && _launchLogCooldown <= 0.0f) {
                _launchLogCooldown = 3.0f;
                const auto p = physics::toGame(frame.pos);
                const auto f = frame.rot.axisY();
                logger::warn("Vehicle: launched upwards ({:.1f} m/s) at game position ({:.0f}, {:.0f}, {:.0f}), heading {:.0f} deg, forward z {:.2f}, wheels on the ground {}",
                    rise, p.x, p.y, p.z, model::headingOf(f) * 180.0f / model::PI, f.z, _lastTelemetry.groundedWheels);
            }
        }
        updateRighting(frame);
        checkFallThrough(frame, (frame.pos.z - previousFrameZ) * 90.0f);
        castWheelRays(frame);
        probeImpact();
        {
            std::scoped_lock lock(_mutex);
            _input = input;
            if (_health.disabled()) {
                // A disabled vehicle does not drive: no throttle and the handbrake on, whatever the driver does (getting out still works).
                _input.throttle = 0.0f;
                _input.handbrake = true;
            }
            _physicsLog = physicsLog;
            _motoLog = _spec.lean.enabled && g_config.logMotoDiag ? motoLog : nullptr;
            ++_frame;
        }

        if (occupied || std::abs(speed()) > 0.05f) {
            physics::activate(_body);
        }
        if (_settleTimer > 0.0f) {
            _settleTimer -= 1.0f / 90.0f;
            physics::activate(_body);
            _settleLog -= 1.0f / 90.0f;
            if (_settleLog <= 0.0f) {
                _settleLog = 0.5f;
                float comp = 0.0f;
                for (std::size_t i = 0; i < _spec.wheels.size() && i < _lastTelemetry.wheels.size(); ++i) {
                    comp += _lastTelemetry.wheels[i].compression;
                }
                comp /= (std::max)(1.0f, static_cast<float>(_spec.wheels.size()));
                logger::info("Vehicle: settling after a spec change: body z {:.3f} m, mean compression {:.3f} m, speed {:.2f} m/s, wheels on the ground {}",
                    frame.pos.z, comp, speed(), _lastTelemetry.groundedWheels);
            }
        }
        physics::StepListener::get().registerForNextStep(_body.bhkWorld);

        // The listener must be called every frame; if it is not, the vehicle cannot move.
        const auto steps = physics::StepListener::get().stepCount();
        if (steps == _lastStepCount && gamePausedByMenu()) {
            _framesWithoutStep = 0;
        } else if (steps == _lastStepCount) {
            if (++_framesWithoutStep == 90) {
                logger::warn("Vehicle: no physics step callback for 90 frames - the step listener is not being called");
            }
        } else {
            _framesWithoutStep = 0;
        }
        _lastStepCount = steps;

        // Occasional cross-checks of the conventions this code relies on.
        _diagTimer += 1.0f / 90.0f;
        if (logger::isDebugEnabled() && _diagTimer > 2.0f) {
            _diagTimer = 0.0f;
            const auto api = physics::readBodyState(_body, false);
            const auto motion = physics::readBodyState(_body, true);
            logger::debug("Vehicle diag: node vs body {}; angular velocity API ({:.3f}, {:.3f}, {:.3f}) vs motion ({:.3f}, {:.3f}, {:.3f}); steps {}",
                poseDifference(poseFromNode(), frame),
                api.angVel.x,
                api.angVel.y,
                api.angVel.z,
                motion.angVel.x,
                motion.angVel.y,
                motion.angVel.z,
                steps);
        }

        if (g_config.debugDraw) {
            drawDebug(frame);
        }
    }

    void Vehicle::castWheelRays(const model::Pose& frame)
    {
        F4CF_PERF_SCOPE("wheel rays");
        std::array<model::WheelContact, model::MAX_WHEELS> contacts{};
        std::size_t count;
        float rayLength;
        std::array<model::Vec3, model::MAX_WHEELS> starts{};
        {
            std::scoped_lock lock(_mutex);
            count = _spec.wheels.size();
            rayLength = _model.wheelRayLength();
            for (std::size_t i = 0; i < count; ++i) {
                starts[i] = _model.wheelRayStartLocal(i);
            }
        }
        const model::Vec3 down = -frame.rot.axisZ();
        _bottomedLogCooldown = (std::max)(0.0f, _bottomedLogCooldown - 1.0f / 90.0f);
        for (std::size_t i = 0; i < count; ++i) {
            const model::Vec3 start = frame.toWorld(starts[i]);
            const model::Vec3 end = start + down * rayLength;
            const auto hit =
                physics::castRay(_body.bhkWorld, gameVec(start), gameVec(end), _rayFilter, _body.bodyId.value, g_config.wheelRayIgnoreMask, g_config.releasePickWorldReference);
            if (hit.hit) {
                contacts[i].hit = true;
                contacts[i].point = physics::toMeters(hit.point);
                contacts[i].normal = hit.normal;
                contacts[i].grip = 1.0f; // surface materials come with the parts/material phase
                contacts[i].layer = hit.filterInfo & 0x7Fu;
                // Diagnostic: a wheel pushed to the end of its travel (95%) means the ground it sees is far above where it should be.
                if (_lastTelemetry.wheels[i].compression >= 0.95f * _spec.suspension.restLength && _bottomedLogCooldown <= 0.0f) {
                    _bottomedLogCooldown = 1.5f;
                    const auto layer = static_cast<RE::COL_LAYER>(hit.filterInfo & 0x7Fu);
                    const auto hitPoint = physics::toMeters(hit.point);
                    logger::warn("Vehicle: wheel {} bottomed out: ground is layer '{}' ({}), body {}, {:.2f} m below the ray start, normal ({:.2f}, {:.2f}, {:.2f}), car at ({:.0f}, {:.0f}, {:.0f}), up.z {:.2f}",
                        i,
                        f4vr::getCollisionLayerName(hit.filterInfo),
                        static_cast<int>(layer),
                        hit.bodyId,
                        start.z - hitPoint.z,
                        hit.normal.x,
                        hit.normal.y,
                        hit.normal.z,
                        physics::toGame(frame.pos).x,
                        physics::toGame(frame.pos).y,
                        physics::toGame(frame.pos).z,
                        frame.rot.axisZ().z);
                }
            }
        }
        std::scoped_lock lock(_mutex);
        _contacts = contacts;
    }

    void Vehicle::onPhysicsStep(const float deltaSeconds)
    {
        F4CF_PERF_SCOPE("vehicle step");
        if (!_valid) {
            return;
        }
        const auto state = physics::readBodyState(_body, false);
        if (!state.valid) {
            return;
        }
        {
            auto framePose = state.frame;
            framePose.rot = framePose.rot * _frameFix;
            std::scoped_lock renderLock(_renderMutex);
            pushSample(_simClock, framePose);
            _simClock += static_cast<double>(deltaSeconds);
        }
        bool righting = false;
        if (_recoverPending.exchange(false)) {
            model::Pose back;
            {
                std::scoped_lock lock(_mutex);
                back = _lastSupportedBody;
            }
            back.pos.z += 0.5f; // drop it onto its wheels, not into the ground
            physics::placeBodyAt(_body, back);
            logger::warn("Vehicle: put back where it last rested on its wheels, game position ({:.0f}, {:.0f}, {:.0f})", back.pos.x * model::HAVOK_TO_GAME,
                back.pos.y * model::HAVOK_TO_GAME, back.pos.z * model::HAVOK_TO_GAME);
            righting = true;
        } else if (_rightPending.exchange(false)) {
            auto f = state.frame;
            f.rot = f.rot * _frameFix;
            if (f.rot.axisZ().z < -0.8f) {
                // On its roof the single kick only rocked it (a Rockboy took 8 tries, 21 Oct): roll it over across several steps, harder, with more lift.
                _rightingStepsLeft = RIGHTING_STEPS;
                logger::info("Vehicle: upside down, rolling it over across {} physics steps", RIGHTING_STEPS);
            } else {
                applyRighting(state);
                righting = true;
            }
        }
        if (_rightingStepsLeft > 0) {
            --_rightingStepsLeft;
            applyRighting(state, UPSIDE_DOWN_SPIN / RIGHTING_STEPS, UPSIDE_DOWN_LIFT / RIGHTING_STEPS);
            righting = true;
        }

        // Impact detection: since the previous callback the velocity changed by what we applied (tyre forces, drag), gravity and nothing else,
        // unless the hull hit something. Anything left over is that collision.
        {
            model::Vec3 residual{};
            if (_hasPrevStep && !righting && _prevDt > 0.0f && _spec.mass > 0.0f) {
                const model::Vec3 expected = _prevVel + _prevImpulse * (1.0f / _spec.mass) + model::Vec3{ 0.0f, 0.0f, -9.81f * _prevDt };
                residual = state.linVel - expected;
            }
            _impactResidual = model::length(residual);
            _impactResidualZ = residual.z;
            if (_impactResidual >= impactThreshold() && model::isFinite(residual)) {
                std::scoped_lock impactLock(_impactMutex);
                if (!_impact.pending) {
                    auto frame = state.frame;
                    frame.rot = frame.rot * _frameFix;
                    _impact = { true, residual, state.com, frame, _lastTelemetry.speed };
                }
            }
        }

        CsvLog* log;
        CsvLog* motoLog;
        std::size_t wheelCount;
        model::Pose stepFrame;
        std::array<model::WheelContact, model::MAX_WHEELS> stepContacts{};
        {
            std::scoped_lock lock(_mutex);
            auto frame = state.frame;
            frame.rot = frame.rot * _frameFix;
            const model::BodyState body{ frame, state.com, state.linVel, state.angVel };
            wheelCount = _spec.wheels.size();
            _model.step(body, _input, std::span(_contacts.data(), wheelCount), deltaSeconds, _impulses);
            _lastTelemetry = _model.telemetry();
            if (_lastTelemetry.groundedWheels >= 2 && frame.rot.axisZ().z > 0.7f && std::abs(state.linVel.z) < 2.0f) {
                _lastSupportedBody = state.frame;
                _hasSupported = true;
            }
            log = _physicsLog;
            motoLog = _motoLog;
            if (motoLog) {
                stepFrame = frame;
                stepContacts = _contacts;
            }
        }

        const bool viaCollisionObject = g_config.impulseApi == ImpulseApi::CollisionObject;
        model::Vec3 appliedImpulse{};
        for (const auto& impulse : _impulses) {
            physics::applyImpulseAt(_body, impulse.impulse, impulse.point, viaCollisionObject);
            appliedImpulse += impulse.impulse;
        }
        _hasPrevStep = !righting;
        _prevVel = state.linVel;
        _prevImpulse = appliedImpulse;
        _prevDt = deltaSeconds;

        if (log) {
            const auto& t = _lastTelemetry;
            std::string row = std::format("{},{:.5f},{:.3f},{:.0f},{},{:.2f},{:.2f},{:.2f},{},{:.3f},{:.3f},{:.3f}",
                _frame,
                t.dt,
                t.speed * 3.6f,
                t.engineRpm,
                t.gear,
                t.throttle,
                t.brake,
                t.steerAngle * 180.0f / model::PI,
                t.groundedWheels,
                state.angVel.z,
                state.linVel.z,
                state.frame.rot.axisZ().z);
            for (std::size_t i = 0; i < CSV_WHEELS; ++i) {
                if (i >= wheelCount) {
                    row += ",0,0,0,0,0,0,0,0,0,0"; // a two-wheeler: keep the columns aligned with the header
                    continue;
                }
                const auto& w = t.wheels[i];
                row += std::format(",{:d},{:d},{:.3f},{:.2f},{:.3f},{:.2f},{:.0f},{:.0f},{:.0f},{:.2f}",
                    w.grounded,
                    w.holding,
                    w.compression,
                    w.spin,
                    w.slipRatio,
                    w.slipAngle * 180.0f / model::PI,
                    w.fz,
                    w.fx,
                    w.fy,
                    w.gripUsed);
            }
            row += std::format(",{:.2f},{:.2f},{:.2f},{:.2f},{}", _impactResidual, _impactResidualZ, t.lean * 180.0f / model::PI, t.leanTarget * 180.0f / model::PI,
                t.handbrake ? 1 : 0);
            log->add(std::move(row));
        }
        if (motoLog) {
            logMotoStep(motoLog, stepFrame, state, stepContacts, deltaSeconds);
        }
    }

    void Vehicle::logMotoStep(CsvLog* motoLog, const model::Pose& frame, const physics::BodyStateMeters& state, const std::array<model::WheelContact, model::MAX_WHEELS>& contacts,
        const float deltaSeconds)
    {
        // [step] One row per physics step for a two-wheeler on rough ground: the lean and what the stabiliser did about it, the body's turn rates in its
        // own axes, the steering asked and given, and per wheel the ground it found (normal, layer), the suspension and the tyre forces.
        const auto& t = _lastTelemetry;
        constexpr float DEG = 180.0f / model::PI;
        const model::Vec3 right = frame.rot.axisX();
        const model::Vec3 forward = frame.rot.axisY();
        const model::Vec3 up = frame.rot.axisZ();
        const float rollRate = model::dot(state.angVel, forward) * DEG;
        const float pitchRate = model::dot(state.angVel, right) * DEG;
        const float yawRate = model::dot(state.angVel, up) * DEG;
        const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        std::string row = std::format("{},{},{:.5f},{:.2f},{:.2f},{:.2f},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.0f},{},{:.1f},{:.1f},{:.1f},{:.3f},{:.2f},{:.2f},{:.2f},{:.2f},{}",
            _frame,
            now,
            deltaSeconds,
            t.speed * 3.6f,
            t.throttle,
            t.brake,
            t.handbrake ? 1 : 0,
            t.steerInput,
            t.steerAngle * DEG,
            t.lean * DEG,
            t.leanTarget * DEG,
            t.stabiliserTorque,
            t.stabiliserSaturated ? 1 : 0,
            rollRate,
            pitchRate,
            yawRate,
            up.z,
            frame.pos.x,
            frame.pos.y,
            frame.pos.z,
            state.linVel.z,
            t.groundedWheels);
        for (std::size_t i = 0; i < 2; ++i) {
            const auto& w = t.wheels[i];
            const auto& c = contacts[i];
            row += std::format(",{:d},{:.3f},{:.2f},{:.2f},{:.2f},{},{:.3f},{:.2f},{:.0f},{:.0f},{:.0f},{:.2f}",
                w.grounded,
                w.compression,
                c.hit ? c.normal.x : 0.0f,
                c.hit ? c.normal.y : 0.0f,
                c.hit ? c.normal.z : 0.0f,
                c.hit ? static_cast<int>(c.layer) : -1,
                w.slipRatio,
                w.slipAngle * DEG,
                w.fx,
                w.fy,
                w.fz,
                w.gripUsed);
        }
        row += std::format(",{:.2f}", _impactResidual);
        motoLog->add(std::move(row));

        // Events worth a log line, with what the bike was doing.
        const auto where = [&]() {
            const auto p = physics::toGame(frame.pos);
            return std::format("at {:.0f} km/h, lean {:.0f} deg (target {:.0f}), stabiliser {:.0f} N*m{}, steer {:.0f} deg, wheels down {}, game position ({:.0f}, {:.0f}, {:.0f})",
                t.speed * 3.6f, t.lean * DEG, t.leanTarget * DEG, t.stabiliserTorque, t.stabiliserSaturated ? " (saturated)" : "", t.steerAngle * DEG, t.groundedWheels, p.x, p.y,
                p.z);
        };
        _motoFrontAir = t.wheels[0].grounded ? 0.0f : _motoFrontAir + deltaSeconds;
        if (_motoFrontAir > 0.1f && !_motoFrontAirLogged) {
            _motoFrontAirLogged = true;
            logger::warn("Moto: front wheel in the air for {:.2f} s, {}", _motoFrontAir, where());
        } else if (_motoFrontAir == 0.0f && _motoFrontAirLogged) {
            _motoFrontAirLogged = false;
            logger::info("Moto: front wheel back on the ground, {}", where());
        }
        const float lean = std::abs(t.lean * DEG);
        if (lean > 30.0f && !_motoLeanLogged) {
            _motoLeanLogged = true;
            logger::warn("Moto: lean past 30 deg, roll rate {:.0f} deg/s, {}", rollRate, where());
        } else if (lean < 25.0f && _motoLeanLogged) {
            _motoLeanLogged = false;
        }
        _motoYawCooldown = (std::max)(0.0f, _motoYawCooldown - deltaSeconds);
        if (std::abs(yawRate) > 90.0f && _motoYawCooldown <= 0.0f) {
            _motoYawCooldown = 1.0f;
            logger::warn("Moto: yaw rate {:.0f} deg/s, {}", yawRate, where());
        }
    }

    float Vehicle::impactThreshold() const
    {
        const float log = g_config.impactLogMps > 0.0f ? g_config.impactLogMps : 1e9f;
        const float damage = g_config.vehicleHealth ? _spec.damage.collisionMinDeltaV : 1e9f;
        return (std::max)((std::min)(log, damage), 0.5f);
    }

    std::vector<model::Vec3> Vehicle::smokePoints() const
    {
        if (!_spec.damage.smokePoints.empty()) {
            return _spec.damage.smokePoints;
        }
        const float height = _boundsMax.z - _boundsMin.z;
        if (_spec.lean.enabled) {
            return { model::Vec3{ 0.0f, _boundsMax.y * 0.25f, _boundsMin.z + height * 0.45f } };
        }
        return { model::Vec3{ 0.0f, _boundsMax.y * 0.62f, _boundsMin.z + height * 0.72f } };
    }

    model::Vec3 Vehicle::repairPoint() const
    {
        if (_spec.damage.hasRepairPoint) {
            return _spec.damage.repairPoint;
        }
        const float height = _boundsMax.z - _boundsMin.z;
        return { 0.0f, _boundsMax.y - 0.15f, _boundsMin.z + height * (_spec.lean.enabled ? 0.55f : 0.68f) };
    }

    void Vehicle::probeImpact()
    {
        // [game] What did the hull just hit? The step thread saw the collision's velocity change; the obstacle lies the other way. Cast rays
        // from the centre of mass toward it (and to either side and up/down of it) and log the layer, body and distance of each hit, plus the
        // ground below the four bottom corners of the hull. The next log tells whether it is scenery, clutter, an actor or the ground.
        _impactCooldown = (std::max)(0.0f, _impactCooldown - 1.0f / 90.0f);
        ImpactEvent ev;
        {
            std::scoped_lock lock(_impactMutex);
            if (!_impact.pending) {
                return;
            }
            ev = _impact;
            _impact.pending = false;
        }
        const float len = model::length(ev.residual);
        if (len < 1e-3f) {
            return;
        }
        // Collision damage (bVehicleHealth): the hull's velocity change, whatever it hit; VehicleHealth keeps its own threshold and cooldown.
        if (g_config.vehicleHealth && !_health.disabled()) {
            const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            const float taken = _health.applyImpact(len, now);
            if (taken > 0.0f) {
                logger::info("VehicleHealth: collision at {:.1f} m/s (driving at {:.0f} km/h) costs {:.1f}: health {:.0f} / {:.0f}{}", len, ev.speed * 3.6f, taken,
                    _health.health(), _health.maxHealth(), _health.disabled() ? " - DISABLED" : "");
            }
        }
        // The probe of what was hit: at most every 0.3 s, and only from fImpactLogMps.
        if (_impactCooldown > 0.0f || g_config.impactLogMps <= 0.0f || len < g_config.impactLogMps) {
            return;
        }
        _impactCooldown = 0.3f;
        const model::Vec3 toward = ev.residual * (-1.0f / len);
        const model::Vec3 worldUp{ 0.0f, 0.0f, 1.0f };
        model::Vec3 side = model::cross(toward, worldUp);
        if (model::length(side) < 0.2f) {
            side = ev.frame.rot.axisX();
        }
        side = side * (1.0f / model::length(side));
        const model::Vec3 lift = model::cross(side, toward);
        const float tilt = 0.45f;
        struct Probe
        {
            const char* name;
            model::Vec3 dir;
        };

        const std::array<Probe, 5> probes{ {
            { "toward", toward },
            { "left", toward + side * tilt },
            { "right", toward - side * tilt },
            { "above", toward + lift * tilt },
            { "below", toward - lift * tilt },
        } };
        const auto pos = physics::toGame(ev.frame.pos);
        std::string text = std::format("Vehicle: IMPACT {:.1f} m/s (dv {:.1f}, {:.1f}, {:.1f}) at {:.1f} km/h, car at ({:.0f}, {:.0f}, {:.0f}), heading {:.0f} deg, up.z {:.2f}; obstacle direction ({:.2f}, {:.2f}, {:.2f})",
            len,
            ev.residual.x,
            ev.residual.y,
            ev.residual.z,
            ev.speed * 3.6f,
            pos.x,
            pos.y,
            pos.z,
            model::headingOf(ev.frame.rot.axisY()) * 180.0f / model::PI,
            ev.frame.rot.axisZ().z,
            toward.x,
            toward.y,
            toward.z);
        // A body on a layer the game does not name (47 and up) is not scenery: ROCK registers its own layers there (hands 43/48/52, weapon 44/51,
        // the player's body 47). Moving one to the non-collidable layer (what this used to do after an impact) took the player's body out of the
        // physics for the rest of the session: no more bumping into the vehicle. It is only reported now.
        const auto muteUnknownLayer = [&](const std::uint32_t bodyId, const std::uint32_t filterInfo, std::string& out) {
            const auto layer = filterInfo & 0x7Fu;
            if (layer < 47 || layer == 0x7Fu) {
                return;
            }
            out += std::format("\n  body {} is on layer {} (a ROCK layer: left alone)", bodyId, layer);
        };
        std::string muteText;
        constexpr float PROBE_LENGTH = 6.0f; // m
        for (const auto& probe : probes) {
            const auto dir = probe.dir * (1.0f / (std::max)(model::length(probe.dir), 1e-3f));
            const auto hit = physics::castRay(
                _body.bhkWorld, physics::toGame(ev.com), physics::toGame(ev.com + dir * PROBE_LENGTH), _rayFilter, _body.bodyId.value, g_config.wheelRayIgnoreMask, g_config.releasePickWorldReference);
            if (hit.hit) {
                muteUnknownLayer(hit.bodyId, hit.filterInfo, muteText);
                const auto point = physics::toMeters(hit.point);
                text += std::format("\n  {}: layer '{}', body {}, {:.2f} m from the centre of mass, normal ({:.2f}, {:.2f}, {:.2f}), at game ({:.0f}, {:.0f}, {:.0f})",
                    probe.name,
                    f4vr::getCollisionLayerName(hit.filterInfo),
                    hit.bodyId,
                    model::length(point - ev.com),
                    hit.normal.x,
                    hit.normal.y,
                    hit.normal.z,
                    hit.point.x,
                    hit.point.y,
                    hit.point.z);
            } else {
                text += std::format("\n  {}: nothing within {:.0f} m", probe.name, PROBE_LENGTH);
            }
        }
        if (_spec.hull.explicitBox) {
            for (const float cx : { _spec.hull.min.x, _spec.hull.max.x }) {
                for (const float cy : { _spec.hull.min.y, _spec.hull.max.y }) {
                    const auto corner = ev.frame.toWorld({ cx, cy, _spec.hull.min.z });
                    const auto hit = physics::castRay(
                        _body.bhkWorld, physics::toGame(corner + worldUp * 0.3f), physics::toGame(corner - worldUp * 2.0f), _rayFilter, _body.bodyId.value, g_config.wheelRayIgnoreMask, g_config.releasePickWorldReference);
                    text += hit.hit ? std::format("\n  ground below hull corner ({:+.1f}, {:+.1f}): {:.2f} m (layer '{}')", cx, cy, corner.z - physics::toMeters(hit.point).z, f4vr::getCollisionLayerName(hit.filterInfo))
                                    : std::format("\n  ground below hull corner ({:+.1f}, {:+.1f}): none within 2 m", cx, cy);
                }
            }
        }
        // Second look, with nothing ignored: the probes above skip actors and the invisible volumes the wheels must roll over, but the hull (on the
        // driver's layer) collides with some of those. A fan of rays around the centre of mass, three elevations, lists the nearest distinct bodies.
        {
            struct FanHit
            {
                std::uint32_t body;
                std::uint32_t filter;
                float distance;
                float height;
                model::Vec3 normal;
            };
            std::vector<FanHit> found;
            constexpr float FAN_LENGTH = 7.0f; // m
            for (const float elevation : { -0.25f, 0.0f, 0.3f }) {
                for (int k = 0; k < 16; ++k) {
                    const float a = static_cast<float>(k) * (2.0f * model::PI / 16.0f);
                    const model::Vec3 dir{ std::cos(a), std::sin(a), elevation };
                    const auto unit = dir * (1.0f / model::length(dir));
                    const auto hit = physics::castRay(_body.bhkWorld, physics::toGame(ev.com), physics::toGame(ev.com + unit * FAN_LENGTH), _rayFilter, _body.bodyId.value, 0,
                        g_config.releasePickWorldReference);
                    if (!hit.hit) {
                        continue;
                    }
                    muteUnknownLayer(hit.bodyId, hit.filterInfo, muteText);
                    const auto point = physics::toMeters(hit.point);
                    const bool known = std::any_of(found.begin(), found.end(), [&](const FanHit& f) { return f.body == hit.bodyId; });
                    if (!known) {
                        found.push_back({ hit.bodyId, hit.filterInfo, model::length(point - ev.com), point.z - ev.com.z, hit.normal });
                    }
                }
            }
            std::sort(found.begin(), found.end(), [](const FanHit& l, const FanHit& r) { return l.distance < r.distance; });
            text += std::format("\n  fan (nothing ignored, {:.0f} m, {} distinct bodies):", FAN_LENGTH, found.size());
            for (std::size_t i = 0; i < found.size() && i < 8; ++i) {
                text += std::format("\n    layer '{}', body {}, {:.2f} m, {:+.2f} m above the centre of mass, normal ({:.2f}, {:.2f}, {:.2f})",
                    f4vr::getCollisionLayerName(found[i].filter), found[i].body, found[i].distance, found[i].height, found[i].normal.x, found[i].normal.y, found[i].normal.z);
            }
        }
        if (auto* player = RE::PlayerCharacter::GetSingleton()) {
            const auto p = physics::toMeters(player->GetPosition());
            text += std::format("\n  player actor at {:.1f} m from the centre of mass, {:+.1f} m above it", model::length(p - ev.com), p.z - ev.com.z);
        }
        logger::warn("{}{}", text, muteText);
    }

    void Vehicle::drawDebug(const model::Pose& frame) const
    {
        auto& dd = debug::dd();
        auto channel = dd.channelScope("vehicle");
        std::array<model::WheelContact, model::MAX_WHEELS> contacts;
        model::VehicleTelemetry tel;
        float rayLength;
        std::array<model::Vec3, model::MAX_WHEELS> starts{};
        {
            std::scoped_lock lock(_mutex);
            contacts = _contacts;
            tel = _lastTelemetry;
            rayLength = _model.wheelRayLength();
            for (std::size_t i = 0; i < _spec.wheels.size(); ++i) {
                starts[i] = _model.wheelRayStartLocal(i);
            }
        }
        const model::Vec3 down = -frame.rot.axisZ();
        for (std::size_t i = 0; i < _spec.wheels.size(); ++i) {
            const model::Vec3 start = frame.toWorld(starts[i]);
            if (contacts[i].hit) {
                dd.line(gameVec(start), gameVec(contacts[i].point), debug::colors::Green);
                // Load on the wheel: 1 game unit per 100 N.
                dd.arrow(gameVec(contacts[i].point), physics::toNi(contacts[i].normal), tel.wheels[i].fz / 100.0f, debug::colors::Yellow);
                dd.point(gameVec(contacts[i].point), 4.0f, false, tel.wheels[i].holding ? debug::colors::Blue : debug::colors::Orange);
            } else {
                dd.line(gameVec(start), gameVec(start + down * rayLength), debug::colors::Red);
            }
        }
        dd.watch("speed km/h", std::format("{:.1f}", tel.speed * 3.6f));
        dd.watch("gear / rpm", std::format("{} / {:.0f}", tel.gear, tel.engineRpm));
        dd.watch("wheels down", std::format("{}", tel.groundedWheels));
    }
}
