#include "model/VehicleModel.h"

#include <algorithm>
#include <cmath>

namespace vfvr::model
{
    namespace
    {
        constexpr float RPM_PER_RAD_S = 60.0f / (2.0f * PI);
        constexpr int WHEEL_SUBSTEPS = 8;
        // Below this |speed| (m/s) a held brake with no throttle counts as asking for the other direction.
        constexpr float DIRECTION_SWITCH_SPEED = 0.8f;
        constexpr float DIRECTION_SWITCH_DELAY = 0.3f;
        // Hold spring per wheel: natural frequency and damping ratio of the anchor that keeps a parked car still.
        constexpr float HOLD_FREQUENCY_HZ = 2.5f;
        constexpr float HOLD_DAMPING_RATIO = 0.8f;
        constexpr float GRAVITY = 9.81f;
    }

    void VehicleModel::init(const VehicleSpec& spec)
    {
        _spec = spec;
        _wheels.assign((std::min)(_spec.wheels.size(), MAX_WHEELS), {});
        _spec.wheels.resize(_wheels.size());
        _drivenCount = 0;
        float drivenRadius = 0.0f;
        for (std::size_t i = 0; i < _wheels.size(); ++i) {
            auto& w = _spec.wheels[i];
            if (w.side == 0.0f) {
                w.side = w.hub.x >= 0.0f ? 1.0f : -1.0f;
            }
            if (w.radius <= 0.0f) {
                w.radius = _spec.tyre.radius;
            }
            if (w.drive) {
                ++_drivenCount;
                drivenRadius += w.radius;
            }
        }
        _drivenRadius = _drivenCount > 0 ? drivenRadius / static_cast<float>(_drivenCount) : _spec.tyre.radius;
        float frontY = -1e9f;
        float rearY = 1e9f;
        for (const auto& w : _spec.wheels) {
            frontY = (std::max)(frontY, w.hub.y);
            rearY = (std::min)(rearY, w.hub.y);
        }
        _wheelbase = (std::max)(frontY - rearY, 0.5f);
        // Axle partners: the closest wheel on the other side at nearly the same y.
        for (std::size_t i = 0; i < _wheels.size(); ++i) {
            float best = 0.3f;
            for (std::size_t j = 0; j < _wheels.size(); ++j) {
                if (i == j || _spec.wheels[i].side == _spec.wheels[j].side) {
                    continue;
                }
                const float dy = std::abs(_spec.wheels[i].hub.y - _spec.wheels[j].hub.y);
                if (dy < best) {
                    best = dy;
                    _wheels[i].partner = static_cast<int>(j);
                }
            }
        }
        if (_spec.engine.gears.empty()) {
            _spec.engine.gears.push_back(1.0f);
        }
        reset();
    }

    void VehicleModel::reset()
    {
        for (auto& w : _wheels) {
            w.spin = 0.0f;
            w.holding = false;
            w.compression = 0.0f;
        }
        _steerAngle = 0.0f;
        _leanTarget = 0.0f;
        _reverse = false;
        _directionTimer = 0.0f;
        _gearIndex = 0;
        _shiftTimer = 0.0f;
        _engineRpm = _spec.engine.idleRpm;
        _throttle = 0.0f;
        _brake = 0.0f;
        _telemetry = {};
    }

    Vec3 VehicleModel::wheelRayStartLocal(const std::size_t index) const
    {
        // Start above the hub by the full travel, so a fully compressed wheel still starts above the ground.
        const auto& hub = _spec.wheels[index].hub;
        return { hub.x, hub.y, hub.z + _spec.suspension.restLength };
    }

    float VehicleModel::wheelRayLength() const
    {
        // Travel + tyre radius reaches the ground at full extension; the margin keeps contact over small bumps
        // between two rays (the plane is reused for every physics step until the next ray).
        float radius = _spec.tyre.radius;
        for (const auto& w : _spec.wheels) {
            radius = (std::max)(radius, w.radius);
        }
        return _spec.suspension.restLength + radius + 0.35f;
    }

    float VehicleModel::engineTorqueAt(const float rpm) const
    {
        const auto& curve = _spec.engine.torqueCurve;
        if (curve.empty()) {
            return 0.0f;
        }
        if (rpm <= curve.front().first) {
            return curve.front().second;
        }
        for (std::size_t i = 1; i < curve.size(); ++i) {
            if (rpm <= curve[i].first) {
                const auto& [r0, t0] = curve[i - 1];
                const auto& [r1, t1] = curve[i];
                const float t = (rpm - r0) / (std::max)(r1 - r0, 1.0f);
                return lerpf(t0, t1, t);
            }
        }
        return curve.back().second;
    }

    float VehicleModel::tyreCurve(const float slip, const float b, const float c)
    {
        return std::sin(c * std::atan(b * slip));
    }

    void VehicleModel::updateDirectionAndGear(const DriverInput& input, const float forwardSpeed, const float dt)
    {
        // Direction: at (almost) a standstill, holding the brake with no throttle selects reverse after a short
        // delay, and throttle selects forward again. In reverse the pedals swap: brake drives, throttle brakes.
        const bool nearlyStopped = std::abs(forwardSpeed) < DIRECTION_SWITCH_SPEED;
        const bool wantsOther = _reverse ? input.throttle > 0.3f && input.brake < 0.05f : input.brake > 0.3f && input.throttle < 0.05f;
        if (nearlyStopped && wantsOther) {
            _directionTimer += dt;
            if (_directionTimer >= DIRECTION_SWITCH_DELAY) {
                _reverse = !_reverse;
                _directionTimer = 0.0f;
                _gearIndex = 0;
            }
        } else {
            _directionTimer = 0.0f;
        }
        _throttle = _reverse ? input.brake : input.throttle;
        _brake = _reverse ? input.throttle : input.brake;

        // Engine speed follows the driven wheels; below idle the clutch slips and the engine sits at idle.
        // Only wheels on the ground count: a wheel hanging in the air spins up freely and must not drag the engine speed
        // (and the gearbox) with it. With every driven wheel airborne the engine follows the road speed instead.
        float drivenSpin = 0.0f;
        int groundedDriven = 0;
        for (std::size_t i = 0; i < _wheels.size(); ++i) {
            if (_spec.wheels[i].drive && _wheels[i].grounded) {
                drivenSpin += std::abs(_wheels[i].spin);
                ++groundedDriven;
            }
        }
        drivenSpin = groundedDriven > 0 ? drivenSpin / static_cast<float>(groundedDriven) : std::abs(forwardSpeed) / _drivenRadius;
        const auto& eng = _spec.engine;
        const float ratio = _reverse ? eng.reverseRatio : eng.gears[static_cast<std::size_t>(_gearIndex)];
        _engineRpm = (std::max)(drivenSpin * ratio * eng.finalDrive * RPM_PER_RAD_S, eng.idleRpm);

        // Automatic gearbox (forward only).
        if (_shiftTimer > 0.0f) {
            _shiftTimer = (std::max)(_shiftTimer - dt, 0.0f);
            return;
        }
        if (_reverse) {
            return;
        }
        const int lastGear = static_cast<int>(eng.gears.size()) - 1;
        if (_engineRpm > eng.shiftUpRpm && _gearIndex < lastGear && _throttle > 0.05f) {
            ++_gearIndex;
            _shiftTimer = eng.shiftTime;
        } else if (_gearIndex > 0) {
            const float lowerRpm = drivenSpin * eng.gears[static_cast<std::size_t>(_gearIndex - 1)] * eng.finalDrive * RPM_PER_RAD_S;
            if (_engineRpm < eng.shiftDownRpm && lowerRpm < eng.shiftUpRpm * 0.9f) {
                --_gearIndex;
                _shiftTimer = eng.shiftTime * 0.5f;
            }
        }
    }

    float VehicleModel::driveTorquePerWheel(const float throttle, const float forwardSpeed) const
    {
        if (_drivenCount == 0 || _shiftTimer > 0.0f) {
            return 0.0f;
        }
        const auto& eng = _spec.engine;
        const float ratio = _reverse ? eng.reverseRatio : eng.gears[static_cast<std::size_t>(_gearIndex)];
        const float direction = _reverse ? -1.0f : 1.0f;
        float crank;
        if (throttle > 0.02f) {
            crank = _engineRpm >= eng.maxRpm ? 0.0f : engineTorqueAt(_engineRpm) * throttle;
            // Governor: drive fades out approaching the profile's top speed (reverse is capped at a third of it).
            const float top = _spec.aero.topSpeedKmh / 3.6f * (_reverse ? 0.33f : 1.0f);
            crank *= 1.0f - smoothstep(top * 0.93f, top, std::abs(forwardSpeed));
        } else {
            // Engine braking while rolling in gear.
            crank = _engineRpm > eng.idleRpm * 1.1f ? -eng.engineBrake : 0.0f;
        }
        return direction * crank * ratio * eng.finalDrive * eng.efficiency / static_cast<float>(_drivenCount);
    }

    float VehicleModel::brakeTorqueFor(const WheelSpec& wheel, const float brake, const bool handbrake) const
    {
        const auto& b = _spec.brakes;
        const float bias = wheel.hub.y >= 0.0f ? b.frontBias : 1.0f - b.frontBias;
        float torque = b.maxTorque * brake * 2.0f * bias;
        if (handbrake && wheel.handbrake) {
            torque += b.handbrakeTorque;
        }
        return torque;
    }

    void VehicleModel::stabiliseRoll(const BodyState& body, const float forwardSpeed, const float dt, std::vector<AppliedImpulse>& out)
    {
        const auto& lean = _spec.lean;
        const Vec3 right = body.frame.rot.axisX();
        const Vec3 forward = body.frame.rot.axisY();
        const Vec3 up = body.frame.rot.axisZ();
        // Roll: how far the bike's right axis dips below the horizon (positive = leaning right, the top toward the right). It keeps its size when the
        // bike pitches, and its sign past 90 degrees.
        const float roll = std::atan2(-right.z, up.z);
        const float rollRate = dot(body.angVel, forward);
        _telemetry.lean = roll;

        // What a steady corner at this speed and turn rate needs: tan(lean) = lateral acceleration / g. A right turn has a negative yaw rate.
        const float maxLean = lean.maxLeanDeg * PI / 180.0f;
        const float lateral = -forwardSpeed * body.angVel.z; // m/s^2 toward the bike's right
        float target = clampf(std::atan(lateral / GRAVITY) * lean.follow, -maxLean, maxLean);
        target *= smoothstep(lean.uprightBelow, lean.uprightBelow * 2.0f, std::abs(forwardSpeed));
        // Short lag: the turn rate rings a little as the tyres bite, and the rider's lean should not copy that.
        _leanTarget += (target - _leanTarget) * (1.0f - std::exp(-dt / (std::max)(lean.targetSeconds, 0.01f)));
        _telemetry.leanTarget = _leanTarget;

        const float fall = lean.fallDeg * PI / 180.0f;
        const float tilt = std::acos(clampf(up.z, -1.0f, 1.0f));
        _telemetry.stabiliserTorque = 0.0f;
        _telemetry.stabiliserSaturated = false;
        if (_telemetry.groundedWheels == 0 || tilt >= fall) {
            return; // in the air or already down: the bike is on its own
        }
        const float authority = 1.0f - smoothstep(fall - 15.0f * PI / 180.0f, fall, tilt);

        // Spring-damper on the roll angle (critically damped-ish, ~2.5 Hz): the angular acceleration it asks for.
        const float omega = 2.0f * PI * (std::max)(lean.frequencyHz, 0.1f);
        const float alpha = -omega * omega * (roll - _leanTarget) - 2.0f * lean.damping * omega * rollRate;

        // The tyres and springs already put a roll torque on the body (the weight of a leaning bike pulls it over, a corner force acts above the
        // ground): cancel it, so the roll follows the spring-damper whatever the wheels do.
        float wheelTorque = 0.0f;
        for (const auto& impulse : out) {
            wheelTorque += dot(cross(impulse.point - body.com, impulse.impulse), forward);
        }
        wheelTorque /= dt;
        const float maxTorque = lean.maxTorque > 0.0f ? lean.maxTorque : 1.5f * _spec.mass * GRAVITY;
        const float wanted = (_spec.rollInertia() * alpha - wheelTorque) * authority;
        const float torque = clampf(wanted, -maxTorque, maxTorque);
        _telemetry.stabiliserTorque = torque;
        _telemetry.stabiliserSaturated = std::abs(wanted) > maxTorque;

        // A couple: +j along the right axis above the centre of mass and -j below it turns the bike about its forward axis (up x right = forward)
        // and pushes it nowhere.
        const float arm = (std::max)(lean.armLength, 0.05f);
        const float j = torque * dt / (2.0f * arm);
        if (std::isfinite(j)) {
            out.push_back({ right * j, body.com + up * arm });
            out.push_back({ right * -j, body.com - up * arm });
        }
    }

    void VehicleModel::stabilisePitch(const BodyState& body, const float dt, std::vector<AppliedImpulse>& out)
    {
        const auto& lean = _spec.lean;
        _telemetry.pitchTorque = 0.0f;
        if (lean.pitchDamping <= 0.0f && lean.airLevelHz <= 0.0f) {
            return;
        }
        const Vec3 right = body.frame.rot.axisX();
        const Vec3 forward = body.frame.rot.axisY();
        const Vec3 up = body.frame.rot.axisZ();
        if (up.z < std::cos(lean.fallDeg * PI / 180.0f)) {
            return; // down: the bike is on its own
        }
        // Pitch rate about the right axis, positive = nose up.
        const float pitchRate = dot(body.angVel, right);
        float alpha = -lean.pitchDamping * pitchRate;
        if (_telemetry.groundedWheels == 0 && lean.airLevelHz > 0.0f) {
            // In the air: bring the bike level with the ground it left (the wheels' last ground normals), critically damped.
            Vec3 n{};
            for (const auto& w : _wheels) {
                n += w.lastNormal;
            }
            n = normalizeOr(n, { 0.0f, 0.0f, 1.0f });
            const float pitchError = std::asin(clampf(dot(forward, n), -1.0f, 1.0f)); // > 0 = nose up against that ground
            const float omega = 2.0f * PI * lean.airLevelHz;
            alpha = -omega * omega * pitchError - 2.0f * omega * pitchRate;
        }
        const float maxTorque = lean.pitchMaxTorque > 0.0f ? lean.pitchMaxTorque : 0.6f * _spec.mass * GRAVITY;
        const float torque = clampf(_spec.pitchInertia() * alpha, -maxTorque, maxTorque);
        _telemetry.pitchTorque = torque;
        // A couple along the up axis, ahead of and behind the centre of mass: it turns the bike about its right axis and pushes it nowhere.
        const float arm = (std::max)(lean.armLength, 0.05f);
        const float j = torque * dt / (2.0f * arm);
        if (std::isfinite(j)) {
            out.push_back({ up * j, body.com + forward * arm });
            out.push_back({ up * -j, body.com - forward * arm });
        }
    }

    void VehicleModel::step(const BodyState& body, const DriverInput& input, const std::span<const WheelContact> contacts, const float dt, std::vector<AppliedImpulse>& out)
    {
        out.clear();
        _telemetry.dt = dt;
        if (!(dt > 0.0f) || dt > 0.1f || _wheels.empty()) {
            return;
        }

        const Vec3 up = body.frame.rot.axisZ();
        const Vec3 forward = body.frame.rot.axisY();
        const float forwardSpeed = dot(body.linVel, forward);

        updateDirectionAndGear(input, forwardSpeed, dt);

        // Steering: speed-sensitive range, rate-limited toward the stick.
        const auto& st = _spec.steering;
        float steerRange = st.maxAngleDeg * PI / 180.0f;
        if (st.maxLateralG > 0.0f) {
            // The angle that corners at maxLateralG at this speed (kinematic: a = v^2 * tan(angle) / wheelbase); 1 m^2/s^2 floor keeps full lock when crawling.
            const float v2 = (std::max)(forwardSpeed * forwardSpeed, 1.0f);
            steerRange = (std::min)(steerRange, std::atan(_wheelbase * st.maxLateralG * GRAVITY / v2));
        } else {
            steerRange *= lerpf(1.0f, st.reducedFactor, clampf(std::abs(forwardSpeed) / (std::max)(st.reduceAtSpeed, 0.1f), 0.0f, 1.0f));
        }
        if (st.yawLimitG > 0.0f) {
            // Measured lateral acceleration (speed * yaw rate): the stick's range fades out as it nears the limit, so a bike already turning as hard as its
            // tyres allow is not asked for more.
            const float limit = st.yawLimitG * GRAVITY;
            steerRange *= 1.0f - smoothstep(limit * 0.7f, limit, std::abs(forwardSpeed * body.angVel.z));
        }
        const float targetSteer = clampf(input.steer, -1.0f, 1.0f) * steerRange;
        const float maxDelta = st.speedDegPerSec * PI / 180.0f * dt;
        _steerAngle += clampf(targetSteer - _steerAngle, -maxDelta, maxDelta);

        // Pulling the handbrake declutches, as a driver does for a handbrake turn: no drive and no engine braking, which on a four-wheel drive
        // would otherwise brake the front wheels as well.
        float driveTorque = input.handbrake ? 0.0f : driveTorquePerWheel(_throttle, forwardSpeed);
        if (_spec.lean.enabled && _spec.lean.throttleCutAtMax > 0.0f) {
            // A two-wheeler leaned over: less drive, so the rear tyre keeps the grip it needs to hold the corner.
            const float roll = std::abs(std::atan2(-body.frame.rot.axisX().z, up.z)) / PI * 180.0f;
            const float cut = _spec.lean.throttleCutAtMax * smoothstep(_spec.lean.throttleCutStartDeg, (std::max)(_spec.lean.maxLeanDeg, _spec.lean.throttleCutStartDeg + 1.0f), roll);
            driveTorque *= 1.0f - clampf(cut, 0.0f, 1.0f);
        }
        const auto& sus = _spec.suspension;
        const auto& tyre = _spec.tyre;
        const float massShare = _spec.mass / static_cast<float>(_wheels.size());
        const float holdK = massShare * std::pow(2.0f * PI * HOLD_FREQUENCY_HZ, 2.0f);
        const float holdC = 2.0f * HOLD_DAMPING_RATIO * std::sqrt(holdK * massShare);
        const bool wantsToMove = _throttle > 0.05f;

        constexpr float MAX_COMPRESSION_SPEED = 2.0f; // m/s
        constexpr float MAX_EXTENSION_SPEED = 1.5f; // m/s
        // Pass 1: suspension compression for every wheel (anti-roll needs both wheels of an axle).
        struct Pass1
        {
            bool contact = false;
            Vec3 hub;
            Vec3 contactPoint;
            Vec3 normal;
            float denom = 1.0f;
        };

        std::array<Pass1, MAX_WHEELS> p1{};
        const float minGroundNormalZ = std::cos(clampf(_spec.maxGroundSlopeDeg, 0.0f, 89.0f) * PI / 180.0f);
        for (std::size_t i = 0; i < _wheels.size(); ++i) {
            auto& ws = _wheels[i];
            auto& r = p1[i];
            r.hub = body.frame.toWorld(_spec.wheels[i].hub);
            const float previousCompression = ws.compression;
            ws.compression = 0.0f;
            // The ray is cast once per game frame and a cracked road, a pebble or a missed ray makes the ground vanish for a moment: the wheel would
            // drop to zero force and the body with it. A real suspension extends at a finite speed, so a wheel that had ground a step ago keeps it
            // (on the last plane) while its compression runs out at MAX_EXTENSION_SPEED.
            const auto droop = [&]() {
                const float c = previousCompression - MAX_EXTENSION_SPEED * dt;
                if (c <= 0.0f) {
                    return;
                }
                const Vec3 n = ws.lastNormal;
                r.denom = (std::max)(dot(up, n), 0.2f);
                r.contact = true;
                r.normal = n;
                r.contactPoint = r.hub - up * (_spec.wheels[i].radius - c);
                ws.compression = c;
            };
            if (i >= contacts.size() || !contacts[i].hit) {
                droop();
                continue;
            }
            const Vec3 n = normalizeOr(contacts[i].normal, up);
            r.denom = dot(up, n);
            if (r.denom < minGroundNormalZ) {
                continue; // a wall or a steep face, not something a wheel can stand on
            }
            // Distance from the (fully extended) hub down to the ground plane along the suspension axis.
            const float d = dot(r.hub - contacts[i].point, n) / r.denom;
            const float radius = _spec.wheels[i].radius;
            if (d > radius) {
                droop(); // in the air, or the ground dropped away for a moment
                continue;
            }
            r.contact = true;
            r.normal = n;
            ws.lastNormal = n;
            r.contactPoint = r.hub - up * d;
            ws.compression = (std::min)(radius - d, sus.restLength);
            // A kerb edge or a ledge must not push one corner up like a wall: the suspension can only be squeezed so fast.
            ws.compression = (std::min)(ws.compression, previousCompression + MAX_COMPRESSION_SPEED * dt);
        }

        // The drive torque goes to the driven wheels that touch the ground (a locked differential), not to one spinning in the air.
        int groundedDriven = 0;
        for (std::size_t i = 0; i < _wheels.size(); ++i) {
            _wheels[i].grounded = p1[i].contact;
            if (_spec.wheels[i].drive && p1[i].contact) {
                ++groundedDriven;
            }
        }
        const float driveShare = groundedDriven > 0 ? static_cast<float>(_drivenCount) / static_cast<float>(groundedDriven) : 0.0f;

        _telemetry.groundedWheels = 0;
        for (std::size_t i = 0; i < _wheels.size(); ++i) {
            auto& ws = _wheels[i];
            const auto& spec = _spec.wheels[i];
            auto& tel = _telemetry.wheels[i];
            const auto& r = p1[i];
            const float radius = spec.radius;
            tel = {};
            // The service brake goes through ABS; the handbrake (mechanical, rear wheels only) does not, so it can lock them.
            const float serviceBrake = brakeTorqueFor(spec, _brake, false);
            const float handbrakeTorque = input.handbrake && spec.handbrake ? _spec.brakes.handbrakeTorque : 0.0f;
            const float brakeTorque = serviceBrake + handbrakeTorque;
            const float wheelDrive = spec.drive && r.contact ? driveTorque * driveShare : 0.0f;

            if (!r.contact) {
                // Free wheel: drive spins it up, brakes and bearing friction slow it down.
                ws.holding = false;
                ws.spin += wheelDrive / tyre.wheelInertia * dt;
                const float slow = (brakeTorque + 2.0f) / tyre.wheelInertia * dt;
                ws.spin = std::abs(ws.spin) <= slow ? 0.0f : ws.spin - signf(ws.spin) * slow;
                tel.spin = ws.spin;
                continue;
            }

            const Vec3 n = r.normal;
            const Vec3 pointVel = body.linVel + cross(body.angVel, r.contactPoint - body.com);

            // --- Suspension ---
            const float compressionRate = -dot(pointVel, n) / r.denom;
            const float damper = compressionRate > 0.0f ? sus.damperBump : sus.damperRebound;
            float fz = sus.springRate * ws.compression + damper * compressionRate;
            if (ws.partner >= 0) {
                const float antiRoll = spec.hub.y >= 0.0f ? sus.antiRollFront : sus.antiRollRear;
                fz += antiRoll * (ws.compression - _wheels[static_cast<std::size_t>(ws.partner)].compression);
            }
            fz = clampf(fz, 0.0f, sus.maxForce);
            // The bump stop, on top of the capped spring and damper: it only acts in the last part of the travel.
            fz += sus.bumpStopForce(ws.compression, compressionRate);

            // --- Contact frame ---
            const Quat steerRot = spec.steer ? Quat::fromAxisAngle(up, -_steerAngle) : Quat{};
            const Vec3 wheelForward = normalizeOr(projectOnPlane(steerRot.rotate(forward), n), forward);
            const Vec3 wheelRight = cross(wheelForward, n);
            const float vLong = dot(pointVel, wheelForward);
            const float vLat = dot(pointVel, wheelRight);
            const float mu = tyre.gripAsphalt * (i < contacts.size() ? contacts[i].grip : 1.0f);
            const float maxFriction = mu * fz;

            float fx = 0.0f;
            float fy = 0.0f;
            float slipRatio = 0.0f;
            float slipAngle = 0.0f;

            const float patchSpeed = std::sqrt(vLong * vLong + vLat * vLat);
            const bool hold = !wantsToMove && patchSpeed < _spec.brakes.holdSpeed && fz > 0.0f;
            if (hold) {
                // Parked: tie the contact patch to an anchor on the ground with a stiff, damped spring that can
                // never pull harder than the tyre's grip. A slope steeper than the grip still slides.
                if (!ws.holding) {
                    ws.holding = true;
                    ws.holdAnchor = r.contactPoint;
                }
                const Vec3 offset = projectOnPlane(r.contactPoint - ws.holdAnchor, n);
                Vec3 f = -(offset * holdK + projectOnPlane(pointVel, n) * holdC);
                const float mag = length(f);
                if (mag > maxFriction) {
                    f = f * (maxFriction / mag);
                    ws.holdAnchor = r.contactPoint; // slipped: re-anchor where the tyre now is
                }
                fx = dot(f, wheelForward);
                fy = dot(f, wheelRight);
                ws.spin = 0.0f;
            } else {
                ws.holding = false;
                const float vRef = (std::max)(std::abs(vLong), tyre.minSlipSpeed);
                slipAngle = std::atan2(vLat, vRef);

                if (_simpleMode) {
                    // Diagnostic model: force straight from the inputs, limited by grip; no wheel spin state.
                    fx = wheelDrive / radius - signf(vLong) * (std::min)(brakeTorque / radius, std::abs(vLong) * massShare / dt);
                    fy = -maxFriction * clampf(vLat / 0.3f, -1.0f, 1.0f);
                    ws.spin = vLong / radius;
                } else {
                    // Wheel spin, integrated in substeps. The tyre force is linearized around the current slip
                    // (secant stiffness) and solved implicitly, which stays stable however stiff the tyre is.
                    const float h = dt / static_cast<float>(WHEEL_SUBSTEPS);
                    const float absSlip = _spec.brakes.absSlip;
                    const float iw = tyre.wheelInertia;
                    const float tcSlip = _spec.engine.tractionControlSlip;
                    float fxSum = 0.0f;
                    float spin = ws.spin;
                    for (int s = 0; s < WHEEL_SUBSTEPS; ++s) {
                        // Brake: Coulomb friction on the hub, can stop the wheel but never reverse it.
                        // ABS drops the brake for this substep while the wheel slips past the threshold.
                        const float slipNow = (spin * radius - vLong) / vRef;
                        const bool absRelease = absSlip > 0.0f && std::abs(vLong) > 1.5f && slipNow * signf(vLong) < -absSlip;
                        const float brakeDelta = ((absRelease ? 0.0f : serviceBrake) + handbrakeTorque) / iw * h;
                        spin = std::abs(spin) <= brakeDelta ? 0.0f : spin - signf(spin) * brakeDelta;

                        // Traction control: back off the drive torque while the driven wheel spins past the grip peak.
                        float drive = wheelDrive;
                        if (tcSlip > 0.0f && drive != 0.0f) {
                            const float over = slipNow * signf(drive);
                            drive *= 1.0f - smoothstep(tcSlip, tcSlip * 2.0f, over);
                        }

                        const float slip = (spin * radius - vLong) / vRef;
                        const float f0 = maxFriction * tyreCurve(slip, tyre.longB, tyre.longC);
                        const float initialSlope = maxFriction * tyre.longB * tyre.longC;
                        const float k = std::abs(slip) > 1e-4f ? std::clamp(f0 / slip, 0.0f, initialSlope) : initialSlope;
                        // Fx(spin) = k * (spin * r - vLong) / vRef; Iw * dspin/dt = drive - r * Fx
                        const float a = k * radius / vRef;
                        spin = (iw / h * spin + drive + radius * k * vLong / vRef) / (iw / h + radius * a);
                        const float slipAfter = (spin * radius - vLong) / vRef;
                        fxSum += maxFriction * tyreCurve(slipAfter, tyre.longB, tyre.longC);
                    }
                    ws.spin = spin;
                    fx = fxSum / static_cast<float>(WHEEL_SUBSTEPS);
                    slipRatio = (spin * radius - vLong) / vRef;
                    fy = -maxFriction * tyreCurve(slipAngle, tyre.latB, tyre.latC);
                }

                // Rolling resistance, faded in so it cannot rock a car that is almost stopped.
                fx -= signf(vLong) * tyre.rollingResistance * fz * smoothstep(0.0f, 0.5f, std::abs(vLong));

                // Combined slip: the tyre can only give its grip once, split between braking/driving and cornering.
                const float total = std::sqrt(fx * fx + fy * fy);
                if (total > maxFriction && total > 0.0f) {
                    if (_spec.lean.enabled) {
                        // A two-wheeler that gives up its side grip under power or braking falls over or slides out: cornering comes first, driving and
                        // braking get what is left (never less than ~40 % of the grip, so the bike still brakes in a lean).
                        const float side = (std::min)(std::abs(fy), maxFriction * 0.92f);
                        const float left = std::sqrt((std::max)(maxFriction * maxFriction - side * side, 0.0f));
                        fx = clampf(fx, -left, left);
                        fy = clampf(fy, -maxFriction, maxFriction);
                    }
                    const float total2 = std::sqrt(fx * fx + fy * fy);
                    if (total2 > maxFriction && total2 > 0.0f) {
                        const float scale = maxFriction / total2;
                        fx *= scale;
                        fy *= scale;
                    }
                }
            }

            // The suspension pushes along the ground normal at the contact patch. The tyre's horizontal force acts there too, or, with
            // frictionLift, higher up toward the height of the centre of mass: a tall car then leans less in a corner and under braking.
            const Vec3 verticalForce = n * fz;
            const Vec3 horizontalForce = wheelForward * fx + wheelRight * fy;
            if (isFinite(verticalForce) && isFinite(horizontalForce)) {
                if (sus.frictionLift > 0.0f) {
                    const float comHeight = (std::max)(dot(body.com - r.contactPoint, up), 0.0f);
                    out.push_back({ verticalForce * dt, r.contactPoint });
                    out.push_back({ horizontalForce * dt, r.contactPoint + up * (clampf(sus.frictionLift, 0.0f, 1.0f) * comHeight) });
                } else {
                    out.push_back({ (verticalForce + horizontalForce) * dt, r.contactPoint });
                }
            }

            ++_telemetry.groundedWheels;
            tel.grounded = true;
            tel.holding = ws.holding;
            tel.compression = ws.compression;
            tel.spin = ws.spin;
            tel.slipRatio = slipRatio;
            tel.slipAngle = slipAngle;
            tel.fz = fz;
            tel.fx = fx;
            tel.fy = fy;
            tel.gripUsed = maxFriction > 0.0f ? std::sqrt(fx * fx + fy * fy) / maxFriction : 0.0f;
            tel.vLong = vLong;
            tel.vLat = vLat;
        }

        // Aerodynamic drag at the centre of mass.
        const float speed = length(body.linVel);
        if (speed > 0.01f) {
            const Vec3 drag = body.linVel * (-_spec.aero.dragCoefficient * speed);
            out.push_back({ drag * dt, body.com });
        }

        _telemetry.lean = 0.0f;
        _telemetry.leanTarget = 0.0f;
        if (_spec.lean.enabled) {
            stabiliseRoll(body, forwardSpeed, dt, out);
            stabilisePitch(body, dt, out);
        } else {
            _telemetry.lean = std::atan2(-body.frame.rot.axisX().z, up.z);
        }

        _telemetry.speed = forwardSpeed;
        _telemetry.engineRpm = _engineRpm;
        _telemetry.gear = _reverse ? -1 : _gearIndex + 1;
        _telemetry.shifting = _shiftTimer > 0.0f;
        _telemetry.throttle = _throttle;
        _telemetry.brake = _brake;
        _telemetry.handbrake = input.handbrake;
        _telemetry.steerAngle = _steerAngle;
        _telemetry.steerInput = clampf(input.steer, -1.0f, 1.0f);
    }
}
