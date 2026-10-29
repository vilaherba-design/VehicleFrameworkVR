#pragma once

// The vehicle model: suspension, tyres with their own spin, drivetrain and brakes.
// Pure C++ with no game dependency, so it runs the same in the plugin (once per Havok step) and in
// the offline tests. It never moves the body itself: step() returns the impulses to apply this step,
// and Havok integrates them together with gravity and collisions.

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "model/VehicleSpec.h"

namespace vfvr::model
{
    inline constexpr std::size_t MAX_WHEELS = 8;

    // Ground under one wheel for this frame, from the wheel ray (world space, meters).
    // The model treats it as a plane, so it stays valid for every physics step until the next ray.
    struct WheelContact
    {
        bool hit = false;
        Vec3 point;
        Vec3 normal{ 0, 0, 1 };
        float grip = 1.0f; // surface multiplier on the tyre's friction (1 = reference asphalt)
        std::uint32_t layer = 0; // collision layer of what the ray hit (diagnostics only)
    };

    struct DriverInput
    {
        float throttle = 0.0f; // 0..1 (right trigger)
        float brake = 0.0f; // 0..1 (left trigger); also reverse when held at a standstill
        float steer = 0.0f; // -1 (left) .. 1 (right)
        bool handbrake = false;
        bool engineOff = false; // battery empty: no drive either way and no change of direction, the brakes still work
    };

    // Body state at the start of the step (world space, meters, m/s, rad/s).
    struct BodyState
    {
        Pose frame; // the vehicle frame (model node) in the world
        Vec3 com; // centre of mass in the world
        Vec3 linVel;
        Vec3 angVel;
    };

    struct AppliedImpulse
    {
        Vec3 impulse; // N*s, world
        Vec3 point; // world
    };

    struct WheelTelemetry
    {
        bool grounded = false;
        bool holding = false;
        float compression = 0.0f; // m
        float spin = 0.0f; // rad/s
        float slipRatio = 0.0f;
        float slipAngle = 0.0f; // rad
        float fz = 0.0f; // N
        float fx = 0.0f; // N, along the wheel heading
        float fy = 0.0f; // N, to the wheel's right
        float gripUsed = 0.0f; // |F| / (mu * Fz), 1 = at the limit
        float vLong = 0.0f; // m/s of the contact patch along the wheel
        float vLat = 0.0f;
    };

    struct VehicleTelemetry
    {
        float dt = 0.0f;
        float speed = 0.0f; // m/s along the vehicle forward axis
        float engineRpm = 0.0f;
        int gear = 1; // -1 = reverse, 1..n
        bool shifting = false;
        float throttle = 0.0f; // effective, after direction logic
        float brake = 0.0f;
        bool handbrake = false; // held by the driver this step
        float steerAngle = 0.0f; // rad
        int groundedWheels = 0;
        float lean = 0.0f; // rad, roll angle (positive = leaning right); every vehicle reports it, only two-wheelers hold it
        float leanTarget = 0.0f; // rad, what the roll stabiliser aims for
        float stabiliserTorque = 0.0f; // N*m the roll stabiliser applied this step (0 in the air or once down)
        float pitchTorque = 0.0f; // N*m the pitch stabiliser applied this step
        bool stabiliserSaturated = false; // it wanted more than lean.maxTorque
        float steerInput = 0.0f; // -1..1, what the driver asked for (steerAngle is what the wheels got)
        std::array<WheelTelemetry, MAX_WHEELS> wheels{};
    };

    class VehicleModel
    {
    public:
        void init(const VehicleSpec& spec);
        void reset();

        // Compute this step's impulses into `out` (cleared first). `contacts` must match spec.wheels.
        void step(const BodyState& body, const DriverInput& input, std::span<const WheelContact> contacts, float dt, std::vector<AppliedImpulse>& out);

        // Diagnostic mode: drive force straight from the throttle, no wheel spin and no slip curve.
        void setSimpleMode(const bool simple)
        {
            _simpleMode = simple;
        }

        bool isSimpleMode() const
        {
            return _simpleMode;
        }

        const VehicleSpec& spec() const
        {
            return _spec;
        }

        const VehicleTelemetry& telemetry() const
        {
            return _telemetry;
        }

        // Ray for wheel `index` in the vehicle frame: start point and length (meters, cast along -Z).
        Vec3 wheelRayStartLocal(std::size_t index) const;
        float wheelRayLength() const;

        // Engine torque at the crank for an rpm, from the profile curve.
        float engineTorqueAt(float rpm) const;

        // Magic-formula tyre curve: normalized force (-1..1) for a slip value.
        static float tyreCurve(float slip, float b, float c);

    private:
        struct WheelState
        {
            float spin = 0.0f;
            bool holding = false;
            Vec3 holdAnchor;
            int partner = -1; // other wheel on the same axle, for anti-roll
            float compression = 0.0f;
            bool grounded = false; // had ground contact in the last step
            Vec3 lastNormal{ 0.0f, 0.0f, 1.0f }; // ground normal of the last real contact (used while the suspension droops after a missed ray)
        };

        void updateDirectionAndGear(const DriverInput& input, float forwardSpeed, float dt);
        // Two-wheelers: add the couple that holds the bike at its lean angle to `out` (the wheels' impulses are already in it).
        void stabiliseRoll(const BodyState& body, float forwardSpeed, float dt, std::vector<AppliedImpulse>& out);
        void stabilisePitch(const BodyState& body, float dt, std::vector<AppliedImpulse>& out);
        float driveTorquePerWheel(float throttle, float forwardSpeed) const;
        float brakeTorqueFor(const WheelSpec& wheel, float brake, bool handbrake) const;

        VehicleSpec _spec;
        std::vector<WheelState> _wheels;
        int _drivenCount = 0;
        bool _simpleMode = false;

        float _steerAngle = 0.0f;
        float _leanTarget = 0.0f; // rad, the lean the stabiliser is following (filtered)
        float _drivenRadius = 0.33f; // mean tyre radius of the driven wheels (m)
        float _wheelbase = 2.5f; // m, distance between the front-most and the rear-most wheel
        bool _reverse = false;
        float _directionTimer = 0.0f;
        int _gearIndex = 0; // index into spec.engine.gears
        float _shiftTimer = 0.0f;
        float _engineRpm = 0.0f;
        float _throttle = 0.0f;
        float _brake = 0.0f;

        VehicleTelemetry _telemetry;
    };
}
