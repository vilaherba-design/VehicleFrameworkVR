#pragma once

// Shared by the offline vehicle model tests: a minimal stand-in for Havok (a rigid body with box inertia, gravity and the model's impulses) and
// the pass/fail bookkeeping. No game, no Havok.

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "model/VehicleModel.h"

using namespace vfvr::model;

namespace vfvr::testing
{
    inline int g_failures = 0;
    inline int g_checks = 0;

    inline void check(const bool ok, const std::string& what, const double value)
    {
        ++g_checks;
        std::printf("  [%s] %s (%.3f)\n", ok ? " OK " : "FAIL", what.c_str(), value);
        if (!ok) {
            ++g_failures;
        }
    }

    inline constexpr float GRAVITY = 9.81f;

    // Minimal stand-in for Havok: a box-inertia rigid body, semi-implicit Euler at the physics rate.
    struct TestWorld
    {
        VehicleModel model;
        Pose frame; // vehicle frame (model node)
        Vec3 comLocal{ 0.0f, 0.1f, 0.55f };
        Vec3 linVel;
        Vec3 angVel;
        float mass = 1400.0f;
        Vec3 inertiaLocal{ 2900.0f, 600.0f, 3050.0f }; // pitch, roll, yaw (kg m^2)
        Vec3 groundNormal{ 0, 0, 1 };
        float time = 0.0f;
        std::vector<AppliedImpulse> impulses;
        FILE* csv = nullptr; // optional telemetry in the plugin's format
        unsigned long long frameIndex = 0;
        std::function<bool(std::size_t, float)> rayMiss; // optional: true = this wheel's ray finds no ground this step (a crack, a pebble, a missed ray)
        std::function<float(float, float)> groundHeight; // optional: height (m) of the ground under (x, y), for bumps; the plane through the origin otherwise

        Vec3 com() const
        {
            return frame.toWorld(comLocal);
        }

        void place(const float height)
        {
            frame = {};
            frame.pos = { 0, 0, height };
            linVel = {};
            angVel = {};
        }

        WheelContact contactFor(std::size_t i) const
        {
            // Ray straight down the vehicle's -Z from above the hub, against the ground plane through the origin.
            const Vec3 start = frame.toWorld(model.wheelRayStartLocal(i));
            const Vec3 dir = -frame.rot.axisZ();
            const float denom = dot(dir, groundNormal);
            WheelContact c;
            if (denom >= -1e-4f) {
                return c;
            }
            const float h = groundHeight ? groundHeight(start.x, start.y) : 0.0f;
            const float t = -(dot(start, groundNormal) - h * groundNormal.z) / denom;
            if (t < 0.0f || t > model.wheelRayLength()) {
                return c;
            }
            c.hit = true;
            c.point = start + dir * t;
            c.normal = groundNormal;
            return c;
        }

        void step(const DriverInput& input, const float dt)
        {
            std::vector<WheelContact> contacts;
            for (std::size_t i = 0; i < model.spec().wheels.size(); ++i) {
                auto c = contactFor(i);
                if (rayMiss && rayMiss(i, time)) {
                    c.hit = false;
                }
                contacts.push_back(c);
            }
            const BodyState state{ frame, com(), linVel, angVel };
            model.step(state, input, contacts, dt, impulses);
            if (csv) {
                writeTelemetryRow(dt);
            }

            Vec3 linImpulse;
            Vec3 angImpulse;
            const Vec3 c = com();
            for (const auto& imp : impulses) {
                linImpulse += imp.impulse;
                angImpulse += cross(imp.point - c, imp.impulse);
            }
            linVel += linImpulse / mass + Vec3{ 0, 0, -GRAVITY } * dt;
            // Angular impulse through the world-space inverse inertia.
            const Vec3 local = frame.rot.rotateInverse(angImpulse);
            const Vec3 dwLocal{ local.x / inertiaLocal.x, local.y / inertiaLocal.y, local.z / inertiaLocal.z };
            angVel += frame.rot.rotate(dwLocal);

            // Integrate the centre of mass, then put the frame back around it.
            const Vec3 newCom = c + linVel * dt;
            const float angle = length(angVel) * dt;
            if (angle > 1e-9f) {
                frame.rot = (Quat::fromAxisAngle(angVel, angle) * frame.rot).normalized();
            }
            frame.pos = newCom - frame.rot.rotate(comLocal);
            time += dt;
        }

        void run(const float seconds, const float dt, const std::function<DriverInput(TestWorld&)>& driver, const std::function<bool(TestWorld&)>& stop = nullptr)
        {
            const float end = time + seconds;
            while (time < end) {
                step(driver(*this), dt);
                if (stop && stop(*this)) {
                    break;
                }
            }
        }

        // Same columns as Vehicle::onPhysicsStep writes in the plugin.
        void writeTelemetryRow(const float dt)
        {
            const auto& t = model.telemetry();
            std::fprintf(csv,
                "%llu,%.5f,%.3f,%.0f,%d,%.2f,%.2f,%.2f,%d,%.3f,%.3f,%.3f",
                ++frameIndex,
                dt,
                t.speed * 3.6f,
                t.engineRpm,
                t.gear,
                t.throttle,
                t.brake,
                t.steerAngle * 180.0f / PI,
                t.groundedWheels,
                angVel.z,
                linVel.z,
                frame.rot.axisZ().z);
            for (std::size_t i = 0; i < model.spec().wheels.size(); ++i) {
                const auto& w = t.wheels[i];
                std::fprintf(csv,
                    ",%d,%d,%.3f,%.2f,%.3f,%.2f,%.0f,%.0f,%.0f,%.2f",
                    w.grounded ? 1 : 0,
                    w.holding ? 1 : 0,
                    w.compression,
                    w.spin,
                    w.slipRatio,
                    w.slipAngle * 180.0f / PI,
                    w.fz,
                    w.fx,
                    w.fy,
                    w.gripUsed);
            }
            std::fprintf(csv, "\n");
        }

        float speed() const
        {
            return dot(linVel, frame.rot.axisY());
        }

        float yawRate() const
        {
            return angVel.z;
        }
    };

    inline VehicleSpec loadSpec(const char* path)
    {
        VehicleSpec spec;
        if (path) {
            spec = loadVehicleSpecFile(path);
            std::printf("Loaded profile '%s' from %s\n", spec.id.c_str(), path);
        } else {
            std::printf("Using the built-in sedan defaults\n");
        }
        // The sedan model bounds, approximately (meters).
        spec.ensureWheels({ -0.95f, -2.45f, 0.0f }, { 0.95f, 2.45f, 1.45f });
        return spec;
    }

    inline TestWorld makeWorld(const VehicleSpec& spec)
    {
        TestWorld w;
        w.mass = spec.mass;
        if (spec.hull.explicitBox) {
            // The game builds the body's mass properties from the hull (uniform density): its centre is the centre of mass.
            w.comLocal = (spec.hull.min + spec.hull.max) * 0.5f;
        }
        w.model.init(spec);
        w.place(0.05f);
        return w;
    }

    inline DriverInput idle(TestWorld&)
    {
        return {};
    }
}


using namespace vfvr::testing;
