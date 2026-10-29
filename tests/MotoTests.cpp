// Offline tests of a two-wheeler profile (rusianmother.json): the same rigid-body stand-in for Havok as the car tests, with the inertia of the profile's
// collision box, so the roll stabiliser can be judged: it must stand the bike upright at rest, lean it into corners the way a rider would, damp a push, and
// leave it alone once it is down. Also the performance targets of the bike (0-100 km/h, top speed, braking).
//
// Usage: MotoTests <path/to/rusianmother.json> [lap.csv]   (exit code 0 = all passed)

#include "TestWorld.h"

namespace
{
    constexpr float DEG = PI / 180.0f;

    // The game builds the body's mass properties from the collision box (uniform density).
    void useHullInertia(TestWorld& w, const VehicleSpec& spec)
    {
        const Vec3 size = spec.hull.max - spec.hull.min;
        const float k = spec.mass / 12.0f;
        w.inertiaLocal = { k * (size.y * size.y + size.z * size.z), k * (size.x * size.x + size.z * size.z), k * (size.x * size.x + size.y * size.y) };
    }

    TestWorld makeBike(const VehicleSpec& spec)
    {
        auto w = makeWorld(spec);
        useHullInertia(w, spec);
        return w;
    }

    float rollOf(const TestWorld& w)
    {
        return std::atan2(-w.frame.rot.axisX().z, w.frame.rot.axisZ().z);
    }

    float pitchOf(const TestWorld& w)
    {
        return std::asin(std::clamp(w.frame.rot.axisY().z, -1.0f, 1.0f));
    }

    float rollRateOf(const TestWorld& w)
    {
        return dot(w.angVel, w.frame.rot.axisY());
    }

    int grounded(const TestWorld& w)
    {
        return w.model.telemetry().groundedWheels;
    }

    DriverInput cruise(const TestWorld& w, const float kmh, const float steer)
    {
        const float v = w.speed() * 3.6f;
        return { v < kmh - 1.0f ? 1.0f : (v < kmh ? 0.25f : 0.0f), 0.0f, steer, false };
    }
}

int main(const int argc, char** argv)
{
    if (argc < 2) {
        std::printf("usage: MotoTests <profile.json> [lap.csv]\n");
        return 2;
    }
    VehicleSpec spec;
    try {
        spec = loadSpec(argv[1]);
    } catch (const std::exception& e) {
        std::printf("Profile load failed: %s\n", e.what());
        return 2;
    }
    if (!spec.lean.enabled || spec.wheels.size() != 2) {
        std::printf("%s is not a two-wheeler profile with a lean block\n", argv[1]);
        return 2;
    }
    std::printf("Roll inertia %.1f kg m^2, mass %.0f kg, lean %.0f deg max, %.1f Hz\n", spec.rollInertia(), spec.mass, spec.lean.maxLeanDeg, spec.lean.frequencyHz);

    std::printf("\n=== Wheel set ===\n");
    {
        // The default sWheelSet asks for "stock": the bike must have it, and it must leave the physics as they are.
        bool loaded = false;
        VehicleSpec stock;
        try {
            stock = loadVehicleSpecFile(argv[1], { { "wheels", "stock" } });
            loaded = true;
        } catch (const std::exception& e) {
            std::printf("  %s\n", e.what());
        }
        check(loaded, "the 'stock' wheel set loads", 0.0f);
        check(loaded && stock.mass == spec.mass && stock.wheels.size() == spec.wheels.size() && stock.wheels[0].radius == spec.wheels[0].radius &&
                stock.tyre.radius == spec.tyre.radius,
            "'stock' leaves the mass and the wheels as they are", stock.mass);
    }

    std::printf("\n=== Saddlebags ===\n");
    check(spec.hasTrunk && spec.trunkCapacity > 0.0f, "the bike has saddlebags (capacity, carry weight)", spec.trunkCapacity);
    check(spec.trunkPoint.y < 0.0f, "the saddlebags are behind the rider (y m)", spec.trunkPoint.y);

    for (const float rate : { 90.0f, 72.0f }) {
        const float dt = 1.0f / rate;
        std::printf("\n=== physics rate %.0f Hz ===\n", rate);

        {
            std::printf("Stands up from a tilt\n");
            auto w = makeBike(spec);
            w.frame.rot = Quat::fromAxisAngle({ 0, 1, 0 }, 12.0f * DEG);
            w.frame.pos = { 0, 0, 0.1f };
            w.run(3.0f, dt, idle);
            check(std::abs(rollOf(w)) < 0.7f * DEG, "upright after 3 s (roll deg)", rollOf(w) / DEG);
            const Vec3 p0 = w.frame.pos;
            float maxSpeed = 0.0f;
            float maxRoll = 0.0f;
            w.run(3.0f, dt, idle, [&](TestWorld& tw) {
                maxSpeed = (std::max)(maxSpeed, length(tw.linVel));
                maxRoll = (std::max)(maxRoll, std::abs(rollOf(tw)));
                return false;
            });
            check(maxSpeed < 0.02f, "no vibration at rest (max speed m/s)", maxSpeed);
            check(maxRoll < 0.7f * DEG, "stays upright at rest (max roll deg)", maxRoll / DEG);
            check(length(w.frame.pos - p0) < 0.01f, "no creep at rest (m over 3 s)", length(w.frame.pos - p0));
            check(grounded(w) == 2, "both wheels on the ground", grounded(w));
            const auto& t = w.model.telemetry();
            check(t.wheels[0].compression > 0.02f && t.wheels[0].compression < 0.11f, "front static compression in travel (m)", t.wheels[0].compression);
            check(t.wheels[1].compression > 0.02f && t.wheels[1].compression < 0.11f, "rear static compression in travel (m)", t.wheels[1].compression);
            // The model's wheel centres sit at their mesh height at the static ride (hub + compression), and its ground line (z 0.8 units) on the ground.
            std::printf("    wheel centres in the model: front %.4f m, rear %.4f m (the mesh has 0.3850 and 0.3630); body origin %.3f m above the ground\n",
                spec.wheels[0].hub.z + t.wheels[0].compression,
                spec.wheels[1].hub.z + t.wheels[1].compression,
                w.frame.pos.z);
        }

        {
            std::printf("A shove at rest\n");
            auto w = makeBike(spec);
            w.run(2.0f, dt, idle);
            w.angVel += w.frame.rot.axisY() * 1.2f; // a push that would lean it over by itself
            float peak = 0.0f;
            w.run(0.6f, dt, idle, [&](TestWorld& tw) {
                peak = (std::max)(peak, std::abs(rollOf(tw)));
                return false;
            });
            w.run(1.4f, dt, idle);
            check(peak < 22.0f * DEG, "the shove leans it no more than 22 deg (deg)", peak / DEG);
            check(std::abs(rollOf(w)) < 1.0f * DEG, "upright 2 s after the shove (roll deg)", rollOf(w) / DEG);
        }

        {
            std::printf("Parked on a 15%% slope\n");
            auto w = makeBike(spec);
            const float slope = std::atan(0.15f);
            w.groundNormal = normalizeOr(Vec3{ 0, -std::sin(slope), std::cos(slope) }, { 0, 0, 1 });
            w.frame.rot = Quat::fromAxisAngle({ 1, 0, 0 }, slope);
            w.frame.pos = { 0, 0, 0.05f };
            w.run(2.0f, dt, idle);
            const Vec3 p0 = w.frame.pos;
            w.run(5.0f, dt, idle);
            check(length(w.frame.pos - p0) < 0.05f, "does not slide (m over 5 s)", length(w.frame.pos - p0));
            check(std::abs(rollOf(w)) < 1.0f * DEG, "upright on the slope (roll deg)", rollOf(w) / DEG);
        }

        {
            std::printf("Acceleration and top speed\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            const float start = w.time;
            float t100 = -1.0f;
            float maxPitch = 0.0f;
            float maxRoll = 0.0f;
            float frontAir = 0.0f;
            w.run(
                60.0f,
                dt,
                [](TestWorld&) {
                    return DriverInput{ 1.0f, 0.0f, 0.0f, false };
                },
                [&](TestWorld& tw) {
                    if (t100 < 0.0f && tw.speed() >= 100.0f / 3.6f) {
                        t100 = tw.time - start;
                    }
                    maxPitch = (std::max)(maxPitch, std::abs(pitchOf(tw)));
                    maxRoll = (std::max)(maxRoll, std::abs(rollOf(tw)));
                    if (!tw.model.telemetry().wheels[0].grounded) {
                        frontAir += tw.time > start ? 1.0f / rate : 0.0f;
                    }
                    return false;
                });
            check(t100 > 3.6f && t100 < 6.5f, "0-100 km/h in about 5 s (s)", t100);
            const float vmax = w.speed() * 3.6f;
            check(vmax > 150.0f && vmax < 175.0f, "top speed about 160 km/h (km/h)", vmax);
            check(std::abs(w.frame.pos.x) < 1.0f, "runs straight (lateral drift m)", w.frame.pos.x);
            check(maxRoll < 1.0f * DEG, "stays upright flat out (max roll deg)", maxRoll / DEG);
            check(maxPitch < 14.0f * DEG, "no wheelie beyond 14 deg (max pitch deg)", maxPitch / DEG);
            check(frontAir < 0.6f, "front wheel in the air less than 0.6 s (s)", frontAir);

            std::printf("Braking from 100 km/h\n");
            w.run(30.0f, dt, idle, [](TestWorld& tw) {
                return tw.speed() <= 100.0f / 3.6f;
            });
            const Vec3 p0 = w.frame.pos;
            float rearAir = 0.0f;
            float brakePitch = 0.0f;
            float brakeRoll = 0.0f;
            w.run(
                15.0f,
                dt,
                [](TestWorld&) {
                    return DriverInput{ 0.0f, 1.0f, 0.0f, false };
                },
                [&](TestWorld& tw) {
                    if (!tw.model.telemetry().wheels[1].grounded) {
                        rearAir += 1.0f / rate;
                    }
                    brakePitch = (std::max)(brakePitch, std::abs(pitchOf(tw)));
                    brakeRoll = (std::max)(brakeRoll, std::abs(rollOf(tw)));
                    return tw.speed() < 0.05f;
                });
            const float dist = length(w.frame.pos - p0);
            check(dist > 30.0f && dist < 48.0f, "100-0 km/h in about 38 m (m)", dist);
            check(rearAir < 0.15f, "rear wheel stays down braking (s in the air)", rearAir);
            check(brakePitch < 9.0f * DEG, "no stoppie (max pitch deg)", brakePitch / DEG);
            check(brakeRoll < 1.5f * DEG, "upright while braking (max roll deg)", brakeRoll / DEG);
        }

        {
            std::printf("Cornering at 60 km/h\n");
            for (const float side : { 1.0f, -1.0f }) {
                auto w = makeBike(spec);
                w.run(1.0f, dt, idle);
                w.run(20.0f, dt, [](TestWorld& tw) { return cruise(tw, 60.0f, 0.0f); }, [](TestWorld& tw) { return tw.speed() >= 60.0f / 3.6f; });
                float latSum = 0.0f;
                float maxLat = 0.0f;
                int latCount = 0;
                const int window = static_cast<int>(0.5f / dt);
                Vec3 lastVel = w.linVel;
                float maxLean = 0.0f;
                float worstTrack = 0.0f;
                float maxSideslip = 0.0f;
                float maxRate = 0.0f;
                const float t0 = w.time;
                w.run(
                    6.0f,
                    dt,
                    [&](TestWorld& tw) { return cruise(tw, 60.0f, side); },
                    [&](TestWorld& tw) {
                        const float vLat = dot(tw.linVel, tw.frame.rot.axisX());
                        const float vLong = dot(tw.linVel, tw.frame.rot.axisY());
                        maxSideslip = (std::max)(maxSideslip, std::abs(std::atan2(vLat, (std::max)(vLong, 0.1f))));
                        const Vec3 accel = (tw.linVel - lastVel) / dt;
                        lastVel = tw.linVel;
                        // Horizontal acceleration across the heading (a leaned bike's own right axis is tilted).
                        const Vec3 heading = tw.frame.rot.axisY();
                        const Vec3 across = normalizeOr(Vec3{ heading.y, -heading.x, 0.0f }, { 1, 0, 0 });
                        latSum += std::abs(dot(accel, across));
                        if (++latCount == window) {
                            maxLat = (std::max)(maxLat, latSum / static_cast<float>(window));
                            latSum = 0.0f;
                            latCount = 0;
                        }
                        maxLean = (std::max)(maxLean, std::abs(rollOf(tw)));
                        maxRate = (std::max)(maxRate, std::abs(rollRateOf(tw)));
                        if (tw.time - t0 > 3.0f) {
                            worstTrack = (std::max)(worstTrack, std::abs(rollOf(tw) - tw.model.telemetry().leanTarget));
                        }
                        return false;
                    });
                const float roll = rollOf(w);
                std::printf("    steer %+.0f: lean %.1f deg (target %.1f), peak %.1f deg, lateral %.2f g, sideslip %.1f deg, peak roll rate %.0f deg/s\n",
                    side,
                    roll / DEG,
                    w.model.telemetry().leanTarget / DEG,
                    maxLean / DEG,
                    maxLat / GRAVITY,
                    maxSideslip / DEG,
                    maxRate / DEG);
                check(roll * side > 25.0f * DEG && roll * side < (spec.lean.maxLeanDeg + 3.0f) * DEG, "leans into the corner (deg)", roll / DEG);
                check(worstTrack < 5.0f * DEG, "follows its lean target within 5 deg after 3 s (deg)", worstTrack / DEG);
                check(maxLat / GRAVITY > 0.65f && maxLat / GRAVITY < 1.0f, "full stick corners at 0.65..1.0 g (g, 0.5 s average)", maxLat / GRAVITY);
                check(maxSideslip < 12.0f * DEG, "no spin-out, sideslip under 12 deg (deg)", maxSideslip / DEG);
                check(w.frame.rot.axisZ().z > 0.6f, "stays up (up.z)", w.frame.rot.axisZ().z);
            }
        }

        {
            std::printf("Full throttle and full stick (what a player does)\n");
            for (const float kmh : { 15.0f, 30.0f, 50.0f, 70.0f }) {
                for (const float side : { 1.0f, -1.0f }) {
                    auto w = makeBike(spec);
                    w.run(1.0f, dt, idle);
                    w.run(30.0f, dt, [&](TestWorld& tw) { return cruise(tw, kmh, 0.0f); }, [&](TestWorld& tw) { return tw.speed() >= kmh / 3.6f; });
                    float maxSideslip = 0.0f;
                    float minUp = 1.0f;
                    w.run(
                        5.0f,
                        dt,
                        [&](TestWorld&) { return DriverInput{ 1.0f, 0.0f, side, false }; },
                        [&](TestWorld& tw) {
                            const float vLat = dot(tw.linVel, tw.frame.rot.axisX());
                            const float vLong = dot(tw.linVel, tw.frame.rot.axisY());
                            maxSideslip = (std::max)(maxSideslip, std::abs(std::atan2(vLat, (std::max)(vLong, 0.1f))));
                            minUp = (std::min)(minUp, tw.frame.rot.axisZ().z);
                            return false;
                        });
                    std::printf("    from %.0f km/h, steer %+.0f: sideslip peak %.1f deg, min up.z %.2f, %.0f km/h after 5 s\n", kmh, side, maxSideslip / DEG, minUp, w.speed() * 3.6f);
                    check(maxSideslip < 25.0f * DEG, "does not spin out under full throttle and full stick (deg)", maxSideslip / DEG);
                    check(minUp > 0.6f, "stays up (up.z)", minUp);
                }
            }
        }

        {
            std::printf("Rays that miss the ground for a moment (cracked road)\n");
            auto w = makeBike(spec);
            w.run(2.0f, dt, idle);
            int rays = 0;
            w.rayMiss = [&](std::size_t, float) { return (++rays % 17) < 2; };
            const float t0 = w.time;
            int toggles = 0;
            int previous = 2;
            float maxVz = 0.0f;
            w.run(
                8.0f,
                dt,
                [&](TestWorld& tw) {
                    const float t = tw.time - t0;
                    return DriverInput{ tw.speed() * 3.6f > 35.0f ? 0.0f : (std::fmod(t, 1.0f) < 0.6f ? 1.0f : 0.0f), 0.0f, 0.0f, false };
                },
                [&](TestWorld& tw) {
                    toggles += grounded(tw) != previous ? 1 : 0;
                    previous = grounded(tw);
                    maxVz = (std::max)(maxVz, std::abs(tw.linVel.z));
                    return false;
                });
            check(toggles == 0, "a missed ray does not drop a wheel (contact changes)", toggles);
            check(maxVz < 0.1f, "the body does not bounce (max vertical speed m/s)", maxVz);
        }

        {
            std::printf("Slow full-lock turn at 5 km/h\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            float maxRoll = 0.0f;
            w.run(
                8.0f,
                dt,
                [](TestWorld& tw) {
                    return DriverInput{ tw.speed() < 5.0f / 3.6f ? 0.35f : 0.0f, 0.0f, 1.0f, false };
                },
                [&](TestWorld& tw) {
                    maxRoll = (std::max)(maxRoll, std::abs(rollOf(tw)));
                    return false;
                });
            const float radius = std::abs(w.speed()) / (std::max)(std::abs(w.yawRate()), 1e-4f);
            check(radius > 2.0f && radius < 6.5f, "turning circle radius at 5 km/h (m)", radius);
            check(w.yawRate() < 0.0f, "full right steer turns right (yaw rate rad/s)", w.yawRate());
            check(maxRoll < 8.0f * DEG, "barely leans at walking pace (max roll deg)", maxRoll / DEG);
        }

        // Rough ground and the inputs that lost the bike in the game (21 Oct): bumps in a corner, full throttle leaned over, a sudden full stick.
        const auto sideslipOf = [](const TestWorld& tw) {
            const Vec3 v = tw.linVel;
            const float along = dot(v, tw.frame.rot.axisY());
            const float across = dot(v, tw.frame.rot.axisX());
            return std::abs(along) > 1.0f ? std::atan2(std::abs(across), std::abs(along)) : 0.0f;
        };

        {
            std::printf("Bumps in a corner at 30 km/h\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            w.run(8.0f, dt, [](TestWorld& tw) { return cruise(tw, 30.0f, 0.0f); }, [](TestWorld& tw) { return tw.speed() * 3.6f > 29.0f; });
            w.run(2.0f, dt, [](TestWorld& tw) { return cruise(tw, 30.0f, 0.35f); });
            // Washboard: 8 cm bumps every 1.6 m, for 3 s.
            w.groundHeight = [](const float x, const float y) {
                return 0.04f * (1.0f - std::cos(2.0f * PI * std::sqrt(x * x + y * y) / 1.6f));
            };
            float minUp = 1.0f;
            float worstLeanError = 0.0f;
            float frontAir = 0.0f;
            float maxCompression = 0.0f;
            w.run(
                3.0f,
                dt,
                [](TestWorld& tw) { return cruise(tw, 30.0f, 0.35f); },
                [&](TestWorld& tw) {
                    const auto& t = tw.model.telemetry();
                    minUp = (std::min)(minUp, tw.frame.rot.axisZ().z);
                    worstLeanError = (std::max)(worstLeanError, std::abs(t.lean - t.leanTarget));
                    frontAir += t.wheels[0].grounded ? 0.0f : dt;
                    maxCompression = (std::max)(maxCompression, (std::max)(t.wheels[0].compression, t.wheels[1].compression));
                    return false;
                });
            std::printf("    min up.z %.2f, worst lean error %.1f deg, front wheel in the air %.0f%% of the time, max compression %.0f%% of the travel\n", minUp,
                worstLeanError / DEG, 100.0f * frontAir / 3.0f, 100.0f * maxCompression / spec.suspension.restLength);
            check(minUp > 0.6f, "stays up over bumps in a corner (up.z)", minUp);
            check(maxCompression < 0.97f * spec.suspension.restLength, "does not bottom out on the bumps (share of travel)", maxCompression / spec.suspension.restLength);
        }

        {
            std::printf("Full throttle leaned over at 30 km/h\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            w.run(8.0f, dt, [](TestWorld& tw) { return cruise(tw, 30.0f, 0.0f); }, [](TestWorld& tw) { return tw.speed() * 3.6f > 29.0f; });
            w.run(2.0f, dt, [](TestWorld& tw) { return cruise(tw, 30.0f, 0.8f); });
            const float leanBefore = rollOf(w);
            float maxSideslip = 0.0f;
            float maxRearSlip = 0.0f;
            float minUp = 1.0f;
            w.run(
                2.0f,
                dt,
                [](TestWorld&) { return DriverInput{ 1.0f, 0.0f, 0.8f, false }; },
                [&](TestWorld& tw) {
                    maxSideslip = (std::max)(maxSideslip, sideslipOf(tw));
                    maxRearSlip = (std::max)(maxRearSlip, std::abs(tw.model.telemetry().wheels[1].slipAngle));
                    minUp = (std::min)(minUp, tw.frame.rot.axisZ().z);
                    return false;
                });
            std::printf("    lean %.0f deg before the throttle; peak sideslip %.1f deg, rear tyre slip angle peak %.1f deg, min up.z %.2f\n", leanBefore / DEG, maxSideslip / DEG,
                maxRearSlip / DEG, minUp);
            check(minUp > 0.6f, "stays up with full throttle leaned over (up.z)", minUp);
        }

        {
            std::printf("Sudden full stick at 40 km/h\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            w.run(8.0f, dt, [](TestWorld& tw) { return cruise(tw, 40.0f, 0.0f); }, [](TestWorld& tw) { return tw.speed() * 3.6f > 39.0f; });
            float peakYaw = 0.0f;
            float peakRoll = 0.0f;
            float maxSideslip = 0.0f;
            float minUp = 1.0f;
            w.run(
                1.0f,
                dt,
                [](TestWorld& tw) { return cruise(tw, 40.0f, 1.0f); },
                [&](TestWorld& tw) {
                    peakYaw = (std::max)(peakYaw, std::abs(tw.yawRate()));
                    peakRoll = (std::max)(peakRoll, std::abs(rollOf(tw)));
                    maxSideslip = (std::max)(maxSideslip, sideslipOf(tw));
                    minUp = (std::min)(minUp, tw.frame.rot.axisZ().z);
                    return false;
                });
            w.run(
                2.0f,
                dt,
                [](TestWorld& tw) { return cruise(tw, 40.0f, 0.0f); },
                [&](TestWorld& tw) {
                    minUp = (std::min)(minUp, tw.frame.rot.axisZ().z);
                    maxSideslip = (std::max)(maxSideslip, sideslipOf(tw));
                    return false;
                });
            std::printf("    peak yaw rate %.0f deg/s, peak lean %.0f deg, peak sideslip %.1f deg, min up.z %.2f, upright again %.1f deg\n", peakYaw / DEG, peakRoll / DEG,
                maxSideslip / DEG, minUp, std::abs(rollOf(w)) / DEG);
            check(minUp > 0.6f, "stays up after a sudden full stick (up.z)", minUp);
        }

        {
            // What the game log showed before the falls: 14-15 km/h, full throttle and a lot of stick (rows 1276-1330 of the 21 Oct CSV).
            std::printf("Full throttle and full stick at 15 km/h\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            w.run(8.0f, dt, [](TestWorld& tw) { return cruise(tw, 15.0f, 0.0f); }, [](TestWorld& tw) { return tw.speed() * 3.6f > 14.5f; });
            float peakYaw = 0.0f;
            float peakRoll = 0.0f;
            float maxRearSlip = 0.0f;
            float minUp = 1.0f;
            w.run(
                3.0f,
                dt,
                [](TestWorld&) { return DriverInput{ 1.0f, 0.0f, 1.0f, false }; },
                [&](TestWorld& tw) {
                    peakYaw = (std::max)(peakYaw, std::abs(tw.yawRate()));
                    peakRoll = (std::max)(peakRoll, std::abs(rollOf(tw)));
                    maxRearSlip = (std::max)(maxRearSlip, std::abs(tw.model.telemetry().wheels[1].slipAngle));
                    minUp = (std::min)(minUp, tw.frame.rot.axisZ().z);
                    return false;
                });
            std::printf("    peak yaw rate %.0f deg/s, peak lean %.0f deg, rear tyre slip angle peak %.1f deg, min up.z %.2f\n", peakYaw / DEG, peakRoll / DEG,
                maxRearSlip / DEG, minUp);
            check(minUp > 0.6f, "stays up with full throttle and full stick at 15 km/h (up.z)", minUp);
        }

        {
            // Rocky ground: irregular bumps up to 10 cm (two crossing waves), straight at 35 km/h.
            std::printf("Rocky ground at 35 km/h\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            w.run(8.0f, dt, [](TestWorld& tw) { return cruise(tw, 35.0f, 0.0f); }, [](TestWorld& tw) { return tw.speed() * 3.6f > 34.0f; });
            w.groundHeight = [](const float x, const float y) {
                return 0.025f * (1.0f - std::cos(2.0f * PI * y / 1.3f)) + 0.025f * (1.0f - std::cos(2.0f * PI * (0.7f * y + 0.7f * x) / 0.9f));
            };
            float minUp = 1.0f;
            float airborne = 0.0f;
            float maxCompression = 0.0f;
            float maxRoll = 0.0f;
            w.run(
                4.0f,
                dt,
                [](TestWorld& tw) { return cruise(tw, 35.0f, 0.0f); },
                [&](TestWorld& tw) {
                    const auto& t = tw.model.telemetry();
                    minUp = (std::min)(minUp, tw.frame.rot.axisZ().z);
                    airborne += (t.wheels[0].grounded ? 0.0f : dt) + (t.wheels[1].grounded ? 0.0f : dt);
                    maxCompression = (std::max)(maxCompression, (std::max)(t.wheels[0].compression, t.wheels[1].compression));
                    maxRoll = (std::max)(maxRoll, std::abs(rollOf(tw)));
                    return false;
                });
            std::printf("    min up.z %.2f, max roll %.1f deg, wheels in the air %.0f%% of the time, max compression %.0f%% of the travel\n", minUp, maxRoll / DEG,
                100.0f * airborne / 8.0f, 100.0f * maxCompression / spec.suspension.restLength);
            check(minUp > 0.6f, "stays up on rocky ground (up.z)", minUp);
            check(maxCompression < 0.97f * spec.suspension.restLength, "does not bottom out on rocky ground (share of travel)", maxCompression / spec.suspension.restLength);
        }

        {
            // What Axel saw: the bike comes down front wheel first (nose down) and goes over. A jump: lifted 0.8 m at 35 km/h, nose 15 deg down.
            std::printf("Lands front wheel first after a 0.8 m jump at 35 km/h\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            w.run(8.0f, dt, [](TestWorld& tw) { return cruise(tw, 35.0f, 0.0f); }, [](TestWorld& tw) { return tw.speed() * 3.6f > 34.0f; });
            w.frame.pos.z += 0.8f;
            w.frame.rot = (Quat::fromAxisAngle(w.frame.rot.axisX(), -15.0f * DEG) * w.frame.rot).normalized();
            w.angVel = {};
            w.linVel.z = 0.0f;
            float maxNoseDown = 0.0f;
            float peakYaw = 0.0f;
            float minUp = 1.0f;
            float rearAirAfterLanding = 0.0f;
            bool landed = false;
            w.run(
                3.0f,
                dt,
                [](TestWorld& tw) { return cruise(tw, 35.0f, 0.0f); },
                [&](TestWorld& tw) {
                    const auto& t = tw.model.telemetry();
                    maxNoseDown = (std::max)(maxNoseDown, -pitchOf(tw));
                    peakYaw = (std::max)(peakYaw, std::abs(tw.yawRate()));
                    minUp = (std::min)(minUp, tw.frame.rot.axisZ().z);
                    landed = landed || t.groundedWheels > 0;
                    rearAirAfterLanding += landed && !t.wheels[1].grounded ? dt : 0.0f;
                    return false;
                });
            std::printf("    max nose-down pitch %.1f deg, peak yaw rate %.0f deg/s, min up.z %.2f, rear wheel in the air after landing %.2f s, %.0f km/h after\n",
                maxNoseDown / DEG, peakYaw / DEG, minUp, rearAirAfterLanding, w.speed() * 3.6f);
            check(minUp > 0.6f, "stays up landing front wheel first (up.z)", minUp);
        }

        {
            std::printf("Hard braking downhill (15%%) from 40 km/h\n");
            auto w = makeBike(spec);
            const float slope = std::atan(0.15f);
            // Downhill: the ground falls away along +y.
            w.groundNormal = normalizeOr(Vec3{ 0, std::sin(slope), std::cos(slope) }, { 0, 0, 1 });
            w.frame.rot = Quat::fromAxisAngle({ 1, 0, 0 }, -slope);
            w.frame.pos = { 0, 0, 0.05f };
            w.run(1.0f, dt, idle);
            w.run(8.0f, dt, [](TestWorld& tw) { return cruise(tw, 40.0f, 0.0f); }, [](TestWorld& tw) { return tw.speed() * 3.6f > 39.0f; });
            const float slopePitch = pitchOf(w);
            float maxNoseDown = 0.0f;
            float rearAir = 0.0f;
            float peakYaw = 0.0f;
            float minUp = 1.0f;
            w.run(
                4.0f,
                dt,
                [](TestWorld&) { return DriverInput{ 0.0f, 1.0f, 0.0f, false }; },
                [&](TestWorld& tw) {
                    maxNoseDown = (std::max)(maxNoseDown, slopePitch - pitchOf(tw));
                    rearAir += tw.model.telemetry().wheels[1].grounded ? 0.0f : dt;
                    peakYaw = (std::max)(peakYaw, std::abs(tw.yawRate()));
                    minUp = (std::min)(minUp, tw.frame.rot.axisZ().z);
                    return std::abs(tw.speed()) < 0.3f;
                });
            std::printf("    nose-down pitch beyond the slope %.1f deg, rear wheel in the air %.2f s, peak yaw rate %.0f deg/s, min up.z %.2f\n", maxNoseDown / DEG, rearAir,
                peakYaw / DEG, minUp);
            check(minUp > 0.6f, "stays up braking hard downhill (up.z)", minUp);
            check(rearAir < 0.3f, "no stoppie braking hard downhill (rear wheel in the air s)", rearAir);
        }

        {
            std::printf("Lane change at 80 km/h\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            w.run(30.0f, dt, [](TestWorld& tw) { return cruise(tw, 80.0f, 0.0f); }, [](TestWorld& tw) { return tw.speed() >= 80.0f / 3.6f; });
            float maxRate = 0.0f;
            float maxRoll = 0.0f;
            const float t0 = w.time;
            w.run(
                2.4f,
                dt,
                [&](TestWorld& tw) {
                    const float t = tw.time - t0;
                    const float steer = t < 0.8f ? 0.5f : (t < 1.6f ? -0.5f : 0.0f);
                    return cruise(tw, 80.0f, steer);
                },
                [&](TestWorld& tw) {
                    maxRate = (std::max)(maxRate, std::abs(rollRateOf(tw)));
                    maxRoll = (std::max)(maxRoll, std::abs(rollOf(tw)));
                    return false;
                });
            float settle = 0.0f;
            w.run(
                2.5f,
                dt,
                [](TestWorld& tw) { return cruise(tw, 80.0f, 0.0f); },
                [&](TestWorld& tw) {
                    if (tw.time - t0 > 4.4f) {
                        settle = (std::max)(settle, std::abs(rollOf(tw)));
                    }
                    return false;
                });
            std::printf("    peak lean %.1f deg, peak roll rate %.0f deg/s\n", maxRoll / DEG, maxRate / DEG);
            check(maxRate < 130.0f * DEG, "roll rate under 130 deg/s (deg/s)", maxRate / DEG);
            check(settle < 2.0f * DEG, "upright again 2 s after the last steer (max roll deg)", settle / DEG);
        }

        {
            std::printf("Reverse\n");
            auto w = makeBike(spec);
            w.run(1.0f, dt, idle);
            float maxRoll = 0.0f;
            w.run(
                4.0f,
                dt,
                [](TestWorld&) {
                    return DriverInput{ 0.0f, 0.6f, 0.0f, false };
                },
                [&](TestWorld& tw) {
                    maxRoll = (std::max)(maxRoll, std::abs(rollOf(tw)));
                    return false;
                });
            check(w.speed() < -1.0f, "holding the brake at a standstill reverses (m/s)", w.speed());
            check(maxRoll < 1.5f * DEG, "upright in reverse (max roll deg)", maxRoll / DEG);
        }

        {
            std::printf("A bike that is down stays down\n");
            auto w = makeBike(spec);
            w.frame.rot = Quat::fromAxisAngle({ 0, 1, 0 }, 85.0f * DEG);
            w.frame.pos = { 0, 0, 0.5f };
            w.run(1.5f, dt, idle);
            check(w.frame.rot.axisZ().z < 0.35f, "the stabiliser lets go of a fallen bike (up.z)", w.frame.rot.axisZ().z);
            const auto& t = w.model.telemetry();
            check(std::abs(t.leanTarget) < 1e-3f, "no lean target while lying (rad)", t.leanTarget);
        }

        {
            std::printf("Seat camera follows part of the lean\n");
            Pose leaned;
            leaned.rot = Quat::fromAxisAngle({ 0, 1, 0 }, 30.0f * DEG);
            const Pose half = scaleRoll(leaned, 0.5f);
            const float rollHalf = std::atan2(-half.rot.axisX().z, half.rot.axisZ().z);
            check(std::abs(rollHalf - 15.0f * DEG) < 0.1f * DEG, "half the lean (deg)", rollHalf / DEG);
            const Pose none = scaleRoll(leaned, 0.0f);
            check(std::abs(std::atan2(-none.rot.axisX().z, none.rot.axisZ().z)) < 0.1f * DEG, "none of the lean (deg)", 0.0f);
            Pose yawed;
            yawed.rot = Quat::fromAxisAngle({ 0, 0, 1 }, -0.8f) * Quat::fromAxisAngle({ 0, 1, 0 }, 20.0f * DEG);
            const Pose y2 = scaleRoll(yawed, 0.5f);
            check(std::abs(headingOf(y2.rot.axisY()) - headingOf(yawed.rot.axisY())) < 0.5f * DEG, "the heading is untouched (deg)", (headingOf(y2.rot.axisY()) - headingOf(yawed.rot.axisY())) / DEG);
        }
    }

    if (argc > 2) {
        FILE* f = std::fopen(argv[2], "w");
        if (!f) {
            std::printf("cannot write %s\n", argv[2]);
            return 2;
        }
        std::fprintf(f, "t,speed_kmh,steer_deg,roll_deg,target_deg,pitch_deg,yaw_rate,throttle,brake,front_comp,rear_comp,front_fy,rear_fy\n");
        auto w = makeBike(spec);
        const float dt = 1.0f / 90.0f;
        const auto row = [&](TestWorld& tw) {
            const auto& t = tw.model.telemetry();
            std::fprintf(f, "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.3f,%.0f,%.0f\n", tw.time, tw.speed() * 3.6f, t.steerAngle / DEG, rollOf(tw) / DEG, t.leanTarget / DEG, pitchOf(tw) / DEG,
                tw.yawRate(), t.throttle, t.brake, t.wheels[0].compression, t.wheels[1].compression, t.wheels[0].fy, t.wheels[1].fy);
            return false;
        };
        w.run(2.0f, dt, idle, row);
        w.run(30.0f, dt, [](TestWorld& tw) { return cruise(tw, 70.0f, 0.0f); }, [&](TestWorld& tw) { row(tw); return tw.speed() >= 70.0f / 3.6f; });
        // S bends: right, left, right at 70 km/h.
        const float t0 = w.time;
        w.run(9.0f, dt, [&](TestWorld& tw) {
            const float t = tw.time - t0;
            const float steer = t < 2.5f ? 0.5f : (t < 5.5f ? -0.5f : (t < 8.0f ? 0.5f : 0.0f));
            return cruise(tw, 70.0f, steer);
        }, row);
        w.run(5.0f, dt, [](TestWorld&) { return DriverInput{ 0.0f, 1.0f, 0.0f, false }; }, [&](TestWorld& tw) { row(tw); return tw.speed() < 0.05f; });
        std::fclose(f);
        std::printf("\nLap written to %s\n", argv[2]);
    }

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
