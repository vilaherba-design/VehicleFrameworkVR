// Offline tests for the vehicle model: no game, no Havok. A small rigid-body integrator stands in for
// Havok (gravity + the model's impulses, flat or tilted ground) so the reference profile can be checked
// against the design targets before anyone puts on a headset:
//   0-100 km/h about 7 s, top speed about 150 km/h, 100-0 braking about 45 m, ~0.9 g lateral grip,
//   parked on a slope it does not slide, at rest it does not vibrate, and it turns at walking pace.
//
// Usage: VehicleModelTests [path/to/sedan.json] [drive.csv]   (exit code 0 = all passed)
// With a second argument it also drives a scripted test lap and writes it in the plugin's telemetry
// format (VehicleFrameworkVR_physics.csv), which tools/analyze_telemetry.py reads.

#include "TestWorld.h"
#include "model/CameraArm.h"
#include "model/MenuGrid.h"
#include "model/MenuPointer.h"
#include "model/ProfileList.h"
#include "model/SeatBody.h"
#include "model/SteeringWheelSpin.h"
#include "model/TrunkLoad.h"

int main(const int argc, char** argv)
{
    const char* path = argc > 1 ? argv[1] : nullptr;
    VehicleSpec spec;
    try {
        spec = loadSpec(path);
    } catch (const std::exception& e) {
        std::printf("Profile load failed: %s\n", e.what());
        return 2;
    }

    // The stock profile, and every wheel set the profile offers (the big-wheel sets change the tyres, the ride height, the mass and the drive).
    struct Variant
    {
        std::string name;
        VehicleSpec spec;
        bool offroad;
    };

    std::vector<Variant> variants;
    variants.push_back({ "stock", spec, false });
    if (path) {
        for (const auto& part : spec.parts) {
            if (part.slot != "wheels" || part.id == "stock") {
                continue;
            }
            try {
                variants.push_back({ part.id + " (" + part.name + ")", loadVehicleSpecFile(path, { { "wheels", part.id } }), true });
            } catch (const std::exception& e) {
                std::printf("Profile with the %s wheels failed: %s\n", part.id.c_str(), e.what());
                return 2;
            }
        }
    }
    const VehicleSpec stockSpec = spec;

    {
        std::printf("\n=== Door handles ===\n");
        const auto handles = spec.doorHandlePoints();
        check(!handles.empty(), "at least one door handle (count)", static_cast<double>(handles.size()));
        // The nearest handle is the driver's own; one on the far side of a car is the car's width away.
        float nearest = 1e9f;
        float farthest = 0.0f;
        for (const auto& h : handles) {
            nearest = (std::min)(nearest, length(h - spec.driverEye));
            farthest = (std::max)(farthest, length(h - spec.driverEye));
        }
        check(nearest < 1.5f, "a door handle within 1.5 m of the driver's eyes (m)", nearest);
        check(farthest < 3.0f, "every door handle within 3 m of the driver's eyes (m)", farthest);
        check(spec.interactRadius > 0.2f && spec.interactRadius < 2.0f, "interact radius 0.2..2 m (m)", spec.interactRadius);
    }

    {
        std::printf("\n=== Body in the seat (third person) ===\n");
        // The head at the chase camera 11 m behind and 3.5 m above the seat, the body FRIK built facing +x (heading 90 deg); the car faces +y.
        const Vec3 seat{ -0.6f, 0.2f, 1.4f };
        const Vec3 head{ -0.6f - 11.0f, 0.2f, 1.4f + 3.5f };
        const float bodyHeading = PI * 0.5f;
        const Pose level{ seat, Quat{} };
        const Pose move = seatBodyTransform(level, head, bodyHeading);
        check(length(move.toWorld(head) - seat) < 1e-4f, "the head lands on the seat's eye point (m)", length(move.toWorld(head) - seat));
        const Vec3 bodyForward{ std::sin(bodyHeading), std::cos(bodyHeading), 0.0f };
        check(length(move.rot.rotate(bodyForward) - Vec3{ 0.0f, 1.0f, 0.0f }) < 1e-4f, "the body faces the car's front (m)",
            length(move.rot.rotate(bodyForward) - Vec3{ 0.0f, 1.0f, 0.0f }));
        const Vec3 feet = move.toWorld(head + Vec3{ 0.0f, 0.0f, -1.6f });
        check(length(feet - (seat + Vec3{ 0.0f, 0.0f, -1.6f })) < 1e-4f, "the feet stay under the head (m)", length(feet - (seat + Vec3{ 0.0f, 0.0f, -1.6f })));
        const Vec3 hand = move.toWorld(head + Vec3{ 0.3f, 0.4f, -0.5f });
        check(std::abs(length(hand - seat) - length(Vec3{ 0.3f, 0.4f, -0.5f })) < 1e-4f, "the body keeps its shape (hand-to-head distance m)", length(hand - seat));
        // The car on its roof: the body stays in the seat, head down with it.
        const Pose upsideDown{ seat, Quat::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, PI) };
        const Vec3 feetFlipped = seatBodyTransform(upsideDown, head, bodyHeading).toWorld(head + Vec3{ 0.0f, 0.0f, -1.6f });
        check(feetFlipped.z > seat.z + 1.5f, "on its roof the feet are above the head, still in the car (feet z m)", feetFlipped.z);
        // Seated legs, in the vehicle frame: the thigh about level and forward, the shin down, the knees a little apart.
        const Vec3 thigh = seatedThighLocal(80.0f, 8.0f, 1.0f);
        const Vec3 shin = seatedShinLocal(80.0f, 90.0f, 8.0f, 1.0f);
        check(thigh.y > 0.95f && thigh.z < 0.0f && thigh.z > -0.2f && thigh.x > 0.0f, "the thigh points forward, 10 deg below level, splayed out (forward share)", thigh.y);
        check(shin.z < -0.95f && shin.x > 0.0f, "the shin hangs down from the knee, still splayed out (down share)", shin.z);
        check(std::abs(std::acos(std::clamp(dot(thigh, shin), -1.0f, 1.0f)) - 0.5f * PI) < 0.05f, "the knee is bent about 90 deg (rad from 90)",
            std::acos(std::clamp(dot(thigh, shin), -1.0f, 1.0f)) - 0.5f * PI);
        const Quat q = rotationBetween({ 0.0f, 0.0f, -1.0f }, thigh);
        check(length(q.rotate({ 0.0f, 0.0f, -1.0f }) - thigh) < 1e-4f, "the hip bend turns the standing thigh onto the seated one (m)", length(q.rotate({ 0.0f, 0.0f, -1.0f }) - thigh));
    }

    {
        std::printf("\n=== Chase camera arm ===\n");
        // An 11.5 m arm (11 m behind, 3.5 m up from the pivot), margin 0.3 m, at least 2 m.
        check(std::abs(cameraArmTarget(11.5f, -1.0f, 0.3f, 2.0f) - 11.5f) < 1e-4f, "nothing in the way: the full arm (m)", cameraArmTarget(11.5f, -1.0f, 0.3f, 2.0f));
        check(std::abs(cameraArmTarget(11.5f, 6.0f, 0.3f, 2.0f) - 5.7f) < 1e-4f, "a wall at 6 m: 0.3 m in front of it (m)", cameraArmTarget(11.5f, 6.0f, 0.3f, 2.0f));
        check(std::abs(cameraArmTarget(11.5f, 1.0f, 0.3f, 2.0f) - 2.0f) < 1e-4f, "a wall at 1 m: never closer than 2 m (m)", cameraArmTarget(11.5f, 1.0f, 0.3f, 2.0f));
        check(std::abs(cameraArmTarget(1.5f, 0.5f, 0.3f, 2.0f) - 1.5f) < 1e-4f, "an arm shorter than the minimum keeps its length (m)", cameraArmTarget(1.5f, 0.5f, 0.3f, 2.0f));
        // A wall appears: the camera comes in within a few frames; it clears: the camera goes back out over about a second.
        float arm = 11.5f;
        for (int i = 0; i < 9; ++i) {
            arm = smoothCameraArm(arm, 5.7f, 1.0f / 90.0f, 0.05f, 0.6f);
        }
        check(arm < 7.5f, "comes in fast: after 0.1 s (m)", arm);
        for (int i = 0; i < 9; ++i) {
            arm = smoothCameraArm(arm, 11.5f, 1.0f / 90.0f, 0.05f, 0.6f);
        }
        check(arm < 8.0f, "goes back out slowly: 0.1 s after the wall clears (m)", arm);
        for (int i = 0; i < 360; ++i) {
            arm = smoothCameraArm(arm, 11.5f, 1.0f / 90.0f, 0.05f, 0.6f);
        }
        check(std::abs(arm - 11.5f) < 0.05f, "and is back at the full arm after 4 s (m)", arm);
        // The ray skips the mods' own layers (48 and up: ROCK's hands and body, FRIK) and still stops at the world: static 1, trees 9, props 10,
        // terrain 13, ground 17.
        const std::uint64_t mask = cameraIgnoreMask(0);
        bool worldSeen = true;
        for (const int layer : { 1, 9, 10, 13, 17 }) {
            worldSeen = worldSeen && (mask & (1ull << layer)) == 0;
        }
        bool modsSkipped = true;
        for (const int layer : { 48, 51, 56, 63 }) {
            modsSkipped = modsSkipped && (mask & (1ull << layer)) != 0;
        }
        check(worldSeen, "walls, rocks, trees and the ground still stop the camera (all seen)", worldSeen ? 1.0 : 0.0);
        check(modsSkipped, "the mods' layers 48..63 (layer51, layer56) are skipped (all skipped)", modsSkipped ? 1.0 : 0.0);
        check(cameraIgnoreMask(1ull << 30) & (1ull << 30), "the INI's own ignored layers are kept (kept)", 1.0);
    }

    {
        std::printf("\n=== Trunk ===\n");
        if (path) {
            check(spec.hasTrunk, "the profile has a trunk", spec.hasTrunk ? 1.0 : 0.0);
            check(spec.trunkCapacity > 0.0f, "trunk capacity (carry weight)", spec.trunkCapacity);
            check(length(spec.trunkPoint - spec.driverEye) < 6.0f, "trunk point within 6 m of the driver's eyes (m)", length(spec.trunkPoint - spec.driverEye));
        }
        try {
            const auto none = parseVehicleSpec("{}");
            check(!none.hasTrunk, "no \"trunk\" section, no trunk", none.hasTrunk ? 1.0 : 0.0);
            const auto some = parseVehicleSpec(R"({ "trunk": { "point": [0, -70, 35], "capacity": 250 } })");
            check(some.hasTrunk && std::abs(some.trunkCapacity - 250.0f) < 1e-3f && std::abs(some.trunkPoint.y + 1.0f) < 0.01f, "trunk section parsed (point y m)",
                some.trunkPoint.y);
        } catch (const std::exception& e) {
            check(false, std::string("trunk JSON: ") + e.what(), 0.0);
        }

        // 10 ammo (0.1 each), then 3 rifles (10 each), then 2 helmets (4 each): 39 in a trunk that already held 70, capacity 100.
        std::vector<TrunkAddition> added;
        recordTrunkChange(added, 1, 10, 0.1f);
        recordTrunkChange(added, 2, 3, 10.0f);
        recordTrunkChange(added, 3, 2, 4.0f);
        auto back = trunkReturns(added, 109.0f, 100.0f);
        check(back.size() == 2 && back[0].formId == 3 && back[0].count == 2 && back[1].formId == 2 && back[1].count == 1,
            "over by 9: both helmets, then one rifle go back (items)", back.empty() ? 0.0 : static_cast<double>(back[0].count + (back.size() > 1 ? back[1].count : 0)));
        check(trunkReturns(added, 100.0f, 100.0f).empty(), "at the capacity nothing goes back (items)", static_cast<double>(trunkReturns(added, 100.0f, 100.0f).size()));
        recordTrunkChange(added, 3, -2, 0.0f);
        back = trunkReturns(added, 101.0f, 100.0f);
        check(back.size() == 1 && back[0].formId == 2 && back[0].count == 1, "helmets taken out again: a rifle goes back (items)", back.empty() ? 0.0 : back[0].count);
        std::vector<TrunkAddition> weightless{ { 4, 5, 0.0f } };
        check(trunkReturns(weightless, 120.0f, 100.0f).empty(), "weightless items never go back (items)", static_cast<double>(trunkReturns(weightless, 120.0f, 100.0f).size()));
        back = trunkReturns({ { 5, 100, 0.5f } }, 101.2f, 100.0f);
        check(back.size() == 1 && back[0].count == 3, "0.5-weight items over by 1.2: three go back (items)", back.empty() ? 0.0 : back[0].count);
    }

    for (const auto& variant : variants)
    for (const float rate : { 90.0f, 72.0f }) {
        const VehicleSpec& spec = variant.spec;
        const float dt = 1.0f / rate;
        std::printf("\n=== %s, physics rate %.0f Hz ===\n", variant.name.c_str(), rate);

        {
            std::printf("Math\n");
            const Quat q = Quat::fromAxisAngle({ 0, 0, 1 }, 0.7f);
            const Quat back = Quat::fromBasis(q.axisX(), q.axisY(), q.axisZ());
            const float err = std::abs(dot(back.rotate({ 0.3f, 1.0f, -0.2f }), { 1, 0, 0 }) - dot(q.rotate({ 0.3f, 1.0f, -0.2f }), { 1, 0, 0 }));
            check(err < 1e-5f, "quaternion from basis round trip", err);
            check(std::abs(headingOf(Quat::fromAxisAngle({ 0, 0, 1 }, -0.5f).axisY()) - 0.5f) < 1e-5f,
                "heading of a right turn is positive",
                headingOf(Quat::fromAxisAngle({ 0, 0, 1 }, -0.5f).axisY()));
        }

        {
            std::printf("Settle at rest\n");
            auto w = makeWorld(spec);
            w.run(2.0f, dt, idle);
            const float z0 = w.frame.pos.z;
            float maxSpeed = 0.0f;
            const Vec3 p0 = w.frame.pos;
            w.run(3.0f, dt, idle, [&](TestWorld& tw) {
                maxSpeed = (std::max)(maxSpeed, length(tw.linVel));
                return false;
            });
            check(maxSpeed < 0.02f, "no vibration at rest (max speed m/s)", maxSpeed);
            check(length(w.frame.pos - p0) < 0.01f, "no creep at rest (m over 3 s)", length(w.frame.pos - p0));
            check(std::abs(w.frame.pos.z - z0) < 0.005f, "ride height stable (m)", w.frame.pos.z - z0);
            const auto& t = w.model.telemetry();
            check(t.groundedWheels == 4, "four wheels on the ground", t.groundedWheels);
            check(t.wheels[0].compression > 0.05f && t.wheels[0].compression < 0.25f, "static compression in travel (m)", t.wheels[0].compression);
            // The body rides above the mesh's own ground line (the wheel meshes then have room to move up inside the arches): ~11 game units
            // (0.16 m) on the original wheels, and the taller 4x4 tyres raise it by at least 10 cm more.
            const float rideHeight = w.frame.toWorld({ 0, 0, 0 }).z;
            if (variant.offroad) {
                check(rideHeight > 0.26f && rideHeight < 0.36f, "4x4: body rides 0.26..0.36 m above the mesh ground line (m)", rideHeight);
                auto stockWorld = makeWorld(stockSpec);
                stockWorld.run(5.0f, dt, idle);
                const float stockHeight = stockWorld.frame.toWorld({ 0, 0, 0 }).z;
                check(rideHeight - stockHeight > 0.10f, "4x4 raises the chassis by more than 10 cm (m)", rideHeight - stockHeight);
            } else {
                check(rideHeight > 0.12f && rideHeight < 0.20f, "body rides 0.12..0.20 m above the mesh ground line (m)", rideHeight);
            }
            // The wheel mesh stays inside its arch: the top of the tyre (wheel centre z = hub + compression, plus the tyre radius) is below the
            // top of the arch (58.2 units in the mesh) and the wheel hangs low enough to leave a gap above it.
            const float wheelCentre = spec.wheels[0].hub.z + t.wheels[0].compression;
            const float tyreTop = (wheelCentre + spec.tyre.radius) * HAVOK_TO_GAME;
            check(tyreTop < 58.2f && tyreTop > 58.2f - 15.0f, "tyre top sits under the arch, with a gap of at most 15 units", tyreTop);
            // And at full bump the visual cap keeps it there: the highest the tyre may go.
            const float highestTop = 28.75f + spec.wheelMaxAboveMesh * HAVOK_TO_GAME + spec.tyre.radius * HAVOK_TO_GAME;
            check(highestTop <= 58.2f, "tyre top at full bump (visual cap) stays under the arch (units)", highestTop);
            if (variant.offroad) {
                int driven = 0;
                for (const auto& wheel : spec.wheels) {
                    driven += wheel.drive ? 1 : 0;
                }
                check(driven == 4, "4x4 wheels: four-wheel drive (driven wheels)", driven);
                check(spec.mass > stockSpec.mass && spec.tyre.radius > stockSpec.tyre.radius + 0.05f, "4x4 wheels: heavier, taller tyres (radius m)", spec.tyre.radius);
            }
        }

        {
            std::printf("Parked on a 15%% slope\n");
            auto w = makeWorld(spec);
            const float slope = std::atan(0.15f);
            w.groundNormal = normalizeOr(Vec3{ 0, -std::sin(slope), std::cos(slope) }, { 0, 0, 1 });
            // Start aligned with the slope, nose uphill.
            w.frame.rot = Quat::fromAxisAngle({ 1, 0, 0 }, slope);
            w.frame.pos = { 0, 0, 0.05f };
            w.run(2.0f, dt, idle);
            const Vec3 p0 = w.frame.pos;
            w.run(5.0f, dt, idle);
            check(length(w.frame.pos - p0) < 0.05f, "does not slide (m over 5 s)", length(w.frame.pos - p0));
        }

        {
            std::printf("Acceleration and top speed\n");
            auto w = makeWorld(spec);
            w.run(1.0f, dt, idle);
            const float start = w.time;
            float t100 = -1.0f;
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
                    return false;
                });
            check(t100 > 5.0f && t100 < 9.0f, "0-100 km/h in about 7 s (s)", t100);
            const float vmax = w.speed() * 3.6f;
            check(vmax > 140.0f && vmax < 158.0f, "top speed about 150 km/h (km/h)", vmax);
            check(std::abs(w.frame.pos.x) < 1.0f, "runs straight (lateral drift m)", w.frame.pos.x);

            std::printf("Braking from 100 km/h\n");
            // Coast down to exactly 100 km/h first.
            w.run(30.0f, dt, idle, [](TestWorld& tw) {
                return tw.speed() <= 100.0f / 3.6f;
            });
            const Vec3 p0 = w.frame.pos;
            w.run(
                15.0f,
                dt,
                [](TestWorld&) {
                    return DriverInput{ 0.0f, 1.0f, 0.0f, false };
                },
                [](TestWorld& tw) {
                    return tw.speed() < 0.05f;
                });
            const float dist = length(w.frame.pos - p0);
            check(dist > 36.0f && dist < 55.0f, "100-0 km/h in about 45 m (m)", dist);
        }

        {
            std::printf("Turning at walking pace\n");
            auto w = makeWorld(spec);
            w.run(1.0f, dt, idle);
            // Hold about 5 km/h with full steering.
            w.run(8.0f, dt, [](TestWorld& tw) {
                const float target = 5.0f / 3.6f;
                return DriverInput{ tw.speed() < target ? 0.35f : 0.0f, 0.0f, 1.0f, false };
            });
            const float radius = std::abs(w.speed()) / (std::max)(std::abs(w.yawRate()), 1e-4f);
            check(radius > 3.5f && radius < 9.0f, "turning circle radius at 5 km/h (m)", radius);
            check(w.yawRate() < 0.0f, "full right steer turns right (yaw rate rad/s)", w.yawRate());
        }

        {
            std::printf("Cornering at 60 km/h\n");
            auto w = makeWorld(spec);
            w.run(1.0f, dt, idle);
            w.run(
                20.0f,
                dt,
                [](TestWorld& tw) {
                    return DriverInput{ tw.speed() < 60.0f / 3.6f ? 1.0f : 0.15f, 0.0f, 0.0f, false };
                },
                [](TestWorld& tw) {
                    return tw.speed() >= 60.0f / 3.6f;
                });
            float maxSideslip = 0.0f;
            // Lateral acceleration, measured from the velocity change and averaged over half a second.
            float maxLat = 0.0f;
            Vec3 lastVel = w.linVel;
            float latSum = 0.0f;
            int latCount = 0;
            const int window = static_cast<int>(0.5f / dt);
            w.run(
                6.0f,
                dt,
                [](TestWorld& tw) {
                    return DriverInput{ tw.speed() < 60.0f / 3.6f ? 0.6f : 0.2f, 0.0f, 0.6f, false };
                },
                [&](TestWorld& tw) {
                    const float vLat = dot(tw.linVel, tw.frame.rot.axisX());
                    const float vLong = dot(tw.linVel, tw.frame.rot.axisY());
                    maxSideslip = (std::max)(maxSideslip, std::abs(std::atan2(vLat, (std::max)(vLong, 0.1f))));
                    const Vec3 accel = (tw.linVel - lastVel) / dt;
                    lastVel = tw.linVel;
                    latSum += std::abs(dot(accel, tw.frame.rot.axisX()));
                    if (++latCount == window) {
                        maxLat = (std::max)(maxLat, latSum / static_cast<float>(window));
                        latSum = 0.0f;
                        latCount = 0;
                    }
                    return false;
                });
            check(maxLat / GRAVITY > 0.75f && maxLat / GRAVITY < 1.0f, "lateral grip about 0.9 g (g, 0.5 s average)", maxLat / GRAVITY);
            check(maxSideslip < 12.0f * PI / 180.0f, "no spin-out, sideslip under 12 deg (deg)", maxSideslip * 180.0f / PI);
            check(std::abs(w.frame.rot.axisZ().z) > 0.95f, "stays on its wheels (up.z)", w.frame.rot.axisZ().z);
        }

        {
            std::printf("Reverse\n");
            auto w = makeWorld(spec);
            w.run(1.0f, dt, idle);
            w.run(4.0f, dt, [](TestWorld&) {
                return DriverInput{ 0.0f, 0.6f, 0.0f, false };
            });
            check(w.speed() < -1.0f, "holding the brake at a standstill reverses (m/s)", w.speed());
            check(w.model.telemetry().gear == -1, "gear shows reverse", w.model.telemetry().gear);
        }

        {
            // A bump taken at 30-40 km/h off-road: the body slams down onto its wheels at 3 m/s. Without the bump stop this bottoms the travel out.
            std::printf("Hard bump\n");
            const auto slam = [&](const VehicleSpec& s, float& maxCompression, float& maxRise) {
                auto w = makeWorld(s);
                w.run(2.0f, dt, idle);
                const float z0 = w.frame.pos.z;
                w.linVel.z = -3.0f;
                maxCompression = 0.0f;
                maxRise = 0.0f;
                w.run(2.0f, dt, idle, [&](TestWorld& tw) {
                    for (int i = 0; i < 4; ++i) {
                        maxCompression = (std::max)(maxCompression, tw.model.telemetry().wheels[i].compression);
                    }
                    maxRise = (std::max)(maxRise, tw.frame.pos.z - z0);
                    return false;
                });
                return std::abs(w.frame.pos.z - z0);
            };
            VehicleSpec noStop = spec;
            noStop.suspension.bumpStopRate = 0.0f;
            noStop.suspension.bumpStopDamping = 0.0f;
            float compressionWithout = 0.0f;
            float riseWithout = 0.0f;
            slam(noStop, compressionWithout, riseWithout);
            float compression = 0.0f;
            float rise = 0.0f;
            const float settled = slam(spec, compression, rise);
            const float travel = spec.suspension.restLength;
            std::printf("  without the bump stop: %.0f%% of the travel\n", 100.0f * compressionWithout / travel);
            check(compression < 0.97f * travel, "the bump stop keeps it off the end of the travel (share of travel)", compression / travel);
            check(rise < 0.25f, "and does not throw it up (m above the ride height)", rise);
            check(settled < 0.01f, "settles back to the ride height (m)", settled);
        }

        {
            std::printf("Handbrake\n");
            auto w = makeWorld(spec);
            w.run(1.0f, dt, idle);
            w.run(
                10.0f,
                dt,
                [](TestWorld&) {
                    return DriverInput{ 0.6f, 0.0f, 0.0f, false };
                },
                [](TestWorld& tw) {
                    return tw.speed() > 20.0f / 3.6f;
                });
            // A quarter of a second in: the handbrake wheels locked (ABS does not release the handbrake), the others still rolling with the road
            // (the handbrake declutches, so no engine braking reaches them on a four-wheel drive).
            float lockedRoll = 0.0f; // the handbrake wheels' rolling speed as a share of the road speed (0 = locked)
            float freeRoll = 1.0f; // the other wheels' (1 = rolling freely)
            const float handbrakeStart = w.time;
            bool sampled = false;
            w.run(
                10.0f,
                dt,
                [](TestWorld&) {
                    return DriverInput{ 0.0f, 0.0f, 0.0f, true };
                },
                [&](TestWorld& tw) {
                    if (!sampled && tw.time - handbrakeStart >= 0.25f) {
                        sampled = true;
                        float locked = 0.0f;
                        float free = 1.0f;
                        for (std::size_t i = 0; i < spec.wheels.size(); ++i) {
                            const auto& wt = tw.model.telemetry().wheels[i];
                            const float radius = spec.wheels[i].radius > 0.0f ? spec.wheels[i].radius : spec.tyre.radius;
                            const float roll = std::abs(wt.vLong) > 0.5f ? wt.spin * radius / wt.vLong : 1.0f;
                            if (spec.wheels[i].handbrake) {
                                locked = (std::max)(locked, std::abs(roll));
                            } else {
                                free = (std::min)(free, roll);
                            }
                        }
                        lockedRoll = locked;
                        freeRoll = free;
                    }
                    return std::abs(tw.speed()) < 0.05f;
                });
            check(lockedRoll < 0.1f, "the handbrake locks its wheels (rolling share)", lockedRoll);
            check(freeRoll > 0.9f, "and leaves the others rolling (rolling share)", freeRoll);
            check(std::abs(w.speed()) < 0.1f, "the handbrake alone stops it from 20 km/h (m/s)", w.speed());
        }

        {
            std::printf("Simple diagnostic mode\n");
            auto w = makeWorld(spec);
            w.model.setSimpleMode(true);
            w.run(1.0f, dt, idle);
            w.run(5.0f, dt, [](TestWorld&) {
                return DriverInput{ 1.0f, 0.0f, 0.0f, false };
            });
            const float v = w.speed();
            check(v > 5.0f, "simple mode accelerates (m/s after 5 s)", v);
            w.run(
                10.0f,
                dt,
                [](TestWorld&) {
                    return DriverInput{ 0.0f, 1.0f, 0.0f, false };
                },
                [](TestWorld& tw) {
                    return tw.speed() < 0.05f;
                });
            check(std::abs(w.speed()) < 0.1f, "simple mode brakes to a stop (m/s)", w.speed());
        }
    }

    if (argc > 2) {
        // Scripted lap for the telemetry analyzer: settle, 0-100, brake to a stop, a slow full-lock turn, park.
        FILE* f = std::fopen(argv[2], "w");
        if (!f) {
            std::printf("cannot write %s\n", argv[2]);
            return 2;
        }
        std::fprintf(f, "frame,dt,speed_kmh,rpm,gear,throttle,brake,steer_deg,wheels_down,yaw_rate,vel_z,up_z");
        for (const char* w : { "w0", "w1", "w2", "w3" }) {
            std::fprintf(f, ",%s_down,%s_hold,%s_comp,%s_spin,%s_slip,%s_slip_deg,%s_fz,%s_fx,%s_fy,%s_grip", w, w, w, w, w, w, w, w, w, w);
        }
        std::fprintf(f, "\n");
        auto w = makeWorld(spec);
        w.csv = f;
        const float dt = 1.0f / 90.0f;
        w.run(3.0f, dt, idle);
        w.run(
            20.0f,
            dt,
            [](TestWorld&) {
                return DriverInput{ 1.0f, 0.0f, 0.0f, false };
            },
            [](TestWorld& tw) {
                return tw.speed() >= 102.0f / 3.6f;
            });
        w.run(
            15.0f,
            dt,
            [](TestWorld&) {
                return DriverInput{ 0.0f, 1.0f, 0.0f, false };
            },
            [](TestWorld& tw) {
                return tw.speed() < 0.05f;
            });
        w.run(1.0f, dt, idle);
        w.run(8.0f, dt, [](TestWorld& tw) {
            return DriverInput{ tw.speed() < 5.0f / 3.6f ? 0.35f : 0.0f, 0.0f, 1.0f, false };
        });
        w.run(
            3.0f,
            dt,
            [](TestWorld&) {
                return DriverInput{ 0.0f, 1.0f, 0.0f, false };
            },
            [](TestWorld& tw) {
                return std::abs(tw.speed()) < 0.05f;
            });
        w.run(4.0f, dt, idle);
        std::fclose(f);
        std::printf("\nTest lap telemetry written to %s\n", argv[2]);
    }

    {
        // Steering wheel: it spins in place about its column (the rim centre stays put, every rim point keeps its distance and its plane), and a right
        // turn turns it clockwise as the driver sees it
        using namespace vfvr::model;
        const Vec3 centre{ -0.283f, 0.586f, 1.300f };
        const Vec3 column = steeringColumn(25.0f);
        const Vec3 side{ 1.0f, 0.0f, 0.0f };
        const Vec3 rimTop = centre + cross(column, side) * 0.186f;
        double worstCentre = 0.0, worstRadius = 0.0, worstPlane = 0.0;
        for (const float road : { -0.6f, -0.2f, 0.1f, 0.58f }) {
            const Quat spin = steeringWheelSpin(25.0f, road, 8.0f);
            worstCentre = (std::max)(worstCentre, static_cast<double>(length(spinAboutCentre(spin, centre, centre) - centre)));
            const Vec3 moved = spinAboutCentre(spin, rimTop, centre);
            worstRadius = (std::max)(worstRadius, static_cast<double>(std::abs(length(moved - centre) - 0.186f)));
            worstPlane = (std::max)(worstPlane, static_cast<double>(std::abs(dot(moved - centre, column))));
        }
        check(worstCentre < 1e-5, "the steering wheel's centre stays put", worstCentre);
        check(worstRadius < 1e-5, "the rim keeps its radius", worstRadius);
        check(worstPlane < 1e-5, "the rim stays in its plane", worstPlane);
        // Seen from the driver (looking along -column), clockwise = the top of the rim moves to the driver's right (+x) on a right turn
        const Vec3 topAfterRight = spinAboutCentre(steeringWheelSpin(25.0f, 0.1f, 8.0f), rimTop, centre);
        check(topAfterRight.x > centre.x, "a right turn turns the rim clockwise for the driver", topAfterRight.x - centre.x);
    }

    {
        // Menu grid: two columns of twelve filled down the first column, a page per 24 entries
        using vfvr::model::MenuGrid;
        check(MenuGrid::pages(6) == 1 && MenuGrid::pages(24) == 1 && MenuGrid::pages(25) == 2 && MenuGrid::pages(0) == 1, "menu pages: 6 and 24 fit one, 25 need two",
            MenuGrid::pages(25));
        check(MenuGrid::entryAt(6, 0, 0, 5) == 5 && MenuGrid::entryAt(6, 0, 0, 6) == -1 && MenuGrid::entryAt(6, 0, 1, 0) == -1,
            "six vehicles fill the first column only", MenuGrid::entryAt(6, 0, 0, 5));
        check(MenuGrid::entryAt(20, 0, 0, 11) == 11 && MenuGrid::entryAt(20, 0, 1, 0) == 12 && MenuGrid::entryAt(20, 0, 1, 7) == 19 && MenuGrid::entryAt(20, 0, 1, 8) == -1,
            "twenty: 12 down the first column, 8 down the second", MenuGrid::entryAt(20, 0, 1, 7));
        check(MenuGrid::entryAt(30, 1, 0, 0) == 24 && MenuGrid::entryAt(30, 1, 0, 5) == 29 && MenuGrid::countOn(30, 1) == 6 && MenuGrid::rowsOn(30, 1) == 6,
            "thirty: the second page holds 24..29", MenuGrid::countOn(30, 1));
        check(MenuGrid::rowsOn(20, 0) == 12 && MenuGrid::rowsOn(3, 0) == 3 && MenuGrid::entryAt(5, 0, 2, 0) == -1, "rows per page and no third column",
            MenuGrid::rowsOn(3, 0));
    }

    {
        // Menu panel pointing: one button at a time under a finger or a beam, one press per touch or pull
        using vfvr::model::hitMenuRect;
        using vfvr::model::MenuRect;
        using vfvr::model::MenuTouch;
        using vfvr::model::MenuTrigger;
        using vfvr::model::rayOnMenuPlane;
        // Six rows as the panel lays them out at scale 2.4: 21.6 x 3.6 units, 0.72 apart
        std::vector<MenuRect> rects;
        for (int r = 0; r < 6; ++r) {
            rects.push_back({ .x = 0.0f, .z = 15.0f - 4.32f * static_cast<float>(r), .halfWidth = 10.8f, .halfHeight = 1.8f });
        }
        check(hitMenuRect(rects, 3.0f, 15.0f) == 0 && hitMenuRect(rects, -9.0f, 15.0f - 4.32f * 3.0f + 1.5f) == 3, "menu: the button under the point", 0.0);
        check(hitMenuRect(rects, 0.0f, 15.0f - 2.16f) == -1 && hitMenuRect(rects, 11.5f, 15.0f) == -1, "menu: nothing in the gap between rows or past the edge", 0.0);
        check(hitMenuRect(rects, 0.0f, 15.0f - 2.0f, 0.5f) == 0, "menu: with a margin the gap goes to the nearer row", hitMenuRect(rects, 0.0f, 15.0f - 2.0f, 0.5f));
        rects[1].enabled = false;
        check(hitMenuRect(rects, 0.0f, 15.0f - 4.32f) == -1, "menu: a disabled button is never under the point", 0.0);
        rects[1].enabled = true;
        float px = 0.0f;
        float pz = 0.0f;
        check(rayOnMenuPlane(0.0f, 0.0f, 0.0f, 0.1f, 1.0f, -0.2f, 65.0f, px, pz) && std::abs(px - 6.5f) < 1e-4f && std::abs(pz + 13.0f) < 1e-4f,
            "menu: a beam meets the panel where it points", px);
        check(!rayOnMenuPlane(0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 65.0f, px, pz) && !rayOnMenuPlane(0.0f, 70.0f, 0.0f, 0.0f, 1.0f, 0.0f, 65.0f, px, pz),
            "menu: a beam pointing away or from behind does not", 0.0);

        // A fingertip coming in on row 2, pushing through, staying in, pulling back and pushing again
        MenuTouch touch;
        const float row2 = 15.0f - 4.32f * 2.0f;
        int presses = 0;
        int last = -1;
        const auto feed = [&](const float z, const float depth) {
            const int p = touch.update(hitMenuRect(rects, 0.0f, z), depth);
            if (p >= 0) {
                ++presses;
                last = p;
            }
        };
        for (const float depth : { -10.0f, -3.0f, -1.5f, -0.5f, 0.2f, 0.7f, 1.5f, 2.5f, 2.0f, 1.0f, 0.1f }) {
            feed(row2, depth);
        }
        check(presses == 1 && last == 2, "menu: one push presses its own row once, however long it stays in", presses);
        feed(row2, -1.0f);
        feed(row2, 0.8f);
        check(presses == 2, "menu: pulled back in front of the plane, it presses again", presses);
        // Pushed in on row 2, then sliding down across rows 3 and 4 still pushed in: neither is pressed
        feed(row2, -2.0f);
        feed(row2, 1.0f);
        const int before = presses;
        for (const float z : { row2 - 2.0f, row2 - 4.32f, row2 - 6.0f, row2 - 8.64f }) {
            feed(z, 1.0f);
        }
        check(presses == before, "menu: sliding across other rows while pushed in presses none of them", presses - before);
        // A hand arriving from behind the panel presses nothing
        MenuTouch fromBehind;
        int behindPresses = 0;
        for (const float depth : { 4.0f, 2.0f, 1.0f, 0.6f }) {
            behindPresses += fromBehind.update(2, depth) >= 0 ? 1 : 0;
        }
        check(behindPresses == 0, "menu: a fingertip coming from behind the panel presses nothing", behindPresses);

        // The trigger: one held when the menu opens presses nothing, then one press per pull
        MenuTrigger trigger;
        int pulls = 0;
        for (const float v : { 1.0f, 0.9f, 0.5f, 0.2f, 0.0f, 0.5f, 0.8f, 1.0f, 0.6f, 0.9f, 0.1f, 0.75f }) {
            pulls += trigger.update(v) ? 1 : 0;
        }
        check(pulls == 2, "menu: a trigger held at opening is ignored, then one press per pull", pulls);
    }

    {
        // INI migration: the user's vehicle list keeps its ids and order, the shipped list's new ids are appended
        using vfvr::model::mergeProfileLists;
        const std::string shipped = "sedan, pickup, rockboy, rusianmother, coupe, armywagon, coupe50, belair, coupeamarillo";
        check(mergeProfileLists("sedan, pickup, rockboy, rusianmother", shipped) == shipped, "migration appends the new vehicles", 0.0);
        check(mergeProfileLists("rockboy,Sedan", shipped) == "rockboy, Sedan, pickup, rusianmother, coupe, armywagon, coupe50, belair, coupeamarillo", "migration keeps the user's order and case",
            0.0);
        check(mergeProfileLists("sedan, mytruck", "sedan") == "sedan, mytruck", "migration keeps ids the defaults lack", 0.0);
        check(mergeProfileLists(shipped, shipped) == shipped, "migration leaves a full list as it is", 0.0);
    }

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
