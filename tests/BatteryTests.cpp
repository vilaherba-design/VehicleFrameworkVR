// Offline tests of the battery (pure C++, no game): the drain between idle and full throttle, emptied once, the charging and its end, a fusion
// core on the road, the save
// state, the dead engine (no drive, no reverse, both pedals brake), and the profile's "battery" block with its defaults.
//
// Usage: BatteryTests   (exit code 0 = all passed)

#include <cmath>
#include <cstdio>
#include <exception>
#include <string>

#include "TestWorld.h"
#include "model/Battery.h"

using namespace vfvr::model;
using namespace vfvr::testing;

namespace
{
    bool approx(const float a, const float b, const float tolerance = 0.01f)
    {
        return std::fabs(a - b) <= tolerance;
    }

    bool parseFails(const char* jsonText)
    {
        try {
            parseVehicleSpec(jsonText);
            return false;
        } catch (const std::exception&) {
            return true;
        }
    }

    // Minutes until the battery is empty at a steady throttle, in 0.1 s steps (a float charge stepped at 90 Hz for hours drifts).
    float minutesToEmpty(Battery battery, const float throttle, const float scale = 1.0f)
    {
        const float dt = 0.1f;
        double t = 0.0;
        while (!battery.empty() && t < 24.0 * 3600.0) {
            battery.drain(dt, throttle, scale);
            t += dt;
        }
        return static_cast<float>(t / 60.0);
    }

    float minutesToFull(Battery battery, const float scale = 1.0f)
    {
        const float dt = 0.1f;
        double t = 0.0;
        while (!battery.full() && t < 24.0 * 3600.0) {
            battery.recharge(dt, scale);
            t += dt;
        }
        return static_cast<float>(t / 60.0);
    }

    DriverInput pedals(const float throttle, const float brake, const bool engineOff)
    {
        DriverInput in{};
        in.throttle = throttle;
        in.brake = brake;
        in.engineOff = engineOff;
        return in;
    }
}

int main()
{
    const BatterySpec defaults{};

    {
        std::printf("\n=== Drain: idle to full throttle ===\n");
        const Battery battery(defaults);
        check(approx(battery.charge(), 1.0f) && !battery.empty() && battery.full(), "a new vehicle comes full", battery.charge());
        check(approx(minutesToEmpty(battery, 1.0f), 15.0f, 0.1f), "flat out it lasts 15 minutes", minutesToEmpty(battery, 1.0f));
        check(approx(minutesToEmpty(battery, 0.0f), 240.0f, 0.5f), "at idle it lasts 240 minutes", minutesToEmpty(battery, 0.0f));
        const float normal = minutesToEmpty(battery, 0.4f);
        check(normal > 25.0f && normal < 40.0f, "normal driving (40 % throttle) lasts about half an hour", normal);
        check(approx(minutesToEmpty(battery, 1.0f, 2.0f), 7.5f, 0.1f), "fBatteryScale 2 halves it", minutesToEmpty(battery, 1.0f, 2.0f));
        check(approx(Battery::drainPerSecond(defaults, 3.0f), Battery::drainPerSecond(defaults, 1.0f), 1e-7f), "throttle over 1 is clamped", 1);
    }

    {
        std::printf("\n=== Emptied once, charged ===\n");
        Battery battery(defaults);
        battery.restore(0.001f);
        int emptied = 0;
        for (int i = 0; i < 900; ++i) {
            emptied += battery.drain(1.0f / 90.0f, 1.0f, 1.0f) ? 1 : 0;
        }
        check(emptied == 1 && battery.empty(), "it reports empty once", emptied);
        check(approx(minutesToFull(battery), 5.0f, 0.1f), "empty to full in 5 minutes at a station", minutesToFull(battery));
        check(approx(minutesToFull(battery, 2.0f), 2.5f, 0.1f), "fChargeScale 2 halves it", minutesToFull(battery, 2.0f));
        int filled = 0;
        for (int i = 0; i < 90 * 400; ++i) {
            filled += battery.recharge(1.0f / 90.0f, 1.0f) ? 1 : 0;
        }
        check(filled == 1 && battery.full() && approx(battery.charge(), 1.0f), "it reports full once and stops at 100 %", filled);
        battery.restore(0.3f);
        check(approx(battery.charge(), 0.3f), "a save with 30 % restores it", battery.charge());
        battery.restore(7.0f);
        check(approx(battery.charge(), 1.0f), "a bad save value is clamped", battery.charge());
        battery.restore(0.0f);
        battery.addShare(0.15f);
        check(approx(battery.charge(), 0.15f) && !battery.empty(), "a fusion core on the road gives 15 %", battery.charge());
        battery.restore(0.95f);
        battery.addShare(0.15f);
        check(approx(battery.charge(), 1.0f), "never over full", battery.charge());
        check(approx(Battery::coreCharge(false, 0.0f), 1.0f) && approx(Battery::coreCharge(true, 0.45f), 0.45f) && approx(Battery::coreCharge(true, 45.0f), 0.45f) &&
                  approx(Battery::coreCharge(true, -3.0f), 0.0f),
            "a core's charge from its health (none = full, 0..1, percentage, negative = empty)", 1);
        Battery parked(defaults);
        parked.drain(0.0f, 1.0f, 1.0f);
        parked.drain(1.0f, 1.0f, 0.0f);
        check(approx(parked.charge(), 1.0f), "no time or scale 0: nothing used", parked.charge());
    }

    {
        std::printf("\n=== The dead engine ===\n");
        VehicleSpec spec;
        try {
            spec = parseVehicleSpec(R"({ "id": "battery" })");
        } catch (const std::exception& e) {
            std::printf("Profile load failed: %s\n", e.what());
            return 2;
        }
        spec.ensureWheels({ -0.95f, -2.45f, 0.0f }, { 0.95f, 2.45f, 1.45f });
        const float dt = 1.0f / 90.0f;
        auto world = makeWorld(spec);
        world.run(1.0f, dt, idle); // settle on the springs
        world.run(3.0f, dt, [](TestWorld&) { return pedals(1.0f, 0.0f, true); });
        check(std::fabs(world.speed()) < 0.3f, "full throttle with an empty battery: the car does not move", world.speed());
        world.run(3.0f, dt, [](TestWorld&) { return pedals(0.0f, 1.0f, true); });
        check(std::fabs(world.speed()) < 0.3f && world.model.telemetry().gear >= 0, "holding the brake does not select reverse", world.model.telemetry().gear);
        world.run(4.0f, dt, [](TestWorld&) { return pedals(1.0f, 0.0f, false); });
        check(world.speed() > 5.0f, "charged, it drives off", world.speed());
        const float before = world.speed();
        world.run(2.0f, dt, [](TestWorld&) { return pedals(1.0f, 0.0f, true); });
        check(world.speed() < before - 1.0f, "emptied at speed: the throttle pedal brakes", world.speed());
    }

    {
        std::printf("\n=== The profile's battery block ===\n");
        const auto plain = parseVehicleSpec(R"({ "id": "old" })");
        check(plain.battery.enabled && approx(plain.battery.minutesFullThrottle, 15.0f) && approx(plain.battery.minutesIdle, 240.0f) &&
                  approx(plain.battery.minutesToCharge, 5.0f),
            "no block: the defaults", 1);
        const auto custom = parseVehicleSpec(R"({ "battery": { "enabled": false, "minutesFullThrottle": 30, "minutesIdle": 600, "minutesToCharge": 60 } })");
        check(!custom.battery.enabled && approx(custom.battery.minutesFullThrottle, 30.0f) && approx(custom.battery.minutesIdle, 600.0f) &&
                  approx(custom.battery.minutesToCharge, 60.0f),
            "a block is read", 1);
        Battery none(custom.battery);
        none.restore(0.0f);
        check(!none.empty(), "a vehicle without a battery is never empty", 1);
        check(parseFails(R"({ "battery": { "minutesFullThrottle": 0 } })"), "0 minutes is rejected", 1);
        check(parseFails(R"({ "battery": { "minutesToCharge": -1 } })"), "a negative charging time is rejected", 1);
    }

    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
