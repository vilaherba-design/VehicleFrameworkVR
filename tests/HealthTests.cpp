// Offline tests of the vehicle health and radiation protection (pure C++, no game): the damage formulas and their scaling with the levels,
// the collision damage with its threshold, cap and cooldown, the health limits and the disabled condition, one explosion counted once, the
// radiation curve, the reserve used up by the ambient intensity and its exhaustion, a light against an armoured vehicle, and the defaults of
// a profile without "damage" / "radiation" blocks.
//
// Usage: HealthTests [path/to/sedan.json]   (exit code 0 = all passed)

#include <cmath>
#include <cstdio>
#include <exception>
#include <string>

#include "TestWorld.h"
#include "model/RadShield.h"
#include "model/VehicleHealth.h"

using namespace vfvr::model;

namespace
{
    bool approx(const float a, const float b, const float tolerance = 0.01f)
    {
        return std::fabs(a - b) <= tolerance;
    }

    // The two example profiles of the design: a light buggy and an armoured truck.
    const char* LIGHT_JSON = R"({
        "id": "light",
        "damage": { "maxHealth": 300, "resistance": { "ballistic": 20, "explosion": 0, "collision": 20 }, "occupantProtected": true },
        "radiation": { "resistance": 150, "capacity": 300 }
    })";
    const char* ARMOURED_JSON = R"({
        "id": "armoured",
        "damage": { "maxHealth": 1200, "resistance": { "ballistic": 250, "explosion": 150, "collision": 200 }, "collision": { "minDeltaV": 4 } },
        "radiation": { "resistance": 1000, "capacity": 3000, "recoverPerSecond": 0.5 }
    })";

    bool parseFails(const char* jsonText)
    {
        try {
            parseVehicleSpec(jsonText);
            return false;
        } catch (const std::exception&) {
            return true;
        }
    }
}

int main(const int argc, char** argv)
{
    const char* path = argc > 1 ? argv[1] : nullptr;
    const DamageScaling scaling{};
    const DamageSpec sedan{}; // the defaults: 400 points, no resistance

    {
        std::printf("\n=== Hits: gunfire damages the vehicle ===\n");
        VehicleHealth health(sedan);
        const HitInput pistol{ HitKind::Ballistic, 18.0f, 1, 1 }; // a 10 mm pistol, level 1 against level 1
        const float taken = health.applyHit(pistol, scaling);
        check(approx(taken, 18.0f), "10 mm pistol at level 1 takes its base damage (18)", taken);
        check(approx(health.health(), 382.0f), "health after the hit (382)", health.health());
        DamageSpec armoured = sedan;
        armoured.ballisticResistance = 100.0f;
        float damage = VehicleHealth::hitDamage(armoured, pistol, scaling);
        check(approx(damage, 9.0f), "ballistic resistance 100 halves it (9)", damage);
        armoured.ballisticResistance = 300.0f;
        damage = VehicleHealth::hitDamage(armoured, pistol, scaling);
        check(approx(damage, 4.5f), "resistance 300 leaves a quarter (4.5)", damage);
        const HitInput grenade{ HitKind::Explosion, 150.0f, 1, 1 };
        damage = VehicleHealth::hitDamage(sedan, grenade, scaling);
        check(approx(damage, 225.0f), "an explosion counts x1.5 (150 -> 225)", damage);
        const HitInput bat{ HitKind::Melee, 20.0f, 1, 1 };
        damage = VehicleHealth::hitDamage(sedan, bat, scaling);
        check(approx(damage, 12.0f), "a melee hit counts x0.6 (20 -> 12)", damage);
        const HitInput claw{ HitKind::Unknown, 0.0f, 1, 1 };
        damage = VehicleHealth::hitDamage(sedan, claw, scaling);
        check(approx(damage, sedan.unknownAttackDamage), "no base damage known: the profile's unknownAttackDamage (10)", damage);
        DamageScaling doubled = scaling;
        doubled.scale = 2.0f;
        damage = VehicleHealth::hitDamage(sedan, pistol, doubled);
        check(approx(damage, 36.0f), "the INI scale multiplies everything (x2 -> 36)", damage);
    }

    {
        std::printf("\n=== Level scaling ===\n");
        float factor = VehicleHealth::levelFactor(1, 1, scaling);
        check(approx(factor, 1.0f), "level 1 against level 1: x1", factor);
        factor = VehicleHealth::levelFactor(50, 1, scaling);
        check(approx(factor, 1.98f), "player level 50: x1.98", factor);
        factor = VehicleHealth::levelFactor(1, 60, scaling);
        check(approx(factor, 1.59f), "attacker level 60: x1.59", factor);
        factor = VehicleHealth::levelFactor(50, 60, scaling);
        check(approx(factor, 1.98f * 1.59f), "both together: x3.15", factor);
        factor = VehicleHealth::levelFactor(500, 1, scaling);
        check(approx(factor, 3.0f), "player level 500 is capped at x3", factor);
        factor = VehicleHealth::levelFactor(1, 500, scaling);
        check(approx(factor, 2.0f), "attacker level 500 is capped at x2", factor);
        factor = VehicleHealth::levelFactor(1, 0, scaling);
        check(approx(factor, 1.0f), "an unknown attacker level counts as 1", factor);
        const HitInput pistol50{ HitKind::Ballistic, 18.0f, 50, 12 };
        const float damage = VehicleHealth::hitDamage(sedan, pistol50, scaling);
        check(approx(damage, 18.0f * 1.98f * 1.11f, 0.05f), "10 mm at player level 50 against a level 12 raider (39.6)", damage);
    }

    {
        std::printf("\n=== Collisions: far below gunfire ===\n");
        DamageSpec spec = sedan;
        spec.maxHealth = 500.0f;
        float damage = VehicleHealth::impactDamage(spec, 2.0f);
        check(approx(damage, 0.0f), "a 2 m/s touch costs nothing", damage);
        damage = VehicleHealth::impactDamage(spec, 3.0f);
        check(approx(damage, 0.0f), "the 3 m/s threshold itself costs nothing", damage);
        damage = VehicleHealth::impactDamage(spec, 4.0f);
        check(approx(damage, 4.0f), "a 4 m/s bump: 4 points", damage);
        damage = VehicleHealth::impactDamage(spec, 10.0f);
        check(approx(damage, 74.08f, 0.1f), "36 km/h into a wall (dv 10): 74 points", damage);
        damage = VehicleHealth::impactDamage(spec, 12.0f);
        check(approx(damage, 75.0f), "dv 12 is capped at 15 % of 500 (75)", damage);
        DamageSpec tough = spec;
        tough.collisionResistance = 100.0f;
        damage = VehicleHealth::impactDamage(tough, 10.0f);
        check(approx(damage, 37.04f, 0.1f), "collision resistance 100 halves it", damage);
        const HitInput pistol{ HitKind::Ballistic, 18.0f, 1, 1 };
        const float bullet = VehicleHealth::hitDamage(spec, pistol, scaling);
        check(VehicleHealth::impactDamage(spec, 4.0f) < bullet, "a bump costs less than one pistol bullet", bullet);

        VehicleHealth health(spec);
        const float first = health.applyImpact(10.0f, 0.0);
        const float again = health.applyImpact(10.0f, 0.3);
        const float later = health.applyImpact(10.0f, 0.6);
        check(approx(first, 74.08f, 0.1f), "the first impact is counted", first);
        check(approx(again, 0.0f), "the same impact 0.3 s later (within the 0.5 s cooldown) is not counted again", again);
        check(approx(later, 74.08f, 0.1f), "a new impact after the cooldown is counted", later);
        VehicleHealth resting(spec);
        float total = 0.0f;
        for (int i = 0; i < 100; ++i) {
            total += resting.applyImpact(1.0f, 0.1 * i);
        }
        check(approx(total, 0.0f), "resting against a wall (a small dv every 0.1 s) costs nothing", total);
    }

    {
        std::printf("\n=== Limits and conditions ===\n");
        VehicleHealth health(sedan);
        health.repair(1.0f);
        check(approx(health.health(), 400.0f), "a repair never passes the maximum", health.health());
        check(health.condition() == VehicleCondition::Operational, "full health: operational", 1);
        health.setHealth(130.0f);
        check(health.condition() == VehicleCondition::Damaged, "130 of 400 (32.5 %) is damaged (under 35 %)", health.share());
        health.setHealth(150.0f);
        check(health.condition() == VehicleCondition::Operational, "150 of 400 (37.5 %) is operational", health.share());
        const HitInput nuke{ HitKind::Explosion, 10000.0f, 1, 1 };
        const float taken = health.applyHit(nuke, scaling);
        check(approx(taken, 150.0f), "a huge hit takes only what is left", taken);
        check(approx(health.health(), 0.0f) && health.disabled(), "health stops at 0: disabled", health.health());
        check(health.condition() == VehicleCondition::Disabled, "condition disabled", 1);
        check(approx(health.applyHit(nuke, scaling), 0.0f), "a disabled vehicle takes no more hits", 0);
        check(approx(health.applyImpact(20.0f, 100.0), 0.0f), "nor collisions", 0);
        check(health.disabled(), "it does not recover on its own", 1);
        health.repair(0.5f);
        check(approx(health.health(), 200.0f) && !health.disabled(), "a 50 % repair gives 200 points and the vehicle back", health.health());
        health.setHealth(-50.0f);
        check(approx(health.health(), 0.0f), "a remembered health below 0 is clamped", health.health());
        health.setHealth(9999.0f);
        check(approx(health.health(), 400.0f), "a remembered health above the maximum is clamped", health.health());
    }

    {
        std::printf("\n=== One explosion, counted once ===\n");
        VehicleHealth health(sedan);
        check(health.noteExplosion(0x1234, 0xABCD, 10.0), "the first event of an explosion counts", 1);
        check(!health.noteExplosion(0x1234, 0xABCD, 10.1), "its second event (driver and hull) 0.1 s later does not", 0);
        check(health.noteExplosion(0x1234, 0xABCD, 10.2), "the same source after the window is a new explosion", 1);
        check(health.noteExplosion(0x1234, 0x9999, 10.2), "a different source at the same time is another explosion", 1);
        check(health.noteExplosion(0x5555, 0xABCD, 10.2), "a different cause too", 1);
    }

    {
        std::printf("\n=== Radiation curve ===\n");
        float value = RadShield::efficacy(10.0f, 1000.0f);
        check(approx(value, 0.907f, 0.005f), "1000 points block 90.7 % of 10 rads/s (a river)", value);
        value = RadShield::efficacy(300.0f, 1000.0f);
        check(approx(value, 0.678f, 0.005f), "1000 points block 67.8 % of 300 rads/s (the Glowing Sea)", value);
        value = RadShield::receivedShare(10.0f, 10.0f);
        check(approx(value, 0.5f), "resistance equal to the intensity halves it", value);
        value = RadShield::receivedShare(300.0f, 100.0f);
        check(approx(value, 0.665f, 0.005f), "100 points under 300 rads/s let 66.5 % through", value);
        value = RadShield::receivedShare(10.0f, 0.0f);
        check(approx(value, 1.0f), "no resistance: all of it gets through", value);
        for (const float ambient : { 3.0f, 10.0f, 300.0f, 2000.0f }) {
            const float received = ambient * RadShield::receivedShare(ambient, 1000.0f);
            const float back = RadShield::ambientFromReceived(received, 1000.0f);
            check(approx(back, ambient, ambient * 0.01f), "the curve inverts: the ambient from what got through", back);
        }
    }

    {
        std::printf("\n=== Protection reserve ===\n");
        RadiationSpec spec{};
        spec.resistance = 1000.0f;
        spec.capacity = 300.0f;
        RadShield shield(spec);
        check(shield.active() && approx(shield.resistance(), 1000.0f), "a fresh shield lends its 1000 points", shield.resistance());
        const float river = shield.tick(10.0f, 1.0f);
        check(approx(river, 9.07f, 0.05f), "10 rads/s for 1 s uses 9.07 (what it blocked)", river);
        const float sea = shield.tick(300.0f, 1.0f);
        check(approx(sea, 203.3f, 1.0f), "300 rads/s for 1 s uses 203: more ambient, faster", sea);
        check(sea > river * 10.0f, "the consumption follows the ambient intensity", sea / river);
        RadShield quiet(spec);
        float used = quiet.tick(0.3f, 1.0f);
        check(approx(used, 0.0f), "0.3 rads/s (0.27 blocked, under the 0.5 threshold) uses nothing", used);
        used = quiet.tick(0.0f, 1.0f);
        check(approx(used, 0.0f), "no radiation: nothing used", used);
        RadShield draining(spec);
        used = 0.0f;
        int ticks = 0;
        while (draining.active() && ticks < 10000) {
            used += draining.tick(300.0f, 0.25f);
            ++ticks;
        }
        check(!draining.active() && approx(draining.remaining(), 0.0f), "the reserve runs out", draining.remaining());
        check(approx(used, 300.0f), "exactly the capacity was used", used);
        check(approx(draining.resistance(), 0.0f), "an empty shield lends nothing: normal game radiation from here", draining.resistance());
        used = draining.tick(300.0f, 1.0f);
        check(approx(used, 0.0f), "and uses nothing more", used);
        draining.repair(1.0f);
        check(draining.active() && approx(draining.remaining(), 300.0f), "a repair fills it again", draining.remaining());
        RadiationSpec recovering = spec;
        recovering.recoverPerSecond = 0.5f;
        RadShield slow(recovering);
        slow.setRemaining(100.0f);
        slow.tick(0.0f, 2.0f);
        check(approx(slow.remaining(), 101.0f), "recoverPerSecond 0.5 gives 1 back in 2 s", slow.remaining());
        const RadiationSpec none{};
        RadShield bare(none);
        used = bare.tick(300.0f, 1.0f);
        check(!bare.active() && approx(bare.resistance(), 0.0f) && approx(used, 0.0f), "a vehicle without protection lends nothing and uses nothing", used);
        RadShield measured(spec);
        used = measured.drain(100.0f, 0.5f);
        check(approx(used, 50.0f), "drain: 100 rads/s kept off for 0.5 s uses 50", used);
        used = measured.drain(0.2f, 1.0f);
        check(approx(used, 0.0f), "drain: 0.2 rads/s kept off is under the threshold, nothing used", used);
        RadShield byCurve(spec);
        RadShield byDrain(spec);
        const float curveUsed = byCurve.tick(10.0f, 1.0f);
        const float drainUsed = byDrain.drain(10.0f * RadShield::efficacy(10.0f, 1000.0f), 1.0f);
        check(approx(curveUsed, drainUsed), "tick is drain of what the curve blocks", drainUsed);
        measured.setRemaining(5.0f);
        used = measured.drain(100.0f, 1.0f);
        check(approx(used, 5.0f) && !measured.active(), "drain never uses more than what is left", used);
    }

    {
        std::printf("\n=== A light against an armoured vehicle (profiles) ===\n");
        VehicleSpec light;
        VehicleSpec armoured;
        try {
            light = parseVehicleSpec(LIGHT_JSON);
            armoured = parseVehicleSpec(ARMOURED_JSON);
        } catch (const std::exception& e) {
            std::printf("Example profile failed: %s\n", e.what());
            return 2;
        }
        check(approx(light.damage.maxHealth, 300.0f) && approx(light.damage.ballisticResistance, 20.0f) && approx(light.damage.collisionResistance, 20.0f) &&
                approx(light.radiation.resistance, 150.0f) && approx(light.radiation.capacity, 300.0f),
            "the light profile parsed", light.damage.maxHealth);
        check(approx(armoured.damage.maxHealth, 1200.0f) && approx(armoured.damage.ballisticResistance, 250.0f) && approx(armoured.damage.explosionResistance, 150.0f) &&
                approx(armoured.damage.collisionMinDeltaV, 4.0f) && approx(armoured.radiation.recoverPerSecond, 0.5f),
            "the armoured profile parsed", armoured.damage.maxHealth);
        const HitInput rifle{ HitKind::Ballistic, 30.0f, 20, 20 };
        const float toLight = VehicleHealth::hitDamage(light.damage, rifle, scaling);
        const float toArmoured = VehicleHealth::hitDamage(armoured.damage, rifle, scaling);
        check(toArmoured < toLight * 0.5f, "the armoured truck takes less than half of what the buggy takes", toArmoured / toLight);
        check(VehicleHealth::impactDamage(armoured.damage, 10.0f) < VehicleHealth::impactDamage(light.damage, 10.0f), "the same for a collision",
            VehicleHealth::impactDamage(armoured.damage, 10.0f));
        check(RadShield::efficacy(10.0f, armoured.radiation.resistance) > RadShield::efficacy(10.0f, light.radiation.resistance), "the armoured cabin blocks more radiation",
            RadShield::efficacy(10.0f, armoured.radiation.resistance));
        RadShield lightShield(light.radiation);
        RadShield armouredShield(armoured.radiation);
        int lightTicks = 0;
        while (lightShield.active() && lightTicks < 100000) {
            lightShield.tick(100.0f, 0.25f);
            ++lightTicks;
        }
        int armouredTicks = 0;
        while (armouredShield.active() && armouredTicks < 100000) {
            armouredShield.tick(100.0f, 0.25f);
            ++armouredTicks;
        }
        check(armouredTicks > lightTicks * 3, "and its protection lasts far longer", static_cast<double>(armouredTicks) / lightTicks);
    }

    if (path) {
        std::printf("\n=== The shipped sedan profile (from file) ===\n");
        VehicleSpec sedanFile;
        try {
            sedanFile = loadVehicleSpecFile(path);
        } catch (const std::exception& e) {
            std::printf("Profile load failed: %s\n", e.what());
            return 2;
        }
        check(approx(sedanFile.damage.maxHealth, 400.0f) && approx(sedanFile.damage.ballisticResistance, 0.0f), "400 points, no resistance", sedanFile.damage.maxHealth);
        check(approx(sedanFile.radiation.resistance, 150.0f) && approx(sedanFile.radiation.capacity, 300.0f), "150 points of radiation resistance, 300 rads of reserve",
            sedanFile.radiation.resistance);
        check(sedanFile.damage.smokePoints.size() == 1 && approx(sedanFile.damage.smokePoints[0].y, 150.0f * GAME_TO_HAVOK, 0.001f), "one smoke point on the bonnet (meters)",
            sedanFile.damage.smokePoints.empty() ? 0.0f : sedanFile.damage.smokePoints[0].y);
        check(sedanFile.damage.hasRepairPoint && approx(sedanFile.damage.repairPoint.y, 215.0f * GAME_TO_HAVOK, 0.001f), "the repair point at the front of the bonnet (meters)",
            sedanFile.damage.repairPoint.y);
    }

    {
        std::printf("\n=== A profile without the new blocks (the vehicles of today) ===\n");
        VehicleSpec spec;
        try {
            spec = parseVehicleSpec(R"({ "id": "old" })");
        } catch (const std::exception& e) {
            std::printf("Profile load failed: %s\n", e.what());
            return 2;
        }
        std::printf("Profile '%s' (inline)\n", spec.id.c_str());
        check(approx(spec.damage.maxHealth, 400.0f), "400 points", spec.damage.maxHealth);
        check(approx(spec.damage.ballisticResistance, 0.0f) && approx(spec.damage.explosionResistance, 0.0f) && approx(spec.damage.collisionResistance, 0.0f), "no resistance",
            spec.damage.ballisticResistance);
        check(spec.damage.occupantProtected && spec.damage.hitsWhileParked && spec.damage.selfDamage, "the driver is covered, parked hits and own fire count", 1);
        check(approx(spec.damage.collisionMinDeltaV, 3.0f) && approx(spec.damage.damagedBelow, 0.35f) && spec.damage.onZeroHealth == "disable", "collision and state defaults", 1);
        check(approx(spec.radiation.resistance, 0.0f) && approx(spec.radiation.capacity, 0.0f), "no radiation protection", spec.radiation.resistance);
        check(!spec.damage.hasRepairPoint && spec.damage.smokePoints.empty(), "no repair point and no smoke points: the vehicle's own defaults", 1);
        VehicleHealth health(spec.damage);
        RadShield shield(spec.radiation);
        check(approx(health.health(), 400.0f) && !health.disabled() && !shield.active(), "it drives as today: full health, nothing lent", health.health());
        check(parseFails(R"({ "damage": { "maxHealth": 0 } })"), "maxHealth 0 is rejected", 1);
        check(parseFails(R"({ "damage": { "resistance": { "ballistic": -5 } } })"), "a negative resistance is rejected", 1);
        check(parseFails(R"({ "damage": { "onZeroHealth": "explode" } })"), "an unknown zero-health state is rejected", 1);
        check(parseFails(R"({ "radiation": { "capacity": -1 } })"), "a negative capacity is rejected", 1);
    }

    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
