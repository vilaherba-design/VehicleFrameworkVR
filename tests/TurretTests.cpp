// Offline tests of the vehicle turret (pure C++, no game): the turret JSON and its errors, the joints turning towards the aim at their speed
// (short way round, stopping at the limits), the fire cone, the rate of fire, the spread and the engine's launch angles.
//
// Usage: TurretTests [turret.json...]   (exit code 0 = all passed)

#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "model/Turret.h"

using namespace vfvr::model;

namespace
{
    int g_failures = 0;
    int g_checks = 0;

    void check(const bool ok, const char* what)
    {
        ++g_checks;
        if (!ok) {
            ++g_failures;
            std::printf("FAIL: %s\n", what);
        }
    }

    bool approx(const float a, const float b, const float tolerance = 0.01f)
    {
        return std::fabs(a - b) <= tolerance;
    }

    bool parseFails(const std::string& jsonText)
    {
        try {
            parseTurretSpec(jsonText);
            return false;
        } catch (const std::exception&) {
            return true;
        }
    }

    constexpr float DEG = PI / 180.0f;

    const char* MINIMAL = R"({"id": "t", "mesh": "a.nif", "nodes": {"yaw": "Y", "pitch": "P"}, "weapon": {"form": "Fallout4.esm|0x1"}})";

    Vec3 dirAt(const float yawDeg, const float pitchDeg)
    {
        const float y = yawDeg * DEG;
        const float p = pitchDeg * DEG;
        return { std::sin(y) * std::cos(p), std::cos(y) * std::cos(p), std::sin(p) };
    }

    // Seconds until the turret is within `tolerance` rad of `dir`, in 1/90 s frames (cap 20 s).
    float secondsToAim(TurretAim aim, const Vec3& dir, const float tolerance = 0.01f)
    {
        float t = 0.0f;
        while (aim.errorTo(dir) > tolerance && t < 20.0f) {
            aim.update(dir, 1.0f / 90.0f);
            t += 1.0f / 90.0f;
        }
        return t;
    }
}

int main(int argc, char** argv)
{
    // JSON
    {
        const TurretSpec spec = parseTurretSpec(MINIMAL);
        check(spec.id == "t" && spec.displayName == "t", "minimal turret: id and name");
        check(approx(spec.yawSpeedDeg, 120.0f) && approx(spec.roundsPerMinute, 600.0f), "minimal turret: defaults");
        check(parseFails(R"({"mesh": "a.nif", "nodes": {"yaw": "Y", "pitch": "P"}, "weapon": {"form": "x"}})"), "no id fails");
        check(parseFails(R"({"id": "t", "nodes": {"yaw": "Y", "pitch": "P"}, "weapon": {"form": "x"}})"), "no mesh fails");
        check(parseFails(R"({"id": "t", "mesh": "a.nif", "nodes": {"yaw": "Y", "pitch": "P"}})"), "no weapon fails");
        check(parseFails(R"({"id": "t", "mesh": "a.nif", "nodes": {"yaw": "Y", "pitch": "P"}, "weapon": {"form": "x", "roundsPerMinute": 0}})"), "0 rpm fails");
        check(parseFails(R"({"id": "t", "mesh": "a.nif", "nodes": {"yaw": "Y", "pitch": "P"}, "weapon": {"form": "x", "ammoPerShot": 1.5}})"),
            "fractional ammo fails");
        const TurretSpec full = parseTurretSpec(
            R"({"id": "mg", "name": "MG", "mesh": "a.nif", "nodes": {"yaw": "Y", "pitch": "P", "muzzle": "M", "muzzleOffset": [0, 70, 0]},
                "aim": {"yawMin": -90, "yawMax": 90, "pitchMin": -5, "pitchMax": 30, "yawSpeed": 60, "pitchSpeed": 40, "fireCone": 3},
                "weapon": {"form": "Fallout4.esm|0x1", "ammo": "Fallout4.esm|0x2", "ammoPerShot": 2, "roundsPerMinute": 300, "spread": 2},
                "kit": "VehicleFrameworkVR_Vehicles.esp|0x810"})");
        check(full.displayName == "MG" && full.muzzleNodes == std::vector<std::string>{ "M" } && approx(full.muzzleOffset.y, 1.0f),
            "full turret: nodes, muzzle in m");
        check(full.fireSoundLoop, "the fire sound is a loop by default");
        const TurretSpec twin = parseTurretSpec(
            R"({"id": "t2", "mesh": "a.nif", "nodes": {"yaw": "Y", "pitch": "P", "muzzle": ["M1", "M2"]},
                "weapon": {"form": "Fallout4.esm|0x1", "soundLoop": false}})");
        check(twin.muzzleNodes == std::vector<std::string>{ "M1", "M2" }, "muzzle list: both muzzles in order");
        check(!twin.fireSoundLoop, "soundLoop false: a single-shot sound per shot");
        check(parseTurretSpec(MINIMAL).muzzleNodes.empty(), "no muzzle: the offset is used");
        check(parseFails(R"({"id": "t", "mesh": "a.nif", "nodes": {"yaw": "Y", "pitch": "P"}, "weapon": {"form": "x", "soundLoop": 1}})"),
            "soundLoop must be a boolean");
        check(approx(full.yawMinDeg, -180.0f) && approx(full.pitchMaxDeg, 90.0f), "limits in the JSON are ignored: 360 and -30..+90 for all");
        check(approx(full.yawSpeedDeg, 60.0f) && approx(full.fireConeDeg, 3.0f) && full.ammoPerShot == 2, "full turret: values");
        check(full.kitForm == "VehicleFrameworkVR_Vehicles.esp|0x810", "full turret: kit");
    }

    // A free turret turns the short way round at its speed.
    {
        TurretSpec spec = parseTurretSpec(MINIMAL);
        TurretAim aim(spec);
        check(aim.freeYaw(), "default turret turns all the way round");
        check(approx(aim.yaw(), 0.0f) && approx(aim.pitch(), 0.0f), "rest: forward and level");
        const float t90 = secondsToAim(aim, dirAt(90.0f, 0.0f));
        check(approx(t90, 0.75f, 0.03f), "90 deg at 120 deg/s ~ 0.75 s");
        TurretAim back = aim;
        back.snap(170.0f * DEG, 0.0f);
        for (int i = 0; i < 9; ++i) {
            back.update(dirAt(-170.0f, 0.0f), 1.0f / 90.0f);
        }
        check(std::fabs(back.yaw()) > 170.0f * DEG, "170 -> -170 goes through 180, not through 0");
        TurretAim across = aim;
        across.snap(170.0f * DEG, 0.0f);
        check(approx(secondsToAim(across, dirAt(-170.0f, 0.0f)), 20.0f / 120.0f, 0.03f), "170 -> -170 takes 20 deg of travel");
        TurretAim up = aim;
        check(approx(secondsToAim(up, dirAt(0.0f, 45.0f)), 0.5f, 0.03f), "45 deg up at 90 deg/s ~ 0.5 s");
        check(approx(spec.pitchMaxDeg - spec.pitchMinDeg, 120.0f), "default: 120 deg of elevation");
        TurretAim high = aim;
        for (int i = 0; i < 300; ++i) {
            high.update(dirAt(180.0f, 60.0f), 1.0f / 90.0f);
        }
        check(approx(high.pitch(), 60.0f * DEG, 0.001f), "aims high behind");
        TurretAim top(spec);
        top.snap(0.0f, 0.0f);
        for (int i = 0; i < 300; ++i) {
            top.update({ 0.0f, 0.0f, 1.0f }, 1.0f / 90.0f);
        }
        check(approx(top.pitch(), 90.0f * DEG, 0.001f), "straight up reaches 90 deg");
        TurretAim low = aim;
        for (int i = 0; i < 300; ++i) {
            low.update(dirAt(0.0f, -60.0f), 1.0f / 90.0f);
        }
        check(approx(low.pitch(), -30.0f * DEG, 0.001f), "pitch stops at pitchMin (-30)");
        TurretAim straightUp = aim;
        straightUp.snap(30.0f * DEG, 0.0f);
        straightUp.update({ 0.0f, 0.0f, 1.0f }, 1.0f / 90.0f);
        check(approx(straightUp.yaw(), 30.0f * DEG, 0.001f), "aim straight up keeps the yaw");
    }

    // A limited turret never crosses the closed arc.
    {
        TurretSpec spec = parseTurretSpec(MINIMAL);
        spec.yawMinDeg = -90.0f;
        spec.yawMaxDeg = 90.0f;
        TurretAim aim(spec);
        check(!aim.freeYaw(), "limited yaw");
        aim.snap(80.0f * DEG, 0.0f);
        float minYaw = aim.yaw();
        for (int i = 0; i < 300; ++i) {
            aim.update(dirAt(-100.0f, 0.0f), 1.0f / 90.0f);
            minYaw = (std::min)(minYaw, aim.yaw());
            check(aim.yaw() <= 90.0f * DEG + 1e-4f && aim.yaw() >= -90.0f * DEG - 1e-4f, "limited yaw stays inside");
        }
        check(approx(aim.yaw(), -90.0f * DEG, 0.001f), "aim behind on the left: stops at -90 after turning through the front");
        TurretAim behind(spec);
        behind.update(dirAt(180.0f, 0.0f), 1.0f / 90.0f);
        check(behind.yaw() >= -90.0f * DEG && behind.yaw() <= 90.0f * DEG, "aim straight behind: stays in range");
        spec.yawMinDeg = 20.0f;
        spec.yawMaxDeg = 60.0f;
        TurretAim side(spec);
        check(approx(side.yaw(), 20.0f * DEG, 0.001f), "rest clamped into a side-only arc");
        side.snap(60.0f * DEG, 0.3f);
        for (int i = 0; i < 300; ++i) {
            side.relax(1.0f / 90.0f);
        }
        check(approx(side.yaw(), 20.0f * DEG, 0.001f) && approx(side.pitch(), 0.0f, 0.001f), "relax goes to the nearest rest");
    }

    // Fire cone.
    {
        TurretSpec spec = parseTurretSpec(MINIMAL);
        spec.fireConeDeg = 5.0f;
        TurretAim aim(spec);
        check(!aim.update(dirAt(90.0f, 0.0f), 1.0f / 90.0f), "far off the aim: no fire");
        bool ready = false;
        for (int i = 0; i < 90 && !ready; ++i) {
            ready = aim.update(dirAt(90.0f, 0.0f), 1.0f / 90.0f);
        }
        check(ready && aim.errorTo(dirAt(90.0f, 0.0f)) <= 5.0f * DEG + 1e-4f, "within the cone: fires");
        TurretAim noCone(parseTurretSpec(MINIMAL));
        check(noCone.update(dirAt(120.0f, 0.0f), 1.0f / 90.0f), "no cone: always fires");
    }

    // Rate of fire.
    {
        FireControl fire(600.0f);
        int shots = 0;
        for (int i = 0; i < 90; ++i) {
            shots += fire.update(true, 1.0f / 90.0f);
        }
        check(shots >= 10 && shots <= 11, "600 rpm held for 1 s = 10 shots");
        FireControl first(60.0f);
        check(first.update(true, 1.0f / 90.0f) == 1, "the first press fires at once");
        int tapped = 0;
        for (int i = 0; i < 80; ++i) {
            tapped += first.update(i % 2 == 0, 1.0f / 90.0f);
        }
        check(tapped == 0, "tapping does not beat the cooldown");
        FireControl hitch(6000.0f);
        hitch.update(true, 0.0f);
        check(hitch.update(true, 1.0f) == 4, "a 1 s hitch fires at most 4 shots");
        FireControl idle(600.0f);
        check(idle.update(false, 1.0f / 90.0f) == 0, "not held: no shots");
    }

    // Spread and launch angles.
    {
        const Vec3 barrel = dirAt(30.0f, 10.0f);
        check(approx(length(scatter(barrel, 0.0f, 0.5f, 0.5f) - barrel), 0.0f, 1e-5f), "no spread: along the barrel");
        float worst = 0.0f;
        for (int i = 0; i < 50; ++i) {
            for (int j = 0; j < 20; ++j) {
                const Vec3 s = scatter(barrel, 2.0f * DEG, i / 50.0f, j / 20.0f);
                check(approx(length(s), 1.0f, 1e-4f), "scattered shot is unit");
                worst = (std::max)(worst, std::acos(clampf(dot(s, barrel), -1.0f, 1.0f)));
            }
        }
        check(worst <= 2.0f * DEG + 1e-4f && worst > 1.5f * DEG, "spread stays inside its cone and fills it");
        check(approx(length(scatter({ 0.0f, 0.0f, 1.0f }, 2.0f * DEG, 0.9f, 0.3f)), 1.0f, 1e-4f), "spread straight up works");
        float z = 0.0f;
        float x = 0.0f;
        launchAngles({ 0.0f, 1.0f, 0.0f }, z, x);
        check(approx(z, 0.0f) && approx(x, 0.0f), "north, level");
        launchAngles({ 1.0f, 0.0f, 0.0f }, z, x);
        check(approx(z, PI / 2.0f), "east = +90 deg");
        launchAngles({ -1.0f, 0.0f, 0.0f }, z, x);
        check(approx(z, 1.5f * PI), "west = 270 deg");
        launchAngles(dirAt(0.0f, 30.0f), z, x);
        check(approx(x, -30.0f * DEG), "up = negative xAngle");
    }

    // The shipped turret files.
    for (int i = 1; i < argc; ++i) {
        try {
            const TurretSpec spec = loadTurretSpecFile(argv[i]);
            check(!spec.kitForm.empty(), "shipped turret has a kit");
            check(!spec.ammoForm.empty(), "shipped turret uses ammo");
            check(!spec.muzzleNodes.empty(), "shipped turret names its muzzle nodes");
        } catch (const std::exception& e) {
            std::printf("FAIL: %s: %s\n", argv[i], e.what());
            ++g_failures;
        }
    }

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
