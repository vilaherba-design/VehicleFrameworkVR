#include "Config.h"

#include <algorithm>

#include "f4vr/CollisionLayers.h"
#include "model/ProfileList.h"

using namespace common;

namespace
{
    const char* DEFAULT_SECTION = Version::PROJECT.data();

    constexpr auto DEFAULT_RAY_IGNORE =
        "actorZone, trigger, charController, biped, bipedNoCC, deadBip, nonCollidable, cameraSphere, projectileZone, gasTrap, avoidBox, collisionBox, acousticSpace, "
        "spellExplosion, water";

    float readFloat(const CSimpleIniA& ini, const char* key, const double defaultValue)
    {
        return static_cast<float>(ini.GetDoubleValue(DEFAULT_SECTION, key, defaultValue));
    }

    template <class E>
    E readEnum(const CSimpleIniA& ini, const char* key, const E defaultValue, const int maxValue)
    {
        const auto value = ini.GetLongValue(DEFAULT_SECTION, key, static_cast<long>(defaultValue));
        return value < 0 || value > maxValue ? defaultValue : static_cast<E>(value);
    }
}

namespace vfvr
{
    void Config::updateIniConfigToLatestVersionCustom(const int currentVersion, int, const CSimpleIniA& oldIni, CSimpleIniA& newIni) const
    {
        const auto resource = common::getEmbeddedResourceAsStringIfExists(IDR_CONFIG_INI);
        if (!resource.has_value()) {
            return;
        }
        CSimpleIniA shipped;
        if (shipped.LoadData(resource.value()) < 0) {
            return;
        }
        // 0.1.83: the menu panel moved 50 cm further away and its scale now follows the distance, so the old placement would put it back in your
        // face at a quarter of its size: the shipped values replace the user's this once. 0.1.84-0.1.87 (iVersion 48-49) were built without the
        // 0.1.83 panel, so their INIs still hold the old placement: the merged build (iVersion 50) replaces it for them too.
        if (currentVersion < 50) {
            for (const char* key : { "fMenuDistance", "fMenuHeight", "fMenuScale" }) {
                if (const char* value = shipped.GetValue(DEFAULT_SECTION, key, nullptr)) {
                    logger::info("Migrating {}.{}: the new default {} replaces the user's value", DEFAULT_SECTION, key, value);
                    newIni.SetValue(DEFAULT_SECTION, key, value);
                }
            }
        }
        // 0.1.89 (iVersion 51): while driving, the hull went from charController (it blocks the AI's line of sight and lets bullets through) to
        // debrisLarge, and the seated player's capsule from nonCollidable (the AI could not find the player) to detection.
        // 0.1.92 (iVersion 53): debrisLarge meets projectiles, so the hull stood between the enemies and the driver they had detected and they held
        // their fire (AIProbe: a turret 5 m away detected the driver at 85 and never shot). transparent meets neither projectiles nor line of sight.
        if (currentVersion < 53) {
            if (const char* value = shipped.GetValue(DEFAULT_SECTION, "sVehicleBodyLayer", nullptr)) {
                logger::info("Migrating {}.sVehicleBodyLayer: the new default {} replaces the user's value", DEFAULT_SECTION, value);
                newIni.SetValue(DEFAULT_SECTION, "sVehicleBodyLayer", value);
            }
        }
        // TEST BUILD 0.1.96 (iVersion 55): the hull on debrisSmall (meets neither the player's capsule nor bullets nor line of sight), the player's
        // capsule left as it is, and FRIK's selfie mode in third person, without the player editing the INI.
        // 0.1.99 (iVersion 56): back to the steady seat after the test (the hull on transparent, the player's capsule off the world: the capsule
        // left as it is shook the car and the HUD), FRIK's selfie mode kept in third person.
        if (currentVersion < 56) {
            for (const char* key : { "sVehicleBodyLayer", "bDisablePlayerCollision", "bThirdPersonSelfie" }) {
                if (const char* value = shipped.GetValue(DEFAULT_SECTION, key, nullptr)) {
                    logger::info("Migrating {}.{}: the new default {} replaces the user's value", DEFAULT_SECTION, key, value);
                    newIni.SetValue(DEFAULT_SECTION, key, value);
                }
            }
        }
        if (currentVersion < 51) {
            for (const char* key : { "sDriverCollisionLayer" }) {
                if (const char* value = shipped.GetValue(DEFAULT_SECTION, key, nullptr)) {
                    logger::info("Migrating {}.{}: the new default {} replaces the user's value", DEFAULT_SECTION, key, value);
                    newIni.SetValue(DEFAULT_SECTION, key, value);
                }
            }
        }
        // Every bump of iVersion adds the vehicles new in the shipped list to the user's own (0.1.108-impactos-coupe50, iVersion 58: coupe50;
        // 0.1.109-belair, iVersion 59: belair; 0.1.110-coupe-amarillo, iVersion 60: coupeamarillo; 0.1.111-bateria, iVersion 61, adds no vehicle but the battery keys).
        const char* user = oldIni.GetValue(DEFAULT_SECTION, "sVehicleProfiles", nullptr);
        if (user == nullptr) {
            return;
        }
        const char* defaults = shipped.GetValue(DEFAULT_SECTION, "sVehicleProfiles", nullptr);
        if (defaults == nullptr) {
            return;
        }
        const auto merged = model::mergeProfileLists(user, defaults);
        if (merged != user) {
            logger::info("Migrating {}.sVehicleProfiles: the new vehicles added to the user's list -> {}", DEFAULT_SECTION, merged);
            newIni.SetValue(DEFAULT_SECTION, "sVehicleProfiles", merged.c_str());
        }
    }

    void Config::loadIniConfigInternal(const CSimpleIniA& ini)
    {
        vehiclesFolder = ini.GetValue(DEFAULT_SECTION, "sVehiclesFolder", "Data\\F4SE\\Plugins\\VehicleFrameworkVR\\Vehicles");
        testVehicleProfile = ini.GetValue(DEFAULT_SECTION, "sTestVehicleProfile", "sedan");
        spawnDistance = readFloat(ini, "fSpawnDistance", 6.0);
        simpleMode = ini.GetBoolValue(DEFAULT_SECTION, "bSimpleMode", false);
        impulseApi = readEnum(ini, "iImpulseApi", ImpulseApi::HknpWorld, 1);
        wheelRayLayer = ini.GetValue(DEFAULT_SECTION, "sWheelRayLayer", "charController");
        vehicleBodyLayer = ini.GetValue(DEFAULT_SECTION, "sVehicleBodyLayer", "transparent");
        parkedBodyLayer = ini.GetValue(DEFAULT_SECTION, "sParkedBodyLayer", "");
        wheelRayIgnoreMask = f4vr::parseCollisionLayerMask(ini.GetValue(DEFAULT_SECTION, "sWheelRayIgnoreLayers", DEFAULT_RAY_IGNORE));
        releasePickWorldReference = ini.GetBoolValue(DEFAULT_SECTION, "bReleasePickWorldReference", true);
        wheelSet = ini.GetValue(DEFAULT_SECTION, "sWheelSet", "stock");
        wheelMenuOnEnter = ini.GetBoolValue(DEFAULT_SECTION, "bWheelMenuOnEnter", false);
        wheelMenuAfterSpawn = ini.GetBoolValue(DEFAULT_SECTION, "bWheelMenuAfterSpawn", true);
        gridMenu = ini.GetBoolValue(DEFAULT_SECTION, "bGridMenu", true);
        menuDistance = std::clamp(readFloat(ini, "fMenuDistance", 65.0), 10.0f, 200.0f);
        menuHeight = std::clamp(readFloat(ini, "fMenuHeight", -12.0), -80.0f, 60.0f);
        menuSide = std::clamp(readFloat(ini, "fMenuSide", -7.0), -80.0f, 80.0f);
        menuScale = std::clamp(readFloat(ini, "fMenuScale", 1.0), 0.3f, 3.0f);
        menuLaser = ini.GetBoolValue(DEFAULT_SECTION, "bMenuLaser", true);
        menuBothHands = ini.GetBoolValue(DEFAULT_SECTION, "bMenuBothHands", true);
        menuFollowAngle = std::clamp(readFloat(ini, "fMenuFollowAngle", 50.0), 10.0f, 180.0f);
        exhaustSmoke = ini.GetBoolValue(DEFAULT_SECTION, "bExhaustSmoke", true);
        engineSound = ini.GetBoolValue(DEFAULT_SECTION, "bEngineSound", true);
        engineVolume = static_cast<float>(ini.GetDoubleValue(DEFAULT_SECTION, "fEngineVolume", 0.6));
        vehicleProfiles = ini.GetValue(DEFAULT_SECTION, "sVehicleProfiles", "sedan, pickup, rockboy, rusianmother, coupe, armywagon, coupe50, belair, coupeamarillo");

        enterDistance = readFloat(ini, "fEnterDistance", 2.5);
        handbrakeStickThreshold = std::clamp(readFloat(ini, "fHandbrakeStickThreshold", 0.7), 0.0f, 1.0f);
        {
            const std::string stick = common::str_tolower(common::trim(ini.GetValue(DEFAULT_SECTION, "sHandbrakeStick", "right")));
            handbrakeStick = stick == "left" ? vrcf::Hand::Offhand : vrcf::Hand::Primary;
        }
        enterGlideSeconds = std::clamp(readFloat(ini, "fEnterGlideSeconds", 0.4), 0.0f, 3.0f);
        deadzone = std::clamp(readFloat(ini, "fDeadzone", 0.08), 0.0f, 0.5f);
        responseCurve = std::clamp(readFloat(ini, "fResponseCurve", 1.4), 0.5f, 3.0f);
        seatPoseSource = readEnum(ini, "iSeatPoseSource", SeatPoseSource::HavokBody, 1);
        cameraUpdatePoint = readEnum(ini, "iCameraUpdatePoint", CameraUpdatePoint::FrikFrameBegin, 1);
        roomYawSign = readFloat(ini, "fRoomYawSign", 1.0) < 0.0f ? -1.0f : 1.0f;
        autoDetectYawSign = ini.GetBoolValue(DEFAULT_SECTION, "bAutoDetectYawSign", true);
        verticalFilterHz = (std::max)(readFloat(ini, "fVerticalFilterHz", 4.0), 0.0f);
        verticalFilterMax = (std::max)(readFloat(ini, "fVerticalFilterMax", 0.04), 0.0f);
        playerNoGravity = ini.GetBoolValue(DEFAULT_SECTION, "bPlayerNoGravity", true);
        lockModelToCamera = ini.GetBoolValue(DEFAULT_SECTION, "bLockModelToCamera", true);
        modelLagFrames = std::clamp(static_cast<int>(ini.GetLongValue(DEFAULT_SECTION, "iModelLagFrames", 1)), 0, 6);
        cabinTiltScale = static_cast<float>(std::clamp(ini.GetDoubleValue(DEFAULT_SECTION, "fCabinTiltScale", 0.4), 0.0, 1.0));
        renderProbe = ini.GetBoolValue(DEFAULT_SECTION, "bRenderProbe", false);
        drawBodyNode = ini.GetBoolValue(DEFAULT_SECTION, "bDrawBodyNode", true);
        modelPreviousWorld = ini.GetBoolValue(DEFAULT_SECTION, "bModelPreviousWorld", true);
        disablePlayerCollision = ini.GetBoolValue(DEFAULT_SECTION, "bDisablePlayerCollision", true);
        driverCollisionLayer = ini.GetValue(DEFAULT_SECTION, "sDriverCollisionLayer", "detection");
        blockFrikSmoothMovement = ini.GetBoolValue(DEFAULT_SECTION, "bBlockFrikSmoothMovement", true);

        spawnTestVehicleBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sSpawnTestVehicleBinding",
            vrcf::InputBinding{ vrcf::Hand::Primary, vrcf::ActivationType::LongPress, vr::k_EButton_ApplicationMenu });
        enterBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sEnterBinding",
            vrcf::InputBinding{ .hand = vrcf::Hand::Primary, .type = vrcf::ActivationType::Press, .button = vr::k_EButton_A, .suppress = true });
        trunkBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sTrunkBinding",
            vrcf::InputBinding{ .hand = vrcf::Hand::Primary, .type = vrcf::ActivationType::Press, .button = vr::k_EButton_A, .suppress = true });
        exitBinding =
            getInputBindingValue(ini, DEFAULT_SECTION, "sExitBinding", vrcf::InputBinding{ vrcf::Hand::Offhand, vrcf::ActivationType::LongPress, vr::k_EButton_ApplicationMenu });
        recenterBinding =
            getInputBindingValue(ini, DEFAULT_SECTION, "sRecenterBinding", vrcf::InputBinding{ vrcf::Hand::Primary, vrcf::ActivationType::Tap, vr::k_EButton_SteamVR_Touchpad });
        lightsBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sLightsBinding",
            vrcf::InputBinding{ vrcf::Hand::Primary, vrcf::ActivationType::LongPress, vr::k_EButton_SteamVR_Touchpad });

        thirdPersonBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sThirdPersonBinding",
            vrcf::InputBinding{ vrcf::Hand::Offhand, vrcf::ActivationType::Press, vr::k_EButton_SteamVR_Touchpad });
        thirdPersonDistance = std::clamp(readFloat(ini, "fThirdPersonDistance", 11.0), 3.0f, 40.0f);
        thirdPersonHeight = std::clamp(readFloat(ini, "fThirdPersonHeight", 3.5), 0.0f, 20.0f);
        thirdPersonOrbitSpeed = std::clamp(readFloat(ini, "fThirdPersonOrbitSpeed", 120.0), 0.0f, 360.0f);
        thirdPersonOrbitInvert = ini.GetBoolValue(DEFAULT_SECTION, "bThirdPersonOrbitInvert", true);
        thirdPersonBodyInSeat = ini.GetBoolValue(DEFAULT_SECTION, "bThirdPersonBodyInSeat", true);
        thirdPersonHideFirstPerson = ini.GetBoolValue(DEFAULT_SECTION, "bThirdPersonHideFirstPerson", true);
        thirdPersonHitBoxes = ini.GetBoolValue(DEFAULT_SECTION, "bThirdPersonHitBoxes", true);
        thirdPersonSelfie = ini.GetBoolValue(DEFAULT_SECTION, "bThirdPersonSelfie", false);
        thirdPersonActorInSeat = ini.GetBoolValue(DEFAULT_SECTION, "bThirdPersonActorInSeat", true);
        {
            const auto legs = common::str_tolower(common::trim(ini.GetValue(DEFAULT_SECTION, "sThirdPersonLegs", "pose")));
            thirdPersonLegs = legs == "hide" || legs == "none" ? legs : std::string("pose");
        }
        seatThighAngleDeg = std::clamp(readFloat(ini, "fSeatThighAngleDeg", 80.0), 0.0f, 120.0f);
        seatKneeAngleDeg = std::clamp(readFloat(ini, "fSeatKneeAngleDeg", 90.0), 0.0f, 150.0f);
        seatKneeSpreadDeg = std::clamp(readFloat(ini, "fSeatKneeSpreadDeg", 8.0), 0.0f, 40.0f);
        holsterWeaponWhileDriving = ini.GetBoolValue(DEFAULT_SECTION, "bHolsterWeaponWhileDriving", true);
        thirdPersonRecenterDelay = std::clamp(readFloat(ini, "fThirdPersonRecenterDelay", 3.0), 0.0f, 60.0f);
        cameraCollision = ini.GetBoolValue(DEFAULT_SECTION, "bCameraCollision", true);
        cameraCollisionMargin = std::clamp(readFloat(ini, "fCameraCollisionMargin", 0.3), 0.0f, 3.0f);
        cameraMinDistance = std::clamp(readFloat(ini, "fCameraMinDistance", 2.0), 0.5f, 20.0f);
        cameraSwitchSeconds = std::clamp(readFloat(ini, "fCameraSwitchSeconds", 0.6), 0.0f, 3.0f);

        hud = ini.GetBoolValue(DEFAULT_SECTION, "bHud", true);
        hudDistance = std::clamp(readFloat(ini, "fHudDistance", 1.0), 0.3f, 5.0f);
        hudDownAngle = std::clamp(readFloat(ini, "fHudDownAngle", 25.0), -60.0f, 80.0f);
        hudScale = std::clamp(readFloat(ini, "fHudScale", 1.0), 0.2f, 5.0f);
        hudFollowSeconds = std::clamp(readFloat(ini, "fHudFollowSeconds", 0.2), 0.0f, 3.0f);
        hudDistanceThirdPerson = std::clamp(readFloat(ini, "fHudDistanceThirdPerson", 1.0), 0.3f, 5.0f);
        hudDownAngleThirdPerson = std::clamp(readFloat(ini, "fHudDownAngleThirdPerson", 40.0), -60.0f, 80.0f);

        telemetry = ini.GetBoolValue(DEFAULT_SECTION, "bTelemetry", true);
        telemetryMaxMinutes = (std::max)(readFloat(ini, "fTelemetryMaxMinutes", 10.0), 0.5f);
        debugDraw = ini.GetBoolValue(DEFAULT_SECTION, "bDebugDraw", false);
        rightAfterSeconds = std::clamp(readFloat(ini, "fRightAfterSeconds", 3.0), 0.0f, 60.0f);
        fallRecoverMeters = (std::max)(readFloat(ini, "fFallRecoverMeters", 3.0), 0.0f);
        renderInterpolation = ini.GetBoolValue(DEFAULT_SECTION, "bRenderInterpolation", true);
        renderDelayMs = std::clamp(readFloat(ini, "fRenderDelayMs", 12.0), 0.0f, 50.0f);
        eyeLiftCm = std::clamp(readFloat(ini, "fEyeLiftCm", 15.0), -50.0f, 50.0f);
        torqueScale = std::clamp(readFloat(ini, "fTorqueScale", 1.3), 0.2f, 5.0f);
        costReport = ini.GetBoolValue(DEFAULT_SECTION, "bCostReport", true);
        hangWatchdog = ini.GetBoolValue(DEFAULT_SECTION, "bHangWatchdog", true);
        hangWatchdogSeconds = static_cast<float>(ini.GetDoubleValue(DEFAULT_SECTION, "fHangWatchdogSeconds", 4.0));
        logPlayerBody = ini.GetBoolValue(DEFAULT_SECTION, "bLogPlayerBody", true);
        logMotoDiag = ini.GetBoolValue(DEFAULT_SECTION, "bLogMotoDiag", true);
        impactLogMps = (std::max)(readFloat(ini, "fImpactLogMps", 2.0), 0.0f);
        occupantProtection = static_cast<int>(ini.GetLongValue(DEFAULT_SECTION, "iOccupantProtection", 1));
        logHitProbe = ini.GetBoolValue(DEFAULT_SECTION, "bLogHitProbe", true);
        logAIProbe = ini.GetBoolValue(DEFAULT_SECTION, "bLogAIProbe", true);
        engageWhileDriving = ini.GetBoolValue(DEFAULT_SECTION, "bEngageWhileDriving", true);
        logVitals = ini.GetBoolValue(DEFAULT_SECTION, "bLogVitals", true);
        vehicleHealth = ini.GetBoolValue(DEFAULT_SECTION, "bVehicleHealth", true);
        vehicleDamageScale = std::clamp(readFloat(ini, "fVehicleDamageScale", 1.0), 0.0f, 100.0f);
        playerLevelFactor = std::clamp(readFloat(ini, "fPlayerLevelFactor", 0.02), 0.0f, 10.0f);
        playerLevelCap = (std::max)(readFloat(ini, "fPlayerLevelCap", 3.0), 1.0f);
        attackerLevelFactor = std::clamp(readFloat(ini, "fAttackerLevelFactor", 0.01), 0.0f, 10.0f);
        attackerLevelCap = (std::max)(readFloat(ini, "fAttackerLevelCap", 2.0), 1.0f);
        vehicleDamageBase = std::clamp(static_cast<int>(ini.GetLongValue(DEFAULT_SECTION, "iVehicleDamageBase", 0)), 0, 1);
        rememberVehicleState = ini.GetBoolValue(DEFAULT_SECTION, "bRememberVehicleState", true);
        damageSmoke = ini.GetBoolValue(DEFAULT_SECTION, "bDamageSmoke", true);
        battery = ini.GetBoolValue(DEFAULT_SECTION, "bBattery", true);
        batteryScale = std::clamp(readFloat(ini, "fBatteryScale", 1.0), 0.0f, 100.0f);
        chargeScale = std::clamp(readFloat(ini, "fChargeScale", 1.0), 0.0f, 100.0f);
        chargeRadius = std::clamp(readFloat(ini, "fChargeRadius", 420.0), 50.0f, 5000.0f);
        coreCharge = std::clamp(readFloat(ini, "fCoreCharge", 0.15), 0.0f, 1.0f);
        repairBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sRepairBinding",
            vrcf::InputBinding{ .hand = vrcf::Hand::Primary, .type = vrcf::ActivationType::HoldDown, .button = vr::k_EButton_A, .suppress = true });
        turrets = ini.GetBoolValue(DEFAULT_SECTION, "bTurrets", true);
        turretFireBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sTurretFireBinding",
            vrcf::InputBinding{ .hand = vrcf::Hand::Primary, .type = vrcf::ActivationType::HoldDown, .button = vr::k_EButton_Grip, .suppress = true });
        turretMenuBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sTurretMenuBinding",
            vrcf::InputBinding{ .hand = vrcf::Hand::Offhand, .type = vrcf::ActivationType::LongPress, .button = vr::k_EButton_Grip, .suppress = true });
        turretsFolder = ini.GetValue(DEFAULT_SECTION, "sTurretsFolder", "Data\\F4SE\\Plugins\\VehicleFrameworkVR\\Turrets");
        turretAimDistance = std::clamp(readFloat(ini, "fTurretAimDistance", 21000.0), 700.0f, 70000.0f);
        turretFireSound = ini.GetBoolValue(DEFAULT_SECTION, "bTurretFireSound", true);
        turretSight = ini.GetBoolValue(DEFAULT_SECTION, "bTurretSight", true);
        turretFreeKit = ini.GetBoolValue(DEFAULT_SECTION, "bTurretFreeKit", false);
        turretMenuAfterSpawn = ini.GetBoolValue(DEFAULT_SECTION, "bTurretMenuAfterSpawn", true);
        logTurret = ini.GetBoolValue(DEFAULT_SECTION, "bLogTurret", true);
    }
}
