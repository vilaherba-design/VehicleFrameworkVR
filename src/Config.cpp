#include "Config.h"

#include <algorithm>

#include "f4vr/CollisionLayers.h"

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
    void Config::loadIniConfigInternal(const CSimpleIniA& ini)
    {
        vehiclesFolder = ini.GetValue(DEFAULT_SECTION, "sVehiclesFolder", "Data\\F4SE\\Plugins\\VehicleFrameworkVR\\Vehicles");
        testVehicleProfile = ini.GetValue(DEFAULT_SECTION, "sTestVehicleProfile", "sedan");
        spawnDistance = readFloat(ini, "fSpawnDistance", 6.0);
        simpleMode = ini.GetBoolValue(DEFAULT_SECTION, "bSimpleMode", false);
        impulseApi = readEnum(ini, "iImpulseApi", ImpulseApi::HknpWorld, 1);
        wheelRayLayer = ini.GetValue(DEFAULT_SECTION, "sWheelRayLayer", "charController");
        vehicleBodyLayer = ini.GetValue(DEFAULT_SECTION, "sVehicleBodyLayer", "charController");
        parkedBodyLayer = ini.GetValue(DEFAULT_SECTION, "sParkedBodyLayer", "");
        wheelRayIgnoreMask = f4vr::parseCollisionLayerMask(ini.GetValue(DEFAULT_SECTION, "sWheelRayIgnoreLayers", DEFAULT_RAY_IGNORE));
        releasePickWorldReference = ini.GetBoolValue(DEFAULT_SECTION, "bReleasePickWorldReference", true);
        wheelSet = ini.GetValue(DEFAULT_SECTION, "sWheelSet", "stock");
        wheelMenuOnEnter = ini.GetBoolValue(DEFAULT_SECTION, "bWheelMenuOnEnter", false);
        wheelMenuAfterSpawn = ini.GetBoolValue(DEFAULT_SECTION, "bWheelMenuAfterSpawn", true);
        exhaustSmoke = ini.GetBoolValue(DEFAULT_SECTION, "bExhaustSmoke", true);
        engineSound = ini.GetBoolValue(DEFAULT_SECTION, "bEngineSound", true);
        engineVolume = static_cast<float>(ini.GetDoubleValue(DEFAULT_SECTION, "fEngineVolume", 0.6));
        vehicleProfiles = ini.GetValue(DEFAULT_SECTION, "sVehicleProfiles", "sedan, pickup, rockboy, rusianmother");

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
        driverCollisionLayer = ini.GetValue(DEFAULT_SECTION, "sDriverCollisionLayer", "nonCollidable");
        blockFrikSmoothMovement = ini.GetBoolValue(DEFAULT_SECTION, "bBlockFrikSmoothMovement", true);

        spawnTestVehicleBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sSpawnTestVehicleBinding",
            vrcf::InputBinding{ vrcf::Hand::Primary, vrcf::ActivationType::LongPress, vr::k_EButton_ApplicationMenu });
        enterBinding = getInputBindingValue(ini, DEFAULT_SECTION, "sEnterBinding", vrcf::InputBinding{ .hand = vrcf::Hand::Primary, .type = vrcf::ActivationType::Press, .button = vr::k_EButton_A, .suppress = true });
        trunkBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sTrunkBinding",
            vrcf::InputBinding{ .hand = vrcf::Hand::Primary, .type = vrcf::ActivationType::Press, .button = vr::k_EButton_A, .suppress = true });
        exitBinding =
            getInputBindingValue(ini, DEFAULT_SECTION, "sExitBinding", vrcf::InputBinding{ vrcf::Hand::Offhand, vrcf::ActivationType::LongPress, vr::k_EButton_ApplicationMenu });
        recenterBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sRecenterBinding",
            vrcf::InputBinding{ vrcf::Hand::Primary, vrcf::ActivationType::Tap, vr::k_EButton_SteamVR_Touchpad });
        lightsBinding = getInputBindingValue(ini, DEFAULT_SECTION, "sLightsBinding", vrcf::InputBinding{ vrcf::Hand::Primary, vrcf::ActivationType::LongPress, vr::k_EButton_SteamVR_Touchpad });

        thirdPersonBinding = getInputBindingValue(ini, DEFAULT_SECTION, "sThirdPersonBinding", vrcf::InputBinding{ vrcf::Hand::Offhand, vrcf::ActivationType::Press, vr::k_EButton_SteamVR_Touchpad });
        thirdPersonDistance = std::clamp(readFloat(ini, "fThirdPersonDistance", 11.0), 3.0f, 40.0f);
        thirdPersonHeight = std::clamp(readFloat(ini, "fThirdPersonHeight", 3.5), 0.0f, 20.0f);
        thirdPersonOrbitSpeed = std::clamp(readFloat(ini, "fThirdPersonOrbitSpeed", 120.0), 0.0f, 360.0f);
        thirdPersonOrbitInvert = ini.GetBoolValue(DEFAULT_SECTION, "bThirdPersonOrbitInvert", true);
        thirdPersonBodyInSeat = ini.GetBoolValue(DEFAULT_SECTION, "bThirdPersonBodyInSeat", true);
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
        logPlayerBody = ini.GetBoolValue(DEFAULT_SECTION, "bLogPlayerBody", true);
        logMotoDiag = ini.GetBoolValue(DEFAULT_SECTION, "bLogMotoDiag", true);
        impactLogMps = (std::max)(readFloat(ini, "fImpactLogMps", 2.0), 0.0f);
        occupantProtection = static_cast<int>(ini.GetLongValue(DEFAULT_SECTION, "iOccupantProtection", 1));
        logHitProbe = ini.GetBoolValue(DEFAULT_SECTION, "bLogHitProbe", true);
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
        repairBinding = getInputBindingValue(ini,
            DEFAULT_SECTION,
            "sRepairBinding",
            vrcf::InputBinding{ .hand = vrcf::Hand::Primary, .type = vrcf::ActivationType::HoldDown, .button = vr::k_EButton_A, .suppress = true });
    }
}
