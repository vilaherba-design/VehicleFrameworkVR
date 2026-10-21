#pragma once

#include <cstdint>
#include <string>

namespace vfvr
{
    // What the driving HUD shows; the vehicle fills it (Vehicle::hudState). Actions (nitro, rockets...) and weapons do not exist yet: only the
    // speed is real, and the two icons show dimmed with a dash while hasAction / hasWeapon are false.
    struct HudState
    {
        enum class AimMode : std::uint8_t
        {
            Auto, // the weapon aims by itself
            Manual, // the weapon aims where the camera looks
        };

        float speedKmh = 0.0f;
        bool hasAction = false;
        std::string actionName;
        int actionCharges = 0;
        bool hasWeapon = false;
        std::string weaponName;
        int ammo = 0;
        AimMode aimMode = AimMode::Auto;

        // Health system (bVehicleHealth): the vehicle's health and radiation shielding, the player's own health, rads and action points, each
        // 0..1 (the shares the bars fill) with the game's numbers next to them for the labels.
        bool healthShown = false;
        float vehicleHealth = 1.0f;
        bool vehicleDamaged = false; // under damage.damagedBelow
        bool vehicleDisabled = false;
        bool hasShield = false; // the vehicle has radiation shielding at all
        float shield = 0.0f; // reserve left
        bool shieldActive = false; // blocking radiation right now
        float playerHealth = 1.0f; // of the full maximum (before the rads take their share of it)
        float playerHealthPoints = 0.0f;
        float playerRads = 0.0f; // of the lethal 1000: the share of the health bar the game paints red
        float playerRadsPoints = 0.0f;
        float playerActionPoints = 1.0f; // of the maximum
        float playerActionPointsValue = 0.0f;
    };
}
