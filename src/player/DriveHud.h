#pragma once

#include <chrono>
#include <memory>

#include "model/VMath.h"
#include "vehicle/HudState.h"

namespace f4cf::render
{
    class Texture;
}

namespace vfvr
{
    /**
     * Speedometer with the action and weapon icons and, under them in the same panel, two rows of bars (bVehicleHealth): the vehicle's health
     * and radiation shielding, then the player's health (with the rads in red from the right, as the game's own HUD paints them) and action
     * points, in the game's HUD colour (iHUDColorR/G/B of the game's INI). Drawn on the framework's overlay (distance-field text, images and
     * quads) while driving: about a metre in front of the eyes and below the line of sight, in first person and behind the chase camera alike.
     *
     * In the seat the panel follows the head's heading and height with a low-pass filter, measured in the vehicle's frame: it rides with the car
     * rigidly at any speed and only lags the player's own head movements. Behind the car it stays put in the view of the chase camera (only its
     * heading is softened), lower down. Nothing is added to the scene graph.
     */
    class DriveHud
    {
    public:
        // Draw this frame's HUD or clear it (`show` false). `eyeWorld` / `eyeHeading` are where the seat put the player's eyes this frame (meters, rad),
        // `vehiclePose` the vehicle frame it placed them from, `thirdPerson` whether that is the chase camera. [game]
        void update(bool show, bool thirdPerson, const model::Pose& vehiclePose, const model::Vec3& eyeWorld, float eyeHeading, const HudState& state);

    private:
        bool _shown = false;
        bool _filterReady = false;
        model::Vec3 _headLocal{}; // filtered head position in the vehicle frame (m)
        float _yawLocal = 0.0f; // filtered head heading relative to the vehicle's (rad)
        float _yawWorld = 0.0f; // third person: filtered camera heading in the world (rad)
        bool _thirdPerson = false;
        std::chrono::steady_clock::time_point _last{};
        std::shared_ptr<f4cf::render::Texture> _actionIcon;
        std::shared_ptr<f4cf::render::Texture> _weaponIcon;
    };
}
