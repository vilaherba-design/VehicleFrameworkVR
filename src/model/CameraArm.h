#pragma once

// The chase camera's arm: how far behind its pivot the camera may sit this frame. A ray from the pivot toward where the camera wants to be finds
// walls, rocks and the ground; the camera then stops short of the hit. Pure logic (no game types), shared by the plugin and the offline tests.

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace vfvr::model
{
    // Collision layers 48 and up are not the game's own: mods add them (ROCK's physical hands and body, FRIK). The camera ray skips them, so the
    // player's own hands at the seat do not pull the camera in; walls, rocks, trees and the ground (layers below 48) still stop it.
    inline std::uint64_t cameraIgnoreMask(const std::uint64_t baseMask)
    {
        return baseMask | (~std::uint64_t{ 0 } << 48);
    }

    // The ray starts this far from the pivot above the seat (m), past anything the player holds there.
    inline constexpr float CAMERA_RAY_START = 0.3f;

    // The arm length the camera may have: `wanted`, or the distance to the hit less `margin` (hitDistance < 0 = nothing in the way), never shorter
    // than `minDistance` (nor longer than `wanted`).
    inline float cameraArmTarget(const float wanted, const float hitDistance, const float margin, const float minDistance)
    {
        if (hitDistance < 0.0f) {
            return wanted;
        }
        const float shortest = (std::min)(minDistance, wanted);
        return std::clamp(hitDistance - margin, shortest, wanted);
    }

    // Moves the arm toward `target`: quickly when it must come in (the camera must not stay inside a wall: `inSeconds`), slowly when it may go back
    // out (`outSeconds`), so a post or a tree passing between the camera and the car does not make it jump in and out.
    inline float smoothCameraArm(const float current, const float target, const float dt, const float inSeconds, const float outSeconds)
    {
        const float tau = target < current ? inSeconds : outSeconds;
        const float alpha = tau > 0.0f ? 1.0f - std::exp(-dt / tau) : 1.0f;
        return current + (target - current) * alpha;
    }
}
