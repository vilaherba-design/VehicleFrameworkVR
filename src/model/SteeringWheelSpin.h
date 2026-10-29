#pragma once

#include <cmath>

#include "model/VMath.h"

namespace vfvr::model
{
    // A car's steering wheel turning with the road wheels: a spin about its column, which leans back towards the driver `tiltDeg` up from the
    // horizontal (vehicle frame: x right, y forward, z up). A right turn (positive road wheel angle) turns the rim clockwise as the driver sees it.
    inline Vec3 steeringColumn(const float tiltDeg)
    {
        const float t = tiltDeg * PI / 180.0f;
        return { 0.0f, -std::cos(t), std::sin(t) };
    }

    inline Quat steeringWheelSpin(const float tiltDeg, const float roadWheelAngle, const float ratio)
    {
        return Quat::fromAxisAngle(steeringColumn(tiltDeg), -roadWheelAngle * ratio);
    }

    // Where a point of the wheel goes when the wheel spins by `spin` about its own centre (the rim centre stays put).
    inline Vec3 spinAboutCentre(const Quat& spin, const Vec3& point, const Vec3& centre)
    {
        return spin.rotate(point - centre) + centre;
    }
}
