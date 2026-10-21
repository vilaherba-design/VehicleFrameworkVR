#pragma once

// Third person with the player's body in the driver's seat. The view is the player's own head, so the chase camera moves the whole VR room (actor
// and head) behind the car, and FRIK builds the body there, under the head. Once FRIK has finished the frame, rigid moves take that body to the
// seat: one for the whole body, from the head's frame to the seat's eye frame with the vehicle's full rotation (so it leans, climbs and rolls with
// the car), then one about each hip and knee that bends the standing legs into a seated pose. Pure logic (no game types), shared by the plugin
// and the offline tests.

#include <algorithm>
#include <cmath>

#include "model/VMath.h"

namespace vfvr::model
{
    // The rigid move of the whole body: the head frame FRIK built (the head's position, the body's heading about world up) onto the seat's eye frame
    // (the eye point, the vehicle's full rotation): T = seatEye * inverse(head). The head keeps its own turn relative to the body.
    inline Pose seatBodyTransform(const Pose& seatEye, const Vec3& head, const float bodyHeading)
    {
        const Pose headFrame{ head, Quat::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -bodyHeading) };
        return seatEye * headFrame.inverse();
    }

    // The shortest rotation that takes direction `from` onto direction `to`.
    inline Quat rotationBetween(const Vec3& from, const Vec3& to)
    {
        const Vec3 a = normalizeOr(from, { 0.0f, 0.0f, 1.0f });
        const Vec3 b = normalizeOr(to, { 0.0f, 0.0f, 1.0f });
        const float c = std::clamp(dot(a, b), -1.0f, 1.0f);
        const Vec3 axis = cross(a, b);
        const float s = length(axis);
        if (s < 1e-6f) {
            if (c > 0.0f) {
                return {};
            }
            // Opposite: any axis at right angles.
            const Vec3 other = std::abs(a.x) < 0.9f ? Vec3{ 1.0f, 0.0f, 0.0f } : Vec3{ 0.0f, 1.0f, 0.0f };
            return Quat::fromAxisAngle(normalizeOr(cross(a, other), { 1.0f, 0.0f, 0.0f }), PI);
        }
        return Quat::fromAxisAngle(axis * (1.0f / s), std::atan2(s, c));
    }

    // A rotation `q` about the point `pivot`, as a rigid move.
    inline Pose rotateAbout(const Vec3& pivot, const Quat& q)
    {
        return { pivot - q.rotate(pivot), q };
    }

    // Seated legs, in the vehicle frame (x right, y forward, z up). `side` is -1 for the left leg, +1 for the right. The thigh leans `thighDeg` from
    // straight down toward the front, splayed out by `spreadDeg`; the shin turns `kneeDeg` back down from the thigh at the knee.
    inline Vec3 seatedThighLocal(const float thighDeg, const float spreadDeg, const float side)
    {
        const float t = thighDeg * PI / 180.0f;
        const float s = spreadDeg * PI / 180.0f;
        return { side * std::sin(s) * std::abs(std::sin(t)), std::cos(s) * std::sin(t), -std::cos(t) }; // the splay stays outward when the shin swings back
    }

    inline Vec3 seatedShinLocal(const float thighDeg, const float kneeDeg, const float spreadDeg, const float side)
    {
        return seatedThighLocal(thighDeg - kneeDeg, spreadDeg, side);
    }
}
