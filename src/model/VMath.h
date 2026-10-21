#pragma once

// Small, dependency-free vector math for the vehicle model.
// The model runs in SI units (meters, kilograms, seconds) and in the game's axis convention:
// x = right, y = forward, z = up. Havok space is (almost exactly) meters, so Havok positions and
// velocities convert 1:1; game units convert with HAVOK_TO_GAME.

#include <algorithm>
#include <cmath>

namespace vfvr::model
{
    // 1 Havok unit = 69.99125 game units (CommonLibF4VR HAVOK_PHYSICS_ADDITIONS.md); 1 m ~ 70 game units.
    inline constexpr float HAVOK_TO_GAME = 69.99125f;
    inline constexpr float GAME_TO_HAVOK = 1.0f / HAVOK_TO_GAME;
    inline constexpr float PI = 3.14159265358979f;

    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        constexpr Vec3() = default;

        constexpr Vec3(const float ax, const float ay, const float az)
            : x(ax),
              y(ay),
              z(az)
        {}

        constexpr Vec3 operator+(const Vec3& o) const
        {
            return { x + o.x, y + o.y, z + o.z };
        }

        constexpr Vec3 operator-(const Vec3& o) const
        {
            return { x - o.x, y - o.y, z - o.z };
        }

        constexpr Vec3 operator-() const
        {
            return { -x, -y, -z };
        }

        constexpr Vec3 operator*(const float s) const
        {
            return { x * s, y * s, z * s };
        }

        constexpr Vec3 operator/(const float s) const
        {
            return { x / s, y / s, z / s };
        }

        constexpr Vec3& operator+=(const Vec3& o)
        {
            x += o.x;
            y += o.y;
            z += o.z;
            return *this;
        }

        constexpr Vec3& operator-=(const Vec3& o)
        {
            x -= o.x;
            y -= o.y;
            z -= o.z;
            return *this;
        }

        constexpr Vec3& operator*=(const float s)
        {
            x *= s;
            y *= s;
            z *= s;
            return *this;
        }
    };

    constexpr Vec3 operator*(const float s, const Vec3& v)
    {
        return v * s;
    }

    constexpr float dot(const Vec3& a, const Vec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    constexpr Vec3 cross(const Vec3& a, const Vec3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    inline float length(const Vec3& v)
    {
        return std::sqrt(dot(v, v));
    }

    inline bool isFinite(const Vec3& v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    // Normalized copy, or `fallback` when the vector has no usable direction.
    inline Vec3 normalizeOr(const Vec3& v, const Vec3& fallback)
    {
        const float len = length(v);
        return len > 1e-6f && std::isfinite(len) ? v / len : fallback;
    }

    // Remove the component of `v` along the unit vector `n`.
    constexpr Vec3 projectOnPlane(const Vec3& v, const Vec3& n)
    {
        return v - n * dot(v, n);
    }

    // Unit quaternion, Havok layout {x, y, z, w}. rotate() maps body (local) vectors to world.
    struct Quat
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 1.0f;

        constexpr Quat() = default;

        constexpr Quat(const float ax, const float ay, const float az, const float aw)
            : x(ax),
              y(ay),
              z(az),
              w(aw)
        {}

        static Quat fromAxisAngle(const Vec3& axis, const float radians)
        {
            const Vec3 a = normalizeOr(axis, { 0, 0, 1 });
            const float s = std::sin(radians * 0.5f);
            return { a.x * s, a.y * s, a.z * s, std::cos(radians * 0.5f) };
        }

        // Rotation whose columns are the given world-space images of the local x/y/z axes
        // (a standard local->world rotation matrix given column by column).
        static Quat fromBasis(const Vec3& xAxis, const Vec3& yAxis, const Vec3& zAxis)
        {
            const float m00 = xAxis.x, m10 = xAxis.y, m20 = xAxis.z;
            const float m01 = yAxis.x, m11 = yAxis.y, m21 = yAxis.z;
            const float m02 = zAxis.x, m12 = zAxis.y, m22 = zAxis.z;
            Quat q;
            const float trace = m00 + m11 + m22;
            if (trace > 0.0f) {
                const float s = std::sqrt(trace + 1.0f) * 2.0f;
                q.w = 0.25f * s;
                q.x = (m21 - m12) / s;
                q.y = (m02 - m20) / s;
                q.z = (m10 - m01) / s;
            } else if (m00 > m11 && m00 > m22) {
                const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
                q.w = (m21 - m12) / s;
                q.x = 0.25f * s;
                q.y = (m01 + m10) / s;
                q.z = (m02 + m20) / s;
            } else if (m11 > m22) {
                const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
                q.w = (m02 - m20) / s;
                q.x = (m01 + m10) / s;
                q.y = 0.25f * s;
                q.z = (m12 + m21) / s;
            } else {
                const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
                q.w = (m10 - m01) / s;
                q.x = (m02 + m20) / s;
                q.y = (m12 + m21) / s;
                q.z = 0.25f * s;
            }
            return q.normalized();
        }

        Quat normalized() const
        {
            const float len = std::sqrt(x * x + y * y + z * z + w * w);
            if (!(len > 1e-8f) || !std::isfinite(len)) {
                return {};
            }
            return { x / len, y / len, z / len, w / len };
        }

        constexpr Quat conjugate() const
        {
            return { -x, -y, -z, w };
        }

        constexpr Quat operator*(const Quat& o) const
        {
            return {
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w,
                w * o.w - x * o.x - y * o.y - z * o.z,
            };
        }

        constexpr Vec3 rotate(const Vec3& v) const
        {
            // v' = v + 2w(q x v) + 2 q x (q x v)
            const Vec3 q{ x, y, z };
            const Vec3 t = cross(q, v) * 2.0f;
            return v + t * w + cross(q, t);
        }

        constexpr Vec3 rotateInverse(const Vec3& v) const
        {
            return conjugate().rotate(v);
        }

        constexpr Vec3 axisX() const
        {
            return rotate({ 1, 0, 0 });
        }

        constexpr Vec3 axisY() const
        {
            return rotate({ 0, 1, 0 });
        }

        constexpr Vec3 axisZ() const
        {
            return rotate({ 0, 0, 1 });
        }
    };

    // Rigid placement: world = pos + rot * local.
    struct Pose
    {
        Vec3 pos;
        Quat rot;

        constexpr Vec3 toWorld(const Vec3& local) const
        {
            return pos + rot.rotate(local);
        }

        constexpr Vec3 toLocal(const Vec3& world) const
        {
            return rot.rotateInverse(world - pos);
        }

        constexpr Pose operator*(const Pose& child) const
        {
            return { toWorld(child.pos), rot * child.rot };
        }

        constexpr Pose inverse() const
        {
            const Quat inv = rot.conjugate();
            return { inv.rotate(-pos), inv };
        }
    };

    // The pose with its roll (the lean about the horizontal heading axis, positive = the top toward the right) scaled to `keep` (0..1) about the vehicle
    // origin, which sits on the ground line: what a camera that should only partly follow a motorcycle's lean sees.
    inline Pose scaleRoll(const Pose& pose, const float keep)
    {
        if (keep >= 1.0f) {
            return pose;
        }
        const Vec3 forward = pose.rot.axisY();
        const Vec3 heading = normalizeOr({ forward.x, forward.y, 0.0f }, { 0.0f, 1.0f, 0.0f });
        const float roll = std::atan2(-pose.rot.axisX().z, pose.rot.axisZ().z);
        const Quat undo = Quat::fromAxisAngle(heading, -(1.0f - (std::max)(keep, 0.0f)) * roll);
        return { pose.pos, (undo * pose.rot).normalized() };
    }

    // Heading (yaw) of a forward vector, game convention: 0 = +Y (north), positive = clockwise seen from above.
    inline float headingOf(const Vec3& forward)
    {
        return std::atan2(forward.x, forward.y);
    }

    // Wrap an angle to (-PI, PI].
    inline float wrapAngle(float a)
    {
        while (a > PI) {
            a -= 2.0f * PI;
        }
        while (a <= -PI) {
            a += 2.0f * PI;
        }
        return a;
    }

    inline float clampf(const float v, const float lo, const float hi)
    {
        return (std::min)((std::max)(v, lo), hi);
    }

    inline float lerpf(const float a, const float b, const float t)
    {
        return a + (b - a) * t;
    }

    inline float signf(const float v)
    {
        return v > 0.0f ? 1.0f : (v < 0.0f ? -1.0f : 0.0f);
    }

    // Smooth 0..1 ramp between edge0 and edge1.
    inline float smoothstep(const float edge0, const float edge1, const float v)
    {
        const float t = clampf((v - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }
}
