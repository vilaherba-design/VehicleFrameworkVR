#pragma once

#include <cmath>
#include <vector>

namespace vfvr::model
{
    // A button of the menu panel in the panel's own plane: centre and half size, in game units, x to the right and z up from the panel's centre.
    struct MenuRect
    {
        float x = 0.0f;
        float z = 0.0f;
        float halfWidth = 0.0f;
        float halfHeight = 0.0f;
        bool enabled = true;
    };

    // Which button a point of the panel's plane is on: the one whose rectangle holds it, widened by `margin` on every side; where widened
    // rectangles overlap, the one whose centre is nearest. -1 for none. At most one button, whatever the spacing.
    inline int hitMenuRect(const std::vector<MenuRect>& rects, const float x, const float z, const float margin = 0.0f)
    {
        int best = -1;
        float bestDistance = 0.0f;
        for (int i = 0; i < static_cast<int>(rects.size()); ++i) {
            const auto& r = rects[static_cast<std::size_t>(i)];
            if (!r.enabled || std::abs(x - r.x) > r.halfWidth + margin || std::abs(z - r.z) > r.halfHeight + margin) {
                continue;
            }
            const float distance = std::hypot(x - r.x, z - r.z);
            if (best < 0 || distance < bestDistance) {
                best = i;
                bestDistance = distance;
            }
        }
        return best;
    }

    // Where a ray meets the panel's plane (the plane y = planeY of the panel's frame, facing -y): true with the point's x and z when it meets it
    // ahead of the origin, coming from the front.
    inline bool rayOnMenuPlane(const float ox, const float oy, const float oz, const float dx, const float dy, const float dz, const float planeY, float& x, float& z)
    {
        if (dy <= 1e-4f || oy >= planeY) {
            return false;
        }
        const float t = (planeY - oy) / dy;
        x = ox + dx * t;
        z = oz + dz * t;
        return true;
    }

    // One fingertip pressing the panel's buttons. `depth` is how far the tip is past the panel's plane (negative = in front of it, on the
    // player's side). A button is pressed once when the tip, having been in front of that same button, pushes `pressDepth` past the plane; the
    // finger has to come back `releaseDepth` in front of the plane, or leave the button, before it presses again. A hand arriving from behind
    // the panel, or sliding across it while pushed in, presses nothing.
    struct MenuTouch
    {
        float hoverFront = 2.1f; // in front of the plane, the tip hovers a button (game units: 3 cm)
        float hoverBehind = 5.0f; // past the plane it still counts as on the button (7 cm), deeper it has gone through
        float pressDepth = 0.6f; // pushed this far past the plane, it presses (about 1 cm)
        float releaseDepth = 0.7f; // and comes back this far in front to press again

        int frontOf = -1; // the button the tip was last in front of
        bool armed = true;

        // The button the tip hovers this frame (-1 for none): the one under it while it is near the plane.
        static int hovered(const int under, const float depth, const float front, const float behind)
        {
            return under >= 0 && depth >= -front && depth <= behind ? under : -1;
        }

        // Feed this frame's button under the tip (hitMenuRect, -1 for none) and its depth; returns the button pressed now, or -1.
        int update(const int under, const float depth)
        {
            const int hover = hovered(under, depth, hoverFront, hoverBehind);
            if (!armed) {
                if (hover < 0 || depth < -releaseDepth) {
                    armed = true;
                    frontOf = hover >= 0 && depth < 0.0f ? hover : -1;
                }
                return -1;
            }
            if (hover < 0) {
                frontOf = -1;
                return -1;
            }
            if (depth < 0.0f) {
                frontOf = hover;
                return -1;
            }
            if (hover != frontOf) {
                frontOf = -1; // came through the plane from elsewhere (behind, or from another button while pushed in)
                return -1;
            }
            if (depth >= pressDepth) {
                armed = false;
                frontOf = -1;
                return hover;
            }
            return -1;
        }
    };

    // A trigger as a button: pressed once when pulled past `on`, again only after it has come back under `off`. Starts disarmed while held,
    // so a trigger already pulled when the menu opens presses nothing.
    struct MenuTrigger
    {
        float on = 0.7f;
        float off = 0.3f;
        bool down = true;

        bool update(const float value)
        {
            if (down) {
                down = value > off;
                return false;
            }
            if (value >= on) {
                down = true;
                return true;
            }
            return false;
        }
    };
}
