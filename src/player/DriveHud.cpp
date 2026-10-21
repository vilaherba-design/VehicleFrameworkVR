#include "player/DriveHud.h"

#include <cmath>
#include <format>

#include "Config.h"
#include "perf/Perf.h"
#include "physics/HavokBridge.h"
#include "render/PrimitiveDrawRenderer.h"
#include "render/Texture.h"

namespace vfvr
{
    namespace
    {
        constexpr auto ACTION_ICON = "VehicleFrameworkVR\\Hud\\action.dds";
        constexpr auto WEAPON_ICON = "VehicleFrameworkVR\\Hud\\weapon.dds";
        constexpr auto DASH = "-"; // the font atlas holds ASCII and Latin-1 only

        // Panel layout at fHudScale 1, in meters on the panel plane (x right, y up from its centre).
        constexpr float PANEL_HALF_WIDTH = 0.21f;
        constexpr float PANEL_HALF_HEIGHT = 0.065f;
        constexpr float SPEED_HEIGHT = 0.045f; // capital height of the speed digits
        constexpr float UNIT_HEIGHT = 0.014f;
        constexpr float ICON_SIZE = 0.05f;
        constexpr float ICON_X = 0.14f; // icon centres left and right of the speed
        constexpr float LABEL_HEIGHT = 0.012f;
        // The bars under the speed (bVehicleHealth): the same panel grows down to PANEL_BOTTOM for two rows of two bars, the vehicle's (health
        // left, radiation shielding right) over the player's (health left, action points right), laid out like the game's own HUD: the left bars
        // fill from the left, the right ones from the right, each with its label above it.
        constexpr float PANEL_BOTTOM = -0.112f;
        constexpr float BAR_HEIGHT = 0.011f;
        constexpr float BAR_INNER_X = 0.018f; // the bars run from this far off the centre line out to BAR_OUTER_X
        constexpr float BAR_OUTER_X = 0.195f;
        constexpr float BAR_LABEL_HEIGHT = 0.010f;
        constexpr float ROW_LABEL_Y[2] = { -0.060f, -0.088f }; // centres of the label rows
        constexpr float ROW_BAR_Y[2] = { -0.0745f, -0.1025f }; // centres of the bar rows
        // The game's HUD colour (Fallout4Prefs.ini [Interface] iHUDColorR/G/B), green unless the player changed it, read once from the game's
        // INI settings; and the game's red for the rads, the disabled vehicle and what is nearly gone.
        constexpr int HUD_COLOR_DEFAULT[3] = { 18, 255, 21 };
        constexpr std::uint64_t INI_PREFS_ID = 767844; // INIPrefSettingCollection
        constexpr std::uint64_t INI_SETTINGS_ID = 791183; // INISettingCollection
        constexpr float SHIELD_LOW = 0.2f;
        constexpr float PLAYER_HEALTH_LOW = 0.25f;

        render::PrimitiveDrawRenderer& hudLayer()
        {
            static render::PrimitiveDrawRenderer instance("VehicleFrameworkVR HUD", render::DRAW_ORDER_DEFAULT, false);
            return instance;
        }

        RE::NiPoint3 toPoint(const model::Vec3& v)
        {
            return { v.x, v.y, v.z };
        }

        // The game's HUD colour, from its INI settings (both collections, the preferences first), or the game's default.
        render::Color hudColor()
        {
            static const render::Color color = [] {
                int rgb[3] = { HUD_COLOR_DEFAULT[0], HUD_COLOR_DEFAULT[1], HUD_COLOR_DEFAULT[2] };
                if (physics::hasAddressId(INI_PREFS_ID) && physics::hasAddressId(INI_SETTINGS_ID)) {
                    constexpr const char* names[3] = { "iHUDColorR:Interface", "iHUDColorG:Interface", "iHUDColorB:Interface" };
                    int found = 0;
                    for (int i = 0; i < 3; ++i) {
                        const auto* setting = RE::GetINISetting(names[i]);
                        if (setting && setting->GetType() == RE::Setting::SETTING_TYPE::kInt) {
                            rgb[i] = std::clamp(setting->GetInt(), 0, 255);
                            ++found;
                        }
                    }
                    logger::info("DriveHud: the game's HUD colour is ({}, {}, {}){}", rgb[0], rgb[1], rgb[2], found == 3 ? "" : " (not all of iHUDColorR/G/B:Interface found: the default)");
                } else {
                    logger::warn("DriveHud: the INI setting collections (address ids {}, {}) are not in the VR address library: the default HUD colour", INI_PREFS_ID, INI_SETTINGS_ID);
                }
                return render::Color::rgba(rgb[0], rgb[1], rgb[2], 255);
            }();
            return color;
        }
    }

    void DriveHud::update(const bool show, const bool thirdPerson, const model::Pose& vehiclePose, const model::Vec3& eyeWorld, const float eyeHeading,
        const HudState& state)
    {
        F4CF_PERF_SCOPE("drive hud");
        if (!show || !g_config.hud) {
            if (_shown) {
                hudLayer().publish({}); // an empty frame puts the layer to sleep
                _shown = false;
            }
            _filterReady = false;
            return;
        }
        if (thirdPerson != _thirdPerson) {
            _filterReady = false; // the two views filter different things
            _thirdPerson = thirdPerson;
        }

        const auto now = std::chrono::steady_clock::now();
        const float dt = _filterReady ? std::clamp(std::chrono::duration<float>(now - _last).count(), 0.0f, 0.1f) : 0.0f;
        _last = now;
        const float tau = g_config.hudFollowSeconds;
        const float alpha = tau > 0.0f ? 1.0f - std::exp(-dt / tau) : 1.0f;
        model::Vec3 head;
        float heading;
        if (thirdPerson) {
            // Chase camera: the panel stays put in the view. The camera already rides level behind the car, so it is used as it is (in the vehicle's
            // frame its 11 m lever arm swung the panel with every pitch of the car); only the heading is softened.
            if (!_filterReady) {
                _yawWorld = eyeHeading;
                _filterReady = true;
            } else {
                _yawWorld = model::wrapAngle(_yawWorld + model::wrapAngle(eyeHeading - _yawWorld) * alpha);
            }
            head = eyeWorld;
            heading = _yawWorld;
        } else {
            // In the seat: the eyes in the vehicle's frame, low-passed, so the panel rides with the car and only lags the player's own head movements.
            // The eyes are where the seat put them this frame, not the head node, which the engine moves only on its next update (a frame behind).
            const float vehicleHeading = model::headingOf(vehiclePose.rot.axisY());
            const model::Vec3 headLocal = vehiclePose.toLocal(eyeWorld);
            const float yawLocal = model::wrapAngle(eyeHeading - vehicleHeading);
            if (!_filterReady) {
                _headLocal = headLocal;
                _yawLocal = yawLocal;
                _filterReady = true;
            } else {
                _headLocal = _headLocal + (headLocal - _headLocal) * alpha;
                _yawLocal = model::wrapAngle(_yawLocal + model::wrapAngle(yawLocal - _yawLocal) * alpha);
            }
            head = vehiclePose.toWorld(_headLocal);
            heading = vehicleHeading + _yawLocal;
        }

        // The panel: fHudDistance in front of the eyes, fHudDownAngle below the level line of sight (their ...ThirdPerson values behind the car),
        // facing the eyes.
        const float distance = thirdPerson ? g_config.hudDistanceThirdPerson : g_config.hudDistance;
        const float down = (thirdPerson ? g_config.hudDownAngleThirdPerson : g_config.hudDownAngle) * model::PI / 180.0f;
        const model::Vec3 forward{ std::sin(heading), std::cos(heading), 0.0f };
        const model::Vec3 right{ std::cos(heading), -std::sin(heading), 0.0f };
        const model::Vec3 worldUp{ 0.0f, 0.0f, 1.0f };
        const model::Vec3 toPanel = forward * std::cos(down) - worldUp * std::sin(down);
        const model::Vec3 up = worldUp * std::cos(down) + forward * std::sin(down);
        const model::Vec3 center = head + toPanel * distance;

        // Everything below in game units on the panel plane.
        const float unit = g_config.hudScale * model::HAVOK_TO_GAME;
        const auto at = [&](const float x, const float y) {
            return physics::toGame(center + right * (x * g_config.hudScale) + up * (y * g_config.hudScale));
        };
        const RE::NiPoint3 r = toPoint(right);
        const RE::NiPoint3 u = toPoint(up);
        const RE::NiPoint3 anchor = physics::toGame(center);
        // Oriented text hangs from the top of its capitals: centring a row of capital height h on y means anchoring it at y + h / 2.
        const auto text = [&](render::PrimitiveDraw& frame, const std::string& s, const float x, const float y, const float h, const render::Color& color) {
            frame.addOrientedText(s, anchor, r, u, h * unit, color, render::TextAlign::Center, x * unit, (y + h * 0.5f) * unit);
        };

        render::PrimitiveDraw frame;
        frame.viewerPosition = physics::toGame(head);
        const auto bright = render::Color::rgba(235, 240, 245, 255);
        const auto dim = render::Color::rgba(150, 155, 160, 110);
        const auto amber = render::Color::rgba(255, 190, 70, 255);
        const float panelBottom = state.healthShown ? PANEL_BOTTOM : -PANEL_HALF_HEIGHT;
        frame.addQuad(at(-PANEL_HALF_WIDTH, PANEL_HALF_HEIGHT), at(PANEL_HALF_WIDTH, PANEL_HALF_HEIGHT), at(PANEL_HALF_WIDTH, panelBottom),
            at(-PANEL_HALF_WIDTH, panelBottom), render::Color::rgba(8, 10, 12, 130));

        text(frame, std::format("{}", static_cast<int>(std::lround(std::abs(state.speedKmh)))), 0.0f, 0.012f, SPEED_HEIGHT, bright);
        text(frame, "km/h", 0.0f, -0.040f, UNIT_HEIGHT, bright);

        if (!_actionIcon) {
            _actionIcon = render::Texture::load(ACTION_ICON);
            _weaponIcon = render::Texture::load(WEAPON_ICON);
        }
        const auto icon = [&](const std::shared_ptr<render::Texture>& texture, const float x, const render::Color& tint) {
            if (!texture) {
                return;
            }
            const auto view = texture->view();
            if (!view) {
                return;
            }
            const float h = ICON_SIZE * 0.5f;
            frame.addImage(view, texture->isSRGB(), at(x - h, 0.012f + h), at(x + h, 0.012f + h), at(x + h, 0.012f - h), at(x - h, 0.012f - h), tint);
        };

        // Left: the action (nitro, rockets...) and its charges.
        icon(_actionIcon, -ICON_X, state.hasAction ? amber : dim);
        text(frame, state.hasAction ? std::format("{} {}", state.actionName, state.actionCharges) : DASH, -ICON_X, -0.040f, LABEL_HEIGHT, state.hasAction ? bright : dim);

        // Right: the weapon, its ammunition and how it aims.
        icon(_weaponIcon, ICON_X, state.hasWeapon ? amber : dim);
        const char* aim = state.aimMode == HudState::AimMode::Auto ? "AUTO" : "MANUAL";
        text(frame, state.hasWeapon ? std::format("{}  {}", state.ammo, aim) : DASH, ICON_X, -0.040f, LABEL_HEIGHT, state.hasWeapon ? bright : dim);

        // Below, in the same panel: the vehicle's health and radiation shielding, then the player's health (the rads in red from the right, as
        // the game's HUD paints them) and action points, all the game's own values, in the game's HUD colour.
        if (state.healthShown) {
            const auto hud = hudColor();
            const auto shade = [&](const int alpha) { return render::Color{ hud.r, hud.g, hud.b, static_cast<float>(alpha) / 255.0f }; };
            const auto hudDim = shade(110);
            const auto track = shade(40);
            const auto outline = shade(150);
            const auto red = render::Color::rgba(255, 45, 35, 255);
            const auto label = [&](const std::string& s, const float x, const float y, const render::TextAlign align, const render::Color& color) {
                frame.addOrientedText(s, anchor, r, u, BAR_LABEL_HEIGHT * unit, color, align, x * unit, (y + BAR_LABEL_HEIGHT * 0.5f) * unit);
            };
            // A bar on `side` (-1 left, +1 right) of `row`, filled from the panel's edge inwards by `value` (0..1) in `fill`, and, on the left bars,
            // `fromRight` of it painted `rightFill` from the inner end (the rads).
            const auto bar = [&](const int row, const int side, const float value, const render::Color& fill, const float fromRight = 0.0f,
                              const render::Color& rightFill = render::Color::rgba(0, 0, 0, 0)) {
                const float x0 = side < 0 ? -BAR_OUTER_X : BAR_INNER_X; // left edge
                const float x1 = side < 0 ? -BAR_INNER_X : BAR_OUTER_X; // right edge
                const float top = ROW_BAR_Y[row] + BAR_HEIGHT * 0.5f;
                const float bottom = ROW_BAR_Y[row] - BAR_HEIGHT * 0.5f;
                const float width = x1 - x0;
                frame.addQuad(at(x0, top), at(x1, top), at(x1, bottom), at(x0, bottom), track);
                const float rightShare = std::clamp(fromRight, 0.0f, 1.0f);
                const float v = std::clamp(value, 0.0f, 1.0f - rightShare);
                if (v > 0.0f) {
                    // The left bars fill from the left, the right bars from the right (the game's AP bar hangs from its right end).
                    const float xa = side < 0 ? x0 : x1 - width * v;
                    const float xb = side < 0 ? x0 + width * v : x1;
                    frame.addQuad(at(xa, top), at(xb, top), at(xb, bottom), at(xa, bottom), fill);
                }
                if (rightShare > 0.0f) {
                    const float xa = x1 - width * rightShare;
                    frame.addQuad(at(xa, top), at(x1, top), at(x1, bottom), at(xa, bottom), rightFill);
                }
                frame.addLine(at(x0, top), at(x1, top), outline);
                frame.addLine(at(x1, top), at(x1, bottom), outline);
                frame.addLine(at(x1, bottom), at(x0, bottom), outline);
                frame.addLine(at(x0, bottom), at(x0, top), outline);
            };

            // Row 1: the vehicle. Its health, red when low (under damage.damagedBelow) or gone; its radiation shielding, red when nearly used up.
            const bool vehicleLow = state.vehicleDisabled || state.vehicleDamaged;
            bar(0, -1, state.vehicleDisabled ? 0.0f : state.vehicleHealth, vehicleLow ? red : hud);
            label(state.vehicleDisabled ? "VEHICLE DISABLED" : std::format("VEHICLE {}%", static_cast<int>(std::lround(state.vehicleHealth * 100.0f))), -BAR_OUTER_X,
                ROW_LABEL_Y[0], render::TextAlign::Left, vehicleLow ? red : hud);
            if (state.hasShield) {
                const bool shieldLow = state.shield < SHIELD_LOW;
                bar(0, 1, state.shield, shieldLow ? red : hud);
                label(std::format("{} {}%", state.shieldActive ? "SHIELDING" : "SHIELD", static_cast<int>(std::lround(state.shield * 100.0f))), BAR_OUTER_X, ROW_LABEL_Y[0],
                    render::TextAlign::Right, shieldLow ? red : hud);
            } else {
                bar(0, 1, 0.0f, hud);
                label("NO SHIELD", BAR_OUTER_X, ROW_LABEL_Y[0], render::TextAlign::Right, hudDim);
            }

            // Row 2: the player, as the game's own HUD: HP from the left with the rads in red from the right, AP from the right.
            const bool healthLow = state.playerHealth < PLAYER_HEALTH_LOW;
            bar(1, -1, state.playerHealth, healthLow ? red : hud, state.playerRads, red);
            label(std::format("HP {}", static_cast<int>(std::lround(state.playerHealthPoints))), -BAR_OUTER_X, ROW_LABEL_Y[1], render::TextAlign::Left, healthLow ? red : hud);
            if (state.playerRadsPoints >= 1.0f) {
                label(std::format("RADS {}", static_cast<int>(std::lround(state.playerRadsPoints))), -BAR_INNER_X, ROW_LABEL_Y[1], render::TextAlign::Right, red);
            }
            bar(1, 1, state.playerActionPoints, hud);
            label(std::format("AP {}", static_cast<int>(std::lround(state.playerActionPointsValue))), BAR_OUTER_X, ROW_LABEL_Y[1], render::TextAlign::Right, hud);
        }

        hudLayer().ensureInstalled();
        hudLayer().publish(std::move(frame));
        _shown = true;
    }
}
