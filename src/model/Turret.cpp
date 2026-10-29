#include "model/Turret.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace vfvr::model
{
    namespace
    {
        using json = nlohmann::json;

        constexpr float DEG = PI / 180.0f;

        void readFloat(const json& j, const char* key, float& out)
        {
            if (const auto it = j.find(key); it != j.end()) {
                if (!it->is_number()) {
                    throw std::runtime_error(std::string("'") + key + "' must be a number");
                }
                out = it->get<float>();
            }
        }

        void readInt(const json& j, const char* key, int& out)
        {
            if (const auto it = j.find(key); it != j.end()) {
                if (!it->is_number_integer()) {
                    throw std::runtime_error(std::string("'") + key + "' must be a whole number");
                }
                out = it->get<int>();
            }
        }

        void readString(const json& j, const char* key, std::string& out)
        {
            if (const auto it = j.find(key); it != j.end()) {
                out = it->get<std::string>();
            }
        }

        // A position in game units -> meters.
        void readGamePoint(const json& j, const char* key, Vec3& out)
        {
            if (const auto it = j.find(key); it != j.end()) {
                if (!it->is_array() || it->size() != 3) {
                    throw std::runtime_error(std::string("'") + key + "' must be [x, y, z] in game units");
                }
                out = Vec3{ (*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>() } * GAME_TO_HAVOK;
            }
        }

        // Move `from` towards `to` by at most `step`.
        float approach(const float from, const float to, const float step)
        {
            if (to > from) {
                return (std::min)(to, from + step);
            }
            return (std::max)(to, from - step);
        }
    }

    TurretSpec parseTurretSpec(const std::string& jsonText)
    {
        const json root = json::parse(jsonText);
        TurretSpec spec;
        readString(root, "id", spec.id);
        spec.displayName = spec.id;
        readString(root, "name", spec.displayName);
        readString(root, "mesh", spec.mesh);
        readFloat(root, "meshScale", spec.meshScale);
        readGamePoint(root, "meshOffset", spec.meshOffset);
        if (const auto it = root.find("nodes"); it != root.end()) {
            readString(*it, "yaw", spec.yawNode);
            readString(*it, "pitch", spec.pitchNode);
            if (const auto muzzle = it->find("muzzle"); muzzle != it->end()) {
                if (muzzle->is_array()) {
                    for (const auto& name : *muzzle) {
                        spec.muzzleNodes.push_back(name.get<std::string>());
                    }
                } else {
                    spec.muzzleNodes.push_back(muzzle->get<std::string>());
                }
            }
            readGamePoint(*it, "muzzleOffset", spec.muzzleOffset);
        }
        if (const auto it = root.find("aim"); it != root.end()) {
            // Every turret turns all the way round and 120 degrees up and down (-30..+90): Axel's rule for all of them. Only the speeds and the
            // fire cone are the turret's own.
            readFloat(*it, "yawSpeed", spec.yawSpeedDeg);
            readFloat(*it, "pitchSpeed", spec.pitchSpeedDeg);
            readFloat(*it, "fireCone", spec.fireConeDeg);
        }
        if (const auto it = root.find("weapon"); it != root.end()) {
            readString(*it, "form", spec.weaponForm);
            readString(*it, "projectile", spec.projectileForm);
            readString(*it, "ammo", spec.ammoForm);
            readInt(*it, "ammoPerShot", spec.ammoPerShot);
            readFloat(*it, "roundsPerMinute", spec.roundsPerMinute);
            readFloat(*it, "spread", spec.spreadDeg);
            readString(*it, "sound", spec.fireSound);
            if (const auto loop = it->find("soundLoop"); loop != it->end()) {
                if (!loop->is_boolean()) {
                    throw std::runtime_error("'weapon.soundLoop' must be true or false");
                }
                spec.fireSoundLoop = loop->get<bool>();
            }
        }
        readString(root, "kit", spec.kitForm);

        if (spec.id.empty()) {
            throw std::runtime_error("a turret needs an 'id'");
        }
        if (spec.mesh.empty() || spec.yawNode.empty() || spec.pitchNode.empty()) {
            throw std::runtime_error("a turret needs 'mesh', 'nodes.yaw' and 'nodes.pitch'");
        }
        if (spec.weaponForm.empty() && spec.projectileForm.empty()) {
            throw std::runtime_error("a turret needs 'weapon.form' or 'weapon.projectile'");
        }
        if (spec.yawMinDeg > spec.yawMaxDeg || spec.pitchMinDeg > spec.pitchMaxDeg) {
            throw std::runtime_error("'aim' minimums must not be above the maximums");
        }
        if (spec.yawSpeedDeg <= 0.0f || spec.pitchSpeedDeg <= 0.0f || spec.roundsPerMinute <= 0.0f || spec.meshScale <= 0.0f) {
            throw std::runtime_error("'aim.yawSpeed', 'aim.pitchSpeed', 'weapon.roundsPerMinute' and 'meshScale' must be more than 0");
        }
        if (spec.ammoPerShot < 0 || spec.spreadDeg < 0.0f || spec.fireConeDeg < 0.0f) {
            throw std::runtime_error("'weapon.ammoPerShot', 'weapon.spread' and 'aim.fireCone' must not be negative");
        }
        return spec;
    }

    TurretSpec loadTurretSpecFile(const std::string& path)
    {
        std::ifstream file(path);
        if (!file) {
            throw std::runtime_error("cannot open " + path);
        }
        std::stringstream text;
        text << file.rdbuf();
        return parseTurretSpec(text.str());
    }

    TurretAim::TurretAim(const TurretSpec& spec)
    {
        _freeYaw = spec.yawMinDeg <= -180.0f && spec.yawMaxDeg >= 180.0f;
        _yawMin = (std::max)(spec.yawMinDeg, -180.0f) * DEG;
        _yawMax = (std::min)(spec.yawMaxDeg, 180.0f) * DEG;
        _pitchMin = clampf(spec.pitchMinDeg, -90.0f, 90.0f) * DEG;
        _pitchMax = clampf(spec.pitchMaxDeg, -90.0f, 90.0f) * DEG;
        _yawSpeed = spec.yawSpeedDeg * DEG;
        _pitchSpeed = spec.pitchSpeedDeg * DEG;
        _fireCone = spec.fireConeDeg * DEG;
        _yaw = _freeYaw ? 0.0f : clampf(0.0f, _yawMin, _yawMax);
        _pitch = clampf(0.0f, _pitchMin, _pitchMax);
    }

    void TurretAim::anglesFor(const Vec3& dir, float& yaw, float& pitch) const
    {
        const float flat = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        yaw = flat > 1e-6f ? std::atan2(dir.x, dir.y) : _yaw;
        pitch = std::atan2(dir.z, flat);
        if (!_freeYaw) {
            yaw = clampf(yaw, _yawMin, _yawMax);
        }
        pitch = clampf(pitch, _pitchMin, _pitchMax);
    }

    void TurretAim::stepTowards(const float yaw, const float pitch, const float dt)
    {
        const float yawStep = _yawSpeed * dt;
        if (_freeYaw) {
            const float diff = wrapAngle(yaw - _yaw);
            _yaw = wrapAngle(_yaw + clampf(diff, -yawStep, yawStep));
        } else {
            // Inside the limits the straight way is the only way: the arc beyond them is closed.
            _yaw = approach(_yaw, yaw, yawStep);
        }
        _pitch = approach(_pitch, pitch, _pitchSpeed * dt);
    }

    bool TurretAim::update(const Vec3& dir, const float dt)
    {
        float yaw = 0.0f;
        float pitch = 0.0f;
        anglesFor(dir, yaw, pitch);
        stepTowards(yaw, pitch, (std::max)(dt, 0.0f));
        return _fireCone <= 0.0f || errorTo(dir) <= _fireCone;
    }

    void TurretAim::relax(const float dt)
    {
        stepTowards(_freeYaw ? 0.0f : clampf(0.0f, _yawMin, _yawMax), clampf(0.0f, _pitchMin, _pitchMax), (std::max)(dt, 0.0f));
    }

    Vec3 TurretAim::barrel() const
    {
        const float c = std::cos(_pitch);
        return { std::sin(_yaw) * c, std::cos(_yaw) * c, std::sin(_pitch) };
    }

    float TurretAim::errorTo(const Vec3& dir) const
    {
        const Vec3 d = normalizeOr(dir, barrel());
        return std::acos(clampf(dot(barrel(), d), -1.0f, 1.0f));
    }

    void TurretAim::snap(const float yaw, const float pitch)
    {
        _yaw = _freeYaw ? wrapAngle(yaw) : clampf(yaw, _yawMin, _yawMax);
        _pitch = clampf(pitch, _pitchMin, _pitchMax);
    }

    FireControl::FireControl(const float roundsPerMinute) :
        _interval(60.0f / (std::max)(roundsPerMinute, 1.0f))
    {}

    int FireControl::update(const bool held, const float dt)
    {
        _cooldown -= (std::max)(dt, 0.0f);
        if (!held) {
            _cooldown = (std::max)(_cooldown, 0.0f);
            return 0;
        }
        int shots = 0;
        // A long frame fires the shots it owes, but never a burst of more than 4 (a hitch must not empty the belt).
        while (_cooldown <= 0.0f && shots < 4) {
            _cooldown += _interval;
            ++shots;
        }
        if (_cooldown < 0.0f) {
            _cooldown = 0.0f;
        }
        return shots;
    }

    Vec3 scatter(const Vec3& barrel, const float spreadRad, const float u1, const float u2)
    {
        const Vec3 b = normalizeOr(barrel, { 0.0f, 1.0f, 0.0f });
        if (spreadRad <= 0.0f) {
            return b;
        }
        // Two axes across the barrel.
        const Vec3 helper = std::fabs(b.z) < 0.9f ? Vec3{ 0.0f, 0.0f, 1.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
        const Vec3 side = normalizeOr(cross(b, helper), { 1.0f, 0.0f, 0.0f });
        const Vec3 up = cross(side, b);
        // Uniform over the disc of the cone's mouth.
        const float r = std::tan(spreadRad) * std::sqrt(clampf(u1, 0.0f, 1.0f));
        const float a = 2.0f * PI * u2;
        return normalizeOr(b + side * (r * std::cos(a)) + up * (r * std::sin(a)), b);
    }

    void launchAngles(const Vec3& dir, float& zAngle, float& xAngle)
    {
        const Vec3 d = normalizeOr(dir, { 0.0f, 1.0f, 0.0f });
        zAngle = std::atan2(d.x, d.y);
        if (zAngle < 0.0f) {
            zAngle += 2.0f * PI;
        }
        xAngle = -std::asin(clampf(d.z, -1.0f, 1.0f));
    }
}
