#include "vehicle/DamageSmoke.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "Config.h"
#include "f4vr/F4VRUtils.h"
#include "physics/HavokBridge.h"
#include "vehicle/SpriteUtils.h"
#include "vehicle/Vehicle.h"

namespace vfvr
{
    namespace
    {
        constexpr std::size_t POOL = 16;

        std::uint64_t nowNanos()
        {
            return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
        }
    }

    float DamageSmoke::random01()
    {
        _rng = _rng * 1664525u + 1013904223u;
        return static_cast<float>(_rng >> 8) / static_cast<float>(1u << 24);
    }

    void DamageSmoke::clear()
    {
        for (auto& puff : _puffs) {
            if (auto* ref = puff.ref.get().get()) {
                ref->Disable();
                ref->SetDelete(true);
            }
        }
        _puffs.clear();
        _accumulator = 0.0f;
    }

    void DamageSmoke::update(Vehicle* vehicle)
    {
        const auto now = nowNanos();
        const float dt = _clock ? (std::min)(static_cast<float>(now - _clock) * 1e-9f, 0.1f) : 0.0f;
        _clock = now;
        if (!vehicle || !vehicle->isValid() || !g_config.vehicleHealth || !g_config.damageSmoke) {
            if (!_puffs.empty()) {
                clear();
            }
            return;
        }
        const auto condition = vehicle->health().condition();
        const bool emitting = condition != model::VehicleCondition::Operational;
        if (!emitting && _puffs.empty()) {
            return;
        }
        const auto& spec = vehicle->spec();
        auto* carrier = vehicle->ref();
        auto* cell = carrier ? carrier->GetParentCell() : nullptr;
        auto* handler = RE::TESDataHandler::GetSingleton();
        if (!cell || !handler) {
            return;
        }
        if (_cooldown > 0) {
            --_cooldown;
        }
        // Only another worldspace, an interior, or a puff that is gone needs a new pool (see the exhaust: cell borders outdoors do not).
        const auto sameSpace = [cell](const RE::TESObjectCELL* other) {
            if (!other) {
                return false;
            }
            if (cell->IsInterior() || other->IsInterior()) {
                return other == cell;
            }
            return other->worldSpace == cell->worldSpace;
        };
        bool recreate = _puffs.size() != POOL;
        bool any3D = false;
        for (auto& puff : _puffs) {
            auto* ref = puff.ref.get().get();
            if (!ref || ref->IsDeleted() || !sameSpace(ref->GetParentCell())) {
                recreate = true;
            } else if (ref->Get3D()) {
                any3D = true;
            }
        }
        _no3DFrames = any3D || _puffs.empty() ? 0 : _no3DFrames + 1;
        if (_no3DFrames > 60) {
            recreate = true;
        }
        const auto points = vehicle->smokePoints();
        const auto pose = vehicle->poseFromBody();
        if (recreate) {
            if (!emitting || _cooldown > 0 || points.empty()) {
                return;
            }
            clear();
            auto* base = lookupBaseObject(spec.exhaustForm);
            if (!base) {
                logger::error("Damage smoke puff object '{}' not found (is VehicleFrameworkVR_Vehicles.esp enabled?)", spec.exhaustForm);
                _cooldown = 600;
                return;
            }
            for (std::size_t i = 0; i < POOL; ++i) {
                RE::NEW_REFR_DATA data;
                data.location = physics::toGame(pose.toWorld(points.front()));
                data.direction = RE::NiPoint3(0, 0, 0);
                data.object = base;
                data.interior = cell->IsInterior() ? cell : nullptr;
                data.world = cell->IsExterior() ? cell->worldSpace : nullptr;
                data.clearStillLoadingFlag = true;
                data.initializeScripts = false;
                Puff puff;
                puff.ref = handler->CreateReferenceAtLocation(data);
                puff.pos = data.location;
                _puffs.push_back(puff);
            }
            _cooldown = 20;
            _no3DFrames = 0;
            logger::info("Damage smoke created ({} puffs) for {}", _puffs.size(), spec.displayName);
        }

        // Emit: a thin wisp while damaged, a thick column while disabled.
        const bool disabled = condition == model::VehicleCondition::Disabled;
        if (emitting && !points.empty()) {
            _accumulator = (std::min)(_accumulator + dt * (disabled ? 6.5f : 1.8f) * static_cast<float>(points.size()), 3.0f);
        }
        while (_accumulator >= 1.0f && !points.empty()) {
            _accumulator -= 1.0f;
            auto& puff = _puffs[_next];
            _next = (_next + 1) % _puffs.size();
            const auto& point = points[_pointIndex++ % points.size()];
            puff.pos = physics::toGame(pose.toWorld(point));
            // Straight up with the heat, with a little wander; the column of a dead engine rises faster and lasts longer.
            const float rise = disabled ? 14.0f + 8.0f * random01() : 9.0f + 4.0f * random01();
            puff.vel = RE::NiPoint3((random01() - 0.5f) * 8.0f, (random01() - 0.5f) * 8.0f, rise);
            puff.age = 0.0f;
            puff.life = disabled ? 2.6f + 0.8f * random01() : 1.8f + 0.5f * random01();
            puff.roll = random01() * 6.2831853f;
            puff.spin = (random01() - 0.5f) * 0.8f;
            puff.size = disabled ? 1.6f + 0.5f * random01() : 0.9f + 0.3f * random01();
        }

        const auto head = hmdPosition();
        for (auto& puff : _puffs) {
            auto* ref = puff.ref.get().get();
            if (!ref || ref->IsDeleted()) {
                continue;
            }
            float scale = 0.001f;
            if (puff.age < puff.life) {
                puff.age += dt;
                puff.roll += puff.spin * dt;
                puff.vel.z += 3.0f * dt;
                const float drag = std::pow(0.6f, dt);
                puff.vel.x *= drag;
                puff.vel.y *= drag;
                puff.pos.x += puff.vel.x * dt;
                puff.pos.y += puff.vel.y * dt;
                puff.pos.z += puff.vel.z * dt;
                const float t = (std::min)(puff.age / puff.life, 1.0f);
                // The sprite's half size is 3 game units: a puff swells to about half a metre across (a metre for the dead engine's column) and
                // then dissolves (it can only be scaled, not faded, so the last stretch shrinks it).
                const float grow = 1.0f + (disabled ? 16.0f : 11.0f) * std::sqrt(t);
                const float fade = t > 0.6f ? std::pow((std::max)(1.0f - (t - 0.6f) / 0.4f, 0.0f), 1.3f) : 1.0f;
                scale = (std::max)(grow * fade * puff.size, 0.001f);
            }
            ref->data.location.x = puff.pos.x;
            ref->data.location.y = puff.pos.y;
            ref->data.location.z = puff.pos.z;
            ref->data.angle.x = 0.0f;
            ref->data.angle.y = 0.0f;
            ref->data.angle.z = 0.0f;
            ref->Update3DPosition(true);
            if (auto* root = ref->Get3D()) {
                root->local.scale = scale;
                faceViewer(root->local.rotate, puff.pos, head, puff.roll);
                f4vr::updateDown(root, true);
            }
        }
    }
}
