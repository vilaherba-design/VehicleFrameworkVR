#include "vehicle/VehicleTurret.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>

#include "Config.h"
#include "common/CommonUtils.h"
#include "f4vr/CollisionLayers.h"
#include "f4sevr/PapyrusUtils.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "health/HealthStore.h"
#include "physics/HavokBridge.h"
#include "vehicle/Trunk.h"
#include "vehicle/Vehicle.h"

namespace vfvr
{
    namespace
    {
        constexpr std::uint64_t PROJECTILE_LAUNCH_ID = 1452334; // Projectile::Launch(ProjectileLaunchData&), VR 0x104E860
        constexpr auto SCRIPT = "VehicleFrameworkVR:Turret";
        constexpr float AMMO_INTERVAL = 0.5f; // s between counts of the ammo for the HUD
        constexpr float MESH_BOUND = 120.0f; // game units: the culling sphere kept on the turret's nodes
        constexpr float SIGHT_MIN = 3.0f; // m: the sight is never drawn nearer than this

        // "Plugin.esm|0x0001F278" -> the form. [game]
        RE::TESForm* lookupForm(const std::string& spec)
        {
            const auto bar = spec.find('|');
            if (bar == std::string::npos) {
                return nullptr;
            }
            const auto plugin = common::trim(spec.substr(0, bar));
            RE::TESFormID id;
            try {
                id = static_cast<RE::TESFormID>(std::stoul(common::trim(spec.substr(bar + 1)), nullptr, 16) & 0x00FF'FFFF);
            } catch (const std::exception&) {
                return nullptr;
            }
            auto* handler = RE::TESDataHandler::GetSingleton();
            return handler ? handler->LookupForm(id, plugin) : nullptr;
        }

        template <class T>
        T* lookup(const std::string& spec)
        {
            auto* form = spec.empty() ? nullptr : lookupForm(spec);
            return form ? form->As<T>() : nullptr;
        }

        // The vehicle frame's world pose from the drawn model root: its world rotation carries the model's axes, the vehicle frame's are those
        // turned by modelRotation.
        model::Pose vehicleFrameOf(const Vehicle& vehicle, const RE::NiAVObject* root)
        {
            const auto node = physics::nodePoseMeters(root);
            return { node.pos, node.rot * vehicle.modelRotation().conjugate() };
        }

        // "Extra after rest" in the parent's frame (rotations are stored transposed: rest * extra in matrix order), as the wheels do.
        RE::NiMatrix3 compose(const RE::NiMatrix3& rest, const model::Quat& extra)
        {
            const auto x = extra.axisX();
            const auto y = extra.axisY();
            const auto z = extra.axisZ();
            RE::NiMatrix3 e = rest;
            e.entry[0][0] = x.x;
            e.entry[0][1] = x.y;
            e.entry[0][2] = x.z;
            e.entry[1][0] = y.x;
            e.entry[1][1] = y.y;
            e.entry[1][2] = y.z;
            e.entry[2][0] = z.x;
            e.entry[2][1] = z.y;
            e.entry[2][2] = z.z;
            RE::NiMatrix3 out = rest;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    out.entry[r][c] = rest.entry[r][0] * e.entry[0][c] + rest.entry[r][1] * e.entry[1][c] + rest.entry[r][2] * e.entry[2][c];
                }
            }
            return out;
        }

        void keepBounds(RE::NiAVObject* object)
        {
            if (!object) {
                return;
            }
            object->worldBound.center = object->world.translate;
            object->worldBound.fRadius = MESH_BOUND;
            if (auto* node = object->IsNode()) {
                for (const auto& child : node->children) {
                    keepBounds(child.get());
                }
            }
        }

        using Launch_t = RE::ProjectileHandle* (*)(RE::ProjectileHandle*, RE::ProjectileLaunchData&);
    }

    VehicleTurret& VehicleTurret::get()
    {
        static VehicleTurret instance;
        return instance;
    }

    void VehicleTurret::loadCatalog(const std::string& folder)
    {
        _catalogFolder = folder;
        _catalog.clear();
        std::error_code ec;
        if (!std::filesystem::is_directory(folder, ec)) {
            logger::warn("Turret: no turret folder '{}': no turrets", folder);
            return;
        }
        std::vector<std::filesystem::path> files;
        for (const auto& entry : std::filesystem::directory_iterator(folder, ec)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                files.push_back(entry.path());
            }
        }
        std::ranges::sort(files);
        for (const auto& path : files) {
            try {
                auto spec = model::loadTurretSpecFile(path.string());
                logger::info("Turret: type '{}' ({}) from {}: mesh {}, weapon {}, ammo {}, {:.0f} rpm, turns {:.0f}/{:.0f} deg/s", spec.id, spec.displayName,
                    path.filename().string(), spec.mesh, spec.weaponForm, spec.ammoForm.empty() ? "none" : spec.ammoForm, spec.roundsPerMinute, spec.yawSpeedDeg,
                    spec.pitchSpeedDeg);
                _catalog.push_back(std::move(spec));
            } catch (const std::exception& e) {
                logger::error("Turret: {} is not a valid turret: {}", path.string(), e.what());
            }
        }
    }

    const model::TurretSpec* VehicleTurret::find(const std::string& id) const
    {
        for (const auto& spec : _catalog) {
            if (spec.id == id) {
                return &spec;
            }
        }
        return nullptr;
    }

    std::string VehicleTurret::fittedId() const
    {
        return _spec ? _spec->id : std::string();
    }

    model::TurretMountSpec VehicleTurret::mountFor(const Vehicle& vehicle) const
    {
        const auto& spec = vehicle.spec();
        for (const auto& mount : spec.turretMounts) {
            if (mount.seat == "driver") {
                return mount;
            }
        }
        // No mount in the profile: on top of the model, above the seat, across the middle.
        model::TurretMountSpec mount;
        mount.id = "roof";
        mount.position = { 0.0f, spec.driverEye.y - 0.25f, vehicle.boundsMax().z };
        return mount;
    }

    void VehicleTurret::restore(Vehicle& vehicle)
    {
        detach();
        _spec.reset();
        _profile = vehicle.spec().id;
        if (!g_config.turrets || _catalog.empty()) {
            return;
        }
        std::string id;
        _free = false;
        if (const auto saved = HealthStore::get().lookupTurret(_profile)) {
            id = *saved;
            if (id.starts_with('*')) {
                _free = true;
                id.erase(0, 1);
            }
        } else {
            const auto mount = mountFor(vehicle);
            id = mount.defaultTurret;
            if (id.empty() && g_config.turretFreeKit) {
                id = _catalog.front().id;
            }
        }
        if (id.empty()) {
            return;
        }
        const auto* spec = find(id);
        if (!spec) {
            logger::warn("Turret: {} carried turret '{}', which is not in the turret folder", vehicle.spec().displayName, id);
            return;
        }
        _spec = *spec;
        _mount = mountFor(vehicle);
        _aim = model::TurretAim(*_spec);
        _fire = model::FireControl(_spec->roundsPerMinute);
        _attachFailed = false;
        logger::info("Turret: {} carries '{}' on its {} mount", vehicle.spec().displayName, _spec->displayName, _mount.id);
    }

    void VehicleTurret::release()
    {
        detach();
        _spec.reset();
        _profile.clear();
        _firing = false;
        _sightValid = false;
    }

    bool VehicleTurret::attach(Vehicle& vehicle)
    {
        auto* root = vehicle.modelRoot();
        auto* rootNode = root ? root->IsNode() : nullptr;
        if (!_spec || !rootNode) {
            return false;
        }
        RE::NiNode* mesh = nullptr;
        // Each step of hanging the turret is logged before it runs: a crash while the game reads the NIF ends the log at "loading".
        logger::info("Turret: loading '{}' for {}", _spec->mesh, vehicle.spec().displayName);
        try {
            mesh = f4vr::getClonedNiNodeForNifFileSetName(_spec->mesh, "VFVR_Turret_" + _mount.id);
        } catch (const std::exception& e) {
            logger::error("Turret: cannot load '{}': {}", _spec->mesh, e.what());
            return false;
        }
        logger::info("Turret: '{}' loaded, finding its nodes", _spec->mesh);
        _yawNode.reset(f4vr::findAVObject(mesh, _spec->yawNode));
        _pitchNode.reset(f4vr::findAVObject(mesh, _spec->pitchNode));
        _muzzleNodes.clear();
        _nextMuzzle = 0;
        for (const auto& name : _spec->muzzleNodes) {
            if (auto* node = f4vr::findAVObject(mesh, name)) {
                _muzzleNodes.emplace_back(node);
            } else {
                logger::warn("Turret: '{}' has no muzzle node '{}'", _spec->mesh, name);
            }
        }
        if (!_yawNode || !_pitchNode) {
            logger::error("Turret: '{}' has no node '{}' or '{}'", _spec->mesh, _spec->yawNode, _spec->pitchNode);
            _yawNode.reset();
            _pitchNode.reset();
            _muzzleNodes.clear();
            return false;
        }
        _yawRest = _yawNode->local.rotate;
        _pitchRest = _pitchNode->local.rotate;

        // The mesh's frame is the mount's: y forward along the mount's heading, z up, placed in the model root's own axes and units.
        const auto& modelRot = vehicle.modelRotation();
        const float rootScale = root->world.scale > 1e-4f ? root->world.scale : 1.0f;
        const model::Quat mountYaw = model::Quat::fromAxisAngle({ 0, 0, 1 }, -_mount.yawDeg * model::PI / 180.0f);
        const model::Vec3 at = modelRot.rotateInverse(_mount.position + mountYaw.rotate(_spec->meshOffset * _spec->meshScale)) * (model::HAVOK_TO_GAME / rootScale);
        const model::Quat mountRot = modelRot.conjugate() * mountYaw;
        const auto x = mountRot.axisX();
        const auto y = mountRot.axisY();
        const auto z = mountRot.axisZ();
        mesh->local.rotate.entry[0][0] = x.x;
        mesh->local.rotate.entry[0][1] = x.y;
        mesh->local.rotate.entry[0][2] = x.z;
        mesh->local.rotate.entry[1][0] = y.x;
        mesh->local.rotate.entry[1][1] = y.y;
        mesh->local.rotate.entry[1][2] = y.z;
        mesh->local.rotate.entry[2][0] = z.x;
        mesh->local.rotate.entry[2][1] = z.y;
        mesh->local.rotate.entry[2][2] = z.z;
        mesh->local.translate = { at.x, at.y, at.z };
        mesh->local.scale = _spec->meshScale / rootScale;
        logger::info("Turret: '{}' nodes yaw {} pitch {} muzzles {}/{}, attaching", _spec->mesh, _yawNode ? "ok" : "MISSING", _pitchNode ? "ok" : "MISSING",
            _muzzleNodes.size(), _spec->muzzleNodes.size());
        rootNode->AttachChild(mesh, true);
        _mesh.reset(mesh);
        _parent = root;
        f4vr::updateDown(mesh, true);
        keepBounds(mesh);
        logger::info("Turret: '{}' hung under {}'s model at ({:.0f}, {:.0f}, {:.0f}) model units", _spec->displayName, vehicle.spec().displayName, at.x, at.y, at.z);
        return true;
    }

    void VehicleTurret::detach()
    {
        stopFireSound();
        if (_mesh) {
            if (auto* parent = _mesh->parent) {
                parent->DetachChild(_mesh.get());
            }
        }
        _mesh.reset();
        _yawNode.reset();
        _pitchNode.reset();
        _muzzleNodes.clear();
        _parent = nullptr;
    }

    void VehicleTurret::writeJoints()
    {
        if (!_yawNode || !_pitchNode) {
            return;
        }
        // Yaw is clockwise from above (a heading): about -z. Pitch up: the barrel (+y) towards +z, about +x.
        _yawNode->local.rotate = compose(_yawRest, model::Quat::fromAxisAngle({ 0, 0, 1 }, -_aim.yaw()));
        _pitchNode->local.rotate = compose(_pitchRest, model::Quat::fromAxisAngle({ 1, 0, 0 }, _aim.pitch()));
        f4vr::updateDown(_mesh.get(), true);
        keepBounds(_mesh.get());
    }

    int VehicleTurret::ammoCount(Vehicle& vehicle) const
    {
        if (!_spec || _spec->ammoForm.empty()) {
            return 0;
        }
        auto* ammo = lookup<RE::TESBoundObject>(_spec->ammoForm);
        if (!ammo) {
            return 0;
        }
        int count = 0;
        RE::TESObjectREFR* sources[2] = { RE::PlayerCharacter::GetSingleton(), Trunk::get().existingContainer(vehicle.spec().id) };
        for (auto* source : sources) {
            if (!source || !source->inventoryList) {
                continue;
            }
            const RE::BSAutoReadLock lock{ source->inventoryList->rwLock };
            for (auto& item : source->inventoryList->data) {
                if (item.object == ammo) {
                    count += static_cast<int>(item.GetCount());
                }
            }
        }
        return count;
    }

    bool VehicleTurret::takeAmmo(Vehicle& vehicle, const int count)
    {
        if (!_spec || _spec->ammoForm.empty() || count <= 0) {
            return true;
        }
        auto* ammo = lookup<RE::TESBoundObject>(_spec->ammoForm);
        if (!ammo) {
            return false;
        }
        int left = count;
        RE::TESObjectREFR* sources[2] = { RE::PlayerCharacter::GetSingleton(), Trunk::get().existingContainer(vehicle.spec().id) };
        for (auto* source : sources) {
            if (!source || !source->inventoryList || left <= 0) {
                continue;
            }
            int have = 0;
            {
                const RE::BSAutoReadLock lock{ source->inventoryList->rwLock };
                for (auto& item : source->inventoryList->data) {
                    if (item.object == ammo) {
                        have += static_cast<int>(item.GetCount());
                    }
                }
            }
            const int take = (std::min)(have, left);
            if (take > 0) {
                RE::TESObjectREFR::RemoveItemData data(ammo, take);
                source->RemoveItem(data);
                left -= take;
            }
        }
        _ammo = (std::max)(_ammo - (count - left), 0);
        return left == 0;
    }

    void VehicleTurret::launch(Vehicle& /*vehicle*/, const model::Vec3& muzzle, const model::Vec3& dir)
    {
        static const bool available = physics::hasAddressId(PROJECTILE_LAUNCH_ID);
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* weapon = lookup<RE::TESObjectWEAP>(_spec->weaponForm);
        // CommonLibF4VR only declares TESAmmo: the launch data takes the form as it is.
        auto* ammo = reinterpret_cast<RE::TESAmmo*>(_spec->ammoForm.empty() ? nullptr : lookupForm(_spec->ammoForm));
        RE::BGSProjectile* projectile = lookup<RE::BGSProjectile>(_spec->projectileForm);
        if (!projectile && weapon && weapon->weaponData.rangedData) {
            projectile = weapon->weaponData.rangedData->overrideProjectile;
        }
        if (!available || !player || !projectile) {
            if (!_launchWarned) {
                _launchWarned = true;
                logger::error("Turret: cannot fire '{}': {}", _spec->displayName,
                    !available ? "Projectile::Launch (id 1452334) is not in the VR address library" : !projectile ? "no projectile (the weapon's override or 'weapon.projectile')" : "no player");
            }
            return;
        }
        // The launch data has no default constructor (its weapon instance has none): zeroed storage, filled field by field. The weapon goes in
        // without instance data (null), as the engine passes a base form; nothing here owns a reference, so nothing is destroyed afterwards.
        alignas(RE::ProjectileLaunchData) std::byte storage[sizeof(RE::ProjectileLaunchData)]{};
        auto& data = *reinterpret_cast<RE::ProjectileLaunchData*>(storage);
        data.origin = physics::toGame(muzzle);
        data.contactNormal = { 0.0f, 0.0f, 0.0f };
        data.projectileBase = projectile;
        data.shooter = player;
        data.shooterCombatController = nullptr;
        data.fromWeapon.object = weapon;
        data.fromAmmo = ammo;
        model::launchAngles(dir, data.zAngle, data.xAngle);
        data.yAngle = 0.0f;
        data.homingTarget = nullptr;
        data.parentCell = player->GetParentCell();
        data.spell = nullptr;
        data.poison = nullptr;
        data.area = 0;
        data.power = 1.0f;
        data.scale = 1.0f;
        data.coneOfFireRadiusMult = 1.0f;
        data.alwaysHit = false;
        data.noDamageOutsideCombat = false;
        data.autoAim = false;
        data.useOrigin = true;
        data.deferInitialization = false;
        data.tracer = true;
        data.forceConeOfFire = false;
        data.intentionalMiss = false;
        data.allow3D = true;
        data.penetrates = false;
        data.ignoreNearCollisions = true;
        static REL::Relocation<Launch_t> projectileLaunch{ REL::ID(PROJECTILE_LAUNCH_ID) };
        RE::ProjectileHandle handle;
        projectileLaunch(&handle, data);
        ++_burstShots;
    }

    void VehicleTurret::onFireSoundStarted(const std::int32_t instance)
    {
        // Let go before the loop started: stop it at once.
        if (!_soundWanted.load()) {
            F4SEVR::execPapyrusGlobalFunction(SCRIPT, "StopFire", static_cast<F4SEVR::SInt32>(instance));
            return;
        }
        _soundInstance = instance;
    }

    void VehicleTurret::playShotSound()
    {
        if (!_spec || !g_config.turretFireSound || _spec->fireSound.empty()) {
            return;
        }
        if (auto* sound = lookupForm(_spec->fireSound)) {
            F4SEVR::execPapyrusGlobalFunction(SCRIPT, "PlayShot", static_cast<F4SEVR::SInt32>(sound->GetFormID()));
        }
    }

    void VehicleTurret::startFireSound()
    {
        if (_soundOn || !_spec || !g_config.turretFireSound || _spec->fireSound.empty() || !_spec->fireSoundLoop) {
            return;
        }
        if (auto* sound = lookupForm(_spec->fireSound)) {
            _soundOn = true;
            _soundWanted = true;
            _soundInstance = 0;
            F4SEVR::execPapyrusGlobalFunction(SCRIPT, "StartFire", static_cast<F4SEVR::SInt32>(sound->GetFormID()));
        }
    }

    void VehicleTurret::stopFireSound()
    {
        if (!_soundOn) {
            return;
        }
        _soundOn = false;
        _soundWanted = false;
        if (const auto instance = _soundInstance.exchange(0); instance != 0) {
            F4SEVR::execPapyrusGlobalFunction(SCRIPT, "StopFire", static_cast<F4SEVR::SInt32>(instance));
        }
    }

    void VehicleTurret::aimAndFire(Vehicle& vehicle, const bool driving, const float dt)
    {
        _sightValid = false;
        if (!_mesh) {
            return;
        }
        const model::Pose frame = vehicleFrameOf(vehicle, _parent);
        const model::Pose mount = frame * model::Pose{ _mount.position, model::Quat::fromAxisAngle({ 0, 0, 1 }, -_mount.yawDeg * model::PI / 180.0f) };
        const auto* nodes = f4vr::getVRPlayerNodes();
        if (!driving || !nodes || !nodes->hmdNode) {
            _aim.relax(dt);
            writeJoints();
            _firing = false;
            stopFireSound();
            return;
        }

        // Where the line of sight meets the world (actors included), or fTurretAimDistance along it.
        const auto& hmd = nodes->hmdNode->world;
        const RE::NiPoint3 eye = hmd.translate;
        const RE::NiPoint3 look = hmd.rotate.Transpose() * RE::NiPoint3(0, 1, 0);
        const RE::NiPoint3 farPoint = eye + look * g_config.turretAimDistance;
        const std::uint64_t ignore = f4vr::collisionLayerBit(RE::COL_LAYER::kCharController) | f4vr::collisionLayerBit(RE::COL_LAYER::kTrigger) |
            f4vr::collisionLayerBit(RE::COL_LAYER::kNonCollidable) | f4vr::collisionLayerBit(RE::COL_LAYER::kCameraSphere);
        const auto& body = vehicle.bodyHandle();
        const auto hit = physics::castRay(body.bhkWorld, eye + look * 150.0f, farPoint, static_cast<std::uint32_t>(RE::COL_LAYER::kLineOfSight), vehicle.bodyId(), ignore,
            g_config.releasePickWorldReference);
        const model::Vec3 target = physics::toMeters(hit.hit ? hit.point : farPoint);

        const model::Vec3 pivot = physics::toMeters(_yawNode->world.translate);
        const bool ready = _aim.update(mount.rot.rotateInverse(target - pivot), dt);
        writeJoints();

        // The barrel's line, for the sight and the shots: from the muzzle nodes (fresh: the joints were just updated under this frame's model
        // root). The sight leaves from their middle; each shot from the next muzzle in turn.
        const model::Vec3 barrel = mount.rot.rotate(_aim.barrel());
        const auto muzzleAt = [&](const std::size_t i) {
            return _muzzleNodes.empty() ? physics::nodePoseMeters(_pitchNode.get()).toWorld(_spec->muzzleOffset)
                                        : physics::toMeters(_muzzleNodes[i % _muzzleNodes.size()]->world.translate);
        };
        model::Vec3 muzzle = muzzleAt(0);
        for (std::size_t i = 1; i < _muzzleNodes.size(); ++i) {
            muzzle = muzzle + muzzleAt(i);
        }
        if (_muzzleNodes.size() > 1) {
            muzzle = muzzle * (1.0f / static_cast<float>(_muzzleNodes.size()));
        }
        _sight = muzzle + barrel * (std::max)(model::length(target - muzzle), SIGHT_MIN);
        _sightValid = true;

        const bool held = g_config.turretFireBinding.isEnabled() &&
            vrcf::VRControllers.isPressHeldDown(g_config.turretFireBinding.hand, g_config.turretFireBinding.button);
        if (held && !_firing) {
            _burstShots = 0;
        }
        const int shots = _fire.update(held && ready, dt);
        bool shot = false;
        for (int i = 0; i < shots; ++i) {
            const int need = _spec->ammoForm.empty() ? 0 : _spec->ammoPerShot;
            if (need > 0 && !takeAmmo(vehicle, need)) {
                if (!_noAmmoNotified) {
                    _noAmmoNotified = true;
                    f4vr::showNotification(std::format("{}: out of ammo", _spec->displayName));
                    logger::info("Turret: '{}' is out of ammo", _spec->displayName);
                }
                break;
            }
            _noAmmoNotified = false;
            std::uniform_real_distribution<float> uniform(0.0f, 1.0f);
            const float spread = _spec->spreadDeg * model::PI / 180.0f;
            launch(vehicle, muzzleAt(_nextMuzzle++), model::scatter(barrel, spread, uniform(_rng), uniform(_rng)));
            if (!_spec->fireSoundLoop) {
                playShotSound();
            }
            shot = true;
        }
        // The loop sounds while the trigger is held and shots leave; out of ammo, off target or let go, it stops.
        if (shot) {
            startFireSound();
        } else if (!held || _noAmmoNotified) {
            stopFireSound();
        }
        if (_firing && !held && g_config.logTurret) {
            logger::info("Turret: burst of {} shots from '{}', {} ammo left, muzzle ({:.0f}, {:.0f}, {:.0f}), yaw {:.0f} deg, pitch {:.0f} deg", _burstShots,
                _spec->displayName, _ammo, muzzle.x * model::HAVOK_TO_GAME, muzzle.y * model::HAVOK_TO_GAME, muzzle.z * model::HAVOK_TO_GAME,
                _aim.yaw() * 180.0f / model::PI, _aim.pitch() * 180.0f / model::PI);
        }
        _firing = held;
    }

    void VehicleTurret::onFrameUpdate(Vehicle* vehicle, const bool driving)
    {
        const auto now = std::chrono::steady_clock::now();
        const float dt = _clockValid ? std::clamp(std::chrono::duration<float>(now - _last).count(), 0.0f, 0.1f) : 0.0f;
        _last = now;
        _clockValid = true;
        if (!g_config.turrets || !vehicle || !vehicle->isValid() || !_spec) {
            if (_mesh && (!vehicle || !vehicle->isValid() || !g_config.turrets)) {
                detach();
            }
            _sightValid = false;
            return;
        }
        if (vehicle->spec().id != _profile) {
            restore(*vehicle);
            if (!_spec) {
                return;
            }
        }
        // The model root can be replaced (a new model reference in another cell): hang the mesh again.
        if (_mesh && _parent != vehicle->modelRoot()) {
            detach();
        }
        if (!_mesh && !_attachFailed) {
            _attachFailed = !attach(*vehicle);
        }
        _ammoTimer -= dt;
        if (_ammoTimer <= 0.0f) {
            _ammoTimer = AMMO_INTERVAL;
            _ammo = ammoCount(*vehicle);
        }
        aimAndFire(*vehicle, driving, dt);
    }

    std::vector<const model::TurretSpec*> VehicleTurret::availableKits(Vehicle& vehicle) const
    {
        std::vector<const model::TurretSpec*> out;
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* trunk = Trunk::get().existingContainer(vehicle.spec().id);
        for (const auto& spec : _catalog) {
            if (g_config.turretFreeKit) {
                out.push_back(&spec);
                continue;
            }
            auto* kit = lookup<RE::TESBoundObject>(spec.kitForm);
            if (!kit) {
                continue;
            }
            bool has = false;
            for (auto* source : { static_cast<RE::TESObjectREFR*>(player), trunk }) {
                if (!source || !source->inventoryList) {
                    continue;
                }
                const RE::BSAutoReadLock lock{ source->inventoryList->rwLock };
                for (auto& item : source->inventoryList->data) {
                    if (item.object == kit && item.GetCount() > 0) {
                        has = true;
                    }
                }
            }
            if (has) {
                out.push_back(&spec);
            }
        }
        return out;
    }

    bool VehicleTurret::fit(Vehicle& vehicle, const std::string& turretId, const bool useKit)
    {
        const auto* spec = find(turretId);
        logger::info("Turret: fitting '{}' to {} ({})", turretId, vehicle.spec().displayName, useKit ? "with its kit" : "free");
        if (!spec) {
            logger::warn("Turret: '{}' is not a turret type", turretId);
            return false;
        }
        if (_spec && _spec->id == turretId) {
            return true;
        }
        if (_spec) {
            remove(vehicle);
        }
        if (useKit && !g_config.turretFreeKit) {
            auto* kit = lookup<RE::TESBoundObject>(spec->kitForm);
            auto* player = RE::PlayerCharacter::GetSingleton();
            auto* trunk = Trunk::get().existingContainer(vehicle.spec().id);
            bool taken = false;
            for (auto* source : { static_cast<RE::TESObjectREFR*>(player), trunk }) {
                if (taken || !source || !source->inventoryList || !kit) {
                    continue;
                }
                bool has = false;
                {
                    const RE::BSAutoReadLock lock{ source->inventoryList->rwLock };
                    for (auto& item : source->inventoryList->data) {
                        has = has || (item.object == kit && item.GetCount() > 0);
                    }
                }
                if (has) {
                    RE::TESObjectREFR::RemoveItemData data(kit, 1);
                    source->RemoveItem(data);
                    taken = true;
                }
            }
            if (!taken) {
                f4vr::showNotification(std::format("You need a {} kit (made at a settlement's chemistry station)", spec->displayName));
                return false;
            }
        }
        HealthStore::get().rememberTurret(vehicle.spec().id, useKit ? spec->id : "*" + spec->id);
        _profile.clear(); // restore() on the next frame picks it up from the store
        restore(vehicle);
        f4vr::showNotification(std::format("{} fitted to {}", spec->displayName, vehicle.spec().displayName));
        return true;
    }

    void VehicleTurret::remove(Vehicle& vehicle)
    {
        if (!_spec) {
            return;
        }
        if (!g_config.turretFreeKit && !_free) {
            if (auto* kit = lookup<RE::TESBoundObject>(_spec->kitForm)) {
                if (auto* player = RE::PlayerCharacter::GetSingleton()) {
                    player->AddObjectToContainer(kit, nullptr, 1, nullptr, RE::ITEM_REMOVE_REASON::kNone);
                }
            }
        }
        f4vr::showNotification(std::format("{} taken off {}", _spec->displayName, vehicle.spec().displayName));
        HealthStore::get().rememberTurret(vehicle.spec().id, "");
        detach();
        _spec.reset();
    }

    void VehicleTurret::fillHud(HudState& state) const
    {
        if (!g_config.turrets || !_spec) {
            return;
        }
        state.hasWeapon = true;
        state.weaponName = _spec->displayName;
        state.ammo = _ammo;
        state.aimMode = HudState::AimMode::Manual;
        state.sightShown = g_config.turretSight && _sightValid;
        state.sight = _sight;
    }
}
