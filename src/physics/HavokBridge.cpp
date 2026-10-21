#include "physics/HavokBridge.h"

#include <array>
#include <cstring>
#include <intrin.h>

#include <fstream>
#include <unordered_set>

#include "f4vr/CollisionLayers.h"

namespace vfvr::physics
{
    namespace
    {
        constexpr std::uint32_t INVALID_BODY = 0x7FFF'FFFF;
        constexpr std::uintptr_t BHK_WORLD_HKNP_WORLD_OFFSET = 0x60;
        constexpr int MAX_RAY_RECASTS = 4;

        // RVAs other FO4VR 1.2.72 mods use for the same functions; logged next to ours as a cross-check.
        constexpr std::uintptr_t ROCK_RVA_ADD_STEP_LISTENER = 0x1DFA7B0;
        constexpr std::uintptr_t ROCK_RVA_PICK_OBJECT = 0x1DF8D60;

        /*
         * NiCollisionObject -> bhkNPCollisionObject by NiRTTI name. RE::fallout_cast goes through the MSVC
         * RTDynamicCast (address-library ID 84112), which does not exist in the VR address library.
         */
        RE::bhkNPCollisionObject* asNPCollisionObject(RE::NiCollisionObject* object)
        {
            if (!object) {
                return nullptr;
            }
            for (const auto* rtti = object->GetRTTI(); rtti; rtti = rtti->GetBaseRTTI()) {
                if (rtti->GetName() && std::string_view(rtti->GetName()) == "bhkNPCollisionObject") {
                    return static_cast<RE::bhkNPCollisionObject*>(object);
                }
            }
            return nullptr;
        }

        RE::hknpBodyId makeBodyId(const std::uint32_t value)
        {
            RE::hknpBodyId id{};
            id.value = value;
            return id;
        }

        model::Vec3 fromHk(const float* v)
        {
            return { v[0], v[1], v[2] };
        }

        model::Vec3 fromHk(const RE::hkVector4f& v)
        {
            return { v.x, v.y, v.z };
        }

        /*
         * Havok reference release (FO4VR hkReferencedObject: 16-bit count in the low word at +0x08,
         * destroy through vtable slot 3 at zero). Adapted from ROCK's HavokRefCount.h
         * (https://github.com/brunocatani/ROCK, GPL-3.0, Bruno Catani).
         */
        void havokRelease(void* object)
        {
            if (!object) {
                return;
            }
            auto* word = reinterpret_cast<volatile long*>(static_cast<char*>(object) + 0x08);
            for (;;) {
                const long oldValue = *word;
                const auto flags = static_cast<std::uint16_t>((static_cast<std::uint32_t>(oldValue) >> 16) & 0xFFFFu);
                const auto count = static_cast<std::uint16_t>(oldValue & 0xFFFF);
                if (flags == 0 || count == 0 || count == 0xFFFF) {
                    return;
                }
                const long newValue = (oldValue & static_cast<long>(0xFFFF0000u)) | static_cast<long>(static_cast<std::uint16_t>(count - 1));
                if (_InterlockedCompareExchange(word, newValue, oldValue) != oldValue) {
                    continue;
                }
                if (count - 1 == 0) {
                    auto** vtable = *reinterpret_cast<void***>(object);
                    reinterpret_cast<void (*)(void*)>(vtable[3])(object);
                }
                return;
            }
        }

        /*
         * bhkWorld::PickObject keeps a reference to the world in the pick data (+0xC0, the member CommonLib
         * calls allHitsResult) that the engine's own callers release when the pick data goes out of scope.
         * CommonLib's bhkPickData has no destructor, so every ray would leak one reference. Only release it
         * when the instruction that stores it is the one we expect (as ROCK does).
         */
        bool pickWorldReleaseVerified()
        {
            static const bool verified = [] {
                static_assert(offsetof(RE::bhkPickData, allHitsResult) == 0xC0);
                if (!REL::Module::IsVR()) {
                    return false;
                }
                constexpr std::array<std::uint8_t, 7> expected{ 0x49, 0x89, 0xB6, 0xC0, 0x00, 0x00, 0x00 }; // mov [r14+0xC0], rsi
                std::array<std::uint8_t, 7> actual{};
                std::memcpy(actual.data(), reinterpret_cast<const void*>(REL::Offset(0x1DF8ED6).address()), actual.size());
                const bool ok = actual == expected;
                logger::info("Raycast world-reference release {}", ok ? "verified" : "NOT verified (disabled; rays may leak a world reference)");
                return ok;
            }();
            return verified;
        }

        // Fallout 4 VR's Havok step listener: an object whose first member is a vtable of 11 slots.
        // Slot layout (same as ROCK's PhysicsStepDriveCoordinator): 1 before whole update, 3 before each
        // (sub)step, 5 between collide and solve, 7 after each step, 9 after whole update; the rest unused.
        using StepFn = void (*)(void* listener, std::uint32_t, void*, float progress, float deltaSeconds);
        using WholeFn = void (*)(void* listener, std::uint32_t, void*);
        using CollideSolveFn = void (*)(void* listener, std::uint32_t, void*, std::uint32_t, float, float);

        struct NativeVTable
        {
            void (*unused0)();
            WholeFn beforeWhole;
            void (*unused2)();
            StepFn beforeAny;
            void (*unused4)();
            CollideSolveFn betweenCollideAndSolve;
            void (*unused6)();
            StepFn afterAny;
            void (*unused8)();
            WholeFn afterWhole;
            void (*unused10)();
        };

        static_assert(offsetof(NativeVTable, beforeAny) == 0x18);
        static_assert(offsetof(NativeVTable, afterWhole) == 0x48);

        void noop()
        {}

        void noopWhole(void*, std::uint32_t, void*)
        {}

        void noopStep(void*, std::uint32_t, void*, float, float)
        {}

        void noopCollideSolve(void*, std::uint32_t, void*, std::uint32_t, float, float)
        {}

        void beforeAnyThunk(void* listener, std::uint32_t, void*, float, float deltaSeconds);

        const NativeVTable NATIVE_VTABLE{ &noop, &noopWhole, &noop, &beforeAnyThunk, &noop, &noopCollideSolve, &noop, &noopStep, &noop, &noopWhole, &noop };
    }

    struct StepListener::Native
    {
        const NativeVTable* vtable = &NATIVE_VTABLE;
        void* engineScratch = nullptr; // the engine writes here
        StepListener* owner = nullptr;
        void* pad = nullptr;
    };

    static_assert(sizeof(StepListener::Native) == 32);

    namespace
    {
        void beforeAnyThunk(void* listener, std::uint32_t, void*, const float, const float deltaSeconds)
        {
            const auto* native = static_cast<StepListener::Native*>(listener);
            if (native && native->owner) {
                native->owner->onBeforeStep(deltaSeconds);
            }
        }
    }

    bool hasAddressId(const std::uint64_t id)
    {
        // CommonLib keeps its table protected, so read the same VR address library CSV ("id,offset" rows).
        static const std::unordered_set<std::uint64_t> ids = [] {
            std::unordered_set<std::uint64_t> set;
            std::ifstream file("Data/F4SE/Plugins/version-1-2-72-0.csv");
            std::string line;
            std::getline(file, line); // header: count, version
            while (std::getline(file, line)) {
                const auto comma = line.find(',');
                if (comma != std::string::npos) {
                    try {
                        set.insert(std::stoull(line.substr(0, comma)));
                    } catch (const std::exception&) {
                    }
                }
            }
            logger::info("Address library CSV: {} IDs read", set.size());
            return set;
        }();
        return ids.contains(id);
    }

    model::Pose nodePoseMeters(const RE::NiAVObject* node)
    {
        // The engine stores rotations transposed relative to local->world (f4cf::common::MatrixUtils), so the
        // world images of the local axes are the rotate matrix's transpose applied to the unit axes.
        const auto rt = node->world.rotate.Transpose();
        const auto x = rt * RE::NiPoint3(1, 0, 0);
        const auto y = rt * RE::NiPoint3(0, 1, 0);
        const auto z = rt * RE::NiPoint3(0, 0, 1);
        return { toMeters(node->world.translate), model::Quat::fromBasis(toVec(x), toVec(y), toVec(z)) };
    }

    RE::hknpWorld* getHknpWorld(RE::bhkWorld* bhkWorld)
    {
        return bhkWorld ? *reinterpret_cast<RE::hknpWorld**>(reinterpret_cast<std::uintptr_t>(bhkWorld) + BHK_WORLD_HKNP_WORLD_OFFSET) : nullptr;
    }

    RE::bhkWorld* getBhkWorld(const RE::TESObjectREFR* ref)
    {
        const auto* cell = ref ? ref->GetParentCell() : nullptr;
        return cell ? cell->GetbhkWorld() : nullptr;
    }

    bool resolveBody(BodyHandle& handle)
    {
        handle.bodyId = makeBodyId(INVALID_BODY);
        handle.world = nullptr;
        auto* col = handle.collisionObject;
        if (!col) {
            return false;
        }
        auto* system = col->spSystem.get();
        auto* instance = system ? system->instance : nullptr;
        if (!instance || !instance->world) {
            return false;
        }
        const auto index = static_cast<std::int32_t>(col->systemBodyIdx);
        if (index < 0 || index >= instance->bodyCount) {
            return false;
        }
        RE::hknpBodyId id = makeBodyId(INVALID_BODY);
        system->GetBodyId(&id, index);
        if (id.value == INVALID_BODY || id.value == 0) {
            return false;
        }
        handle.world = instance->world;
        handle.bodyId = id;
        return true;
    }

    std::optional<BodyHandle> findBody(RE::NiAVObject* root, std::string& diagnostics)
    {
        // Depth-first, a few levels deep: vanilla statics carry their collision on the root or a direct child.
        std::function<RE::NiAVObject*(RE::NiAVObject*, int)> search = [&](RE::NiAVObject* node, const int depth) -> RE::NiAVObject* {
            if (!node || depth > 4) {
                return nullptr;
            }
            if (node->collisionObject && asNPCollisionObject(node->collisionObject.get())) {
                return node;
            }
            if (auto* asNode = node->IsNode()) {
                for (const auto& child : asNode->children) {
                    if (auto* found = search(child.get(), depth + 1)) {
                        return found;
                    }
                }
            }
            return nullptr;
        };

        auto* owner = search(root, 0);
        if (!owner) {
            diagnostics = "no Havok collision object found under the model root";
            return std::nullopt;
        }
        BodyHandle handle;
        handle.node = owner;
        handle.collisionObject = asNPCollisionObject(owner->collisionObject.get());
        if (!resolveBody(handle)) {
            diagnostics = std::format("collision object on '{}' has no body in the world yet", owner->name.c_str());
            return std::nullopt;
        }
        diagnostics = std::format("body {} on node '{}'", handle.bodyId.value, owner->name.c_str());
        return handle;
    }

    BodyStateMeters readBodyState(const BodyHandle& handle, const bool angularFromMotion)
    {
        BodyStateMeters s;
        if (!handle.isValid()) {
            return s;
        }
        const auto& body = handle.world->GetBody(handle.bodyId);
        const auto* motion = handle.world->GetBodyMotion(handle.bodyId);
        if (!motion || body.motionIndex == 0) {
            return s;
        }
        s.frame.pos = fromHk(body.translation);
        s.frame.rot = model::Quat::fromBasis(fromHk(body.transformCol0), fromHk(body.transformCol1), fromHk(body.transformCol2));
        s.com = fromHk(motion->position);
        s.linVel = fromHk(motion->linearVelocity);
        if (angularFromMotion) {
            const model::Quat motionRot{ motion->orientation.x, motion->orientation.y, motion->orientation.z, motion->orientation.w };
            s.angVel = motionRot.normalized().rotate(fromHk(motion->angularVelocity));
        } else {
            RE::hkVector4f w;
            handle.world->GetBodyAngularVelocity(handle.bodyId, w);
            s.angVel = fromHk(w);
        }
        s.valid = model::isFinite(s.frame.pos) && model::isFinite(s.com) && model::isFinite(s.linVel) && model::isFinite(s.angVel);
        return s;
    }

    int bodyMotionPreset(const BodyHandle& handle)
    {
        if (!handle.isValid()) {
            return -1;
        }
        const auto& body = handle.world->GetBody(handle.bodyId);
        return body.motionIndex == 0 ? 0 : body.motionPropertiesId & 0xFF;
    }

    bool bodyIsDynamic(const BodyHandle& handle)
    {
        const int preset = bodyMotionPreset(handle);
        return preset > 0 && preset != 2;
    }

    int bodyShapeType(const BodyHandle& handle)
    {
        if (!handle.isValid()) {
            return -1;
        }
        const auto* shape = handle.world->GetBody(handle.bodyId).shape;
        return shape ? static_cast<int>(shape->GetType()) : -1;
    }

    namespace
    {
        /*
         * Convex hull from points through the game's own builder (FO4VR 1.2.72: config init RVA 0x16D4AB0,
         * build from a strided hkVector4 array RVA 0x16D4B30). Addresses and call shape from ROCK's
         * HavokConvexShapeBuilder.cpp (https://github.com/brunocatani/ROCK, GPL-3.0, Bruno Catani).
         */
        struct alignas(16) HkPoint
        {
            float x, y, z, w;
        };

        struct HkStridedPointArray
        {
            const void* data;
            std::int32_t count;
            std::int32_t stride;
        };

        RE::hknpShape* buildBoxShape(const model::Vec3& lo, const model::Vec3& hi, const HullShape& hull)
        {
            using InitConfig_t = void* (*)(void*);
            using BuildShapeFromPoints_t = RE::hknpShape* (*)(const HkStridedPointArray*, float, void*);
            static REL::Relocation<InitConfig_t> initConfig{ REL::Offset(0x16D4AB0) };
            static REL::Relocation<BuildShapeFromPoints_t> buildShape{ REL::Offset(0x16D4B30) };

            // A box with its lower edges cut away: a flat box edge catches on kerbs and debris and levers the car up, a ramp slides over
            // them. L is the car's length axis, W the width axis; the ramp is cut on L (front and rear), the inset on W.
            const bool lengthX = hull.lengthAxis == 0;
            const float loL = lengthX ? lo.x : lo.y;
            const float hiL = lengthX ? hi.x : hi.y;
            const float loW = lengthX ? lo.y : lo.x;
            const float hiW = lengthX ? hi.y : hi.x;
            const float height = hi.z - lo.z;
            const float rampH = (std::min)(hull.rampHeight, 0.6f * height);
            const float rampL = (std::min)(hull.rampLength, 0.3f * (hiL - loL));
            const float side = (std::min)(hull.sideInset, 0.1f * (hiW - loW));
            const auto point = [&](const float l, const float w, const float z) { return lengthX ? HkPoint{ l, w, z, 0.0f } : HkPoint{ w, l, z, 0.0f }; };
            std::array<HkPoint, 12> points{};
            int n = 0;
            for (int i = 0; i < 4; ++i) {
                points[n++] = point((i & 1) ? hiL - rampL : loL + rampL, (i & 2) ? hiW - side : loW + side, lo.z);
            }
            for (int i = 0; i < 4; ++i) {
                points[n++] = point((i & 1) ? hiL : loL, (i & 2) ? hiW : loW, lo.z + rampH);
            }
            for (int i = 0; i < 4; ++i) {
                points[n++] = point((i & 1) ? hiL : loL, (i & 2) ? hiW : loW, hi.z);
            }
            alignas(16) std::array<std::uint8_t, 0x80> config{};
            initConfig(config.data());
            const HkStridedPointArray array{ points.data(), n, static_cast<std::int32_t>(sizeof(HkPoint)) };
            return buildShape(&array, 0.02f, config.data());
        }

        // FO4VR 1.2.72 RVAs from ROCK's HavokOffsets.h (GPL-3.0): Bethesda's "make body keyframed" (gives a
        // static body its own motion) and hknpWorld motion mass-property rebuild.
        constexpr std::uintptr_t RVA_SET_BODY_KEYFRAMED = 0x1DF5CB0;
        constexpr std::uintptr_t RVA_REBUILD_MOTION_MASS = 0x1546570;

        // Body-space AABB of a body's current shape (meters).
        bool shapeLocalAabb(const BodyHandle& handle, model::Vec3& lo, model::Vec3& hi)
        {
            const auto* shape = handle.world->GetBody(handle.bodyId).shape;
            if (!shape) {
                return false;
            }
            RE::hkTransformf identity;
            identity.SetIdentity();

            struct alignas(16) Aabb
            {
                float min[4];
                float max[4];
            } aabb{};

            shape->CalcAabb(identity, *reinterpret_cast<RE::hkAabb*>(&aabb));
            lo = { aabb.min[0], aabb.min[1], aabb.min[2] };
            hi = { aabb.max[0], aabb.max[1], aabb.max[2] };
            return model::isFinite(lo) && model::isFinite(hi) && hi.x > lo.x && hi.y > lo.y && hi.z > lo.z;
        }

        std::uint32_t motionIndexOf(const BodyHandle& handle)
        {
            return handle.isValid() ? handle.world->GetBody(handle.bodyId).motionIndex : 0;
        }

        constexpr int SHAPE_COMPRESSED_MESH = 5;
        constexpr int SHAPE_EXTERN_MESH = 6;
        constexpr int SHAPE_STATIC_COMPOUND = 7;
    }

    std::uint32_t getFilterInfo(const BodyHandle& handle)
    {
        return handle.isValid() ? handle.world->GetBody(handle.bodyId).collisionFilterInfo : 0u;
    }

    void placeBodyAt(const BodyHandle& handle, const model::Pose& pose)
    {
        if (!handle.isValid()) {
            return;
        }
        // hkTransform: three rotation columns, then the translation (the layout readBodyState reads from the body).
        RE::hkTransformf transform;
        transform.SetIdentity();
        const model::Vec3 columns[3] = { pose.rot.axisX(), pose.rot.axisY(), pose.rot.axisZ() };
        for (int c = 0; c < 3; ++c) {
            transform.rotation.entry[c][0] = columns[c].x;
            transform.rotation.entry[c][1] = columns[c].y;
            transform.rotation.entry[c][2] = columns[c].z;
            transform.rotation.entry[c][3] = 0.0f;
        }
        transform.translation.x = pose.pos.x;
        transform.translation.y = pose.pos.y;
        transform.translation.z = pose.pos.z;
        transform.translation.w = 0.0f;
        handle.world->SetBodyTransform(handle.bodyId, transform, 0);
        RE::hkVector4f linear(0.0f, 0.0f, 0.0f, 0.0f);
        RE::hkVector4f angular(0.0f, 0.0f, 0.0f, 0.0f);
        handle.world->SetBodyVelocity(handle.bodyId, linear, angular);
    }

    void setBodyLayer(const BodyHandle& handle, const RE::COL_LAYER layer)
    {
        if (!handle.isValid()) {
            return;
        }
        const auto filter = handle.world->GetBody(handle.bodyId).collisionFilterInfo;
        handle.world->SetBodyCollisionFilterInfo(handle.bodyId, (filter & ~0x7Fu) | static_cast<std::uint32_t>(layer), 1);
    }

    bool makeKeyframed(BodyHandle& handle)
    {
        if (!handle.collisionObject) {
            return false;
        }
        handle.collisionObject->SetMotionType(static_cast<RE::hknpMotionPropertiesId::Preset>(2));
        resolveBody(handle);
        return bodyMotionPreset(handle) == 2;
    }

    RE::COL_LAYER getBodyLayer(const BodyHandle& handle)
    {
        if (!handle.isValid()) {
            return RE::COL_LAYER::kNonCollidable;
        }
        return static_cast<RE::COL_LAYER>(handle.world->GetBody(handle.bodyId).collisionFilterInfo & 0x7Fu);
    }

    bool makeDynamic(BodyHandle& handle, const float massKg, const model::Vec3& boxMin, const model::Vec3& boxMax, const bool forceBox, const HullShape& hull, std::string& diagnostics)
    {
        if (!handle.collisionObject) {
            diagnostics = "no collision object";
            return false;
        }
        const int before = bodyMotionPreset(handle);
        const int shapeType = bodyShapeType(handle);
        std::string shapeNote;
        if (forceBox || shapeType == SHAPE_COMPRESSED_MESH || shapeType == SHAPE_EXTERN_MESH || shapeType == SHAPE_STATIC_COMPOUND) {
            // Size the box from the mesh collision itself when it can be measured, else from the given bounds.
            model::Vec3 lo = boxMin;
            model::Vec3 hi = boxMax;
            model::Vec3 meshLo, meshHi;
            if (!forceBox && shapeLocalAabb(handle, meshLo, meshHi)) {
                logger::info(
                    "Vehicle: mesh collision AABB ({:.2f}, {:.2f}, {:.2f}) .. ({:.2f}, {:.2f}, {:.2f}) m; object bounds ({:.2f}, {:.2f}, {:.2f}) .. ({:.2f}, {:.2f}, {:.2f}) m",
                    meshLo.x,
                    meshLo.y,
                    meshLo.z,
                    meshHi.x,
                    meshHi.y,
                    meshHi.z,
                    boxMin.x,
                    boxMin.y,
                    boxMin.z,
                    boxMax.x,
                    boxMax.y,
                    boxMax.z);
                lo = { meshLo.x, meshLo.y, (std::max)(meshLo.z, boxMin.z) };
                hi = meshHi;
            }
            auto* box = buildBoxShape(lo, hi, hull);
            if (!box) {
                diagnostics = std::format("shape type {} cannot be dynamic and the box chassis could not be built", shapeType);
                return false;
            }
            handle.world->SetBodyShape(handle.bodyId, box);
            shapeNote = std::format(", shape {} replaced by a box chassis {:.2f} x {:.2f} x {:.2f} m (now type {})",
                shapeType,
                hi.x - lo.x,
                hi.y - lo.y,
                hi.z - lo.z,
                bodyShapeType(handle));
        }

        // A: Bethesda's own switch. Works on bodies that were created with a motion.
        if (!bodyIsDynamic(handle)) {
            handle.collisionObject->SetMotionType(RE::hknpMotionPropertiesId::Preset::DYNAMIC);
        }
        if (!resolveBody(handle)) {
            diagnostics = "body lost after switching to dynamic";
            return false;
        }
        std::string route = "wrapper";
        if (motionIndexOf(handle) == 0) {
            // B: the body was created static (shared motion 0). Give it its own motion through Bethesda's
            // keyframed path, then switch that motion to dynamic.
            logger::info("Vehicle: static body has no motion; trying the keyframed route");
            using SetBodyKeyframed_t = void (*)(void*, std::uint32_t);
            static REL::Relocation<SetBodyKeyframed_t> setBodyKeyframed{ REL::Offset(RVA_SET_BODY_KEYFRAMED) };
            setBodyKeyframed(handle.world, handle.bodyId.value);
            logger::info("Vehicle: after keyframed: motion index {}, motion type {}", motionIndexOf(handle), bodyMotionPreset(handle));
            if (motionIndexOf(handle) != 0) {
                handle.collisionObject->SetMotionType(RE::hknpMotionPropertiesId::Preset::DYNAMIC);
                resolveBody(handle);
                if (!bodyIsDynamic(handle) && hasAddressId(275629)) {
                    RE::hknpMotionPropertiesId_Handle dynamicId{};
                    dynamicId.value = 1;
                    handle.world->SetBodyMotionProperties(handle.bodyId, dynamicId);
                }
                route = "keyframed route";
            }
        }
        const int after = bodyMotionPreset(handle);
        if (!bodyIsDynamic(handle)) {
            diagnostics = std::format("motion type {} -> {} (expected a simulated type, not static 0 or keyframed 2){}", before, after, shapeNote);
            return false;
        }
        // Inertia from the (new) shape, then the profile's mass.
        using RebuildMass_t = void (*)(void*, std::uint32_t, int);
        static REL::Relocation<RebuildMass_t> rebuildMass{ REL::Offset(RVA_REBUILD_MOTION_MASS) };
        rebuildMass(handle.world, motionIndexOf(handle), 0);
        handle.collisionObject->SetMass(massKg);
        handle.world->ActivateBody(handle.bodyId);
        diagnostics = std::format("motion type {} -> {} (dynamic, {}), motion {}, mass {} kg, body {}{}",
            before,
            after,
            route,
            motionIndexOf(handle),
            massKg,
            handle.bodyId.value,
            shapeNote);
        return true;
    }

    void applyImpulseAt(const BodyHandle& handle, const model::Vec3& impulse, const model::Vec3& point, const bool useCollisionObject)
    {
        RE::hkVector4f j(impulse.x, impulse.y, impulse.z, 0.0f);
        RE::hkVector4f p(point.x, point.y, point.z, 0.0f);
        if (useCollisionObject && handle.collisionObject) {
            handle.collisionObject->ApplyPointImpulseAt(j, p);
        } else if (handle.isValid()) {
            handle.world->ApplyBodyImpulseAt(handle.bodyId, j, p);
        }
    }

    void setBodyMass(const BodyHandle& handle, const float massKg)
    {
        if (handle.collisionObject && massKg > 0.0f) {
            handle.collisionObject->SetMass(massKg);
        }
    }

    void activate(const BodyHandle& handle)
    {
        if (handle.isValid()) {
            handle.world->ActivateBody(handle.bodyId);
        }
    }

    RayHit castRay(RE::bhkWorld* world, const RE::NiPoint3& start, const RE::NiPoint3& end, const std::uint32_t filterInfo, const std::uint32_t ignoreBodyId,
        const std::uint64_t ignoreLayerMask, const bool releaseWorldReference)
    {
        RayHit result;
        if (!world) {
            return result;
        }
        const bool release = releaseWorldReference && pickWorldReleaseVerified();
        const RE::NiPoint3 dir = end - start;
        const float totalLength = dir.Length();
        if (totalLength < 1e-3f) {
            return result;
        }
        RE::NiPoint3 from = start;
        float travelled = 0.0f;
        for (int attempt = 0; attempt < MAX_RAY_RECASTS; ++attempt) {
            RE::bhkPickData pick;
            pick.SetStartEnd(from, end);
            pick.collisionFilter.filter = filterInfo;
            const bool picked = world->PickObject(pick) && pick.HasHit();
            const float fraction = picked ? std::clamp(pick.GetHitFraction(), 0.0f, 1.0f) : 1.0f;
            const auto hitBody = pick.result.hitBodyInfo.m_bodyId.value;
            const auto hitFilter = pick.result.hitBodyInfo.m_shapeCollisionFilterInfo.storage;
            const auto hitNormal = fromHk(pick.result.normal);
            const auto hitMaterial = static_cast<std::uint16_t>(pick.result.hitBodyInfo.m_shapeMaterialId.value);
            if (release) {
                havokRelease(std::exchange(pick.allHitsResult, nullptr));
            }
            if (!picked) {
                return result;
            }
            const float remaining = totalLength - travelled;
            const float hitDistance = fraction * remaining;
            if (hitBody == ignoreBodyId || f4vr::isCollisionLayerInMask(hitFilter, ignoreLayerMask)) {
                // Skip it: start again just past the hit.
                travelled += hitDistance + 1.0f;
                if (travelled >= totalLength) {
                    return result;
                }
                from = start + dir * (travelled / totalLength);
                continue;
            }
            result.hit = true;
            result.fraction = (travelled + hitDistance) / totalLength;
            result.point = start + dir * result.fraction;
            result.normal = model::normalizeOr(hitNormal, { 0, 0, 1 });
            result.bodyId = hitBody;
            result.filterInfo = hitFilter;
            result.materialId = hitMaterial;
            return result;
        }
        return result;
    }

    StepListener& StepListener::get()
    {
        static StepListener instance;
        return instance;
    }

    StepListener::StepListener()
    {
        // Never freed: the engine may still hold the pointer after a world goes away.
        _native = new Native();
        _native->owner = this;
    }

    void StepListener::setCallback(const Callback callback, void* userData)
    {
        _userData.store(userData, std::memory_order_release);
        _callback.store(callback, std::memory_order_release);
    }

    void StepListener::clearCallback()
    {
        _callback.store(nullptr, std::memory_order_release);
        _userData.store(nullptr, std::memory_order_release);
    }

    void StepListener::registerForNextStep(RE::bhkWorld* world)
    {
        if (world) {
            world->AddPhysicsStepListener(*reinterpret_cast<RE::bhkIWorldStepListener*>(_native));
        }
    }

    void StepListener::onBeforeStep(const float deltaSeconds)
    {
        _stepCount.fetch_add(1, std::memory_order_relaxed);
        const auto callback = _callback.load(std::memory_order_acquire);
        if (callback) {
            callback(_userData.load(std::memory_order_acquire), deltaSeconds);
        }
    }

    void logEngineAddresses()
    {
        const auto base = REL::Module::get().base();
        const auto addListener = REL::ID(137330).address() - base;
        const auto pickObject = REL::ID(547249).address() - base;
        logger::info("Engine addresses: AddPhysicsStepListener RVA 0x{:X} (ROCK uses 0x{:X}) {}",
            addListener,
            ROCK_RVA_ADD_STEP_LISTENER,
            addListener == ROCK_RVA_ADD_STEP_LISTENER ? "match" : "MISMATCH");
        logger::info("Engine addresses: PickObject RVA 0x{:X} (ROCK uses 0x{:X}) {}", pickObject, ROCK_RVA_PICK_OBJECT, pickObject == ROCK_RVA_PICK_OBJECT ? "match" : "MISMATCH");

        // Every address-library ID this plugin calls. A missing one would end the game at its first use.
        constexpr std::array<std::pair<std::uint64_t, const char*>, 22> ids{ {
            { 137330, "bhkWorld::AddPhysicsStepListener" },
            { 547249, "bhkWorld::PickObject" },
            { 526783, "bhkPickData ctor" },
            { 747470, "bhkPickData::SetStartEnd" },
            { 1181584, "bhkPickData::HasHit" },
            { 476687, "bhkPickData::GetHitFraction" },
            { 1326073, "TESObjectCELL::GetbhkWorld" },
            { 526734, "bhkPhysicsSystem::GetBodyId" },
            { 200912, "bhkNPCollisionObject::SetMotionType" },
            { 70998, "bhkNPCollisionObject::SetMass" },
            { 229984, "bhkNPCollisionObject::GetCollisionFilterInfo" },
            { 844417, "bhkNPCollisionObject::ApplyPointImpulseAt" },
            { 531033, "hknpWorld::SetBodyShape" },
            { 275629, "hknpWorld::SetBodyMotionProperties" },
            { 405369, "hknpWorld::GetBodyAngularVelocity" },
            { 800106, "hknpWorld::ApplyBodyImpulseAt" },
            { 904571, "hknpWorld::ActivateBody" },
            { 500304, "TESDataHandler::CreateReferenceAtLocation" },
            { 711558, "TESDataHandler singleton" },
            { 84112, "RTDynamicCast (not used)" },
            { 618186, "hknpWorld::SetBodyMotion (not used)" },
            { 527037, "hknpBSWorld::setBodyKeyframed (not used)" },
        } };
        std::string missing;
        for (const auto& [id, name] : ids) {
            if (!hasAddressId(id)) {
                missing += std::format(" {} ({})", id, name);
            }
        }
        logger::info("Address library: {}", missing.empty() ? "all IDs present" : "MISSING:" + missing);
    }
}
