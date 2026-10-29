#pragma once

// The one place that talks to Havok (hknp) and Bethesda's bhk wrappers. Everything else in the plugin
// works in the model's math types (meters, model::Vec3/Quat) and calls through here.
//
// Threading: functions marked [game] run on the game thread only; [step] are safe inside a physics step
// listener callback (they only read body/motion arrays or call per-body hknpWorld functions).

#include <atomic>
#include <cstdint>
#include <optional>
#include <vector>
#include <string>

#include "model/VMath.h"

namespace vfvr::physics
{
    // A Havok body behind a scene-graph node.
    struct BodyHandle
    {
        RE::NiAVObject* node = nullptr; // node that owns the collision object
        RE::bhkNPCollisionObject* collisionObject = nullptr;
        RE::bhkWorld* bhkWorld = nullptr;
        RE::hknpWorld* world = nullptr;
        RE::hknpBodyId bodyId{};

        bool isValid() const
        {
            return collisionObject && world && bodyId.value != 0x7FFF'FFFF;
        }
    };

    // Conversions between the game's NiPoint3 (game units) and the model (meters).
    inline model::Vec3 toMeters(const RE::NiPoint3& p)
    {
        return model::Vec3{ p.x, p.y, p.z } * model::GAME_TO_HAVOK;
    }

    inline RE::NiPoint3 toGame(const model::Vec3& v)
    {
        return { v.x * model::HAVOK_TO_GAME, v.y * model::HAVOK_TO_GAME, v.z * model::HAVOK_TO_GAME };
    }

    inline model::Vec3 toVec(const RE::NiPoint3& p)
    {
        return { p.x, p.y, p.z };
    }

    inline RE::NiPoint3 toNi(const model::Vec3& v)
    {
        return { v.x, v.y, v.z };
    }

    // World pose of a scene-graph node in meters (the node's own frame: x right, y forward, z up). [game]
    model::Pose nodePoseMeters(const RE::NiAVObject* node);

    // hknpWorld behind a bhkWorld (bhkWorld + 0x60, confirmed by CommonLibF4VR and ROCK).
    RE::hknpWorld* getHknpWorld(RE::bhkWorld* bhkWorld);

    // The bhkWorld of the cell a reference is in. [game]
    RE::bhkWorld* getBhkWorld(const RE::TESObjectREFR* ref);

    // First node (depth-first, root included) that carries a Havok collision object, and its body. [game]
    std::optional<BodyHandle> findBody(RE::NiAVObject* root, std::string& diagnostics);

    // Every node under `root` (to `maxDepth` levels) with a Havok collision object whose body is in the world, in tree order: the biped
    // bodies of a skeleton (the hit boxes the projectiles meet).
    std::vector<BodyHandle> findBodies(RE::NiAVObject* root, int maxDepth, std::size_t maxCount);

    // Re-resolve the body id of a collision object (it can change when the motion type changes). [game]
    bool resolveBody(BodyHandle& handle);

    // Body state for the model. [step]
    struct BodyStateMeters
    {
        model::Pose frame; // body transform (shape origin): matches the owning node's frame
        model::Vec3 com;
        model::Vec3 linVel;
        model::Vec3 angVel;
        bool valid = false;
    };

    BodyStateMeters readBodyState(const BodyHandle& handle, bool angularFromMotion);

    // Motion type of the body as Havok stores it: 0 static, 1 dynamic, 2 keyframed... [game/step]
    int bodyMotionPreset(const BodyHandle& handle);

    // True when the body has its own motion and a simulated preset (dynamic, debris, box inertia...), i.e. anything
    // but static (0) and keyframed (2). A bottle cap is created as DEBRIS (3), which is dynamic. [game/step]
    bool bodyIsDynamic(const BodyHandle& handle);

    // Hknp shape type of the body (hknpShapeType::Enum), or -1. [game]
    int bodyShapeType(const BodyHandle& handle);

    // The convex chassis that replaces a car's collision: a box with its lower edges cut into ramps, so a kerb or a ditch edge slides
    // the car up instead of stopping it. `lengthAxis` is the body axis (0 = x, 1 = y) the car's front and rear lie on.
    struct HullShape
    {
        int lengthAxis = 1;
        float rampLength = 0.4f; // m the bottom edge is pulled in at both ends
        float rampHeight = 0.4f; // m up at which the ramp reaches the full length
        float sideInset = 0.12f; // m the bottom edge is pulled in at the sides
    };

    // Switch a body to dynamic and give it a mass, through Bethesda's wrapper. When its shape is a triangle
    // mesh (vanilla cars: static compressed mesh, which Havok cannot simulate as dynamic) the shape is first
    // replaced by a convex box from `boxMin`..`boxMax` (body space, meters). [game]
    // `forceBox` replaces the shape even when it is convex (a carrier object's own small shape).
    bool makeDynamic(BodyHandle& handle, float massKg, const model::Vec3& boxMin, const model::Vec3& boxMax, bool forceBox, const HullShape& hull, std::string& diagnostics);

    // Change the mass of a dynamic body (and its inertia, which scales with it). [game]
    void setBodyMass(const BodyHandle& handle, float massKg);

    // Put a body on another collision layer (keeps its group bits). [game]
    void setBodyLayer(const BodyHandle& handle, RE::COL_LAYER layer);

    // Move a body to `pose` (its own frame, as readBodyState gives it) and stop it: the recovery of a body that fell out of the world. Not for
    // regular movement. [step]
    void placeBodyAt(const BodyHandle& handle, const model::Pose& pose);
    std::uint32_t getFilterInfo(const BodyHandle& handle);

    // Make a simulated body keyframed (it keeps its place instead of falling or being pushed; the engine then follows its node). Returns whether it ended up keyframed. [game]
    bool makeKeyframed(BodyHandle& handle);

    // The collision layer a body is on now. [game]
    RE::COL_LAYER getBodyLayer(const BodyHandle& handle);

    // Apply an impulse (N*s) at a world point (meters). [step]
    void applyImpulseAt(const BodyHandle& handle, const model::Vec3& impulse, const model::Vec3& point, bool useCollisionObject);

    // Keep the body awake. [game]
    void activate(const BodyHandle& handle);

    // Ray against the world, skipping hits on `ignoreBodyId` and on layers in `ignoreLayerMask` by re-casting
    // past them (up to a few times). Points in game units. [game]
    struct RayHit
    {
        bool hit = false;
        RE::NiPoint3 point;
        model::Vec3 normal{ 0, 0, 1 };
        float fraction = 1.0f;
        std::uint32_t bodyId = 0x7FFF'FFFF;
        std::uint32_t filterInfo = 0;
        std::uint16_t materialId = 0;
    };

    RayHit castRay(RE::bhkWorld* world, const RE::NiPoint3& start, const RE::NiPoint3& end, std::uint32_t filterInfo, std::uint32_t ignoreBodyId, std::uint64_t ignoreLayerMask,
        bool releaseWorldReference);

    // Physics step listener. Bethesda clears the listener list on every world update, so the listener is
    // re-registered every frame; the object itself lives for the whole process (the engine may still hold
    // it after a world is gone). The callback runs once per Havok (sub)step, before collide, with that step's
    // delta time.
    class StepListener
    {
    public:
        using Callback = void (*)(void* userData, float deltaSeconds);

        static StepListener& get();

        void setCallback(Callback callback, void* userData);
        void clearCallback();

        // Register for the next update of this world. [game]
        void registerForNextStep(RE::bhkWorld* world);

        std::uint64_t stepCount() const
        {
            return _stepCount.load(std::memory_order_relaxed);
        }

        // Called by the engine through the native listener vtable.
        void onBeforeStep(float deltaSeconds);

        // The object handed to the engine (vtable pointer first). Public so the vtable thunks can reach the owner.
        struct Native;

    private:
        StepListener();

        Native* _native = nullptr;
        std::atomic<Callback> _callback{ nullptr };
        std::atomic<void*> _userData{ nullptr };
        std::atomic<std::uint64_t> _stepCount{ 0 };
    };

    // Whether the VR address library has this ID (a missing ID ends the game at its first use).
    bool hasAddressId(std::uint64_t id);

    // One-time log of the engine addresses this file depends on, to compare against other mods. [game]
    void logEngineAddresses();
}
