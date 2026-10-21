#pragma once

#include <string>

#include <array>
#include <chrono>
#include <vector>

#include "model/VMath.h"
#include "physics/HavokBridge.h"
#include "model/SeatBody.h"
#include "player/DriveHud.h"

namespace vfvr
{
    class CsvLog;
    class Vehicle;

    /**
     * The player in the driver's seat, first person.
     *
     * The player is never parented to the vehicle. Every frame the actor is moved (as the game moves it
     * when walking) so that the head lands on the seat's eye point, and the VR room is turned with the
     * vehicle's heading (as the game's own smooth turn does). Head tracking and room-scale stay the
     * player's: leaning or turning the head moves the view relative to the seat. Pitch and roll of the
     * vehicle are not applied, so the horizon stays level.
     *
     * The math, all in meters, with the room's heading R and scale s, and positions in room space
     * measured each frame (head h, actor a):
     *   room origin wanted  = seatEye - R * s * h0          (h0 = head in the room when calibrated)
     *   actor position      = room origin + R * s * a       (the actor's place in the room is kept)
     * so the head is exactly on the seat at calibration, and leaning moves it by R * s * (h - h0).
     */
    class DriverSeat
    {
    public:
        // [game] Hand the controllers back to the game (a menu is open) or take them again (back to driving). Only while seated.
        void setGameInputSuppressed(bool suppressed);

        bool isDriving() const
        {
            return _driving;
        }

        // Head close enough to the driver's seat to get in. [game]
        bool canEnter(const Vehicle& vehicle) const;
        float headToSeatMeters(const Vehicle& vehicle) const;

        void enter(Vehicle& vehicle);
        // Leave the seat. `vehicle` may be null or invalid (vehicle gone): the player then stays where they are.
        void exit(Vehicle* vehicle, const std::string& reason);
        // Drop all state without touching the game (after a save load, where the game already reset it).
        void reset();

        // Re-measure the head in the room and face the vehicle's forward. [game]
        void recenter(const Vehicle& vehicle);

        // Switch between the driver's seat and a chase camera behind the vehicle. [game]
        void toggleThirdPerson();
        bool isThirdPerson() const { return _thirdPerson; }
        // Put the chase camera straight behind the vehicle. [game]
        void resetOrbit()
        {
            _orbitYaw = 0.0f;
            _orbitIdle = 0.0f;
        }
        // Right-stick state for the chase camera orbit (-1..1 each). [game]
        void setCameraStick(const float x, const float y)
        {
            _stickX = x;
            _stickY = y;
        }

        // Put the player in the seat for this frame. With `placeModel` the seat also draws the vehicle model (from FRIK's FrameBegin, right after
        // Vehicle::beginCameraFrame): where the engine really has the camera, see bLockModelToCamera. Without it the model has been drawn already. [game]
        void updateCamera(Vehicle& vehicle, CsvLog* cameraLog, bool placeModel);

        // FRIK's FrameEnd: what the engine did with the head node since updateCamera placed the player; completes the camera CSV row. [game]
        void onFrameEnd(CsvLog* cameraLog);

        // Third person with bThirdPersonBodyInSeat: move the body FRIK built under the chase camera into the driver's seat. Call once FRIK's frame is
        // final (its AfterWorldFinal phase). Nothing is parented: the next frame FRIK builds the body from scratch again. [game]
        void placeBodyInSeat(const Vehicle& vehicle);
        // FRIK's FrameBegin: where the weapon is now, against where placeBodyInSeat left it (bLogPlayerBody diagnostics). [game]
        void weaponCheckAtFrameBegin();
        // The body is shown in the seat this frame (FRIK must not hide it). [game]
        bool bodyInSeat() const { return _bodyMoveValid; }

        // Debug draw of the seat and exit point. [game]
        void drawDebug(const Vehicle& vehicle) const;

    private:
        struct RoomState
        {
            model::Vec3 origin; // world, meters
            float heading = 0.0f;
            float scale = 1.0f;
            model::Vec3 hmdWorld;
            model::Vec3 hmdLocal; // head in room space (meters, unscaled)
            float hmdLocalHeading = 0.0f; // head yaw inside the room
            float hmdHeading = 0.0f; // head yaw in the world, as the engine renders it
            model::Vec3 actorWorld;
            model::Vec3 actorLocal;
        };

        // While driving, the room heading is the one we commanded: the room node's own world rotation does not
        // follow the VR yaw the engine applies, so measuring it gives a stale value.
        bool readRoom(RoomState& out) const;
        model::Pose vehiclePose(const Vehicle& vehicle) const;
        model::Vec3 seatEyeWorld(const Vehicle& vehicle) const;
        void calibrate(const Vehicle& vehicle);
        void applyRoomYaw(float targetRoomHeading, const RoomState& room);
        void setPlayerCollision(bool driving);
        void setPlayerBodyLayer(bool driving, const Vehicle* vehicle);

        bool _driving = false;

        // The player's own character-controller body, moved to a layer nothing collides with while driving (see setPlayerBodyLayer).
        bool _playerBodyChanged = false;
        physics::BodyHandle _playerBody{};
        std::uint32_t _playerBodyFilter = 0;
        int _playerBodyTries = 0;

        // Calibration.
        model::Vec3 _hmdLocal0;
        float _headingOffset = 0.0f; // room heading - vehicle heading wanted
        float _yaw0 = 0.0f; // engine VR yaw at calibration
        float _roomHeading0 = 0.0f; // measured room heading at calibration
        float _yawSign = 1.0f;
        bool _yawSignChecked = false;
        int _yawCheckFrames = 0; // frames in a row in which the head went the mirrored way
        int _yawConfirmFrames = 0; // frames in a row in which the head went the commanded way
        int _yawSettle = 0; // frames to wait after (re)starting a command before judging it
        int _yawFlips = 0;
        float _hmdHeading0 = 0.0f; // head yaw in the world at calibration
        bool _headingTracked = false; // readRoom uses the commanded heading
        float _lastTargetRoomHeading = 0.0f;
        model::Vec3 _lastSeatRaw{};
        bool _hasLastSeatRaw = false;
        model::Vec3 _lastPlacedEye{}; // where the previous frame put the camera's eye point (the head should be there, plus the player's own lean)
        bool _hasLastPlaced = false;
        model::Quat _lastPlacedRot{};
        model::Vec3 _lastActorTarget{}; // where the previous frame put the player actor: the engine's own movement shows as drift from it
        bool _hasLastActor = false;

        // The model is drawn where the engine has the camera (one frame behind our placement): the pose the previous frame placed the player with.
        std::array<model::Pose, 8> _rawHist{}; // [0] = the pose of the previous frame, [1] the one before...
        int _rawHistCount = 0;
        model::Pose _prevRawPose{};
        bool _hasPrevRaw = false;
        // Does the engine move the head node within the frame we place the player in (so the model must not be held back), or only on its next update?
        // Measured at FrameEnd: the head's movement along the vehicle's axis against the movement we asked for.
        bool _lockSuspended = false;
        bool _followDecided = false;
        int _followSamples = 0;
        float _followMovedCm = 0.0f;
        float _followAskedCm = 0.0f;
        // The frame just placed, completed by FRIK's FrameEnd: its camera CSV row (empty without telemetry) is written there, or without the FrameEnd
        // columns when FrameEnd does not come.
        std::string _pendingRow;
        bool _beginPending = false;
        model::Vec3 _beginHmd{};
        model::Quat _beginRot{};
        float _beginAskFwdCm = 0.0f;
        bool _beginLockEligible = false;
        void flushPendingRow(CsvLog* cameraLog);

        // Getting in: the eyes glide from where the head was (vehicle frame, so a rolling car carries it) into the seat over fEnterGlideSeconds.
        bool _glideActive = false;
        model::Vec3 _glideFromLocal{};
        std::chrono::steady_clock::time_point _glideStart{};

        DriveHud _hud;

        // Comfort filter on the seat height.
        float _filteredSeatZ = 0.0f;
        bool _filterReady = false;
        bool _thirdPerson = false;
        float _stickX = 0.0f;
        float _stickY = 0.0f;
        float _orbitYaw = 0.0f; // rad, chase camera angle around the car (0 = behind)
        float _orbitIdle = 0.0f; // s since the stick was last moved
        // Chase camera arm, shortened by what stands between the camera and the car (see model/CameraArm.h).
        float _cameraArm = 0.0f; // m from the pivot above the seat
        bool _cameraArmReady = false;
        model::Vec3 chaseCameraEye(const Vehicle& vehicle, const model::Vec3& seatEye, float heading, float dt);
        // Switching view: 0 = the eyes in the seat, 1 = at the chase camera; it moves between the two over fCameraSwitchSeconds.
        float _viewBlend = 0.0f;
        // The body is moved into the seat this frame (the camera is away from it).
        bool _bodyMoveValid = false;
        // Seated legs: per skeleton, the bone indices of each leg and of the bones below its thigh and its shin (found by name once).
        struct LegBones
        {
            int thigh = -1;
            int calf = -1;
            int foot = -1;
            std::vector<int> belowThigh; // the thigh and every bone under it
            std::vector<int> belowCalf;
        };
        struct SkeletonLegs
        {
            const void* tree = nullptr;
            int count = 0;
            LegBones legs[2]; // left, right
        };
        SkeletonLegs _legs3rd;
        SkeletonLegs _legs1st;
        // The weapon is holstered for the drive (bHolsterWeaponWhileDriving): whether it was drawn when the player got in, and when it was last
        // put away; plus the weapon check across frames (diagnostics).
        bool _weaponWasDrawn = false;
        std::uint64_t _lastHolsterMs = 0;
        void keepWeaponHolstered();
        bool _weaponCheckPending = false;
        RE::NiPoint3 _weaponAfterMove{};
        std::uint64_t _lastWeaponLogMs = 0;

        // Things to restore when leaving.
        bool _collisionChanged = false;
        std::uint32_t _savedCollisionFilter = 0;
        bool _frikBlocked = false;

        std::uint64_t _frame = 0;
    };
}
