#pragma once

#include <cstdint>

namespace vfvr
{
    class Vehicle;

    /**
     * The vehicle's radiation protection on the real player. While the player sits in a vehicle whose RadShield is active, its resistance
     * points are added to the player's radExposureResistance as a temporary modifier: the game applies its own resistance curve, so there is
     * no second pool of rads and nothing is applied twice. The radiation the vehicle keeps off the player (estimated every tick from the rads
     * the player does get, with the curve inverted) uses the reserve up; when it runs out the modifier comes off in the same tick and the
     * player gets the game's normal radiation.
     *
     * The modifier is also taken off on leaving the seat, when the vehicle goes, and after a save load (the co-save remembers what was on the
     * player when the game was saved, see HealthStore).
     */
    class RadProtection
    {
    public:
        static RadProtection& get();

        // Every frame: `vehicle` the active one (null = none), `seated` whether the player sits in it. [game]
        void onFrameUpdate(Vehicle* vehicle, bool seated);

        // Take the modifier off, whatever the state (vehicle release). [game]
        void clear(const char* reason);

        // The resistance points on the player right now (0 = none), for the co-save.
        float applied() const
        {
            return _applied;
        }

        // The ambient radiation estimated at the last tick (rads/s), for the log.
        float ambientEstimate() const
        {
            return _ambientEstimate;
        }

        // A save load found this modifier on the player (written by HealthStore when the game was saved): taken off on the next frame. [any]
        void undoOnLoad(const float applied)
        {
            _undoPending = applied;
        }

        // A save load replaced the player's values: nothing of ours is on them any more (the loaded save's own share is undone by undoOnLoad). [game]
        void onSessionLoaded()
        {
            _applied = 0.0f;
            _hasLastRads = false;
        }

    private:
        bool modify(float delta);
        void tick(Vehicle& vehicle, float dt);

        float _applied = 0.0f;
        float _undoPending = 0.0f;
        std::uint64_t _lastTickMs = 0;
        float _lastRads = 0.0f;
        bool _hasLastRads = false;
        float _ambientEstimate = 0.0f;
        std::uint64_t _lastLogMs = 0;
    };
}
