#pragma once

#include <cstdint>
#include <string>

namespace vfvr
{
    /**
     * Diagnostic log of the player's vitals ("Vitals:" lines): health, rads and their modifiers, radiation resistance, the Pip-Boy's current
     * rads reading, the level and the protection flags. For checking what the game does to the player in and out of the seat, and in a
     * radiation zone with and without the occupant shield.
     */
    class VitalsProbe
    {
    public:
        static VitalsProbe& get();

        // Every frame: logs every 2 s while driving, every 10 s otherwise, and within 0.5 s of a change. [game]
        void onFrameUpdate(bool driving);

    private:
        std::uint64_t _lastLogMs = 0;
        std::string _lastState;
    };
}
