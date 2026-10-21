#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace vfvr
{
    /**
     * What the save game remembers of the health system: per vehicle profile its health and radiation reserve (a vehicle spawned again from the
     * same profile takes them over, see bRememberVehicleState), the resistance modifier a vehicle had on the player when the game was saved
     * (undone on load, see RadProtection), and whether the driver's cover perk was given to the player (see OccupantShield). Lives in the
     * plugin's F4SE co-save next to the trunks: F4SE gives a plugin one set of save callbacks, so Trunk's callbacks write and read these
     * records too.
     */
    class HealthStore
    {
    public:
        struct Entry
        {
            float health = -1.0f; // < 0 = unknown
            float shield = -1.0f;
        };

        static HealthStore& get();

        // The active vehicle's latest state, every frame. [game]
        void remember(const std::string& profileId, float health, float shield);

        // The remembered state of a profile, if any. [game]
        std::optional<Entry> lookup(const std::string& profileId) const;

        // The driver's cover perk: given to the player in this save line. [game]
        bool driverPerkGiven() const
        {
            return _driverPerkGiven;
        }

        void markDriverPerkGiven()
        {
            _driverPerkGiven = true;
        }

        // Co-save records, from Trunk's F4SE callbacks. [any]
        void write(const F4SE::SerializationInterface* intfc) const;
        // True when the record was one of ours (and was read).
        bool read(const F4SE::SerializationInterface* intfc, std::uint32_t type, std::uint32_t version, std::uint32_t length);
        void reset();

    private:
        mutable std::mutex _mutex;
        std::map<std::string, Entry> _entries;
        bool _driverPerkGiven = false;
    };
}
