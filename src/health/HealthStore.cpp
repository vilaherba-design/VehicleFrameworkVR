#include "health/HealthStore.h"

#include "health/RadProtection.h"

namespace vfvr
{
    namespace
    {
        constexpr std::uint32_t RECORD_HEALTH = 0x56484C54; // 'VHLT': per profile, health and radiation reserve
        constexpr std::uint32_t RECORD_RADS = 0x56524144; // 'VRAD': the resistance modifier on the player at save time
        constexpr std::uint32_t RECORD_PERK = 0x5650524B; // 'VPRK': the driver's cover perk was given to the player
        constexpr std::uint32_t RECORD_VERSION = 1;
        constexpr std::uint32_t MAX_ENTRIES = 64;
        constexpr std::uint32_t MAX_PROFILE = 256;
    }

    HealthStore& HealthStore::get()
    {
        static HealthStore instance;
        return instance;
    }

    void HealthStore::remember(const std::string& profileId, const float health, const float shield)
    {
        if (profileId.empty()) {
            return;
        }
        const std::scoped_lock lock(_mutex);
        auto& entry = _entries[profileId];
        entry.health = health;
        entry.shield = shield;
    }

    std::optional<HealthStore::Entry> HealthStore::lookup(const std::string& profileId) const
    {
        const std::scoped_lock lock(_mutex);
        const auto it = _entries.find(profileId);
        if (it == _entries.end()) {
            return std::nullopt;
        }
        return it->second;
    }

    void HealthStore::write(const F4SE::SerializationInterface* intfc) const
    {
        {
            const std::scoped_lock lock(_mutex);
            if (!intfc->OpenRecord(RECORD_HEALTH, RECORD_VERSION)) {
                logger::error("HealthStore: could not write the vehicle health record");
            } else {
                const auto count = static_cast<std::uint32_t>((std::min)(_entries.size(), static_cast<std::size_t>(MAX_ENTRIES)));
                intfc->WriteRecordData(count);
                std::uint32_t written = 0;
                for (const auto& [profile, entry] : _entries) {
                    if (written++ >= count) {
                        break;
                    }
                    const auto length = static_cast<std::uint32_t>((std::min)(profile.size(), static_cast<std::size_t>(MAX_PROFILE)));
                    intfc->WriteRecordData(length);
                    intfc->WriteRecordData(profile.data(), length);
                    intfc->WriteRecordData(entry.health);
                    intfc->WriteRecordData(entry.shield);
                }
            }
        }
        const float applied = RadProtection::get().applied();
        if (!intfc->OpenRecord(RECORD_RADS, RECORD_VERSION)) {
            logger::error("HealthStore: could not write the radiation record");
        } else {
            intfc->WriteRecordData(applied);
        }
        const std::uint8_t perkGiven = _driverPerkGiven ? 1 : 0;
        if (!intfc->OpenRecord(RECORD_PERK, RECORD_VERSION)) {
            logger::error("HealthStore: could not write the driver perk record");
        } else {
            intfc->WriteRecordData(perkGiven);
        }
    }

    bool HealthStore::read(const F4SE::SerializationInterface* intfc, const std::uint32_t type, const std::uint32_t version, std::uint32_t)
    {
        if (version != RECORD_VERSION) {
            return false;
        }
        if (type == RECORD_HEALTH) {
            std::map<std::string, Entry> loaded;
            std::uint32_t count = 0;
            intfc->ReadRecordData(count);
            for (std::uint32_t i = 0; i < count && i < MAX_ENTRIES; ++i) {
                std::uint32_t size = 0;
                if (intfc->ReadRecordData(size) != sizeof(size) || size > MAX_PROFILE) {
                    break;
                }
                std::string profile(size, '\0');
                Entry entry;
                intfc->ReadRecordData(profile.data(), size);
                intfc->ReadRecordData(entry.health);
                intfc->ReadRecordData(entry.shield);
                loaded[profile] = entry;
            }
            for (const auto& [profile, entry] : loaded) {
                logger::info("HealthStore: '{}' health {:.0f}, radiation reserve {:.0f}", profile, entry.health, entry.shield);
            }
            const std::scoped_lock lock(_mutex);
            _entries = std::move(loaded);
            return true;
        }
        if (type == RECORD_RADS) {
            float applied = 0.0f;
            intfc->ReadRecordData(applied);
            if (applied != 0.0f) {
                RadProtection::get().undoOnLoad(applied);
            }
            return true;
        }
        if (type == RECORD_PERK) {
            std::uint8_t given = 0;
            intfc->ReadRecordData(given);
            _driverPerkGiven = given != 0;
            if (_driverPerkGiven) {
                logger::info("HealthStore: the driver's cover perk was given in this save");
            }
            return true;
        }
        return false;
    }

    void HealthStore::reset()
    {
        const std::scoped_lock lock(_mutex);
        _entries.clear();
        _driverPerkGiven = false;
    }
}
