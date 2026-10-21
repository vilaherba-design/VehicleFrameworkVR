#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "model/TrunkLoad.h"

namespace vfvr
{
    /**
     * Vehicle trunks. Each vehicle profile has one container reference of its own (the plugin's "VFVR_Trunk" base, no model), created persistent the
     * first time that trunk is opened and kept by the save game, so what is in it survives the vehicle being spawned again. The profile -> reference
     * FormID map lives in the F4SE co-save.
     *
     * Opening activates the container from wherever it is, which shows the game's own container menu. Fallout 4 puts no weight limit on containers,
     * so the capacity is applied when the menu closes: the latest items put in go back to the player until the trunk is within its capacity.
     * The items put in and taken out are followed with the container changed event.
     */
    class Trunk : public RE::BSTEventSink<RE::TESContainerChangedEvent>
    {
    public:
        static Trunk& get();

        // F4SE co-save callbacks; call while the plugin loads. [game]
        void registerSerialization() const;
        // The container changed event; call once the game data is loaded. [game]
        void registerEvents();

        // Open the trunk of `profileId`. `hiddenSpot` is where a new container is placed (out of sight, near the vehicle). Shows a notification and
        // returns false when it cannot. [game]
        bool open(const std::string& profileId, const std::string& displayName, float capacity, const RE::NiPoint3& hiddenSpot);

        // A trunk menu is being opened or is open.
        bool isBusy() const;

        // Follows the menu: once it closes, applies the capacity. [game]
        void onFrameUpdate();

        // [any] the engine's thread that moves the items.
        RE::BSEventNotifyControl ProcessEvent(const RE::TESContainerChangedEvent& event, RE::BSTEventSource<RE::TESContainerChangedEvent>* source) override;

    private:
        enum class State : std::uint8_t
        {
            Idle,
            Opening, // activated, waiting for the container menu to show
            Open,
        };

        RE::TESObjectREFR* findContainer(const std::string& profileId);
        RE::TESObjectREFR* createContainer(const std::string& profileId, const RE::NiPoint3& spot);
        void applyCapacity();

        static void onSave(const F4SE::SerializationInterface* intfc);
        static void onLoad(const F4SE::SerializationInterface* intfc);
        static void onRevert(const F4SE::SerializationInterface* intfc);

        mutable std::mutex _mutex;
        std::map<std::string, std::uint32_t> _containers; // profile id -> container reference FormID
        State _state = State::Idle;
        std::uint32_t _openId = 0;
        std::string _openProfile;
        float _capacity = 0.0f;
        int _waitFrames = 0;
        std::vector<model::TrunkAddition> _added;
        bool _eventsRegistered = false;
    };
}
