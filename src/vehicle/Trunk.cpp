#include "vehicle/Trunk.h"

#include "health/HealthStore.h"

#include <format>

#include "f4vr/F4VRUtils.h"

namespace vfvr
{
    namespace
    {
        constexpr auto PLUGIN_FILE = "VehicleFrameworkVR_Vehicles.esp";
        constexpr RE::TESFormID TRUNK_BASE_ID = 0x805; // VFVR_Trunk (CONT), see tools/make_vehicle_esp.py
        constexpr std::uint32_t SERIALIZATION_ID = 0x56465652; // 'VFVR'
        constexpr std::uint32_t RECORD_TRUNKS = 0x54524E4B; // 'TRNK'
        constexpr std::uint32_t RECORD_VERSION = 1;
        constexpr int MENU_WAIT_FRAMES = 90; // ~1 s for the container menu to show after the activation
        constexpr int MAX_RETURN_PASSES = 4;

        bool containerMenuOpen()
        {
            const auto* ui = RE::UI::GetSingleton();
            return ui && ui->GetMenuOpen("ContainerMenu");
        }

        float unitWeightOf(const std::uint32_t formId)
        {
            const auto* form = RE::TESForm::GetFormByID(formId);
            return form ? RE::TESWeightForm::GetFormWeight(form, nullptr) : 0.0f;
        }
    }

    Trunk& Trunk::get()
    {
        static Trunk instance;
        return instance;
    }

    void Trunk::registerSerialization() const
    {
        const auto* serialization = F4SE::GetSerializationInterface();
        if (!serialization) {
            logger::error("Trunk: no F4SE serialization interface, trunks will not be remembered across saves");
            return;
        }
        serialization->SetUniqueID(SERIALIZATION_ID);
        serialization->SetSaveCallback(&Trunk::onSave);
        serialization->SetLoadCallback(&Trunk::onLoad);
        serialization->SetRevertCallback(&Trunk::onRevert);
    }

    void Trunk::registerEvents()
    {
        if (_eventsRegistered) {
            return;
        }
        if (auto* source = RE::TESContainerChangedEvent::GetEventSource()) {
            source->RegisterSink(this);
            _eventsRegistered = true;
        } else {
            logger::error("Trunk: no container changed event source, the trunk capacity cannot be applied");
        }
    }

    bool Trunk::isBusy() const
    {
        const std::scoped_lock lock(_mutex);
        return _state != State::Idle;
    }

    RE::TESObjectREFR* Trunk::findContainer(const std::string& profileId)
    {
        std::uint32_t id = 0;
        {
            const std::scoped_lock lock(_mutex);
            if (const auto it = _containers.find(profileId); it != _containers.end()) {
                id = it->second;
            }
        }
        if (id == 0) {
            return nullptr;
        }
        auto* ref = RE::TESForm::GetFormByID<RE::TESObjectREFR>(id);
        if (!ref || ref->IsDeleted() || !ref->data.objectReference || ref->data.objectReference->GetFormType() != RE::ENUM_FORM_ID::kCONT) {
            logger::warn("Trunk: the container {:08X} of '{}' is gone, a new (empty) one is made", id, profileId);
            return nullptr;
        }
        return ref;
    }

    RE::TESObjectREFR* Trunk::createContainer(const std::string& profileId, const RE::NiPoint3& spot)
    {
        auto* handler = RE::TESDataHandler::GetSingleton();
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* cell = player ? player->GetParentCell() : nullptr;
        auto* form = handler ? handler->LookupForm(TRUNK_BASE_ID, PLUGIN_FILE) : nullptr;
        auto* base = form ? form->As<RE::TESBoundObject>() : nullptr;
        if (!cell || !base) {
            logger::error("Trunk: {} (VFVR_Trunk in {})", base ? "the player has no cell" : "the trunk base object is missing", PLUGIN_FILE);
            return nullptr;
        }
        RE::NEW_REFR_DATA data;
        data.location = spot;
        data.direction = RE::NiPoint3(0, 0, 0);
        data.object = base;
        data.interior = cell->IsInterior() ? cell : nullptr;
        data.world = cell->IsExterior() ? cell->worldSpace : nullptr;
        data.forcePersist = true; // kept by the save game wherever the player goes
        data.clearStillLoadingFlag = true;
        const auto handle = handler->CreateReferenceAtLocation(data);
        auto* ref = handle.get().get();
        if (!ref) {
            logger::error("Trunk: could not create the container of '{}'", profileId);
            return nullptr;
        }
        {
            const std::scoped_lock lock(_mutex);
            _containers[profileId] = ref->GetFormID();
        }
        logger::info("Trunk: created container {:08X} for '{}'", ref->GetFormID(), profileId);
        return ref;
    }

    bool Trunk::open(const std::string& profileId, const std::string& displayName, const float capacity, const RE::NiPoint3& hiddenSpot)
    {
        if (isBusy()) {
            return false;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* ref = findContainer(profileId);
        if (!ref) {
            ref = createContainer(profileId, hiddenSpot);
        }
        if (!player || !ref) {
            f4vr::showNotification("The trunk could not be opened (see log)");
            return false;
        }
        const float weight = ref->GetWeightInContainer();
        {
            const std::scoped_lock lock(_mutex);
            _state = State::Opening;
            _openId = ref->GetFormID();
            _openProfile = profileId;
            _capacity = capacity;
            _waitFrames = 0;
            _added.clear();
        }
        // From a script: the container is activated wherever it is, like Papyrus' Activate.
        const bool activated = ref->ActivateRef(player, nullptr, 1, true, true, false);
        logger::info("Trunk: opening '{}' ({}), container {:08X}, weight {:.1f} / {:.0f}{}", profileId, displayName, _openId, weight, capacity,
            activated ? "" : " - the activation was refused");
        if (!activated) {
            const std::scoped_lock lock(_mutex);
            _state = State::Idle;
            _openId = 0;
            f4vr::showNotification("The trunk could not be opened (see log)");
            return false;
        }
        f4vr::showNotification(std::format("{} trunk: {:.0f} / {:.0f}", displayName, weight, capacity));
        return true;
    }

    void Trunk::onFrameUpdate()
    {
        State state;
        {
            const std::scoped_lock lock(_mutex);
            state = _state;
        }
        if (state == State::Opening) {
            if (containerMenuOpen()) {
                const std::scoped_lock lock(_mutex);
                _state = State::Open;
            } else if (++_waitFrames > MENU_WAIT_FRAMES) {
                logger::error("Trunk: the container menu did not open for {:08X}", _openId);
                f4vr::showNotification("The trunk could not be opened (see log)");
                const std::scoped_lock lock(_mutex);
                _state = State::Idle;
                _openId = 0;
            }
        } else if (state == State::Open && !containerMenuOpen()) {
            applyCapacity();
        }
    }

    void Trunk::applyCapacity()
    {
        std::uint32_t id;
        float capacity;
        std::string profile;
        std::vector<model::TrunkAddition> added;
        {
            const std::scoped_lock lock(_mutex);
            id = _openId;
            capacity = _capacity;
            profile = _openProfile;
            added = std::move(_added);
            _added.clear();
            _state = State::Idle;
            _openId = 0;
        }
        auto* ref = RE::TESForm::GetFormByID<RE::TESObjectREFR>(id);
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!ref || !player) {
            return;
        }
        float weight = ref->GetWeightInContainer();
        int returned = 0;
        for (int pass = 0; pass < MAX_RETURN_PASSES && weight > capacity + 1e-3f; ++pass) {
            const auto returns = model::trunkReturns(added, weight, capacity);
            if (returns.empty()) {
                break;
            }
            for (const auto& r : returns) {
                auto* object = RE::TESForm::GetFormByID<RE::TESBoundObject>(r.formId);
                if (!object) {
                    continue;
                }
                RE::TESObjectREFR::RemoveItemData data(object, r.count);
                data.reason = RE::ITEM_REMOVE_REASON::kStoreContainer;
                data.a_otherContainer = player;
                ref->RemoveItem(data);
                returned += r.count;
                model::recordTrunkChange(added, r.formId, -r.count, 0.0f);
            }
            weight = ref->GetWeightInContainer();
        }
        logger::info("Trunk: closed '{}', container {:08X}, weight {:.1f} / {:.0f}{}", profile, id, weight, capacity,
            returned > 0 ? std::format(", {} item(s) given back", returned) : "");
        if (returned > 0) {
            f4vr::showNotification(std::format("Trunk full ({:.0f} / {:.0f}): {} item(s) given back", weight, capacity, returned));
        } else if (weight > capacity + 1e-3f) {
            f4vr::showNotification(std::format("The trunk is over its capacity ({:.0f} / {:.0f})", weight, capacity));
        }
    }

    RE::BSEventNotifyControl Trunk::ProcessEvent(const RE::TESContainerChangedEvent& event, RE::BSTEventSource<RE::TESContainerChangedEvent>*)
    {
        std::uint32_t id;
        {
            const std::scoped_lock lock(_mutex);
            id = _state == State::Idle ? 0 : _openId;
        }
        if (id == 0 || event.itemCount <= 0 || (event.newContainerFormID != id && event.oldContainerFormID != id)) {
            return RE::BSEventNotifyControl::kContinue;
        }
        const bool in = event.newContainerFormID == id;
        const float unitWeight = in ? unitWeightOf(event.baseObjectFormID) : 0.0f;
        const std::scoped_lock lock(_mutex);
        model::recordTrunkChange(_added, event.baseObjectFormID, in ? event.itemCount : -event.itemCount, unitWeight);
        return RE::BSEventNotifyControl::kContinue;
    }

    void Trunk::onSave(const F4SE::SerializationInterface* intfc)
    {
        auto& self = get();
        const std::scoped_lock lock(self._mutex);
        if (!intfc->OpenRecord(RECORD_TRUNKS, RECORD_VERSION)) {
            logger::error("Trunk: could not write the co-save record");
            return;
        }
        const auto count = static_cast<std::uint32_t>(self._containers.size());
        intfc->WriteRecordData(count);
        for (const auto& [profile, id] : self._containers) {
            const auto length = static_cast<std::uint32_t>(profile.size());
            intfc->WriteRecordData(length);
            intfc->WriteRecordData(profile.data(), length);
            intfc->WriteRecordData(id);
        }
        // F4SE gives a plugin one set of callbacks: the health system's records go in the same co-save.
        HealthStore::get().write(intfc);
    }

    void Trunk::onLoad(const F4SE::SerializationInterface* intfc)
    {
        auto& self = get();
        std::map<std::string, std::uint32_t> loaded;
        std::uint32_t type = 0;
        std::uint32_t version = 0;
        std::uint32_t length = 0;
        while (intfc->GetNextRecordInfo(type, version, length)) {
            if (HealthStore::get().read(intfc, type, version, length)) {
                continue;
            }
            if (type != RECORD_TRUNKS || version != RECORD_VERSION) {
                continue;
            }
            std::uint32_t count = 0;
            intfc->ReadRecordData(count);
            for (std::uint32_t i = 0; i < count && i < 256; ++i) {
                std::uint32_t size = 0;
                if (intfc->ReadRecordData(size) != sizeof(size) || size > 256) {
                    break;
                }
                std::string profile(size, '\0');
                std::uint32_t id = 0;
                intfc->ReadRecordData(profile.data(), size);
                intfc->ReadRecordData(id);
                if (const auto resolved = intfc->ResolveFormID(id)) {
                    loaded[profile] = *resolved;
                } else {
                    logger::warn("Trunk: the container {:08X} of '{}' is no longer in the save", id, profile);
                }
            }
        }
        for (const auto& [profile, id] : loaded) {
            logger::info("Trunk: '{}' -> container {:08X}", profile, id);
        }
        const std::scoped_lock lock(self._mutex);
        self._containers = std::move(loaded);
        self._state = State::Idle;
        self._openId = 0;
        self._added.clear();
    }

    void Trunk::onRevert(const F4SE::SerializationInterface*)
    {
        HealthStore::get().reset();
        auto& self = get();
        const std::scoped_lock lock(self._mutex);
        self._containers.clear();
        self._state = State::Idle;
        self._openId = 0;
        self._added.clear();
    }
}
