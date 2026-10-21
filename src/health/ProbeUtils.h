#pragma once

#include <cstdint>
#include <format>
#include <string>
#include <utility>

#include "physics/HavokBridge.h"

namespace vfvr::probe
{
    // "Name (0x0000ABCD, type N)" of a form; the name is its full name, empty for forms without one. [any]
    inline std::string describeForm(const RE::TESForm* form)
    {
        if (!form) {
            return "none";
        }
        const auto name = RE::TESFullName::GetFullName(*form);
        return std::format("{}{}(0x{:08X}, type {})", name, name.empty() ? "" : " ", form->GetFormID(), std::to_underlying(form->GetFormType()));
    }

    inline std::string describeFormId(const std::uint32_t formId)
    {
        if (formId == 0) {
            return "none";
        }
        const auto* form = RE::TESForm::GetFormByID(formId);
        return form ? describeForm(form) : std::format("unknown (0x{:08X})", formId);
    }

    // A reference by the name shown in game (its base object's name when the display name cannot be read). [any]
    inline std::string describeRef(RE::TESObjectREFR* ref)
    {
        if (!ref) {
            return "none";
        }
        static const bool hasDisplayName = physics::hasAddressId(1212056); // TESObjectREFR::GetDisplayFullName
        const char* name = hasDisplayName ? ref->GetDisplayFullName() : nullptr;
        std::string label = name ? name : "";
        if (label.empty()) {
            if (const auto* base = ref->GetObjectReference()) {
                label = std::string(RE::TESFullName::GetFullName(*base));
            }
        }
        return std::format("{}{}(0x{:08X})", label, label.empty() ? "" : " ", ref->GetFormID());
    }

    // The level of an actor reference, -1 for anything else. [any]
    inline int levelOf(RE::TESObjectREFR* ref)
    {
        if (!ref || ref->GetFormType() != RE::ENUM_FORM_ID::kACHR) {
            return -1;
        }
        static const bool hasLevel = physics::hasAddressId(661617); // Actor::GetLevel
        return hasLevel ? static_cast<int>(static_cast<RE::Actor*>(ref)->GetLevel()) : -1;
    }
}
