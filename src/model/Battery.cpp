#include "model/Battery.h"

#include <algorithm>

namespace vfvr::model
{
    Battery::Battery(const BatterySpec& spec)
        : _spec(spec)
    {}

    float Battery::drainPerSecond(const BatterySpec& spec, const float throttle)
    {
        const float idle = 1.0f / ((std::max)(spec.minutesIdle, 0.01f) * 60.0f);
        const float full = 1.0f / ((std::max)(spec.minutesFullThrottle, 0.01f) * 60.0f);
        const float t = std::clamp(throttle, 0.0f, 1.0f);
        return idle + (full - idle) * t;
    }

    bool Battery::drain(const float seconds, const float throttle, const float scale)
    {
        if (!_spec.enabled || _charge <= 0.0f || seconds <= 0.0f || scale <= 0.0f) {
            return false;
        }
        _charge = (std::max)(0.0f, _charge - drainPerSecond(_spec, throttle) * seconds * scale);
        return _charge <= 0.0f;
    }

    bool Battery::recharge(const float seconds, const float scale)
    {
        if (!_spec.enabled || _charge >= 1.0f || seconds <= 0.0f || scale <= 0.0f) {
            return false;
        }
        _charge = (std::min)(1.0f, _charge + seconds * scale / ((std::max)(_spec.minutesToCharge, 0.01f) * 60.0f));
        return _charge >= 1.0f;
    }

    void Battery::addShare(const float share)
    {
        _charge = std::clamp(_charge + (std::max)(share, 0.0f), 0.0f, 1.0f);
    }

    float Battery::coreCharge(const bool hasHealth, const float health)
    {
        if (!hasHealth) {
            return 1.0f;
        }
        return std::clamp(health > 1.0f ? health / 100.0f : health, 0.0f, 1.0f);
    }

    void Battery::restore(const float charge)
    {
        _charge = std::clamp(charge, 0.0f, 1.0f);
    }
}
