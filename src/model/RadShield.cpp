#include "model/RadShield.h"

#include <algorithm>
#include <cmath>

namespace vfvr::model
{
    namespace
    {
        constexpr float CURVE_EXPONENT = 0.365f;

        float clampReserve(const float remaining, const float capacity)
        {
            return std::clamp(remaining, 0.0f, (std::max)(capacity, 0.0f));
        }
    }

    RadShield::RadShield(const RadiationSpec& spec)
        : _spec(spec)
        , _remaining((std::max)(spec.capacity, 0.0f))
    {}

    float RadShield::share() const
    {
        return _spec.capacity > 0.0f ? std::clamp(_remaining / _spec.capacity, 0.0f, 1.0f) : 0.0f;
    }

    float RadShield::receivedShare(const float ambient, const float resistance)
    {
        if (resistance <= 0.0f) {
            return 1.0f;
        }
        if (ambient <= 0.0f) {
            return 0.0f;
        }
        if (resistance >= ambient) {
            return 0.5f * std::pow(ambient / resistance, CURVE_EXPONENT);
        }
        return 1.0f - 0.5f * std::pow(resistance / ambient, CURVE_EXPONENT);
    }

    float RadShield::ambientFromReceived(const float received, const float resistance)
    {
        if (received <= 0.0f) {
            return 0.0f;
        }
        if (resistance <= 0.0f) {
            return received;
        }
        // R >= r branch: received = 0.5 * r^1.365 / R^0.365  ->  r = (2 * received * R^0.365)^(1 / 1.365)
        const float r = std::pow(2.0f * received * std::pow(resistance, CURVE_EXPONENT), 1.0f / (1.0f + CURVE_EXPONENT));
        if (r <= resistance) {
            return r;
        }
        // R < r branch: received = r * (1 - 0.5 * (R / r)^0.365), rising with r: bisection between R and where all of it gets through.
        float low = resistance;
        float high = (std::max)(received * 2.0f, resistance * 2.0f);
        for (int i = 0; i < 40; ++i) {
            const float mid = 0.5f * (low + high);
            const float got = mid * receivedShare(mid, resistance);
            if (got < received) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return 0.5f * (low + high);
    }

    float RadShield::tick(const float ambient, const float dt)
    {
        const float blocked = active() && ambient > 0.0f ? ambient * efficacy(ambient, _spec.resistance) : 0.0f;
        return drain(blocked, dt);
    }

    float RadShield::drain(const float blocked, const float dt)
    {
        if (dt <= 0.0f || _spec.capacity <= 0.0f) {
            return 0.0f;
        }
        float used = 0.0f;
        if (active() && blocked > 0.0f && blocked >= _spec.drainThreshold) {
            used = (std::min)(blocked * (std::max)(_spec.drainPerRad, 0.0f) * dt, _remaining);
        }
        _remaining = clampReserve(_remaining - used + (std::max)(_spec.recoverPerSecond, 0.0f) * dt, _spec.capacity);
        return used;
    }

    void RadShield::repair(const float share)
    {
        _remaining = clampReserve(_remaining + (std::max)(share, 0.0f) * _spec.capacity, _spec.capacity);
    }

    void RadShield::setRemaining(const float remaining)
    {
        _remaining = clampReserve(remaining, _spec.capacity);
    }
}
