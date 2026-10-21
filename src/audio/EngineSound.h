#pragma once

#include <atomic>
#include <mutex>
#include <thread>

namespace vfvr
{
    /**
     * What one engine sounds like. Plain numbers so the synthesis needs nothing from the vehicle model; the mod fills it from the profile's
     * "engine" section ("cylinders" and the "sound" block).
     */
    struct EngineVoice
    {
        int cylinders = 4; // sets the firing frequency: cylinders / 2 pulses per revolution
        float gain = 1.0f; // this vehicle's loudness, relative (0..2)
        float pitch = 1.0f; // scales the firing frequency (0.5..2): a deeper or a higher engine at the same rpm
        float oddDecay = 0.78f; // each odd harmonic relative to the one before it (0.3..0.95): lower is rounder, higher is buzzier
        float evenDecay = 0.62f; // the same for the even ones
        float rumble = 0.0f; // 0..1, the half-order component: the chug of a big slow engine
        float lumpiness = 0.2f; // 0..1, how uneven the firing is: a V8's burble, a single's thump
        float noise = 1.0f; // 0..2, intake and exhaust rush
        float brightness = 1.0f; // 0.3..2, low-pass cutoff scale
        float whine = 0.0f; // 0..1, supercharger or turbo whine that rises with rpm and load
        float whineOrder = 12.0f; // whine frequency = rpm / 60 * whineOrder
    };

    /**
     * Synthesised engine sound played straight through the Windows wave-out device: no game sound records, no assets, and the pitch follows the
     * engine rpm. Quiet by design (the player hears it for a whole drive). Not spatial: it is the driver's own engine.
     *
     * Threads: setState() may be called from the game thread every frame; the synthesis runs on its own thread, started by start().
     */
    class EngineSound
    {
    public:
        ~EngineSound();

        void start();
        void stop();

        // Called each frame. `running` false fades the sound out. `volume` is 0..1 (config); `voice` is the vehicle's own sound.
        void setState(bool running, float rpm, float throttle, float volume, const EngineVoice& voice);

    private:
        void run();

        std::atomic<bool> _quit{ false };
        std::atomic<bool> _running{ false };
        std::atomic<float> _rpm{ 0.0f };
        std::atomic<float> _throttle{ 0.0f };
        std::atomic<float> _volume{ 0.5f };
        std::mutex _voiceMutex;
        EngineVoice _voice;
        std::thread _thread;
    };

    // One block of the synthesis, exposed for tests (no device involved). `state` carries the oscillators between calls.
    struct EngineSynthState
    {
        float phase = 0.0f; // firing phase, 0..1
        float subPhase = 0.0f; // half the firing frequency, 0..1
        float cycPhase = 0.0f; // engine cycle (two revolutions), 0..1
        float whinePhase = 0.0f;
        float gain = 0.0f;
        float rpm = 900.0f;
        float load = 0.0f;
        float lp = 0.0f;
        float noiseLp = 0.0f;
        unsigned rng = 12345u;
        float jitter = 1.0f;
        float jitterTarget = 1.0f;
    };
    void synthEngineBlock(EngineSynthState& st, bool running, float rpm, float throttle, float volume, const EngineVoice& voice, float sampleRate, short* out, int count);
} // namespace vfvr
