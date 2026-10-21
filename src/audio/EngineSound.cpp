#include "audio/EngineSound.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#ifdef _WIN32
#    define WIN32_LEAN_AND_MEAN
#    define NOMINMAX
#    include <windows.h>
#    include <mmsystem.h>
#    pragma comment(lib, "winmm.lib")
#endif

namespace vfvr
{
    namespace
    {
        constexpr float PI2 = 6.28318530718f;
        constexpr int RATE = 22050;
        constexpr int BLOCK = 1024;
        constexpr int BUFFERS = 4;

        float noise(unsigned& s)
        {
            s = s * 1664525u + 1013904223u;
            return static_cast<float>(static_cast<int>(s >> 8) & 0xFFFF) / 32768.0f - 1.0f;
        }
    } // namespace

    void synthEngineBlock(EngineSynthState& st, const bool running, const float rpm, const float throttle, const float volume, const EngineVoice& voice, const float sampleRate, short* out, const int count)
    {
        // Overall level: kept low on purpose (the engine is heard for a whole drive; the first version was a bit loud).
        constexpr float MASTER = 0.62f;
        const float targetGain = running ? std::clamp(volume, 0.0f, 1.0f) * std::clamp(voice.gain, 0.0f, 2.0f) : 0.0f;
        const float cyl = static_cast<float>(std::clamp(voice.cylinders, 1, 12));
        const float pitch = std::clamp(voice.pitch, 0.5f, 2.0f);
        const float oddDecay = std::clamp(voice.oddDecay, 0.3f, 0.95f);
        const float evenDecay = std::clamp(voice.evenDecay, 0.3f, 0.95f);
        const float lump = std::clamp(voice.lumpiness, 0.0f, 1.0f);
        const float rumble = std::clamp(voice.rumble, 0.0f, 1.0f);
        const float noiseAmount = std::clamp(voice.noise, 0.0f, 2.0f);
        const float brightness = std::clamp(voice.brightness, 0.3f, 2.0f);
        const float whine = std::clamp(voice.whine, 0.0f, 1.0f);
        const float whineOrder = std::clamp(voice.whineOrder, 1.0f, 60.0f);
        for (int i = 0; i < count; ++i) {
            // Smooth the controls: rpm and load move over ~80 ms, the level fades in/out over ~0.4 s.
            st.rpm += (std::clamp(rpm, 500.0f, 8000.0f) - st.rpm) * (1.0f / (0.08f * sampleRate));
            st.load += (std::clamp(throttle, 0.0f, 1.0f) - st.load) * (1.0f / (0.12f * sampleRate));
            st.gain += (targetGain - st.gain) * (1.0f / (0.4f * sampleRate));
            // Firing frequency of a four-stroke: cylinders / 2 pulses per revolution.
            const float f0 = st.rpm / 60.0f * cyl * 0.5f * pitch;
            st.phase += f0 / sampleRate;
            st.subPhase += 0.5f * f0 / sampleRate;
            st.cycPhase += st.rpm / 120.0f / sampleRate;
            st.whinePhase += st.rpm / 60.0f * whineOrder / sampleRate;
            if (st.subPhase >= 1.0f) {
                st.subPhase -= 1.0f;
            }
            if (st.cycPhase >= 1.0f) {
                st.cycPhase -= 1.0f;
            }
            if (st.whinePhase >= 1.0f) {
                st.whinePhase -= std::floor(st.whinePhase);
            }
            if (st.phase >= 1.0f) {
                st.phase -= 1.0f;
                // Firing-to-firing unevenness: what makes an idle sound like an engine, and a V8 burble or a single thump.
                st.jitterTarget = 1.0f + (0.08f + 0.45f * lump) * noise(st.rng);
            }
            st.jitter += (st.jitterTarget - st.jitter) * 0.01f;
            // One heavy firing per engine cycle on top of that (the cycle is two revolutions): the slow swell of big, lumpy engines.
            const float cycleSwell = 1.0f + 0.35f * lump * std::sin(st.cycPhase * PI2);
            const float p = st.phase * PI2;
            // Harmonics of the firing frequency, steeply decaying; the odd ones and the even ones decay at their own rates (the voice's character).
            float s = 0.0f;
            float a = 1.0f;
            float sumSq = 0.0f;
            const int maxK = std::min(14, static_cast<int>(7000.0f / std::max(f0, 1.0f)));
            for (int k = 1; k <= maxK; ++k) {
                s += a * std::sin(p * static_cast<float>(k) + 0.7f * static_cast<float>(k * k % 5));
                sumSq += a * a;
                a *= (k % 2) ? oddDecay : evenDecay;
            }
            // Same loudness whatever the decay rates are: the voice changes the colour, the gain sets the level.
            s *= 0.5f / std::sqrt(std::max(sumSq, 0.1f)) * st.jitter * cycleSwell;
            // Half-order chug.
            s += rumble * 0.45f * std::sin(st.subPhase * PI2) * (0.6f + 0.4f * st.load);
            // Intake/exhaust rush: low-passed noise that grows with load.
            st.noiseLp += (noise(st.rng) - st.noiseLp) * 0.12f;
            s += st.noiseLp * (0.05f + 0.25f * st.load) * noiseAmount;
            // Overall tone: rounder at idle, brighter under load and at high rpm.
            const float cutoff = std::clamp((0.05f + 0.00004f * st.rpm + 0.10f * st.load) * brightness, 0.03f, 0.55f);
            st.lp += (s - st.lp) * cutoff;
            const float level = st.gain * MASTER * (0.10f + 0.10f * st.load + 0.00002f * st.rpm);
            // Whine sits above the low-pass: it is meant to be a thin, high note under the engine.
            const float w = whine * (0.02f + 0.10f * st.load) * std::sin(st.whinePhase * PI2) * st.gain * MASTER;
            const float v = std::clamp(st.lp * level + w, -1.0f, 1.0f);
            out[i] = static_cast<short>(v * 30000.0f);
        }
    }

    EngineSound::~EngineSound()
    {
        stop();
    }

    void EngineSound::start()
    {
#ifdef _WIN32
        if (_thread.joinable()) {
            return;
        }
        _quit = false;
        _thread = std::thread([this] { run(); });
#endif
    }

    void EngineSound::stop()
    {
        if (_thread.joinable()) {
            _quit = true;
            _thread.join();
        }
    }

    void EngineSound::setState(const bool running, const float rpm, const float throttle, const float volume, const EngineVoice& voice)
    {
        _running = running;
        _rpm = rpm;
        _throttle = throttle;
        _volume = volume;
        std::scoped_lock lock(_voiceMutex);
        _voice = voice;
    }

    void EngineSound::run()
    {
#ifdef _WIN32
        WAVEFORMATEX fmt{};
        fmt.wFormatTag = WAVE_FORMAT_PCM;
        fmt.nChannels = 1;
        fmt.nSamplesPerSec = RATE;
        fmt.wBitsPerSample = 16;
        fmt.nBlockAlign = 2;
        fmt.nAvgBytesPerSec = RATE * 2;
        HWAVEOUT device = nullptr;
        if (waveOutOpen(&device, WAVE_MAPPER, &fmt, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) {
            return;
        }
        static short pcm[BUFFERS][BLOCK];
        WAVEHDR hdr[BUFFERS]{};
        EngineSynthState st;
        const EngineVoice silent;
        for (int b = 0; b < BUFFERS; ++b) {
            hdr[b].lpData = reinterpret_cast<LPSTR>(pcm[b]);
            hdr[b].dwBufferLength = BLOCK * 2;
            synthEngineBlock(st, false, 900.0f, 0.0f, 0.0f, silent, static_cast<float>(RATE), pcm[b], BLOCK);
            waveOutPrepareHeader(device, &hdr[b], sizeof(WAVEHDR));
            waveOutWrite(device, &hdr[b], sizeof(WAVEHDR));
        }
        while (!_quit) {
            bool any = false;
            for (int b = 0; b < BUFFERS; ++b) {
                if (hdr[b].dwFlags & WHDR_DONE) {
                    EngineVoice voice;
                    {
                        std::scoped_lock lock(_voiceMutex);
                        voice = _voice;
                    }
                    synthEngineBlock(st, _running, _rpm, _throttle, _volume, voice, static_cast<float>(RATE), pcm[b], BLOCK);
                    hdr[b].dwFlags &= ~static_cast<DWORD>(WHDR_DONE);
                    waveOutWrite(device, &hdr[b], sizeof(WAVEHDR));
                    any = true;
                }
            }
            if (!any) {
                std::this_thread::sleep_for(std::chrono::milliseconds(8));
            }
        }
        waveOutReset(device);
        for (auto& h : hdr) {
            waveOutUnprepareHeader(device, &h, sizeof(WAVEHDR));
        }
        waveOutClose(device);
#endif
    }
} // namespace vfvr
