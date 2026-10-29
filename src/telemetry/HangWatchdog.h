#pragma once

#include <atomic>
#include <cstdint>

namespace vfvr
{
    /**
     * Diagnostics for a frozen game. The game thread reports every frame of the plugin and the step of it that it is in (a static string); a
     * background thread looks every half second and, when no frame came for fHangWatchdogSeconds while the watchdog was armed (a menu panel
     * open or a vehicle driven: a loading screen then is not a freeze), logs the step and the game thread's call stack once per stall. The
     * stack is walked with the thread suspended, by the unwind data of each module (no allocation, no symbol files while it is suspended),
     * and logged as module + offset: the offsets of Fallout4VR.exe are the address library's.
     *
     * A frame that comes again after a stall is logged too, with how long it took: a long hitch is not a freeze.
     */
    class HangWatchdog
    {
    public:
        static HangWatchdog& get();

        // Every frame, first thing: the game thread is alive; `armed` while a stall would be a freeze. Starts the watchdog the first time. [game]
        void beat(bool armed);

        // The step of the frame the game thread is in now (a string literal). [game]
        void step(const char* name)
        {
            _step.store(name, std::memory_order_relaxed);
        }

        const char* currentStep() const
        {
            return _step.load(std::memory_order_relaxed);
        }

    private:
        void run();
        void logStall(double seconds);

        std::atomic<std::uint64_t> _beatMs{ 0 };
        std::atomic<bool> _armed{ false };
        std::atomic<const char*> _step{ "none" };
        std::atomic<std::uint32_t> _gameThreadId{ 0 };
        bool _started = false;
        std::atomic<bool> _stallLogged{ false }; // the current stall was logged (the game thread logs the end of it)
        std::atomic<std::uint64_t> _stallStartMs{ 0 };
    };

    // Marks the step of the frame for the watchdog while in scope, and puts the previous one back after it.
    class HangStep
    {
    public:
        explicit HangStep(const char* name);
        ~HangStep();
        HangStep(const HangStep&) = delete;
        HangStep& operator=(const HangStep&) = delete;

    private:
        const char* _previous;
    };
}
