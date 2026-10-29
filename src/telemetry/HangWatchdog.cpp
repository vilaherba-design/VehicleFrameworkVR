#include "telemetry/HangWatchdog.h"

#include <array>
#include <chrono>
#include <format>
#include <string>
#include <thread>

#include "Config.h"
#include "common/CommonUtils.h"

#ifdef _WIN32
#    define WIN32_LEAN_AND_MEAN
#    define NOMINMAX
#    include <windows.h>
#    include <tlhelp32.h>
#endif

namespace vfvr
{
    namespace
    {
        constexpr std::uint64_t CHECK_MS = 500;
        constexpr std::size_t MAX_FRAMES = 48;

        struct Stack
        {
            std::array<std::uint64_t, MAX_FRAMES> frames{};
            std::size_t count = 0;
            bool walked = false;
        };

        // Walk a suspended thread's stack by the modules' unwind data. Nothing here allocates or takes a lock the frozen thread may hold, and a
        // bad read ends the walk instead of crashing the game.
        void walkSuspended(const HANDLE thread, Stack& out)
        {
            CONTEXT context{};
            context.ContextFlags = CONTEXT_FULL;
            if (!GetThreadContext(thread, &context)) {
                return;
            }
            out.walked = true;
            __try {
                while (out.count < MAX_FRAMES && context.Rip != 0) {
                    out.frames[out.count++] = context.Rip;
                    DWORD64 imageBase = 0;
                    const auto* function = RtlLookupFunctionEntry(context.Rip, &imageBase, nullptr);
                    if (!function) {
                        // A leaf function: the return address is on top of the stack.
                        context.Rip = *reinterpret_cast<const DWORD64*>(context.Rsp);
                        context.Rsp += 8;
                        continue;
                    }
                    PVOID handlerData = nullptr;
                    DWORD64 establisherFrame = 0;
                    RtlVirtualUnwind(UNW_FLAG_NHANDLER, imageBase, context.Rip, const_cast<PRUNTIME_FUNCTION>(function), &context, &handlerData, &establisherFrame,
                        nullptr);
                }
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                // The walk ends at the frame that could not be read.
            }
        }

        Stack captureThread(const std::uint32_t threadId)
        {
            Stack stack;
            const HANDLE thread = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION, FALSE, threadId);
            if (!thread) {
                return stack;
            }
            if (SuspendThread(thread) != static_cast<DWORD>(-1)) {
                walkSuspended(thread, stack);
                ResumeThread(thread);
            }
            CloseHandle(thread);
            return stack;
        }

        // "Module.dll+0x1234" for an address (after the thread runs again: this looks modules up).
        std::string describeAddress(const std::uint64_t address)
        {
            HMODULE module = nullptr;
            if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(address), &module) ||
                !module) {
                return std::format("0x{:X}", address);
            }
            wchar_t path[MAX_PATH]{};
            const DWORD length = GetModuleFileNameW(module, path, MAX_PATH);
            std::wstring name(path, length);
            if (const auto slash = name.find_last_of(L"\\/"); slash != std::wstring::npos) {
                name = name.substr(slash + 1);
            }
            std::string narrow;
            for (const wchar_t c : name) {
                narrow.push_back(c < 128 ? static_cast<char>(c) : '?');
            }
            return std::format("{}+0x{:X}", narrow, address - reinterpret_cast<std::uint64_t>(module));
        }

        void logStack(const char* title, const Stack& stack)
        {
            if (!stack.walked) {
                logger::warn("HangWatchdog: {}: the stack could not be read", title);
                return;
            }
            std::string lines;
            for (std::size_t i = 0; i < stack.count; ++i) {
                lines += std::format("\n    #{:02} {}", i, describeAddress(stack.frames[i]));
            }
            logger::warn("HangWatchdog: {} ({} frames):{}", title, stack.count, lines);
        }

        // Does this stack pass through one of these modules' code?
        bool touches(const Stack& stack, const HMODULE module)
        {
            if (!module) {
                return false;
            }
            for (std::size_t i = 0; i < stack.count; ++i) {
                HMODULE owner = nullptr;
                if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(stack.frames[i]), &owner) &&
                    owner == module) {
                    return true;
                }
            }
            return false;
        }

        HMODULE thisModule()
        {
            HMODULE module = nullptr;
            GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(&thisModule), &module);
            return module;
        }
    }

    HangWatchdog& HangWatchdog::get()
    {
        static HangWatchdog instance;
        return instance;
    }

    HangStep::HangStep(const char* name)
        : _previous(HangWatchdog::get().currentStep())
    {
        HangWatchdog::get().step(name);
    }

    HangStep::~HangStep()
    {
        HangWatchdog::get().step(_previous);
    }

    void HangWatchdog::beat(const bool armed)
    {
        const auto now = common::nowMillis();
        if (_stallLogged.exchange(false)) {
            logger::warn("HangWatchdog: the game thread runs again after {:.1f} s", static_cast<double>(now - _stallStartMs.load()) * 0.001);
        }
        _beatMs.store(now, std::memory_order_relaxed);
        _armed.store(armed && g_config.hangWatchdog, std::memory_order_relaxed);
        _step.store("frame start", std::memory_order_relaxed);
        if (!_started) {
            _started = true;
            _gameThreadId.store(GetCurrentThreadId());
            std::thread([this] { run(); }).detach();
            logger::info("HangWatchdog: watching the game thread {} (armed while a menu panel is open or a vehicle is driven)", _gameThreadId.load());
        }
    }

    void HangWatchdog::run()
    {
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(CHECK_MS));
            const auto last = _beatMs.load(std::memory_order_relaxed);
            const auto now = common::nowMillis();
            const float limit = (std::max)(g_config.hangWatchdogSeconds, 1.0f);
            if (!_armed.load(std::memory_order_relaxed) || _stallLogged.load() || last == 0 || now < last ||
                static_cast<float>(now - last) * 0.001f < limit) {
                continue;
            }
            _stallStartMs.store(last);
            logStall(static_cast<double>(now - last) * 0.001);
            _stallLogged.store(true);
        }
    }

    void HangWatchdog::logStall(const double seconds)
    {
        const char* step = _step.load(std::memory_order_relaxed);
        logger::warn("HangWatchdog: no plugin frame for {:.1f} s while armed; the game thread was last in step '{}'", seconds, step ? step : "?");
        const auto gameThread = _gameThreadId.load();
        logStack(std::format("game thread {}", gameThread).c_str(), captureThread(gameThread));

        // Any other thread inside this plugin right now (a render hook, the physics step): its stack too.
        const HMODULE self = thisModule();
        const HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
        if (snapshot == INVALID_HANDLE_VALUE) {
            return;
        }
        THREADENTRY32 entry{};
        entry.dwSize = sizeof(entry);
        const DWORD process = GetCurrentProcessId();
        const DWORD watchdog = GetCurrentThreadId();
        int logged = 0;
        for (BOOL more = Thread32First(snapshot, &entry); more && logged < 8; more = Thread32Next(snapshot, &entry)) {
            if (entry.th32OwnerProcessID != process || entry.th32ThreadID == gameThread || entry.th32ThreadID == watchdog) {
                continue;
            }
            const auto stack = captureThread(entry.th32ThreadID);
            if (touches(stack, self)) {
                logStack(std::format("thread {} (in this plugin)", entry.th32ThreadID).c_str(), stack);
                ++logged;
            }
        }
        CloseHandle(snapshot);
    }
}
