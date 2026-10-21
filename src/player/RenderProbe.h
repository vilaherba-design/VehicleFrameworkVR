#pragma once

#include <cstdint>
#include <atomic>
#include <chrono>
#include <format>
#include <string>

#include "model/VMath.h"
#include "render/SubmitHook.h"
#include "telemetry/CsvLog.h"

/**
     * Diagnostic, off by default (bRenderProbe): for a few hundred frames, writes one CSV row per frame the game submits to the headset with the
     * camera constants the engine rendered that frame with, next to the seat's own frame counters. Comparing those rows with the seat placements
     * (camera CSV) shows which placement the picture really used, and so how far the camera is from the cabin at render time.
     * Draw callbacks make the framework copy the scene depth every frame, so the probe records for a short window only.
     */
namespace vfvr::render_probe
{
    namespace detail
    {
        inline constexpr std::size_t MAX_ROWS = 1500; // ~25 s at 60 Hz

        inline std::atomic<std::uint64_t> g_beginFrame{ 0 };
        inline std::atomic<std::uint64_t> g_endFrame{ 0 };
        inline std::atomic<bool> g_recording{ false };
        inline std::atomic<std::uint64_t> g_submits{ 0 };
        // The model root: what we last wrote to it, and the node itself, read again when the frame is submitted (a second look at the same
        // transform after the engine has had the rest of the frame; reading it from the submit hook is safe in practice, it runs on the game thread).
        inline std::atomic<const RE::NiAVObject*> g_visual{ nullptr };
        inline std::atomic<int> g_calls{ 0 };
        inline std::atomic<int> g_noRoot{ 0 };
        inline std::atomic<float> g_writtenX{ 0.0f };
        inline std::atomic<float> g_writtenY{ 0.0f };
        inline std::atomic<float> g_writtenZ{ 0.0f };

        inline CsvLog g_log;
        inline bool g_logOpen = false;
        inline f4cf::render::DrawCallbackId g_callback = f4cf::render::INVALID_DRAW_CALLBACK;
        inline bool g_callbackActive = false;
        inline bool g_armed = true; // re-armed when the switch goes off, so one switch-on records one window
        inline std::chrono::steady_clock::time_point g_start{};
        inline std::string g_dir;

        inline void onSubmit(const f4cf::render::SubmitFrame& frame)
        {
            if (!g_recording.load(std::memory_order_relaxed) || !frame.camera) {
                return;
            }
            const auto n = g_submits.fetch_add(1, std::memory_order_relaxed);
            if (n >= MAX_ROWS) {
                return;
            }
            const auto& c = *frame.camera;
            const float ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - g_start).count();
            // Camera offsets in meters, both eyes, plus the translation row of eye 0's matrix (it carries the pose the engine rendered with).
            const auto m = [](const float v) { return v * model::GAME_TO_HAVOK; };
            float vx = 0.0f;
            float vy = 0.0f;
            float vz = 0.0f;
            if (const auto* node = g_visual.load(std::memory_order_relaxed)) {
                vx = node->world.translate.x;
                vy = node->world.translate.y;
                vz = node->world.translate.z;
            }
            g_log.add(std::format("{},{:.3f},{},{},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f},{:.5f}",
                n,
                ms,
                g_beginFrame.load(std::memory_order_relaxed),
                g_endFrame.load(std::memory_order_relaxed),
                m(c.posAdjust[0].x),
                m(c.posAdjust[0].y),
                m(c.posAdjust[0].z),
                m(c.posAdjust[1].x),
                m(c.posAdjust[1].y),
                m(c.posAdjust[1].z),
                c.viewProj[0].r[3].m128_f32[0],
                c.viewProj[0].r[3].m128_f32[1],
                c.viewProj[0].r[3].m128_f32[2],
                c.viewProj[0].r[3].m128_f32[3],
                m(vx),
                m(vy),
                m(vz),
                m(g_writtenX.load(std::memory_order_relaxed)),
                m(g_writtenY.load(std::memory_order_relaxed)),
                m(g_writtenZ.load(std::memory_order_relaxed))));
        }
    }

    using namespace detail;

    // The model root was just written at this world position (game units) (game thread).
    inline void noteWritten(const RE::NiAVObject* root, const float x, const float y, const float z)
    {
        g_visual.store(root, std::memory_order_relaxed);
        g_writtenX.store(x, std::memory_order_relaxed);
        g_writtenY.store(y, std::memory_order_relaxed);
        g_writtenZ.store(z, std::memory_order_relaxed);
    }

    // placeVisualAt was entered; hasRoot says whether there was a model root to write (game thread).
    inline void noteCall(const bool hasRoot, const RE::NiAVObject* carrierRoot)
    {
        if (!hasRoot) {
            g_visual.store(carrierRoot, std::memory_order_relaxed); // no separate model: the reference's own 3D is what gets drawn
        }
        g_calls.fetch_add(1, std::memory_order_relaxed);
        if (!hasRoot) {
            g_noRoot.fetch_add(1, std::memory_order_relaxed);
        }
    }

    inline void setDirectory(const std::string& dir)
    {
        g_dir = dir;
    }

    inline void noteFrameBegin(const std::uint64_t seatFrame)
    {
        g_beginFrame.store(seatFrame, std::memory_order_relaxed);
    }

    inline void noteFrameEnd(const std::uint64_t seatFrame)
    {
        g_endFrame.store(seatFrame, std::memory_order_relaxed);
    }

    inline void tick(const bool wanted)
    {
        if (!wanted) {
            g_armed = true;
            if (g_callbackActive) {
                g_recording.store(false);
                f4cf::render::setDrawCallbackActive(g_callback, false);
                g_callbackActive = false;
            }
            if (g_logOpen) {
                g_log.flush();
                g_log.close();
                g_logOpen = false;
                logger::info("Render probe: stopped ({} submits seen, placeVisualAt called {} times, {} without a model root)", g_submits.load(), g_calls.load(), g_noRoot.load());
            }
            return;
        }
        if (g_logOpen) {
            g_log.flush();
            if (g_submits.load() >= MAX_ROWS && g_callbackActive) {
                g_recording.store(false);
                f4cf::render::setDrawCallbackActive(g_callback, false);
                g_callbackActive = false;
                logger::info("Render probe: recorded {} submits; switch bRenderProbe off to finish the file", MAX_ROWS);
            }
            return;
        }
        if (!g_armed) {
            return;
        }
        g_armed = false;
        if (g_callback == f4cf::render::INVALID_DRAW_CALLBACK) {
            g_callback = f4cf::render::registerDrawCallback("VehicleFrameworkVR render probe", onSubmit, f4cf::render::DRAW_ORDER_DEBUG);
        }
        f4cf::render::ensureInstalled();
        if (g_dir.empty()) {
            return;
        }
        g_log.open(g_dir + "\\VehicleFrameworkVR_probe.csv", "submit,t_ms,seat_begin_frame,seat_end_frame,cam0_x,cam0_y,cam0_z,cam1_x,cam1_y,cam1_z,vp_r3_0,vp_r3_1,vp_r3_2,vp_r3_3,vis_x,vis_y,vis_z,written_x,written_y,written_z", MAX_ROWS + 10);
        g_logOpen = true;
        g_submits.store(0);
        g_start = std::chrono::steady_clock::now();
        g_recording.store(true);
        f4cf::render::setDrawCallbackActive(g_callback, true);
        g_callbackActive = true;
        logger::info("Render probe: recording");
    }
}
