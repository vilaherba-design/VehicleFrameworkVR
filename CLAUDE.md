# CLAUDE.md

Guidance for Claude Code working in this repository.

## Project

Vehicle Framework VR (`VehicleFrameworkVR`) is a C++23 F4SE plugin for Fallout 4 VR 1.2.72 that drives
vehicles with continuous Havok physics. Built on [F4VR Common Framework](external/F4VR-CommonFramework/CLAUDE.md)
(submodule) and CommonLibF4VR. GPL-3.0. Current state: **phase 1 prototype** (sedan, first-person seat,
test grenade, telemetry). The design document is in Spanish; user-facing docs are in Spanish, code and
comments in English.

## Build and test

```bash
git submodule update --init --recursive
cmake --preset default                  # VS 2026; see CMakePresets.json for vs2022
cmake --build build --config Release
ctest --test-dir build -C Release       # offline vehicle model tests (tests/VehicleModelTests.cpp)
```

The model tests are pure C++ (no game, no PCH) and also build on Linux:
`g++ -std=c++23 -Isrc -I<nlohmann include> tests/VehicleModelTests.cpp src/model/*.cpp`.
Run them after any change to `src/model/` or `sedan.json`; they check the design targets (0-100 km/h
~7 s, top speed ~150 km/h, 100-0 ~45 m, ~0.9 g lateral, no slide on a 15% slope, no vibration at rest)
at 90 and 72 Hz. With a second argument they write a scripted lap in the plugin's CSV format.

`tools/analyze_telemetry.py` turns the in-game CSVs (and the log) into a pass/fail table of the phase 1
exit criteria.

## Architecture

| Component | Files | Thread | Responsibility |
|---|---|---|---|
| Mod | `src/VehicleFrameworkVR.*` | game | Lifecycle, test spawn (binding or Papyrus), driver input, seat bindings, cost report |
| Config | `src/Config.*`, `data/config/VehicleFrameworkVR.ini` | watcher/game | INI with hot reload |
| Model | `src/model/` | step | Suspension, tyre (magic formula + wheel spin, implicit), drivetrain, ABS, parking hold. Returns impulses, never moves the body |
| Havok bridge | `src/physics/HavokBridge.*` | game/step | Body lookup, dynamic switch, rays (PickObject + world-ref release), impulses, step listener |
| Vehicle | `src/vehicle/Vehicle.*` | game + step | Rays per frame -> ground planes; model per Havok step; telemetry rows |
| Driver seat | `src/player/DriverSeat.*` | game | First-person placement: move the actor and turn the VR room, never parent |
| Telemetry | `src/telemetry/CsvLog.*` | any -> game | Buffered CSV |
| Papyrus | `src/papyrus/`, `data/mod/Scripts/Source/User/VehicleFrameworkVR/` | VM | `SpawnTestVehicleAt` (queues to the game thread) |

Frame flow: game thread casts wheel rays and publishes contacts + input under a mutex, re-registers the
step listener (the engine clears the list every update), keeps the body awake. Havok calls the listener
before each (sub)step; the model computes impulses against the cached ground planes and they are applied
with `hknpWorld::ApplyBodyImpulseAt`. The seat is placed from the plugin frame or FRIK's `FrameBegin`.

## Engine facts this code relies on (FO4VR 1.2.72)

- `bhkWorld + 0x60` is the `hknpWorld*`; 1 Havok unit = 69.99125 game units (~1 m).
- Step listener: object with an 11-slot vtable, slot 3 = before each step `(listener, u32, void*, progress, dt)`.
  Must be re-registered every frame (`bhkWorld::AddPhysicsStepListener`, REL::ID 137330 = RVA 0x1DFA7B0).
- `bhkPickData` keeps a world reference at +0xC0 after `PickObject`; release it or every ray leaks one.
- Player character controller is at `MiddleHighProcessData + 0x3E8` in VR (CommonLib's member is the flat 0x3E0).
  Virtual calls on it only after checking slot 0x34 == RVA 0x1E4DF60.
- `hknpBody` +0x00..0x3F holds the body transform (column-major rotation + translation); the motion holds the COM.
- VR turning rotates the room (`f4vr::PlayerRotation`), not the actor.

Sources: CommonLibF4VR `HAVOK_PHYSICS_ADDITIONS.md`, ROCK (`src/physics-interaction/native/`), FRIK API v2 docs.
`src/api/FRIKApiV2.h` is copied verbatim from FRIK: never edit or reformat it.

## Conventions

- clang-format (`.clang-format`): LLVM-based, 180 columns, 4 spaces. `pre-commit run --files <files>`.
- `namespace vfvr` (model in `vfvr::model`, Havok in `vfvr::physics`).
- Nothing per frame in Papyrus. Nothing parented to the vehicle. Vehicles and parts are data (JSON).
- Mark functions by thread in comments: `[game]`, `[step]`.
- Bump `iVersion` in the INI `[Debug]` section when the shipped INI changes.
