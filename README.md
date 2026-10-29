# Vehicle Framework VR

[Español](#español) · [English](#english)

## Español

Framework de vehículos para Fallout 4 VR. Es un plugin F4SE VR escrito en C++ (CommonLibF4VR) con coches y una
moto que usan física de ruedas por rayos sobre Havok, cámara en primera y tercera persona pensada para VR,
maletero, HUD, salud del vehículo, batería y torretas.

Este repositorio contiene **solo el código fuente**. Los assets del juego (modelos, texturas, materiales y el ESP)
no están aquí: se distribuyen únicamente en la descarga de Nexus Mods. Los scripts de `tools/` que generan
algunos de esos assets sí están incluidos.

### Requisitos para jugar

- Fallout 4 VR 1.2.72
- [F4SE VR](https://f4se.silverlock.org/)
- VR Address Library for F4SEVR
- [FRIK](https://www.nexusmods.com/fallout4/mods/53464) 0.79.1 o superior
- Opcionales: [ROCK](https://www.nexusmods.com/fallout4/mods/108881) y [Daytripper 4](https://www.nexusmods.com/fallout4/mods/91141)

### Compilar

Necesitas Visual Studio 2022 o 2026 con "Desarrollo para el escritorio con C++", CMake 4.2+, Git y
[vcpkg](https://github.com/microsoft/vcpkg) con la variable de entorno `VCPKG_ROOT` apuntando a él.

```bat
git clone https://github.com/vilaherba-design/VehicleFrameworkVR.git
cd VehicleFrameworkVR
git submodule update --init --recursive
copy CMakeUserPresets.json.template CMakeUserPresets.json
rem Edita CMakeUserPresets.json: quita el bloque de comentarios y pon COPY_PLUGIN_BASE_PATH
rem (o POST_BUILD_COPY_PLUGIN en false si no quieres copiar el DLL)
cmake --preset custom
cmake --build --preset custom-release
ctest --test-dir build -C Release
```

`ctest` ejecuta las pruebas del modelo del vehículo fuera del juego. El DLL queda en `build\Release\`.

### Novedades desde 0.1.77

- Vehículos nuevos definidos por perfiles JSON: Rockboy v7, Atomic Coupe, Army Wagon (4x4 militar), Coupe 50, Bel Air y Coupe amarillo, con juegos de ruedas propios (clásicas y militares).
- Menú de vehículos y ruedas como panel flotante de dos columnas, por páginas.
- Batería: los vehículos gastan batería, se recargan en estaciones de carga construibles en el taller del asentamiento, y en carretera aceptan un núcleo de fusión.
- Torretas montadas en el vehículo (varios tipos, varias bocas, sonidos de disparo y menú tras colocarlas); el kit se fabrica en el banco de química.
- Combate y salud: los enemigos ven y atacan al conductor, la cobertura del conductor reduce el daño a una décima y el resto se carga a la salud del vehículo; daño por impactos.
- Tercera persona: el cuerpo se queda en el asiento y solo se mueve la cámara (compatible con el modo selfie de FRIK).
- Vigilante de bloqueos en el log, etiqueta de la compilación en el log y en el juego, y migración del INI que añade los vehículos nuevos.

### Verificar la versión publicada

La versión **0.1.114-torretas2** de Nexus Mods se compiló desde el commit
`35c6cf7d7d61fe057298ed1452df9ca454913472` del repositorio de desarrollo (rama `torreta-113`). Este repositorio público contiene exactamente ese
código, sin los assets del juego (modelos, texturas, materiales, ESP y scripts Papyrus compilados), que solo se
distribuyen en la descarga de Nexus Mods. Las fuentes `.psc` sí están incluidas.

SHA256 de `F4SE/Plugins/VehicleFrameworkVR.dll` dentro del zip 0.1.114-torretas2:

```
CC8BC3913341F6719F9E2729BE4C13E346DD6DE7BCE6B2C4A4BD7D38916422CF
```

En PowerShell: `Get-FileHash VehicleFrameworkVR.dll -Algorithm SHA256`.
Una compilación propia no tiene por qué dar el mismo hash byte a byte (MSVC incrusta marcas de tiempo y rutas),
pero el código es el mismo. Las versiones anteriores tienen su etiqueta (`v0.1.77`).

## English

Vehicle framework for Fallout 4 VR. It is an F4SE VR plugin written in C++ (CommonLibF4VR) with cars and a
motorbike driven by raycast wheel physics on top of Havok, a VR-friendly first and third person camera, a trunk,
a HUD, vehicle health, a battery and turrets.

This repository contains **source code only**. Game assets (models, textures, materials and the ESP) are not
here; they ship only in the Nexus Mods download. The `tools/` scripts that generate some of those assets are
included.

### Requirements to play

- Fallout 4 VR 1.2.72
- [F4SE VR](https://f4se.silverlock.org/)
- VR Address Library for F4SEVR
- [FRIK](https://www.nexusmods.com/fallout4/mods/53464) 0.79.1 or newer
- Optional: [ROCK](https://www.nexusmods.com/fallout4/mods/108881) and [Daytripper 4](https://www.nexusmods.com/fallout4/mods/91141)

### Build

You need Visual Studio 2022 or 2026 with "Desktop development with C++", CMake 4.2+, Git and
[vcpkg](https://github.com/microsoft/vcpkg) with the `VCPKG_ROOT` environment variable pointing to it.

```bat
git clone https://github.com/vilaherba-design/VehicleFrameworkVR.git
cd VehicleFrameworkVR
git submodule update --init --recursive
copy CMakeUserPresets.json.template CMakeUserPresets.json
rem Edit CMakeUserPresets.json: remove the comment block and set COPY_PLUGIN_BASE_PATH
rem (or set POST_BUILD_COPY_PLUGIN to false to skip copying the DLL)
cmake --preset custom
cmake --build --preset custom-release
ctest --test-dir build -C Release
```

`ctest` runs the offline vehicle model tests. The DLL ends up in `build\Release\`.

### What's new since 0.1.77

- New vehicles defined by JSON profiles: Rockboy v7, Atomic Coupe, Army Wagon (military 4x4), Coupe 50, Bel Air and Coupe amarillo, with their own wheel sets (classic and military).
- Vehicle and wheel menus as a floating two-column panel, in pages.
- Battery: vehicles drain a battery, recharge at charging stations buildable in the settlement workshop, and take a fusion core on the road.
- Vehicle-mounted turrets (several types, multiple muzzles, firing sounds and a menu after placing them); the kit is made at the chemistry station.
- Combat and health: enemies see and attack the driver, driver cover cuts damage to a tenth and the rest is charged to the vehicle's health; impact damage.
- Third person: the body stays in the seat and only the camera moves (works with FRIK's selfie mode).
- Hang watchdog in the log, build label in the log and in game, and an INI migration that adds new vehicles.

### Verify the release

Release **0.1.114-torretas2** on Nexus Mods was built from commit `35c6cf7d7d61fe057298ed1452df9ca454913472` of the development repository
(branch `torreta-113`). This public repository contains exactly that code, without the game assets (models,
textures, materials, ESP and compiled Papyrus scripts), which ship only in the Nexus Mods download. The `.psc`
sources are included.

SHA256 of `F4SE/Plugins/VehicleFrameworkVR.dll` inside the 0.1.114-torretas2 zip:

```
CC8BC3913341F6719F9E2729BE4C13E346DD6DE7BCE6B2C4A4BD7D38916422CF
```

In PowerShell: `Get-FileHash VehicleFrameworkVR.dll -Algorithm SHA256`.
Your own build will not necessarily match byte for byte (MSVC embeds timestamps and paths), but the code is the
same. Earlier releases are tagged (`v0.1.77`).

## Credits / Créditos

- [CommonLibF4VR](https://github.com/ArthurHub/CommonLibF4VR) and
  [F4VR Common Framework](https://github.com/ArthurHub/F4VR-CommonFramework) by ArthurHub
- [FRIK](https://github.com/rollingrock/Fallout-4-VR-Body) by rollingrock and ArthurHub (API v2 header copied unchanged)
- [ROCK](https://github.com/brunocatani/ROCK) (GPL-3.0, some code adapted)

## License / Licencia

[GPL-3.0](LICENSE), same as F4VR Common Framework.
