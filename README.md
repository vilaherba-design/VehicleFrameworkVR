# Vehicle Framework VR

[Español](#español) · [English](#english)

## Español

Framework de vehículos para Fallout 4 VR. Es un plugin F4SE VR escrito en C++ (CommonLibF4VR) con coches y una
moto que usan física de ruedas por rayos sobre Havok, cámara en primera y tercera persona pensada para VR,
maletero, HUD y salud del vehículo.

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

### Verificar la versión publicada

La versión **0.1.77** de Nexus Mods se compiló desde el commit
`91c2b0046e7838b48bcee4334f21b90b6fcccf15` del repositorio de desarrollo. Este repositorio público contiene
exactamente ese código (sin los assets del juego).

SHA256 de `F4SE/Plugins/VehicleFrameworkVR.dll` dentro del zip 0.1.77:

```
B36143C3F5424E82C096F3DFA9E150B386498FC15C0D8F87E531B1A05581A261
```

En PowerShell: `Get-FileHash VehicleFrameworkVR.dll -Algorithm SHA256`.
Una compilación propia no tiene por qué dar el mismo hash byte a byte (MSVC incrusta marcas de tiempo y rutas),
pero el código es el mismo.

## English

Vehicle framework for Fallout 4 VR. It is an F4SE VR plugin written in C++ (CommonLibF4VR) with cars and a
motorbike driven by raycast wheel physics on top of Havok, a VR-friendly first and third person camera, a trunk,
a HUD and vehicle health.

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

### Verify the release

Release **0.1.77** on Nexus Mods was built from commit `91c2b0046e7838b48bcee4334f21b90b6fcccf15` of the
development repository. This public repository contains exactly that code (without the game assets).

SHA256 of `F4SE/Plugins/VehicleFrameworkVR.dll` inside the 0.1.77 zip:

```
B36143C3F5424E82C096F3DFA9E150B386498FC15C0D8F87E531B1A05581A261
```

In PowerShell: `Get-FileHash VehicleFrameworkVR.dll -Algorithm SHA256`.
Your own build will not necessarily match byte for byte (MSVC embeds timestamps and paths), but the code is the
same.

## Credits / Créditos

- [CommonLibF4VR](https://github.com/ArthurHub/CommonLibF4VR) and
  [F4VR Common Framework](https://github.com/ArthurHub/F4VR-CommonFramework) by ArthurHub
- [FRIK](https://github.com/rollingrock/Fallout-4-VR-Body) by rollingrock and ArthurHub (API v2 header copied unchanged)
- [ROCK](https://github.com/brunocatani/ROCK) (GPL-3.0, some code adapted)

## License / Licencia

[GPL-3.0](LICENSE), same as F4VR Common Framework.
