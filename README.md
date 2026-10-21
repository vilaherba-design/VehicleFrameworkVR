# Vehicle Framework VR

Framework de vehículos para Fallout 4 VR: coches, motos y barcos movidos con física Havok continua desde
un plugin F4SE en C++, con cámara en primera y tercera persona pensada para VR.

Construido sobre [F4VR Common Framework](https://github.com/ArthurHub/F4VR-CommonFramework) y
[CommonLibF4VR](https://github.com/ArthurHub/CommonLibF4VR), la misma base que FRIK.

**Estado: fase 1, prototipo del sedán.** Física por velocidad con rueda virtual, cámara en primera persona,
granada de pruebas y telemetría. La guía para compilar, instalar y probar está en
[docs/fase-1-pruebas.md](docs/fase-1-pruebas.md).

## Principios

- Nada por fotograma en Papyrus: el plugin aplica las fuerzas en cada paso de Havok y Papyrus solo recibe eventos.
- Nada emparentado al vehículo: el jugador se mueve como al caminar y la sala VR gira con el coche, así el
  tracking y el roomscale quedan intactos.
- Todo definido por datos: cada vehículo es un JSON en `Data\F4SE\Plugins\VehicleFrameworkVR\Vehicles\`.
- Confort VR primero: horizonte nivelado y sin tirones de cámara.
- Telemetría desde el primer día: CSV por paso de física y por fotograma, y un script que comprueba los criterios.

## Requisitos

- Fallout 4 VR 1.2.72, [F4SE VR](https://f4se.silverlock.org/) y VR Address Library for F4SEVR
- [FRIK](https://www.nexusmods.com/fallout4/mods/53464) 0.79.1 o superior
- Opcionales: [ROCK](https://www.nexusmods.com/fallout4/mods/108881) y [Daytripper 4](https://www.nexusmods.com/fallout4/mods/91141)

## Estructura

| Carpeta | Contenido |
|---|---|
| `src/model/` | Modelo del vehículo sin dependencias del juego: suspensión, neumático, transmisión, frenos |
| `src/physics/` | Puente con Havok: cuerpos, rayos, impulsos, escucha del paso de física |
| `src/vehicle/` | Vehículo en el juego: une la referencia, el cuerpo y el modelo |
| `src/player/` | Asiento del conductor y cámara en primera persona |
| `src/telemetry/` | CSV de telemetría |
| `src/papyrus/` | Funciones nativas para los scripts |
| `data/` | INI por defecto, perfiles JSON y scripts de Papyrus |
| `tests/` | Pruebas del modelo fuera del juego |
| `tools/` | Informe de criterios a partir de la telemetría |

## Compilar

Visual Studio 2022 o 2026 (x64), CMake 4.2+ y vcpkg (`VCPKG_ROOT`).

```bat
git submodule update --init --recursive
cmake --preset default
cmake --build build --config Release
ctest --test-dir build -C Release
```

Para copiar el DLL a MO2 en cada compilación, usa `CMakeUserPresets.json` (plantilla incluida).

## Licencia

GPL-3.0, como F4VR Common Framework. Incluye código adaptado de [ROCK](https://github.com/brunocatani/ROCK)
(GPL-3.0) y la cabecera de la API v2 de [FRIK](https://github.com/rollingrock/Fallout-4-VR-Body) copiada sin cambios.
