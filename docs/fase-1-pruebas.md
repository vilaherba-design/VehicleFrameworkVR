# Fase 1: prototipo del sedán. Compilar, instalar y probar

Esta fase valida los riesgos de los que depende todo lo demás. Su puerta de salida es que la distancia
entre la cabeza y el asiento se mantenga estable a 60 km/h con FRIK activo.

## Estado (versión 0.1.39, 20 de octubre de 2026)

El plugin compila en Windows (MSVC 14.44, Visual Studio 2022 Build Tools) y ya se juega: el coche aparece, se conduce, las ruedas giran y
dirigen, hay cámara en primera y tercera persona, luces, humo de escape, sonido de motor propio por vehículo y cuatro vehículos (sedán,
Pick-R-Up, Rockboy y la moto Rusianmother). Pruebas fuera del juego: `VehicleModelTests` 156/156 y `MotoTests` 86/86, a 90 y 72 Hz.

**La fase no está cerrada.** Falta superar la puerta (la cámara a 60 km/h) y registrar una aceleración y una frenada a fondo.
Informe de `tools/analyze_telemetry.py` con el último test jugado (20-oct, 21:02-21:11, plugin 0.1.38):

| Criterio | Resultado | Dato |
|---|---|---|
| Puerta: cabeza-asiento estable a 60 km/h (p95 ≤ 3 cm, desviación ≤ 1,5 cm, 3 s) | **SIN DATOS** | Máx. 42 km/h en primera persona. El error de anclaje crecía con la velocidad: 1,8 cm parado, 4,3 cm a 5-20 km/h, 9,0 cm a 20-30 km/h y 12,9 cm a 30-45 km/h (p95 21 cm) |
| Acelera 0-100 km/h | **SIN DATOS** | Velocidad máx. 70 km/h |
| Frena 100-0 km/h | **SIN DATOS** | No hay una frenada desde 95 km/h o más |
| Gira por debajo de 10 km/h | PASA | Radio de giro mediano 8,0 m |
| Parado en cuesta no resbala | PASA | 61,6 s sujeto, cuesta de 14,5° |
| Parado no vibra | PASA | 1,73 cm/s de desviación (límite 2) |
| Coste por fotograma < 0,2 ms | PASA | Mediana 0,086 ms, máx. 0,099 ms |
| Telemetría por rueda | PASA | 4 ruedas, 18 679 pasos |
| El sedán aparece con la granada | Pendiente | El atajo mantener B lo sustituye; falta el ESP de la granada (sección 7) |

Con esos datos la puerta habría **fallado**: el error crecía con la velocidad, señal de un asiento que va por detrás del coche. La versión
0.1.39 cambia la fuente del asiento (ahora el cuerpo Havok, `iSeatPoseSource = 1`) y añade columnas de diagnóstico; falta medirla.

### Qué falta para cerrar la fase

1. Probar la 0.1.39 con el recorrido de la sección 3 (con 100 km/h a fondo, frenada completa y 10 s a 60 km/h en primera persona) y pasar el informe.
2. Que la puerta dé PASA: p95 del error cabeza-asiento ≤ 3 cm entre 55 y 65 km/h durante 3 s o más.
3. (Opcional para cerrar) El ESP de la granada de pruebas.

### Plan de fases del diseño y dónde estamos

| Fase | Nombre | Estado |
|---|---|---|
| 1 | Prototipo del sedán | Casi: falta la puerta y los dos tramos a fondo |
| 2 | Tercera persona y controles | Adelantada una primera versión de la cámara de persecución (clic del stick izquierdo; el stick derecho la orbita alrededor del coche). Faltan los tres modos (dron, persecución por pasos, persecución suave), el brazo con rayo contra paredes, los fundidos, ocultar el cuerpo del conductor y los perfiles de mando por asiento |
| 3 | Sistema de piezas | Adelantada la ranura de ruedas (originales, 4x4 y beadlock, con menú). Faltan el co-save de la pieza instalada, más ranuras y piezas visuales |
| 4 | Nitro, saltos y cohetes | Sin empezar |
| 5 | Armas y vida del vehículo | Sin empezar |
| 6 | Taller y HUD VR | Sin empezar (los menús de vehículo y ruedas son provisionales) |
| 7 | Motos, barcos y mapa | Adelantada la moto (Rusianmother, con estabilizador de equilibrio). Faltan barcos, flotación y colocar vehículos por la Commonwealth |

Siguiente según el diseño: la fase 2, una vez cerrada la 1.

## Qué incluye esta entrega

| Pieza | Dónde | Estado |
|---|---|---|
| Plugin F4SE (C++) | `src/` | Compila en Windows; funciona en el juego |
| Modelo del vehículo (suspensión, neumático, rueda con giro propio, transmisión, ABS, freno de parado, estabilizador de equilibrio) | `src/model/` | Probado fuera del juego: 156 + 86 comprobaciones a 90 y 72 Hz |
| Perfiles de los vehículos | `data/mod/F4SE/Plugins/VehicleFrameworkVR/Vehicles/*.json` | Sedán, Pick-R-Up, Rockboy y Rusianmother |
| Modelos, texturas y materiales | `data/mod/Meshes`, `Textures`, `Materials` | Generados con `tools/` (ver abajo); ESP en `VehicleFrameworkVR_Vehicles.esp` |
| Cámara en primera y tercera persona (mover y girar la sala, sin emparentar) | `src/player/DriverSeat.*` | Primera persona: la puerta, por validar con 0.1.39 |
| Telemetría CSV por paso de física y por fotograma | `src/telemetry/`, `src/vehicle/Vehicle.cpp` | Lista |
| Informe automático de los criterios de salida | `tools/analyze_telemetry.py` | Probado con datos reales |
| Sonido de motor sintetizado, humo de escape, luces | `src/audio/`, `src/vehicle/Vehicle.cpp` | Funcionan; ajuste fino por probar |
| Scripts de la granada de pruebas | `data/mod/Scripts/Source/User/VehicleFrameworkVR/` | Faltan el ESP y compilar los scripts (sección 7) |

El atajo de desarrollo **mantener B** (mano principal) saca un menú de vehículos y hace aparecer el elegido a 6 m sin granada ni ESP,
así que se puede probar todo lo demás antes de hacer el ESP de la granada.

## Vehículos incluidos

| Perfil | Vehículo | Masa | Tracción | Punta | Notas |
|---|---|---|---|---|---|
| `sedan` | Sedán (`Sedan01.nif` limpio del juego) | 1400 kg | trasera | 150 km/h | La referencia de manejo. Ruedas originales, 4x4 y beadlock |
| `pickup` | Pick-R-Up | 2100 kg | total | 135 km/h | V8, 480 N·m. Cámara a [0, 14, 131]. Ruedas originales, 4x4 y beadlock |
| `rockboy` | Rockboy (diseño de Claude Design) | 1850 kg | total | 145 km/h | Corre como el sedán con más par (470 N·m frente a 300). Cámara a [-29,5, -12, 119]. Seis cilindros. Ruedas originales, 4x4 y beadlock |
| `rusianmother` | Rusianmother (moto, diseño de Claude Design) | 260 kg | trasera | 165 km/h | Bicilíndrica de 72 N·m. Se mantiene en pie y se inclina en las curvas (`lean`). Horquilla, manillar y faro giran con la dirección |

Se eligen con el menú de mantener B (`sVehicleProfiles`, hasta 4). Con más de un juego de ruedas, después sale el menú de ruedas.

## Novedades desde 0.1.30

### Cámara en primera persona (0.1.31 a 0.1.39)

- **Causa de la cámara que se queda quieta (vídeo del Rockboy):** el asiento leía el nodo del motor, y ese nodo dejó de seguir al cuerpo Havok.
  La cámara se quedaba parada mientras el coche se alejaba. Ahora `iSeatPoseSource = 1` (el cuerpo Havok) es el valor por defecto.
- **Posiciones de los ojos** a la altura de cada cabina: Pick-R-Up [0, 14, 131], Rockboy [-29,5, -12, 119], moto [0, 3, 108] (unidades del juego, marco del
  modelo). Se ajustan en `seat.driverEye` del perfil.
- **Diagnóstico nuevo:** el CSV de cámara lleva `view` (0 = primera persona), `node_body_cm` y `node_lead_cm`: la distancia entre el nodo dibujado y el cuerpo
  Havok, y cuánto va el cuerpo por delante. Si `node_body_cm` crece con la velocidad, el nodo se está quedando atrás.
- **Moto:** `seat.leanFollow` (0,5) reparte la inclinación que ve el conductor: la cabeza no se balancea tanto como la moto.

### Sonido y humo

- **Voz por vehículo** (`engine.sound` y `engine.cylinders` en cada perfil): ganancia, tono, armónicos, "latido" (`rumble`, `lumpiness`), ruido y brillo.
  Volumen general más bajo (`fEngineVolume = 0.6`). Suena por Windows, no por el audio del juego.
- **Humo de escape:** sprite de humo suave (`ExhaustPuff.nif`, `SmokePuff_d.dds`, `SmokePuff.BGEM`, un material de efecto con bordes difuminados) en lugar del efecto
  anterior, que se veía como burbujas. Puntos de salida en `exhaust.points`.

### Rockboy rehecho (0.1.39)

- **Por qué se veía mal:** las normales no se habían convertido a los ejes del juego (la permutación de ejes del glb se aplicó a las posiciones, no a las
  normales) y la luz sombreaba al revés en la mitad de las caras. Corregido en `tools/nif/build_rockboy.py` y comprobado contra la geometría (coseno 0,993-0,999).
- **Topología limpia a partir del modelo original de Claude Design:** carrocería 450 k → 100 k triángulos (error relativo 0,17 %) y piezas 335 k → 113 k, con
  el simplificador cuadrático de meshoptimizer. Total ≈ 213 000 triángulos, en formas de menos de 65 535 vértices (índices de 16 bits).

### Moto Rusianmother (0.1.39)

- **Modelo:** `tools/nif/build_rusianmother.py <scrambler.glb> <Sedan01.nif> <BeadlockWheel.BGSM> <data>`. 1362 piezas y 141 k triángulos del glb, simplificadas a
  ≈ 102 000, con un atlas de pintura (`RusianmotherAtlas_{d,n,s}.dds`, una celda por material). Cuatro grupos móviles: cuerpo, `SteerFork` (horquilla,
  manillar, faro), `WheelFront` y `WheelRear`.
- **Equilibrio del conductor (`lean`):** con dos ruedas en el eje central no hay barras estabilizadoras que la sujeten, así que un estabilizador calcula el par
  que inclina la moto hasta el ángulo de la curva (atan(aceleración lateral / g), medida con la velocidad de giro) y la mantiene en pie parada. Si cae más de
  `fallDeg`, el estabilizador la suelta.
- **Dirección:** `steering.maxLateralG` limita el recorrido del stick al ángulo que da esa aceleración lateral a la velocidad actual (a 60 km/h son pocos grados),
  porque con el ángulo de coche la moto se caía. `steering.pivot` y `steering.axis` son el eje de dirección inclinado, sobre el que giran horquilla y rueda.
- Pruebas: `tests/MotoTests.cpp`, 86 comprobaciones a 90 y 72 Hz: se levanta de una inclinación y aguanta un empujón parada, no se desliza en una cuesta del 15 %,
  0-100 en 5,5 s y 158 km/h de punta, 100-0 en 44 m sin dar la vuelta, curva a 60 km/h con el stick a fondo (34° de inclinación, 0,85 g, derrape de 3,7°), vuelta a
  5 km/h, cambio de carril a 80 km/h (velocidad de balanceo 122°/s, límite 130), marcha atrás, el estabilizador no levanta una moto caída y la cámara sigue la parte
  de inclinación configurada.

### Telemetría

Las filas de física llevan ahora `lean_deg` y `lean_target_deg` tras `impact_z`; una moto rellena con ceros las columnas de las ruedas que no tiene (siempre 4).

### Claves nuevas de los perfiles

| Clave | Para qué |
|---|---|
| `wheels[].radius` | Radio propio de cada rueda (m); si falta, `tyre.radius` |
| `steering.maxLateralG` | Recorrido del stick = ángulo que da esa aceleración lateral a la velocidad actual (0 = desactivado, se usa `reduceAtSpeed`) |
| `steering.pivot`, `steering.axis` | Punto y eje de dirección inclinado. Los nodos `Steer...` y las ruedas dirigidas giran sobre él |
| `lean` | Estabilizador de equilibrio: `maxLeanDeg`, `follow`, `frequencyHz`, `damping`, `uprightBelow`, `armLength`, `rollInertia`, `maxTorque`, `fallDeg`, `enabled` |
| `seat.leanFollow` | Parte de la inclinación que sigue la cámara (1 = toda) |
| `engine.cylinders`, `engine.sound` | Cadencia y voz del motor |
| `exhaust.points` | Salidas de humo |
| `lights.form`, `lights.points`, `lights.pitchDownDeg`, `lights.yawDeg` | Luces |
| `wheelVisual.set`, `wheelVisual.maxAboveMesh`, `wheelVisual.lift` | Juego de ruedas visible y cuánto puede asomar la rueda sobre la chapa |
| `hull`, `boxHull` | Casco de colisión explícito, con rampas en los extremos |

### Herramientas (`tools/`)

`tools/nif/build_clean.py` (coche limpio y ruedas), `build_rockboy.py`, `build_rusianmother.py`, `wheel_beadlock.py`, `make_wheel_assets.py`,
`make_smoke_assets.py`, `make_vehicle_esp.py` (genera el ESP con todos los vehículos), `preview.py` (vista previa de un nif). Los dos modelos de Claude Design
usan `tools/nif/meshopt.py`, que carga `libmeshopt.so` (meshoptimizer, MIT, se compila aparte con g++; instrucciones en el propio archivo). Solo hace falta para
regenerar los nif: la compilación del plugin no lo usa y los nif ya generados van en `data/mod`.

## 1. Compilar

Requisitos (los mismos que FRIK): Visual Studio 2022 o 2026 con *Desarrollo para el escritorio con C++*,
CMake 4.2 o superior, Git y vcpkg con la variable de entorno `VCPKG_ROOT`.

```bat
cd VehicleFrameworkVR
git submodule update --init --recursive
copy CMakeUserPresets.json.template CMakeUserPresets.json
```

En `CMakeUserPresets.json`, quita el bloque de comentarios del principio y pon en `COPY_PLUGIN_BASE_PATH`
la carpeta del mod en MO2 (por ejemplo `C:/Modding/FO4VR/mods/Vehicle Framework VR`). Después:

```bat
cmake --preset custom
cmake --build --preset custom-release
```

La primera configuración tarda: vcpkg compila las dependencias una vez. Con Visual Studio 2022, cambia
`"inherits": "default"` por un preset que herede de `vs2022` (ver `CMakePresets.json`).

**Pruebas del modelo:** la compilación también genera `VehicleModelTests.exe` (sedán) y `MotoTests.exe` (Rusianmother). Ejecuta
`ctest --test-dir build -C Release` para repetir las comprobaciones con los perfiles del repo. Sirve para comprobar un cambio de ajuste del JSON
antes de ponerte el casco. Los umbrales de `VehicleModelTests` son del sedán: con otro perfil, algunos fallan por diseño (pickup y Rockboy no llegan a
150 km/h).

## 2. Instalar en MO2

La compilación copia `VehicleFrameworkVR.dll` en `<mod>/F4SE/Plugins/`. Copia además a la carpeta del mod
el contenido de `data/mod/` (o usa `cmake --build --preset custom-package` y instala el `.7z` de `build/package/`).
El ESP `VehicleFrameworkVR_Vehicles.esp` tiene que estar activo (sin él se usa el sedán oxidado del juego y los demás vehículos no aparecen).

Necesitas: F4SE VR, VR Address Library y FRIK 0.79.1 o superior. ROCK y Daytripper 4 se prueban con y sin ellos.

Revisa en `Fallout4Custom.ini` (o el ini que use tu instalación):

```ini
[HAVOK]
fMaxTime=0.0111   ; 1/90 con el Quest 3 a 90 Hz (0.0139 a 72 Hz)
```

## 3. Rutina de cada prueba

1. Cierra el juego. Borra `Documents\My Games\Fallout4VR\F4SE\VehicleFrameworkVR*.csv` y `VehicleFrameworkVR*.log`, y también
   `Documents\My Games\Fallout4VR\Mods_Config\VehicleFrameworkVR\VehicleFrameworkVR.ini`: el plugin lo vuelve a crear con los valores por defecto de esa versión
   (`iVersion` del ini, ahora 14). Así no se arrastran ajustes viejos.
2. Arranca, carga una partida en exterior con terreno llano y alguna cuesta cerca.
3. Mantén **B** (mano principal): sale el menú de vehículos. El elegido aparece a 6 m, mirando hacia donde miras, y, si tiene varios juegos de ruedas,
   sale el menú de ruedas. El juego avisa con "Test vehicle ready".
4. Acércate a la puerta del conductor y mantén **A** para subir.
5. Haz el recorrido de abajo y baja con **mantener Y** (mando izquierdo).
6. Cierra el juego y mándame los dos CSV y el log, con el número de versión del plugin.

Controles mientras conduces:

| Acción | Control |
|---|---|
| Acelerar | Gatillo derecho (analógico) |
| Frenar; mantenido parado, marcha atrás | Gatillo izquierdo (analógico) |
| Dirección | Stick izquierdo |
| Recentrar el asiento | Tocar el clic del stick derecho |
| Luces | Mantener el clic del stick derecho |
| Tercera persona | Clic del stick izquierdo (en tercera persona, el stick derecho orbita) |
| Bajar | Mantener Y |

Con la moto la inclinación es automática: solo dirección, acelerador y freno.

Recorrido (unos 3 minutos, con el sedán):

1. Parado 5 s sin tocar nada (vibración en reposo).
2. A fondo desde parado hasta pasar de 100 km/h, en recta (0-100). Hace falta recta larga.
3. Freno a fondo hasta parar (100-0).
4. Volante a tope a paso de peatón, una vuelta completa (giro lento).
5. Mantén 60 km/h durante al menos 10 s, con alguna curva suave, **en primera persona** (la puerta).
6. Para en una cuesta, suelta todo 5 s (no debe resbalar).
7. Marcha atrás unos metros.

## 4. Informe de criterios

Con Python 3 instalado:

```bat
python tools\analyze_telemetry.py ^
  --physics "%USERPROFILE%\Documents\My Games\Fallout4VR\F4SE\VehicleFrameworkVR_physics.csv" ^
  --camera  "%USERPROFILE%\Documents\My Games\Fallout4VR\F4SE\VehicleFrameworkVR_camera.csv" ^
  --log     "%USERPROFILE%\Documents\My Games\Fallout4VR\F4SE\VehicleFrameworkVR.log"
```

Saca una tabla con cada criterio de salida: PASA, NO PASA o SIN DATOS (falta ese tramo del recorrido).
Los umbrales están al principio del script; son propuestas hasta tener más datos.

## 5. Qué mirar en el log

Al cargar el juego:

- `Engine addresses: AddPhysicsStepListener ... match` y `PickObject ... match`: las direcciones coinciden con las que usa ROCK.
- `FRIK API v2 (vN) ready` y `FRIK FrameBegin callback registered`.
- `Raycast world-reference release verified`.

Al aparecer el vehículo:

- `Vehicle: body N on node '...'; shape type X, motion type 0`, seguido de `motion type 0 -> 1 (dynamic), mass 1400 kg`.
  Si en vez de eso sale `expected 1 = dynamic`, la forma de colisión no admite ser dinámica y toca el plan B del diseño (chasis caja).
- `wheel layout FL hub [...]`: posiciones de las ruedas. Con `boxHull` y ruedas explícitas en el perfil, son las del JSON.
- `node vs Havok body frame: X cm`: si no es casi 0, el nodo dibujado va detrás del cuerpo (ver cámara arriba).
- `Vehicle: IMPACT ...`: el casco ha chocado con algo; dice qué hay delante y debajo del coche.

Al subir y conducir:

- `Character controller vtable verified`: si sale `NOT verified`, no se desactiva la colisión del jugador y el coche puede salir empujado.
- `Seat: room yaw direction confirmed` o `... was reversed`: el sentido de giro de la sala se detecta solo en la primera curva.
- Cada 2 s: `head-seat anchor error X cm`. Es la medida de la puerta.
- Cada 5 s: `Cost per frame: X ms total`. El objetivo es menos de 0,2 ms.
- Si sale `no physics step callback for 90 frames` con el juego en marcha (no con un menú abierto), la escucha del paso de física no funciona y el coche no se moverá.

## 6. Ajustes de diagnóstico (se aplican en caliente)

Todo está en `Documents\My Games\Fallout4VR\Mods_Config\VehicleFrameworkVR\VehicleFrameworkVR.ini`:

| Ajuste | Para qué |
|---|---|
| `bSimpleMode = true` | Física simple sin curva de neumático: separa problemas del neumático de problemas de cámara |
| `bDebugDraw = true` | Dibuja rayos de rueda (verde con contacto, rojo sin él), carga de cada rueda, asiento y punto de salida |
| `iCameraUpdatePoint = 0 / 1` | Coloca al conductor en el fotograma del plugin o en el FrameBegin de FRIK |
| `iSeatPoseSource = 0 / 1` | El asiento sigue al nodo dibujado (0) o al cuerpo Havok (1, por defecto) |
| `fVerticalFilterHz`, `fVerticalFilterMax` | Filtro de la vibración vertical de la cámara |
| `iImpulseApi = 0 / 1` | Función del motor que aplica las fuerzas de rueda |
| `sWheelRayLayer`, `sWheelRayIgnoreLayers` | Capa de colisión de los rayos de rueda y capas que atraviesan |
| `fImpactLogMps` | Cambio de velocidad (m/s) que el modelo no explica y se registra como `IMPACT` |
| `bEngineSound`, `fEngineVolume`, `bExhaustSmoke` | Sonido del motor y humo de escape |
| `sVehicleProfiles`, `sWheelSet`, `bWheelMenuAfterSpawn`, `bWheelMenuOnEnter` | Vehículos del menú y juegos de ruedas |
| `fRightAfterSeconds` | Segundos de espera antes de que un coche volcado se enderece solo (0 = desactivado) |

Los perfiles de los vehículos (`Vehicles\*.json`) se vuelven a leer cada vez que aparece un vehículo: cambia un valor,
mantén B otra vez y el nuevo ya lo usa.

## 7. Granada de pruebas (ESP)

Hace falta para el primer criterio ("el sedán aparece con la granada"). En el Creation Kit, con un ESP
normal (no ESL) llamado `VehicleFrameworkVR.esp`:

1. **Compila los scripts.** Copia `Scripts\Source\User\VehicleFrameworkVR\` a `Data\Scripts\Source\User\`
   y compílalos (Gestión de scripts de Papyrus del CK, o `PapyrusCompiler.exe` con `-f=Institute_Papyrus_Flags.flg`). Salen
   `Data\Scripts\VehicleFrameworkVR\Native.pex` y `TestGrenadeMarker.pex`.
2. **Marcador.** Activador nuevo `VFVR_TestGrenadeMarker`, sin modelo o con uno de marcador, con el script
   `VehicleFrameworkVR:TestGrenadeMarker`.
3. **Explosión.** Duplica la explosión de la granada de señales del vertibird (arma `BoSVertibirdGrenade`,
   sigue su proyectil hasta la explosión) como `VFVR_TestGrenadeExplosion`. Pon daño y fuerza a 0 y en
   *Placed Object* el activador `VFVR_TestGrenadeMarker`. Conserva el humo de color.
4. **Proyectil.** Duplica el proyectil de esa granada como `VFVR_TestGrenadeProjectile`, con la explosión
   anterior y la detonación al impacto (*Explosion - Alt. Trigger* / impacto), para que los 5 s cuenten desde que cae.
5. **Arma.** Duplica `BoSVertibirdGrenade` como `VFVR_TestGrenade` ("Granada de vehículo (pruebas)") con ese proyectil.
6. **Receta (opcional).** Un *Constructible Object* en el banco de química, o en consola:
   `player.additem <FormID de VFVR_TestGrenade> 5`.

Al caer, el marcador espera 5 s (un solo `StartTimer`, sin bucles), llama a
`VehicleFrameworkVR:Native.SpawnTestVehicleAt` y se borra. El plugin coloca el vehículo de prueba (`sTestVehicleProfile`) en ese punto, medio
metro sobre el suelo y en la línea del lanzamiento, lo separa si estás a menos de 3,5 m y borra el vehículo de prueba anterior.

## 8. Decisiones de esta implementación respecto al diseño

- **Rayos de rueda una vez por fotograma, fuerzas en cada paso de física.** Los rayos se lanzan en el hilo
  del juego y cada rueda guarda el plano del suelo; en cada paso de Havok la suspensión y el neumático se
  calculan contra ese plano con el estado del cuerpo de ese paso. Así no se lanza ningún rayo dentro del
  paso de física, donde ROCK documenta problemas de bloqueo del mundo.
- **La escucha del paso de física se registra cada fotograma**, porque el motor vacía la lista en cada
  actualización (lo mismo que hace ROCK).
- **ABS** en el modelo: sin él, el sedán bloqueaba ruedas y frenaba en 58 m en vez de 45.
- **Barras estabilizadoras distintas delante y detrás** (más rígida delante): el sedán tiende a subvirar y
  no hace trompos al girar a 60 km/h con el stick a medio recorrido.
- **Dirección que se reduce con la velocidad** (a 25 m/s queda el 22 %), por estabilidad y confort en VR. En la moto, `maxLateralG`.
- **Freno de parado:** por debajo de 0,35 m/s sin acelerador, cada rueda se ancla al suelo con un muelle
  limitado por el agarre. No resbala en una cuesta del 15 % y no vibra en reposo.
- **Superficies:** el agarre por material del suelo queda para la fase de piezas; ahora todo es asfalto.
- **Coche limpio:** el sedán, la Pick-R-Up, el Rockboy y la moto son objetos propios del ESP (`VFVR_*`), con su nif en `Meshes\VehicleFrameworkVR\Vehicles\` y los
  bloques de física del sedán de posguerra. Sin ESP, solo se puede usar el sedán oxidado del juego.
- **Cuerpo de colisión caja** (`boxHull`): el cuerpo dinámico es una caja con rampas en los extremos; la masa y la inercia las calcula el motor a partir de ella.
  Las ruedas de colisión son rayos: el visual es cosmético y `wheelVisual.maxAboveMesh` limita cuánto asoma sobre la chapa.
- **Granada:** el plugin hace la colocación (rumbo, distancia mínima, suelo y borrado del anterior) y el
  script solo espera 5 s y llama al plugin.
