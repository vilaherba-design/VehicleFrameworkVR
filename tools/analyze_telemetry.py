#!/usr/bin/env python3
"""
Phase 1 exit-criteria report from the plugin's telemetry.

Reads the CSV files the plugin writes in Documents\\My Games\\Fallout4VR\\F4SE\\ and, optionally, the
plugin log, and prints a Markdown report with one line per exit criterion of phase 1:

  - the gate: head-to-seat distance stable at 60 km/h with FRIK (camera CSV)
  - accelerates, brakes and turns with the slip model, including at low speed (physics CSV)
  - parked it does not slide, and at rest it does not vibrate (physics CSV)
  - plugin cost under 0.2 ms per frame (log)
  - the CSV logs per-wheel spin, slip, grip and forces (physics CSV)

Usage:
  python analyze_telemetry.py --physics VehicleFrameworkVR_physics.csv [--camera VehicleFrameworkVR_camera.csv] [--log VehicleFrameworkVR.log]

Only the Python standard library is needed. Thresholds are proposals; they live at the top of this file.
"""

import argparse
import csv
import math
import re
import statistics
import sys

# --- Thresholds (proposals, tune once real data comes in) ---
GATE_SPEED_RANGE_KMH = (55.0, 65.0)
GATE_MIN_SECONDS = 3.0
GATE_P95_CM = 3.0  # 95% of frames within 3 cm of the seat anchor
GATE_STD_CM = 1.5  # and the error does not jump around
REST_MAX_SPEED_KMH = 0.5
REST_MIN_SECONDS = 2.0
REST_MAX_VEL_Z_STD = 0.02  # m/s: below this the car is not vibrating
LOW_SPEED_KMH = 10.0
LOW_SPEED_MIN_STEER_DEG = 15.0
LOW_SPEED_MIN_YAW_RATE = 0.1  # rad/s: it actually turns
COST_BUDGET_MS = 0.2
WHEEL_COLUMNS = ("spin", "slip", "slip_deg", "fz", "fx", "fy", "grip")


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        for key, value in row.items():
            try:
                row[key] = float(value)
            except (TypeError, ValueError):
                pass
    return rows


def percentile(values, p):
    if not values:
        return float("nan")
    ordered = sorted(values)
    k = (len(ordered) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return ordered[lo] if lo == hi else ordered[lo] + (ordered[hi] - ordered[lo]) * (k - lo)


def runs(rows, predicate):
    """Consecutive stretches of rows that satisfy the predicate."""
    current = []
    for row in rows:
        if predicate(row):
            current.append(row)
        elif current:
            yield current
            current = []
    if current:
        yield current


def duration(rows):
    return sum(r["dt"] for r in rows)


def wheel_count(rows):
    return sum(1 for key in rows[0] if re.fullmatch(r"w\d+_down", key)) if rows else 0


def verdict(ok):
    return "PASA" if ok else "NO PASA"


def first_person(row):
    """First-person rows only. Newer logs say it in the `view` column (0 first person, 1 chase camera); older ones do not, and there the chase camera
    shows as a head-to-seat distance of metres (the camera is ~11.5 m behind the seat)."""
    if "view" in row:
        return row["view"] == 0
    return row["head_seat_cm"] < 500.0


def check_gate(camera):
    lo, hi = GATE_SPEED_RANGE_KMH
    first = [r for r in camera if first_person(r)]
    window = [r for r in first if lo <= r["speed_kmh"] <= hi]
    seconds = duration(window)
    if seconds < GATE_MIN_SECONDS:
        top = max((r["speed_kmh"] for r in first), default=0.0)
        return ("Puerta: cabeza-asiento estable a 60 km/h", None,
                f"solo {seconds:.1f} s en primera persona entre {lo:.0f} y {hi:.0f} km/h (hacen falta {GATE_MIN_SECONDS:.0f} s; velocidad máx en primera persona {top:.0f} km/h)")
    errors = [r["anchor_error_cm"] for r in window]
    p95 = percentile(errors, 0.95)
    std = statistics.pstdev(errors)
    yaw = [abs(r["room_yaw_error_deg"]) for r in window]
    ok = p95 <= GATE_P95_CM and std <= GATE_STD_CM
    frozen = sum(1 for r in window if r["seat_step_cm"] == 0.0)
    detail = (f"{seconds:.1f} s a {lo:.0f}-{hi:.0f} km/h en primera persona ({frozen} fotogramas con el asiento parado): error de anclaje medio {statistics.mean(errors):.2f} cm, "
              f"p95 {p95:.2f} cm, desviación {std:.2f} cm, máx {max(errors):.2f} cm; error de rumbo de la sala p95 {percentile(yaw, 0.95):.2f}°")
    return ("Puerta: cabeza-asiento estable a 60 km/h", ok, detail)


def check_rest(physics):
    stretches = [s for s in runs(physics, lambda r: abs(r["speed_kmh"]) < REST_MAX_SPEED_KMH and r["throttle"] < 0.05
                                  and r["wheels_down"] >= 3) if duration(s) >= REST_MIN_SECONDS]
    if not stretches:
        return ("Parado no vibra", None, f"no hay tramos parado de al menos {REST_MIN_SECONDS:.0f} s")
    # Skip the first second of each stretch: the car is still settling.
    samples = []
    for s in stretches:
        t = 0.0
        for r in s:
            t += r["dt"]
            if t > 1.0:
                samples.append(r["vel_z"])
    if len(samples) < 10:
        return ("Parado no vibra", None, "tramos parado demasiado cortos tras descartar el asentamiento")
    std = statistics.pstdev(samples)
    ok = std <= REST_MAX_VEL_Z_STD
    return ("Parado no vibra", ok, f"{len(stretches)} tramos, {duration([r for s in stretches for r in s]):.1f} s; "
                                   f"desviación de la velocidad vertical {std * 100:.2f} cm/s (límite {REST_MAX_VEL_Z_STD * 100:.0f} cm/s)")


def check_hold(physics):
    n = wheel_count(physics)
    # Rows with every wheel held, skipping the first second of each stretch (the car is still coming to rest).
    held = []
    for stretch in runs(physics, lambda r: all(r.get(f"w{i}_hold", 0) == 1 for i in range(n))):
        t = 0.0
        for r in stretch:
            t += r["dt"]
            if t > 1.0:
                held.append(r)
    if duration(held) < 1.0:
        return ("Parado en cuesta no resbala", None, "no hay tramos con las ruedas sujetas (freno de estacionamiento automático)")
    max_speed = max(abs(r["speed_kmh"]) for r in held)
    tilt = max(math.degrees(math.acos(max(-1.0, min(1.0, r["up_z"])))) for r in held)
    ok = max_speed < REST_MAX_SPEED_KMH
    return ("Parado en cuesta no resbala", ok, f"{duration(held):.1f} s sujeto, inclinación máx {tilt:.1f}°, velocidad máx {max_speed:.2f} km/h"
                                               + ("" if tilt > 3.0 else " (aviso: sin cuesta en los datos; prueba en una pendiente)"))


def check_acceleration(physics):
    best = None
    for s in runs(physics, lambda r: r["throttle"] > 0.9 and r["gear"] > 0):
        t = 0.0
        started = False
        for r in s:
            if not started and r["speed_kmh"] < 3.0:
                started = True
                t = 0.0
            elif started:
                t += r["dt"]
                if r["speed_kmh"] >= 100.0:
                    best = t if best is None else min(best, t)
                    break
    if best is None:
        top = max((r["speed_kmh"] for r in physics), default=0.0)
        return ("Acelera (0-100 km/h)", None, f"no hay una aceleración a fondo desde parado hasta 100 km/h; velocidad máx {top:.0f} km/h")
    ok = 7.0 <= best <= 14.0
    return ("Acelera (0-100 km/h)", ok, f"{best:.1f} s (objetivo unos 10 s)")


def check_braking(physics):
    best = None
    for s in runs(physics, lambda r: r["brake"] > 0.9 and r["gear"] > 0):
        if s[0]["speed_kmh"] < 95.0:
            continue
        distance = 0.0
        for r in s:
            distance += r["speed_kmh"] / 3.6 * r["dt"]
            if r["speed_kmh"] < 1.0:
                best = distance if best is None else min(best, distance)
                break
    if best is None:
        return ("Frena (100-0 km/h)", None, "no hay una frenada a fondo desde 95 km/h o más hasta parar")
    ok = 35.0 <= best <= 55.0
    return ("Frena (100-0 km/h)", ok, f"{best:.1f} m (objetivo unos 45 m)")


def check_low_speed_turn(physics):
    turning = [r for r in physics if 1.0 < abs(r["speed_kmh"]) < LOW_SPEED_KMH and abs(r["steer_deg"]) > LOW_SPEED_MIN_STEER_DEG]
    if duration(turning) < 1.0:
        return ("Gira por debajo de 10 km/h", None, "no hay tramos lentos con el volante girado")
    yaw = max(abs(r["yaw_rate"]) for r in turning)
    radius = statistics.median(abs(r["speed_kmh"]) / 3.6 / max(abs(r["yaw_rate"]), 1e-3) for r in turning)
    ok = yaw >= LOW_SPEED_MIN_YAW_RATE
    return ("Gira por debajo de 10 km/h", ok, f"giro máx {yaw:.2f} rad/s, radio de giro mediano {radius:.1f} m")


def check_wheel_columns(physics):
    n = wheel_count(physics)
    missing = [f"w{i}_{c}" for i in range(n) for c in WHEEL_COLUMNS if f"w{i}_{c}" not in physics[0]]
    moving = [r for r in physics if abs(r["speed_kmh"]) > 5.0]
    alive = moving and any(abs(r.get("w0_spin", 0)) > 0.0 for r in moving)
    ok = n >= 4 and not missing and bool(alive)
    detail = f"{n} ruedas, {len(physics)} pasos" + (f"; faltan {', '.join(missing)}" if missing else "")
    if not alive:
        detail += "; sin datos de rueda en marcha"
    return ("Telemetría por rueda (giro, deslizamiento, agarre, fuerzas)", ok, detail)


def check_cost(log_path):
    pattern = re.compile(r"Cost per frame: ([0-9.]+) ms total \(game thread ([0-9.]+) ms, physics step ([0-9.]+) ms\)")
    totals = []
    with open(log_path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = pattern.search(line)
            if m:
                totals.append(float(m.group(1)))
    if not totals:
        return ("Coste por fotograma < 0,2 ms", None, "el log no tiene líneas 'Cost per frame' (bCostReport = true y conducir un rato)")
    ok = max(totals) < COST_BUDGET_MS
    return ("Coste por fotograma < 0,2 ms", ok, f"{len(totals)} informes de 5 s: mediana {statistics.median(totals):.3f} ms, máx {max(totals):.3f} ms")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--physics", required=True, help="VehicleFrameworkVR_physics.csv")
    parser.add_argument("--camera", help="VehicleFrameworkVR_camera.csv")
    parser.add_argument("--log", help="VehicleFrameworkVR.log")
    args = parser.parse_args()

    physics = read_csv(args.physics)
    if not physics:
        print("El CSV de física está vacío.")
        return 2
    results = []
    if args.camera:
        camera = read_csv(args.camera)
        results.append(check_gate(camera) if camera else ("Puerta: cabeza-asiento estable a 60 km/h", None, "CSV de cámara vacío"))
    else:
        results.append(("Puerta: cabeza-asiento estable a 60 km/h", None, "falta --camera"))
    results += [check_acceleration(physics), check_braking(physics), check_low_speed_turn(physics), check_hold(physics), check_rest(physics)]
    results.append(check_cost(args.log) if args.log else ("Coste por fotograma < 0,2 ms", None, "falta --log"))
    results.append(check_wheel_columns(physics))

    print("# Fase 1: criterios de salida\n")
    print("| Criterio | Resultado | Datos |")
    print("|---|---|---|")
    for name, ok, detail in results:
        status = "SIN DATOS" if ok is None else verdict(ok)
        print(f"| {name} | {status} | {detail} |")
    failed = any(ok is False for _, ok, _ in results)
    missing = any(ok is None for _, ok, _ in results)
    print()
    print("Resultado: " + ("hay criterios que no pasan." if failed else "faltan datos para algún criterio." if missing else "todos los criterios pasan."))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
