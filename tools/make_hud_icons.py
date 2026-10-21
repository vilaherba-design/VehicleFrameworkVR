#!/usr/bin/env python3
"""Build the driving HUD icons: white glyphs on a transparent background (the HUD tints them), as uncompressed 32-bit DDS.

  action.dds  - a lightning bolt (the vehicle's action: nitro, rockets...)
  weapon.dds  - a crosshair (the vehicle's weapon)

No third-party modules: the shapes are rasterized here with 4x4 supersampling.
Usage: make_hud_icons.py <output folder>   (data/mod/Textures/VehicleFrameworkVR/Hud)
"""
import math
import os
import struct
import sys

SIZE = 64
SAMPLES = 4


def inside_polygon(x, y, poly):
    hit = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            hit = not hit
        j = i
    return hit


def bolt(x, y):
    # unit square, y down
    poly = [(0.58, 0.04), (0.20, 0.56), (0.46, 0.56), (0.36, 0.96), (0.80, 0.40), (0.53, 0.40), (0.66, 0.04)]
    return inside_polygon(x, y, poly)


def crosshair(x, y):
    dx, dy = x - 0.5, y - 0.5
    r = math.hypot(dx, dy)
    ring = 0.27 <= r <= 0.35
    dot = r <= 0.06
    arm = 0.035
    ticks = (abs(dx) <= arm and 0.14 <= abs(dy) <= 0.47) or (abs(dy) <= arm and 0.14 <= abs(dx) <= 0.47)
    return ring or dot or ticks


def render(shape):
    pixels = bytearray()
    for py in range(SIZE):
        for px in range(SIZE):
            hits = 0
            for sy in range(SAMPLES):
                for sx in range(SAMPLES):
                    if shape((px + (sx + 0.5) / SAMPLES) / SIZE, (py + (sy + 0.5) / SAMPLES) / SIZE):
                        hits += 1
            alpha = round(255 * hits / (SAMPLES * SAMPLES))
            pixels += bytes((255, 255, 255, alpha))  # B, G, R, A
    return bytes(pixels)


def write_dds(path, pixels):
    pixel_format = struct.pack("<II4sIIIII", 32, 0x41, b"\0\0\0\0", 32, 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000)
    header = struct.pack("<IIIIIII", 124, 0x100F, SIZE, SIZE, SIZE * 4, 0, 0) + b"\0" * 44 + pixel_format + struct.pack("<IIIII", 0x1000, 0, 0, 0, 0)
    assert len(header) == 124
    with open(path, "wb") as f:
        f.write(b"DDS " + header + pixels)


def main(folder):
    os.makedirs(folder, exist_ok=True)
    for name, shape in (("action.dds", bolt), ("weapon.dds", crosshair)):
        path = os.path.join(folder, name)
        write_dds(path, render(shape))
        print("wrote", path)


if __name__ == "__main__":
    main(sys.argv[1])
