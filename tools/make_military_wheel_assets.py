#!/usr/bin/env python3
"""Writes the material and the textures of the military wheels (the "military" wheel set of the Army Wagon).

The wheel (tools/nif/wheel_military.py) is mapped by a planar projection along the axle onto one painted disc: dusty black rubber with raised
lettering on the sidewall, the rusty olive-brown steel rim and hub. The normal map comes from a height map (letters, tread grain, rust pits);
the specular map follows the game's convention (R = specular, G = glossiness). The material is BeadlockWheel.BGSM (the vanilla Sedan02.BGSM
with its texture paths replaced) with the paths pointed at the MilitaryWheel textures.

Usage: make_military_wheel_assets.py <BeadlockWheel.BGSM> <mod data dir> <texconv.exe> [png dir to keep the sources]
Writes: <data>/Materials/VehicleFrameworkVR/Vehicles/MilitaryWheel.BGSM and <data>/Textures/VehicleFrameworkVR/Vehicles/MilitaryWheel_{d,n,s}.dds
"""
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont
from scipy import ndimage

import make_wheel_assets as mwa

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nif'))
import wheel_military as wm  # noqa: E402

TEXTURE_NAME = 'MilitaryWheel'
SIZE = 1024
TEXTS = ('MILITARY TACTICAL  \u2022  9.00-16 NDCC  \u2022  ', 'MUD TERRAIN  \u2022  TUBELESS  \u2022  LOAD RANGE D  \u2022  ')


def uv(material):                    # the palette interface of the other wheels: not used, this wheel has real UVs (wheel_military.uv)
    return (0.5, 0.5)


def noise(rng, sigma, size=SIZE):
    n = ndimage.gaussian_filter(rng.standard_normal((size, size)), sigma, mode='wrap')
    return n / (n.std() + 1e-9)


def design_radius():
    """Design radius (metres) of every texel, for the outer face (the z of the outer sidewall in the UV mapping)."""
    y, x = np.mgrid[0:SIZE, 0:SIZE]
    du, dv = (x + 0.5) / SIZE - 0.5, (y + 0.5) / SIZE - 0.5
    f = wm.UV_R / wm.RT * (1.0 - 0.06 * (0.157 + wm.HALF_WIDTH) / (2 * wm.HALF_WIDTH))
    return np.hypot(du, dv) / f


def lettering():
    """Height map of the sidewall lettering: two texts around the lettering band, one on each half of the tyre, reading clockwise."""
    img = Image.new('L', (SIZE, SIZE), 0)
    f = wm.UV_R / wm.RT * 0.95
    r_mid = (wm.LETTER_R[0] + wm.LETTER_R[1]) / 2 * f * SIZE
    font = ImageFont.truetype(os.path.join(os.environ.get('WINDIR', 'C:/Windows'), 'Fonts', 'arialbd.ttf'), 30)
    for k, text in enumerate(TEXTS):
        widths = [font.getlength(ch) for ch in text]
        total = sum(widths)
        arc = math.pi * 0.92                                          # each text fills most of a half turn
        scale = arc * r_mid / total
        a = -math.pi / 2 - arc / 2 + k * math.pi                     # start angle (image coordinates, y down): the top half, then the bottom
        for ch, w in zip(text, widths):
            step = w * scale / r_mid
            mid = a + step / 2
            tile = Image.new('L', (64, 64), 0)
            ImageDraw.Draw(tile).text((32, 32), ch, fill=255, font=font, anchor='mm')
            tile = tile.resize((int(64 * scale), int(64 * scale)), Image.LANCZOS).rotate(-math.degrees(mid) - 90, resample=Image.BICUBIC)
            cx, cy = SIZE / 2 + r_mid * math.cos(mid), SIZE / 2 + r_mid * math.sin(mid)
            img.paste(tile, (int(cx - tile.width / 2), int(cy - tile.height / 2)), tile)
            a += step
    return np.asarray(img, np.float64) / 255


def paint():
    rng = np.random.default_rng(7)
    r = design_radius()
    rubber = r > wm.RIM_R
    letters = lettering() * ((r > wm.LETTER_R[0] - 0.02) & (r < wm.LETTER_R[1] + 0.02))
    n1, n2, n3 = noise(rng, 1.2), noise(rng, 5), noise(rng, 18)
    # dust: brown, heavier toward the tread and in blotches
    dust = np.clip(-0.1 + 0.5 * np.clip((r - 0.36) / 0.1, 0, 1) + 0.3 * n3 + 0.15 * n2, 0, 1) * rubber * 0.35       # a little dust: the rubber stays charcoal
    base_rubber = np.stack([0.018, 0.017, 0.016]) * (1 + 0.15 * n1[..., None])
    dust_col = np.array([0.16, 0.115, 0.07])
    col = base_rubber * (1 - dust[..., None]) + dust_col * dust[..., None]
    lt = letters[..., None] * 0.3                                       # raised and a little lighter, not painted on
    col = col * (1 - lt) + (np.array([0.04, 0.037, 0.034]) + 0.5 * dust_col * dust[..., None]) * lt
    # steel: olive-brown paint with rust blotches and dark grime toward the hub
    rust = np.clip(0.6 + 0.5 * n3 + 0.3 * n2 + 0.15 * n1, 0, 1)          # more rust on the bronze rim (the sheet)
    paint_col = np.array([0.045, 0.042, 0.037])                  # dark steel, with rusty bronze over it (the sheet's rims)
    rust_col = np.array([0.17, 0.07, 0.022])
    steel = paint_col * (1 - rust[..., None]) + rust_col * rust[..., None]
    steel *= (0.85 + 0.15 * np.clip(r / 0.2, 0, 1))[..., None] + 0.05 * n1[..., None]
    col = np.where(rubber[..., None], col, steel)
    # caked mud in the tread and on the shoulders (outer ring), in clumps; a little splashed on the rim's lower edge
    caked = np.clip((0.55 * n2 + 0.45 * n3 + (r - 0.3) * 3 - 0.95) * 2.5, 0, 1) * rubber * 0.4         # mud blotches low on the flank only
    caked = np.maximum(caked, np.clip((0.5 * n3 + 0.3 * n2 - 0.75) * 3, 0, 1) * (r > 0.2) * (r < 0.26))
    mud_c = np.array([0.15, 0.11, 0.07]) * (0.85 + 0.2 * n1[..., None])
    col = col * (1 - caked[..., None]) + mud_c * caked[..., None]
    # the rim's ten holes (wheel_military.py): dark, where the geometry has them
    y, x = np.mgrid[0:SIZE, 0:SIZE]
    fz = wm.UV_R / wm.RT * (1.0 - 0.06 * (0.0855 + wm.HALF_WIDTH) / (2 * wm.HALF_WIDTH))
    hole = np.zeros((SIZE, SIZE), bool)
    for i in range(10):
        a = i / 10 * 2 * math.pi
        cu, cv = (0.5 + math.cos(a) * wm.HOLE_RING * fz) * SIZE, (0.5 - math.sin(a) * wm.HOLE_RING * fz) * SIZE
        hole |= np.hypot(x + 0.5 - cu, y + 0.5 - cv) < wm.HOLE_R * fz * SIZE * 1.05
    col[hole] = np.array([0.012, 0.011, 0.01])
    # the groove texel (wheel_military.GROOVE_UV): packed brown mud between the blocks
    gx0, gy0 = int(wm.GROOVE_UV[0] * SIZE), int(wm.GROOVE_UV[1] * SIZE)
    col[max(0, gy0 - 30):gy0 + 30, max(0, gx0 - 30):gx0 + 30] = np.array([0.12, 0.085, 0.05])
    caked[max(0, gy0 - 30):gy0 + 30, max(0, gx0 - 30):gx0 + 30] = 1.0
    # specular R, glossiness G
    spec = np.where(rubber, 0.08 + 0.05 * letters, 0.3 - 0.18 * rust) * (1 - 0.6 * caked)
    gloss = np.where(rubber, 0.22 - 0.1 * dust, 0.42 - 0.25 * rust) * (1 - 0.5 * caked)
    # height: lettering, rubber grain, rust pits
    height = 0.5 * caked + np.where(rubber, 0.6 * letters + 0.06 * n1, -0.25 * np.clip(rust - 0.6, 0, 1) + 0.05 * n1)
    height = ndimage.gaussian_filter(height, 0.7)
    gx = ndimage.sobel(height, 1) * 1.6
    gy = ndimage.sobel(height, 0) * 1.6
    nz = 1.0 / np.sqrt(gx * gx + gy * gy + 1.0)
    nrm = np.stack([-gx * nz, gy * nz, nz], -1)            # DirectX: green points down the image for a raised edge below
    return col, spec, gloss, nrm


def paint_atlas(size=2048):
    """the atlas layout of wheel_military.build(): the old painted disc resampled onto the outer and inner discs, the tread strip painted
    as charcoal rubber with packed mud in the grooves (from the baked AO, WHEEL_AO), the AO darkening everything; dark texel black"""
    col_d, spec_d, gloss_d, nrm_d = paint()                       # the disc, old layout (SIZE x SIZE)
    y, x = np.mgrid[0:size, 0:size]
    u, v = (x + 0.5) / size, (y + 0.5) / size
    fA = wm.DISC_UV / wm.DISC_R
    fold = wm.UV_R / wm.RT * (1.0 - 0.06 * (0.157 + wm.HALF_WIDTH) / (2 * wm.HALF_WIDTH))
    col = np.zeros((size, size, 3))
    spec, gloss = np.zeros((size, size)), np.zeros((size, size))
    nrm = np.zeros((size, size, 3))
    nrm[..., 2] = 1.0
    for cx, sgn in ((0.25, 1.0), (0.75, -1.0)):
        sel = (np.abs(u - cx) < 0.25) & (v < 0.5)
        dx, dy = (u[sel] - cx) / fA * sgn, -(v[sel] - 0.25) / fA
        uo, vo = (0.5 + dx * fold) * SIZE - 0.5, (0.5 - dy * fold) * SIZE - 0.5
        coords = [vo, uo]
        for c_ in range(3):
            col[..., c_][sel] = ndimage.map_coordinates(col_d[..., c_], coords, order=1, mode="nearest")
            nrm[..., c_][sel] = ndimage.map_coordinates(nrm_d[..., c_], coords, order=1, mode="nearest")
        spec[sel] = ndimage.map_coordinates(spec_d, coords, order=1, mode="nearest")
        gloss[sel] = ndimage.map_coordinates(gloss_d, coords, order=1, mode="nearest")
        if sgn < 0:
            nrm[..., 0][sel] *= -1
    ao = np.ones((size, size))
    aop = os.environ.get("WHEEL_AO")
    if aop and os.path.exists(aop):
        ao = np.asarray(Image.open(aop).convert("L").resize((size, size)), np.float64) / 255
    rng = np.random.default_rng(21)
    def nz(sig):
        n_ = ndimage.gaussian_filter(rng.standard_normal((size, size)), sig, mode="wrap")
        return n_ / (n_.std() + 1e-9)
    n1, n2, n3 = nz(1.2), nz(5), nz(14)
    strip = v >= 0.5
    rub = np.array([0.02, 0.019, 0.018]) * (1 + 0.15 * n1[..., None])
    groove = np.clip((0.93 - ao) * 6 + 0.2 * n2, 0, 1) * strip                  # packed mud where the blocks shade the carcass
    dust = np.clip(0.15 + 0.25 * n3, 0, 1) * strip * 0.4
    mud_c = np.array([0.13, 0.095, 0.06]) * (0.85 + 0.25 * n1[..., None])
    sc_ = rub * (1 - dust[..., None]) + np.array([0.16, 0.12, 0.08]) * dust[..., None]
    sc_ = sc_ * (1 - groove[..., None]) + mud_c * groove[..., None]
    col[strip] = sc_[strip]
    spec[strip] = (0.07 * (1 - groove) + 0.02 * groove)[strip]
    gloss[strip] = (0.22 * (1 - groove) + 0.08 * groove)[strip]
    hs = (0.08 * n1 + 0.6 * groove * np.clip(nz(0.9) - 0.3, 0, 1)) * strip       # grain, and pebbles in the mud
    gx, gy = ndimage.sobel(hs, 1) * 1.5, ndimage.sobel(hs, 0) * 1.5
    nzz = 1.0 / np.sqrt(gx * gx + gy * gy + 1.0)
    nrm[strip] = np.stack([-gx * nzz, gy * nzz, nzz], -1)[strip]
    col = col * (0.45 + 0.55 * ao)[..., None]
    # the normals baked from the bevelled high wheel (bake_rueda.py, <ao>_normal.png) under the painted detail
    if aop and os.path.exists(aop[:-4] + "_normal.png"):
        nb = np.asarray(Image.open(aop[:-4] + "_normal.png").convert("RGB").resize((size, size)), np.float64) / 255 * 2 - 1
        nrm = np.dstack([nb[..., 0] + nrm[..., 0], nb[..., 1] + nrm[..., 1], nb[..., 2]])
        nrm /= np.linalg.norm(nrm, axis=2, keepdims=True) + 1e-9
    dx0, dy0 = int(wm.DARK_UV[0] * size), int(wm.DARK_UV[1] * size)
    col[dy0 - 6:dy0 + 6, dx0 - 6:dx0 + 6] = 0.01
    return col, spec, gloss, nrm


def main(bgsm_path, data_dir, texconv, keep=None):
    col, spec, gloss, nrm = paint_atlas() if getattr(wm, "ATLAS", False) else paint()
    tex_dir = os.path.join(data_dir, 'Textures', *mwa.TEXTURE_DIR.split('/'))
    mat_dir = os.path.join(data_dir, 'Materials', *mwa.TEXTURE_DIR.split('/'))
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(mat_dir, exist_ok=True)
    src = keep or tempfile.mkdtemp()
    os.makedirs(src, exist_ok=True)
    to8 = lambda a: (np.clip(a, 0, 1) * 255 + 0.5).astype(np.uint8)
    srgb = np.where(col <= 0.0031308, col * 12.92, 1.055 * np.power(np.clip(col, 0, 1), 1 / 2.4) - 0.055)
    pngs = {'_d': to8(srgb), '_n': to8(nrm * 0.5 + 0.5), '_s': to8(np.stack([spec, gloss, np.zeros_like(spec)], -1))}
    for suffix, fmt in (('_d', 'BC7_UNORM'), ('_n', 'BC5_UNORM'), ('_s', 'BC5_UNORM')):
        p = os.path.join(src, TEXTURE_NAME + suffix + '.png')
        Image.fromarray(pngs[suffix]).save(p)
        subprocess.run([texconv, '-nologo', '-y', '--ignore-srgb', '-m', '0', '-f', fmt, '-o', tex_dir, p], check=True, capture_output=True)
    old = mwa.TEXTURE_DIR + '/' + mwa.TEXTURE_NAME
    new = mwa.TEXTURE_DIR + '/' + TEXTURE_NAME
    data = mwa.patch_bgsm(open(bgsm_path, 'rb').read(), {old + '_d.dds': new + '_d.dds', old + '_n.dds': new + '_n.dds', old + '_s.dds': new + '_s.dds'})
    with open(os.path.join(mat_dir, TEXTURE_NAME + '.BGSM'), 'wb') as f:
        f.write(data)
    print('wrote', mat_dir, 'and', tex_dir)


if __name__ == '__main__':
    if len(sys.argv) not in (4, 5):
        sys.exit(__doc__)
    main(*sys.argv[1:])
