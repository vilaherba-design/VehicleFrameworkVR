#!/usr/bin/env python3
"""Writes the material and the textures of the classic wheels (the "classic" wheel set of the coupe: white wall, red rim, chrome cap).

Same scheme as make_wheel_assets.py (one small palette texture, a flat cell per material); the material is BeadlockWheel.BGSM (itself the vanilla
Sedan02.BGSM with its texture paths replaced) with the paths pointed at the ClassicWheel textures.

Usage: make_classic_wheel_assets.py <BeadlockWheel.BGSM> <mod data dir>
Writes: <data>/Materials/VehicleFrameworkVR/Vehicles/ClassicWheel.BGSM and <data>/Textures/VehicleFrameworkVR/Vehicles/ClassicWheel_{d,n,s}.dds
"""
import os
import sys

import numpy as np

import make_wheel_assets as mwa

# material -> (cell index, diffuse sRGB, specular/env mask R, smoothness G)
MATERIALS = {
    'rubber': (0, (30, 29, 28), 20, 40),
    'white': (1, (226, 222, 210), 40, 70),
    'red': (2, (150, 22, 24), 120, 150),
    'chrome': (3, (205, 205, 210), 235, 215),
    'dark': (4, (30, 30, 32), 60, 80),
}
TEXTURE_NAME = 'ClassicWheel'


def cell_uv(material):
    idx = MATERIALS[material][0]
    return ((idx % mwa.COLS + 0.5) / mwa.COLS, (idx // mwa.COLS + 0.5) / mwa.ROWS)


def paint(channel_fn):
    img = np.zeros((mwa.HEIGHT, mwa.WIDTH, 3), np.uint8)
    img[:] = channel_fn(None)
    for name, (idx, rgb, spec, smooth) in MATERIALS.items():
        x, y = (idx % mwa.COLS) * mwa.CELL, (idx // mwa.COLS) * mwa.CELL
        img[y:y + mwa.CELL, x:x + mwa.CELL] = channel_fn((rgb, spec, smooth))
    return img


def main(bgsm_path, data_dir):
    src = open(bgsm_path, 'rb').read()
    tex_dir = os.path.join(data_dir, 'Textures', *mwa.TEXTURE_DIR.split('/'))
    mat_dir = os.path.join(data_dir, 'Materials', *mwa.TEXTURE_DIR.split('/'))
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(mat_dir, exist_ok=True)
    mwa.write_dds(os.path.join(tex_dir, TEXTURE_NAME + '_d.dds'), paint(lambda m: (0, 0, 0) if m is None else m[0]), 'bc1')
    mwa.write_dds(os.path.join(tex_dir, TEXTURE_NAME + '_n.dds'), paint(lambda m: (128, 128, 255)), 'bc5')
    mwa.write_dds(os.path.join(tex_dir, TEXTURE_NAME + '_s.dds'), paint(lambda m: (0, 0, 0) if m is None else (m[1], m[2], 0)), 'bc5')
    old = mwa.TEXTURE_DIR + '/' + mwa.TEXTURE_NAME
    new = mwa.TEXTURE_DIR + '/' + TEXTURE_NAME
    data = mwa.patch_bgsm(src, {old + '_d.dds': new + '_d.dds', old + '_n.dds': new + '_n.dds', old + '_s.dds': new + '_s.dds'})
    with open(os.path.join(mat_dir, TEXTURE_NAME + '.BGSM'), 'wb') as f:
        f.write(data)
    print('wrote', mat_dir, 'and', tex_dir)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
