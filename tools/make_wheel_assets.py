#!/usr/bin/env python3
"""Writes the material and the textures of the beadlock wheels (the "beadlock" wheel set of the sedan).

One small palette texture serves the whole wheel: every part of the wheel mesh (tyre, mud lugs, bronze rim, dark rim parts, steel bolts) has its UV on a
flat-coloured cell, so each wheel is one shape with one material. The material is the vanilla Sedan02.BGSM with its three texture paths replaced (the
environment cubemap and every shader setting stay as the game's own sedan has them), so Sedan02.BGSM has to be extracted from Fallout4 - Materials.ba2.

Usage: make_wheel_assets.py <Sedan02.BGSM> <mod data dir>
Writes: <data>/Materials/VehicleFrameworkVR/Vehicles/BeadlockWheel.BGSM and <data>/Textures/VehicleFrameworkVR/Vehicles/BeadlockWheel_{d,n,s}.dds
"""
import os
import struct
import sys

import numpy as np

# material -> (cell index, diffuse sRGB, specular/env mask R, smoothness G)
MATERIALS = {
    'rubber': (0, (46, 43, 40), 22, 38),
    'mud': (1, (84, 74, 62), 16, 28),
    'bronze': (2, (176, 143, 94), 190, 150),
    'dark': (3, (44, 44, 46), 95, 110),
    'steel': (4, (192, 189, 180), 215, 175),
}
COLS, ROWS, CELL = 4, 2, 32
WIDTH, HEIGHT = COLS * CELL, ROWS * CELL

TEXTURE_DIR = 'VehicleFrameworkVR/Vehicles'
TEXTURE_NAME = 'BeadlockWheel'


def cell_uv(material):
    """UV of the centre of the material's cell (v grows downwards in DDS files)."""
    idx = MATERIALS[material][0]
    return ((idx % COLS + 0.5) / COLS, (idx // COLS + 0.5) / ROWS)


def paint(channel_fn):
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    img[:] = channel_fn(None)
    for name, (idx, rgb, spec, smooth) in MATERIALS.items():
        x, y = (idx % COLS) * CELL, (idx // COLS) * CELL
        img[y:y + CELL, x:x + CELL] = channel_fn((rgb, spec, smooth))
    return img


def mip_chain(img):
    chain = [img.astype(np.float64)]
    while chain[-1].shape[0] > 1 or chain[-1].shape[1] > 1:
        a = chain[-1]
        h, w = max(a.shape[0] // 2, 1), max(a.shape[1] // 2, 1)
        out = np.zeros((h, w, a.shape[2]))
        for y in range(h):
            for x in range(w):
                out[y, x] = a[y * 2:y * 2 + 2, x * 2:x * 2 + 2].reshape(-1, a.shape[2]).mean(0)
        chain.append(out)
    return [np.clip(np.round(c), 0, 255).astype(np.uint8) for c in chain]


def blocks_of(img):
    """4x4 blocks (row-major) of an image, edge-padded to a multiple of 4."""
    h, w = img.shape[:2]
    ph, pw = (-h) % 4, (-w) % 4
    if ph or pw:
        img = np.pad(img, ((0, ph), (0, pw), (0, 0)), mode='edge')
    for by in range(0, img.shape[0], 4):
        for bx in range(0, img.shape[1], 4):
            yield img[by:by + 4, bx:bx + 4].reshape(16, -1)


def to565(c):
    return ((int(c[0]) >> 3) << 11) | ((int(c[1]) >> 2) << 5) | (int(c[2]) >> 3)


def from565(v):
    r, g, b = (v >> 11) & 31, (v >> 5) & 63, v & 31
    return np.array([(r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)], float)


def bc1_block(px):
    luma = px @ np.array([0.3, 0.59, 0.11])
    c0, c1 = to565(px[int(np.argmax(luma))]), to565(px[int(np.argmin(luma))])
    if c0 == c1:
        return struct.pack('<HHI', c0, c1, 0)
    if c0 < c1:
        c0, c1 = c1, c0
    p0, p1 = from565(c0), from565(c1)
    pal = np.stack([p0, p1, (2 * p0 + p1) / 3, (p0 + 2 * p1) / 3])
    idx = np.argmin(((px[:, None, :].astype(float) - pal[None]) ** 2).sum(-1), axis=1)
    bits = 0
    for i, v in enumerate(idx):
        bits |= int(v) << (2 * i)
    return struct.pack('<HHI', c0, c1, bits)


def bc4_block(vals):
    vals = vals.astype(int)
    hi, lo = int(vals.max()), int(vals.min())
    if hi == lo:
        return bytes([hi, lo]) + bytes(6)
    pal = [hi, lo] + [((7 - k) * hi + k * lo) // 7 for k in range(1, 7)]
    idx = [int(np.argmin([abs(int(v) - p) for p in pal])) for v in vals]
    bits = 0
    for i, v in enumerate(idx):
        bits |= v << (3 * i)
    return bytes([hi, lo]) + bits.to_bytes(6, 'little')


def encode(img, fmt):
    out = bytearray()
    for px in blocks_of(img):
        if fmt == 'bc1':
            out += bc1_block(px)
        else:   # bc5: red then green
            out += bc4_block(px[:, 0]) + bc4_block(px[:, 1])
    return bytes(out)


def write_dds(path, img, fmt):
    chain = mip_chain(img)
    dxgi = {'bc1': 71, 'bc5': 83}[fmt]
    block = 8 if fmt == 'bc1' else 16
    h, w = img.shape[:2]
    linear = max(1, (w + 3) // 4) * max(1, (h + 3) // 4) * block
    hdr = b'DDS ' + struct.pack('<7I', 124, 0xA1007, h, w, linear, 0, len(chain)) + bytes(44)
    hdr += struct.pack('<2I4s5I', 32, 4, b'DX10', 0, 0, 0, 0, 0) + struct.pack('<5I', 0x401008, 0, 0, 0, 0)
    hdr += struct.pack('<5I', dxgi, 3, 0, 1, 0)
    with open(path, 'wb') as f:
        f.write(hdr + b''.join(encode(m, fmt) for m in chain))


def patch_bgsm(src, textures):
    """Replace the length-prefixed texture path strings of a BGSM (old path -> new path)."""
    data = bytes(src)
    for old, new in textures.items():
        o, n = old.encode() + b'\0', new.encode() + b'\0'
        a, b = struct.pack('<I', len(o)) + o, struct.pack('<I', len(n)) + n
        assert data.count(a) == 1, 'texture path not found in the material: ' + old
        data = data.replace(a, b)
    return data


def main(bgsm_path, data_dir):
    src = open(bgsm_path, 'rb').read()
    assert src[:4] == b'BGSM' and struct.unpack_from('<I', src, 4)[0] == 2, 'expected a version 2 BGSM'
    tex_dir = os.path.join(data_dir, 'Textures', *TEXTURE_DIR.split('/'))
    mat_dir = os.path.join(data_dir, 'Materials', *TEXTURE_DIR.split('/'))
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(mat_dir, exist_ok=True)
    write_dds(os.path.join(tex_dir, TEXTURE_NAME + '_d.dds'), paint(lambda m: (0, 0, 0) if m is None else m[0]), 'bc1')
    write_dds(os.path.join(tex_dir, TEXTURE_NAME + '_n.dds'), paint(lambda m: (128, 128, 255)), 'bc5')
    write_dds(os.path.join(tex_dir, TEXTURE_NAME + '_s.dds'), paint(lambda m: (0, 0, 0) if m is None else (m[1], m[2], 0)), 'bc5')
    new = patch_bgsm(src, {
        'Vehicles/Automotive/Sedan02_d.dds': TEXTURE_DIR + '/' + TEXTURE_NAME + '_d.dds',
        'Vehicles/Automotive/Sedan02_n.dds': TEXTURE_DIR + '/' + TEXTURE_NAME + '_n.dds',
        'Vehicles/Automotive/Sedan02_s.dds': TEXTURE_DIR + '/' + TEXTURE_NAME + '_s.dds',
    })
    with open(os.path.join(mat_dir, TEXTURE_NAME + '.BGSM'), 'wb') as f:
        f.write(new)
    print('wrote', mat_dir, 'and', tex_dir)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
