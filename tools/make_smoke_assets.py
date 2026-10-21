"""Build the exhaust smoke assets: the puff texture (procedural cloud, RGBA DDS) and its effect material (BGEM).

The old puff was a glass-like sphere (the vanilla car glass material), which read as a bubble. The new one is a flat soft cloud sprite that the plugin
turns toward the player's head every frame (see make_puff_nif.py for the mesh): its look is all in the texture's alpha, so it does not depend on the
effect shader's view-angle falloff.

Usage: python3 make_smoke_assets.py <data/mod folder> <vanilla CarPrewarSanctuary_Glass01.BGEM>
"""
import os
import struct
import sys

import numpy as np
from PIL import Image

TEX_PATH = 'VehicleFrameworkVR/Effects/SmokePuff_d.dds'     # as written in the material (relative to Data\Textures)
SIZE = 128


# ---------------------------------------------------------------- texture
def smoothstep(a, b, x):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def fbm(rng, size, beta=2.2, lo=0.0, hi=1.0):
    """Cloud noise in 0..1: white noise shaped to a 1/f^beta spectrum (no grid, no streaks), then stretched to the range."""
    white = rng.standard_normal((size, size))
    fy = np.fft.fftfreq(size)[:, None]
    fx = np.fft.rfftfreq(size)[None, :]
    f = np.sqrt(fx * fx + fy * fy)
    f[0, 0] = 1.0
    spec = np.fft.rfft2(white) * f ** (-beta / 2.0)
    spec[0, 0] = 0.0
    n = np.fft.irfft2(spec, s=(size, size))
    n = (n - n.min()) / max(n.max() - n.min(), 1e-12)
    return lo + (hi - lo) * n


def puff_image(size=SIZE, seed=6):
    """RGBA float image (0..1): a swirl of thin grey smoke, densest in the middle, transparent at and beyond the border."""
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:size, 0:size]
    u = (x + 0.5) / size * 2.0 - 1.0
    v = (y + 0.5) / size * 2.0 - 1.0
    r = np.sqrt(u * u + v * v)
    # Domain-warped density: soft blobs around the middle seen through two noise fields, so the edges curl like smoke instead of being round.
    uw = u + 0.30 * (fbm(rng, size, beta=3.4) - 0.5)
    vw = v + 0.30 * (fbm(rng, size, beta=3.4) - 0.5)
    dens = np.zeros((size, size))
    for i in range(14):
        ang = rng.uniform(0.0, 2.0 * np.pi)
        rad = 0.40 * np.sqrt(rng.uniform(0.0, 1.0))
        s = rng.uniform(0.13, 0.26)
        dens += rng.uniform(0.5, 1.0) * np.exp(-((uw - rad * np.cos(ang)) ** 2 + (vw - rad * np.sin(ang)) ** 2) / (2.0 * s * s))
    detail = fbm(rng, size, beta=2.7)                   # turbulence inside
    alpha = (1.0 - np.exp(-0.95 * dens)) * (0.30 + 0.70 * detail ** 1.2)
    alpha *= 1.0 - smoothstep(0.55, 1.0, r)             # soft all the way to the quad's edge, nothing at it
    alpha = np.clip(alpha / max(alpha.max(), 1e-6) * 0.92, 0.0, 1.0)
    # Colour: a flat dirty grey with a gentle light from the upper left and a little darker towards the bottom (no hard shading: smoke is not a rock).
    gy, gx = np.gradient((0.6 * dens + 0.8 * detail) * 3.0)
    nrm = np.stack([-gx, -gy, np.ones_like(dens)], axis=-1)
    nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
    light = np.array([-0.45, -0.55, 0.70])
    light /= np.linalg.norm(light)
    shade = np.clip(nrm @ light, 0.0, 1.0)
    grey = 0.33 + 0.22 * shade
    grey *= 1.0 - 0.14 * np.clip(v * 0.5 + 0.5, 0.0, 1.0)
    rgb = np.stack([grey * 1.0, grey * 0.985, grey * 0.96], axis=-1)
    return np.concatenate([np.clip(rgb, 0.0, 1.0), alpha[..., None]], axis=-1)


def write_dds_bgra(path, img):
    """Uncompressed B8G8R8A8 DDS with a DX10 header and a full mip chain (box filtered)."""
    cur = (np.clip(img, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
    chain = [cur]
    while cur.shape[0] > 1 or cur.shape[1] > 1:
        f = cur.astype(np.float32)
        h, w = f.shape[:2]
        h2, w2 = max(h // 2, 1), max(w // 2, 1)
        f = f[:h2 * 2, :w2 * 2].reshape(h2, 2 if h > 1 else 1, w2, 2 if w > 1 else 1, 4).mean(axis=(1, 3))
        cur = (f + 0.5).astype(np.uint8)
        chain.append(cur)
    h, w = chain[0].shape[:2]
    hdr = b'DDS ' + struct.pack('<7I', 124, 0x100F | 0x20000, h, w, w * 4, 0, len(chain)) + bytes(44)   # caps, height, width, pitch, depth, mips
    hdr += struct.pack('<2I4s5I', 32, 4, b'DX10', 0, 0, 0, 0, 0) + struct.pack('<5I', 0x401008, 0, 0, 0, 0)
    hdr += struct.pack('<5I', 87, 3, 0, 1, 0)             # DXGI_FORMAT_B8G8R8A8_UNORM, 2D texture, 1 slice
    body = b''.join(np.ascontiguousarray(m[..., [2, 1, 0, 3]]).tobytes() for m in chain)
    with open(path, 'wb') as f:
        f.write(hdr + body)


# ---------------------------------------------------------------- material (BGEM v2)
def sized(s):
    b = s.encode() + b'\0'
    return struct.pack('<I', len(b)) + b


def parse_bgem(data):
    """The vanilla car-glass BGEM as a dict (only what we change is named; the rest of the header is kept as raw bytes)."""
    assert data[:4] == b'BGEM' and struct.unpack_from('<I', data, 4)[0] == 2, 'not a BGEM v2'
    p = 63
    strings = []
    for _ in range(5):
        n, = struct.unpack_from('<I', data, p)
        strings.append(data[p + 4:p + 4 + n - 1].decode())
        p += 4 + n
    assert data[p - 1] == 0
    flags = data[p:p + 6]
    p += 6
    colour = struct.unpack_from('<3f', data, p)
    scale, f_start, f_stop, o_start, o_stop, lighting = struct.unpack_from('<6f', data, p + 12)
    min_lod = data[p + 36]
    soft_depth, = struct.unpack_from('<f', data, p + 37)
    assert p + 41 == len(data), (p + 41, len(data))
    return {'head': data[:63], 'strings': strings, 'flags': list(flags), 'colour': colour, 'scale': scale, 'falloff': (f_start, f_stop, o_start, o_stop),
            'lighting': lighting, 'min_lod': min_lod, 'soft_depth': soft_depth}


def build_bgem(m):
    out = bytearray(m['head'])
    for s in m['strings']:
        out += sized(s) if s else struct.pack('<IB', 1, 0)
    out += bytes(m['flags'])
    out += struct.pack('<3f', *m['colour'])
    out += struct.pack('<5f', m['scale'], *m['falloff'][:4])
    out += struct.pack('<f', m['lighting']) + bytes([m['min_lod']]) + struct.pack('<f', m['soft_depth'])
    return bytes(out)


# header byte offsets (see the decoded layout in the repo docs): alpha 28, blend 32, src 33, dst 37, z write 43, z test 44, two-sided 48, non-occluder 50,
# refraction 51, env mapping 57
def smoke_material(template):
    m = parse_bgem(template)
    assert build_bgem(m) == template, 'BGEM round trip failed: the layout is not what was decoded'
    head = bytearray(m['head'])
    struct.pack_into('<f', head, 28, 0.75)       # material opacity: the texture's alpha times this
    head[32] = 1                                 # alpha blended
    struct.pack_into('<II', head, 33, 6, 7)      # source alpha / inverse source alpha
    head[43] = 0                                 # no depth write
    head[44] = 1                                 # depth test
    head[48] = 0                                 # one-sided: the plugin faces the sprite to the viewer
    head[50] = 1                                 # does not occlude
    head[51] = 0                                 # no refraction
    head[57] = 0                                 # no environment mapping
    m['head'] = bytes(head)
    m['strings'] = [TEX_PATH, '', '', 'Shared/FlatFlat_n.dds', '']
    m['flags'] = [0, 1, 0, 0, 0, 0]              # blood, effect lighting, falloff, falloff colour, grayscale->palette alpha, soft
    m['colour'] = (1.0, 1.0, 1.0)
    m['scale'] = 1.0
    m['falloff'] = (1.0, 0.0, 1.0, 1.0)
    m['lighting'] = 0.8                          # 1 = fully lit by the scene, 0 = unlit: mostly lit, so it darkens at night but stays seen
    m['soft_depth'] = 100.0
    return build_bgem(m)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    mod, glass = sys.argv[1], sys.argv[2]
    tex_dir = os.path.join(mod, 'Textures', 'VehicleFrameworkVR', 'Effects')
    mat_dir = os.path.join(mod, 'Materials', 'VehicleFrameworkVR', 'Effects')
    os.makedirs(tex_dir, exist_ok=True)
    os.makedirs(mat_dir, exist_ok=True)
    img = puff_image()
    write_dds_bgra(os.path.join(tex_dir, 'SmokePuff_d.dds'), img)
    with open(glass, 'rb') as f:
        mat = smoke_material(f.read())
    with open(os.path.join(mat_dir, 'SmokePuff.BGEM'), 'wb') as f:
        f.write(mat)
    print('smoke texture %dx%d, material %d bytes' % (SIZE, SIZE, len(mat)))


if __name__ == '__main__':
    main()
