"""Build the exhaust smoke puff nif: one flat square sprite (half-size 3 game units) with the soft smoke effect material.

The plugin scales it, moves it and turns it to face the player's head every frame; it is double faced (a front and a back, both with their normals),
so it shows whichever way the engine culls. The look is the texture's alpha (see ../make_smoke_assets.py).
Header and block types come from the vanilla Pick-R-Up nif; the effect shader block is written here from the decoded layout (BSVER 130) and checked
against the vanilla block it was decoded from.

Usage: python3 make_puff_nif.py <out.nif> <PickUpTruck01.nif>
"""
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nifhdr import parse  # noqa: E402

BGEM_PATH = 'Materials\\VehicleFrameworkVR\\Effects\\SmokePuff.BGEM'
TEX_BASE = 'textures\\VehicleFrameworkVR\\Effects\\SmokePuff_d.dds'
TEX_NORMAL = 'textures\\Shared\\FlatFlat_n.dds'
HALF = 3.0          # game units; the plugin scales the node


def sized(s):
    b = s.encode()
    return struct.pack('<I', len(b)) + b


def effect_shader(name_idx, base_tex, normal_tex, flags1, flags2, light_u8, falloff, colour, scale, soft_depth, env_tex='', env_mask='', grey=''):
    """BSEffectShaderProperty of a BSVER 130 nif."""
    b = struct.pack('<iIi', name_idx, 0, -1)                    # name (the material path), no extra data, no controller
    b += struct.pack('<II', flags1, flags2)
    b += struct.pack('<4f', 0, 0, 1, 1)                          # uv offset, uv scale
    b += sized(base_tex)
    b += struct.pack('<4B', 3, light_u8, 0, 0)                   # clamp mode (wrap), lighting influence, env map min lod, unused
    b += struct.pack('<4f', *falloff)                            # start angle, stop angle, start opacity, stop opacity
    b += struct.pack('<4f', *colour)                             # base colour rgba
    b += struct.pack('<2f', scale, soft_depth)                   # base colour scale, soft falloff depth
    b += sized(grey) + sized(env_tex) + sized(normal_tex) + sized(env_mask)
    b += struct.pack('<f', 1.0)                                  # env map scale
    return b


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    out_path, src = sys.argv[1], sys.argv[2]
    d, bsv, strs, blocks, _ = parse(src)
    assert bsv == 130
    types_wanted = ['NiNode', 'BSTriShape', 'BSEffectShaderProperty', 'NiAlphaProperty']
    # the vanilla glass: shape 16, shader 17, alpha 18
    assert blocks[16][0] == 'BSTriShape' and blocks[17][0] == 'BSEffectShaderProperty' and blocks[18][0] == 'NiAlphaProperty'
    van = bytes(d[blocks[17][1]:blocks[17][1] + blocks[17][2]])
    # self-check of the shader layout: rebuild the vanilla block from its own values
    rebuilt = effect_shader(12, 'textures\\Vehicles\\Vertibird\\Vertibird06_d.dds', 'textures\\Shared\\FlatFlat_n.dds', 0xA00000C0, 0x40000010, 0xF9,
                            struct.unpack_from('<4f', van, 89), struct.unpack_from('<4f', van, 105), 1.0, 100.0,
                            env_tex='textures\\Shared\\Cubemaps\\CGPrewarStreet_e.dds', env_mask='textures\\Vehicles\\Vertibird\\Vertibird06_s.dds')
    assert rebuilt == van, 'the effect shader layout does not reproduce the vanilla block'

    # ---- raw header pieces (same walk as build_clean.py)
    p = d.index(b'\n') + 1
    nb_pos = p + 9
    nb, = struct.unpack_from('<I', d, nb_pos)
    q = nb_pos + 8
    for _ in range(4):
        q += 1 + d[q]
    types_start = q
    ntypes, = struct.unpack_from('<H', d, q)
    q += 2
    types = []
    for _ in range(ntypes):
        n, = struct.unpack_from('<I', d, q)
        q += 4
        types.append(d[q:q + n].decode())
        q += n
    idx_pos = q
    q += 2 * nb
    q += 4 * nb
    q += 8
    for _ in range(len(strs)):
        n, = struct.unpack_from('<I', d, q)
        q += 4 + n
    groups_tail = d[q:blocks[0][1]]
    tindex = {t: i for i, t in enumerate(types)}
    newstrs = ['VFVR_Puff', 'VFVR_Puff:0', BGEM_PATH]

    def enc(v):
        return np.clip(np.round((np.asarray(v, dtype=np.float64) + 1.0) * 127.5), 0, 255).astype(np.uint8)

    def pack_vertex(pos, uv, n, t):
        n = np.asarray(n, float)
        t = np.asarray(t, float)
        n = n / (np.linalg.norm(n) + 1e-12)
        t = t - n * float(t @ n)
        t = t / (np.linalg.norm(t) + 1e-12)
        b = np.cross(t, n)
        o = bytearray(20)
        o[0:6] = np.array(pos, dtype=np.float16).tobytes()
        o[6:8] = np.array([b[0]], dtype=np.float16).tobytes()
        o[8:12] = np.array(uv, dtype=np.float16).tobytes()
        o[12:15] = bytes(enc(n))
        o[15] = int(enc(b[1]))
        o[16:19] = bytes(enc(t))
        o[19] = int(enc(b[2]))
        return bytes(o)

    # ---- the sprite: front faces -Y (x to the right, z up, u along x, v down the image), the back faces +Y with the picture the right way round
    H = HALF
    front = [(-H, 0, -H, 0, 1), (H, 0, -H, 1, 1), (H, 0, H, 1, 0), (-H, 0, H, 0, 0)]
    back = [(H, 0, -H, 0, 1), (-H, 0, -H, 1, 1), (-H, 0, H, 1, 0), (H, 0, H, 0, 0)]
    verts = [pack_vertex((x, y, z), (u, v), (0, -1, 0), (1, 0, 0)) for x, y, z, u, v in front]
    verts += [pack_vertex((x, y, z), (u, v), (0, 1, 0), (-1, 0, 0)) for x, y, z, u, v in back]
    tris = [(0, 1, 2), (0, 2, 3), (4, 5, 6), (4, 6, 7)]
    vb = b''.join(verts)
    tb = np.array(tris, dtype=np.uint16).tobytes()

    vd = 0x1b00000430205                                          # position, uv, normal, tangent: 20 bytes a vertex
    h = struct.pack('<iIi', 1, 0, -1)                             # name 'VFVR_Puff:0', no extra data, no controller
    h += struct.pack('<I', 0x0E)
    h += struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', 1, 0, 0, 0, 1, 0, 0, 0, 1) + struct.pack('<f', 1.0) + struct.pack('<i', -1)
    h += struct.pack('<4f', 0, 0, 0, H * math.sqrt(2.0) * 1.02)
    h += struct.pack('<3i', -1, 2, 3)                             # skin, shader, alpha
    h += struct.pack('<Q', vd)
    h += struct.pack('<I', len(tris)) + struct.pack('<H', len(verts)) + struct.pack('<I', len(vb) + len(tb))
    shape = h + vb + tb

    # shader flags: the glass' minus falloff (bit 6) and environment mapping (bit 7) in the first word, minus double sided (bit 4) in the second
    shader = effect_shader(2, TEX_BASE, TEX_NORMAL, 0xA0000000, 0x40000000, 0xCC, (1.0, 0.0, 1.0, 1.0), (1.0, 1.0, 1.0, 1.0), 1.0, 100.0)
    alpha = bytearray(d[blocks[18][1]:blocks[18][1] + blocks[18][2]])
    struct.pack_into('<i', alpha, 0, -1)
    node = struct.pack('<iIi', 0, 0, -1) + struct.pack('<I', 0x0E) + struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', 1, 0, 0, 0, 1, 0, 0, 0, 1) \
        + struct.pack('<f', 1.0) + struct.pack('<i', -1) + struct.pack('<Ii', 1, 1)
    allb = [node, shape, shader, bytes(alpha)]
    idx = [tindex[t] for t in types_wanted]
    out = bytearray(d[:nb_pos]) + struct.pack('<I', len(allb)) + d[nb_pos + 4:types_start] + struct.pack('<H', ntypes)
    out += d[types_start + 2:idx_pos]
    out += struct.pack('<%dH' % len(allb), *idx) + struct.pack('<%dI' % len(allb), *[len(b) for b in allb])
    out += struct.pack('<II', len(newstrs), max(len(x) for x in newstrs))
    for x in newstrs:
        b = x.encode()
        out += struct.pack('<I', len(b)) + b
    out += groups_tail
    for b in allb:
        out += b
    last = blocks[-1]
    out += d[last[1] + last[2]:]
    with open(out_path, 'wb') as f:
        f.write(out)
    print('puff nif', len(out), 'bytes,', len(verts), 'verts', len(tris), 'tris')


main()
