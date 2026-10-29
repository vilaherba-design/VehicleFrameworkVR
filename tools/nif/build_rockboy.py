"""Build the Rockboy vehicle: nif, textures and materials, from the Claude Design glb ("Wasteland Wagon", no wheels).

The glb has 787k triangles. The body shell (one 450k-triangle mesh with the weathered paint texture) is decimated with the quadric simplifier of
meshoptimizer (normals and UV islands weighted into the error, regularised so the triangles stay even); the small parts (about 1200 objects) are welded,
decimated to a 1.5 mm error and re-lit with crease-angle normals (smooth where the surface curves, hard where it folds). Everything is split into
shapes of at most 60000 vertices (the nif has 16-bit indices). Small parts share one palette texture (a flat colour per material: the artifact's
procedural textures are replaced by their average); the body keeps its paint texture at half size. The glass is the vanilla car glass material.
Wheels, collision and the physics block come from the built sedan nif (Sedan01.nif): wheels are turned to the Rockboy's frame (model forward +Y),
made 1.25x taller and 0.85x narrower and moved to the Rockboy's axles.

The glb comes from tools/nif/wagon/wasteland_wagon_v5.html (open it in a browser, Download GLB); v5 = 0.1.78 Vault-Tec look.

Needs libmeshopt.so (see meshopt.py).

Usage: build_rockboy.py <wagon.glb> <Sedan01.nif (built)> <BeadlockWheel.BGSM (built, a Sedan02 clone)> <mod data dir>
Writes: <data>/Meshes/VehicleFrameworkVR/Vehicles/Rockboy.nif, <data>/Textures/VehicleFrameworkVR/Vehicles/Rockboy{Paint,Palette}_{d,n,s}.dds,
        <data>/Materials/VehicleFrameworkVR/Vehicles/Rockboy{Paint,Palette}.BGSM
"""
import io
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import geo  # noqa: E402
import meshopt  # noqa: E402
import meshprep  # noqa: E402
import glbscene  # noqa: E402
from make_wheel_assets import patch_bgsm, write_dds  # noqa: E402
from nifhdr import parse  # noqa: E402
from shapes import load as load_shapes  # noqa: E402

if len(sys.argv) != 5:
    sys.exit(__doc__)
GLB, SEDAN, BGSM_T, DATA = sys.argv[1:5]
UNIT = 70.0                      # game units per metre
TEX_DIR = 'VehicleFrameworkVR/Vehicles'
BODY_TRIS = 100000               # triangles of the body shell after decimation
PART_ERR = 0.0015                # decimation error of the small parts, metres
GLASS_ERR = 0.001
CREASE = 40.0                    # crease angle of the small parts' normals, degrees
MAX_VERTS = 60000                # per shape (16-bit indices)
FLAGS = 0x100E
WHEEL_RADIAL, WHEEL_AXIAL = 1.25, 0.85
HUB_X, HUB_Y, HUB_Z = 68.5, 97.3, 36.4      # game units: axle half track, half wheelbase, wheel centre height


def to_game(v):
    """three.js (x forward, y up, z right) -> nif (x right, y forward, z up), metres."""
    return np.stack([v[:, 2], v[:, 0], v[:, 1]], axis=1)


# ------------------------------------------------------------------ glb -> simplified shapes
js, binc, prims = glbscene.prims(GLB)
import glb as glbmod  # noqa: E402

# materials: average colour, spec, smoothness
from PIL import Image  # noqa: E402


def image_of_texture(ti):
    img = js['images'][js['textures'][ti]['source']]
    bv = js['bufferViews'][img['bufferView']]
    off = bv.get('byteOffset', 0)
    return Image.open(io.BytesIO(binc[off:off + bv['byteLength']])).convert('RGB')


def srgb(c):
    c = np.clip(np.asarray(c, float), 0, 1)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1 / 2.4) - 0.055)


MATS = {}
for mi, m in enumerate(js['materials']):
    pbr = m.get('pbrMetallicRoughness', {})
    if 'baseColorTexture' in pbr:
        im = image_of_texture(pbr['baseColorTexture']['index'])
        rgb = np.asarray(im.resize((64, 64), Image.BILINEAR), float).reshape(-1, 3).mean(0)
    else:
        rgb = srgb(pbr.get('baseColorFactor', [0.5, 0.5, 0.5, 1])[:3]) * 255
    metal = pbr.get('metallicFactor', 1.0 if 'metallicFactor' not in pbr else pbr['metallicFactor'])
    rough = pbr.get('roughnessFactor', 1.0)
    MATS[m['name']] = dict(idx=mi, rgb=np.clip(rgb, 0, 255).astype(np.uint8), spec=int(40 + 200 * metal), smooth=int(255 * (1 - rough) * 0.85 + 20))
palette_names = [n for n in MATS if n not in ('paint_blue_weathered', 'glass_dirty')]
COLS, ROWS, CELL = 6, 4, 16
assert len(palette_names) <= COLS * ROWS


def cell_uv(name):
    k = palette_names.index(name)
    return ((k % COLS + 0.5) / COLS, (k // COLS + 0.5) / ROWS)


body_prim = [p for p in prims if p['name'] == 'body_shell'][0]
glass_prims = [p for p in prims if p['mat'] == 'glass_dirty']
small = [p for p in prims if p is not body_prim and p['mat'] != 'glass_dirty']

# body: quadric decimation that keeps normals and UV islands
bpos = body_prim['pos'].astype(np.float32)
bnor = body_prim['nor'].astype(np.float32)
buv = body_prim['uv'].astype(np.float32)
btri, berr = meshopt.simplify(bpos, body_prim['tri'], np.concatenate([bnor, buv], axis=1), [0.6, 0.6, 0.6, 2.0, 2.0], target_tris=BODY_TRIS, error=0.05,
                              options=meshopt.REGULARIZE)
btri, bpos, bnor, buv = meshprep.compact(btri, bpos, bnor, buv)
print('body', len(body_prim['tri']), '->', len(btri), 'tris', len(bpos), 'verts, relative error', round(berr, 5))

# small parts -> palette shapes
pal_p, pal_n, pal_u, pal_t = [], [], [], []
count = 0
for p in small:
    ext = np.sort(p['pos'].max(0) - p['pos'].min(0))
    if ext[2] < 0.012:
        continue
    pw, tw = meshprep.weld(p['pos'], p['tri'])
    if len(tw) == 0:
        continue
    nt, _ = meshopt.simplify(pw.astype(np.float32), tw, target_tris=12, error=PART_ERR, options=meshopt.ERROR_ABSOLUTE | meshopt.REGULARIZE_LIGHT)
    nt, pw = meshprep.compact(nt, pw)
    pos, tri, nor = meshprep.crease_normals(pw, nt, CREASE)
    uv = np.tile(cell_uv(p['mat']), (len(pos), 1))
    base = sum(len(x) for x in pal_p)
    pal_p.append(pos); pal_n.append(nor); pal_u.append(uv); pal_t.append(tri + base)
    count += len(tri)
print('parts', sum(len(p['tri']) for p in small), '->', count, 'tris')
pal_pos = np.concatenate(pal_p); pal_nor = np.concatenate(pal_n); pal_uv = np.concatenate(pal_u); pal_tri = np.concatenate(pal_t)
print('  ', len(pal_pos), 'vertices')

# glass
g_p, g_t = [], []
for p in glass_prims:
    pw, tw = meshprep.weld(p['pos'], p['tri'])
    nt, _ = meshopt.simplify(pw.astype(np.float32), tw, target_tris=12, error=GLASS_ERR, options=meshopt.ERROR_ABSOLUTE)
    nt, pw = meshprep.compact(nt, pw)
    g_t.append(nt + sum(len(x) for x in g_p)); g_p.append(pw)
glass_pos = np.concatenate(g_p); glass_tri = np.concatenate(g_t)
print('glass', len(glass_tri), 'tris')

# to game units / axes
def conv(pos):
    return (to_game(pos) * UNIT).astype(np.float64)


allp = conv(np.concatenate([bpos, pal_pos, glass_pos]))
print('extent (game units)', allp.min(0).round(1), allp.max(0).round(1))

# ------------------------------------------------------------------ textures and materials
tex_dir = os.path.join(DATA, 'Textures', *TEX_DIR.split('/'))
mat_dir = os.path.join(DATA, 'Materials', *TEX_DIR.split('/'))
os.makedirs(tex_dir, exist_ok=True)
os.makedirs(mat_dir, exist_ok=True)


def bc1_fast(img):
    h, w = img.shape[:2]
    b = img.reshape(h // 4, 4, w // 4, 4, 3).transpose(0, 2, 1, 3, 4).reshape(-1, 16, 3).astype(np.float64)
    mn, mx = b.min(1), b.max(1)

    def c565(c):
        r = np.round(c[:, 0] / 255 * 31).astype(np.int64); g = np.round(c[:, 1] / 255 * 63).astype(np.int64); bl = np.round(c[:, 2] / 255 * 31).astype(np.int64)
        return (r << 11) | (g << 5) | bl

    def from565(v):
        return np.stack([((v >> 11) & 31) / 31 * 255, ((v >> 5) & 63) / 63 * 255, (v & 31) / 31 * 255], 1)

    c0, c1 = c565(mx), c565(mn)
    c0, c1 = np.maximum(c0, c1), np.minimum(c0, c1)
    p0, p1 = from565(c0), from565(c1)
    d = p0 - p1
    dd = (d * d).sum(1)
    t = np.where(dd[:, None] > 0, ((b - p1[:, None, :]) * d[:, None, :]).sum(2) / np.maximum(dd[:, None], 1e-9), 0)
    q = np.clip(np.round(t * 3), 0, 3).astype(np.int64)
    idx = np.array([1, 3, 2, 0])[q]
    idx = np.where((c0 == c1)[:, None], 0, idx)
    bits = np.zeros(len(b), np.uint64)
    for i in range(16):
        bits |= idx[:, i].astype(np.uint64) << np.uint64(2 * i)
    out = np.zeros((len(b), 8), np.uint8)
    out[:, 0] = c0 & 255; out[:, 1] = c0 >> 8; out[:, 2] = c1 & 255; out[:, 3] = c1 >> 8
    for k in range(4):
        out[:, 4 + k] = ((bits >> np.uint64(8 * k)) & np.uint64(255)).astype(np.uint8)
    return out.tobytes()


def write_bc1(path, im):
    chain = []
    cur = im
    while True:
        chain.append(np.asarray(cur))
        if cur.width == 1 and cur.height == 1:
            break
        cur = cur.resize((max(cur.width // 2, 1), max(cur.height // 2, 1)), Image.BOX)
    chain = [c for c in chain if c.shape[0] >= 4 and c.shape[1] >= 4]
    h, w = chain[0].shape[:2]
    linear = (w // 4) * (h // 4) * 8
    hdr = b'DDS ' + struct.pack('<7I', 124, 0xA1007, h, w, linear, 0, len(chain)) + bytes(44)
    hdr += struct.pack('<2I4s5I', 32, 4, b'DX10', 0, 0, 0, 0, 0) + struct.pack('<5I', 0x401008, 0, 0, 0, 0)
    hdr += struct.pack('<5I', 71, 3, 0, 1, 0)
    with open(path, 'wb') as f:
        f.write(hdr + b''.join(bc1_fast(c) for c in chain))


tmpl = open(BGSM_T, 'rb').read()
assert tmpl[:4] == b'BGSM'


def material(name, d, n, s):
    new = patch_bgsm(tmpl, {
        'VehicleFrameworkVR/Vehicles/BeadlockWheel_d.dds': TEX_DIR + '/%s_d.dds' % name,
        'VehicleFrameworkVR/Vehicles/BeadlockWheel_n.dds': TEX_DIR + '/%s_n.dds' % name,
        'VehicleFrameworkVR/Vehicles/BeadlockWheel_s.dds': TEX_DIR + '/%s_s.dds' % name,
    })
    open(os.path.join(mat_dir, name + '.BGSM'), 'wb').write(new)


# paint: the artifact's texture at half size, flat normal, uniform spec
paint_img = image_of_texture(js['materials'][MATS['paint_blue_weathered']['idx']]['pbrMetallicRoughness']['baseColorTexture']['index'])
paint_img = paint_img.resize((2048, 1024), Image.LANCZOS)
write_bc1(os.path.join(tex_dir, 'RockboyPaint_d.dds'), paint_img)
flat = np.zeros((8, 8, 3), np.uint8)
write_dds(os.path.join(tex_dir, 'RockboyPaint_n.dds'), np.full((8, 8, 3), (128, 128, 255), np.uint8), 'bc5')
write_dds(os.path.join(tex_dir, 'RockboyPaint_s.dds'), np.full((8, 8, 3), (70, 90, 0), np.uint8), 'bc5')
material('RockboyPaint', 0, 0, 0)
# palette
W, H = COLS * CELL, ROWS * CELL
pd = np.zeros((H, W, 3), np.uint8); ps = np.zeros((H, W, 3), np.uint8)
for k, nm in enumerate(palette_names):
    x, y = (k % COLS) * CELL, (k // COLS) * CELL
    pd[y:y + CELL, x:x + CELL] = MATS[nm]['rgb']
    ps[y:y + CELL, x:x + CELL] = (MATS[nm]['spec'], MATS[nm]['smooth'], 0)
write_dds(os.path.join(tex_dir, 'RockboyPalette_d.dds'), pd, 'bc1')
write_dds(os.path.join(tex_dir, 'RockboyPalette_n.dds'), np.full((H, W, 3), (128, 128, 255), np.uint8), 'bc5')
write_dds(os.path.join(tex_dir, 'RockboyPalette_s.dds'), ps, 'bc5')
material('RockboyPalette', 0, 0, 0)

# ------------------------------------------------------------------ nif
d, bsv, strs, blocks, _ = parse(SEDAN)
assert bsv == 130
sh = {s['idx']: s for s in load_shapes(SEDAN)[4]}
VD = sh[7]['vd']
assert (VD >> 44) & 0x400 == 0 and (VD & 0xF) == 5
# header pieces (same walk as build_clean.py)
p0 = d.index(b'\n') + 1
nb_pos = p0 + 9
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
oidx = list(struct.unpack_from('<%dH' % nb, d, q))
q += 2 * nb
q += 4 * nb
q += 8
for _ in range(len(strs)):
    n, = struct.unpack_from('<I', d, q)
    q += 4 + n
groups_tail = d[q:blocks[0][1]]
tindex = {t: i for i, t in enumerate(types)}
ob = [bytearray(d[o:o + s]) for (t, o, s) in blocks]
newstrs = list(strs)


def S(s):
    if s not in newstrs:
        newstrs.append(s)
    return newstrs.index(s)


assert blocks[16][0] == 'NiNode' and blocks[13][0] == 'BSTriShape' and blocks[14][0] == 'BSEffectShaderProperty' and blocks[15][0] == 'NiAlphaProperty'
WHEEL_FIRST = 16


def enc(v):
    return np.clip(np.round((np.asarray(v, dtype=np.float64) + 1.0) * 127.5), 0, 255).astype(np.uint8)


def pack_vertices(pos, uv, nor):
    n = np.asarray(nor, float)
    n = n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
    up = np.where(np.abs(n[:, 2:3]) < 0.95, np.array([[0, 0, 1.0]]), np.array([[1.0, 0, 0]]))
    t = np.cross(up, n)
    t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-12
    b = np.cross(t, n)
    out = np.zeros((len(pos), 20), np.uint8)
    out[:, 0:6] = np.frombuffer(np.asarray(pos, np.float16).tobytes(), np.uint8).reshape(-1, 6)
    out[:, 6:8] = np.frombuffer(np.asarray(b[:, 0], np.float16).tobytes(), np.uint8).reshape(-1, 2)
    out[:, 8:12] = np.frombuffer(np.asarray(uv, np.float16).tobytes(), np.uint8).reshape(-1, 4)
    out[:, 12:15] = enc(n)
    out[:, 15] = enc(b[:, 1])
    out[:, 16:19] = enc(t)
    out[:, 19] = enc(b[:, 2])
    return out.tobytes()


def shape_bytes(name, shader_ref, alpha_ref, pos, uv, nor, tri):
    c = (pos.min(0) + pos.max(0)) / 2
    r = float(np.linalg.norm(pos - c, axis=1).max())
    vb = pack_vertices(pos, uv, nor)
    tb = np.asarray(tri, np.uint16).tobytes()
    h = struct.pack('<iIi', S(name), 0, -1) + struct.pack('<I', FLAGS)
    h += struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', 1, 0, 0, 0, 1, 0, 0, 0, 1) + struct.pack('<f', 1.0) + struct.pack('<i', -1)
    h += struct.pack('<4f', *[float(x) for x in c], r)
    h += struct.pack('<3i', -1, shader_ref, alpha_ref)
    h += struct.pack('<Q', VD)
    h += struct.pack('<I', len(tri)) + struct.pack('<H', len(pos)) + struct.pack('<I', len(vb) + len(tb))
    return h + vb + tb


sp_t = bytes(ob[8])      # BSLightingShaderProperty of the sedan body
ts_t = bytes(ob[9])      # its texture set
assert struct.unpack_from('<i', sp_t, 40)[0] == 9
SEDAN_TEX = 'Sedan02'


def lighting_blocks(mat_name):
    ts = ts_t
    for letter in 'dns':
        old = ('textures\\Vehicles\\Automotive\\%s_%s.dds' % (SEDAN_TEX, letter)).encode()
        new = ('textures\\' + TEX_DIR.replace('/', '\\') + '\\%s_%s.dds' % (mat_name, letter)).encode()
        a, b = struct.pack('<I', len(old)) + old, struct.pack('<I', len(new)) + new
        assert ts.count(a) == 1, old
        ts = ts.replace(a, b)
    return ts


new = []     # (type name, bytes) in order; the shape's shader and texture set follow it
BASE = 4
shape_idx = []


def chunked(tri, key, *arrays):
    """Split into shapes of at most MAX_VERTS vertices (sorted along key, one value per triangle); yields (tri, *arrays) per shape."""
    for sel in meshprep.split_chunks(tri, MAX_VERTS, key):
        yield meshprep.compact(tri[sel], *arrays)


def add_lit(name, pos_g, uv, nor, tri, material):
    base = BASE + len(new)
    new.append(('BSTriShape', shape_bytes(name, base + 1, -1, pos_g, uv, nor, tri)))
    sp = bytearray(sp_t)
    struct.pack_into('<i', sp, 4, S('Materials\\' + TEX_DIR.replace('/', '\\') + '\\%s.BGSM' % material))
    struct.pack_into('<i', sp, 40, base + 2)
    new.append(('BSLightingShaderProperty', bytes(sp)))
    new.append(('BSShaderTextureSet', lighting_blocks(material)))
    shape_idx.append(base)


body_g = conv(bpos)
pal_g = conv(pal_pos)
glass_g = conv(glass_pos)
# the glb's normals are in its own axes: turn them like the positions (they are unit vectors: no scale)
body_n = to_game(bnor)
pal_n_g = to_game(pal_nor)
for i, (t, p_, n_, u_) in enumerate(chunked(btri, body_g[btri][:, :, 1].mean(1), body_g, body_n, buv)):
    add_lit('Rockboy:body%d' % i, p_, u_, n_, t, 'RockboyPaint')
    print('  body shape', i, len(t), 'tris', len(p_), 'verts')
for i, (t, p_, n_, u_) in enumerate(chunked(pal_tri, pal_g[pal_tri][:, :, 1].mean(1), pal_g, pal_n_g, pal_uv)):
    add_lit('Rockboy:parts%d' % i, p_, u_, n_, t, 'RockboyPalette')
    print('  parts shape', i, len(t), 'tris', len(p_), 'verts')
i_glass = BASE + len(new)
gnor = geo.normals(glass_g, glass_tri)
new.append(('BSTriShape', shape_bytes('Rockboy:glass', i_glass + 1, i_glass + 2, glass_g, np.zeros((len(glass_g), 2)), gnor, glass_tri)))
eff = bytearray(ob[14]); alpha = bytearray(ob[15])
new += [('BSEffectShaderProperty', bytes(eff)), ('NiAlphaProperty', bytes(alpha))]
m = len(new)
delta = BASE + m - WHEEL_FIRST     # old wheel-group index -> new index

# ---- wheel group: patch refs, reorient and resize the wheel meshes, move the nodes
HUBS = {'WheelFL': (-HUB_X, HUB_Y, HUB_Z), 'WheelFR': (HUB_X, HUB_Y, HUB_Z), 'WheelRL': (-HUB_X, -HUB_Y, HUB_Z), 'WheelRR': (HUB_X, -HUB_Y, HUB_Z)}
wheel_nodes = []
wb = []
for k in range(WHEEL_FIRST, len(ob)):
    t = blocks[k][0]
    b = bytearray(ob[k])
    if t == 'NiNode':
        nc, = struct.unpack_from('<I', b, 72)
        refs = list(struct.unpack_from('<%di' % nc, b, 76))
        struct.pack_into('<%di' % nc, b, 76, *[r + delta for r in refs])
        name = strs[struct.unpack_from('<i', b, 0)[0]]
        hx, hy, hz = HUBS[name]
        struct.pack_into('<3f', b, 16, hx, hy, hz)
        wheel_nodes.append(k + delta)
    elif t == 'BSTriShape':
        skin, shader, al = struct.unpack_from('<3i', b, 88)
        struct.pack_into('<3i', b, 88, skin, shader + delta, al)
        nt, = struct.unpack_from('<I', b, 108)
        nv, = struct.unpack_from('<H', b, 112)
        v = np.frombuffer(bytes(b[118:118 + nv * 20]), np.uint8).reshape(nv, 20).copy()
        pos = np.frombuffer(v[:, 0:6].tobytes(), np.float16).reshape(nv, 3).astype(np.float64)
        # sedan model frame (axle along y) -> Rockboy frame (axle along x): (x, y, z) -> (y, -x, z)
        rot = np.stack([pos[:, 1] * WHEEL_AXIAL, -pos[:, 0] * WHEEL_RADIAL, pos[:, 2] * WHEEL_RADIAL], axis=1)
        v[:, 0:6] = np.frombuffer(rot.astype(np.float16).tobytes(), np.uint8).reshape(nv, 6)

        def dec(a):
            return a.astype(np.float64) / 127.5 - 1.0

        n = np.stack([dec(v[:, 12]), dec(v[:, 13]), dec(v[:, 14])], 1)
        tg = np.stack([dec(v[:, 16]), dec(v[:, 17]), dec(v[:, 18])], 1)
        bx = np.frombuffer(v[:, 6:8].tobytes(), np.float16).astype(np.float64)
        bt = np.stack([bx, dec(v[:, 15]), dec(v[:, 19])], 1)
        for arr, cols in ((n, (12, 13, 14)), (tg, (16, 17, 18))):
            r2 = np.stack([arr[:, 1], -arr[:, 0], arr[:, 2]], 1)
            for j, c in enumerate(cols):
                v[:, c] = enc(r2[:, j])
        r2 = np.stack([bt[:, 1], -bt[:, 0], bt[:, 2]], 1)
        v[:, 6:8] = np.frombuffer(r2[:, 0].astype(np.float16).tobytes(), np.uint8).reshape(nv, 2)
        v[:, 15] = enc(r2[:, 1]); v[:, 19] = enc(r2[:, 2])
        b[118:118 + nv * 20] = v.tobytes()
        c = (rot.min(0) + rot.max(0)) / 2
        r = float(np.linalg.norm(rot - c, axis=1).max())
        struct.pack_into('<4f', b, 72, float(c[0]), float(c[1]), float(c[2]), r)
    elif t == 'BSLightingShaderProperty':
        ts, = struct.unpack_from('<i', b, 40)
        struct.pack_into('<i', b, 40, ts + delta)
    wb.append((t, bytes(b)))

# ---- root: children = the three body shapes + the wheel nodes
root = bytearray(ob[0])
nex, = struct.unpack_from('<I', root, 4)
cpos = 8 + 4 * nex + 4 + 4 + 52 + 4
nc, = struct.unpack_from('<I', root, cpos)
assert len(root) == cpos + 4 + 4 * nc
kids = shape_idx + [i_glass] + wheel_nodes
root = bytes(root[:cpos]) + struct.pack('<I', len(kids)) + struct.pack('<%di' % len(kids), *kids)

allb = [(blocks[0][0], root)] + [(blocks[k][0], bytes(ob[k])) for k in (1, 2, 3)] + new + wb
oi = [tindex[t] for t, _ in allb]
out = bytearray(d[:nb_pos]) + struct.pack('<I', len(allb)) + d[nb_pos + 4:types_start] + struct.pack('<H', ntypes) + d[types_start + 2:idx_pos]
out += struct.pack('<%dH' % len(allb), *oi) + struct.pack('<%dI' % len(allb), *[len(b) for _, b in allb])
out += struct.pack('<II', len(newstrs), max(len(x) for x in newstrs))
for x in newstrs:
    bb = x.encode()
    out += struct.pack('<I', len(bb)) + bb
out += groups_tail
for _, b in allb:
    out += b
last = blocks[-1]
out += d[last[1] + last[2]:]
dst = os.path.join(DATA, 'Meshes', *TEX_DIR.split('/'), 'Rockboy.nif')
os.makedirs(os.path.dirname(dst), exist_ok=True)
open(dst, 'wb').write(out)
print('wrote', dst, len(out), 'bytes,', len(allb), 'blocks')
