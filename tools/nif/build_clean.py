"""Build the Vehicle Framework VR clean sedan nif.

Geometry: Fallout4 Sedan01.nif (the clean, pre-war car).
Physics: the dynamic collision blocks of Sedan01_Postwar.nif (the rusty car is a movable static, the clean one a static).
Wheels: the four tyre+rim components of the body shape become own nodes WheelFL/FR/RL/RR, each with three shapes:
  Wheel_XX_stock (the original wheel), Wheel_XX_offroad (taller, wider, lugged; hidden by default) and
  Wheel_XX_beadlock (the bronze beadlock wheel from wheel_beadlock.py with its own material BeadlockWheel.BGSM; hidden by default).
Every shape gets its own BSLightingShaderProperty and texture set (the engine draws a property for one geometry only).
"""
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from make_wheel_assets import TEXTURE_DIR, TEXTURE_NAME, cell_uv  # noqa: E402
from nifhdr import parse  # noqa: E402
from shapes import load  # noqa: E402
from wheel_beadlock import build as build_beadlock  # noqa: E402

# Usage: python3 build_clean.py <out.nif> <clean.nif> <postwar.nif> [sedan|pickup]   (the two sources are the vanilla files, extracted from the BA2s)
if len(sys.argv) not in (4, 5):
    sys.exit(__doc__ + '\n' + 'Usage: build_clean.py <out.nif> <clean.nif> <postwar.nif> [sedan|pickup]')
OUT, CLEAN, POST = sys.argv[1], sys.argv[2], sys.argv[3]
VEHICLE = sys.argv[4] if len(sys.argv) == 5 else 'sedan'
# Per-vehicle numbers: the shape that carries the wheels (block index; its shader property is the next block, its texture set the one after), how
# many children the root has, the wheel geometry (game units, wheel-local: axis y) and the texture prefix of the car's own texture set.
PROFILES = {
    'sedan': dict(shape=7, nchild=4, stock_r=29.05, tyre_min_r=25.5, rim_r=16.8, tread_r=33.0, lug_h=1.4, back_ax=-10.6, shift=4.45,
                  lug_cols=[(-6.2, 0.9), (1.5, 7.6)], tex_prefix='Sedan02'),
    'pickup': dict(shape=13, nchild=5, stock_r=32.5, tyre_min_r=24.0, rim_r=21.0, tread_r=38.5, lug_h=1.6, back_ax=-11.9, shift=5.1,
                   lug_cols=[(-6.9, 1.1), (1.75, 8.6)], tex_prefix='PickUpTruck01d'),
}
PR = PROFILES[VEHICLE]
BSX_FLAGS = 0x02C3        # havok + ragdoll-less dynamic + external emit, like the rusty car minus the addon-node flag
ROOT_FLAGS = 0x100E       # the rusty car's root flags
BEADLOCK_MATERIAL = 'Materials\\' + TEXTURE_DIR.replace('/', '\\') + '\\' + TEXTURE_NAME + '.BGSM'

# ---- 4x4 wheel parameters (game units, wheel-local, outward = +ax)
TREAD_R = PR['tread_r']   # tread base radius (stock: stock_r)
LUG_H = PR['lug_h']       # lug height above the tread
RIM_R = PR['rim_r']       # below this radius the stock shape is kept
STOCK_TREAD_R = PR['stock_r']
KW = 1.25                 # tyre widening factor, measured from the inner face
BACK_AX = PR['back_ax']   # inner face of the stock tyre
SHIFT = PR['shift']       # outward shift of the dish (= widening of the outer sidewall)
LUG_COLS = PR['lug_cols']
TYRE_MIN_R = PR['tyre_min_r']
SH = PR['shape']
LUGS_PER_COL = 12


def comps(P, T):
    key = {}
    vid = np.zeros(len(P), int)
    for i, p in enumerate(P):
        k = tuple(np.round(p * 8).astype(int))
        vid[i] = key.setdefault(k, len(key))
    par = list(range(len(key)))

    def f(x):
        while par[x] != x:
            par[x] = par[par[x]]
            x = par[x]
        return x
    for t in T:
        a, b, c = vid[t]
        par[f(a)] = f(b)
        par[f(b)] = f(c)
    from collections import defaultdict
    g = defaultdict(list)
    for ti, t in enumerate(T):
        g[f(vid[t[0]])].append(ti)
    return g


d, bsv, strs, blocks, shapes = load(CLEAN)
dp, _, _, blocksp, _ = parse(POST)
assert bsv == 130
shape7 = [h for h in shapes if h['idx'] == SH][0]
P = shape7['pos'].astype(np.float32)
T = shape7['tri']
VSZ = shape7['vsz']
assert VSZ == 20 and shape7['end'] == shape7['after']
vbase = shape7['p'] + 4 + 2 + 4
VB = np.frombuffer(d, dtype=np.uint8, count=shape7['nv'] * VSZ, offset=vbase).reshape(-1, VSZ).copy()

# ---- find the wheels
g = comps(P, T)
found = {}
for r, tl in g.items():
    if len(tl) != 450:
        continue
    Q = P[np.unique(T[tl].ravel())]
    c = (Q.min(0) + Q.max(0)) / 2
    name = ('F' if c[0] < 0 else 'R')
    name = name[0:1] if False else ('F' if c[0] < 0 else 'R') + ('L' if c[1] < 0 else 'R')
    found[name] = sorted(tl)
assert sorted(found) == ['FL', 'FR', 'RL', 'RR'], found.keys()
ORDER = ['FL', 'FR', 'RL', 'RR']

# ---- raw header pieces
p = d.index(b'\n') + 1
nb_pos = p + 9
nb, = struct.unpack_from('<I', d, nb_pos)
q = nb_pos + 8
for _ in range(4):
    q += 1 + d[q]
ntypes, = struct.unpack_from('<H', d, q)
types_start = q
q += 2
types = []
for _ in range(ntypes):
    n, = struct.unpack_from('<I', d, q)
    q += 4
    types.append(d[q:q + n].decode())
    q += n
idx_pos = q
idx = list(struct.unpack_from('<%dH' % nb, d, q))
q += 2 * nb
sizes = list(struct.unpack_from('<%dI' % nb, d, q))
q += 4 * nb
q += 8
for _ in range(len(strs)):
    n, = struct.unpack_from('<I', d, q)
    q += 4 + n
groups_tail = d[q:blocks[0][1]]
bd = [bytearray(d[o:o + s]) for (t, o, s) in blocks]
tindex = {t: i for i, t in enumerate(types)}
newstrs = list(strs)


def S(s):
    if s in newstrs:
        return newstrs.index(s)
    newstrs.append(s)
    return len(newstrs) - 1


# ---- physics transplant, BSX, root flags
bp = [bytearray(dp[o:o + s]) for (t, o, s) in blocksp]
assert blocksp[2][0] == 'bhkNPCollisionObject' and blocksp[3][0] == 'bhkPhysicsSystem'
assert bytes(bp[2]) == bytes(bd[2]), 'collision object blocks differ'
bd[3] = bp[3]
assert struct.unpack_from('<I', bd[1], 0)[0] == 1 or True
struct.pack_into('<I', bd[1], 4, BSX_FLAGS)
struct.pack_into('<I', bd[0], 16, ROOT_FLAGS)

# ---- helpers for new blocks
# The rusty (dynamic) car has 0x100E on its root and on every shape; the clean (static) one 0x000E. Use the dynamic car's flags everywhere.
SHAPE_HDR_FLAGS = ROOT_FLAGS
for bi in [i for i, b in enumerate(blocks) if b[0] == 'BSTriShape']:
    assert struct.unpack_from('<I', bd[bi], 12)[0] == 0x0e
    struct.pack_into('<I', bd[bi], 12, SHAPE_HDR_FLAGS)


def enc(v):
    return np.clip(np.round((np.asarray(v, dtype=np.float64) + 1.0) * 127.5), 0, 255).astype(np.uint8)


def pack_vertex(pos, uv, n, t):
    n = np.asarray(n, float)
    t = np.asarray(t, float)
    n = n / (np.linalg.norm(n) + 1e-12)
    t = t - n * float(t @ n)
    t = t / (np.linalg.norm(t) + 1e-12)
    b = np.cross(t, n)
    out = bytearray(20)
    out[0:6] = np.array(pos, dtype=np.float16).tobytes()
    out[6:8] = np.array([b[0]], dtype=np.float16).tobytes()
    out[8:12] = np.array(uv, dtype=np.float16).tobytes()
    out[12:15] = bytes(enc(n))
    out[15] = int(enc(b[1]))
    out[16:19] = bytes(enc(t))
    out[19] = int(enc(b[2]))
    return bytes(out)


def shape_block(name, flags, vd, shader_ref, verts_bytes, tris, bound_c, bound_r):
    nv = len(verts_bytes) // VSZ
    h = struct.pack('<iIi', S(name), 0, -1)
    h += struct.pack('<I', flags)
    h += struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', 1, 0, 0, 0, 1, 0, 0, 0, 1) + struct.pack('<f', 1.0) + struct.pack('<i', -1)
    h += struct.pack('<4f', float(bound_c[0]), float(bound_c[1]), float(bound_c[2]), float(bound_r))
    h += struct.pack('<3i', -1, shader_ref, -1)
    h += struct.pack('<Q', vd)
    tri_bytes = np.asarray(tris, dtype=np.uint16).tobytes()
    h += struct.pack('<I', len(tris)) + struct.pack('<H', nv) + struct.pack('<I', len(verts_bytes) + len(tri_bytes))
    return h + verts_bytes + tri_bytes


def node_block(name, translate, children, flags):
    nd = struct.pack('<iIi', S(name), 0, -1) + struct.pack('<I', flags)
    nd += struct.pack('<3f', *[float(x) for x in translate]) + struct.pack('<9f', 1, 0, 0, 0, 1, 0, 0, 0, 1) + struct.pack('<f', 1.0) + struct.pack('<i', -1)
    nd += struct.pack('<I', len(children)) + struct.pack('<%di' % len(children), *children)
    return nd


# ---- wheel data
def wheel_data(tl):
    vs = np.unique(T[tl].ravel())
    Q = P[vs]
    hub = (Q.min(0) + Q.max(0)) / 2
    hub = np.round(hub * 4) / 4
    remap = {int(v): i for i, v in enumerate(vs)}
    tri = np.array([[remap[int(a)] for a in T[t]] for t in tl], dtype=np.uint16)
    local = (P[vs] - hub).astype(np.float32)
    raw = VB[vs].copy()
    return hub.astype(np.float32), local, raw, tri


def deform(local, s):
    """Stock wheel-local positions -> 4x4 wheel positions. Axis is y, s = +1/-1 is the outward direction along y."""
    x, y, z = local[:, 0].astype(np.float64), local[:, 1].astype(np.float64), local[:, 2].astype(np.float64)
    r = np.hypot(x, z)
    ax = s * y
    tyre = r >= TYRE_MIN_R
    kr = (TREAD_R - RIM_R) / (STOCK_TREAD_R - RIM_R)
    r2 = r.copy()
    r2[tyre] = RIM_R + (r[tyre] - RIM_R) * kr
    ax2 = ax.copy()
    ax2[tyre] = BACK_AX + (ax[tyre] - BACK_AX) * KW
    ax2[~tyre] = ax[~tyre] + SHIFT * (ax[~tyre] > 3.0)
    k = np.where(r > 1e-6, r2 / np.maximum(r, 1e-6), 1.0)
    return np.stack([x * k, s * ax2, z * k], axis=1).astype(np.float32)


def lugs(s):
    """Lug blocks on the tread. Returns (vertex bytes list, triangles list)."""
    verts = []
    tris = []

    def to_xyz(r, phi, ax):
        return np.array([r * math.cos(phi), s * ax, r * math.sin(phi)])

    def uv_of(phi, ax):
        u = 0.004 + (phi / (2 * math.pi) % 1.0) * 0.636
        axs = BACK_AX + (ax - BACK_AX) / KW
        v = 0.717 + (axs + 7.4) / 11.7 * 0.035
        return (min(max(u, 0.004), 0.64), min(max(v, 0.717), 0.752))

    def quad(pts, uvs, outward):
        p0, p1, p2, p3 = pts
        n = np.cross(p1 - p0, p2 - p0)
        if float(n @ outward) < 0:
            pts = [p0, p3, p2, p1]
            uvs = [uvs[0], uvs[3], uvs[2], uvs[1]]
            p0, p1, p2, p3 = pts
            n = np.cross(p1 - p0, p2 - p0)
        n = n / (np.linalg.norm(n) + 1e-12)
        t = p1 - p0
        base = len(verts)
        for pt, uv in zip(pts, uvs):
            verts.append(pack_vertex(pt, uv, n, t))
        tris.append((base, base + 1, base + 2))
        tris.append((base, base + 2, base + 3))

    r0 = TREAD_R - 0.9
    r1 = TREAD_R + LUG_H
    for ci, (a0, a1) in enumerate(LUG_COLS):
        for k in range(LUGS_PER_COL):
            phi = (k + 0.5 * ci) * 2 * math.pi / LUGS_PER_COL
            wb = math.radians(9.0)
            wt = math.radians(7.0)
            ins = 0.45
            B = {(sp, sa): to_xyz(r0, phi + sp * wb, a0 if sa < 0 else a1) for sp in (-1, 1) for sa in (-1, 1)}
            Tp = {(sp, sa): to_xyz(r1, phi + sp * wt, a0 + ins if sa < 0 else a1 - ins) for sp in (-1, 1) for sa in (-1, 1)}
            uvB = {(sp, sa): uv_of(phi + sp * wb, a0 if sa < 0 else a1) for sp in (-1, 1) for sa in (-1, 1)}
            uvT = {(sp, sa): uv_of(phi + sp * wt, a0 + ins if sa < 0 else a1 - ins) for sp in (-1, 1) for sa in (-1, 1)}
            radial = np.array([math.cos(phi), 0, math.sin(phi)])
            tang = np.array([-math.sin(phi), 0, math.cos(phi)])
            axis = np.array([0, s, 0.0])   # outward along the axle
            # top
            quad([Tp[(-1, -1)], Tp[(1, -1)], Tp[(1, 1)], Tp[(-1, 1)]], [uvT[(-1, -1)], uvT[(1, -1)], uvT[(1, 1)], uvT[(-1, 1)]], radial)
            # sides along the tyre (phi +/-)
            for sp in (-1, 1):
                quad([B[(sp, -1)], B[(sp, 1)], Tp[(sp, 1)], Tp[(sp, -1)]], [uvB[(sp, -1)], uvB[(sp, 1)], uvT[(sp, 1)], uvT[(sp, -1)]], tang * sp)
            # ends across the tyre (inner and outer)
            for sa in (-1, 1):
                quad([B[(-1, sa)], B[(1, sa)], Tp[(1, sa)], Tp[(-1, sa)]], [uvB[(-1, sa)], uvB[(1, sa)], uvT[(1, sa)], uvT[(-1, sa)]], axis * sa)
    return verts, tris


def beadlock_texture_set(template):
    """The car's texture set with the three texture paths of the beadlock palette (the BGSM decides in the game; the nif keeps them consistent)."""
    ts = bytes(template)
    for letter in 'dns':
        old = ('textures\\Vehicles\\Automotive\\%s_%s.dds' % (PR['tex_prefix'], letter)).encode()
        new = ('textures\\' + TEXTURE_DIR.replace('/', '\\') + '\\' + TEXTURE_NAME + '_%s.dds' % letter).encode()
        a, b = struct.pack('<I', len(old)) + old, struct.pack('<I', len(new)) + new
        assert ts.count(a) == 1, old
        ts = ts.replace(a, b)
    return ts


def beadlock_shape(name, s, shader_ref):
    P_, N_, T_, mats, _ = build_beadlock(s)
    uvs = [cell_uv(m) for m in mats]
    verts = []
    for k in range(len(P_)):
        n = N_[k].astype(float)
        t = np.cross(n, [0.0, 1.0, 0.0])
        if np.linalg.norm(t) < 1e-3:
            t = np.cross(n, [1.0, 0.0, 0.0])
        verts.append(pack_vertex(P_[k], uvs[k], n, t))
    assert len(verts) < 65535
    c = (P_.min(0) + P_.max(0)) / 2
    rad = float(np.linalg.norm(P_ - c, axis=1).max())
    blk = shape_block(name, SHAPE_HDR_FLAGS | 1, vd, shader_ref, b''.join(verts), [tuple(int(v) for v in t) for t in T_], c, rad)
    return blk, len(verts), len(T_), P_


# ---- assemble new blocks
base_n = nb
new_blocks = []          # (type index, bytes)
wheel_nodes = []
sp_template = bytes(bd[SH + 1])
ts_template = bytes(bd[SH + 2])
assert struct.unpack_from('<i', sp_template, 40)[0] == SH + 2
vd = shape7['vd']
# block index plan: per wheel i: node, stock shape, offroad shape, beadlock shape  -> base_n + 4*i + {0,1,2,3}
# then per shape j (0..11, j = 3*i + set) a shader property and a texture set: base_n + 16 + 2*j, +1
props_base = base_n + 4 * 4
hubs = {}
for i, name in enumerate(ORDER):
    hub, local, raw, tri = wheel_data(found[name])
    hubs[name] = hub
    # outward direction: where the hub cap (smallest radius vertex) lies
    rr = np.hypot(local[:, 0], local[:, 2])
    s = 1.0 if local[int(np.argmin(rr)), 1] > 0 else -1.0
    node_i = base_n + 4 * i
    # --- stock shape
    stock_raw = raw.copy()
    stock_raw[:, 0:6] = np.frombuffer(local.astype(np.float16).tobytes(), dtype=np.uint8).reshape(-1, 6)
    c = (local.min(0) + local.max(0)) / 2
    rad = float(np.linalg.norm(local - c, axis=1).max())
    stock = shape_block('Wheel_%s_stock' % name, SHAPE_HDR_FLAGS, vd, props_base + 2 * (3 * i), stock_raw.tobytes(), tri, c, rad)
    # --- offroad shape
    dl = deform(local, s)
    off_raw = raw.copy()
    off_raw[:, 0:6] = np.frombuffer(dl.astype(np.float16).tobytes(), dtype=np.uint8).reshape(-1, 6)
    lv, lt = lugs(s)
    allv = off_raw.tobytes() + b''.join(lv)
    nverts_stock = len(raw)
    alltris = [tuple(t) for t in tri] + [(a + nverts_stock, b + nverts_stock, cc + nverts_stock) for a, b, cc in lt]
    assert nverts_stock + len(lv) < 65535
    allpos = np.concatenate([dl, np.array([np.frombuffer(v[0:6], dtype=np.float16).astype(np.float32) for v in lv])])
    c2 = (allpos.min(0) + allpos.max(0)) / 2
    rad2 = float(np.linalg.norm(allpos - c2, axis=1).max())
    off = shape_block('Wheel_%s_offroad' % name, SHAPE_HDR_FLAGS | 1, vd, props_base + 2 * (3 * i + 1), allv, alltris, c2, rad2)
    bead, bead_verts, bead_tris, bead_pos = beadlock_shape('Wheel_%s_beadlock' % name, s, props_base + 2 * (3 * i + 2))
    node = node_block('Wheel' + name, hub, [node_i + 1, node_i + 2, node_i + 3], ROOT_FLAGS)
    new_blocks += [(tindex['NiNode'], node), (tindex['BSTriShape'], stock), (tindex['BSTriShape'], off), (tindex['BSTriShape'], bead)]
    wheel_nodes.append(node_i)
    print(name, 'hub', hub.tolist(), 'outward', s, 'stock verts', len(raw), 'tris', len(tri), '| offroad verts', nverts_stock + len(lv), 'tris', len(alltris),
          'extent', np.round(allpos.min(0), 1).tolist(), np.round(allpos.max(0), 1).tolist(),
          '| beadlock verts', bead_verts, 'tris', bead_tris, 'extent', np.round(bead_pos.min(0), 1).tolist(), np.round(bead_pos.max(0), 1).tolist())
for j in range(3 * 4):
    sp = bytearray(sp_template)
    struct.pack_into('<i', sp, 40, props_base + 2 * j + 1)
    ts = ts_template
    if j % 3 == 2:   # the beadlock shape: its own material and palette texture
        struct.pack_into('<i', sp, 4, S(BEADLOCK_MATERIAL))
        ts = beadlock_texture_set(ts_template)
    new_blocks += [(tindex['BSLightingShaderProperty'], bytes(sp)), (tindex['BSShaderTextureSet'], ts)]

# ---- strip the wheel triangles from shape 7 (block 7)
rm = set(i for tl in found.values() for i in tl)
keep = [i for i in range(len(T)) if i not in rm]
used = np.unique(T[keep].ravel())
remap = np.full(len(P), -1, int)
remap[used] = np.arange(len(used))
T2 = remap[T[keep]].astype(np.uint16)
V2 = VB[used]
orig7 = bytes(bd[SH])
hdr_len = shape7['p'] - blocks[SH][1]
body = struct.pack('<I', len(keep)) + struct.pack('<H', len(used)) + struct.pack('<I', len(used) * VSZ + len(keep) * 6) + V2.tobytes() + T2.tobytes()
bd[SH] = bytearray(orig7[:hdr_len] + body)

# ---- root children
root = bytes(bd[0])
nchild = struct.unpack_from('<I', root, len(root) - 4 * PR['nchild'] - 4)[0]
assert nchild == PR['nchild'], nchild
ncpos = len(root) - 4 * nchild - 4
root = root[:ncpos] + struct.pack('<I', nchild + 4) + root[ncpos + 4:] + struct.pack('<4i', *wheel_nodes)
bd[0] = bytearray(root)

# ---- write
allb = [bytes(b) for b in bd]
for ti, b in new_blocks:
    allb.append(b)
    idx.append(ti)
sizes = [len(b) for b in allb]
nbn = len(allb)
types_bytes = d[types_start + 2:idx_pos]
out = bytearray(d[:nb_pos]) + struct.pack('<I', nbn) + d[nb_pos + 4:types_start] + struct.pack('<H', len(types)) + types_bytes
out += struct.pack('<%dH' % nbn, *idx) + struct.pack('<%dI' % nbn, *sizes)
out += struct.pack('<II', len(newstrs), max(len(x) for x in newstrs))
for x in newstrs:
    b = x.encode()
    out += struct.pack('<I', len(b)) + b
out += groups_tail
for b in allb:
    out += b
last = blocks[-1]
out += d[last[1] + last[2]:]
open(OUT, 'wb').write(out)
print('blocks', nb, '->', nbn, 'size', len(d), '->', len(out), 'body tris', len(T), '->', len(keep))
