"""Build the Rusianmother motorcycle: nif, textures and material, from the Claude Design glb ("scrambler post-apocaliptica").

The glb has 1362 parts and 141k triangles, all in one scene. Every part is welded, decimated with the quadric simplifier of meshoptimizer to a small
absolute error (0.5-1.2 mm: the silhouette does not change, the redundant triangles of the procedural meshes go) and re-lit with crease-angle normals
(smooth where the surface curves, hard where it folds). The parts are merged into four groups that the plugin moves independently:

  * the body (frame, engine, tank, seat, exhaust, rear fender, rack, saddlebags ...): shapes at the root of the nif;
  * "SteerFork": fork, front fender, headlight, handlebar, speedometer, grips, levers, cables ... one node that the plugin turns about the raked steering
    axis of the profile (nodes named Steer... follow the steering); its glass (the headlight lens) is a second shape inside it;
  * "WheelFront" / "WheelRear": the wheels, one node each at its hub with the shape Wheel_F_stock / Wheel_R_stock (tyre, lugs, rim, spokes, hub, brake disc
    or sprocket), modelled around the hub so the node can spin about the x axis.

All of it shares one weathered-paint atlas: one cell per material of the glb (its base colour times the glb's grime texture, flat where the material has
none), every part projected onto its cell. The headlight lens is the vanilla car glass material. Collision and the physics block come from the built
sedan nif (Sedan01.nif); the profile builds its own collision box.

Needs libmeshopt.so (see meshopt.py).

Usage: build_rusianmother.py <scrambler.glb> <Sedan01.nif (built)> <BeadlockWheel.BGSM (built, a Sedan02 clone)> <mod data dir>
Writes: <data>/Meshes/VehicleFrameworkVR/Vehicles/Rusianmother.nif, <data>/Textures/VehicleFrameworkVR/Vehicles/RusianmotherAtlas_{d,n,s}.dds,
        <data>/Materials/VehicleFrameworkVR/Vehicles/RusianmotherAtlas.BGSM
"""
import io
import os
import re
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import geo  # noqa: E402
import glbscene  # noqa: E402
import meshopt  # noqa: E402
import meshprep  # noqa: E402
from make_wheel_assets import patch_bgsm, write_dds  # noqa: E402
from nifhdr import parse  # noqa: E402
from PIL import Image  # noqa: E402
from shapes import load as load_shapes  # noqa: E402

if len(sys.argv) != 5:
    sys.exit(__doc__)
GLB, SEDAN, BGSM_T, DATA = sys.argv[1:5]
UNIT = 70.0                       # game units per metre
TEX_DIR = 'VehicleFrameworkVR/Vehicles'
NAME = 'Rusianmother'
ATLAS = NAME + 'Atlas'
MAX_VERTS = 60000                 # per shape (16-bit indices)
FLAGS = 0x100E
CREASE = 40.0                     # crease angle of the normals, degrees
COLS, ROWS, CELL, INSET = 4, 4, 256, 4
W, H = COLS * CELL, ROWS * CELL


def to_game(v):
    """glb axes (x forward, y up, z right) -> nif (x right, y forward, z up). A cyclic permutation: it keeps the winding, and it turns normals too."""
    return np.stack([v[:, 2], v[:, 0], v[:, 1]], axis=1)


def stem(name):
    return re.sub(r'[_.\-]?\d+$', '', name)


# ------------------------------------------------------------------ which part goes where
WHEEL_F = {'front_wheel_tire', 'front_wheel_knob', 'front_wheel_rim', 'front_wheel_rim_well', 'front_wheel_hub', 'front_wheel_hub_flange',
           'front_wheel_spoke', 'front_wheel_nipple', 'front_disc', 'front_disc_carrier', 'disc_bolt', 'disc_hole'}
WHEEL_R = {'rear_wheel_tire', 'rear_wheel_knob', 'rear_wheel_rim', 'rear_wheel_rim_well', 'rear_wheel_hub', 'rear_wheel_hub_flange',
           'rear_wheel_spoke', 'rear_wheel_nipple', 'rear_disc', 'rear_disc_carrier', 'sprocket_carrier', 'rear_sprocket', 'sprocket_tooth',
           'sprocket_hole'}
STEER = {'fork_slider', 'fork_slider_cap', 'axle_clamp', 'fork_gaiter', 'fork_tube', 'fork_cap', 'triple_clamp_lower', 'triple_clamp_upper',
         'steering_stem', 'fork_pinch_bolt', 'front_axle', 'front_caliper', 'caliper_bolt', 'front_fender', 'fender_stay', 'fender_bracket',
         'headlight_bucket', 'headlight_dome', 'headlight_lens', 'headlight_reflector', 'headlight_bulb', 'headlight_bezel', 'headlight_cage_ring',
         'cage_bar_v', 'cage_bar_h', 'headlight_ear', 'speedometer', 'speedometer_face', 'speedometer_bezel', 'bar_riser', 'handlebar',
         'bar_crossbrace', 'crossbrace_pad', 'grip', 'grip_ring', 'lever_perch', 'control_lever', 'lever_pivot', 'master_cylinder',
         'front_brake_line', 'clutch_cable', 'throttle_cable'}
GLASS = {'headlight_lens'}        # the vanilla car glass material instead of the atlas

# decimation error (metres, absolute) of the parts that are seen close up or that curve; the rest takes the default
FINE = 0.0006
DEFAULT = 0.0012
ERR = {'fuel_tank': 0.0008, 'seat': 0.0008, 'rear_fender': 0.0008, 'front_fender': 0.0006, 'chain_roller': 0.002, 'rear_shock_left_spring': 0.0015,
       'rear_shock_right_spring': 0.0015}


def group_of(name):
    s = stem(name)
    if s in WHEEL_F:
        return 'wheelF'
    if s in WHEEL_R:
        return 'wheelR'
    if s in STEER:
        return 'steer'
    return 'static'


def error_of(name, group):
    s = stem(name)
    if s in ERR:
        return ERR[s]
    return FINE if group in ('steer', 'wheelF') else (0.0008 if group == 'wheelR' else DEFAULT)


# ------------------------------------------------------------------ glb -> clean parts
js, binc, prims = glbscene.prims(GLB)


def image_of_texture(ti):
    img = js['images'][js['textures'][ti]['source']]
    bv = js['bufferViews'][img['bufferView']]
    off = bv.get('byteOffset', 0)
    return Image.open(io.BytesIO(binc[off:off + bv['byteLength']])).convert('RGB')


def srgb_to_linear(c):
    c = np.clip(np.asarray(c, float), 0, 1)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def linear_to_srgb(c):
    c = np.clip(np.asarray(c, float), 0, 1)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1 / 2.4) - 0.055)


MATS = {}
for mi, m in enumerate(js['materials']):
    pbr = m.get('pbrMetallicRoughness', {})
    MATS[m['name']] = dict(idx=mi, factor=np.array(pbr.get('baseColorFactor', [1, 1, 1, 1])[:3], float), textured='baseColorTexture' in pbr,
                           metal=pbr.get('metallicFactor', 1.0), rough=pbr.get('roughnessFactor', 1.0), emissive=np.array(m.get('emissiveFactor', [0, 0, 0]), float))
atlas_names = sorted(MATS)
assert len(atlas_names) <= COLS * ROWS
CELL_OF = {n: i for i, n in enumerate(atlas_names)}


def clean(pos, tri, err, crease=CREASE):
    pw, tw = meshprep.weld(pos, tri)
    if len(tw) == 0:
        return None
    nt, _ = meshopt.simplify(pw.astype(np.float32), tw, target_tris=12, error=err, options=meshopt.ERROR_ABSOLUTE | meshopt.REGULARIZE_LIGHT)
    nt, pw = meshprep.compact(nt, pw)
    return meshprep.crease_normals(pw, nt, crease)


def planar_uv(pos):
    """The part's own bounding box seen from its two widest axes, 0..1 (the glb's grime texture is mapped the same way)."""
    lo, hi = pos.min(0), pos.max(0)
    ext = hi - lo
    a1, a2 = np.argsort(-ext)[:2]
    return np.stack([(pos[:, a1] - lo[a1]) / max(ext[a1], 1e-6), (pos[:, a2] - lo[a2]) / max(ext[a2], 1e-6)], axis=1)


def atlas_uv(uv, material):
    k = CELL_OF[material]
    cx, cy = k % COLS, k // COLS
    return np.stack([(cx * CELL + INSET + uv[:, 0] * (CELL - 2 * INSET)) / W, (cy * CELL + INSET + uv[:, 1] * (CELL - 2 * INSET)) / H], axis=1)


groups = {g: dict(pos=[], nor=[], uv=[], tri=[], n=0) for g in ('static', 'steer', 'wheelF', 'wheelR')}
glass = dict(pos=[], tri=[], n=0)
hub_boxes = {}
src_tris = out_tris = 0
for p in prims:
    s = stem(p['name'])
    g = group_of(p['name'])
    src_tris += len(p['tri'])
    if s in ('front_wheel_hub', 'rear_wheel_hub'):
        hub_boxes[s] = (p['pos'].min(0), p['pos'].max(0))
    res = clean(p['pos'], p['tri'], error_of(p['name'], g))
    if res is None:
        continue
    pos, tri, nor = res
    out_tris += len(tri)
    if s in GLASS:
        glass['tri'].append(tri + glass['n'])
        glass['pos'].append(pos)
        glass['n'] += len(pos)
        continue
    uv = atlas_uv(planar_uv(pos), p['mat'])
    grp = groups[g]
    grp['pos'].append(pos)
    grp['nor'].append(nor)
    grp['uv'].append(uv)
    grp['tri'].append(tri + grp['n'])
    grp['n'] += len(pos)
print('parts', len(prims), '-', src_tris, '->', out_tris, 'tris')


def hub_of(key):
    lo, hi = hub_boxes[key]
    c = to_game(((lo + hi) / 2)[None, :])[0] * UNIT
    return np.array([0.0, c[1], c[2]])


HUB = {'wheelF': hub_of('front_wheel_hub'), 'wheelR': hub_of('rear_wheel_hub')}
print('hubs (game units)', HUB['wheelF'].round(2), HUB['wheelR'].round(2))

merged = {}
for g, grp in groups.items():
    pos = to_game(np.concatenate(grp['pos'])) * UNIT
    nor = to_game(np.concatenate(grp['nor']))
    uv = np.concatenate(grp['uv'])
    tri = np.concatenate(grp['tri'])
    if g in HUB:
        pos = pos - HUB[g]
    merged[g] = (pos, nor, uv, tri)
    print('  %-7s %6d tris %6d verts   extent %s .. %s' % (g, len(tri), len(pos), (pos.min(0) + (HUB[g] if g in HUB else 0)).round(1),
                                                         (pos.max(0) + (HUB[g] if g in HUB else 0)).round(1)))
glass_pos = to_game(np.concatenate(glass['pos'])) * UNIT
glass_tri = np.concatenate(glass['tri'])
print('  glass   %6d tris' % len(glass_tri))
allp = np.concatenate([merged['static'][0], merged['steer'][0], merged['wheelF'][0] + HUB['wheelF'], merged['wheelR'][0] + HUB['wheelR'], glass_pos])
print('model extent (game units)', allp.min(0).round(1), allp.max(0).round(1))

# the room the wheels have in their fenders (profile wheelVisual.maxAboveMesh): the tyre top against the fender's inner face above it
for key, fender, group in (('front', 'front_fender', 'wheelF'), ('rear', 'rear_fender', 'wheelR')):
    wp = merged[group][0] + HUB[group]
    top = wp[:, 2].max()
    fp = np.concatenate([to_game(q['pos']) * UNIT for q in prims if q['name'] == fender])
    sel = (np.abs(fp[:, 1] - HUB[group][1]) < 6) & (fp[:, 2] > top)
    print('  %s tyre top z %.2f; fender inner face above it z %.2f (gap %.2f units)' % (key, top, fp[sel, 2].min() if sel.any() else float('nan'),
                                                                                       (fp[sel, 2].min() - top) if sel.any() else float('nan')))

# ------------------------------------------------------------------ atlas, material
tex_dir = os.path.join(DATA, 'Textures', *TEX_DIR.split('/'))
mat_dir = os.path.join(DATA, 'Materials', *TEX_DIR.split('/'))
os.makedirs(tex_dir, exist_ok=True)
os.makedirs(mat_dir, exist_ok=True)

grime_srgb = np.asarray(image_of_texture(0).resize((CELL, CELL), Image.LANCZOS), float) / 255.0
grime = srgb_to_linear(grime_srgb)
grime_lum = grime.mean(2)
grime_lum /= grime_lum.mean()
atlas_d = np.zeros((H, W, 3), np.uint8)
atlas_s = np.zeros((H, W, 3), np.uint8)
for name, k in CELL_OF.items():
    m = MATS[name]
    lin = m['factor'][None, None, :] * (grime if m['textured'] else 1.0) + m['emissive'][None, None, :]
    spec = 40 + 200 * m['metal']
    smooth = 255 * (1 - m['rough']) * 0.85 + 20
    gloss = smooth * (np.clip(0.65 + 0.35 * grime_lum, 0.5, 1.15) if m['textured'] else 1.0)
    cx, cy = (k % COLS) * CELL, (k // COLS) * CELL
    cell = np.broadcast_to(lin, (CELL, CELL, 3))
    atlas_d[cy:cy + CELL, cx:cx + CELL] = np.round(linear_to_srgb(cell) * 255).astype(np.uint8)
    atlas_s[cy:cy + CELL, cx:cx + CELL, 0] = int(spec)
    atlas_s[cy:cy + CELL, cx:cx + CELL, 1] = np.clip(np.round(np.broadcast_to(gloss, (CELL, CELL))), 0, 255).astype(np.uint8)
    print('  cell %2d %-22s base %s' % (k, name, atlas_d[cy + CELL // 2, cx + CELL // 2]))
write_dds(os.path.join(tex_dir, ATLAS + '_d.dds'), atlas_d, 'bc1')
write_dds(os.path.join(tex_dir, ATLAS + '_n.dds'), np.full((8, 8, 3), (128, 128, 255), np.uint8), 'bc5')
write_dds(os.path.join(tex_dir, ATLAS + '_s.dds'), atlas_s, 'bc5')
tmpl = open(BGSM_T, 'rb').read()
assert tmpl[:4] == b'BGSM'
open(os.path.join(mat_dir, ATLAS + '.BGSM'), 'wb').write(patch_bgsm(tmpl, {
    'VehicleFrameworkVR/Vehicles/BeadlockWheel_d.dds': TEX_DIR + '/%s_d.dds' % ATLAS,
    'VehicleFrameworkVR/Vehicles/BeadlockWheel_n.dds': TEX_DIR + '/%s_n.dds' % ATLAS,
    'VehicleFrameworkVR/Vehicles/BeadlockWheel_s.dds': TEX_DIR + '/%s_s.dds' % ATLAS,
}))

# ------------------------------------------------------------------ nif
d, bsv, strs, blocks, _ = parse(SEDAN)
assert bsv == 130
sh = {s['idx']: s for s in load_shapes(SEDAN)[4]}
VD = sh[7]['vd']
assert (VD >> 44) & 0x400 == 0 and (VD & 0xF) == 5
# header pieces (same walk as build_rockboy.py)
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
    assert len(pos) <= 65535 and len(tri) > 0
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
node_t = bytes(ob[16])   # a wheel NiNode of the sedan: everything before the children count is reused
assert struct.unpack_from('<i', sp_t, 40)[0] == 9
SEDAN_TEX = 'Sedan02'


def texture_set(mat_name):
    ts = ts_t
    for letter in 'dns':
        old = ('textures\\Vehicles\\Automotive\\%s_%s.dds' % (SEDAN_TEX, letter)).encode()
        new_ = ('textures\\' + TEX_DIR.replace('/', '\\') + '\\%s_%s.dds' % (mat_name, letter)).encode()
        a, b = struct.pack('<I', len(old)) + old, struct.pack('<I', len(new_)) + new_
        assert ts.count(a) == 1, old
        ts = ts.replace(a, b)
    return ts


new = []     # (type name, bytes) in order, from block index 4
BASE = 4


def add_lit(name, pos, uv, nor, tri):
    """A BSTriShape with its own lighting shader property (the atlas material) and texture set. Returns the shape's block index."""
    base = BASE + len(new)
    new.append(('BSTriShape', shape_bytes(name, base + 1, -1, pos, uv, nor, tri)))
    sp = bytearray(sp_t)
    struct.pack_into('<i', sp, 4, S('Materials\\' + TEX_DIR.replace('/', '\\') + '\\%s.BGSM' % ATLAS))
    struct.pack_into('<i', sp, 40, base + 2)
    new.append(('BSLightingShaderProperty', bytes(sp)))
    new.append(('BSShaderTextureSet', texture_set(ATLAS)))
    return base


def add_glass(name, pos, tri):
    base = BASE + len(new)
    nor = geo.normals(pos, tri)
    new.append(('BSTriShape', shape_bytes(name, base + 1, base + 2, pos, np.zeros((len(pos), 2)), nor, tri)))
    new.append(('BSEffectShaderProperty', bytes(ob[14])))
    new.append(('NiAlphaProperty', bytes(ob[15])))
    return base


def add_node(name, translation, children):
    base = BASE + len(new)
    b = bytearray(node_t[:72])
    struct.pack_into('<i', b, 0, S(name))
    struct.pack_into('<3f', b, 16, *[float(x) for x in translation])
    new.append(('NiNode', bytes(b) + struct.pack('<I', len(children)) + struct.pack('<%di' % len(children), *children)))
    return base


def chunked(key_axis, pos, nor, uv, tri):
    """Shapes of at most MAX_VERTS vertices, cut along one axis; yields (tri, pos, nor, uv) per shape."""
    for sel in meshprep.split_chunks(tri, MAX_VERTS, pos[tri][:, :, key_axis].mean(1)):
        yield meshprep.compact(tri[sel], pos, nor, uv)


root_kids = []
for i, (t, p_, n_, u_) in enumerate(chunked(1, *merged['static'])):
    root_kids.append(add_lit('%s:body%d' % (NAME, i), p_, u_, n_, t))
    print('  body shape', i, len(t), 'tris', len(p_), 'verts')

steer_kids = []
for i, (t, p_, n_, u_) in enumerate(chunked(1, *merged['steer'])):
    steer_kids.append(add_lit('%s:fork%d' % (NAME, i), p_, u_, n_, t))
    print('  fork shape', i, len(t), 'tris', len(p_), 'verts')
steer_kids.append(add_glass('%s:lens' % NAME, glass_pos, glass_tri))
root_kids.append(add_node('SteerFork', (0, 0, 0), steer_kids))

for g, label, shape_name in (('wheelF', 'WheelFront', 'Wheel_F_stock'), ('wheelR', 'WheelRear', 'Wheel_R_stock')):
    pos, nor, uv, tri = merged[g]
    kid = add_lit(shape_name, pos, uv, nor, tri)
    print('  %s: %d tris %d verts' % (shape_name, len(tri), len(pos)))
    root_kids.append(add_node(label, HUB[g], [kid]))

# root: the children list is replaced
root = bytearray(ob[0])
nex, = struct.unpack_from('<I', root, 4)
cpos = 8 + 4 * nex + 4 + 4 + 52 + 4
nc, = struct.unpack_from('<I', root, cpos)
assert len(root) == cpos + 4 + 4 * nc
root = bytes(root[:cpos]) + struct.pack('<I', len(root_kids)) + struct.pack('<%di' % len(root_kids), *root_kids)

allb = [(blocks[0][0], root)] + [(blocks[k][0], bytes(ob[k])) for k in (1, 2, 3)] + new
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
dst = os.path.join(DATA, 'Meshes', *TEX_DIR.split('/'), NAME + '.nif')
os.makedirs(os.path.dirname(dst), exist_ok=True)
open(dst, 'wb').write(out)
print('wrote', dst, len(out), 'bytes,', len(allb), 'blocks')
