"""Build a vehicle nif for the framework from a prepared glb (vf: prep_export_glb.py), on the frame of the built Rockboy.nif.

Generalised from build_rockboy_v7.py. Kept from the template nif byte for byte: root node, BSX flags, collision object and physics system.
Built new: the body shapes (materials <Prefix>Paint, <Prefix><Material>: BGSM patched from RockboyPaint.BGSM, textures with texconv), the glass
(the template's glass blocks), "node:<Name>" meshes under their own NiNode (the steering wheel), and the wheel group: four nodes WheelFL/FR/RL/RR at
the given hubs (copies of the template's wheel node), each with one shape per wheel set made by a wheel generator ("classic",
tools/nif/wheel_classic.py, with ClassicWheel.BGSM from tools/make_classic_wheel_assets.py, and "military", wheel_military.py with its textured
MilitaryWheel.BGSM from tools/make_military_wheel_assets.py; the shader and texture set are copies of the
template's beadlock wheel blocks with the paths replaced).

Usage: build_vehicle.py <export.glb> <template Rockboy.nif> <mod data dir> <texconv.exe> <Prefix> <wheels.json>
wheels.json: {"hubs": {"FL": [x, y, z], ...}, "radius": r, "sets": ["classic"]}  (game units; the wheel is scaled to radius r).
A set without a generator ("stock", "offroad", "beadlock") is the template's own Wheel_<corner>_<set> shape, scaled to radius r.
Writes <data>/Meshes/VehicleFrameworkVR/Vehicles/<Prefix>.nif, its textures and materials.
"""
import io
import os
import struct
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import glb  # noqa: E402
import meshprep  # noqa: E402
from make_wheel_assets import patch_bgsm  # noqa: E402
from nifhdr import parse  # noqa: E402

if len(sys.argv) != 7:
    sys.exit(__doc__)
GLB, TEMPLATE, DATA, TEXCONV, PREFIX, WHEELS = sys.argv[1:7]
import json  # noqa: E402
WHEELS = json.load(open(WHEELS))
TEX_DIR = 'VehicleFrameworkVR/Vehicles'
MAX_VERTS = 60000                 # per shape (16-bit indices)
FLAGS = 0x100E


def to_game(v):
    return np.stack([-v[:, 0], v[:, 2], v[:, 1]], axis=1)


# ------------------------------------------------------------------ glb(s)
meshes = []
for path in GLB.split(','):
    js, binc = glb.load(path)
    for node in js['nodes']:
        if 'mesh' not in node:
            continue
        assert len(js['meshes'][node['mesh']]['primitives']) == 1, 'one primitive per mesh: ' + node.get('name', '')
        pr = js['meshes'][node['mesh']]['primitives'][0]
        m = dict(name=node.get('name', ''), mat=js['materials'][pr['material']], js=js, binc=binc,
                 pos=to_game(glb.accessor(js, binc, pr['attributes']['POSITION']).astype(np.float64)),
                 nor=to_game(glb.accessor(js, binc, pr['attributes']['NORMAL']).astype(np.float64)),
                 uv=glb.accessor(js, binc, pr['attributes']['TEXCOORD_0']).astype(np.float64) if 'TEXCOORD_0' in pr['attributes'] else None,
                 tri=glb.accessor(js, binc, pr['indices']).reshape(-1, 3).astype(np.int64))
        m['nor'] /= np.linalg.norm(m['nor'], axis=1, keepdims=True) + 1e-12
        meshes.append(m)
        print('mesh', m['name'], 'material', m['mat'].get('name'), len(m['tri']), 'tris')
print('total', sum(len(m['tri']) for m in meshes), 'triangles')

# ------------------------------------------------------------------ textures and materials
tex_dir = os.path.join(DATA, 'Textures', *TEX_DIR.split('/'))
mat_dir = os.path.join(DATA, 'Materials', *TEX_DIR.split('/'))
os.makedirs(tex_dir, exist_ok=True)
paint_bgsm = open(os.environ.get('PAINT_BGSM') or os.path.join(mat_dir, 'RockboyPaint.BGSM'), 'rb').read()   # PAINT_BGSM: the template material when <data> is a fresh folder


def image(m, texinfo):
    js = m['js']
    bv = js['bufferViews'][js['images'][js['textures'][texinfo['index']]['source']]['bufferView']]
    off = bv.get('byteOffset', 0)
    return Image.open(io.BytesIO(m['binc'][off:off + bv['byteLength']]))


def material_name(mat):
    n = mat.get('name', '')
    if n == 'rueda_repuesto':
        return 'MilitaryWheel'
    return PREFIX + 'Paint' if n == 'body' else PREFIX + n[:1].upper() + n[1:]


work = tempfile.mkdtemp()
done = set()
MASKED = set()                                     # materials with alpha test (glTF alphaMode MASK)
for m in meshes:
    mat = m['mat']
    if mat.get('name') in ('glass', 'rueda_repuesto') or material_name(mat) in done:     # spares: MilitaryWheel's own textures
        continue
    name = material_name(mat)
    done.add(name)
    pbr = mat['pbrMetallicRoughness']
    masked = mat.get('alphaMode') == 'MASK'        # alpha test: the colour map keeps its alpha (BC7), the BGSM and the shapes test it
    albedo = image(m, pbr['baseColorTexture']).convert('RGBA' if masked else 'RGB')
    if 'metallicRoughnessTexture' in pbr:
        mr = np.asarray(image(m, pbr['metallicRoughnessTexture']).convert('RGB'), np.float64) / 255
    else:
        mr = np.zeros((4, 4, 3))
        mr[..., 1], mr[..., 2] = pbr.get('roughnessFactor', 1.0), pbr.get('metallicFactor', 0.0)
    spec = np.zeros(mr.shape, np.uint8)
    spec[..., 0] = np.round((0.08 + 0.72 * mr[..., 2]) * 255)
    spec[..., 1] = np.round((1 - mr[..., 1]) * 255)
    # a normal map in the glb is already in the game's tangent frame (bake_normals.py: red along dP/du, green along dP/dv)
    normal = image(m, mat['normalTexture']).convert('RGB') if 'normalTexture' in mat else Image.new('RGB', (64, 64), (128, 128, 255))
    src = {'_d': albedo, '_s': Image.fromarray(spec).resize((max(albedo.width // 2, 4), max(albedo.height // 2, 4)), Image.LANCZOS),
           '_n': normal}
    fmt = {'_d': 'BC7_UNORM', '_s': 'BC5_UNORM', '_n': 'BC5_UNORM'}
    for suf, im in src.items():
        p = os.path.join(work, name + suf + '.png')
        im.save(p)
        cov = ['--keep-coverage', '0.5'] if masked and suf == '_d' else []      # the cut-out keeps its size in the smaller mips
        subprocess.run([TEXCONV, '-nologo', '-y', '--ignore-srgb', '-m', '0', '-f', fmt[suf]] + cov + ['-o', tex_dir, p], check=True,
                       stdout=subprocess.DEVNULL)
        print('  texture', name + suf, im.size, fmt[suf])
    if True:
        new = patch_bgsm(paint_bgsm, {TEX_DIR + '/RockboyPaint%s.dds' % s: TEX_DIR + '/%s%s.dds' % (name, s) for s in ('_d', '_n', '_s')})
        if masked:                                 # BGSM v2: alpha test reference at byte 41, alpha test flag at byte 42
            new = bytearray(new)
            new[41], new[42] = int(round(mat.get('alphaCutoff', 0.5) * 255)), 1
            new = bytes(new)
            MASKED.add(name)
        open(os.path.join(mat_dir, name + '.BGSM'), 'wb').write(new)
        print('  material', name + '.BGSM')

# ------------------------------------------------------------------ nif
d, bsv, strs, blocks, _ = parse(TEMPLATE)
assert bsv == 130
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
q += 6 * nb + 8
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


assert [b[0] for b in blocks[:4]] == ['NiNode', 'BSXFlags', 'bhkNPCollisionObject', 'bhkPhysicsSystem']
NODE_FIRST = next(k for k in range(1, len(blocks)) if blocks[k][0] == 'NiNode')
WHEEL_FIRST = next(k for k in range(1, len(blocks)) if blocks[k][0] == 'NiNode' and strs[struct.unpack_from('<i', ob[k], 0)[0]] == 'WheelFL')
assert blocks[4][0] == 'BSTriShape' and blocks[5][0] == 'BSLightingShaderProperty' and blocks[6][0] == 'BSShaderTextureSet'
VD, = struct.unpack_from('<Q', ob[4], 100)
assert (VD >> 44) & 0x400 == 0 and (VD & 0xF) == 5
# template glass: shape -> BSEffectShaderProperty + NiAlphaProperty
glass_k = next(k for k in range(4, WHEEL_FIRST) if blocks[k][0] == 'BSTriShape' and struct.unpack_from('<3i', ob[k], 88)[2] >= 0)
assert blocks[glass_k + 1][0] == 'BSEffectShaderProperty' and blocks[glass_k + 2][0] == 'NiAlphaProperty'
sp_t, ts_t = bytes(ob[5]), bytes(ob[6])
assert strs[struct.unpack_from('<i', sp_t, 4)[0]].endswith('\\RockboyPaint.BGSM')


def enc(v):
    return np.clip(np.round((np.asarray(v, dtype=np.float64) + 1.0) * 127.5), 0, 255).astype(np.uint8)


def pack_vertices(p, t_uv, n, t, b):
    """t, b: the "tangent" and "bitangent" fields (meshprep.uv_tangents)."""
    out = np.zeros((len(p), 20), np.uint8)
    out[:, 0:6] = np.frombuffer(np.asarray(p, np.float16).tobytes(), np.uint8).reshape(-1, 6)
    out[:, 6:8] = np.frombuffer(np.asarray(b[:, 0], np.float16).tobytes(), np.uint8).reshape(-1, 2)
    out[:, 8:12] = np.frombuffer(np.asarray(t_uv, np.float16).tobytes(), np.uint8).reshape(-1, 4)
    out[:, 12:15] = enc(n)
    out[:, 15] = enc(b[:, 1])
    out[:, 16:19] = enc(t)
    out[:, 19] = enc(b[:, 2])
    return out.tobytes()


def shape_bytes(name, shader_ref, alpha_ref, p, t_uv, n, t, tg, bt):
    c = (p.min(0) + p.max(0)) / 2
    r = float(np.linalg.norm(p - c, axis=1).max())
    vb = pack_vertices(p, t_uv, n, tg, bt)
    tb = np.asarray(t, np.uint16).tobytes()
    h = struct.pack('<iIi', S(name), 0, -1) + struct.pack('<I', FLAGS)
    h += struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', 1, 0, 0, 0, 1, 0, 0, 0, 1) + struct.pack('<f', 1.0) + struct.pack('<i', -1)
    h += struct.pack('<4f', *[float(x) for x in c], r)
    h += struct.pack('<3i', -1, shader_ref, alpha_ref)
    h += struct.pack('<Q', VD)
    h += struct.pack('<I', len(t)) + struct.pack('<H', len(p)) + struct.pack('<I', len(vb) + len(tb))
    return h + vb + tb


def textureset(name):
    ts = ts_t
    for letter in 'dns':
        old = ('textures\\' + TEX_DIR.replace('/', '\\') + '\\RockboyPaint_%s.dds' % letter).encode()
        new = ('textures\\' + TEX_DIR.replace('/', '\\') + '\\%s_%s.dds' % (name, letter)).encode()
        a, b = struct.pack('<I', len(old)) + old, struct.pack('<I', len(new)) + new
        assert ts.count(a) == 1, old
        ts = ts.replace(a, b)
    return ts


new = []
BASE = 4
root_kids = []
node_jobs = []          # (block index of the node to fill in later, name, centre, child shape indices)
for m in meshes:
    is_glass = m['mat'].get('name') == 'glass'
    own_node = m['name'].startswith('node:')
    centre = (m['pos'].min(0) + m['pos'].max(0)) / 2 if own_node else np.zeros(3)
    pos = m['pos'] - centre
    uv = m['uv'] if m['uv'] is not None else np.zeros((len(pos), 2))
    shapes = []
    # tangent frame on the whole mesh (per chunk it would jump at the chunk borders); the normal map bake uses the same function
    tg, bt = meshprep.uv_tangents(pos, uv, m['nor'], m['tri'])
    for i, sel in enumerate(meshprep.split_chunks(m['tri'], MAX_VERTS, pos[m['tri']][:, :, 1].mean(1))):
        t, p_, n_, u_, tg_, bt_ = meshprep.compact(m['tri'][sel], pos, m['nor'], uv, tg, bt)
        base = BASE + len(new)
        label = '%s:%s%d' % (PREFIX, m['name'].replace('node:', ''), i)
        if is_glass:
            new.append(('BSTriShape', shape_bytes(label, base + 1, base + 2, p_, u_, n_, t, tg_, bt_)))
            new.append(('BSEffectShaderProperty', bytes(ob[glass_k + 1])))
            new.append(('NiAlphaProperty', bytes(ob[glass_k + 2])))
        else:
            name = material_name(m['mat'])
            masked = name in MASKED
            new.append(('BSTriShape', shape_bytes(label, base + 1, base + 3 if masked else -1, p_, u_, n_, t, tg_, bt_)))
            sp = bytearray(sp_t)
            struct.pack_into('<i', sp, 4, S('Materials\\' + TEX_DIR.replace('/', '\\') + '\\%s.BGSM' % name))
            struct.pack_into('<i', sp, 40, base + 2)
            new.append(('BSLightingShaderProperty', bytes(sp)))
            new.append(('BSShaderTextureSet', textureset(name)))
            if masked:                             # the template glass's NiAlphaProperty with blending off and the test on (0x12EC, ref 128)
                ap = bytearray(ob[glass_k + 2])
                struct.pack_into('<HB', ap, len(ap) - 3, 0x12EC, 128)
                new.append(('NiAlphaProperty', bytes(ap)))
        shapes.append(base)
        print('  shape', label, len(t), 'tris', len(p_), 'verts')
    if own_node:
        k = BASE + len(new)
        new.append(('NiNode', None))
        node_jobs.append((k, m['name'][5:], centre, shapes))
        root_kids.append(k)
    else:
        root_kids += shapes
for k, name, centre, shapes in node_jobs:
    b = bytearray(ob[WHEEL_FIRST][:76])
    struct.pack_into('<i', b, 0, S(name))
    struct.pack_into('<3f', b, 16, *[float(x) for x in centre])
    struct.pack_into('<I', b, 72, len(shapes))
    b += struct.pack('<%di' % len(shapes), *shapes)
    new[k - BASE] = ('NiNode', bytes(b))
    print('  node', name, 'at', centre.round(1))

# wheel group: built new. Template blocks: the WheelFL node and its beadlock shape / shader / texture set.
import importlib  # noqa: E402
tmpl_node = bytes(ob[WHEEL_FIRST][:76])
bead = next(k for k in range(WHEEL_FIRST, len(ob)) if blocks[k][0] == 'BSTriShape' and strs[struct.unpack_from('<i', ob[k], 0)[0]].endswith('_beadlock'))
_, bead_sh, _ = struct.unpack_from('<3i', ob[bead], 88)
bead_ts, = struct.unpack_from('<i', ob[bead_sh], 40)
WVD, = struct.unpack_from('<Q', ob[bead], 100)
assert WVD == VD, 'wheel vertex format differs from the body one'
GENERATORS = {'classic': ('wheel_classic', 'make_classic_wheel_assets', 'ClassicWheel'),
              'military': ('wheel_military', 'make_military_wheel_assets', 'MilitaryWheel')}
wb = []
wheel_nodes = []
first = BASE + len(new)
for corner in ('FL', 'FR', 'RL', 'RR'):
    hub = WHEELS['hubs'][corner]
    s_out = -1.0 if corner[1] == 'L' else 1.0                 # outward along x: left wheels look to -x
    shapes = []
    jobs = []
    for set_name in WHEELS['sets']:
        if set_name not in GENERATORS:
            jobs.append((set_name, None, None, None, None, None, None, None))      # a set of the template, copied and scaled below
            continue
        gen, assets, tex = GENERATORS[set_name]
        g = importlib.import_module(gen)
        a = importlib.import_module(assets)
        P, N, T, mats, Pd = g.build(1.0, inner_face_ax=-9.0)
        # generator space: axle on y, outward +y -> node space: axle on x, outward s_out; the axis swap is a reflection, so the winding flips
        P = np.stack([s_out * P[:, 1], P[:, 0], P[:, 2]], 1).astype(np.float64)
        N = np.stack([s_out * N[:, 1], N[:, 0], N[:, 2]], 1).astype(np.float64)
        T = T[:, [0, 2, 1]] if s_out > 0 else T
        P[:, 0] -= (P[:, 0].min() + P[:, 0].max()) / 2           # the hub at the node
        P *= WHEELS['radius'] / np.abs(P[:, 1:]).max()
        # a textured generator maps its own UVs (mirrored on the left wheels); the palette ones put each material on a flat cell
        uv = g.uv(Pd, s_out, mats) if hasattr(g, 'uv') else np.array([a.cell_uv(m_) for m_ in mats], np.float64)
        tg, bt = meshprep.uv_tangents(P, uv, N, T)
        jobs.append((set_name, tex, P, uv, N, T, tg, bt))
    node_k = first + len(wb)
    wb.append(('NiNode', None))
    for set_name, tex, P, uv, N, T, tg, bt in jobs:
        k = first + len(wb)
        if tex is None:
            # the template's Wheel_<corner>_<set> shape: vertex positions scaled to the radius, its shader and texture set copied as they are
            src = next(j for j in range(WHEEL_FIRST, len(ob)) if blocks[j][0] == 'BSTriShape' and strs[struct.unpack_from('<i', ob[j], 0)[0]] == 'Wheel_%s_%s' % (corner, set_name))
            b = bytearray(ob[src])
            nv, = struct.unpack_from('<H', b, 112)
            vb = np.frombuffer(bytes(b[118:118 + nv * 20]), np.uint8).reshape(-1, 20).copy()
            pos = np.frombuffer(vb[:, 0:6].tobytes(), np.float16).reshape(-1, 3).astype(np.float64)
            f = WHEELS['radius'] / np.abs(pos[:, 1:]).max()
            pos *= f
            vb[:, 0:6] = np.frombuffer(pos.astype(np.float16).tobytes(), np.uint8).reshape(-1, 6)
            b[118:118 + nv * 20] = vb.tobytes()
            cx_, cy_, cz_, r_ = struct.unpack_from('<4f', b, 72)
            struct.pack_into('<4f', b, 72, cx_ * f, cy_ * f, cz_ * f, r_ * f)
            struct.pack_into('<i', b, 0, S('Wheel_%s_%s' % (corner, set_name)))
            _, sh_src, _ = struct.unpack_from('<3i', ob[src], 88)
            ts_src, = struct.unpack_from('<i', ob[sh_src], 40)
            struct.pack_into('<3i', b, 88, -1, k + 1, -1)
            wb.append(('BSTriShape', bytes(b)))
            sp = bytearray(ob[sh_src])
            struct.pack_into('<i', sp, 40, k + 2)
            wb.append(('BSLightingShaderProperty', bytes(sp)))
            wb.append(('BSShaderTextureSet', bytes(ob[ts_src])))
            shapes.append(k)
            print('  wheel', corner, set_name, '(template, x%.3f)' % f, 'at', hub)
            continue
        wb.append(('BSTriShape', shape_bytes('Wheel_%s_%s' % (corner, set_name), k + 1, -1, P, uv, N, T, tg, bt)))
        sp = bytearray(ob[bead_sh])
        struct.pack_into('<i', sp, 4, S('Materials\\' + TEX_DIR.replace('/', '\\') + '\\%s.BGSM' % tex))
        struct.pack_into('<i', sp, 40, k + 2)
        wb.append(('BSLightingShaderProperty', bytes(sp)))
        ts = bytes(ob[bead_ts])
        for letter in 'dns':
            old = ('textures\\' + TEX_DIR.replace('/', '\\') + '\\BeadlockWheel_%s.dds' % letter).encode()
            nw = ('textures\\' + TEX_DIR.replace('/', '\\') + '\\%s_%s.dds' % (tex, letter)).encode()
            o_, n_ = struct.pack('<I', len(old)) + old, struct.pack('<I', len(nw)) + nw
            assert ts.count(o_) == 1, old
            ts = ts.replace(o_, n_)
        wb.append(('BSShaderTextureSet', ts))
        shapes.append(k)
        print('  wheel', corner, set_name, len(T), 'tris at', hub)
    b = bytearray(tmpl_node)
    struct.pack_into('<i', b, 0, S('Wheel' + corner))
    struct.pack_into('<3f', b, 16, *[float(x) for x in hub])
    struct.pack_into('<I', b, 72, len(shapes))
    b += struct.pack('<%di' % len(shapes), *shapes)
    wb[node_k - first] = ('NiNode', bytes(b))
    wheel_nodes.append(node_k)

root = bytearray(ob[0])
nex, = struct.unpack_from('<I', root, 4)
cpos = 8 + 4 * nex + 4 + 4 + 52 + 4
nc, = struct.unpack_from('<I', root, cpos)
assert len(root) == cpos + 4 + 4 * nc
kids = root_kids + wheel_nodes
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
dst = os.path.join(DATA, 'Meshes', *TEX_DIR.split('/'), PREFIX + '.nif')
os.makedirs(os.path.dirname(dst), exist_ok=True)
open(dst, 'wb').write(out)
print('wrote', dst, len(out), 'bytes,', len(allb), 'blocks')
