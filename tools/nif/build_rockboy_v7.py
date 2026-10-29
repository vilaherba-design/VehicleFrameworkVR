"""Rockboy v7 (0.1.79): replace the body of the built Rockboy.nif with the TRELLIS.2 cabin glb, keeping everything else.

Pipeline (see docs in each tool; the work scripts named here live in _rockboy_v7/scripts next to the repository):
  TRELLIS.2 (1536 cascade, seed 42, from Axel's side reference; once at 340k faces for the game, once at 1.5M for baking) -> align_trellis.py
  (onto the wheel nodes, tyres cut out; the 1.5M mesh with ALIGN_LIKE) -> proyectar_refs.py (Axel's four references projected onto the texture)
  -> rockboy_cabin.py (windshield opened and glazed, old interior and melted bull bar removed) -> hornear_normales.py + normales_tangente.py
  (normal map from the 1.5M mesh) -> rockboy_cabin.py again with the normal map -> this script, with the interior (interior_base.py +
  sd_img2img.py + proyectar_interior.py) and the exterior kit (exterior_kit.py: bull bar, winch, lamps, grille, tail lights, ladder...).

Kept from the template nif byte for byte: root node, BSX flags, collision object, physics system and the whole wheel group (three wheel sets per node).
Replaced: the body shapes, their shaders and texture sets, and the glass.

Every glb mesh becomes shapes (split at MAX_VERTS vertices). By material:
  - "glass": the template's glass blocks (vanilla CarPrewarSanctuary_Glass01.BGEM with its alpha property).
  - "body": RockboyPaint (the existing RockboyPaint.BGSM already points at RockboyPaint_{d,n,s}.dds).
  - any other textured material <m>: Rockboy<M> textures and a Rockboy<M>.BGSM made from RockboyPaint.BGSM with the texture paths replaced.
A mesh named "node:<Name>" goes under its own NiNode <Name> (a copy of a wheel node) placed at the mesh's bounding box centre, so it can be moved later
(the steering wheel).

Textures: albedo -> _d (BC7), metallic/roughness -> _s (R specular = 0.08 + 0.72 metal, G gloss = 1 - roughness, half size, BC5), the glb's normal
map (if any) -> _n (BC5), otherwise a flat one. Vertex tangents follow the UVs in the game's convention (meshprep.uv_tangents), so a normal map baked
in that frame lights correctly.

Usage: build_rockboy_v7.py <cabin.glb> <template Rockboy.nif (0.1.78)> <mod data dir> <texconv.exe>
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

if len(sys.argv) != 5:
    sys.exit(__doc__)
GLB, TEMPLATE, DATA, TEXCONV = sys.argv[1:5]
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
paint_bgsm = open(os.path.join(mat_dir, 'RockboyPaint.BGSM'), 'rb').read()


def image(m, texinfo):
    js = m['js']
    bv = js['bufferViews'][js['images'][js['textures'][texinfo['index']]['source']]['bufferView']]
    off = bv.get('byteOffset', 0)
    return Image.open(io.BytesIO(m['binc'][off:off + bv['byteLength']]))


def material_name(mat):
    n = mat.get('name', '')
    return 'RockboyPaint' if n == 'body' else 'Rockboy' + n[:1].upper() + n[1:]


work = tempfile.mkdtemp()
done = set()
for m in meshes:
    mat = m['mat']
    if mat.get('name') == 'glass' or material_name(mat) in done:
        continue
    name = material_name(mat)
    done.add(name)
    pbr = mat['pbrMetallicRoughness']
    albedo = image(m, pbr['baseColorTexture']).convert('RGB')
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
        subprocess.run([TEXCONV, '-nologo', '-y', '--ignore-srgb', '-m', '0', '-f', fmt[suf], '-o', tex_dir, p], check=True, stdout=subprocess.DEVNULL)
        print('  texture', name + suf, im.size, fmt[suf])
    if name != 'RockboyPaint':
        new = patch_bgsm(paint_bgsm, {TEX_DIR + '/RockboyPaint%s.dds' % s: TEX_DIR + '/%s%s.dds' % (name, s) for s in ('_d', '_n', '_s')})
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
WHEEL_FIRST = next(k for k in range(1, len(blocks)) if blocks[k][0] == 'NiNode')
assert strs[struct.unpack_from('<i', ob[WHEEL_FIRST], 0)[0]] == 'WheelFL'
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
        label = 'Rockboy:%s%d' % (m['name'].replace('node:', ''), i)
        if is_glass:
            new.append(('BSTriShape', shape_bytes(label, base + 1, base + 2, p_, u_, n_, t, tg_, bt_)))
            new.append(('BSEffectShaderProperty', bytes(ob[glass_k + 1])))
            new.append(('NiAlphaProperty', bytes(ob[glass_k + 2])))
        else:
            name = material_name(m['mat'])
            new.append(('BSTriShape', shape_bytes(label, base + 1, -1, p_, u_, n_, t, tg_, bt_)))
            sp = bytearray(sp_t)
            struct.pack_into('<i', sp, 4, S('Materials\\' + TEX_DIR.replace('/', '\\') + '\\%s.BGSM' % name))
            struct.pack_into('<i', sp, 40, base + 2)
            new.append(('BSLightingShaderProperty', bytes(sp)))
            new.append(('BSShaderTextureSet', textureset(name)))
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
delta = BASE + len(new) - WHEEL_FIRST

# wheel group: same bytes, references moved by delta
wheel_nodes = []
wb = []
for k in range(WHEEL_FIRST, len(ob)):
    t = blocks[k][0]
    b = bytearray(ob[k])
    if t == 'NiNode':
        nc, = struct.unpack_from('<I', b, 72)
        refs = struct.unpack_from('<%di' % nc, b, 76)
        struct.pack_into('<%di' % nc, b, 76, *[r + delta for r in refs])
        wheel_nodes.append(k + delta)
    elif t == 'BSTriShape':
        skin, shader, al = struct.unpack_from('<3i', b, 88)
        struct.pack_into('<3i', b, 88, skin, shader + delta, al)
    elif t == 'BSLightingShaderProperty':
        ts, = struct.unpack_from('<i', b, 40)
        struct.pack_into('<i', b, 40, ts + delta)
    elif t != 'BSShaderTextureSet':          # texture sets hold no references
        raise ValueError('unexpected block in the wheel group: ' + t)
    wb.append((t, bytes(b)))

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
dst = os.path.join(DATA, 'Meshes', *TEX_DIR.split('/'), 'Rockboy.nif')
os.makedirs(os.path.dirname(dst), exist_ok=True)
open(dst, 'wb').write(out)
print('wrote', dst, len(out), 'bytes,', len(allb), 'blocks')
