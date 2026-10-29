"""Align a TRELLIS.2 car mesh (one textured primitive, with its own wheels) to a vehicle's wheel nodes and cut its tyres out.

The mesh's wheels are found by fitting a circle to the bottom profile of each tyre. The mesh is then scaled (x to the track, y and z to the wheelbase)
and moved so its axles land on the framework's wheel nodes, and the triangles inside each tyre's cylinder are removed (the framework adds its own
wheels). Decorative spare wheels far from the axles stay.

Writes a glb in GAME UNITS with glTF axes (x = -game x, y = game z, z = game y), same texture(s) and material, ready for build_rockboy_v7.py.

Usage: align_trellis.py <in.glb> <out.glb> [hub_x hub_y hub_z]   (default: the Rockboy's 68.5 97.3 36.4)
Writes the transform to <out>.align.json. With the environment variable ALIGN_LIKE=<some>.align.json the wheels are not fitted again and that
transform is used (the undecimated mesh of the same TRELLIS run, for baking normals, must land exactly on the decimated one).
"""
import os
import json
import struct
import sys

import numpy as np

import glb
import meshprep

CUT_MARGIN = 1.04                 # cylinder radius = fitted tyre radius * this
CUT_INNER = 14.0                  # from this far inside the wheel centre plane outwards (keeps the axle stubs and the frame)


def to_game(v):
    """glTF (y up, the TRELLIS car faces +z) -> game (x right, y forward, z up)."""
    return np.stack([-v[:, 0], v[:, 2], v[:, 1]], axis=1)


def from_game(v):
    return np.stack([-v[:, 0], v[:, 2], v[:, 1]], axis=1)


def fit_wheels(G):
    """{(side, end): (x, y, z, r)} for side, end in +-1 (end +1 = front)."""
    z0, half_w = G[:, 2].min(), G[:, 0].max()
    out = {}
    for sx in (-1, 1):
        for sy in (-1, 1):
            m = (np.sign(G[:, 0]) == sx) & (np.sign(G[:, 1]) == sy) & (np.abs(G[:, 0]) > 0.6 * half_w) & (G[:, 2] < z0 + 0.06)
            Q = G[m]
            yc = Q[Q[:, 2] < z0 + 0.004][:, 1].mean()
            pts = []
            for y in np.arange(yc - 0.09, yc + 0.09, 0.004):
                s = Q[np.abs(Q[:, 1] - y) < 0.002]
                if len(s):
                    pts.append((y, s[:, 2].min()))
            pts = np.array(pts)
            # algebraic circle fit: y^2 + z^2 + a y + b z + c = 0
            A = np.stack([pts[:, 0], pts[:, 1], np.ones(len(pts))], 1)
            a, b, c = np.linalg.lstsq(A, -(pts[:, 0] ** 2 + pts[:, 1] ** 2), rcond=None)[0]
            cy, cz = -a / 2, -b / 2
            r = np.sqrt(cy * cy + cz * cz - c)
            tyre = G[(np.sign(G[:, 0]) == sx) & (np.abs(G[:, 1] - cy) < 0.5 * r) & (G[:, 2] < cz) & (np.abs(G[:, 0]) > 0.6 * half_w)]
            out[(sx, sy)] = np.array([(tyre[:, 0].min() + tyre[:, 0].max()) / 2, cy, cz, r])
    return out


def write_glb(path, js, binc, pr, pos, nor, uv, tri):
    """Same document, with the primitive's geometry replaced (images and other views copied)."""
    acc_ids = [pr['attributes']['POSITION'], pr['attributes']['NORMAL'], pr['attributes']['TEXCOORD_0'], pr['indices']]
    arrays = [np.asarray(pos, np.float32), np.asarray(nor, np.float32), np.asarray(uv, np.float32), np.asarray(tri, np.uint32).reshape(-1)]
    replaced = {js['accessors'][a]['bufferView']: (a, arr) for a, arr in zip(acc_ids, arrays)}
    chunks, views, o = [], [], 0
    for i, v in enumerate(js['bufferViews']):
        if i in replaced:
            a, arr = replaced[i]
            data = arr.tobytes()
            acc = js['accessors'][a]
            acc['count'] = len(arr)
            acc.pop('byteOffset', None)
            if acc['type'] == 'VEC3':
                acc['min'], acc['max'] = arr.min(0).tolist(), arr.max(0).tolist()
            else:
                acc.pop('min', None), acc.pop('max', None)
            v = {k: x for k, x in v.items() if k != 'byteStride'}
        else:
            data = binc[v.get('byteOffset', 0):v.get('byteOffset', 0) + v['byteLength']]
        pad = (-len(data)) % 4
        views.append(dict(v, byteOffset=o, byteLength=len(data)))
        chunks.append(data + b'\0' * pad)
        o += len(data) + pad
    js['bufferViews'] = views
    js['buffers'] = [{'byteLength': o}]
    for n in js['nodes']:
        for k in ('matrix', 'rotation', 'scale', 'translation'):
            n.pop(k, None)
    bin_ = b''.join(chunks)
    jb = json.dumps(js, separators=(',', ':')).encode()
    jb += b' ' * ((-len(jb)) % 4)
    with open(path, 'wb') as f:
        f.write(struct.pack('<4sII', b'glTF', 2, 12 + 8 + len(jb) + 8 + len(bin_)))
        f.write(struct.pack('<II', len(jb), 0x4E4F534A) + jb)
        f.write(struct.pack('<II', len(bin_), 0x004E4942) + bin_)


def main(argv):
    if len(argv) not in (2, 5):
        sys.exit(__doc__)
    src, dst = argv[:2]
    hub_x, hub_y, hub_z = map(float, argv[2:5]) if len(argv) == 5 else (68.5, 97.3, 36.4)
    js, binc = glb.load(src)
    assert len(js['meshes']) == 1 and len(js['meshes'][0]['primitives']) == 1
    pr = js['meshes'][0]['primitives'][0]
    node = [n for n in js['nodes'] if 'mesh' in n][0]
    assert not any(k in node for k in ('matrix', 'rotation', 'scale', 'translation')), node
    pos = glb.accessor(js, binc, pr['attributes']['POSITION']).astype(np.float64)
    nor = glb.accessor(js, binc, pr['attributes']['NORMAL']).astype(np.float64)
    uv = glb.accessor(js, binc, pr['attributes']['TEXCOORD_0']).astype(np.float64)
    tri = glb.accessor(js, binc, pr['indices']).reshape(-1, 3).astype(np.int64)
    G, N = to_game(pos), to_game(nor)
    # ROT_Z=<grados>: gira el modelo alrededor del eje vertical antes de alinear (salidas multivista que miran a otro eje)
    if os.environ.get('ROT_Z'):
        a = np.radians(float(os.environ['ROT_Z']))
        Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        G, N = G @ Rz.T, N @ Rz.T
    print('in', len(G), 'verts', len(tri), 'tris')

    if os.environ.get('ALIGN_LIKE'):
        prm = json.load(open(os.environ['ALIGN_LIKE']))
        print('transform from', os.environ['ALIGN_LIKE'])
    else:
        W = fit_wheels(G)
        for k, w in W.items():
            print('  wheel', k, 'centre', w[:3].round(4), 'radius', round(w[3], 4))
        yf = np.mean([W[(sx, 1)][1] for sx in (-1, 1)])
        yr = np.mean([W[(sx, -1)][1] for sx in (-1, 1)])
        prm = dict(s_x=hub_x / np.mean([abs(w[0]) for w in W.values()]), s_yz=2 * hub_y / (yf - yr), y_mid=(yf + yr) / 2,
                   z_hub=np.mean([w[2] for w in W.values()]), radius=np.mean([w[3] for w in W.values()]))
    json.dump({k: float(v) for k, v in prm.items()}, open(dst + '.align.json', 'w'), indent=1)
    s_x, s_yz, zh = prm['s_x'], prm['s_yz'], prm['z_hub']
    print('scale x %.1f, y/z %.1f game units per mesh unit' % (s_x, s_yz))
    G = np.stack([G[:, 0] * s_x, (G[:, 1] - prm['y_mid']) * s_yz, (G[:, 2] - zh) * s_yz + hub_z], 1)
    N = N / np.array([s_x, s_yz, s_yz])           # normals: inverse transpose of the scale
    N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    radius = prm['radius'] * s_yz
    print('tyre radius %.1f game units' % radius)

    inside = np.zeros(len(G), bool)
    for sx in (-1, 1):
        for sy in (-1, 1):
            d = np.hypot(G[:, 1] - sy * hub_y, G[:, 2] - hub_z)
            inside |= (d < radius * CUT_MARGIN) & (np.sign(G[:, 0]) == sx) & (np.abs(G[:, 0]) > hub_x - CUT_INNER)
    keep = ~inside[tri].all(1)
    print('tyres: removed', int((~keep).sum()), 'of', len(tri), 'triangles')
    tri, G, N, uv = meshprep.compact(tri[keep], G, N, uv)
    print('out', len(G), 'verts', len(tri), 'tris, extent', G.min(0).round(1), G.max(0).round(1))
    write_glb(dst, js, binc, pr, from_game(G), from_game(N), uv, tri)
    print('wrote', dst)


if __name__ == '__main__':
    main(sys.argv[1:])
