"""Minimal glb writer for the vehicle tools: meshes in GAME axes (x right, y forward, z up), written with glTF axes (x = -game x, y = game z,
z = game y) so Blender and the other tools read them like the TRELLIS meshes."""
import json
import struct

import numpy as np


def _gl(v):
    v = np.asarray(v, np.float64)
    return np.stack([-v[:, 0], v[:, 2], v[:, 1]], axis=1).astype(np.float32)


def write(path, meshes, materials, images=()):
    """meshes: [dict(name, pos, nor, uv (or None), tri, material)], materials: glTF material dicts (a texture index i refers to images[i]),
    images: PNG bytes."""
    chunks, views, accessors = [], [], []

    def add(data, target=None):
        data = bytes(data)
        o = sum(len(c) for c in chunks)
        chunks.append(data + b'\0' * ((-len(data)) % 4))
        v = {'buffer': 0, 'byteOffset': o, 'byteLength': len(data)}
        if target:
            v['target'] = target
        views.append(v)
        return len(views) - 1

    def acc(arr, kind, comp, target):
        arr = np.ascontiguousarray(arr)
        a = {'bufferView': add(arr.tobytes(), target), 'componentType': comp, 'count': len(arr) if kind != 'SCALAR' else arr.size, 'type': kind}
        if kind == 'VEC3':
            a['min'], a['max'] = arr.min(0).tolist(), arr.max(0).tolist()
        accessors.append(a)
        return len(accessors) - 1

    gmeshes, nodes = [], []
    for m in meshes:
        attrs = {'POSITION': acc(_gl(m['pos']), 'VEC3', 5126, 34962), 'NORMAL': acc(_gl(m['nor']), 'VEC3', 5126, 34962)}
        if m.get('uv') is not None:
            attrs['TEXCOORD_0'] = acc(np.asarray(m['uv'], np.float32), 'VEC2', 5126, 34962)
        ind = acc(np.asarray(m['tri'], np.uint32).reshape(-1), 'SCALAR', 5125, 34963)
        gmeshes.append({'name': m['name'], 'primitives': [{'attributes': attrs, 'indices': ind, 'material': m['material']}]})
        nodes.append({'name': m['name'], 'mesh': len(gmeshes) - 1})
    imgs = [{'bufferView': add(b), 'mimeType': 'image/png'} for b in images]
    js = {'asset': {'version': '2.0', 'generator': 'VehicleFrameworkVR glbwrite'}, 'scene': 0, 'scenes': [{'nodes': list(range(len(nodes)))}],
          'nodes': nodes, 'meshes': gmeshes, 'materials': materials, 'accessors': accessors, 'bufferViews': views,
          'buffers': [{'byteLength': sum(len(c) for c in chunks)}]}
    if imgs:
        js['images'] = imgs
        js['textures'] = [{'source': i} for i in range(len(imgs))]
    bin_ = b''.join(chunks)
    jb = json.dumps(js, separators=(',', ':')).encode()
    jb += b' ' * ((-len(jb)) % 4)
    with open(path, 'wb') as f:
        f.write(struct.pack('<4sII', b'glTF', 2, 12 + 8 + len(jb) + 8 + len(bin_)))
        f.write(struct.pack('<II', len(jb), 0x4E4F534A) + jb)
        f.write(struct.pack('<II', len(bin_), 0x004E4942) + bin_)
