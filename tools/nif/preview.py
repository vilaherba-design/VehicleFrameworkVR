"""Quick shaded preview of a nif (all BSTriShapes, wheel nodes applied): preview.py <nif> <out.png>"""
import struct, sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from shapes import load
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
d, bsv, strs, blocks, sh = load(sys.argv[1])
# node translations for wheel nodes: block index -> translate; children from nodes
trans = {}
for i, (t, o, s) in enumerate(blocks):
    if t == 'NiNode' and i != 0:
        trans[i] = np.array(struct.unpack_from('<3f', d, o + 16))
parent = {}
for i, (t, o, s) in enumerate(blocks):
    if t == 'NiNode' and i != 0:
        nc, = struct.unpack_from('<I', d, o + 72)
        for r in struct.unpack_from('<%di' % nc, d, o + 76):
            parent[r] = i
polys = []; cols = []
colmap = {}
for s in sh:
    pos = s['pos'].astype(float) + trans.get(parent.get(s['idx']), 0)
    col = colmap.setdefault(s['name'].split('_')[0] if s['name'] else '', plt.cm.tab20(len(colmap) % 20))
    polys.append((pos, s['tri'], col))
views = {'side': (1, 2, 0), 'front': (0, 2, 1), 'top': (0, 1, 2)}
fig, ax = plt.subplots(1, 3, figsize=(24, 7))
for a, (name, (ix, iy, idepth)) in zip(ax, views.items()):
    P = []; C = []; Z = []
    for pos, tri, col in polys:
        tp = pos[tri]
        n = np.cross(tp[:, 1] - tp[:, 0], tp[:, 2] - tp[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
        light = np.abs(n[:, idepth]) * 0.6 + 0.4
        P.append(tp[:, :, [ix, iy]]); Z.append(tp[:, :, idepth].mean(1))
        C.append(np.array(col)[None, :3] * light[:, None])
    P = np.concatenate(P); Z = np.concatenate(Z); C = np.concatenate(C)
    o = np.argsort(Z)
    a.add_collection(PolyCollection(P[o], facecolors=C[o], edgecolors='none'))
    a.set_xlim(P[:, :, 0].min(), P[:, :, 0].max()); a.set_ylim(P[:, :, 1].min(), P[:, :, 1].max()); a.set_aspect('equal'); a.set_title(name)
plt.savefig(sys.argv[2], dpi=60)
