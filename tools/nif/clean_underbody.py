"""Clean the underbody of an aligned TRELLIS car (glb from align_trellis.py, game units): the framework draws the wheels, so the generated
suspension, hubs, axles and loose underbody bits go.

Removed (triangle centres, game units, wheel nodes at x +-68.5, y +-97.3, z 36.4):
  - inside each wheel's space: within HUB_CLEAR of the hub axis, between |x| 20 and 95 (knuckles, springs, brake drums, leftover tyre)
  - below FLOOR_Z between the bumpers (axles, diff, driveshaft, exhaust run); the bumpers' shackles (|y| > BUMPER_Y) stay
  - then every loose island under ISLAND_TRIS triangles whose top is below ISLAND_Z (debris left by the cuts)
Usage: clean_underbody.py <aligned.glb> <out.glb>
"""
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

import glb
import meshprep
from align_trellis import to_game, from_game, write_glb

HUB_X, HUB_Y, HUB_Z = 68.5, 97.3, 36.4
HUB_CLEAR = 42.0
FLOOR_Z, BUMPER_Y = 45.0, 145.0
ISLAND_TRIS, ISLAND_Z = 400, 62.0


def main(argv):
    if len(argv) != 2:
        sys.exit(__doc__)
    src, dst = argv
    js, binc = glb.load(src)
    pr = js['meshes'][0]['primitives'][0]
    G = to_game(glb.accessor(js, binc, pr['attributes']['POSITION']).astype(np.float64))
    N = to_game(glb.accessor(js, binc, pr['attributes']['NORMAL']).astype(np.float64))
    uv = glb.accessor(js, binc, pr['attributes']['TEXCOORD_0']).astype(np.float64)
    tri = glb.accessor(js, binc, pr['indices']).reshape(-1, 3).astype(np.int64)
    c = G[tri].mean(1)
    drop = np.zeros(len(tri), bool)
    for sy in (-1, 1):
        d = np.hypot(c[:, 1] - sy * HUB_Y, c[:, 2] - HUB_Z)
        drop |= (d < HUB_CLEAR) & (np.abs(c[:, 0]) > 20) & (np.abs(c[:, 0]) < 95)
    drop |= (c[:, 2] < FLOOR_Z) & (np.abs(c[:, 1]) < BUMPER_Y)
    print('wheel space and underbody: removed', int(drop.sum()), 'of', len(tri))
    tri = tri[~drop]
    # loose islands
    n = len(G)
    e = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])
    # vertices that share a position belong together (TRELLIS splits UV seams)
    key = np.round(G / 0.05).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    a = coo_matrix((np.ones(len(e)), (inv[e[:, 0]], inv[e[:, 1]])), shape=(inv.max() + 1,) * 2)
    _, lab = connected_components(a, directed=False)
    tl = lab[inv[tri[:, 0]]]
    cnt = np.bincount(tl, minlength=lab.max() + 1)
    top = np.full(lab.max() + 1, -1e9)
    np.maximum.at(top, tl, G[tri].max(1)[:, 2])
    small = (cnt[tl] < ISLAND_TRIS) & (top[tl] < ISLAND_Z)
    print('loose low islands: removed', int(small.sum()), 'triangles in', len(np.unique(tl[small])), 'islands')
    tri = tri[~small]
    tri, G, N, uv = meshprep.compact(tri, G, N, uv)
    print('out', len(tri), 'tris, extent', G.min(0).round(1), G.max(0).round(1))
    write_glb(dst, js, binc, pr, from_game(G), from_game(N), uv, tri)


if __name__ == '__main__':
    main(sys.argv[1:])
