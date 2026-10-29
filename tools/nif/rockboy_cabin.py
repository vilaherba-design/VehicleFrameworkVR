"""Rockboy v7 cabin: open the windshield of the aligned TRELLIS body and glaze it.

TRELLIS turned the windshield of the reference (glass with a wire guard behind it) into opaque bars. The triangles in the windshield opening, close
to its plane, are removed and replaced by a glass pane (both faces, so it shows from the seat too); the build script gives it the vanilla car glass.
The front side windows are open in the reference and stay open.

Measured on the aligned mesh (TRELLIS.2 1536, seed 42; game units, cross sections at the driver's x and across the windshield): the windshield
plane runs from the roof's front edge (y 24, z 142) down to the cowl (y 57, z 114.6); the opening is x +-50 at the top and +-53 at the bottom.

Usage: rockboy_cabin.py <aligned.glb (align_trellis.py)> <out.glb> [normal map png, for the body]
Writes two meshes: "body" (the TRELLIS material and texture) and "glass".
"""
import sys

import numpy as np

import glb
import glbwrite
import meshprep

TOP, BOTTOM = np.array([24.0, 142.0]), np.array([57.0, 114.6])     # (y, z) of the windshield's top and bottom edges
HALF_TOP, HALF_BOTTOM = 50.0, 53.0                                    # half widths of the opening
CUT_DEPTH = 9.0                                                       # remove what lies this close to the plane...
CUT_INSET = 1.5                                                       # ...and this far inside the frame
GLASS_OVERLAP = 1.5                                                   # the pane reaches into the frame
ISLAND_TRIS = 150                                                     # loose pieces smaller than this go after the cuts
# TRELLIS's front interior, removed (triangle centres inside this box): doors' inner skins at x +-59.5, floor z 60, roof lining z 141, rear seat
# backs y -71, dash y ~52
INTERIOR_HALF, INTERIOR_HALF_UPPER, INTERIOR_SILL = 58.2, 53.0, 107.0     # the new door panels are at x +-58.3
INTERIOR_Y, INTERIOR_Z = (-73.0, 56.0), (61.5, 144.5)
# TRELLIS's bull bar and bumper, removed (triangle centres ahead of the nose face at y 148.5, below the bonnet)
FRONT_KIT_Y, FRONT_KIT_Z, FRONT_KIT_HALF = 152.5, 100.0, 88.0

# Other meshes: CABIN_JSON=<file> overrides any of the measurements above (keys: top, bottom, half_top, half_bottom, interior_half,
# interior_half_upper, interior_sill, interior_y, interior_z, front_kit_y; front_kit_y = null keeps the generated bull bar)
import json as _json
import os as _os
if _os.environ.get('CABIN_JSON'):
    _c = _json.load(open(_os.environ['CABIN_JSON']))
    TOP = np.array(_c.get('top', TOP), float)
    BOTTOM = np.array(_c.get('bottom', BOTTOM), float)
    HALF_TOP, HALF_BOTTOM = _c.get('half_top', HALF_TOP), _c.get('half_bottom', HALF_BOTTOM)
    INTERIOR_HALF = _c.get('interior_half', INTERIOR_HALF)
    INTERIOR_HALF_UPPER = _c.get('interior_half_upper', INTERIOR_HALF_UPPER)
    INTERIOR_SILL = _c.get('interior_sill', INTERIOR_SILL)
    INTERIOR_Y, INTERIOR_Z = tuple(_c.get('interior_y', INTERIOR_Y)), tuple(_c.get('interior_z', INTERIOR_Z))
    if 'front_kit_y' in _c:
        FRONT_KIT_Y = 1e9 if _c['front_kit_y'] is None else _c['front_kit_y']
    # cut_boxes: [[x0, x1, y0, y1, z0, z1], ...] removed (triangle centres), where modelled parts replace generated ones
    CUT_BOXES = _c.get('cut_boxes', [])
else:
    CUT_BOXES = []


def to_game(v):
    return np.stack([-v[:, 0], v[:, 2], v[:, 1]], axis=1)


def plane_coords(P):
    """(s along the slope from the top edge, 0..1 at the bottom edge; d = distance from the plane, + outwards)."""
    along = BOTTOM - TOP
    L = np.linalg.norm(along)
    u = along / L
    n = np.array([-u[1], u[0]])                 # outward: forward and up
    if n[1] < 0:
        n = -n
    q = P[:, 1:3] - TOP
    return q @ u / L, q @ n, L, u, n


def half_width(s):
    return HALF_TOP + (HALF_BOTTOM - HALF_TOP) * s


def main(argv):
    if len(argv) not in (2, 3):
        sys.exit(__doc__)
    src, dst = argv[:2]
    normal_png = argv[2] if len(argv) == 3 else None
    js, binc = glb.load(src)
    pr = js['meshes'][0]['primitives'][0]
    G = to_game(glb.accessor(js, binc, pr['attributes']['POSITION']).astype(np.float64))
    N = to_game(glb.accessor(js, binc, pr['attributes']['NORMAL']).astype(np.float64))
    uv = glb.accessor(js, binc, pr['attributes']['TEXCOORD_0']).astype(np.float64)
    tri = glb.accessor(js, binc, pr['indices']).reshape(-1, 3).astype(np.int64)

    s, d, L, u, n = plane_coords(G)
    inset = CUT_INSET / L
    inside = (np.abs(d) < CUT_DEPTH) & (s > inset) & (s < 1 - inset) & (np.abs(G[:, 0]) < half_width(s) - CUT_INSET)
    keep = inside[tri].sum(1) < 2            # two corners inside is enough: long slivers from the pillars to the bars would stay as spikes
    print('windshield: removed', int((~keep).sum()), 'triangles')
    # TRELLIS's own front interior (seats, steering wheel, dash): the new interior replaces it. The door, floor and roof skins stay (the new panels sit
    # just inside them); above the sills the box narrows so the pillars stay.
    # Below the sills a triangle goes if any corner is inside (the TRELLIS door skins are not flat and would poke through the new panels)
    lim_x = np.where(G[:, 2] > INTERIOR_SILL, INTERIOR_HALF_UPPER, INTERIOR_HALF)
    vin = (np.abs(G[:, 0]) < lim_x) & (G[:, 1] > INTERIOR_Y[0]) & (G[:, 1] < INTERIOR_Y[1]) & (G[:, 2] > INTERIOR_Z[0]) & (G[:, 2] < INTERIOR_Z[1])
    c = G[tri].mean(1)
    old = np.where(c[:, 2] < INTERIOR_SILL, vin[tri].any(1), vin[tri].all(1))
    keep &= ~old
    print('old interior: removed', int(old.sum()), 'triangles')
    # TRELLIS's melted bull bar and bumper (ahead of the nose face): the modelled front kit replaces them
    front = (c[:, 1] > FRONT_KIT_Y) & (c[:, 2] < FRONT_KIT_Z) & (np.abs(c[:, 0]) < FRONT_KIT_HALF)
    if FRONT_KIT_Y < 1e8:
        front |= (c[:, 1] > 140) & (c[:, 2] < 80) & (np.abs(c[:, 0]) > 68)     # the bumper's end pieces, outside the wings
    keep &= ~front
    print('front bull bar: removed', int(front.sum()), 'triangles')
    caja = np.zeros(len(c), bool)
    for x0, x1, y0, y1, z0, z1 in CUT_BOXES:
        caja |= (c[:, 0] > x0) & (c[:, 0] < x1) & (c[:, 1] > y0) & (c[:, 1] < y1) & (c[:, 2] > z0) & (c[:, 2] < z1)
    keep &= ~caja
    if CUT_BOXES:
        print('cut boxes: removed', int(caja.sum()), 'triangles')
    # loose bits left by the cuts (spikes by the pillars, shreds at the roof edge): small islands go
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    t2 = tri[keep]
    key = np.round(G / 0.05).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    e = np.concatenate([t2[:, [0, 1]], t2[:, [1, 2]], t2[:, [2, 0]]])
    a = coo_matrix((np.ones(len(e)), (inv[e[:, 0]], inv[e[:, 1]])), shape=(inv.max() + 1,) * 2)
    _, lab = connected_components(a, directed=False)
    tl = lab[inv[t2[:, 0]]]
    cnt = np.bincount(tl, minlength=lab.max() + 1)
    small = cnt[tl] < ISLAND_TRIS
    idx = np.flatnonzero(keep)
    keep[idx[small]] = False
    print('loose islands: removed', int(small.sum()), 'triangles in', len(np.unique(tl[small])), 'islands')
    tri, G, N, uv = meshprep.compact(tri[keep], G, N, uv)

    # glass pane: a grid on the plane, a little past the opening on every side, both faces
    ns, nx = 6, 12
    ss = np.linspace(-GLASS_OVERLAP / L, 1 + GLASS_OVERLAP / L, ns + 1)
    gp = []
    for si in ss:
        hw = half_width(si) + GLASS_OVERLAP
        yz = TOP + (BOTTOM - TOP) * si
        for x in np.linspace(-hw, hw, nx + 1):
            gp.append((x, yz[0], yz[1]))
    gp = np.array(gp)
    q = []
    for i in range(ns):
        for j in range(nx):
            a, b, c, e = i * (nx + 1) + j, i * (nx + 1) + j + 1, (i + 1) * (nx + 1) + j, (i + 1) * (nx + 1) + j + 1
            q += [(a, c, b), (b, c, e)]
    q = np.array(q)
    out_n = np.array([0, n[0], n[1]])
    # orient the outer face's winding to the outward normal (counter-clockwise seen from outside)
    f = np.cross(gp[q[0, 1]] - gp[q[0, 0]], gp[q[0, 2]] - gp[q[0, 0]])
    if f @ out_n < 0:
        q = q[:, ::-1]
    gpos = np.concatenate([gp + out_n * 0.15, gp - out_n * 0.15])
    gnor = np.concatenate([np.tile(out_n, (len(gp), 1)), np.tile(-out_n, (len(gp), 1))])
    gtri = np.concatenate([q, q[:, ::-1] + len(gp)])
    print('glass', len(gtri), 'triangles')

    images = []
    for im in js.get('images', []):
        bv = js['bufferViews'][im['bufferView']]
        images.append(binc[bv.get('byteOffset', 0):bv.get('byteOffset', 0) + bv['byteLength']])
    mat = dict(js['materials'][pr['material']])
    mat['name'] = 'body'
    # texture indices -> image indices (glbwrite makes one texture per image, in order)
    pbr = mat.get('pbrMetallicRoughness', {})
    for k in ('baseColorTexture', 'metallicRoughnessTexture'):
        if k in pbr:
            pbr[k] = {'index': js['textures'][pbr[k]['index']]['source']}
    if normal_png:
        # tangent-space normal map in the game's frame (red along dP/du, green along dP/dv), see the build script
        images.append(open(normal_png, 'rb').read())
        mat['normalTexture'] = {'index': len(images) - 1}
    glass = {'name': 'glass', 'pbrMetallicRoughness': {'baseColorFactor': [0.55, 0.62, 0.62, 0.25], 'metallicFactor': 0.0, 'roughnessFactor': 0.3},
             'alphaMode': 'BLEND', 'doubleSided': False}
    glbwrite.write(dst, [dict(name='body', pos=G, nor=N, uv=uv, tri=tri, material=0),
                         dict(name='glass', pos=gpos, nor=gnor, uv=None, tri=gtri, material=1)], [mat, glass], images)
    print('wrote', dst, len(tri), 'body triangles')


if __name__ == '__main__':
    main(sys.argv[1:])
