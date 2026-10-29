"""Procedural military wheel for the Army Wagon (the wheels of Axel's ChatGPT sheet): a tall mud-terrain tyre with big square tread blocks that
step down the shoulders onto the sidewalls, a rusty steel disc rim with a ring of round holes, a centre hub and six lug nuts.

Design space: metres, wheel axle on +z (outer face at +z), radial plane x/y, like wheel_beadlock.py (whose Mesh helper it reuses).
`build()` returns the mesh in wheel-local nif space (game units, axle on y, outward = s along y) with one material tag per vertex
(rubber, steel, dark). Unlike the palette wheels it is textured: `uv()` maps every vertex onto one disc painted by
tools/make_military_wheel_assets.py (sidewall lettering, dust, rust), a planar projection along the axle.

Usage for a quick look: python3 wheel_military.py out.png
"""
import math
import sys

import numpy as np

from wheel_beadlock import GAME_UNITS_PER_M, Mesh, polar

RT = 0.47                  # tread base radius (scaled to the vehicle's radius by the builder)
BLOCKS = 20                # tread block positions around the tyre
SEG = 32
HALF_WIDTH = 0.17          # half the tyre width at the sidewall lugs
# tyre section, bead (inner) to tread, as (r, z): a tall square-shouldered sidewall
PROFILE = [(0.25, 0.122), (0.29, 0.146), (0.35, 0.157), (0.42, 0.152), (0.455, 0.132), (0.468, 0.09), (RT, 0.0)]
UV_R = 0.44                # disc radius in UV (the texture's square holds the disc)
LETTER_R = (0.33, 0.40)    # design radii of the lettering band on the outer sidewall
RIM_R = 0.252              # rim edge: rubber outside (the texture)
HOLE_RING, HOLE_R = 0.176, 0.019
GROOVE_UV = (0.035, 0.035)  # a corner of the texture, outside the disc: packed mud


def design_mesh(blocks=None, seg=None):
    """the wheel; blocks/seg lower for the spare wheels"""
    BLOCKS_, SEG_ = blocks or BLOCKS, seg or SEG
    mesh = Mesh()
    full = PROFILE + [(r, -z) for (r, z) in reversed(PROFILE[:-1])]
    mesh.lathe('rubber', full, SEG_, ref=(0.34, 0.0), smooth=True)
    h = math.pi / BLOCKS_
    BW = 1.8 / BLOCKS_                                        # block width along the tread
    for i in range(BLOCKS_):
        a = i / BLOCKS_ * 2 * math.pi
        # tread, three rows: a centre block and, staggered, a big lug each side reaching over the edge, with deep grooves between
        mesh.box('lug', (0.05, BW * 0.8, 0.05), polar(a, RT + 0.023, 0.0), skip=('-x',))               # centre row
        mesh.box('lug', (0.05, BW * 0.95, 0.075), polar(a + h, RT + 0.022, 0.075, 0.0, -0.15), skip=('-x',))
        mesh.box('lug', (0.05, BW * 0.95, 0.075), polar(a + h, RT + 0.022, -0.075, 0.0, 0.15), skip=('-x',))
        mesh.box('lug', (0.045, BW * 0.85, 0.05), polar(a, RT + 0.012, 0.125, 0.0, -0.35), skip=('-x',))   # edge lugs: a toothed outline
        mesh.box('lug', (0.045, BW * 0.85, 0.05), polar(a, RT + 0.012, -0.125, 0.0, 0.35), skip=('-x',))
        # shoulder blocks stepping down onto the sidewall, alternating long and short like the sheet's tyre
        for sgn, off in ((1, h), (-1, 0.0)):
            long_ = (i % 2 == 0) if sgn > 0 else (i % 2 == 1)
            mesh.box('lug', (0.065 if long_ else 0.045, BW * 0.85, 0.034), polar(a + off, 0.432 if long_ else 0.442, sgn * 0.15, 0.0, sgn * 0.75),
                     skip=('-z',) if sgn > 0 else ('+z',))
            if sgn > 0:                                       # the flat sidewall lugs only on the outer face (the inner one is hardly seen)
                mesh.box('lug', (0.04 if long_ else 0.026, BW * 0.73, 0.014), polar(a + off, 0.405 if long_ else 0.413, 0.158), skip=('-z',))
    # rim: lip, dished disc with a raised ring, the hub
    # a big rim like the sheet's (about half the tyre's radius): lip, a dished disc with a raised ring and ten holes, the hub
    # a rim protector rib on the outer sidewall
    mesh.lathe('rubber', [(0.268, 0.135), (0.276, 0.145), (0.286, 0.142)], SEG_, ref=(0.276, 0.13), smooth=False)
    mesh.lathe('steel', [(0.236, 0.112), (0.252, 0.121), (0.257, 0.129), (0.246, 0.131), (0.234, 0.124)], SEG_, ref=(0.243, 0.118), smooth=False)
    disc = [(0.236, 0.112), (0.2, 0.088), (0.15, 0.083), (0.137, 0.093), (0.112, 0.093), (0.098, 0.082), (0.082, 0.078)]
    mesh.lathe('steel', disc, SEG_, ref=(0.15, 0.06), smooth=False)
    for i in range(10):                                        # the ten lightening holes: dark, with a raised bevelled lip around each
        a = i / 10 * 2 * math.pi
        c_ = (math.cos(a) * HOLE_RING, math.sin(a) * HOLE_RING)
        mesh.disc('dark', HOLE_R, 0.0864, 1.0, 8, center=c_)
        mesh.lathe('steel', [(HOLE_R, 0.0864), (HOLE_R + 0.004, 0.0915), (HOLE_R + 0.012, 0.0868)], 8, ref=(HOLE_R + 0.005, 0.08),
                   smooth=False, center=c_)
    # the step from the dish up to the flange, and the bead seat ring
    mesh.lathe('steel', [(0.204, 0.0905), (0.2, 0.0975), (0.192, 0.0975), (0.187, 0.0895)], SEG_, ref=(0.196, 0.085), smooth=False)
    mesh.lathe('steel', [(0.082, 0.078), (0.078, 0.097), (0.066, 0.104), (0.042, 0.104)], 28, ref=(0.0, 0.08), smooth=False)
    mesh.lathe('steel', [(0.042, 0.104), (0.038, 0.118), (0.026, 0.126), (0.0, 0.128)], 20, ref=(0.0, 0.1), smooth=True)
    for i in range(12):                                         # rim bolts around the disc (the sheet's rim)
        a = (i + 0.5) / 12 * 2 * math.pi
        mesh.lathe('steel', [(0.0095, 0.098), (0.0095, 0.109), (0.0, 0.111)], 6, ref=(0.0, 0.1), smooth=False,
                   center=(math.cos(a) * 0.219, math.sin(a) * 0.219))      # bolt heads standing on the flange
    for i in range(6):
        a = i / 6 * 2 * math.pi
        mesh.lathe('steel', [(0.0085, 0.104), (0.0085, 0.116), (0.006, 0.119), (0.0, 0.119)], 6, ref=(0.0, 0.11), smooth=False,
                   center=(math.cos(a) * 0.055, math.sin(a) * 0.055))
    mesh.disc('dark', 0.26, -0.11, -1.0, SEG_)                 # the back of the wheel, seen from under the car
    for ref_ in ((0.4, 0.0), (0.0, 0.0)):                     # the rim barrel, both faces (single-sided in the game): no see-through gap at the bead
        mesh.lathe('dark', [(0.253, -0.125), (0.253, 0.114)], SEG_, ref=ref_, smooth=False)
    # the lugs' edges a little soft: each face's corner normals lean out towards the corner, so the edges shade round while the faces stay
    # flat in the middle (LUG_SOFT 0 = hard boxes)
    off = 0
    for k, Pk in enumerate(mesh.P):
        if len(Pk) == 4 and mesh.mat[off] == 'lug' and LUG_SOFT > 0:
            d = Pk - Pk.mean(0)
            d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-12
            n_ = mesh.N[k] + LUG_SOFT * d
            mesh.N[k] = n_ / (np.linalg.norm(n_, axis=1, keepdims=True) + 1e-12)
        off += len(Pk)
    return mesh


LUG_SOFT = 0.3             # how much the lug edges are rounded in the shading (Axel: a little)
ATLAS = True              # atlas UVs (build() fills _UV); False: the old planar disc
DISC_R, DISC_UV = 0.52, 0.24
STRIP_U = 0.97            # the strip holds a turn in u 0..0.97, so a triangle across the seam stays under 1
DARK_UV = (0.49, 0.49)
_UV = None


def uv(Pd, s_out, mats=None):
    if ATLAS and _UV is not None and len(_UV) == len(Pd):
        out = _UV.copy()
        if s_out < 0:                                         # the left wheels are mirror images: flip the outer disc so the lettering reads
            a_ = (out[:, 0] < 0.5) & (out[:, 1] < 0.5) & ~((out[:, 0] == DARK_UV[0]) & (out[:, 1] == DARK_UV[1]))
            out[a_, 0] = 0.5 - out[a_, 0]
        return out
    return uv_disc(Pd, s_out, mats)


def uv_disc(Pd, s_out, mats=None):
    """Planar UV along the axle onto the painted disc. The radius shrinks a little with z so the tread band keeps some area in UV. Left wheels
    (s_out < 0) are a mirror image in the nif, so u is flipped there to keep the lettering readable."""
    f = UV_R / RT * (1.0 - 0.06 * (Pd[:, 2] + HALF_WIDTH) / (2 * HALF_WIDTH))
    u = Pd[:, 0] * f * (1.0 if s_out > 0 else -1.0)
    out = np.stack([0.5 + u, 0.5 - Pd[:, 1] * f], 1)
    if mats is not None:                                      # the dark parts (holes, barrel, back) on a hole's dark texel
        fz = UV_R / RT * (1.0 - 0.06 * (0.0855 + HALF_WIDTH) / (2 * HALF_WIDTH))
        dark = np.array([m == 'dark' for m in mats])
        out[dark] = (0.5 + HOLE_RING * fz, 0.5)
        # the carcass between the tread blocks (groove bottoms) on the mud texel in the corner; the blocks keep the disc (black rubber)
        rr = np.hypot(Pd[:, 0], Pd[:, 1])
        groove = np.array([m == 'rubber' for m in mats]) & (rr > 0.40)
        out[groove] = GROOVE_UV
    return out


def build(s, inner_face_ax=-9.0):
    """Wheel-local nif mesh (game units): axle on y, `s` (+1/-1) is the outward direction, tyre inner face at ax = inner_face_ax.
    Returns P (n,3) float32, N (n,3) float32, tris (m,3) int, mats (list of n tags), design positions (n,3) in metres."""
    mesh = design_mesh()
    Pd = np.concatenate(mesh.P)
    Nd = np.concatenate(mesh.N)
    T = np.concatenate(mesh.T)
    ax0 = inner_face_ax + HALF_WIDTH * GAME_UNITS_PER_M

    def to_nif(v):
        out = np.empty_like(v)
        out[:, 0] = v[:, 0]
        out[:, 2] = v[:, 1]
        out[:, 1] = s * v[:, 2]
        return out
    P = to_nif(Pd) * GAME_UNITS_PER_M
    P[:, 1] += s * ax0
    N = to_nif(Nd)
    a, b, c = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
    f = np.cross(b - a, c - a)
    keep = np.linalg.norm(f, axis=1) > 1e-9
    T, f = T[keep], f[keep]
    nsum = N[T[:, 0]] + N[T[:, 1]] + N[T[:, 2]]
    flip = np.einsum('ij,ij->i', f, nsum) < 0
    T[flip] = T[flip][:, [0, 2, 1]]
    mats = list(mesh.mat)
    if ATLAS:
        # atlas UVs: every triangle gets its own corners; the outer face (rim and flank) on a disc in the top-left quarter, the inner face
        # on a disc in the top-right quarter, the tread band and the lugs' sides on a strip across the bottom half; dark parts on a black texel
        global _UV
        Ts = T.reshape(-1)
        P, N, Pd, Nd_ = P[Ts], N[Ts], Pd[Ts], Nd[Ts]
        mats = [mats[i] for i in Ts]
        T = np.arange(len(Ts)).reshape(-1, 3)
        fn = Nd_.reshape(-1, 3, 3).sum(1)
        fn /= np.linalg.norm(fn, axis=1, keepdims=True) + 1e-12
        cls = np.where(fn[:, 2] > 0.45, 0, np.where(fn[:, 2] < -0.45, 1, 2))
        uv_ = np.zeros((len(Pd), 2))
        fA = DISC_UV / DISC_R
        for k, cx in ((0, 0.25), (1, 0.75)):
            sel = np.repeat(cls == k, 3)
            uv_[sel, 0] = cx + Pd[sel, 0] * fA * (1 if k == 0 else -1)
            uv_[sel, 1] = 0.25 - Pd[sel, 1] * fA
        sel3 = cls == 2
        th = (np.arctan2(Pd[:, 1], Pd[:, 0]) / (2 * math.pi)) % 1.0
        th3 = th.reshape(-1, 3)
        wrap = (th3.max(1) - th3.min(1)) > 0.5
        th3[wrap] = np.where(th3[wrap] < 0.5, th3[wrap] + 1.0, th3[wrap])
        th = th3.reshape(-1)
        sel = np.repeat(sel3, 3)
        uv_[sel, 0] = th[sel] * STRIP_U
        uv_[sel, 1] = 0.52 + (np.clip(Pd[sel, 2], -0.2, 0.2) + 0.2) / 0.4 * 0.46
        dark = np.array([m == 'dark' for m in mats])
        uv_[dark] = DARK_UV
        _UV = uv_
    return P.astype(np.float32), N.astype(np.float32), T, mats, Pd


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    P, N, T, mats, Pd = build(1.0)
    print('verts', len(P), 'tris', len(T), 'extent', P.min(0).round(1), P.max(0).round(1))
    colors = {'rubber': (0.13, 0.12, 0.11), 'steel': (0.45, 0.33, 0.2), 'dark': (0.06, 0.06, 0.06)}
    fig, axs = plt.subplots(1, 3, figsize=(18, 6))
    for ax, (az, el, title) in zip(axs, [(180, 0, 'outer face'), (150, 25, '3/4'), (90, 0, 'tread profile')]):
        a, e = math.radians(az), math.radians(el)
        Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        Rx = np.array([[1, 0, 0], [0, math.cos(e), -math.sin(e)], [0, math.sin(e), math.cos(e)]])
        R = Rx @ Rz
        Q = P[T] @ R.T
        n = np.cross(Q[:, 1] - Q[:, 0], Q[:, 2] - Q[:, 0])
        nn = n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
        front = nn[:, 1] < 0
        order = np.argsort(-Q[:, :, 1].mean(1))
        light = np.array([-0.4, -0.8, 0.5])
        light /= np.linalg.norm(light)
        col = np.array([colors[mats[t[0]]] for t in T])
        shade = (np.clip(-(nn @ light), 0, 1) * 0.7 + 0.3)[:, None]
        sel = [i for i in order if front[i]]
        ax.add_collection(PolyCollection(Q[sel][:, :, [0, 2]], facecolors=np.clip(col[sel] * shade[sel] * 1.6, 0, 1), edgecolors='none'))
        ax.set_xlim(-38, 38)
        ax.set_ylim(-38, 38)
        ax.set_aspect('equal')
        ax.set_title(title)
        ax.axis('off')
    plt.savefig(sys.argv[1] if len(sys.argv) > 1 else 'wheel_military.png', dpi=70)
