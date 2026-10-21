"""Procedural beadlock off-road wheel for the sedan (low-poly port of the "Rueda off-road" model made with Claude Design).

Design space: metres, wheel axle on +z (outer face at +z), radial plane x/y. `build()` returns the mesh in the sedan's wheel-local nif space (game units, axle on y,
outward = s along y) with one material tag per vertex, so the caller can map each material onto a texture region:

  rubber  tyre carcass and tread blocks          mud   sidewall lugs and rib
  bronze  rim flange, spokes, hub                dark  inner well, beadlock ring, hub cap
  steel   beadlock bolts and lug nuts

Usage for a quick look: python3 wheel_beadlock.py out.png
"""
import math
import sys

import numpy as np

GAME_UNITS_PER_M = 69.99125

# ---- parameters of the design (metres)
RT = 0.452                 # tread base radius
BLOCKS = 28                # tread block positions around the tyre (the design has 34)
CARCASS_SEG = 48
PROFILE = [(0.214, 0.112), (0.235, 0.128), (0.27, 0.142), (0.32, 0.152), (0.37, 0.156), (0.41, 0.152), (0.435, 0.142), (0.448, 0.125), (RT, 0.10), (RT, 0.0)]
TYRE_HALF_WIDTH = 0.156


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], float)


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], float)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)


def tr(x, y, z):
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def polar(a, r, z, tilt_x=0.0, tilt_y=0.0):
    return rz(a) @ tr(r, 0, z) @ rx(tilt_x) @ ry(tilt_y)


class Mesh:
    def __init__(self):
        self.P, self.N, self.T, self.mat = [], [], [], []
        self.count = 0

    def add(self, mat, P, N, T):
        P = np.asarray(P, float).reshape(-1, 3)
        N = np.asarray(N, float).reshape(-1, 3)
        T = np.asarray(T, int).reshape(-1, 3) + self.count
        self.P.append(P)
        self.N.append(N)
        self.T.append(T)
        self.mat += [mat] * len(P)
        self.count += len(P)

    def lathe(self, mat, prof, seg, ref, smooth=True, center=(0.0, 0.0)):
        """Surface of revolution about +z of the polyline `prof` of (r, z). `ref` is a (r, z) point inside the solid: normals point away from it."""
        prof = np.asarray(prof, float)
        n = len(prof)
        th = np.arange(seg) * 2 * math.pi / seg
        ct, st = np.cos(th), np.sin(th)

        def outward(t_r, t_z, pr, pz):
            nr, nz = t_z, -t_r
            if nr * (pr - ref[0]) + nz * (pz - ref[1]) < 0:
                nr, nz = -nr, -nz
            ln = math.hypot(nr, nz) or 1.0
            return nr / ln, nz / ln

        rings = []      # (list of (r, z, nr, nz) per ring)
        quads = []      # (ring a, ring b)
        if smooth:
            for i in range(n):
                a, b = prof[max(i - 1, 0)], prof[min(i + 1, n - 1)]
                nr, nz = outward(b[0] - a[0], b[1] - a[1], prof[i][0], prof[i][1])
                rings.append((prof[i][0], prof[i][1], nr, nz))
            quads = [(i, i + 1) for i in range(n - 1)]
        else:
            for i in range(n - 1):
                a, b = prof[i], prof[i + 1]
                nr, nz = outward(b[0] - a[0], b[1] - a[1], (a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                rings.append((a[0], a[1], nr, nz))
                rings.append((b[0], b[1], nr, nz))
            quads = [(2 * i, 2 * i + 1) for i in range(n - 1)]
        P, N = [], []
        for (r, z, nr, nz) in rings:
            for j in range(seg):
                P.append((center[0] + r * ct[j], center[1] + r * st[j], z))
                N.append((nr * ct[j], nr * st[j], nz))
        T = []
        for (a, b) in quads:
            for j in range(seg):
                j2 = (j + 1) % seg
                A0, A1, B0, B1 = a * seg + j, a * seg + j2, b * seg + j, b * seg + j2
                T += [(A0, A1, B1), (A0, B1, B0)]
        self.add(mat, P, N, T)

    def disc(self, mat, radius, z, normal_z, seg, center=(0.0, 0.0)):
        th = np.arange(seg) * 2 * math.pi / seg
        P = [(center[0], center[1], z)] + [(center[0] + radius * math.cos(t), center[1] + radius * math.sin(t), z) for t in th]
        N = [(0, 0, normal_z)] * (seg + 1)
        T = [(0, 1 + j, 1 + (j + 1) % seg) for j in range(seg)]
        self.add(mat, P, N, T)

    def box(self, mat, size, m, skip=()):
        hx, hy, hz = size[0] / 2, size[1] / 2, size[2] / 2
        faces = {
            '+x': ((1, 0, 0), [(hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz)]),
            '-x': ((-1, 0, 0), [(-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz), (-hx, hy, -hz)]),
            '+y': ((0, 1, 0), [(-hx, hy, -hz), (-hx, hy, hz), (hx, hy, hz), (hx, hy, -hz)]),
            '-y': ((0, -1, 0), [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)]),
            '+z': ((0, 0, 1), [(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]),
            '-z': ((0, 0, -1), [(-hx, -hy, -hz), (-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz)]),
        }
        rot = m[:3, :3]
        for name, (nrm, corners) in faces.items():
            if name in skip:
                continue
            C = np.array(corners, float)
            P = C @ rot.T + m[:3, 3]
            N = np.tile(rot @ np.array(nrm, float), (4, 1))
            self.add(mat, P, N, [(0, 1, 2), (0, 2, 3)])

    def bar(self, mat, p1, p2, w, d):
        dx, dy, dz = (p2[i] - p1[i] for i in range(3))
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        mid = [(p1[i] + p2[i]) / 2 for i in range(3)]
        m = tr(*mid) @ rz(math.atan2(dy, dx)) @ ry(-math.atan2(dz, math.hypot(dx, dy)))
        self.box(mat, (length, w, d), m)


def design_mesh():
    mesh = Mesh()
    # ---- tyre carcass: bead on one side, tread in the middle, bead on the other side
    full = PROFILE + [(r, -z) for (r, z) in reversed(PROFILE[:-1])]
    mesh.lathe('rubber', full, CARCASS_SEG, ref=(0.33, 0.0), smooth=True)
    # ---- tread blocks and sidewall lugs
    block = (0.022, 0.05, 0.075)
    block_s = (0.022, 0.044, 0.06)
    lug_l = (0.075, 0.042, 0.016)
    lug_s = (0.045, 0.036, 0.014)
    h = math.pi / BLOCKS
    for i in range(BLOCKS):
        a = i / BLOCKS * 2 * math.pi
        mesh.box('rubber', block, polar(a, RT + 0.009, 0.05, 0.32), skip=('-x',))
        mesh.box('rubber', block, polar(a + h, RT + 0.009, -0.05, -0.32), skip=('-x',))
        mesh.box('rubber', block_s, polar(a + h, RT + 0.006, 0.118, -0.2, -0.35), skip=('-x',))
        mesh.box('rubber', block_s, polar(a, RT + 0.006, -0.118, 0.2, 0.35), skip=('-x',))
        long_first = i % 2 == 1
        l1, l2 = (lug_l, lug_s) if long_first else (lug_s, lug_l)
        r1, r2 = (0.412, 0.425) if long_first else (0.425, 0.412)
        mesh.box('mud', l1, polar(a + h, r1, 0.152, 0, 0.28), skip=('-x', '+z'))
        mesh.box('mud', l2, polar(a, r2, -0.152, 0, -0.28), skip=('-x', '-z'))
    # sidewall rim-protector rib (a small ring on the outer sidewall)
    rib = [(0.248 - 0.006, 0.133), (0.248, 0.139), (0.248 + 0.006, 0.133)]
    mesh.lathe('mud', rib, 40, ref=(0.248, 0.125), smooth=False)
    # ---- rim
    mesh.disc('dark', 0.2, -0.04, 1.0, 40)                                                   # inner well seen between the spokes
    mesh.lathe('bronze', [(0.19, 0.10), (0.205, 0.112), (0.205, 0.116), (0.188, 0.114), (0.18, 0.104)], 40, ref=(0.19, 0.108), smooth=False)   # outer flange
    mesh.lathe('dark', [(0.196, 0.114), (0.196, 0.136), (0.2, 0.141), (0.238, 0.141), (0.242, 0.136), (0.242, 0.114)], 40, ref=(0.22, 0.125), smooth=False)   # beadlock ring
    for i in range(24):
        a = i / 24 * 2 * math.pi
        mesh.lathe('steel', [(0.0055, 0.141), (0.0055, 0.149), (0.0, 0.149)], 4, ref=(0.0, 0.145), smooth=False, center=(math.cos(a) * 0.219, math.sin(a) * 0.219))
    # ---- six split-Y spokes
    for i in range(6):
        t = i / 6 * 2 * math.pi

        def p(r, a, z):
            return (math.cos(a) * r, math.sin(a) * r, z)
        mesh.bar('bronze', p(0.07, t, 0.098), p(0.125, t, 0.092), 0.034, 0.03)
        for sgn in (-1, 1):
            mesh.bar('bronze', p(0.12, t, 0.092), p(0.198, t + sgn * 0.2, 0.084), 0.024, 0.028)
    # ---- hub, hub cap, lug nuts
    mesh.lathe('bronze', [(0.082, 0.065), (0.078, 0.115)], 28, ref=(0.0, 0.09), smooth=False)
    mesh.disc('bronze', 0.078, 0.115, 1.0, 28)
    mesh.lathe('dark', [(0.0, 0.135), (0.03, 0.134), (0.045, 0.128), (0.05, 0.115), (0.05, 0.112)], 28, ref=(0.0, 0.12), smooth=False)
    for i in range(6):
        a = i / 6 * 2 * math.pi + math.pi / 6
        mesh.lathe('steel', [(0.009, 0.111), (0.009, 0.133), (0.0, 0.133)], 6, ref=(0.0, 0.122), smooth=False, center=(math.cos(a) * 0.064, math.sin(a) * 0.064))
    return mesh


def build(s, inner_face_ax=-9.6):
    """Wheel-local nif mesh (game units): axle on y, `s` (+1/-1) is the outward direction, tyre inner face at ax = inner_face_ax
    (the default puts the innermost sidewall lug at the stock tyre's inner face, -10.6).
    Returns P (n,3) float32, N (n,3) float32, tris (m,3) int, mats (list of n tags), design positions (n,3) in metres."""
    mesh = design_mesh()
    Pd = np.concatenate(mesh.P)
    Nd = np.concatenate(mesh.N)
    T = np.concatenate(mesh.T)
    ax0 = inner_face_ax + TYRE_HALF_WIDTH * GAME_UNITS_PER_M
    # design (x, y, z) -> nif (x, s*(z*S + ax0), y*S); a reflection when s = +1, so the winding is fixed below from the normals
    def to_nif(v, translate=True):
        out = np.empty_like(v)
        out[:, 0] = v[:, 0]
        out[:, 2] = v[:, 1]
        out[:, 1] = s * v[:, 2]
        return out
    P = to_nif(Pd) * GAME_UNITS_PER_M
    P[:, 1] += s * ax0
    N = to_nif(Nd)
    # drop degenerate triangles, then orient every triangle along its vertex normals
    a, b, c = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
    f = np.cross(b - a, c - a)
    area = np.linalg.norm(f, axis=1)
    keep = area > 1e-9
    T, f = T[keep], f[keep]
    nsum = N[T[:, 0]] + N[T[:, 1]] + N[T[:, 2]]
    flip = np.einsum('ij,ij->i', f, nsum) < 0
    T[flip] = T[flip][:, [0, 2, 1]]
    return P.astype(np.float32), N.astype(np.float32), T, list(mesh.mat), Pd


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    P, N, T, mats, Pd = build(1.0)
    print('verts', len(P), 'tris', len(T), 'extent', P.min(0).round(1), P.max(0).round(1))
    colors = {'rubber': (0.16, 0.15, 0.14), 'mud': (0.29, 0.25, 0.21), 'bronze': (0.61, 0.51, 0.35), 'dark': (0.15, 0.15, 0.15), 'steel': (0.71, 0.70, 0.66)}
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
        ax.add_collection(PolyCollection(Q[sel][:, :, [0, 2]], facecolors=np.clip(col[sel] * shade[sel] * 1.3, 0, 1), edgecolors='none'))
        ax.set_xlim(-40, 40)
        ax.set_ylim(-40, 40)
        ax.set_aspect('equal')
        ax.set_title(title)
        ax.axis('off')
    plt.savefig(sys.argv[1] if len(sys.argv) > 1 else 'wheel.png', dpi=70)
