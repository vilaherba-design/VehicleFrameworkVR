"""Procedural 1950s wheel for the coupe: a smooth bias-ply tyre with a white wall, a red steel rim and a chrome hub cap with a trim ring.

Design space: metres, wheel axle on +z (outer face at +z), radial plane x/y, like wheel_beadlock.py (whose Mesh helper it reuses).
`build()` returns the mesh in wheel-local nif space (game units, axle on y, outward = s along y) with one material tag per vertex:

  rubber  tyre carcass and tread        white  the white wall on the outer sidewall
  red     painted steel rim and flange  chrome hub cap and trim ring
  dark    inner well seen through the rim

Usage for a quick look: python3 wheel_classic.py out.png
"""
import math
import sys

import numpy as np

from wheel_beadlock import GAME_UNITS_PER_M, Mesh

RT = 0.425                 # tread radius (about 30 game units: the coupe's measured wheels)
TYRE_HALF_WIDTH = 0.10
SEG = 56
# tyre section, bead (inner) to tread, as (r, z): a rounded balloon tyre
PROFILE = [(0.268, 0.082), (0.29, 0.096), (0.33, 0.104), (0.37, 0.102), (0.40, 0.092), (0.418, 0.074), (RT, 0.05), (RT, 0.0)]


def design_mesh():
    mesh = Mesh()
    full = PROFILE + [(r, -z) for (r, z) in reversed(PROFILE[:-1])]
    mesh.lathe('rubber', full, SEG, ref=(0.34, 0.0), smooth=True)
    # white wall: a band standing 2 mm proud of the outer sidewall
    mesh.lathe('white', [(0.305, 0.1015), (0.31, 0.1055), (0.375, 0.1045), (0.38, 0.1005)], SEG, ref=(0.34, 0.09), smooth=False)
    # rim: painted flange and well, the dark inner well behind
    mesh.lathe('red', [(0.262, 0.086), (0.27, 0.092), (0.262, 0.096), (0.24, 0.094), (0.205, 0.08), (0.2, 0.06)], SEG, ref=(0.235, 0.07), smooth=False)
    mesh.disc('dark', 0.2, 0.0, 1.0, SEG)
    mesh.lathe('red', [(0.2, 0.06), (0.17, 0.055), (0.155, 0.06)], SEG, ref=(0.18, 0.04), smooth=False)
    # chrome trim ring and dome hub cap
    mesh.lathe('chrome', [(0.205, 0.09), (0.212, 0.098), (0.235, 0.1), (0.25, 0.095)], SEG, ref=(0.225, 0.085), smooth=True)
    cap = [(0.158, 0.062), (0.155, 0.075), (0.14, 0.092), (0.11, 0.104), (0.07, 0.112), (0.03, 0.115), (0.0, 0.116)]
    mesh.lathe('chrome', cap, SEG, ref=(0.0, 0.05), smooth=True)
    return mesh


def build(s, inner_face_ax=-7.0):
    """Wheel-local nif mesh (game units): axle on y, `s` (+1/-1) is the outward direction, tyre inner face at ax = inner_face_ax.
    Returns P (n,3) float32, N (n,3) float32, tris (m,3) int, mats (list of n tags), design positions (n,3) in metres."""
    mesh = design_mesh()
    Pd = np.concatenate(mesh.P)
    Nd = np.concatenate(mesh.N)
    T = np.concatenate(mesh.T)
    ax0 = inner_face_ax + TYRE_HALF_WIDTH * GAME_UNITS_PER_M

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
    return P.astype(np.float32), N.astype(np.float32), T, list(mesh.mat), Pd


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    P, N, T, mats, Pd = build(1.0)
    print('verts', len(P), 'tris', len(T), 'extent', P.min(0).round(1), P.max(0).round(1))
    colors = {'rubber': (0.06, 0.06, 0.06), 'white': (0.92, 0.91, 0.86), 'red': (0.62, 0.08, 0.08), 'chrome': (0.85, 0.85, 0.88), 'dark': (0.1, 0.1, 0.1)}
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
        ax.set_xlim(-35, 35)
        ax.set_ylim(-35, 35)
        ax.set_aspect('equal')
        ax.set_title(title)
        ax.axis('off')
    plt.savefig(sys.argv[1] if len(sys.argv) > 1 else 'wheel_classic.png', dpi=70)
