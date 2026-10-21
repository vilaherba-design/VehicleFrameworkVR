"""Mesh clean-up helpers for the glb -> nif converters: vertex welding, vertex compaction, crease-angle normals and splitting into shapes that fit the
nif's 16-bit vertex indices. The decimation itself is the quadric simplifier of meshoptimizer (see meshopt.py)."""
import numpy as np


def weld(pos, tri, q=1e-5):
    """Merge vertices that share a position (to q metres) and drop the triangles that collapse. Returns (pos, tri)."""
    key = np.round(np.asarray(pos, np.float64) / q).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    t = inv[np.asarray(tri).reshape(-1)].reshape(-1, 3)
    ok = (t[:, 0] != t[:, 1]) & (t[:, 1] != t[:, 2]) & (t[:, 0] != t[:, 2])
    return np.asarray(pos)[first], t[ok]


def compact(tri, *arrays):
    """Keep only the vertices the triangles use. Returns (tri, *arrays)."""
    used, inv = np.unique(tri, return_inverse=True)
    return (inv.reshape(-1, 3),) + tuple(None if a is None else a[used] for a in arrays)


def crease_normals(pos, tri, crease_deg=40.0):
    """Per-corner normals averaged (area weighted) over the faces within crease_deg of the corner's own face, like an auto-smooth modifier: smooth
    curved surfaces, hard edges where the faces differ by more than the angle. Vertices are split where their corners disagree.
    Returns (pos2, tri2, nor2)."""
    pos = np.asarray(pos, np.float64)
    tri = np.asarray(tri, np.int64)
    fn = np.cross(pos[tri[:, 1]] - pos[tri[:, 0]], pos[tri[:, 2]] - pos[tri[:, 0]])
    area = np.linalg.norm(fn, axis=1)
    fn = fn / np.maximum(area[:, None], 1e-20)
    cv = tri.reshape(-1)
    cf = np.repeat(np.arange(len(tri)), 3)
    order = np.argsort(cv, kind='stable')
    cv_s, cf_s = cv[order], cf[order]
    starts = np.concatenate([[0], np.flatnonzero(np.diff(cv_s)) + 1])
    deg = np.diff(np.concatenate([starts, [len(cv_s)]]))
    cn = np.zeros((len(cv), 3))
    cosc = np.cos(np.radians(crease_deg))
    for d in np.unique(deg):
        sel = np.flatnonzero(deg == d)
        idx = starts[sel][:, None] + np.arange(d)[None, :]
        f = cf_s[idx]
        c = order[idx]
        n = fn[f]
        w = (np.einsum('gik,gjk->gij', n, n) >= cosc) * area[f][:, None, :]
        cn[c.reshape(-1)] = np.einsum('gij,gjk->gik', w, n).reshape(-1, 3)
    cn /= np.maximum(np.linalg.norm(cn, axis=1, keepdims=True), 1e-20)
    key = np.concatenate([cv[:, None], np.round(cn * 200).astype(np.int64)], axis=1)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return pos[cv[first]], inv.reshape(-1, 3), cn[first]


def split_chunks(tri, max_verts, centroids):
    """Split a triangle list into groups (sorted along `centroids`, a (m,) key per triangle) so that each group uses at most max_verts vertices.
    Returns a list of triangle-index arrays."""
    order = np.argsort(centroids, kind='stable')
    k = max(1, int(np.ceil(len(np.unique(tri)) / max_verts)))
    while True:
        parts = np.array_split(order, k)
        if all(len(np.unique(tri[p])) <= max_verts for p in parts):
            return parts
        k += 1
