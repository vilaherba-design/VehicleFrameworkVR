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


def uv_tangents(pos, uv, nor, tri):
    """Per-vertex tangent frame from the UVs, in the game's convention (measured on vanilla FO4 meshes): the vertex "tangent" field holds dP/dv
    and the "bitangent" field dP/du, both orthogonalised against the normal (normal = tangent x bitangent). A normal map's red channel follows
    dP/du and its green channel dP/dv (DirectX). Vertices without usable UV gradients get an arbitrary frame. Returns (tangent, bitangent)."""
    p = np.asarray(pos, np.float64)
    t_uv = np.asarray(uv, np.float64)
    n = np.asarray(nor, np.float64)
    tri = np.asarray(tri, np.int64)
    e1, e2 = p[tri[:, 1]] - p[tri[:, 0]], p[tri[:, 2]] - p[tri[:, 0]]
    d1, d2 = t_uv[tri[:, 1]] - t_uv[tri[:, 0]], t_uv[tri[:, 2]] - t_uv[tri[:, 0]]
    r = d1[:, 0] * d2[:, 1] - d2[:, 0] * d1[:, 1]
    ok = np.abs(r) > 1e-14
    rr = np.where(ok, r, 1.0)[:, None]
    dpdu = (e1 * d2[:, 1:2] - e2 * d1[:, 1:2]) / rr
    dpdv = (e2 * d1[:, 0:1] - e1 * d2[:, 0:1]) / rr
    # each triangle votes with its direction weighted by its 3D area: raw gradients of nearly degenerate UV triangles are huge and would
    # dominate the vertex average (star-shaped artefacts in a baked normal map)
    area = np.linalg.norm(np.cross(e1, e2), axis=1)[:, None]
    dpdu = dpdu / (np.linalg.norm(dpdu, axis=1, keepdims=True) + 1e-20) * area
    dpdv = dpdv / (np.linalg.norm(dpdv, axis=1, keepdims=True) + 1e-20) * area
    U, V = np.zeros_like(p), np.zeros_like(p)
    for k in range(3):
        np.add.at(U, tri[ok, k], dpdu[ok])
        np.add.at(V, tri[ok, k], dpdv[ok])
    t = V - n * np.einsum('ij,ij->i', V, n)[:, None]
    b = U - n * np.einsum('ij,ij->i', U, n)[:, None]
    lt, lb = np.linalg.norm(t, axis=1), np.linalg.norm(b, axis=1)
    bad = (lt < 1e-9) | (lb < 1e-9)
    t /= np.maximum(lt, 1e-12)[:, None]
    b /= np.maximum(lb, 1e-12)[:, None]
    up = np.where(np.abs(n[:, 2:3]) < 0.95, np.array([[0, 0, 1.0]]), np.array([[1.0, 0, 0]]))
    ta = np.cross(up, n)
    ta /= np.linalg.norm(ta, axis=1, keepdims=True) + 1e-12
    t[bad], b[bad] = ta[bad], np.cross(ta, n)[bad]
    return t, b


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
