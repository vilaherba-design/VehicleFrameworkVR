"""ctypes binding for the quadric-error simplifier of meshoptimizer (https://github.com/zeux/meshoptimizer, MIT).

Build the library once (a dev tool, it is not part of the plugin):
    git clone --depth 1 https://github.com/zeux/meshoptimizer.git && cd meshoptimizer
    g++ -O2 -std=c++17 -shared -fPIC -o libmeshopt.so src/*.cpp
and point MESHOPT_LIB at it (default /tmp/mo_build/libmeshopt.so).
"""
import ctypes
import os

import numpy as np

_lib = ctypes.CDLL(os.environ.get('MESHOPT_LIB', '/tmp/mo_build/libmeshopt.so'))
_f = _lib.meshopt_simplifyWithAttributes
_f.restype = ctypes.c_size_t
_f.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t,
               ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_float, ctypes.c_uint, ctypes.c_void_p]

LOCK_BORDER, SPARSE, ERROR_ABSOLUTE, PRUNE, REGULARIZE, PERMISSIVE, REGULARIZE_LIGHT, PRESERVE_FOLDS = 1, 2, 4, 8, 16, 32, 64, 128


def simplify(pos, tri, attrs=None, weights=None, target_tris=0, error=1.0, options=0, lock=None):
    """Simplify an indexed triangle mesh. pos (n,3), tri (m,3), attrs (n,k) float32 (normals, uv ...) with per-attribute weights.
    Returns (new_tri (m2,3), relative error). target_tris=0 with error>0 simplifies as far as the error allows."""
    pos = np.ascontiguousarray(pos, np.float32)
    idx = np.ascontiguousarray(tri, np.uint32).reshape(-1)
    out = np.zeros(len(idx), np.uint32)
    err = ctypes.c_float(0)
    if attrs is not None:
        attrs = np.ascontiguousarray(attrs, np.float32)
        w = np.ascontiguousarray(weights, np.float32)
        ap, astr, wp, na = attrs.ctypes.data, attrs.shape[1] * 4, w.ctypes.data, attrs.shape[1]
    else:
        ap, astr, wp, na = None, 0, None, 0
    lp = None
    if lock is not None:
        lock = np.ascontiguousarray(lock, np.uint8)
        lp = lock.ctypes.data
    n = _f(out.ctypes.data, idx.ctypes.data, len(idx), pos.ctypes.data, len(pos), 12, ap, astr, wp, na, lp, int(target_tris) * 3, error, options, ctypes.byref(err))
    return out[:n].reshape(-1, 3).astype(np.int64), err.value
