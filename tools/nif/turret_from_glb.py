"""Articulated vehicle turret NIF from a TRELLIS .glb (already decimated, e.g. gltfpack -si 0.12 -noq), the contract of TurretMG.nif:

  BSFadeNode <name>  (origin at the centre of the bottom of the base ring, Y forward, Z up, game units)
    NiNode TurretBase   the fixed base ring
    NiNode TurretYaw    at the vertical axis, at the height where the ring meets the body; turns about its Z
      NiNode TurretPitch  where the weapon enters the body, at the barrels' height (the trunnion axle); turns about its X
        NiNode TurretMuzzle, TurretMuzzle2...  at the muzzles, Y along the barrel
All rotations are identity; each part's vertices are relative to its node. Every triangle goes to one part whole (no cuts, so no caps):
  base   below the height where the radius drops from the ring to the body;
  pitch  in front of the yaw axis (y > 0), above the base and outside the body's radius at its height (the cradle, the trunnions, the weapons);
  yaw    the rest.
The mesh is scaled so the base ring is RING_DIAMETER across. Textures from the glb: _d (base colour), _s (R specular 0.08 + 0.72 metal,
G gloss 1 - roughness), _n (from the base colour's luminance, DirectX), as BC7 / BC5 with mipmaps (texconv), and a BGSM like the charging
station's.

Usage: turret_from_glb.py <model.glb> <Name> <turn degrees about Z> [--preview <png>]
Writes data/mod/Meshes/VehicleFrameworkVR/Turrets/<Name>.nif, Textures/VehicleFrameworkVR/Turrets/<Name>_{d,n,s}.dds,
Materials/VehicleFrameworkVR/Turrets/<Name>.BGSM, and prints the pivots, the muzzles and the triangle counts.
"""
import importlib.util
import io
import os
import struct
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import glb  # noqa: E402
import meshprep  # noqa: E402

DATA = os.path.join(REPO, "data", "mod")
TEMPLATE = os.path.join(DATA, "Meshes", "VehicleFrameworkVR", "Props", "ChargingStation.nif")      # header, shader and texture set
TEMPLATE_BGSM = os.path.join(DATA, "Materials", "VehicleFrameworkVR", "Props", "ChargingStation.BGSM")
TEXCONV = os.environ.get("TEXCONV", "texconv.exe")     # DirectXTex texconv: on PATH, or set TEXCONV
BGSM_TOOL = os.environ.get("BGSM_TOOL", "bgsm.py")     # BGSM writer script: set BGSM_TOOL to its path
RING_DIAMETER = 70.0            # game units (1 m)
NODE_FLAGS = 14
VA_VERTEX, VA_UV, VA_NORMAL, VA_TANGENT = 0x1, 0x2, 0x8, 0x10
# 20-byte vertex: half position + bitangent x, half UV, normal + bitangent y, tangent + bitangent z (as build_vehicle.py)
VERTEX_DESC = 5 | (2 << 8) | (3 << 16) | (4 << 20) | ((VA_VERTEX | VA_UV | VA_NORMAL | VA_TANGENT) << 44)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def read_glb(path, turn_deg):
    js, binc = glb.load(path)
    assert len(js["meshes"]) == 1 and len(js["meshes"][0]["primitives"]) == 1, "one mesh, one primitive expected"
    assert all(set(n) <= {"mesh", "name"} for n in js["nodes"]), "node transforms not supported"
    pr = js["meshes"][0]["primitives"][0]
    pos = glb.accessor(js, binc, pr["attributes"]["POSITION"]).astype(np.float64)
    nor = glb.accessor(js, binc, pr["attributes"]["NORMAL"]).astype(np.float64)
    uv = glb.accessor(js, binc, pr["attributes"]["TEXCOORD_0"]).astype(np.float64)
    tri = glb.accessor(js, binc, pr["indices"]).reshape(-1, 3).astype(np.int64)
    # glTF Y up -> Z up (as Blender imports it), then the turn that puts the weapon towards +Y
    pos = np.stack([pos[:, 0], -pos[:, 2], pos[:, 1]], 1)
    nor = np.stack([nor[:, 0], -nor[:, 2], nor[:, 1]], 1)
    a = np.radians(turn_deg)
    rot = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    pos, nor = pos @ rot.T, nor @ rot.T
    mat = js["materials"][pr["material"]]["pbrMetallicRoughness"]

    def image(tex):
        src = js["textures"][tex["index"]]["source"]
        bv = js["bufferViews"][js["images"][src]["bufferView"]]
        return Image.open(io.BytesIO(binc[bv.get("byteOffset", 0):bv.get("byteOffset", 0) + bv["byteLength"]]))
    return pos, nor, uv, tri, image(mat["baseColorTexture"]), image(mat["metallicRoughnessTexture"])


def place(pos):
    """Scale and move so the base ring is RING_DIAMETER across with its bottom centre at the origin."""
    zmin, zmax = pos[:, 2].min(), pos[:, 2].max()
    low = pos[pos[:, 2] < zmin + 0.08 * (zmax - zmin)]
    cx, cy = (low[:, 0].min() + low[:, 0].max()) / 2, (low[:, 1].min() + low[:, 1].max()) / 2
    r = np.hypot(low[:, 0] - cx, low[:, 1] - cy).max()
    return (pos - [cx, cy, zmin]) * (RING_DIAMETER / 2 / r)


def radius_profile(pos, step=1.0):
    """Largest radius about the yaw axis per height band, from the rear half (no weapon there)."""
    rear = pos[pos[:, 1] < 0]
    rr = np.hypot(rear[:, 0], rear[:, 1])
    bands = np.arange(0, pos[:, 2].max() + step, step)
    prof = np.zeros(len(bands))
    for i, z in enumerate(bands):
        m = (rear[:, 2] >= z) & (rear[:, 2] < z + step)
        prof[i] = rr[m].max() if m.any() else 0.0
    for i in range(1, len(prof)):            # empty bands keep the one below
        if prof[i] == 0:
            prof[i] = prof[i - 1]
    return bands, prof


def split(pos, tri):
    bands, prof = radius_profile(pos)
    ring = np.median(prof[(bands >= 5) & (bands <= 10)])
    body = np.median(prof[(bands >= 18) & (bands <= 30)])
    threshold = (ring + body) / 2
    base_top = next(z for z, r in zip(bands, prof) if z >= 4 and r < threshold)
    c = pos[tri].mean(1)
    r_at = np.interp(c[:, 2], bands, prof)
    part = np.full(len(tri), 1)                                        # 0 base, 1 yaw, 2 pitch
    part[c[:, 2] < base_top] = 0
    candidate = (c[:, 1] > 0) & (c[:, 2] > base_top + 2) & (np.hypot(c[:, 0], c[:, 1]) > r_at * 1.04 + 0.5)
    # Bits of the body that bulge past the profile are small islands: keep only the candidate pieces that are large or reach far forward.
    idx = np.flatnonzero(candidate)
    comp = components(pos, tri[idx])
    reach = c[idx, 1].max()
    clear = np.hypot(c[idx, 0], c[idx, 1]) - r_at[idx]
    for k in np.unique(comp):
        m = comp == k
        # large, reaching far forward, or clear of the body all round (a collar on the barrel): it moves with the weapon
        if m.sum() >= 0.02 * len(idx) or c[idx[m], 1].max() > 0.7 * reach or clear[m].min() > 1.5:
            part[idx[m]] = 2
    return part, float(base_top), float(ring), float(body)


def components(pos, tri):
    """Connected pieces of a triangle list (vertices welded by position, so UV seams do not split them): a label per triangle."""
    key = np.round(pos / 0.05).astype(np.int64)
    _, weld = np.unique(key, axis=0, return_inverse=True)
    v = weld.reshape(-1)[tri]
    parent = np.arange(v.max() + 1)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b, d in v:
        ra, rb, rd = find(a), find(b), find(d)
        parent[rb] = ra
        parent[find(rd)] = ra
    return np.array([find(a) for a in v[:, 0]])


def pitch_pivot(pos, tri, part):
    p = pos[np.unique(tri[part == 2])]
    y0 = np.percentile(p[:, 1], 5)
    z0 = np.median(p[(p[:, 1] < y0 + 8), 2])
    return np.array([0.0, y0, z0])


def muzzles(pos, tri, part, gap=2.0):
    """Muzzle points: the front tips of the weapon part, one per group separated by more than `gap` across X."""
    p = pos[np.unique(tri[part == 2])]
    tip = p[p[:, 1] > p[:, 1].max() - 1.5]
    xs = np.sort(tip[:, 0])
    groups, start = [], 0
    for i in range(1, len(xs) + 1):
        if i == len(xs) or xs[i] - xs[i - 1] > gap:
            groups.append((xs[start], xs[i - 1]))
            start = i
    out = []
    for lo, hi in groups:
        g = tip[(tip[:, 0] >= lo) & (tip[:, 0] <= hi)]
        out.append(np.array([(lo + hi) / 2, p[:, 1].max(), g[:, 2].mean()]))
    return sorted(out, key=lambda m: m[0])


def textures(base, mr, name, work):
    os.makedirs(work, exist_ok=True)
    col = np.asarray(base.convert("RGB"), np.float32) / 255
    m = np.asarray(mr.convert("RGB"), np.float32) / 255
    rough, metal = m[..., 1], m[..., 2]                                   # glTF: G roughness, B metalness
    spec = np.zeros_like(col)
    spec[..., 0] = 0.08 + 0.72 * metal
    spec[..., 1] = 1 - rough
    lum = col @ np.array([0.299, 0.587, 0.114], np.float32)
    gx = np.roll(lum, -1, 1) - np.roll(lum, 1, 1)
    gy = np.roll(lum, -1, 0) - np.roll(lum, 1, 0)
    k = 2.0
    n = np.dstack([-gx * k, gy * k, np.ones_like(lum)])                   # DirectX: green points down the image
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    files = {"d": col, "s": spec, "n": n * 0.5 + 0.5}
    out = {}
    for key, arr in files.items():
        png = os.path.join(work, "%s_%s.png" % (name, key))
        Image.fromarray((np.clip(arr, 0, 1) * 255 + 0.5).astype(np.uint8)).save(png)
        out[key] = png
    dst = os.path.join(DATA, "Textures", "VehicleFrameworkVR", "Turrets")
    os.makedirs(dst, exist_ok=True)
    for key, fmt in (("d", "BC7_UNORM"), ("n", "BC5_UNORM"), ("s", "BC5_UNORM")):
        subprocess.run([TEXCONV, "-nologo", "-y", "--ignore-srgb", "-m", "0", "-f", fmt, "-o", dst, out[key]], check=True,
                       stdout=subprocess.DEVNULL)
    return ["VehicleFrameworkVR/Turrets/%s_%s.dds" % (name, k) for k in "dns"]


def pack_part(pos, nor, uv, tri, offset):
    t, p, n, u = meshprep.compact(tri, pos, nor, uv)
    tg, bt = meshprep.uv_tangents(p, u, n, t)
    p = p - offset

    def enc(v):
        return np.clip(np.round((np.asarray(v, np.float64) + 1.0) * 127.5), 0, 255).astype(np.uint8)
    out = np.zeros((len(p), 20), np.uint8)
    out[:, 0:6] = np.frombuffer(np.asarray(p, np.float16).tobytes(), np.uint8).reshape(-1, 6)
    out[:, 6:8] = np.frombuffer(np.asarray(bt[:, 0], np.float16).tobytes(), np.uint8).reshape(-1, 2)
    out[:, 8:12] = np.frombuffer(np.asarray(u, np.float16).tobytes(), np.uint8).reshape(-1, 4)
    out[:, 12:15] = enc(n)
    out[:, 15] = enc(bt[:, 1])
    out[:, 16:19] = enc(tg)
    out[:, 19] = enc(bt[:, 2])
    c = (p.min(0) + p.max(0)) / 2
    r = float(np.linalg.norm(p - c, axis=1).max())
    return out.tobytes(), np.asarray(t, np.uint16).tobytes(), len(p), len(t), c, r


def main(path, name, turn, preview=None):
    na = load_module("nif_arbol", os.path.join(HERE, "nif_arbol.py"))
    rig = load_module("torreta_rigida", os.path.join(HERE, "turret_rigid.py"))
    pos, nor, uv, tri, base_img, mr_img = read_glb(path, float(turn))
    pos = place(pos)
    part, base_top, ring_r, body_r = split(pos, tri)
    yaw = np.array([0.0, 0.0, base_top])
    pitch = pitch_pivot(pos, tri, part)
    tips = muzzles(pos, tri, part)
    pitch[2] = np.mean([t[2] for t in tips])        # the trunnions carry the barrels: the axle at the barrels' height

    tex = textures(base_img, mr_img, name, os.path.join(tempfile.gettempdir(), "vfvr_turret_tex"))
    bgsm = load_module("bgsm", BGSM_TOOL)
    values, maps = bgsm.leer(TEMPLATE_BGSM)
    maps.update(Diffuse=tex[0], Normal=tex[1], Specular=tex[2])
    mat_dir = os.path.join(DATA, "Materials", "VehicleFrameworkVR", "Turrets")
    os.makedirs(mat_dir, exist_ok=True)
    bgsm.escribir(os.path.join(mat_dir, name + ".BGSM"), values, maps)

    tmpl = na.Nif(TEMPLATE)
    shape_i = next(i for i, (t, _, _) in enumerate(tmpl.b) if t == "BSTriShape")
    sp_i = tmpl.bloque(shape_i)["shader"]
    ts_i = tmpl.bloque(sp_i)["texset"]
    _, sp_off, sp_size = tmpl.b[sp_i]
    _, ts_off, ts_size = tmpl.b[ts_i]
    slots = tmpl.bloque(ts_i)["tex"]

    w = rig.Escritor(tmpl)
    root = w.add("BSFadeNode")
    nodes = {k: w.add("NiNode") for k in ("base", "yaw", "pitch")}
    muzzle_nodes = [w.add("NiNode") for _ in tips]
    kids = {"base": [], "yaw": [nodes["pitch"]], "pitch": list(muzzle_nodes)}
    offsets = {"base": np.zeros(3), "yaw": yaw, "pitch": pitch}
    counts = {}
    for k, key in enumerate(("base", "yaw", "pitch")):
        sel = tri[part == k]
        vb, tb, nv, nt, c, r = pack_part(pos, nor, uv, sel, offsets[key])
        assert nv < 65536
        counts[key] = nt
        k_sh = w.add("BSTriShape")
        k_sp = w.add("BSLightingShaderProperty")
        k_ts = w.add("BSShaderTextureSet")
        sp = bytearray(tmpl.d[sp_off:sp_off + sp_size])
        struct.pack_into("<i", sp, 4, w.S("Materials\\VehicleFrameworkVR\\Turrets\\%s.BGSM" % name))
        struct.pack_into("<i", sp, 12, -1)
        struct.pack_into("<i", sp, 40, k_ts)
        struct.pack_into("<i", sp, rig.WET_MATERIAL, -1)        # the block's other string (wet material): none, as the template's
        w.bloques[k_sp][1] = bytes(sp)
        new_slots = list(slots)
        new_slots[0], new_slots[1] = "textures\\" + tex[0].replace("/", "\\"), "textures\\" + tex[1].replace("/", "\\")
        s_slot = next((i for i, s in enumerate(slots) if s.lower().endswith("_s.dds")), 2)
        new_slots[s_slot] = "textures\\" + tex[2].replace("/", "\\")
        w.bloques[k_ts][1] = struct.pack("<I", len(new_slots)) + b"".join(struct.pack("<I", len(s)) + s.encode() for s in new_slots)
        b = rig.Escritor.avobj(w.S("%s_%s" % (name, key)), NODE_FLAGS, (0, 0, 0))
        b += struct.pack("<4f", *c, r) + struct.pack("<3i", -1, k_sp, -1) + struct.pack("<Q", VERTEX_DESC)
        b += struct.pack("<IHI", nt, nv, len(vb) + len(tb)) + vb + tb
        w.bloques[k_sh][1] = b
        kids[key].append(k_sh)
    w.bloques[root][1] = rig.Escritor.avobj(w.S(name), NODE_FLAGS, (0, 0, 0), [nodes["base"], nodes["yaw"]])
    w.bloques[nodes["base"]][1] = rig.Escritor.avobj(w.S("TurretBase"), NODE_FLAGS, (0, 0, 0), kids["base"])
    w.bloques[nodes["yaw"]][1] = rig.Escritor.avobj(w.S("TurretYaw"), NODE_FLAGS, tuple(yaw), kids["yaw"])
    w.bloques[nodes["pitch"]][1] = rig.Escritor.avobj(w.S("TurretPitch"), NODE_FLAGS, tuple(pitch - yaw), kids["pitch"])
    names = ["TurretMuzzle"] + ["TurretMuzzle%d" % (i + 2) for i in range(len(tips) - 1)]
    for k_m, nm, tip in zip(muzzle_nodes, names, tips):
        w.bloques[k_m][1] = rig.Escritor.avobj(w.S(nm), NODE_FLAGS, tuple(tip - pitch), [])
    dst = os.path.join(DATA, "Meshes", "VehicleFrameworkVR", "Turrets", name + ".nif")
    size = w.guardar(dst)
    print("wrote %s (%d bytes, %d blocks)" % (dst, size, len(w.bloques)))
    print("base top %.1f (ring r %.1f, body r %.1f); size %s" % (base_top, ring_r, body_r, np.round(pos.max(0) - pos.min(0), 1)))
    print("triangles: base %d, yaw %d, pitch %d (total %d)" % (counts["base"], counts["yaw"], counts["pitch"], len(tri)))
    print("TurretYaw %s, TurretPitch %s" % (np.round(yaw, 2), np.round(pitch, 2)))
    for nm, tip in zip(names, tips):
        print("%s %s" % (nm, np.round(tip, 2)))


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    if len(a) != 3:
        sys.exit(__doc__)
    main(*a)
