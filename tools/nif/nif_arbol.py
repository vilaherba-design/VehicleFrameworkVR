"""Read-only dump of a FO4 NIF (BSVER 130): node tree, shapes, skin, materials and connect points. Also the block reader turret_rigid.py
and turret_from_glb.py use.

  python nif_arbol.py <file.nif> [--json out.json]
"""
import json, math, struct, sys

VA_SKINNED, VA_FULLPREC = 0x40, 0x400


def half(h):
    return struct.unpack("<e", struct.pack("<H", h))[0]


class Nif:
    def __init__(self, ruta):
        d = self.d = open(ruta, "rb").read()
        p = d.index(b"\n") + 1
        ver, endian, uv, nb = struct.unpack_from("<IBII", d, p); p += 13
        self.bsv, = struct.unpack_from("<I", d, p); p += 4
        for _ in range(4 if self.bsv == 130 else 3):
            p += 1 + d[p]
        nt, = struct.unpack_from("<H", d, p); p += 2
        tipos = []
        for _ in range(nt):
            n, = struct.unpack_from("<I", d, p); p += 4
            tipos.append(d[p:p + n].decode()); p += n
        idx = struct.unpack_from("<%dH" % nb, d, p); p += 2 * nb
        tam = struct.unpack_from("<%dI" % nb, d, p); p += 4 * nb
        ns, = struct.unpack_from("<I", d, p); p += 8
        self.s = []
        for _ in range(ns):
            n, = struct.unpack_from("<I", d, p); p += 4
            self.s.append(d[p:p + n].decode(errors="replace")); p += n
        ng, = struct.unpack_from("<I", d, p); p += 4 + 4 * ng
        self.b = []
        for i in range(nb):
            self.b.append((tipos[idx[i] & 0x7FFF], p, tam[i])); p += tam[i]
        self.cache = {}

    # --- lectura secuencial ---
    def _rd(self, fmt):
        v = struct.unpack_from("<" + fmt, self.d, self.p); self.p += struct.calcsize("<" + fmt)
        return v if len(v) > 1 else v[0]

    def _str(self):
        i = self._rd("i")
        return None if i < 0 or i >= len(self.s) else self.s[i]

    def _sized(self):
        n = self._rd("I"); t = self.d[self.p:self.p + n].decode(errors="replace"); self.p += n
        return t

    def objnet(self):
        o = {"name": self._str()}
        n = self._rd("I"); o["extra"] = [self._rd("i") for _ in range(n)]
        o["controller"] = self._rd("i")
        return o

    def avobj(self, o):
        o["flags"] = self._rd("I")
        o["t"] = list(self._rd("3f")); r = self._rd("9f"); o["r"] = [r[0:3], r[3:6], r[6:9]]
        o["scale"] = self._rd("f"); o["collision"] = self._rd("i")
        return o

    def bloque(self, i):
        if i in self.cache:
            return self.cache[i]
        tipo, off, size = self.b[i]
        self.p = off
        o = {"i": i, "type": tipo}
        if tipo in ("NiNode", "BSFadeNode", "BSLeafAnimNode", "BSOrderedNode", "BSMultiBoundNode", "BSValueNode",
                    "NiBillboardNode", "NiSwitchNode", "BSTreeNode", "NiLODNode", "BSRangeNode", "BSDamageStage", "BSBlastNode", "BSDebrisNode"):
            o.update(self.objnet()); self.avobj(o)
            n = self._rd("I"); o["children"] = [self._rd("i") for _ in range(n)]
        elif tipo in ("BSTriShape", "BSSubIndexTriShape", "BSMeshLODTriShape", "BSDynamicTriShape"):
            o.update(self.objnet()); self.avobj(o)
            o["bound"] = list(self._rd("4f"))
            o["skin"], o["shader"], o["alpha"] = self._rd("3i")
            desc = self._rd("Q"); o["vdesc"] = desc
            if tipo == "BSMeshLODTriShape" or True:
                ntri = self._rd("I"); nv = self._rd("H"); dsz = self._rd("I")
            o["ntri"], o["nvert"] = ntri, nv
            attrs = (desc >> 44) & 0xFFF; stride = (desc & 0xF) * 4
            o["skinned_attr"] = bool(attrs & VA_SKINNED)
            pts = []
            if dsz and stride:
                base = self.p
                for k in range(nv):
                    q = base + k * stride
                    if attrs & VA_FULLPREC:
                        pts.append(struct.unpack_from("<3f", self.d, q))
                    else:
                        pts.append(tuple(half(h) for h in struct.unpack_from("<3H", self.d, q)))
            o["aabb"] = aabb(pts) if pts else None
        elif tipo == "BSSkin::Instance":
            o["skel_root"], o["data"] = self._rd("2i")
            n = self._rd("I"); o["bones"] = [self._rd("i") for _ in range(n)]
        elif tipo == "NiSkinInstance" or tipo == "BSDismemberSkinInstance":
            o["data"], o["partition"], o["skel_root"] = self._rd("3i")
            n = self._rd("I"); o["bones"] = [self._rd("i") for _ in range(n)]
        elif tipo == "BSLightingShaderProperty" or tipo == "BSEffectShaderProperty":
            if tipo == "BSLightingShaderProperty":
                o["shader_type"] = self._rd("I")  # en BSVER 130 va antes del nombre
            o.update(self.objnet())
            o["sf1"], o["sf2"] = self._rd("2I"); self._rd("4f")
            if tipo == "BSLightingShaderProperty":
                o["texset"] = self._rd("i")
            else:
                o["source_tex"] = self._sized()
        elif tipo == "BSShaderTextureSet":
            n = self._rd("I"); o["tex"] = [self._sized() for _ in range(n)]
        elif tipo in ("NiStringExtraData",):
            o["name"] = self._str(); o["value"] = self._str()
        elif tipo == "BSXFlags" or tipo == "NiIntegerExtraData":
            o["name"] = self._str(); o["value"] = self._rd("I")
        elif tipo == "BSConnectPoint::Parents":
            o["name"] = self._str(); n = self._rd("I"); pts = []
            for _ in range(n):
                par = self._sized(); nm = self._sized(); q = self._rd("4f"); t = self._rd("3f"); sc = self._rd("f")
                pts.append({"parent": par, "name": nm, "rot": q, "t": t, "scale": sc})
            o["points"] = pts
        elif tipo == "BSConnectPoint::Children":
            o["name"] = self._str(); o["skinned"] = self._rd("B"); n = self._rd("I")
            o["targets"] = [self._sized() for _ in range(n)]
        elif tipo == "BSBehaviorGraphExtraData":
            o["name"] = self._str(); o["file"] = self._str(); o["controls_base_skel"] = self._rd("B")
        elif tipo == "NiAlphaProperty":
            o.update(self.objnet()); o["aflags"] = self._rd("H"); o["thr"] = self._rd("B")
        elif tipo in ("BSInvMarker", "BSBound", "BSFurnitureMarkerNode", "BSClothExtraData", "BSEyeCenterExtraData",
                      "NiFloatExtraData", "NiBooleanExtraData", "BSPositionData", "BSDecalPlacementVectorExtraData"):
            o["name"] = self._str()
            if tipo == "BSBound":
                o["center"] = self._rd("3f"); o["half"] = self._rd("3f")
        self.cache[i] = o
        return o


def aabb(pts):
    mn = [min(p[k] for p in pts) for k in range(3)]; mx = [max(p[k] for p in pts) for k in range(3)]
    return [mn, mx]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def xf(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def volcar(ruta):
    n = Nif(ruta); out = []
    padres = {}
    for i, (t, _, _) in enumerate(n.b):
        o = n.bloque(i)
        for c in o.get("children", []):
            padres[c] = i
    # transformaciones globales (NiAVObject: v_padre = R*v*s + t)
    glob = {}

    def g(i):
        if i in glob:
            return glob[i]
        o = n.bloque(i)
        R, T, S = o.get("r", [[1, 0, 0], [0, 1, 0], [0, 0, 1]]), o.get("t", [0, 0, 0]), o.get("scale", 1.0)
        if i in padres:
            PR, PT, PS = g(padres[i])
            R2 = matmul(PR, R); T2 = [a + b for a, b in zip(xf(PR, [x * PS for x in T]), PT)]; S2 = PS * S
        else:
            R2, T2, S2 = R, T, S
        glob[i] = (R2, T2, S2)
        return glob[i]

    lineas = []
    raiz = 0

    def arbol(i, prof):
        o = n.bloque(i)
        nombre = o.get("name")
        R, T, S = g(i)
        tag = ""
        if o["type"] in ("BSTriShape", "BSSubIndexTriShape", "BSMeshLODTriShape", "BSDynamicTriShape"):
            tag = f"  tris={o['ntri']} verts={o['nvert']} skin={'sí' if o['skin'] >= 0 else 'no'}"
        lineas.append("  " * prof + f"[{i}] {o['type']} '{nombre}'  pos_global=({T[0]:.1f},{T[1]:.1f},{T[2]:.1f}){tag}")
        for e in o.get("extra", []):
            if 0 <= e < len(n.b):
                x = n.bloque(e)
                desc = x.get("value", x.get("file", ""))
                if x["type"] == "BSConnectPoint::Parents":
                    desc = "; ".join(f"{p['parent']}->{p['name']} t=({p['t'][0]:.1f},{p['t'][1]:.1f},{p['t'][2]:.1f})" for p in x["points"])
                lineas.append("  " * prof + f"    · extra [{e}] {x['type']} '{x.get('name')}' {desc}")
        for c in o.get("children", []):
            if 0 <= c < len(n.b):
                arbol(c, prof + 1)
    arbol(raiz, 0)

    shapes = []
    total_tri = 0; gmn = [1e9] * 3; gmx = [-1e9] * 3
    for i, (t, _, _) in enumerate(n.b):
        if t not in ("BSTriShape", "BSSubIndexTriShape", "BSMeshLODTriShape", "BSDynamicTriShape"):
            continue
        o = n.bloque(i); total_tri += o["ntri"]
        sh = {"i": i, "type": t, "name": o["name"], "tris": o["ntri"], "verts": o["nvert"]}
        if o["aabb"]:
            R, T, S = g(i) if not (o["skin"] >= 0) else ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0], 1.0)
            for c in ([a, b, cc] for a in (o["aabb"][0][0], o["aabb"][1][0]) for b in (o["aabb"][0][1], o["aabb"][1][1]) for cc in (o["aabb"][0][2], o["aabb"][1][2])):
                w = [x + y for x, y in zip(xf(R, [v * S for v in c]), T)]
                gmn = [min(a, b) for a, b in zip(gmn, w)]; gmx = [max(a, b) for a, b in zip(gmx, w)]
            sh["aabb_local"] = o["aabb"]
        if o["skin"] >= 0:
            sk = n.bloque(o["skin"])
            sh["skin_type"] = sk["type"]
            sh["bones"] = [n.bloque(b).get("name") if 0 <= b < len(n.b) else None for b in sk.get("bones", [])]
            sh["skel_root"] = n.bloque(sk["skel_root"]).get("name") if sk.get("skel_root", -1) >= 0 else None
        if o["shader"] >= 0:
            sp = n.bloque(o["shader"])
            sh["material"] = sp.get("name")
            if sp.get("texset", -1) >= 0:
                sh["textures"] = [x for x in n.bloque(sp["texset"])["tex"] if x]
            if "source_tex" in sp:
                sh["textures"] = [sp["source_tex"]]
        shapes.append(sh)
    return {"file": ruta, "bsv": n.bsv, "blocks": len(n.b), "tree": lineas, "shapes": shapes,
            "total_tris": total_tri, "aabb": [gmn, gmx] if total_tri else None,
            "types": sorted(set(t for t, _, _ in n.b))}


if __name__ == "__main__":
    r = volcar(sys.argv[1])
    print(f"# {r['file']}  (BSVER {r['bsv']}, {r['blocks']} bloques)")
    print("tipos:", ", ".join(r["types"]))
    print("\n".join(r["tree"]))
    for s in r["shapes"]:
        print(f"- shape [{s['i']}] {s['type']} '{s['name']}' tris={s['tris']} verts={s['verts']}")
        if "bones" in s:
            print(f"    skin {s['skin_type']} raíz={s['skel_root']} huesos={s['bones']}")
        if "material" in s:
            print(f"    material={s['material']} texturas={s.get('textures')}")
    print("triángulos totales:", r["total_tris"])
    if r["aabb"]:
        mn, mx = r["aabb"]
        print("AABB (u):", [round(x, 1) for x in mn], [round(x, 1) for x in mx],
              "tamaño:", [round(b - a, 1) for a, b in zip(mn, mx)])
    if "--json" in sys.argv:
        json.dump(r, open(sys.argv[sys.argv.index("--json") + 1], "w"), indent=1, default=str)
