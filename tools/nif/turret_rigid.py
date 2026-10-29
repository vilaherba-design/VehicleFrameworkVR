"""Rigid articulated turret from the vanilla tripod turret with skin (TurretStanding.nif): TurretMG.nif and TurretMG_sinpatas.nif.

  python turret_rigid.py <TurretStanding.nif> <TurretStandingSkeleton.nif> <out.nif> [--sin-patas]

Each triangle goes to the piece of the bone with the most weight summed over its 3 vertices:
  Pelvis/COM -> TurretBase (fixed); BODY, Motor, *Ammo_Chain, Latch, Lid -> TurretYaw; gun, Gun_Armor -> TurretPitch.
The skinned vertices are already in the bind pose (model space): BSSkin::BoneData is the inverse of the NIF's own nodes, so only the pivots
are subtracted. The shaders lose the Skinned flag (SLSF1 bit 1), which a shape without skin must not carry. Game units, Y forward, Z up.
"""
import importlib.util, os, struct, sys
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("nif_arbol", os.path.join(AQUI, "nif_arbol.py"))  # next to this file
na = importlib.util.module_from_spec(spec); spec.loader.exec_module(na)

GRUPO = {"pelvis": "base", "com": "base",
         "body": "yaw", "motor": "yaw", "lammo_chain": "yaw", "rammo_chain": "yaw", "latch": "yaw", "lid": "yaw",
         "gun": "pitch", "gun_armor": "pitch"}
VA_SKINNED = 0x40
WET_MATERIAL = 60     # BSLightingShaderProperty (BSVER 130): offset of the wet material string index, the block's second string


def nodos_globales(nif):
    """{nombre en minúsculas: posición global} de los NiNode de un NIF."""
    padres, out = {}, {}
    for i, (t, _, _) in enumerate(nif.b):
        for c in nif.bloque(i).get("children", []):
            padres[c] = i

    def g(i):
        o = nif.bloque(i)
        R, T, S = np.array(o["r"]), np.array(o["t"]), o["scale"]
        if i in padres:
            PR, PT, PS = g(padres[i])
            return PR @ R, PR @ (T * PS) + PT, PS * S
        return R, T, S
    for i, (t, _, _) in enumerate(nif.b):
        if t == "NiNode":
            out[nif.bloque(i)["name"].lower()] = g(i)
    return out


def leer_shape(nif, i):
    o = nif.bloque(i)
    _, off, size = nif.b[i]
    desc = o["vdesc"]; stride = (desc & 0xF) * 4
    sk_off = ((desc >> 28) & 0xF) * 4
    attrs = (desc >> 44) & 0xFFF
    assert attrs & VA_SKINNED and sk_off + 12 == stride, "el skin tiene que ir al final del vértice"
    assert not attrs & 0x400, "se esperan posiciones en half"
    # cabecera BSTriShape: hasta data size inclusive
    nif.p = off; nif.objnet(); nif.avobj({}); nif._rd("4f"); nif._rd("3i"); nif._rd("Q")
    nt = nif._rd("I"); nv = nif._rd("H"); nif._rd("I")
    vb = np.frombuffer(nif.d, np.uint8, nv * stride, nif.p).reshape(nv, stride).copy()
    tri = np.frombuffer(nif.d, np.uint16, nt * 3, nif.p + nv * stride).reshape(nt, 3).astype(np.int64)
    pos = np.frombuffer(vb[:, 0:6].tobytes(), np.float16).reshape(nv, 3).astype(np.float64)
    w = np.frombuffer(vb[:, sk_off:sk_off + 8].tobytes(), np.float16).reshape(nv, 4).astype(np.float64)
    idx = vb[:, sk_off + 8:sk_off + 12].astype(np.int64)
    huesos = [nif.bloque(b)["name"].lower() for b in nif.bloque(o["skin"])["bones"]]
    nueva = (desc & ~(0xF << 28) & ~0xF & ~(VA_SKINNED << 44)) | ((stride - 12) // 4)
    return dict(o=o, vb=vb[:, :stride - 12], pos=pos, w=w, idx=idx, tri=tri, huesos=huesos, desc=nueva)


def grupo_por_triangulo(sh):
    n = len(sh["huesos"])
    peso = np.zeros((len(sh["pos"]), n))
    for k in range(4):
        np.add.at(peso, (np.arange(len(peso)), sh["idx"][:, k]), sh["w"][:, k])
    por_tri = peso[sh["tri"]].sum(1)                 # (nt, huesos)
    hueso = por_tri.argmax(1)
    return np.array([GRUPO[sh["huesos"][h]] for h in hueso])


class Escritor:
    def __init__(self, plantilla):
        self.d = plantilla.d
        self.cad, self.bloques = [], []

    def S(self, s):
        if s is None:
            return -1
        if s not in self.cad:
            self.cad.append(s)
        return self.cad.index(s)

    def add(self, tipo, datos=None):
        self.bloques.append([tipo, datos]); return len(self.bloques) - 1

    @staticmethod
    def avobj(nombre_i, flags, t, hijos=None):
        b = struct.pack("<iIi", nombre_i, 0, -1) + struct.pack("<I", flags) + struct.pack("<3f", *t)
        b += struct.pack("<9f", 1, 0, 0, 0, 1, 0, 0, 0, 1) + struct.pack("<f", 1.0) + struct.pack("<i", -1)
        if hijos is not None:
            b += struct.pack("<I", len(hijos)) + struct.pack("<%di" % len(hijos), *hijos)
        return b

    def guardar(self, ruta):
        d = self.d
        p = d.index(b"\n") + 1
        cab_ini = d[:p + 9]                                   # cadena + versión + endian + user version
        q = p + 13 + 4                                        # tras num blocks y BS version
        for _ in range(4):
            q += 1 + d[q]
        bs = d[p + 13:q]                                      # BS version + export strings
        tipos = []
        for t, _ in self.bloques:
            if t not in tipos:
                tipos.append(t)
        out = bytearray(cab_ini) + struct.pack("<I", len(self.bloques)) + bs
        out += struct.pack("<H", len(tipos))
        for t in tipos:
            out += struct.pack("<I", len(t)) + t.encode()
        out += struct.pack("<%dH" % len(self.bloques), *[tipos.index(t) for t, _ in self.bloques])
        out += struct.pack("<%dI" % len(self.bloques), *[len(b) for _, b in self.bloques])
        out += struct.pack("<II", len(self.cad), max(len(s.encode()) for s in self.cad))
        for s in self.cad:
            out += struct.pack("<I", len(s.encode())) + s.encode()
        out += struct.pack("<I", 0)                           # grupos
        for _, b in self.bloques:
            out += b
        out += struct.pack("<Ii", 1, 0)                       # pie: una raíz, el bloque 0
        os.makedirs(os.path.dirname(os.path.abspath(ruta)), exist_ok=True)
        open(ruta, "wb").write(out)
        check_strings(ruta)
        return len(out)


def check_strings(ruta):
    """Every string index the written blocks hold must be in the string table (-1 = none): the game reads past the table otherwise."""
    n = na.Nif(ruta)
    for i, (tipo, off, _) in enumerate(n.b):
        campos = []
        if tipo in ("NiNode", "BSFadeNode", "BSTriShape", "NiAlphaProperty"):
            campos = [off]
        elif tipo == "BSLightingShaderProperty":
            campos = [off + 4, off + WET_MATERIAL]
        for c in campos:
            k, = struct.unpack_from("<i", n.d, c)
            if not -1 <= k < len(n.s):
                raise ValueError("%s: block %d (%s) holds string index %d, the table has %d" % (ruta, i, tipo, k, len(n.s)))


def main(malla, esqueleto, salida, sin_patas=False):
    nif = na.Nif(malla); esq = na.Nif(esqueleto)
    gm, ge = nodos_globales(nif), nodos_globales(esq)
    piv = {"yaw": ge["body"][1], "pitch": ge["gun"][1], "muzzle": ge["weapon"][1]}
    for k, nm in (("yaw", "body"), ("pitch", "gun")):
        assert np.allclose(gm[nm][1], piv[k], atol=0.01), (nm, gm[nm][1], piv[k])
    off = {"base": np.zeros(3), "yaw": piv["yaw"], "pitch": piv["pitch"]}

    w = Escritor(nif)
    raiz = w.add("BSFadeNode")
    nodos = {"base": w.add("NiNode"), "yaw": w.add("NiNode"), "pitch": w.add("NiNode")}
    muzzle = w.add("NiNode")
    hijos = {"base": [], "yaw": [nodos["pitch"]], "pitch": [muzzle]}
    informe = {}
    for i, (t, _, _) in enumerate(nif.b):
        if t != "BSSubIndexTriShape":
            continue
        sh = leer_shape(nif, i)
        g = grupo_por_triangulo(sh)
        for grupo in ("base", "yaw", "pitch"):
            sel = g == grupo
            if not sel.any() or (sin_patas and grupo == "base"):
                informe.setdefault(sh["o"]["name"], {})[grupo] = int(sel.sum()) if not sin_patas or grupo != "base" else 0
                continue
            tri = sh["tri"][sel]
            usados = np.unique(tri)
            mapa = -np.ones(len(sh["pos"]), np.int64); mapa[usados] = np.arange(len(usados))
            tri2 = mapa[tri]
            vb = sh["vb"][usados].copy()
            p = sh["pos"][usados] - off[grupo]
            vb[:, 0:6] = np.frombuffer(p.astype(np.float16).tobytes(), np.uint8).reshape(-1, 6)
            ph = vb[:, 0:6].copy().view(np.float16).reshape(-1, 3).astype(np.float64)
            c = (ph.min(0) + ph.max(0)) / 2; r = float(np.linalg.norm(ph - c, axis=1).max())
            k_sh = w.add("BSTriShape")
            # shader: copia con controlador quitado, nombre y texture set reasignados
            so = sh["o"]["shader"]; _, s_off, s_size = nif.b[so]
            sp = bytearray(nif.d[s_off:s_off + s_size])
            nombre_old, nex = struct.unpack_from("<iI", sp, 4); assert nex == 0
            struct.pack_into("<i", sp, 4, w.S(nif.s[nombre_old]) if nombre_old >= 0 else -1)
            struct.pack_into("<i", sp, 12, -1)
            # The other string in the block (BSVER 130): the wet material at +60. A stale index points past the new string table and the
            # game crashes reading the NIF (0.1.113).
            wet_old, = struct.unpack_from("<i", sp, WET_MATERIAL)
            struct.pack_into("<i", sp, WET_MATERIAL, w.S(nif.s[wet_old]) if 0 <= wet_old < len(nif.s) else -1)
            sf1, = struct.unpack_from("<I", sp, 16)
            struct.pack_into("<I", sp, 16, sf1 & ~0x2)          # no Skinned flag on a shape without skin
            k_sp = w.add("BSLightingShaderProperty", None)
            k_ts = w.add("BSShaderTextureSet")
            ts_i = nif.bloque(so)["texset"]; _, t_off, t_size = nif.b[ts_i]
            w.bloques[k_ts][1] = bytes(nif.d[t_off:t_off + t_size])
            struct.pack_into("<i", sp, 40, k_ts)
            w.bloques[k_sp][1] = bytes(sp)
            k_al = -1
            if sh["o"]["alpha"] >= 0:
                _, a_off, a_size = nif.b[sh["o"]["alpha"]]
                al = bytearray(nif.d[a_off:a_off + a_size])
                an, anex = struct.unpack_from("<iI", al, 0); assert anex == 0
                struct.pack_into("<i", al, 0, w.S(nif.s[an]) if an >= 0 else -1)
                struct.pack_into("<i", al, 8, -1)
                k_al = w.add("NiAlphaProperty", bytes(al))
            nombre = "%s_%s" % (sh["o"]["name"].replace(":", "_"), grupo)
            b = Escritor.avobj(w.S(nombre), sh["o"]["flags"], (0, 0, 0))
            b += struct.pack("<4f", *c, r) + struct.pack("<3i", -1, k_sp, k_al) + struct.pack("<Q", sh["desc"])
            vbb = vb.tobytes(); tb = tri2.astype(np.uint16).tobytes()
            b += struct.pack("<IHI", len(tri2), len(usados), len(vbb) + len(tb)) + vbb + tb
            w.bloques[k_sh][1] = b
            hijos[grupo].append(k_sh)
            informe.setdefault(sh["o"]["name"], {})[grupo] = len(tri2)
    w.bloques[raiz][1] = Escritor.avobj(w.S("TurretMG"), 14, (0, 0, 0), [nodos["base"], nodos["yaw"]])
    w.bloques[nodos["base"]][1] = Escritor.avobj(w.S("TurretBase"), 14, (0, 0, 0), hijos["base"])
    w.bloques[nodos["yaw"]][1] = Escritor.avobj(w.S("TurretYaw"), 14, tuple(piv["yaw"]), hijos["yaw"])
    w.bloques[nodos["pitch"]][1] = Escritor.avobj(w.S("TurretPitch"), 14, tuple(piv["pitch"] - piv["yaw"]), hijos["pitch"])
    w.bloques[muzzle][1] = Escritor.avobj(w.S("TurretMuzzle"), 14, tuple(piv["muzzle"] - piv["pitch"]), [])
    tam = w.guardar(salida)
    print(f"escrito {salida} ({tam} bytes, {len(w.bloques)} bloques)")
    print("pivotes (frame TurretMG):", {k: [round(float(x), 3) for x in v] for k, v in piv.items()})
    print("triángulos por shape y pieza:", informe)


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    main(a[0], a[1], a[2], "--sin-patas" in sys.argv)
