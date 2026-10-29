"""Ambient occlusion of the military wheel (wheel_military.py, atlas UVs) baked in Cycles, headless: the grooves between the tread blocks, the
shoulders and the rim's dish and holes darken. make_military_wheel_assets.py uses it to darken the albedo and to put the mud in the grooves.

Usage: blender.exe -b --factory-startup -P bake_rueda.py -- <out ao.png> [size=2048] [samples=64]
"""
import os
import sys

import bpy

a = sys.argv[sys.argv.index("--") + 1:]
out = a[0]
SIZE = int(a[1]) if len(a) > 1 else 2048
SAMPLES = int(a[2]) if len(a) > 2 else 64
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wheel_military as wm  # noqa: E402

P, N, T, mats, Pd = wm.build(1.0)
uv = wm.uv(Pd, 1.0, mats)
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
me = bpy.data.meshes.new("rueda")
me.from_pydata([tuple(map(float, p)) for p in P], [], [tuple(map(int, t)) for t in T])
uvl = me.uv_layers.new(name="UV")
for poly in me.polygons:
    for li in poly.loop_indices:
        vi = me.loops[li].vertex_index
        uvl.data[li].uv = (float(uv[vi, 0]), 1.0 - float(uv[vi, 1]))
for poly in me.polygons:
    poly.use_smooth = True
o = bpy.data.objects.new("rueda", me)
sc.collection.objects.link(o)
mat = bpy.data.materials.new("m")
mat.use_nodes = True
me.materials.append(mat)
img = bpy.data.images.new("ao", SIZE, SIZE)
tn = mat.node_tree.nodes.new("ShaderNodeTexImage")
tn.image = img
mat.node_tree.nodes.active = tn
sc.world = bpy.data.worlds.new("w")
sc.world.light_settings.distance = 4.0                       # game units: a groove's depth
sc.render.engine = "CYCLES"
try:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "CUDA"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = True
    sc.cycles.device = "GPU"
except Exception as ex:
    print("cycles GPU:", ex)
sc.cycles.samples = SAMPLES
sc.render.bake.margin = 6
bpy.ops.object.select_all(action="DESELECT")
o.select_set(True)
bpy.context.view_layer.objects.active = o
bpy.ops.object.bake(type="AO")
img.filepath_raw = out
img.file_format = "PNG"
img.save()
print("BAKE_RUEDA", out)

# normals from a high version: the low wheel with every edge bevelled (rounded block and lug edges, rounded rim steps and bolt heads) and
# subdivided, baked selected-to-active onto the low one in tangent space (DirectX green, like the body); <out>_normal.png
hi = o.copy()
hi.data = o.data.copy()
sc.collection.objects.link(hi)
import bmesh  # noqa: E402
bm_ = bmesh.new()
bm_.from_mesh(hi.data)
bmesh.ops.remove_doubles(bm_, verts=bm_.verts, dist=0.0005)    # the low mesh has a corner per triangle (atlas UVs): weld for the bevel
bm_.to_mesh(hi.data)
bm_.free()
bv = hi.modifiers.new("bisel", "BEVEL")
bv.width = 1.1                                               # rounded lug edges (Axel: a little softer)
bv.segments = 4
bv.limit_method = "ANGLE"
bv.angle_limit = 0.6
sd = hi.modifiers.new("sub", "SUBSURF")
sd.levels = 2
sd.subdivision_type = "SIMPLE"
bpy.context.view_layer.objects.active = hi
for m_ in ("bisel", "sub"):
    bpy.ops.object.modifier_apply(modifier=m_)
for poly in hi.data.polygons:
    poly.use_smooth = True
print("ALTA rueda", len(hi.data.polygons), "faces")
img_n = bpy.data.images.new("n", SIZE, SIZE)
img_n.colorspace_settings.name = "Non-Color"
tn.image = img_n
bk = sc.render.bake
bk.use_selected_to_active = True
bk.cage_extrusion = 1.2
bk.max_ray_distance = 2.5
bk.normal_space = "TANGENT"
bk.normal_r, bk.normal_g, bk.normal_b = "POS_X", "NEG_Y", "POS_Z"
sc.cycles.samples = 1
bpy.ops.object.select_all(action="DESELECT")
hi.select_set(True)
o.select_set(True)
bpy.context.view_layer.objects.active = o
bpy.ops.object.bake(type="NORMAL")
img_n.filepath_raw = out[:-4] + "_normal.png"
img_n.file_format = "PNG"
img_n.save()
print("BAKE_RUEDA normal", img_n.filepath_raw)
