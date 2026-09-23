# sar_physics_preview.py -- Blender-side rigid body PREVIEW for DYNAMIC_PROPS.
#
# This is NOT the simulation. Gazebo runs the real physics from the mass,
# inertia and surface values exported into each prop's model.sdf. This exists
# so you can hit Play in Blender and watch a barrel topple before committing to
# a layout. Blender rigid bodies do not export.
#
# It makes only the terrain tiles UNDER the props passive colliders, because
# making all 36 tiles passive mesh colliders is enough to make Bullet crawl.
import bpy

props = bpy.data.collections.get("DYNAMIC_PROPS")
terr = bpy.data.collections.get("SAR_TERRAIN")
if props is None or terr is None:
    raise SystemExit("run this inside military_world.blend")

if bpy.context.scene.rigidbody_world is None:
    bpy.ops.rigidbody.world_add()

def add_rb(ob, kind):
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    bpy.ops.rigidbody.object_add(type=kind)
    ob.select_set(False)

used_tiles = set()
for o in props.all_objects:
    if o.type != 'MESH' or o.get("physics") != "DYNAMIC":
        continue
    add_rb(o, 'ACTIVE')
    rb = o.rigid_body
    rb.mass = float(o.get("mass_kg", 10.0))
    rb.friction = float(o.get("friction_mu", 0.5))
    rb.restitution = float(o.get("restitution", 0.1))
    rb.collision_shape = {"BOX": 'BOX', "CYLINDER": 'CYLINDER'}.get(
        o.get("collision", "BOX"), 'CONVEX_HULL')
    # nearest terrain tile
    best = None; bd = 1e18
    for t in terr.objects:
        d = (t.location - o.location).length
        if d < bd:
            bd = d; best = t
    if best is not None:
        used_tiles.add(best.name)

for name in used_tiles:
    t = bpy.data.objects[name]
    if t.rigid_body is None:
        add_rb(t, 'PASSIVE')
        t.rigid_body.collision_shape = 'MESH'
        t.rigid_body.friction = 0.8

print("preview armed: %d props, %d passive tiles. Press Play." %
      (len([o for o in props.all_objects if o.rigid_body]), len(used_tiles)))
