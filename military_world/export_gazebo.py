#!/usr/bin/env python3
"""
export_gazebo.py -- Blender -> Gazebo Sim Harmonic exporter for military_world.

Run:
  blender --background military_world.blend --python export_gazebo.py -- \
          --out ./gazebo_export

Produces a directory you can run directly:

  <out>/military_world.sdf     one complete, runnable world
  <out>/meshes/*.glb           one mesh per unique Blender mesh datablock

What it does
------------
* Exports one mesh per unique MESH DATABLOCK, not per object. Roughly 3000 of
  the objects in this scene are linked duplicates sharing ~60 meshes (trees,
  rocks, rubble, props), so this turns a 3500-file, several-hundred-megabyte
  export into about sixty files. Per-object scale is carried on <mesh><scale>.
* Writes ONE world file with every object as a <model>. Ground truth is never
  exported: GROUND_TRUTH is not linked into SAR_EXPORT.
* Groups the parts of each human into a SINGLE model with one link and several
  visuals, instead of seven separate models. A walking person you have to move
  by driving seven poses in lockstep is a trap; one model, one pose.
* Chooses each collision primitive from the object's `collision` custom
  property:
      MESH_EXACT          -> the mesh itself (terrain, roads, bridge deck)
      BOX / BOX_COMPOUND  -> box from the world-space bounding box
      CYLINDER            -> cylinder from bbox radius/height
      CONVEX_HULL         -> the mesh, for the engine to decompose
      CAPSULE_APPROX      -> capsule from bbox (humans)
      CYLINDER_TRUNK_ONLY -> trunk only; nobody pays for leaf collision
      BOX_LOW             -> reduced box (bushes: soft obstacle)
      NONE_VISUAL_ONLY    -> visual only, no <collision>
* Objects carrying physics="DYNAMIC" export as NON-STATIC bodies with the mass,
  principal inertia and surface parameters computed in the generator. Those are
  the 77 props that only move when something pushes them.
* Thermal metadata becomes a <temperature> on each visual, in Kelvin, for the
  Gazebo thermal camera system. It is a configuration handoff, not a
  simulation -- see the thermal note in README.md.

Options
-------
  --out PATH        output directory (default ./gazebo_export)
  --format glb|dae|obj   mesh format; falls back automatically if unavailable.
                    glb is the default because the Debian/Ubuntu Blender
                    package ships with the COLLADA exporter compiled out.
  --models          ALSO write models/<name>/model.sdf + model.config for every
                    object, for use as a standalone model library.
  --no-thermal      omit the per-visual Thermal <plugin> blocks. gz-sim parses
                    each one in a bare <sdf> wrapper and logs
                    "XML Element[plugin], child of element[sdf], not defined in
                    SDF" for every visual -- ~740 warnings at startup. They are
                    harmless, but under -v 4 the noise dominates the log and
                    slows the load. Temperatures remain in sar_thermal_table.csv
                    and sar_metadata.json either way.
  --actors          compatibility alias: scripted motion is now ON by default.
  --no-actors       freeze the scripted movers for comparison.
                    Regular mesh models use the local sar-waypoint-system
                    plugin. Build it with CMake or use run_gazebo.sh.
  --no-batch        one model per object instead of merged scatter batches.
                    That is ~2900 models, which Gazebo does not load
                    comfortably. Kept for comparison, not for use.
  --verbose         do not suppress Blender's per-export chatter. Without this,
                    the glTF addon's output is muted, because it prints
                    "ERROR: Draco mesh compression is not available" once per
                    mesh -- 712 times -- which looks exactly like a failure and
                    is not one. Draco is optional compression we do not use.
"""
import bpy, os, sys, json, math, time, contextlib
from mathutils import Vector, Matrix, Euler
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sar_motion import timed_route, plugin_element
from sar_physics import configure_world


def args():
    a = sys.argv
    a = a[a.index("--") + 1:] if "--" in a else []
    out = "./gazebo_export"
    fmt = "glb"
    models = "--models" in a
    verbose = "--verbose" in a
    batch = "--no-batch" not in a
    thermal = "--no-thermal" not in a
    actors = "--no-actors" not in a and "--static" not in a
    if "--out" in a:
        out = a[a.index("--out") + 1]
    if "--format" in a:
        fmt = a[a.index("--format") + 1].lower().lstrip(".")
    return out, fmt, models, verbose, batch, actors, thermal


OUT, FMT, WRITE_MODELS, VERBOSE, BATCH, ACTORS, THERMAL = args()


@contextlib.contextmanager
def quiet():
    """Mute the exporter addon at the file-descriptor level.

    The glTF addon logs through C-level stdout/stderr, so a python-side
    logging filter does not reach it. Its loudest line is a Draco compression
    ERROR emitted once per mesh whether or not compression is requested; over
    712 meshes that buries the actual progress and reads as a crash.
    """
    if VERBOSE:
        yield
        return
    sys.stdout.flush(); sys.stderr.flush()
    old1, old2 = os.dup(1), os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1); os.dup2(devnull, 2)
    try:
        yield
    finally:
        sys.stdout.flush(); sys.stderr.flush()
        os.dup2(old1, 1); os.dup2(old2, 2)
        os.close(devnull); os.close(old1); os.close(old2)


def say(msg):
    print(msg)
    sys.stdout.flush()
MESHES = os.path.join(OUT, "meshes")
MODELS = os.path.join(OUT, "models")
os.makedirs(MESHES, exist_ok=True)
if WRITE_MODELS:
    os.makedirs(MODELS, exist_ok=True)

exp = bpy.data.collections.get("SAR_EXPORT")
if exp is None:
    raise SystemExit("SAR_EXPORT collection not found -- wrong .blend?")


def _have(opname):
    mod, fn = opname.split(".")
    return hasattr(getattr(bpy.ops, mod), fn)


def pick_format(fmt):
    order = {"glb": ["glb", "dae", "obj"], "gltf": ["glb", "dae", "obj"],
             "dae": ["dae", "glb", "obj"], "obj": ["obj", "glb", "dae"]}.get(
                 fmt, ["glb", "dae", "obj"])
    for f in order:
        if f == "glb" and _have("export_scene.gltf"):
            return "glb"
        if f == "dae" and _have("wm.collada_export"):
            return "dae"
        if f == "obj" and (_have("wm.obj_export") or _have("export_scene.obj")):
            return "obj"
    raise SystemExit("no usable mesh exporter in this Blender build")


FMT = pick_format(FMT)
EXT = "." + FMT
say("mesh format: %s%s" % (EXT, "" if VERBOSE else
                           "   (exporter output muted; pass --verbose for it)"))


def export_selected(path):
    """Z-up, metres, no scene transform -- what Gazebo expects."""
    if FMT == "glb":
        bpy.ops.export_scene.gltf(filepath=path, export_format='GLB',
                                  use_selection=True, export_yup=False,
                                  export_apply=True)
    elif FMT == "dae":
        try:
            bpy.ops.wm.collada_export(filepath=path, selected=True,
                                      apply_modifiers=True,
                                      export_global_forward_selection='Y',
                                      export_global_up_selection='Z')
        except TypeError:
            bpy.ops.wm.collada_export(filepath=path, selected=True)
    else:
        if _have("wm.obj_export"):
            bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True,
                                  up_axis='Z', forward_axis='Y')
        else:
            bpy.ops.export_scene.obj(filepath=path, use_selection=True,
                                     axis_up='Z', axis_forward='Y')


def safe(name):
    return "".join(c if (c.isalnum() or c in "._-") else "_" for c in name)


bpy.context.view_layer.update()
objs = [o for o in exp.all_objects if o.type == 'MESH']

# ---------------------------------------------------------------------------
# 0. batching
# ---------------------------------------------------------------------------
# Gazebo charges per MODEL, not per mesh file: an SDF parse, an Ogre scene node
# and a physics body each. The first version of this exporter emitted one model
# per object -- 2922 of them, 2494 being scattered trees, bushes, rocks and
# rubble -- and Gazebo fell over loading it. Scatter is now merged into one
# model per (family, 50 m cell): one joined visual mesh, and cheap primitive
# collisions inside that single link.
#
# The fidelity trade is deliberate. A scattered rock exports as an oriented box
# rather than a convex hull, because ~1000 mesh collisions is what makes the
# solver crawl, and a box is an honest stand-in for a 40 cm rock the robot must
# not drive through. Everything the robot interacts with on purpose -- terrain,
# roads, the bridge deck, buildings, vehicles, humans and the 77 dynamic props
# -- keeps its exact collision. --no-batch restores one model per object.
BATCH_FAMILY = {
    "TREE": "VEG", "BUSH": "VEG", "GRASS": "VEG", "FALLEN_TREE": "VEG",
    "ROCK": "ROCK", "BOULDER": "ROCK",
    "RUBBLE_PIECE": "RUBBLE", "DEBRIS": "RUBBLE",
}
BATCH_TEMP = {"VEG": 20.0, "ROCK": 14.0, "RUBBLE": 22.0}
CELL = 50.0


def group_root(o):
    p = o.parent
    if p is not None and p.type == 'EMPTY' and p.get("sar_object") in (
            "HUMAN", "DECOY"):
        return p
    return None


bpy.context.scene.frame_set(1)   # actor meshes are authored at the start pose
batches = {}
groups = {}
actor_groups = {}
singles = []
for o in objs:
    fam = BATCH_FAMILY.get(o.get("sar_object")) if BATCH else None
    if fam and o.get("physics") != "DYNAMIC":
        t = o.matrix_world.translation
        key = (fam, int(math.floor(t.x / CELL)), int(math.floor(t.y / CELL)))
        batches.setdefault(key, []).append(o)
        continue
    # Keep ordinary model visuals/collisions; attach motion below.
    r = group_root(o)
    if r is None:
        if o.get("gazebo_actor") and o.get("path_waypoints_xyz"):
            actor_groups.setdefault(o.name, (o, []))[1].append(o)
        singles.append(o)
    else:
        if r.get("gazebo_actor") and r.get("path_waypoints_xyz"):
            actor_groups.setdefault(r.name, (r, []))[1].append(o)
        groups.setdefault(r.name, (r, []))[1].append(o)

need_mesh = list(singles) + [o for (_r, ps) in groups.values() for o in ps]
n_unique = len({o.data.name for o in need_mesh})
n_batched = sum(len(v) for v in batches.values())
say("objects: %d total -- %d batched into %d merged models, %d individual, "
    "%d available as scripted movers"
    % (len(objs), n_batched, len(batches), len(objs) - n_batched,
       len(actor_groups)))
say("unique meshes to export: %d (+ %d merged)" % (n_unique, len(batches)))
say("this takes a minute or two -- let it run")
_t0 = time.time()

# ---------------------------------------------------------------------------
# 1. one mesh file per unique datablock
# ---------------------------------------------------------------------------
mesh_file = {}
for o in need_mesh:
    key = o.data.name
    if key in mesh_file:
        continue
    fn = safe(key) + EXT
    mesh_file[key] = fn
    # Export a detached, unanimated copy. Zeroing a child's local transform
    # still exports its parent's world transform in glTF (double placement in
    # Gazebo). The SDF alone must own position, rotation and scale.
    mesh_object = o.copy()
    mesh_object.parent = None
    mesh_object.animation_data_clear()
    mesh_object.constraints.clear()
    mesh_object.matrix_parent_inverse = Matrix.Identity(4)
    mesh_object.matrix_world = Matrix.Identity(4)
    bpy.context.scene.collection.objects.link(mesh_object)
    bpy.ops.object.select_all(action='DESELECT')
    mesh_object.select_set(True)
    bpy.context.view_layer.objects.active = mesh_object
    try:
        with quiet():
            export_selected(os.path.join(MESHES, fn))
    finally:
        bpy.data.objects.remove(mesh_object, do_unlink=True)
    n = len(mesh_file)
    if n % 50 == 0 or n == n_unique:
        el = time.time() - _t0
        say("  meshes %4d/%d  (%.0fs elapsed, ~%.0fs left)"
            % (n, n_unique, el, el / max(1, n) * (n_unique - n)))
bpy.context.view_layer.update()
say("unique meshes exported: %d in %.0fs" % (len(mesh_file), time.time() - _t0))


# ---------------------------------------------------------------------------
# 2. one merged mesh per batch
# ---------------------------------------------------------------------------
def merge_export(parts, xform, path):
    """Join copies of `parts`, each placed by xform(part), and export once."""
    with quiet():
        bpy.ops.object.select_all(action='DESELECT')
        copies = []
        for o in parts:
            c = o.copy()
            c.data = o.data.copy()
            bpy.context.scene.collection.objects.link(c)
            c.matrix_world = xform(o)
            copies.append(c)
        for c in copies:
            c.select_set(True)
        bpy.context.view_layer.objects.active = copies[0]
        if len(copies) > 1:
            bpy.ops.object.join()
        joined = bpy.context.view_layer.objects.active
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        export_selected(path)
        me = joined.data
        bpy.data.objects.remove(joined, do_unlink=True)
        if me.users == 0:
            bpy.data.meshes.remove(me)


batch_file = {}
if batches:
    _t1 = time.time()
    for bi, (key, members) in enumerate(sorted(batches.items())):
        fam, ci, cj = key
        # NB: no "+" in the filename. URI parsing commonly decodes "+"
        # as a space, which silently breaks the mesh lookup.
        bname = "batch_%s_%s%03d_%s%03d" % (
            fam, "n" if ci < 0 else "p", abs(ci),
            "n" if cj < 0 else "p", abs(cj))
        with quiet():
            bpy.ops.object.select_all(action='DESELECT')
            copies = []
            for o in members:
                c = o.copy()
                c.data = o.data.copy()
                bpy.context.scene.collection.objects.link(c)
                c.matrix_world = o.matrix_world.copy()
                copies.append(c)
            for c in copies:
                c.select_set(True)
            bpy.context.view_layer.objects.active = copies[0]
            if len(copies) > 1:
                bpy.ops.object.join()
            joined = bpy.context.view_layer.objects.active
            bpy.ops.object.transform_apply(location=True, rotation=True,
                                           scale=True)
            export_selected(os.path.join(MESHES, bname + EXT))
            me = joined.data
            bpy.data.objects.remove(joined, do_unlink=True)
            if me.users == 0:
                bpy.data.meshes.remove(me)
        batch_file[key] = bname + EXT
        if (bi + 1) % 10 == 0 or bi + 1 == len(batches):
            say("  merged %3d/%d batches (%.0fs)"
                % (bi + 1, len(batches), time.time() - _t1))
    bpy.context.view_layer.update()
say("models: %d individual + %d grouped + %d merged batches"
    % (len(singles), len(groups), len(batches)))


def c2k(c):
    return c + 273.15


def dims_of(o):
    bb = [tuple(v) for v in o.bound_box]
    scale = o.matrix_world.to_scale()
    return tuple((max(b[i] for b in bb) - min(b[i] for b in bb)) * abs(scale[i])
                 for i in range(3))


def mesh_uri(o):
    return "meshes/" + mesh_file[o.data.name]


def scale_xml(o):
    s = o.matrix_world.to_scale()
    if abs(s.x - 1) < 1e-6 and abs(s.y - 1) < 1e-6 and abs(s.z - 1) < 1e-6:
        return ""
    return "<scale>%.5f %.5f %.5f</scale>" % (s.x, s.y, s.z)


def surface_xml(o):
    if o.get("physics") != "DYNAMIC":
        mu = o.get("friction_mu")
        if mu is None:
            return ""
        return ('<surface><friction><ode><mu>%.2f</mu><mu2>%.2f</mu2></ode>'
                '</friction></surface>' % (float(mu), float(mu)))
    mu = float(o.get("friction_mu", 0.5))
    rest = float(o.get("restitution", 0.1))
    return ('<surface><friction><ode><mu>%.2f</mu><mu2>%.2f</mu2></ode>'
            '</friction><bounce><restitution_coefficient>%.2f'
            '</restitution_coefficient><threshold>0.05</threshold></bounce>'
            '<contact><ode><max_vel>0.1</max_vel><min_depth>0.001</min_depth>'
            '</ode></contact></surface>' % (mu, mu, rest))


def collision_xml(o, name, pose=""):
    coll = o.get("collision", "CONVEX_HULL")
    if coll == "NONE_VISUAL_ONLY":
        return ""
    sx, sy, sz = dims_of(o)
    surf = surface_xml(o)
    if coll in ("BOX", "BOX_COMPOUND"):
        geom = "<box><size>%.4f %.4f %.4f</size></box>" % (sx, sy, sz)
    elif coll == "CYLINDER":
        geom = ("<cylinder><radius>%.4f</radius><length>%.4f</length>"
                "</cylinder>" % (max(sx, sy) * 0.5, sz))
    elif coll == "CAPSULE_APPROX":
        # A box, not a <capsule>. Capsule is legal SDF 1.10 and Harmonic
        # supports it, but it is one more thing that has to line up across
        # sdformat, gz-physics and the render backend, and for a proxy limb the
        # difference is not worth the exposure.
        geom = "<box><size>%.4f %.4f %.4f</size></box>" % (sx, sy, sz)
    elif coll == "CYLINDER_TRUNK_ONLY":
        geom = ("<cylinder><radius>%.4f</radius><length>%.4f</length>"
                "</cylinder>" % (max(0.06, min(sx, sy) * 0.18), sz * 0.7))
    elif coll == "BOX_LOW":
        geom = "<box><size>%.4f %.4f %.4f</size></box>" % (
            sx * 0.7, sy * 0.7, sz * 0.5)
    else:
        geom = "<mesh><uri>%s</uri>%s</mesh>" % (mesh_uri(o), scale_xml(o))
    if coll in ("BOX", "BOX_COMPOUND", "CYLINDER", "CAPSULE_APPROX",
                "CYLINDER_TRUNK_ONLY", "BOX_LOW"):
        # Body-part vertices are authored above the origin (e.g. the head at
        # 1.6 m). A box at the origin would collide with the feet instead.
        scale = o.matrix_world.to_scale()
        bb = [tuple(v) for v in o.bound_box]
        center = Vector(tuple((min(b[i] for b in bb) + max(b[i] for b in bb))
                              * 0.5 * scale[i] for i in range(3)))
        if coll in ("CYLINDER_TRUNK_ONLY", "BOX_LOW"):
            fraction = 0.35 if coll == "CYLINDER_TRUNK_ONLY" else 0.25
            center.z = min(b[2] for b in bb) * scale.z + sz * fraction
        values = list(map(float, ET.fromstring(pose).text.split())) if pose else [0.0] * 6
        center = Vector(values[:3]) + Euler(values[3:], 'XYZ').to_matrix() @ center
        pose = '<pose>%.5f %.5f %.5f %.5f %.5f %.5f</pose>' % (*center, *values[3:])
    return ('<collision name="%s">%s<geometry>%s</geometry>%s</collision>'
            % (name, pose, geom, surf))


def visual_xml(o, name, pose=""):
    t = o.get("thermal_temp_c")
    if t is None and o.get("thermal_class"):
        t = 20.0
    therm = ""
    if t is not None and THERMAL:
        therm = ('<plugin filename="gz-sim-thermal-system" '
                 'name="gz::sim::systems::Thermal"><temperature>%.2f'
                 '</temperature></plugin>' % c2k(float(t)))
    return ('<visual name="%s">%s<geometry><mesh><uri>%s</uri>%s</mesh>'
            '</geometry>%s</visual>'
            % (name, pose, mesh_uri(o), scale_xml(o), therm))


def batch_collision(o, name):
    """Primitive collision for a batched object, posed in world coordinates --
    the merged model sits at the origin with its geometry already baked in."""
    coll = o.get("collision", "CONVEX_HULL")
    if coll == "NONE_VISUAL_ONLY":
        return ""
    sx, sy, sz = dims_of(o)
    mw = o.matrix_world
    bb = [mw @ Vector(c) for c in o.bound_box]
    cx = (min(v.x for v in bb) + max(v.x for v in bb)) * 0.5
    cy = (min(v.y for v in bb) + max(v.y for v in bb)) * 0.5
    zlo = min(v.z for v in bb)
    cz = (zlo + max(v.z for v in bb)) * 0.5
    e = mw.to_euler('XYZ')
    if coll == "CYLINDER_TRUNK_ONLY":
        r = max(0.06, min(sx, sy) * 0.18)
        h = sz * 0.7
        pose = "%.3f %.3f %.3f 0 0 0" % (cx, cy, zlo + h * 0.5)
        geom = ("<cylinder><radius>%.3f</radius><length>%.3f</length>"
                "</cylinder>" % (r, h))
    elif coll == "BOX_LOW":
        pose = "%.3f %.3f %.3f 0 0 %.4f" % (cx, cy, zlo + sz * 0.25, e.z)
        geom = "<box><size>%.3f %.3f %.3f</size></box>" % (
            sx * 0.7, sy * 0.7, sz * 0.5)
    else:
        pose = "%.3f %.3f %.3f %.4f %.4f %.4f" % (cx, cy, cz, e.x, e.y, e.z)
        geom = "<box><size>%.3f %.3f %.3f</size></box>" % (sx, sy, sz)
    return ('<collision name="%s"><pose>%s</pose><geometry>%s</geometry>'
            '</collision>' % (name, pose, geom))


def pose_xml(mat):
    t = mat.to_translation()
    e = mat.to_euler('XYZ')
    return "%.4f %.4f %.4f %.5f %.5f %.5f" % (t.x, t.y, t.z, e.x, e.y, e.z)


def inertial_xml(o):
    if o.get("physics") != "DYNAMIC":
        return ""
    return ('      <inertial><pose>0 0 0 0 0 0</pose><mass>%.4f</mass>'
            '<inertia><ixx>%.6f</ixx><ixy>0</ixy><ixz>0</ixz><iyy>%.6f</iyy>'
            '<iyz>0</iyz><izz>%.6f</izz></inertia></inertial>\n'
            % (float(o.get("mass_kg", 10.0)), float(o.get("ixx", 0.1)),
               float(o.get("iyy", 0.1)), float(o.get("izz", 0.1))))


def meta_comment(o):
    keep = ("sar_object", "semantic_class", "terrain_type", "traversability",
            "traverse_cost", "friction_mu", "risk_level", "thermal_class",
            "thermal_temp_c", "search_zone", "sar_sector", "person_id",
            "victim_status", "thermal_contrast", "occlusion", "hazard_id",
            "hazard_type", "severity", "prop_kind", "mass_kg", "decoy",
            "motion_pattern", "route_exposure", "building_id", "physics")
    d = {k: o[k] for k in keep if k in o.keys()}
    return json.dumps(d).replace("--", "- -")


def motion_xml(root):
    keys = timed_route(json.loads(root["path_waypoints_xyz"]),
                       float(root.get("walk_speed_mps", 1.0)),
                       (float(root.get("dwell_min_s", 0.0)) +
                        float(root.get("dwell_max_s", 0.0))) / 2)
    # Local +Y is forward; +X is right. The generator defines lateral as left.
    offset = (-float(root.get("lateral_offset_m", 0.0)),
              float(root.get("lead_offset_m", 0.0)),
              float(root.get("actor_z_offset", 0.0)))
    return ET.tostring(plugin_element(keys, offset), encoding="unicode")


# ---------------------------------------------------------------------------
# 3. the world
# ---------------------------------------------------------------------------
W = []
A = W.append
A('<?xml version="1.0" ?>')
A('<sdf version="1.10">')
A('  <world name="military_world">')
A('    <!-- NON-WEAPONISED search-and-rescue / disaster-response range. -->')
A('    <!-- Generated by export_gazebo.py. Ground truth is NOT in here. -->')
A('    <physics name="default" type="dart">')
A('      <max_step_size>0.002</max_step_size>')
A('      <real_time_factor>1.0</real_time_factor>')
A('    </physics>')
A('    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>')
A('    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">')
A('      <render_engine>ogre2</render_engine>')
A('    </plugin>')
A('    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>')
A('    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>')
A('    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>')
A('    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>')
A('    <!-- No world-level Thermal system: it takes no <temperature> at world')
A('         scope and gz-sim logs an error for it. Temperatures are declared')
A('         per-visual, which is where the Thermal system actually reads them. -->')
A('    <gravity>0 0 -9.81</gravity>')
A('    <scene>')
A('      <ambient>0.45 0.45 0.45 1</ambient>')
A('      <background>0.36 0.50 0.74 1</background>')
A('      <grid>false</grid>')
A('      <!-- FOG preset: see sar_metadata.json weather_presets -->')
A('      <!-- <fog><type>linear</type><density>0.02</density>')
A('           <start>5</start><end>35</end></fog> -->')
A('    </scene>')
A('    <light type="directional" name="sun">')
A('      <cast_shadows>true</cast_shadows>')
A('      <pose>0 0 120 0 0 0</pose>')
A('      <diffuse>1.0 0.97 0.92 1</diffuse>')
A('      <specular>0.25 0.25 0.25 1</specular>')
A('      <direction>-0.5 0.35 -0.78</direction>')
A('    </light>')

HEADER = list(W)          # everything emitted so far
BLOCKS = []               # (kind, [lines]) -- one entry per model


def add_block(kind, lines):
    BLOCKS.append((kind, lines))


n_static = n_dyn = 0
for o in singles:
    L = []
    a = L.append
    dynamic = (o.get("physics") == "DYNAMIC")
    a('    <model name="%s">' % safe(o.name))
    a('      <static>%s</static>' % ("false" if dynamic else "true"))
    a('      <pose>%s</pose>' % pose_xml(o.matrix_world))
    a('      <!-- %s -->' % meta_comment(o))
    a('      <link name="link">')
    ix = inertial_xml(o)
    if ix:
        a(ix.rstrip("\n"))
    a('        ' + visual_xml(o, "v"))
    c = collision_xml(o, "c")
    if c:
        a('        ' + c)
    a('      </link>')
    if ACTORS and o.name in actor_groups:
        a('      ' + motion_xml(o))
    a('    </model>')
    if dynamic:
        add_block("dynamic", L)
        n_dyn += 1
    else:
        add_block("terrain" if o.get("sar_object") == "TERRAIN_TILE"
                  else "single", L)
        n_static += 1

for key, members in sorted(batches.items()):
    fam, ci, cj = key
    L = []
    a = L.append
    bname = "SAR_Batch_%s_%s%03d_%s%03d" % (
        fam, "n" if ci < 0 else "p", abs(ci),
        "n" if cj < 0 else "p", abs(cj))
    a('    <model name="%s">' % safe(bname))
    a('      <static>true</static>')
    a('      <pose>0 0 0 0 0 0</pose>')
    a('      <!-- {"batch_family": "%s", "cell": [%d, %d], "members": %d, '
      '"collision": "PRIMITIVE_PER_MEMBER"} -->' % (fam, ci, cj, len(members)))
    a('      <link name="link">')
    bt = ('<plugin filename="gz-sim-thermal-system" '
          'name="gz::sim::systems::Thermal"><temperature>%.2f</temperature>'
          '</plugin>' % c2k(BATCH_TEMP[fam])) if THERMAL else ''
    a('        <visual name="v"><geometry><mesh><uri>meshes/%s</uri></mesh>'
      '</geometry>%s</visual>' % (batch_file[key], bt))
    nc = 0
    for m in members:
        cxml = batch_collision(m, "c%04d" % nc)
        if cxml:
            a('        ' + cxml)
            nc += 1
    a('      </link>')
    a('    </model>')
    add_block("batch", L)
    n_static += 1

for gname, (root, parts) in groups.items():
    L = []
    a = L.append
    # A model pose has no scale; keep inherited scale in each mesh and shape.
    frame = root.matrix_world.to_quaternion().to_matrix().to_4x4()
    frame.translation = root.matrix_world.translation
    inv = frame.inverted()
    a('    <model name="%s">' % safe(root.name))
    a('      <static>true</static>')
    a('      <pose>%s</pose>' % pose_xml(root.matrix_world))
    a('      <!-- %s -->' % meta_comment(root))
    a('      <link name="body">')
    for i, pp in enumerate(parts):
        rel = "<pose>%s</pose>" % pose_xml(inv @ pp.matrix_world)
        a('        ' + visual_xml(pp, "v%02d" % i, rel))
        c = collision_xml(pp, "c%02d" % i, rel)
        if c:
            a('        ' + c)
    a('      </link>')
    if ACTORS and root.name in actor_groups:
        a('      ' + motion_xml(root))
    a('    </model>')
    add_block("humanstatic" if root.get("gazebo_actor") else "group", L)
    n_static += 1


def emit(path, blocks):
    out = list(HEADER)
    for _kind, lines in blocks:
        out.extend(lines)
    out.append('  </world>')
    out.append('</sdf>')
    with open(path, "w") as f:
        root = ET.fromstring("\n".join(out), parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
        configure_world(root)
        f.write(ET.tostring(root, encoding="unicode") + "\n")
    return len(blocks)


MAIN = list(BLOCKS)
world_path = os.path.join(OUT, "military_world.sdf")
emit(world_path, MAIN)

# ---------------------------------------------------------------------------
# staged test worlds
# ---------------------------------------------------------------------------
# If the full world will not load, these localise the cause in one pass instead
# of a guessing game. Run them in order; the first that fails names the layer
# that is wrong:
#   stage1  one terrain mesh          -> mesh format / loader / resource path
#   stage2  all 36 terrain tiles      -> mesh count or terrain collision cost
#   stage3  terrain + structures      -> a specific building/vehicle/human model
#   stage4  + batched scatter         -> the merged meshes
#   full    + the 77 dynamic props    -> non-static bodies / inertia
terr = [b for b in BLOCKS if b[0] == "terrain"]
stages = [
    ("stage1_one_mesh.sdf", terr[:1]),
    ("stage2_terrain.sdf", terr),
    ("stage3_structures.sdf", [b for b in BLOCKS if b[0] in ("terrain", "single",
                                                             "group")]),
    ("stage4_scatter.sdf", [b for b in MAIN if b[0] != "dynamic"]),
    ("stage5_movers.sdf", terr + [b for b in MAIN
      if any('sar::WaypointSystem' in line for line in b[1])]),
]
for fn, blocks in stages:
    n = emit(os.path.join(OUT, fn), blocks)
    say("  staged test world %-24s %4d models" % (fn, n))

say("world written: %s" % world_path)
say("  models in military_world.sdf: %d; scripted movers: %d"
    % (len(MAIN), len(actor_groups) if ACTORS else 0))

# Small, mesh-identical demo: two people on clear crossing routes near origin.
# Does not alter the full world's authored routes or terrain.
import copy
movers = [ET.fromstring("\n".join(lines)) for _, lines in MAIN
          if any('sar::WaypointSystem' in line for line in lines)]
people = [m for m in movers if m.get("name", "").startswith("SAR_DynamicPerson")]
if people:
    demo = [('ground', ['<model name="demo_ground"><static>true</static>'
            '<pose>0 0 -0.1 0 0 0</pose><link name="ground">'
            '<collision name="c"><geometry><box><size>24 24 0.2</size></box></geometry></collision>'
            '<visual name="v"><geometry><box><size>24 24 0.2</size></box></geometry>'
            '<material><ambient>0.35 0.4 0.3 1</ambient><diffuse>0.35 0.4 0.3 1</diffuse>'
            '</material></visual></link></model>'])]
    for model, route in zip(people[:2], [
            [(-4, 0, 0.03), (4, 0, 0.03), (4, 4, 0.03), (-4, 4, 0.03), (-4, 0, 0.03)],
            [(0, -4, 0.03), (0, 4, 0.03), (-4, 4, 0.03), (-4, -4, 0.03), (0, -4, 0.03)]]):
        model = copy.deepcopy(model)
        model.remove(model.find("plugin[@name='sar::WaypointSystem']"))
        keys = timed_route(route, 1.2)
        model.find("pose").text = " ".join(map(str, keys[0][1]))
        model.append(plugin_element(keys))
        demo.append(('mover', [ET.tostring(model, encoding="unicode")]))
    emit(os.path.join(OUT, "dynamic_demo.sdf"), demo)

# ---------------------------------------------------------------------------
# 4. optional standalone model library
# ---------------------------------------------------------------------------
if WRITE_MODELS:
    for o in objs:
        mdir = os.path.join(MODELS, safe(o.name))
        os.makedirs(mdir, exist_ok=True)
        dynamic = (o.get("physics") == "DYNAMIC")
        with open(os.path.join(mdir, "model.sdf"), "w") as f:
            f.write('<?xml version="1.0" ?>\n<sdf version="1.10">\n')
            f.write('  <model name="%s">\n' % safe(o.name))
            f.write('    <static>%s</static>\n'
                    % ("false" if dynamic else "true"))
            f.write('    <!-- %s -->\n' % meta_comment(o))
            f.write('    <link name="link">\n')
            f.write(inertial_xml(o))
            f.write('      %s\n' % visual_xml(o, "v").replace(
                "meshes/", "../../meshes/"))
            c = collision_xml(o, "c").replace("meshes/", "../../meshes/")
            if c:
                f.write('      %s\n' % c)
            f.write('    </link>\n  </model>\n</sdf>\n')
        with open(os.path.join(mdir, "model.config"), "w") as f:
            f.write('<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n'
                    '  <version>1.0</version>\n'
                    '  <sdf version="1.10">model.sdf</sdf>\n'
                    '  <description>%s</description>\n</model>\n'
                    % (safe(o.name), o.get("sar_object", "SAR object")))
    say("model library written: %s" % MODELS)

say("done -> %s" % OUT)
say("")
say("run it with:")
say("  export GZ_SIM_RESOURCE_PATH=%s" % os.path.abspath(OUT))
say("  export GZ_SIM_SYSTEM_PLUGIN_PATH=%s" % os.path.join(os.path.dirname(os.path.abspath(__file__)), "build"))
say("  gz sim -r %s" % os.path.abspath(world_path))
