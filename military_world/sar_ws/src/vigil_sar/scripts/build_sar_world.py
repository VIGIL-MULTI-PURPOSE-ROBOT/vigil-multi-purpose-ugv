#!/usr/bin/python3
"""Generate the SAR simulation world from the untouched military_world export.

    python3 build_sar_world.py [--config config/sar_mission.yaml] [--out FILE] [--check]

Input  : <military_world>/gazebo_export/military_world.sdf   (never modified)
Output : <military_world>/sar_ws/generated/military_sar.sdf  (+ _manifest.json)

What changes (everything else - terrain, buildings, vehicles, hazards - is byte-for-byte
the same geometry):
  1. Human heat signature. Gazebo Harmonic's thermal camera reads a per-VISUAL
     <plugin gz::sim::systems::Thermal><temperature> (K). Only models whose semantic tag is
     "sar_object": "HUMAN" are rewritten, from human_thermal.* in the config:
     head / torso / limbs, LOW-contrast (covered) casualties, and a contrast scale.
     Non-human temperatures are left as exported (concrete 293 K, rock 287 K, warm-case
     decoy 314 K ...); a report lists any non-human visual that falls inside the human band.
  2. Casualties inside buildings (world.add_building_humans): standing humans placed
     2-3 m behind the ground-floor doors of URB_001, URB_002 and URB_004 (door gaps and
     line of sight were ray-checked against the wall/interior meshes). They reuse the
     exported human meshes and get the same human heat.
  3. Physics as vigil_rough_terrain: Earth gravity, 1 ms step, DART default engine settings, ground
     friction mu 1.0 (world.ground_friction); <atmosphere> temperature (thermal background),
     segmentation label 1 on terrain / roads / apron / bridge (cliff "void" test).
  4. Walking people: sar_physics.configure_world() from military_world (unchanged code),
     controller from world.people_controller; or frozen if world.moving_people is false.
  5. Mesh URIs made absolute so the generated file can live outside gazebo_export/.
"""
import argparse
import copy
import importlib.util
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from sar_paths import find_military_world, load_config  # noqa: E402

THERMAL = 'gz::sim::systems::Thermal'
LABEL_FILE, LABEL_NAME = 'gz-sim-label-system', 'gz::sim::systems::Label'
GROUND_OBJECTS = ('TERRAIN_TILE', 'APRON', 'ROAD', 'BRIDGE')
GROUND_CLASSES = ('GROUND', 'ROAD', 'BRIDGE')     # semantic_class of drivable surfaces

# Casualties inside buildings. Floor heights from the slab meshes, door gaps from the
# ground-floor wall meshes (ray test at 0.6 m and 1.5 m), line of sight from the street
# checked against walls, interior partitions and furniture.
BUILDING_HUMANS = [
    dict(name='SAR_BuildingPerson_101', building='SAR_Building_URB_001', x=70.3, y=46.6, floor=0.16, yaw=-1.5708,
         note='Standing 2.6 m behind the south door (x 69.8-70.8) of the office block.'),
    dict(name='SAR_BuildingPerson_102', building='SAR_Building_URB_002', x=88.8, y=47.6, floor=0.32, yaw=-1.5708,
         note='Standing 3.6 m behind the south door (x 88.3-89.3); URB_002 has no interior walls.'),
    dict(name='SAR_BuildingPerson_103', building='SAR_Building_URB_004', x=112.4, y=52.0, floor=0.42, yaw=-1.3,
         note='Warehouse hall, visible through the 4.5 m roller entrance (x 110.2-114.7).'),
    dict(name='SAR_BuildingPerson_104', building='SAR_Building_URB_004', x=114.2, y=53.4, floor=0.42, yaw=-1.9,
         note='Second casualty 2.3 m from 103: tests that two close humans get two ids.'),
]
TEMPLATE_HUMAN = 'SAR_StaticPerson_009'      # standing, exported meshes


def semantic(model):
    for node in model:
        if node.tag is ET.Comment and node.text and node.text.strip().startswith('{'):
            try:
                return json.loads(node.text.strip()), node
            except ValueError:
                return {}, node
    return {}, None


def part_of(visual):
    uri = visual.findtext('geometry/mesh/uri') or ''
    for key in ('Head', 'Torso', 'Hips', 'Arm', 'Leg'):
        if f'_{key}' in uri:
            return key
    return 'Torso'


def human_temperature(part, contrast_class, ht, ambient):
    if contrast_class == 'LOW':
        t = float(ht.get('covered_temperature', 297.65))
    elif part == 'Head':
        t = float(ht.get('head_temperature', 307.65))
    elif part in ('Arm', 'Leg'):
        t = float(ht.get('limb_temperature', 303.65))
    else:
        t = float(ht.get('body_temperature', 305.15))
    k = float(ht.get('thermal_contrast', 1.0))
    return ambient + k * (t - ambient)


def set_temperature(visual, temp):
    plug = visual.find(f"plugin[@name='{THERMAL}']")
    if plug is None:
        plug = ET.SubElement(visual, 'plugin', filename='gz-sim-thermal-system', name=THERMAL)
    node = plug.find('temperature')
    if node is None:
        node = ET.SubElement(plug, 'temperature')
    node.text = f'{temp:.2f}'


def visual_temps(model):
    out = []
    for v in model.iter('visual'):
        t = v.findtext(f"plugin[@name='{THERMAL}']/temperature")
        if t is not None:
            out.append(float(t))
    return out


def load_sar_physics(mw):
    spec = importlib.util.spec_from_file_location('sar_physics', mw / 'sar_physics.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

HULL_CLASSES = ('WALL', 'FENCE', 'STRUCTURE')


def pose_of(node):
    values = [float(v) for v in (node.findtext('pose') or '0 0 0 0 0 0').split()]
    return (values + [0.0] * 6)[:6]


# Segmentation label per obstacle category (world.label_obstacles). Ground keeps label 1 (the cliff
# void test in terrain_core only asks "label 0 or not", so extra labels change nothing there).
# obstacle_core.py reads the same numbers from config obstacles.labels.
OBSTACLE_LABELS = dict(HUMAN=10, VEHICLE=20, BUILDING=30, ROCK=40, DEBRIS=50, VEGETATION=60, OBJECT=70)


def obstacle_category(model, sem=None):
    """HUMAN / VEHICLE / BUILDING / ROCK / DEBRIS / VEGETATION / OBJECT, or None for the ground.
    Many export models carry no semantic tag (SAR_Building_URB_*, SAR_Batch_ROCK_*, SAR_Vehicle_*),
    so the model name decides when the tag does not."""
    sem = semantic(model)[0] if sem is None else sem
    obj = (sem.get('sar_object') or '').upper()
    cls = (sem.get('semantic_class') or '').upper()
    name = (model.get('name') or '').upper()
    if obj in GROUND_OBJECTS or cls in GROUND_CLASSES or name.startswith(('SAR_TERRAIN', 'SAR_ROAD', 'SAR_STREET',
                                                                          'SAR_TRACK')):
        return None
    if obj == 'HUMAN' or cls == 'HUMAN' or 'PERSON' in name:
        return 'HUMAN'
    if cls == 'VEHICLE' or 'VEHICLE' in name:
        return 'VEHICLE'
    if cls == 'ROCK' or 'ROCK' in name:
        return 'ROCK'
    if cls == 'RUBBLE' or any(k in name for k in ('RUBBLE', 'DEBRIS', 'PANEL', 'PIPE')):
        return 'DEBRIS'
    if cls in ('VEGETATION', 'TREE') or any(k in name for k in ('VEG', 'TREE', 'BUSH', '_LOG')):
        return 'VEGETATION'
    if cls in ('WALL', 'BUILDING', 'STRUCTURE') or ('SAR_BUILDING' in name and 'FURNITURE' not in name):
        return 'BUILDING'
    if 'WATER' in name or cls == 'WATER':
        return None
    return 'OBJECT'


def _rpy(r, p, y):
    import math
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def single_box_collision(link, name='body_box'):
    """Replace every box collision of a link by ONE box: the axis-aligned (link frame) bounding box of
    all their rotated corners. A person's 7 body-part boxes become one 0.5 x 0.4 x 1.7 m box; the rover
    still stops against the whole person, the physics tests 1 shape instead of 7."""
    cols = [c for c in link.findall('collision') if c.find('geometry/box/size') is not None]
    if not cols:
        return None
    lo, hi = [1e9] * 3, [-1e9] * 3
    for c in cols:
        sx, sy, sz = [float(v) for v in c.findtext('geometry/box/size').split()]
        px, py, pz, r, p_, y = pose_of(c)
        R = _rpy(r, p_, y)
        for dx in (-sx / 2, sx / 2):
            for dy in (-sy / 2, sy / 2):
                for dz in (-sz / 2, sz / 2):
                    q = [(px, py, pz)[k] + R[k][0] * dx + R[k][1] * dy + R[k][2] * dz for k in range(3)]
                    lo = [min(a, b) for a, b in zip(lo, q)]
                    hi = [max(a, b) for a, b in zip(hi, q)]
    surface = cols[0].find('surface')
    index = list(link).index(cols[0])
    for c in cols:
        link.remove(c)
    new = ET.Element('collision', name=name)
    ET.SubElement(new, 'pose').text = ' '.join(f'{(a + b) / 2:.4f}' for a, b in zip(lo, hi)) + ' 0 0 0'
    ET.SubElement(ET.SubElement(ET.SubElement(new, 'geometry'), 'box'), 'size').text = \
        ' '.join(f'{b - a:.4f}' for a, b in zip(lo, hi))
    if surface is not None:
        new.append(copy.deepcopy(surface))
    link.insert(index, new)
    return [b - a for a, b in zip(lo, hi)], len(cols)


def _part_of_collision(link, col):
    """Body part of an exported collision box: c00..c06 pair with visuals v00..v06 (same index)."""
    name = col.get('name') or ''
    vis = link.find(f"visual[@name='v{name[1:]}']") if name.startswith('c') else None
    return part_of(vis) if vis is not None else 'Torso'


def primitive_collisions(link):
    """Replace a person's 7 exported body-part BOXES by primitives that follow the body:
         head          -> sphere
         torso, hips   -> upright cylinder (the trunk)
         legs, arms    -> cylinder along the limb
    Each primitive keeps its box's pose (position AND the limb's roll / pitch), so the collision body
    stands, lies or leans exactly like the visual person, and it is part of the person's own link:
    it moves with the person whatever moves it. Radii cover the box cross-section (conservative:
    the rover is kept off the whole limb). Returns [(part, shape, dims)]."""
    out = []
    for col in list(link.findall('collision')):
        size = col.find('geometry/box/size')
        if size is None:
            continue
        sx, sy, sz = [float(v) for v in size.text.split()]
        part = _part_of_collision(link, col)
        geom = col.find('geometry')
        for child in list(geom):
            geom.remove(child)
        if part == 'Head':
            r = 0.5 * max(sx, sy, sz)
            ET.SubElement(ET.SubElement(geom, 'sphere'), 'radius').text = f'{r:.4f}'
            out.append((part, 'sphere', [round(r, 3)]))
        else:
            r = 0.5 * max(sx, sy)
            cyl = ET.SubElement(geom, 'cylinder')
            ET.SubElement(cyl, 'radius').text = f'{r:.4f}'
            ET.SubElement(cyl, 'length').text = f'{sz:.4f}'
            out.append((part, 'cylinder', [round(r, 3), round(sz, 3)]))
        col.set('name', f"{col.get('name')}_{part.lower()}")
    return out


def scale_walker_speed(model, scale, max_speed):
    """Walkers (controller 1) follow their timed waypoints exactly, so their speed is the authored one
    (0.3 - 1.85 m/s in the export: realistic walking). world.human_speed_scale multiplies it (by
    compressing the waypoint times) and world.human_max_speed caps every leg. Returns the new top speed."""
    import math
    plug = model.find("plugin[@name='sar::WaypointSystem']")
    if plug is None:
        return None
    wps = plug.findall('waypoint')
    keys = [(float(w.findtext('time')), [float(v) for v in w.findtext('pose').split()]) for w in wps]
    new_t, top, t_acc = [0.0], 0.0, 0.0
    for (t0, a), (t1, b) in zip(keys, keys[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        dt = (t1 - t0) / max(1e-6, scale)
        if max_speed and dist / max(dt, 1e-6) > max_speed:
            dt = dist / max_speed
        t_acc += dt
        new_t.append(t_acc)
        top = max(top, dist / max(dt, 1e-6))
    for w, t in zip(wps, new_t):
        w.find('time').text = f'{t:.6f}'
    return top


def fix_hull_collisions(world, cfg, report):
    """Replace bounding-box collisions that are wrong with the model's own mesh.

    The export gives every model ONE box collision, sized from the mesh bounding box. That is
    right for a crate, a slab or a wall, but wrong for a shell: SAR_Base_Fence, the fence around
    the home base, became a SOLID 50 x 34 x 2 m block centred on (0, -16) - and the robot start
    (0, -8) is inside it. The rover spawned inside that invisible block, was held in the air at
    spawn height (the 2026-09-23 "rover floats / cannot reach B" run: the depth camera then saw
    the real ground 0.66 m below and called it a HIGH CLIFF, and the whole base was one obstacle,
    so no path to B existed). The same hull problem hits the bridge parapets and the building
    interior walls. The mesh collision of those models is exact (posts, rails, the gate opening).
    Visuals, poses and every other collision stay exactly as exported.
    """
    w, sp, rb = cfg['world'], cfg['spawn'], cfg['robot']
    if not w.get('fix_hull_collisions', True):
        return
    area_limit = float(w.get('hull_collision_area', 60.0))
    sx, sy, sz = float(sp.get('x', 0.0)), float(sp.get('y', -8.0)), float(sp.get('z', 0.3))
    reach_x = float(rb.get('length', 1.53)) / 2 + 0.5
    reach_y = float(rb.get('width', 1.12)) / 2 + 0.5
    for model in world.findall('model'):
        cls = (semantic(model)[0].get('semantic_class') or '').upper()
        mp = pose_of(model)
        for link in model.iter('link'):
            visual = next((v for v in link.findall('visual') if v.find('geometry/mesh') is not None), None)
            for col in list(link.findall('collision')):
                size = col.find('geometry/box/size')
                if size is None:
                    continue
                bx, by, bz = [float(v) for v in size.text.split()]
                cp = pose_of(col)
                cx, cy, cz = mp[0] + cp[0], mp[1] + cp[1], mp[2] + cp[2]
                top, bottom = cz + bz / 2, cz - bz / 2
                blocks_start = (abs(cx - sx) < bx / 2 + reach_x and abs(cy - sy) < by / 2 + reach_y
                                and top > sz + 0.05 and bottom < sz + 1.8)
                hull = cls in HULL_CLASSES and bx * by >= area_limit and min(bx, by) >= 2.0
                if not (blocks_start and top > sz + 0.05) and not hull:
                    continue
                why = 'covers the robot start' if blocks_start else f'{bx * by:.0f} m2 box around a {cls.lower()}'
                if visual is not None:
                    # take the visual's geometry AND its pose: the box pose was the bounding-box
                    # centre, which would shift the mesh; the mesh carries its own placement.
                    col.remove(col.find('geometry'))
                    col.append(copy.deepcopy(visual.find('geometry')))
                    old_pose = col.find('pose')
                    if old_pose is not None:
                        col.remove(old_pose)
                    vis_pose = visual.find('pose')
                    if vis_pose is not None:
                        col.insert(0, copy.deepcopy(vis_pose))
                    action = 'mesh collision'
                else:
                    link.remove(col)
                    action = 'collision removed (no mesh)'
                report['collision_fixes'].append(dict(model=model.get('name'), semantic_class=cls,
                                                      box=[bx, by, bz], reason=why, action=action))
    # anything still standing on the robot start is worth shouting about
    for model in world.findall('model'):
        mp = pose_of(model)
        for col in model.iter('collision'):
            size = col.find('geometry/box/size')
            if size is None:
                continue
            bx, by, bz = [float(v) for v in size.text.split()]
            cp = pose_of(col)
            cx, cy, cz = mp[0] + cp[0], mp[1] + cp[1], mp[2] + cp[2]
            if (abs(cx - sx) < bx / 2 + reach_x and abs(cy - sy) < by / 2 + reach_y
                    and cz + bz / 2 > sz + 0.05 and cz - bz / 2 < sz + 1.8):
                report['start_blocked_by'].append(model.get('name'))


def build(cfg, mw, out_path=None):
    w, ht, tc = cfg['world'], cfg['human_thermal'], cfg['thermal_camera']
    src = mw / w.get('source_sdf', 'gazebo_export/military_world.sdf')
    export_dir = src.parent.resolve()
    out = Path(out_path) if out_path else mw / w.get('generated_sdf', 'sar_ws/generated/military_sar.sdf')
    ambient = float(w.get('ambient_temperature_k', 293.15))
    band = (float(tc.get('temperature_threshold', 296.5)), float(tc.get('human_max_temperature', 311.0)))
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(src, parser=parser)
    root = tree.getroot()
    world = root.find('world')
    report = dict(source=str(src), output=str(out), humans=[], labelled=0, in_band_non_human=[],
                  building_humans=[], movers=0, thermal_plugins=0, ambient_plugins_dropped=0,
                  collision_fixes=[], start_blocked_by=[])

    # 4. walking people (existing military_world code), before we touch anything else
    movers = [m for m in world.findall('model') if m.find("plugin[@name='sar::WaypointSystem']") is not None]
    # walkers outside the operational zone stand still: the rover never goes there, and each walker is
    # a system update every physics step. They stay in the world, visible, at their start pose.
    zone = tuple(w.get(k) for k in ('zone_x_min', 'zone_x_max', 'zone_y_min', 'zone_y_max'))
    report['frozen_outside_zone'] = []
    if w.get('freeze_people_outside_zone', True) and None not in zone:
        for m in list(movers):
            x, y = pose_of(m)[:2]
            if not (zone[0] <= x <= zone[1] and zone[2] <= y <= zone[3]):
                for plug in m.findall("plugin[@name='sar::WaypointSystem']"):
                    m.remove(plug)
                st = m.find('static')
                if st is None:
                    st = ET.SubElement(m, 'static')
                st.text = 'true'
                movers.remove(m)
                report['frozen_outside_zone'].append(m.get('name'))
    if w.get('moving_people', True):
        ctrl = int(w.get('people_controller', 1))
        load_sar_physics(mw).configure_world(root, ctrl)
        report['movers'] = len(movers)
        report['people_controller'] = ctrl
        if ctrl == 2:
            # contact-aware people (military_world sar::WaypointSystem controller 2): dynamic bodies
            # under gravity, walked by a FORCE towards their route and kept upright by a balance torque.
            # They are real rigid bodies: the rover cannot overlap them, and a person blocked by the
            # rover stops (its route clock stops) instead of being teleported through it. Their
            # walking speed is set here (sar_physics uses 1.8 / 2.2 m/s, a jog): world.human_contact_speed.
            v = float(w.get('human_contact_speed', 1.3))
            vmax_c = float(w.get('human_max_speed', 0.0) or 0.0)
            if vmax_c > 0:
                v = min(v, vmax_c)
            for m in movers:
                plug = m.find("plugin[@name='sar::WaypointSystem']")
                node = plug.find('cruise_speed')
                if node is not None and plug.findtext('carried') != 'true':
                    node.text = f'{v:g}'
            report['contact_walk_speed'] = v
    else:
        for m in movers:
            for plug in m.findall("plugin[@name='sar::WaypointSystem']"):
                m.remove(plug)
            st = m.find('static')
            if st is None:
                st = ET.SubElement(m, 'static')
            st.text = 'true'

    # 3. physics step + thermal background
    phys = world.find('physics')
    phys.find('max_step_size').text = str(float(w.get('physics_step', 0.001)))
    # simulation speed = Gazebo's target real-time factor. Only HOW FAST the simulation executes: the
    # step, gravity, masses, friction and torques are unchanged, so every motion is physically the same.
    rtf = phys.find('real_time_factor')
    if rtf is None:
        rtf = ET.SubElement(phys, 'real_time_factor')
    rtf.text = f"{max(0.1, float(w.get('simulation_speed', 1.0))):g}"
    # gravity pinned to Earth's (physics.engine.gravity, 9.81 m/s2) instead of trusting the export
    g_node = world.find('gravity')
    if g_node is None:
        g_node = ET.Element('gravity')
        world.insert(list(world).index(phys), g_node)
    g_node.text = f"0 0 {-abs(float(w.get('gravity', 9.81))):g}"
    atm = world.find('atmosphere')
    if atm is None:
        atm = ET.Element('atmosphere', type='adiabatic')
        world.insert(list(world).index(phys) + 1, atm)
    t_node = atm.find('temperature')
    if t_node is None:
        t_node = ET.SubElement(atm, 'temperature')
    t_node.text = f'{ambient:.2f}'

    # Same physics engine settings in EVERY run, and the same ones as vigil_rough_terrain. Its
    # worlds (rock_terrain.sdf, test/flat.sdf) carry NO <dart> block: DART's own collision
    # detector and LCP solver. The export ships DART with bullet + PGS, and
    # sar_physics.configure_world() removes that block only for the kinematic people controller,
    # so the engine used to depend on world.moving_people. collision_detector "default" (the
    # vigil_rough_terrain setting) removes the block in every run; a detector name pins that one.
    contact_people = bool(w.get('moving_people', True)) and int(w.get('people_controller', 1)) == 2
    # the rover's physics was validated with world.collision_detector; contact people use the SAME
    # detector unless world.people_contact_detector is "sar_physics" (its bullet / PGS choice)
    if contact_people and str(w.get('people_contact_detector', 'world')).strip().lower() == 'world':
        contact_people = False
    detector = str(w.get('collision_detector', 'default')).strip().lower()
    solver_type = str(w.get('physics_solver', 'default')).strip().lower()
    if not contact_people:
        dart = phys.find('dart')
        if detector in ('', 'default', 'none'):
            if dart is not None:
                phys.remove(dart)
        else:
            if dart is None:
                dart = ET.SubElement(phys, 'dart')
            node = dart.find('collision_detector')
            if node is None:
                node = ET.SubElement(dart, 'collision_detector')
            node.text = detector
            solver = dart.find('solver')
            if solver_type in ('', 'default', 'none'):
                if solver is not None:
                    dart.remove(solver)
            else:
                if solver is None:
                    solver = ET.SubElement(dart, 'solver')
                node = solver.find('solver_type')
                if node is None:
                    node = ET.SubElement(solver, 'solver_type')
                node.text = solver_type
    dart = phys.find('dart')
    report['physics'] = dict(step=float(w.get('physics_step', 0.001)), gravity=g_node.text,
                             real_time_factor=float(rtf.text),
                             collision_detector=((dart.findtext('collision_detector') or 'default')
                                                 if dart is not None else 'default (DART, as vigil_rough_terrain)'),
                             solver=((dart.findtext('solver/solver_type') or 'default')
                                     if dart is not None else 'default (DART, as vigil_rough_terrain)'))

    # Ground friction as vigil_rough_terrain: its ground (models/rocky_terrain_n4, test worlds) is
    # mu = mu2 = 1.0. The export gives every ground tile its own value - gravel 0.55, sand 0.35,
    # snow 0.30, mud 0.28, water 0.05 - and Gazebo uses the SMALLER of tyre (0.85) and ground, so
    # on the A -> B route the rover had 35-70 % less grip than in vigil_rough_terrain: wheels spun
    # on the 8 m/s2 ramp, it slid in turns and could not climb what the rough rover climbs.
    # world.ground_friction (physics.engine.ground_friction) sets terrain / road / track / apron /
    # bridge surfaces; "export" keeps the exported values. Obstacles and props are untouched.
    gf = w.get('ground_friction', 1.0)
    report['ground_friction'] = dict(value=gf, surfaces=0, changed={})
    if str(gf).strip().lower() not in ('export', 'none', ''):
        gf = float(gf)
        for model in world.findall('model'):
            sem = semantic(model)[0]
            if not (sem.get('sar_object') in GROUND_OBJECTS or
                    (sem.get('semantic_class') or '').upper() in GROUND_CLASSES):
                continue
            for col in model.iter('collision'):
                surf = col.find('surface')
                if surf is None:
                    surf = ET.SubElement(col, 'surface')
                fr = surf.find('friction')
                if fr is None:
                    fr = ET.SubElement(surf, 'friction')
                ode = fr.find('ode')
                if ode is None:
                    ode = ET.SubElement(fr, 'ode')
                for tag in ('mu', 'mu2'):
                    node = ode.find(tag)
                    if node is None:
                        node = ET.SubElement(ode, tag)
                    old_mu = node.text
                    node.text = f'{gf:g}'
                    if tag == 'mu':
                        key = f'{float(old_mu):.2f}' if old_mu else 'unset'
                        report['ground_friction']['changed'][key] = report['ground_friction']['changed'].get(key, 0) + 1
                report['ground_friction']['surfaces'] += 1

    template = None
    for model in world.findall('model'):
        sem, _ = semantic(model)
        obj = sem.get('sar_object')
        if model.get('name') == TEMPLATE_HUMAN:
            template = model
        if obj == 'HUMAN':
            # 1. human heat signature (HUMAN models only)
            for v in model.iter('visual'):
                set_temperature(v, human_temperature(part_of(v), sem.get('thermal_contrast'), ht, ambient))
            report['humans'].append(dict(name=model.get('name'), person_id=sem.get('person_id'),
                                         contrast=sem.get('thermal_contrast'),
                                         temps=sorted(set(round(t, 2) for t in visual_temps(model)))))
        else:
            hot = [t for t in visual_temps(model) if band[0] <= t <= band[1]]
            if hot:
                report['in_band_non_human'].append(dict(name=model.get('name'), temps=sorted(set(hot))))
            if w.get('label_ground', True) and obj in GROUND_OBJECTS:
                pose = model.find('pose')
                lab = ET.Element('plugin', filename=LABEL_FILE, name=LABEL_NAME)
                ET.SubElement(lab, 'label').text = '1'
                model.insert(list(model).index(pose) + 1 if pose is not None else 0, lab)
                report['labelled'] += 1

    # 2. casualties inside buildings
    if w.get('add_building_humans', True):
        if template is None:
            raise RuntimeError(f'{TEMPLATE_HUMAN} not found in {src}; cannot add building humans')
        tpl_z = float(template.findtext('pose').split()[2])
        for h in BUILDING_HUMANS:
            m = copy.deepcopy(template)
            m.set('name', h['name'])
            for plug in m.findall("plugin[@name='sar::WaypointSystem']"):
                m.remove(plug)
            m.find('static').text = 'true'
            m.find('pose').text = f"{h['x']:.4f} {h['y']:.4f} {h['floor'] + tpl_z:.4f} 0 0 {h['yaw']:.5f}"
            sem, node = semantic(m)
            sem.update(person_id=h['name'].replace('SAR_BuildingPerson_', 'BLDG_'), search_zone='SAR_ZONE_C',
                       sar_sector='URBAN', occlusion='BUILDING_INTERIOR', victim_status='TRAPPED',
                       thermal_contrast='HIGH', building_id=h['building'], added_by='vigil_sar/build_sar_world.py')
            if node is not None:
                node.text = ' ' + json.dumps(sem) + ' '
            for v in m.iter('visual'):
                set_temperature(v, human_temperature(part_of(v), 'HIGH', ht, ambient))
            world.append(m)
            report['building_humans'].append(dict(h, temps=sorted(set(round(t, 2) for t in visual_temps(m)))))

    # 10. obstacle geometry, labels and walking speed (dynamic-obstacle avoidance)
    #   a) world.human_collision "primitives": EVERY person (walking, standing, lying, trapped, inside a
    #      building) gets a collision body made of primitives that follow the body parts - sphere head,
    #      cylinder trunk, cylinder legs and arms - in the person's own link, so it moves with the person.
    #      ("box" = one box round the whole body, "parts" = the export's 7 boxes.)
    #   b) world.label_obstacles: a segmentation label per category (HUMAN 10, VEHICLE 20, BUILDING 30,
    #      ROCK 40, DEBRIS 50, VEGETATION 60, OBJECT 70) on every obstacle model inside the zone, so
    #      obstacle_tracker.py knows WHAT the depth camera sees (a person gets a wider berth than a crate).
    #   c) world.human_speed_scale / human_max_speed: walking speed stays the authored, realistic one
    #      unless configured otherwise; it never follows the simulation speed (that is Gazebo's RTF).
    report['human_collision_boxes'] = []
    report['obstacle_labels'] = {}
    zone10 = [w.get(k) for k in ('zone_x_min', 'zone_x_max', 'zone_y_min', 'zone_y_max')]
    lab_margin = float(w.get('collision_zone_margin', 5.0))
    scale = float(w.get('human_speed_scale', 1.0))
    vmax = float(w.get('human_max_speed', 0.0) or 0.0)
    report['walker_speed'] = dict(scale=scale, max_speed=vmax, top_speeds={})
    for model in world.findall('model'):
        sem = semantic(model)[0]
        cat = obstacle_category(model, sem)
        walking = model.find("plugin[@name='sar::WaypointSystem']") is not None
        if cat == 'HUMAN':
            mode = str(w.get('human_collision', 'primitives')).strip().lower()
            if not walking and 'static_human_collision' in w:
                mode = str(w['static_human_collision']).strip().lower()
            for link in model.findall('link'):
                if mode == 'primitives':
                    parts = primitive_collisions(link)
                    if parts:
                        report['human_collision_boxes'].append(dict(model=model.get('name'), walking=walking,
                                                                    shapes=parts))
                elif mode == 'box':
                    res = single_box_collision(link)
                    if res:
                        report['human_collision_boxes'].append(dict(model=model.get('name'), walking=walking,
                                                                    shapes=[('body', 'box', [round(v, 3) for v in res[0]])]))
        if model.find("plugin[@name='sar::WaypointSystem']") is not None and (scale != 1.0 or vmax > 0.0):
            top = scale_walker_speed(model, scale, vmax)
            if top is not None:
                report['walker_speed']['top_speeds'][model.get('name')] = round(top, 2)
        if cat and w.get('label_obstacles', True):
            x, y = pose_of(model)[:2]
            inside = None in zone10 or (zone10[0] - lab_margin <= x <= zone10[1] + lab_margin and
                                        zone10[2] - lab_margin <= y <= zone10[3] + lab_margin)
            # batch models (one model, many rocks) sit at the origin with world-placed shapes
            if not inside and x == 0.0 and y == 0.0:
                inside = True
            if inside and model.find(f"plugin[@name='{LABEL_NAME}']") is None:
                lab = ET.Element('plugin', filename=LABEL_FILE, name=LABEL_NAME)
                ET.SubElement(lab, 'label').text = str(OBSTACLE_LABELS[cat])
                pose = model.find('pose')
                model.insert(list(model).index(pose) + 1 if pose is not None else 0, lab)
                report['obstacle_labels'][cat] = report['obstacle_labels'].get(cat, 0) + 1

    # 6. every visual whose <temperature> IS the ambient temperature carries a per-visual Thermal
    #    system for nothing: Gazebo renders a visual without one at exactly that ambient temperature
    #    (the atmosphere gradient varies it by <1 K, far below the 296.5 K human band). The export
    #    ships ~390 of them; each one is a system instance in the server AND in the GUI, where they
    #    also print thousands of "XML Element[plugin], child of element[sdf]" warnings and block the
    #    Qt main thread while loading - the reason the GUI showed "not responding / Force Quit".
    #    Geometry, poses, materials and every non-ambient temperature stay exactly as exported.
    if w.get('drop_ambient_thermal_plugins', True):
        for model in world.findall('model'):
            if semantic(model)[0].get('sar_object') == 'HUMAN':
                continue          # a human keeps every plugin, whatever its temperature works out to
            for visual in model.iter('visual'):
                plug = visual.find(f"plugin[@name='{THERMAL}']")
                if plug is None:
                    continue
                temp = plug.findtext('temperature')
                try:
                    is_ambient = temp is not None and abs(float(temp) - ambient) < 0.05
                except ValueError:
                    is_ambient = False
                if is_ambient:
                    visual.remove(plug)
                    report['ambient_plugins_dropped'] += 1
    report['thermal_plugins'] = sum(1 for v in world.iter('visual')
                                    if v.find(f"plugin[@name='{THERMAL}']") is not None)

    # 8. simulation cost. The rover's own physics stays exactly vigil_rough_terrain's (1 ms step,
    #    same springs, same friction); what made military_world run at ~6 % of real time is the
    #    WORLD: every physics step (1000 / s) DART also integrated 77 loose props sitting on the
    #    terrain and tested the 7 body boxes of every walking person against the 36 terrain meshes.
    #    a) world.static_props: barrels / cones / crates / pallets become static - they look and
    #       collide exactly the same (the rover still hits them), they just cannot be knocked over.
    #    b) world.people_ignore_ground: kinematic walkers (controller 1 sets their pose every step,
    #       they never rest on the ground) skip the pointless person-vs-terrain contact tests via
    #       collide_bitmask. People still collide with the rover, props and buildings.
    report['static_props'] = 0
    if w.get('static_props', True):
        for model in world.findall('model'):
            if (semantic(model)[0].get('sar_object') or '') == 'DYNAMIC_PROP':
                st = model.find('static')
                if st is None:
                    st = ET.SubElement(model, 'static')
                if st.text != 'true':
                    st.text = 'true'
                    report['static_props'] += 1
    report['people_ignore_ground'] = 0
    kinematic = bool(w.get('moving_people', True)) and int(w.get('people_controller', 1)) == 1
    contact_walkers = bool(w.get('moving_people', True)) and int(w.get('people_controller', 1)) == 2
    report['collide_bitmasks'] = None

    def set_mask(col, mask):
        surf = col.find('surface')
        if surf is None:
            surf = ET.SubElement(col, 'surface')
        cont = surf.find('contact')
        if cont is None:
            cont = ET.SubElement(surf, 'contact')
        node = cont.find('collide_bitmask')
        if node is None:
            node = ET.SubElement(cont, 'collide_bitmask')
        node.text = mask
    if contact_walkers:
        # Contact collision groups (a pair collides when their masks share a bit):
        #   ground 0x01 | static world 0x02 | walking people 0x05 | the rover 0xffff (default, untouched)
        #   people x ground  : 0x05 & 0x01 -> YES  (they stand and walk on the terrain)
        #   people x rover   : 0x05 & 0xffff -> YES (real contact: the rover can never overlap a person)
        #   people x people  : 0x05 & 0x05 -> YES
        #   people x static world : 0x05 & 0x02 -> no (authored routes pass door frames / props, as before)
        #   rover x everything: YES
        for model in world.findall('model'):
            sem = semantic(model)[0]
            if (sem.get('semantic_class') or '').upper() in GROUND_CLASSES or sem.get('sar_object') in GROUND_OBJECTS:
                mask = '0x01'
            elif model.find("plugin[@name='sar::WaypointSystem']") is not None:
                mask = '0x05'
            elif w.get('walkers_hit_only_rover', True):
                mask = '0x02'
            else:
                continue
            for col in model.iter('collision'):
                set_mask(col, mask)
        report['collide_bitmasks'] = dict(ground='0x01', static_world='0x02', walkers='0x05', rover='0xffff')
    if w.get('people_ignore_ground', True) and kinematic:
        def set_mask(col, mask):
            surf = col.find('surface')
            if surf is None:
                surf = ET.SubElement(col, 'surface')
            cont = surf.find('contact')
            if cont is None:
                cont = ET.SubElement(surf, 'contact')
            node = cont.find('collide_bitmask')
            if node is None:
                node = ET.SubElement(cont, 'collide_bitmask')
            node.text = mask
        for model in world.findall('model'):
            sem = semantic(model)[0]
            if (sem.get('semantic_class') or '').upper() in GROUND_CLASSES or sem.get('sar_object') in GROUND_OBJECTS:
                for col in model.iter('collision'):
                    set_mask(col, '0x01')            # ground: bit 1
            elif model.find("plugin[@name='sar::WaypointSystem']") is not None:
                for col in model.iter('collision'):
                    set_mask(col, '0x02')            # walker: bit 2 -> no ground contact
                report['people_ignore_ground'] += 1
            elif w.get('walkers_hit_only_rover', True):
                for col in model.iter('collision'):
                    set_mask(col, '0x01')            # static world: bit 1 -> no walker contact tests
        # the rover keeps the default 0xffff and hits the ground, the static world AND the walkers.
        # A walker (0x02) is tested only against the rover: kinematic people walk their authored
        # routes (through a door frame now and then) without thousands of pointless wall contacts.

    # 9. collision shapes the rover can never touch (measured 2026-09-24, diagnosis/measure_physics.log:
    #    current 6.5 % real time, people frozen 17 %, object collisions removed 51 %, ground only 100 %).
    #    The world has ~3,100 collision shapes (every tree, bush, rock and rubble piece is made of
    #    boxes) and DART tests them against each other and the moving bodies every 1 ms step. Cameras
    #    and the thermal camera see VISUALS, not collision shapes, so nothing changes for perception:
    #    a) world.collision_zone_only: objects outside the operational zone (+ margin) keep their
    #       looks but lose their collision shapes - the rover never drives there.
    #    b) world.collision_max_bottom: shapes that start higher than this above the model's base
    #       (tree canopies, upper floors, roof parts) cannot touch a 1.14 m rover.
    #    c) world.walker_collisions false: the kinematic walkers are placed on their route every step
    #       and already walk through walls and the rover (military_world README); their collision
    #       boxes only cost time. They stay visible and warm; the depth camera still sees them.
    zone = [w.get(k) for k in ('zone_x_min', 'zone_x_max', 'zone_y_min', 'zone_y_max')]
    margin = float(w.get('collision_zone_margin', 5.0))
    max_bottom = w.get('collision_max_bottom', None)
    drop_walkers = not w.get('walker_collisions', True) and kinematic
    report['collisions_removed'] = dict(outside_zone=0, too_high=0, walkers=0, kept=0)

    def shape_height(col):
        g = col.find('geometry')
        g = g[0] if g is not None and len(g) else None
        if g is None:
            return None
        if g.tag == 'box':
            return float(g.findtext('size').split()[2])
        if g.tag == 'cylinder':
            return float(g.findtext('length'))
        if g.tag == 'sphere':
            return 2.0 * float(g.findtext('radius'))
        return None                                    # meshes / planes: keep
    for model in world.findall('model'):
        sem = semantic(model)[0]
        if (sem.get('semantic_class') or '').upper() in GROUND_CLASSES or sem.get('sar_object') in GROUND_OBJECTS:
            continue                                   # the ground always collides
        walker = model.find("plugin[@name='sar::WaypointSystem']") is not None
        if obstacle_category(model, sem) == 'HUMAN' and w.get('humans_always_collide', True) \
                and not (walker and drop_walkers):
            report['collisions_removed']['kept'] += sum(1 for _ in model.iter('collision'))
            continue                                   # every person keeps its (single) box, everywhere
        mp = pose_of(model)
        for link in model.iter('link'):
            lp = pose_of(link) if link.find('pose') is not None else [0.0] * 6
            for col in list(link.findall('collision')):
                why = None
                if walker and drop_walkers:
                    why = 'walkers'
                elif w.get('collision_zone_only', True) and None not in zone:
                    cp = pose_of(col)
                    x, y = mp[0] + lp[0] + cp[0], mp[1] + lp[1] + cp[1]
                    if not (zone[0] - margin <= x <= zone[1] + margin and zone[2] - margin <= y <= zone[3] + margin):
                        why = 'outside_zone'
                if why is None and max_bottom is not None and not walker:
                    h = shape_height(col)
                    if h is not None:
                        bottom = lp[2] + pose_of(col)[2] - h / 2.0
                        if bottom > float(max_bottom):
                            why = 'too_high'
                if why:
                    link.remove(col)
                    report['collisions_removed'][why] += 1
                else:
                    report['collisions_removed']['kept'] += 1

    # 7. collisions that trap the rover (see fix_hull_collisions)
    fix_hull_collisions(world, cfg, report)

    # 5. absolute mesh URIs
    for uri in root.iter('uri'):
        if uri.text and uri.text.startswith('meshes/'):
            uri.text = 'file://' + str(export_dir / uri.text)

    out.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out, encoding='utf-8', xml_declaration=True)
    report['n_humans'] = len(report['humans']) + len(report['building_humans'])
    man = out.with_name(out.stem + '_manifest.json')
    man.write_text(json.dumps(report, indent=1))
    return report


def main():
    here = Path(os.path.realpath(__file__)).parent
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=str(here.parent / 'config/sar_mission.yaml'))
    ap.add_argument('--military-world', default='')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    cfg = load_config(a.config)
    mw = find_military_world(a.military_world or cfg['world'].get('military_world_dir', ''))
    rep = build(cfg, mw, a.out or None)
    print(f"[build_sar_world] {rep['output']}")
    print(f"  humans with heat signature: {rep['n_humans']} ({len(rep['building_humans'])} added inside buildings)")
    print(f"  simulation cost: {rep.get('static_props', 0)} props made static, {rep.get('people_ignore_ground', 0)} "
          f"walkers skip terrain contact")
    cr = rep.get('collisions_removed', {})
    print(f"  collision shapes: {cr.get('kept', 0)} kept; removed {cr.get('outside_zone', 0)} outside the zone, "
          f"{cr.get('too_high', 0)} out of the rover's reach, {cr.get('walkers', 0)} of walkers")
    print(f"  segmentation-labelled ground models: {rep['labelled']}, walking people: {rep['movers']} "
          f"({len(rep.get('frozen_outside_zone', []))} outside the operational zone stand still)")
    hb = rep.get('human_collision_boxes', [])
    print(f"  simulation speed target {rep['physics']['real_time_factor']:g}x real time; people with primitive "
          f"collision bodies: {len(hb)} ({sum(1 for h in hb if h.get('walking'))} walking, controller "
          f"{rep.get('people_controller', '-')}, {rep.get('contact_walk_speed', '-')} m/s); collide bitmasks "
          f"{rep.get('collide_bitmasks')}; obstacle labels {rep.get('obstacle_labels', {})}")
    print(f"  physics: {rep['physics']['step'] * 1000:.0f} ms step, {rep['physics']['collision_detector']} "
          f"collision detector, {rep['physics']['solver']} solver, gravity {rep['physics']['gravity']}")
    gfr = rep['ground_friction']
    print(f"  ground friction: {gfr['value']} on {gfr['surfaces']} drivable surfaces (export values were {gfr['changed']})")
    print(f"  per-visual thermal plugins kept: {rep['thermal_plugins']} "
          f"(dropped {rep['ambient_plugins_dropped']} that only repeated the ambient temperature)")
    for fix in rep['collision_fixes']:
        print(f"  collision fix: {fix['model']} -> {fix['action']} ({fix['reason']})")
    if rep['start_blocked_by']:
        print(f"  WARNING something still stands on the robot start: {rep['start_blocked_by']}")
    if rep['in_band_non_human']:
        print(f"  WARNING non-human visuals inside the human band: {rep['in_band_non_human']}")
    else:
        print('  no non-human object is inside the human temperature band')


if __name__ == '__main__':
    main()
