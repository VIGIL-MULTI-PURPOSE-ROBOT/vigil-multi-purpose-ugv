#!/usr/bin/python3
"""Gazebo test worlds for the drive / navigation scenarios (flat road, moderate slope, steep climb,
obstacle, cliff, and the PEOPLE scenarios) - used by test_motion.sh --scenarios.

People scenarios (real military_world person meshes and the SAME primitive collision body and
contact-aware walking controller as the SAR world - build_sar_world.primitive_collisions and the
sar::WaypointSystem controller 2):
  human_block     a standing person 5 m ahead; the TEST drives straight at them (no navigator):
                  the rover must be stopped by the contact, never pass through
  human_standing  a standing person on the A-B line; the navigator must keep its distance and reach B
  human_crossing  a person walking back and forth across the A-B line; no contact, reach B

    python3 scenario_worlds.py OUT_DIR            # writes OUT_DIR/<name>.sdf + OUT_DIR/scenarios.json
    python3 scenario_worlds.py --list

The geometry of moderate_slope, steep_hill and cliff_front is vigil_rough_terrain's
scripts/scenarios.py (read, not changed), turned into this package's frame: the rover spawns at
A = (0, -8) facing +y (config spawn), so rough-terrain (x, y) -> (-y, x - 8). Physics as the SAR world
and vigil_rough_terrain: Earth gravity, DART defaults, 1 ms step, ground friction mu 1.0. Every
surface carries segmentation label 1 (ground) or 2 (obstacle) for the cliff "void" test.
"""
import copy
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

A = (0.0, -8.0)            # spawn (config/sar_mission.yaml spawn), heading +y
CLIFF_H = 1.5


def to_sar(x, y):
    return -y + A[0], x + A[1]


def plateau(x0, x1, y0, y1, h, holes=()):
    """vigil_rough_terrain scenarios.plateau: a plateau (top at h) minus rectangular holes."""
    xs = sorted({x0, x1, *[max(x0, min(x1, v)) for hx0, hx1, _, _ in holes for v in (hx0, hx1)]})
    ys = sorted({y0, y1, *[max(y0, min(y1, v)) for _, _, hy0, hy1 in holes for v in (hy0, hy1)]})
    out = []
    for a, b in zip(xs[:-1], xs[1:]):
        for c, d in zip(ys[:-1], ys[1:]):
            mx, my = (a + b) / 2, (c + d) / 2
            if any(hx0 <= mx <= hx1 and hy0 <= my <= hy1 for hx0, hx1, hy0, hy1 in holes):
                continue
            if b - a > 1e-6 and d - c > 1e-6:
                out.append(dict(pos=(mx, my, h / 2), size=(b - a, d - c, h), pitch=0.0, label=1))
    return out


def ramp(x0, x1, y0, y1, angle_deg, thickness=0.6):
    """vigil_rough_terrain scenarios.ramp: top face rises from (x0, 0) to (x1, (x1-x0) tan a)."""
    a = math.radians(angle_deg)
    L = (x1 - x0) / math.cos(a)
    top_mid = ((x0 + x1) / 2, ((x1 - x0) * math.tan(a)) / 2)
    nx, nz = -math.sin(a), math.cos(a)
    cx, cz = top_mid[0] - nx * thickness / 2, top_mid[1] - nz * thickness / 2
    return [dict(pos=(cx, (y0 + y1) / 2, cz), size=(L, y1 - y0, thickness), pitch=-a, label=1)]


def wall(x, y0, y1, height=1.0, depth=0.6):
    return [dict(pos=(x + depth / 2, (y0 + y1) / 2, height / 2), size=(depth, y1 - y0, height), pitch=0.0, label=2)]


# rough-terrain frame: rover at (0, 0) facing +x
SCENARIOS = {
    'flat_road': dict(
        desc='30 m of open flat road: accelerate smoothly to exactly 3.0 m/s, no wheelie, stop at B.',
        objects=[], spawn_z=0.0, goal=(30.0, 0.0), slope_deg=0.0, max_time=150.0,
        expect=dict(reach=True, top_speed=3.0)),
    'moderate_slope': dict(
        desc='15 deg ramp up to a 0.80 m plateau: drive up without stopping or lifting the front.',
        objects=ramp(3.0, 6.0, -4.0, 4.0, 15.0) + plateau(6.0, 13.0, -4.0, 4.0, 3.0 * math.tan(math.radians(15))),
        spawn_z=0.0, goal=(10.0, 0.0), slope_deg=15.0, max_time=150.0, expect=dict(reach=True)),
    'steep_hill': dict(
        desc='28 deg hill across the route (climbing limit 33 deg): climb it, slowly, and reach B.',
        objects=ramp(3.0, 6.0, -4.0, 4.0, 28.0) + plateau(6.0, 13.0, -4.0, 4.0, 3.0 * math.tan(math.radians(28))),
        spawn_z=0.0, goal=(10.0, 0.0), slope_deg=28.0, max_time=200.0, expect=dict(reach=True)),
    'obstacle': dict(
        desc='1.0 m high, 5 m wide wall across the straight line 6 m ahead: slow by clearance, steer '
             'round it without a standing stop, reach B behind it.',
        objects=wall(6.0, -2.5, 2.5), spawn_z=0.0, goal=(12.0, 0.0), slope_deg=0.0, max_time=200.0,
        obstacle_box=(6.0, 6.6, -2.5, 2.5), expect=dict(reach=True)),
    'cliff_front': dict(
        desc='1.5 m deep pit on the straight line (vigil_rough_terrain cliff_front): never fall in, '
             'go round it, keep going to B.',
        objects=plateau(-3.0, 13.0, -4.0, 4.0, CLIFF_H, holes=[(3.5, 6.5, -1.6, 1.6)]),
        spawn_z=CLIFF_H, goal=(10.0, 0.0), slope_deg=0.0, max_time=240.0,
        pit_box=(3.5, 6.5, -1.6, 1.6), expect=dict(reach=True, min_z=CLIFF_H - 0.3)),
}

# people (rough frame, like the objects): kind 'static' stands still, 'walker' walks between a and b
SCENARIOS.update({
    'human_block': dict(
        desc='Standing person 5 m straight ahead; the test drives INTO them at 0.8 m/s (no navigator): the '
             'rover must be stopped by the real contact and never pass through the person.',
        objects=[], spawn_z=0.0, goal=(12.0, 0.0), slope_deg=0.0, max_time=25.0, drive='probe',
        people=[dict(name='person_1', kind='static', at=(5.0, 0.0))], expect=dict(blocked=True)),
    'human_standing': dict(
        desc='Standing person on the A-B line 8 m ahead: detect, keep the human safety distance, go round, reach B.',
        objects=[], spawn_z=0.0, goal=(15.0, 0.0), slope_deg=0.0, max_time=150.0,
        people=[dict(name='person_1', kind='static', at=(8.0, 0.0))], expect=dict(reach=True, min_gap=1.0)),
    'human_crossing': dict(
        desc='Person walking back and forth across the A-B line (1.2 m/s, 7 m ahead): predict, yield or go '
             'round, never touch them, reach B.',
        objects=[], spawn_z=0.0, goal=(15.0, 0.0), slope_deg=0.0, max_time=180.0,
        people=[dict(name='walker_1', kind='walker', a=(7.0, 7.0), b=(7.0, -7.0), speed=1.2)],
        expect=dict(reach=True, min_gap=0.5)),
})
PERSON_RADIUS = 0.30          # m, for the probe's footprint-gap measurement (trunk + arms)


def person_models(people, mw=None):
    """Person models for a scenario, from the military_world export (StaticPerson_009, standing), with
    the SAR world's primitive collision body; walkers get the contact-aware sar::WaypointSystem."""
    if not people:
        return ''
    from sar_paths import find_military_world
    from build_sar_world import primitive_collisions, set_temperature
    mw = mw or find_military_world(os.environ.get('MILITARY_WORLD_DIR', ''))
    src = mw / 'gazebo_export/military_world.sdf'
    world = ET.parse(src).getroot().find('world')
    tpl = world.find("model[@name='SAR_StaticPerson_009']")
    out = []
    for person in people:
        m = copy.deepcopy(tpl)
        m.set('name', person['name'])
        for plug in m.findall('plugin'):
            m.remove(plug)
        for uri in m.iter('uri'):
            if uri.text and uri.text.startswith('meshes/'):
                uri.text = 'file://' + str((src.parent / uri.text).resolve())
        for v in m.iter('visual'):
            set_temperature(v, 305.15)
        link = m.find('link')
        primitive_collisions(link)
        # stand the person ON the ground: lowest point of the primitive body -> 5 mm above z = 0
        low = math.inf
        for col in link.findall('collision'):
            cx, cy, cz, r, pch, _ = [float(v) for v in col.findtext('pose').split()]
            g = col.find('geometry')[0]
            if g.tag == 'sphere':
                low = min(low, cz - float(g.findtext('radius')))
            else:
                L, R = float(g.findtext('length')), float(g.findtext('radius'))
                tilt = math.acos(max(-1.0, min(1.0, math.cos(r) * math.cos(pch))))
                low = min(low, cz - 0.5 * L * math.cos(tilt) - R * math.sin(tilt))
        base_z = 0.005 - low
        x, y = to_sar(*(person['at'] if person['kind'] == 'static' else person['a']))
        m.find('pose').text = f'{x:.3f} {y:.3f} {base_z:.3f} 0 0 0'
        lab = ET.SubElement(m, 'plugin', filename='gz-sim-label-system', name='gz::sim::systems::Label')
        ET.SubElement(lab, 'label').text = '10'
        odo = ET.SubElement(m, 'plugin', filename='gz-sim-odometry-publisher-system',
                            name='gz::sim::systems::OdometryPublisher')
        for k, v in (('odom_frame', 'world'), ('robot_base_frame', person['name']), ('dimensions', '3'),
                     ('odom_topic', f"/scenario/{person['name']}/odometry"), ('odom_publish_frequency', '30')):
            ET.SubElement(odo, k).text = v
        if person['kind'] == 'static':
            m.find('static').text = 'true'
        else:
            m.find('static').text = 'false'
            ET.SubElement(link, 'gravity').text = 'true'
            inertial = ET.SubElement(link, 'inertial')
            ET.SubElement(inertial, 'mass').text = '75'
            ET.SubElement(inertial, 'pose').text = '0 0 0.85 0 0 0'
            ine = ET.SubElement(inertial, 'inertia')
            for k, v in (('ixx', 20), ('iyy', 20), ('izz', 4), ('ixy', 0), ('ixz', 0), ('iyz', 0)):
                ET.SubElement(ine, k).text = str(v)
            for col in link.findall('collision'):
                surf = ET.SubElement(col, 'surface')
                fr = ET.SubElement(ET.SubElement(surf, 'friction'), 'ode')
                ET.SubElement(fr, 'mu').text = '0.08'
                ET.SubElement(fr, 'mu2').text = '0.08'
            ax, ay = to_sar(*person['a'])
            bx, by = to_sar(*person['b'])
            d = math.hypot(bx - ax, by - ay)
            leg = d / float(person['speed'])
            yaw = math.atan2(by - ay, bx - ax)
            plug = ET.SubElement(m, 'plugin', filename='sar-waypoint-system', name='sar::WaypointSystem')
            ET.SubElement(plug, 'offset').text = '0 0 0'
            for t, (px, py, pyaw) in ((0.0, (ax, ay, yaw)), (leg, (bx, by, yaw)), (leg + 1.0, (bx, by, yaw + math.pi)),
                                      (2 * leg + 1.0, (ax, ay, yaw + math.pi)), (2 * leg + 2.0, (ax, ay, yaw))):
                wp = ET.SubElement(plug, 'waypoint')
                ET.SubElement(wp, 'time').text = f'{t:.3f}'
                ET.SubElement(wp, 'pose').text = f'{px:.3f} {py:.3f} {base_z:.3f} 0 0 {pyaw:.4f}'
            for k, v in (('controller_version', '2'), ('mass', '75'), ('carried', 'false'),
                         ('cruise_speed', f"{float(person['speed']):g}")):
                ET.SubElement(plug, k).text = v
        out.append('    ' + ET.tostring(m, encoding='unicode') + '\n')
    return ''.join(out)


HEADER = """<?xml version="1.0"?>
<!-- GENERATED by vigil_sar scripts/scenario_worlds.py - scenario '{name}': {desc} -->
<sdf version="1.9">
  <world name="military_world">
    <gravity>0 0 -9.81</gravity>
    <physics name="default" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.70 0.80 0.90 1</background><shadows>false</shadows><grid>false</grid></scene>
    <light name="sun" type="directional"><pose>0 0 20 0 0 0</pose><cast_shadows>false</cast_shadows>
      <diffuse>0.85 0.85 0.85 1</diffuse><specular>0.15 0.15 0.15 1</specular><direction>-0.4 0.2 -0.9</direction></light>
    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.42 0.36 0.30 1</ambient><diffuse>0.42 0.36 0.30 1</diffuse></material></visual>
      </link>
      <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>1</label></plugin>
    </model>
"""

BOX = """    <model name="{name}"><static>true</static><pose>{x:.4f} {y:.4f} {z:.4f} 0 {pitch:.6f} {yaw:.6f}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>
      </link>
      <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{label}</label></plugin>
    </model>
"""


def box_to_sar(b):
    """Rough-frame box (x0, x1, y0, y1) -> SAR-frame (X0, X1, Y0, Y1)."""
    x0, x1, y0, y1 = b
    X = sorted([-y0 + A[0], -y1 + A[0]])
    Y = sorted([x0 + A[1], x1 + A[1]])
    return (X[0], X[1], Y[0], Y[1])


def world_sdf(name, sc):
    out = [HEADER.format(name=name, desc=sc['desc'])]
    for k, o in enumerate(sc['objects']):
        (x, y, z), (sx, sy, sz) = o['pos'], o['size']
        X, Y = to_sar(x, y)
        colour = '0.55 0.47 0.40' if o['label'] == 1 else '0.35 0.36 0.40'
        out.append(BOX.format(name=f'block_{k}', x=X, y=Y, z=z, sx=sx, sy=sy, sz=sz, pitch=o['pitch'],
                              yaw=math.pi / 2, c=colour, label=o['label']))
    out.append(person_models(sc.get('people')))
    gx, gy = to_sar(*sc['goal'])
    out.append(f"""    <model name="goal_B"><static>true</static><pose>{gx:.3f} {gy:.3f} {goal_height(sc) + 0.02:.3f} 0 0 0</pose>
      <link name="link"><visual name="v"><geometry><cylinder><radius>0.25</radius><length>0.02</length></cylinder></geometry>
        <material><ambient>1 0.2 0.8 1</ambient><diffuse>1 0.2 0.8 1</diffuse><emissive>0.6 0.1 0.5 1</emissive></material></visual></link>
    </model>
  </world>
</sdf>
""")
    return ''.join(out)


def goal_height(sc):
    gx, gy = sc['goal']
    z = 0.0
    for o in sc['objects']:
        (x, y, cz), (sx, sy, sz) = o['pos'], o['size']
        if abs(o['pitch']) < 1e-9 and o['label'] == 1 and abs(gx - x) <= sx / 2 and abs(gy - y) <= sy / 2:
            z = max(z, cz + sz / 2)
    return z


def describe(name, sc):
    gx, gy = to_sar(*sc['goal'])
    d = dict(name=name, desc=sc['desc'], spawn=[A[0], A[1], sc['spawn_z'] + 0.35], goal=[gx, gy],
             slope_deg=sc['slope_deg'], max_time=sc['max_time'], expect=sc['expect'],
             goal_z=goal_height(sc))
    for key in ('obstacle_box', 'pit_box'):
        if key in sc:
            d[key] = box_to_sar(sc[key])
    d['drive'] = sc.get('drive', 'navigator')
    d['people'] = []
    for person in sc.get('people', []):
        e = dict(name=person['name'], kind=person['kind'], radius=PERSON_RADIUS,
                 odometry=f"/scenario/{person['name']}/odometry")
        if person['kind'] == 'static':
            e['at'] = list(to_sar(*person['at']))
        d['people'].append(e)
    return d


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--list':
        print(' '.join(SCENARIOS))
        return
    out = Path(sys.argv[1] if len(sys.argv) > 1 else '.')
    out.mkdir(parents=True, exist_ok=True)
    meta = {}
    for name, sc in SCENARIOS.items():
        (out / f'{name}.sdf').write_text(world_sdf(name, sc))
        meta[name] = describe(name, sc)
    (out / 'scenarios.json').write_text(json.dumps(meta, indent=1))
    print(f'wrote {len(meta)} scenario worlds to {out}')


if __name__ == '__main__':
    main()
