#!/usr/bin/python3
"""Scale check: is military_world at metre scale, and is the rover in proportion to it?

    ros2 run vigil_sar scale_check.py            # or: python3 scripts/scale_check.py
    python3 scripts/scale_check.py --json out.json

Measures, from the files Gazebo actually loads (1 Gazebo unit = 1 m):
  rover     URDF via xacro (scripts/rover_stability.py): length, width, height, wheel diameters,
            wheelbase, track, mass
  world     gazebo_export/military_world.sdf + its .glb meshes (read directly, no extra library):
            people, cars, containers, walls, doors, desks, roads, streets, tracks, terrain, extent
  config    config/sar_mission.yaml / physics.yaml: robot dimensions, operational zone, ranges
and compares each against real-world references and against the rover. Exit code 1 if a check
fails. Nothing is modified.
"""
import argparse
import json
import math
import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sar_paths import find_military_world, load_config  # noqa: E402


# ------------------------------------------------------------------ glb (glTF binary) reader
def glb_triangles(path):
    """(vertices Nx3, faces Mx3) of every triangle primitive in a .glb (node transforms: none in
    this export; a translation is applied if present)."""
    b = Path(path).read_bytes()
    _, _, _ = struct.unpack('<4sII', b[:12])
    jlen, _ = struct.unpack('<I4s', b[12:20])
    js = json.loads(b[20:20 + jlen])
    off = 20 + jlen
    blen, _ = struct.unpack('<I4s', b[off:off + 8])
    binary = b[off + 8:off + 8 + blen]
    types = {5126: ('<f4', 4), 5123: ('<u2', 2), 5125: ('<u4', 4), 5121: ('<u1', 1)}
    ncomp = {'SCALAR': 1, 'VEC3': 3, 'VEC2': 2, 'VEC4': 4}

    def acc(i):
        a = js['accessors'][i]
        bv = js['bufferViews'][a['bufferView']]
        dt, size = types[a['componentType']]
        n = ncomp[a['type']]
        start = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        stride = bv.get('byteStride', size * n)
        raw = np.frombuffer(binary, np.uint8, count=stride * (a['count'] - 1) + size * n, offset=start)
        rows = np.lib.stride_tricks.as_strided(raw, (a['count'], size * n), (stride, 1))
        return np.frombuffer(rows.copy().tobytes(), dt).reshape(a['count'], n)
    V, F = [], []
    shift = np.zeros(3)
    for node in js.get('nodes', []):
        if 'translation' in node:
            shift = np.array(node['translation'], float)
    for mesh in js['meshes']:
        for prim in mesh['primitives']:
            if prim.get('mode', 4) != 4:
                continue
            v = acc(prim['attributes']['POSITION']).astype(float) + shift
            f = acc(prim['indices']).reshape(-1, 3) if 'indices' in prim else np.arange(len(v)).reshape(-1, 3)
            F.append(f + sum(len(x) for x in V))
            V.append(v)
    return np.vstack(V), np.vstack(F)


def strip_width(V, F, cell=0.1):
    """Median width of a road/track ribbon: rasterise the upward faces, 2 x the distance-transform
    value along the centre line (local maxima)."""
    import cv2
    n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    up = F[n[:, 2] > 1e-9]
    lo = V[:, :2].min(0) - 1
    hi = V[:, :2].max(0) + 1
    W, H = (np.ceil((hi - lo) / cell)).astype(int) + 1
    img = np.zeros((H, W), np.uint8)
    pts = ((V[:, :2] - lo) / cell).round().astype(np.int32)
    for tri in up:
        cv2.fillConvexPoly(img, pts[tri], 1)
    d = cv2.distanceTransform(img, cv2.DIST_L2, 5)
    ridge = (d >= cv2.dilate(d, np.ones((3, 3), np.uint8))) & (d > 1)
    return float(2 * np.median(d[ridge]) * cell) if ridge.any() else 0.0


def gaps_along(V, F, axis, z):
    """Openings (m) in a wall mesh along `axis` at height z above its base (door / window test)."""
    base = V[:, 2].min()
    zc = base + z
    xs = []
    for tri in F:
        p = V[tri]
        if p[:, 2].min() <= zc <= p[:, 2].max():
            xs.append((p[:, axis].min(), p[:, axis].max()))
    if not xs:
        return []
    xs.sort()
    out, cur = [], list(xs[0])
    for a, b in xs[1:]:
        if a > cur[1] + 0.3:
            out.append((round(cur[1], 2), round(a, 2), round(a - cur[1], 2)))
            cur = [a, b]
        else:
            cur[1] = max(cur[1], b)
    return out


# ------------------------------------------------------------------ measurements
def semantic(model):
    for c in model:
        if c.tag is ET.Comment and c.text and c.text.strip().startswith('{'):
            try:
                return json.loads(c.text.strip())
            except ValueError:
                return {}
    return {}


def measure_world(mw):
    sdf = mw / 'gazebo_export/military_world.sdf'
    meshes = mw / 'gazebo_export/meshes'
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    world = ET.parse(sdf, parser=parser).getroot().find('world')
    out = dict(boxes={}, roads={}, terrain=None, humans=[], doors={})
    for m in world.findall('model'):
        name = m.get('name')
        sem = semantic(m)
        size = m.find('.//collision/geometry/box/size')
        if size is not None:
            out['boxes'][name] = dict(size=[float(v) for v in size.text.split()], cls=sem.get('sar_object'))
    mesh = lambda n: meshes / f'SARM_{n}.glb'  # noqa: E731
    # people: the standing template, all its parts
    parts = sorted(meshes.glob('SARM_SAR_StaticPerson_009_*.glb'))
    if parts:
        z = np.concatenate([glb_triangles(p)[0][:, 2] for p in parts])
        out['person_height'] = float(z.max())      # model origin = feet; head top is the height
    for n in ('SAR_Road_MainSupply', 'SAR_Road_East', 'SAR_Road_West', 'SAR_Street_Urban_A', 'SAR_Street_Urban_B',
              'SAR_Street_Urban_C', 'SAR_Street_Urban_D', 'SAR_Track_Desert', 'SAR_Track_Forest',
              'SAR_Track_RubbleBypass', 'SAR_Track_Mountain_Safe', 'SAR_Bridge_001_Deck'):
        if mesh(n).exists():
            V, F = glb_triangles(mesh(n))
            out['roads'][n] = round(strip_width(V, F), 2)
    for n, axis in (('SAR_Building_URB_001_Wall_S_00', 0), ('SAR_Building_URB_004_Wall_S_00', 0)):
        if mesh(n).exists():
            V, F = glb_triangles(mesh(n))
            low = gaps_along(V, F, axis, 0.3)                       # a door reaches the floor
            out['doors'][n] = max((g[2] for g in low), default=0.0)
    tiles = sorted(meshes.glob('SARM_SAR_Terrain_*.glb'))
    if tiles:
        xs, ys, zs = [], [], []
        for m in world.findall('model'):
            if m.get('name', '').startswith('SAR_Terrain_'):
                x, y = [float(v) for v in m.findtext('pose').split()[:2]]
                xs += [x - 25, x + 25]
                ys += [y - 25, y + 25]
        for t in tiles:
            zs.append(glb_triangles(t)[0][:, 2])
        z = np.concatenate(zs)
        out['terrain'] = dict(x=[min(xs), max(xs)], y=[min(ys), max(ys)], z=[float(z.min()), float(z.max())])
    return out


def checks(rover, world, cfg, phys):
    """[(name, measured, reference / rule, ok)]"""
    rows = []

    def add(name, value, rule, ok):
        rows.append((name, value, rule, bool(ok)))
    L, Wd = rover['length'], rover['width']
    add('rover length x width x height', f"{L:.2f} x {Wd:.2f} x {rover['height']:.2f} m",
        'UGV class of Clearpath Warthog 1.52 x 1.38 m', 1.0 < L < 2.5 and 0.8 < Wd < 1.6)
    add('rover wheels', f"dia {rover['d_front']:.2f} / {rover['d_small']:.2f} m, wheelbase {rover['wheelbase']:.2f} m, "
        f"track {rover['track']:.2f} m, {rover['mass']:.1f} kg", 'wheel dia 0.3-0.6 m, 150-400 kg',
        0.3 < rover['d_front'] < 0.6 and 150 < rover['mass'] < 400)
    r = cfg['robot']
    add('config robot dims = URDF', f"config {r['length']} x {r['width']} m, wheel r {r['wheel_radius_front']} / "
        f"{r['wheel_radius_small']}", 'within 2 cm of the URDF',
        abs(r['length'] - L) < 0.02 and abs(r['width'] - Wd) < 0.02
        and abs(r['wheel_radius_front'] - rover['d_front'] / 2) < 0.02
        and abs(r['wheel_radius_small'] - rover['d_small'] / 2) < 0.02)
    pr = phys.get('rover', {})
    if 'wheel_radius_front' in pr:
        add('physics.yaml wheel radii = URDF', f"{pr['wheel_radius_front']} / {pr['wheel_radius_small']} m",
            'within 1 mm', abs(pr['wheel_radius_front'] - rover['d_front'] / 2) < 1e-3
            and abs(pr['wheel_radius_small'] - rover['d_small'] / 2) < 1e-3)
    ph = world.get('person_height')
    if ph:
        add('standing person', f'{ph:.2f} m', 'adult 1.5-1.95 m -> metre scale', 1.5 < ph < 1.95)
    b = world['boxes']
    car = b.get('SAR_Vehicle_Civilian_004_Body', {}).get('size')
    if car:
        add('civilian car', f'{car[0]:.2f} x {car[1]:.2f} m ({car[0] / L:.1f} rover lengths)',
            'car 3.5-5.5 x 1.6-2.0 m', 3.5 < car[0] < 5.5 and 1.6 < car[1] < 2.0)
    cont = b.get('SAR_Base_Container_01', {}).get('size')
    if cont:
        add('shipping container', 'x'.join(f'{v:.2f}' for v in cont) + ' m', 'ISO 20 ft 6.06 x 2.44 x 2.59',
            abs(cont[0] - 6.06) < 0.1 and abs(cont[1] - 2.44) < 0.1)
    wall = b.get('SAR_Building_URB_001_Wall_S_00', {}).get('size')
    if wall:
        add('building wall height', f'{wall[2]:.2f} m', '1 storey 2.5-4 m', 2.5 < wall[2] < 4.0)
    desk = b.get('SAR_Building_URB_001_Furniture_DESK_02', {}).get('size')
    if desk:
        add('desk', 'x'.join(f'{v:.2f}' for v in desk) + ' m', 'desk top 0.70-0.78 m', 0.70 < desk[2] < 0.78)
    for n, wd in world['doors'].items():
        add(f'widest ground-level opening {n[13:20]}', f'{wd:.2f} m (rover {Wd:.2f} m)',
            'door 0.8-1.2 m / vehicle door 3-6 m', 0.8 < wd < 6.0)
    for n, wd in world['roads'].items():
        kind = 'bridge' if 'Bridge' in n else 'track' if 'Track' in n else 'street' if 'Street' in n else 'road'
        lo = {'road': 5.0, 'street': 3.5, 'track': 3.0, 'bridge': 3.0}[kind]
        add(f'{kind} {n[4:]}', f'{wd:.2f} m = {wd / Wd:.1f} x rover width', f'>= {lo} m and >= 2.5 x rover',
            wd >= lo and wd >= 2.5 * Wd)
    t = world.get('terrain')
    if t:
        add('whole world', f"{t['x'][1] - t['x'][0]:.0f} x {t['y'][1] - t['y'][0]:.0f} m, height "
            f"{t['z'][0]:.1f} .. {t['z'][1]:.1f} m ({(t['x'][1] - t['x'][0]) / L:.0f} rover lengths)",
            'informational', True)
    w = cfg['world']
    z = [w.get(k) for k in ('zone_x_min', 'zone_x_max', 'zone_y_min', 'zone_y_max')]
    if None not in z:
        zw, zh = z[1] - z[0], z[3] - z[2]
        sp = cfg['spawn']
        g = cfg['navigation']
        inside = lambda x, y: z[0] <= x <= z[1] and z[2] <= y <= z[3]  # noqa: E731
        add('operational zone', f'x [{z[0]:g},{z[1]:g}] y [{z[2]:g},{z[3]:g}] = {zw:g} x {zh:g} m',
            '100-200 m per side, contains A and the default B', 100 <= zw <= 200 and 100 <= zh <= 200
            and inside(sp.get('x', 0), sp.get('y', -8)) and inside(g['goal_x'], g['goal_y']))
    # sensor ranges against the rover's own stopping distance
    d = phys['drive']
    v = float(d['max_linear'])
    stop = v * v / (2 * d['max_decel']) + v * d['max_decel'] / d['max_jerk_brake'] / 2
    latency = 1.0 / float(cfg['terrain'].get('grid_publish_rate', 1.0)) + 0.2
    need = stop + v * latency + 0.5
    add('cliff detection range', f"{cfg['terrain']['cliff_detection_distance']:.1f} m",
        f'>= stopping {stop:.1f} m + map latency {v * latency:.1f} m + 0.5 m = {need:.1f} m at {v:g} m/s',
        cfg['terrain']['cliff_detection_distance'] >= need)
    add('depth camera range', f"{cfg['camera']['depth_far']:.0f} m", f'>= 3 x stopping distance ({3 * stop:.1f} m)',
        cfg['camera']['depth_far'] >= 3 * stop)
    tc = cfg['thermal_camera']
    rng = float(tc['detection_range'])
    vfov = 2 * math.atan(math.tan(tc['horizontal_fov'] / 2) * tc['resolution_height'] / tc['resolution_width'])
    px_tall = tc['resolution_height'] * (2 * math.atan((ph or 1.74) / 2 / rng)) / vfov
    px_lying = tc['resolution_height'] * (2 * math.atan(0.35 / 2 / rng)) / vfov
    add('thermal detection range', f'{rng:.0f} m: standing person {px_tall:.0f} px tall, lying {px_lying:.1f} px',
        'standing >= 12 px', px_tall >= 12)
    return rows


def rover_dims(height=None):
    """Length / width from the outermost tyre surfaces, wheel sizes, wheelbase (front to rearmost
    LOADED axle), track (narrowest wheel pair, centre to centre), mass - all from the URDF."""
    import rover_stability as rs
    root = rs.build_urdf()
    r = rs.analyse(root)
    links = {l.get('name'): l for l in root.findall('link')}
    parent = {j.find('child').get('link'): j for j in root.findall('joint')}

    def pose(name):
        if name not in parent:
            return np.eye(4)
        j = parent[name]
        return pose(j.find('parent').get('link')) @ rs._tf(j.find('origin'))
    ys = []
    for n, l in links.items():
        if n.endswith('_wheel'):
            ys.append((float(pose(n)[1, 3]), float(l.find('collision/geometry/cylinder').get('length'))))
    wheels = r['wheels']
    loaded = [w for w in wheels if w['loaded']]
    front = max(loaded, key=lambda w: w['x'])
    rear = min(loaded, key=lambda w: w['x'])
    return dict(length=max(w['x'] + w['r'] for w in wheels) - min(w['x'] - w['r'] for w in wheels),
                width=2 * max(abs(y) + wd / 2 for y, wd in ys), height=height or 0.0,
                d_front=2 * front['r'], d_small=2 * min(w['r'] for w in wheels),
                wheelbase=front['x'] - rear['x'], track=2 * min(abs(y) for y, _ in ys), mass=r['mass'])


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default='')
    ap.add_argument('--physics', default='')
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('vigil_sar'))
    except Exception:
        share = here.parent
    cfg = load_config(a.config or share / 'config/sar_mission.yaml')
    import yaml
    phys = yaml.safe_load(open(a.physics or share / 'config/physics.yaml'))['/**']['ros__parameters']['physics']
    mw = find_military_world(cfg['world'].get('military_world_dir', ''))
    rover = rover_dims(cfg['robot'].get('height'))
    world = measure_world(mw)
    rows = checks(rover, world, cfg, phys)
    print('SCALE CHECK (1 Gazebo unit = 1 m)')
    for name, value, rule, ok in rows:
        print(f"{'PASS' if ok else 'FAIL'}  {name:38s} {value}   [{rule}]")
    n = sum(ok for *_, ok in rows)
    print(f'\n{n}/{len(rows)} checks passed')
    if a.json:
        Path(a.json).write_text(json.dumps(dict(rover=rover, world=world, rows=rows), indent=1, default=str))
    sys.exit(0 if n == len(rows) else 1)


if __name__ == '__main__':
    main()
