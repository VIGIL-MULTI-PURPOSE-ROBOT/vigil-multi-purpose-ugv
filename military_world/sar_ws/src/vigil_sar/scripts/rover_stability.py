#!/usr/bin/python3
"""Mass, centre of mass, gravity flags and wheelie limits of the rover exactly as Gazebo gets it.

    ros2 run vigil_sar rover_stability.py            # prints the report
    python3 scripts/rover_stability.py               # same, from the source tree

It runs xacro on urdf/agri_ugv.urdf.xacro with the simulation arguments, walks the kinematic tree at
the spawn configuration (every joint at 0 = the CAD pose) and reports:
  * total mass, centre of mass (base_footprint frame), the heaviest links
  * links whose gravity is switched off (URDF <gazebo><gravity>/<turnGravityOff>, SDF <gravity>0) -
    there must be none: gravity has to act on every link that has mass
  * each wheel's contact height, i.e. which wheels carry the rover
  * the longitudinal acceleration above which the front lifts (a wheelie), on flat ground and on
    slopes, and the braking deceleration above which it noses over

Wheelie limit: taking moments about the rearmost loaded tyre contact, the front unloads when
    m a h + sum(I_wheel / r_wheel) a  >  m g (x_com - x_rear) cos(slope) - m g h sin(slope)
(h = height of the centre of mass above the contact line; the wheel term is the reaction of spinning
the wheels up). This is the rigid-body limit; the springs make the real limit a little lower, which
is why drive.max_accel keeps a large margin under it.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def package_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('vigil_sar'))
    except Exception:
        return Path(__file__).resolve().parents[1]


def build_urdf(pkg=None, mappings=None):
    import xacro
    from xacro import substitution_args
    pkg = Path(pkg or package_dir())
    orig = substitution_args._find

    def _find(resolved, a, args, context):
        if args and args[0] == 'vigil_sar':
            return str(pkg)
        return orig(resolved, a, args, context)
    substitution_args._find = _find
    try:
        m = {'simulation': 'true', 'vision_sensors': 'true', 'thermal_camera': 'true'}
        m.update(mappings or {})
        return ET.fromstring(xacro.process_file(str(pkg / 'urdf/agri_ugv.urdf.xacro'), mappings=m).toxml())
    finally:
        substitution_args._find = orig


def _rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _tf(origin):
    g = lambda k: [float(v) for v in ((origin.get(k) if origin is not None else None) or '0 0 0').split()]  # noqa: E731
    M = np.eye(4)
    M[:3, :3] = _rot(*g('rpy'))
    M[:3, 3] = g('xyz')
    return M


def analyse(root, slopes=(0.0, 15.0, 25.0, 28.0, 33.0), g=9.81):
    links = {l.get('name'): l for l in root.findall('link')}
    parent = {j.find('child').get('link'): j for j in root.findall('joint')}

    def pose(name):
        if name not in parent:
            return np.eye(4)
        j = parent[name]
        return pose(j.find('parent').get('link')) @ _tf(j.find('origin'))

    masses = []
    for n, l in links.items():
        i = l.find('inertial')
        if i is not None:
            masses.append((n, float(i.find('mass').get('value')), (pose(n) @ _tf(i.find('origin')))[:3, 3]))
    M = sum(m for _, m, _ in masses)
    com = sum(m * p for _, m, p in masses) / M
    no_gravity = sorted({g_.get('reference') or '(robot)' for g_ in root.iter('gazebo')
                         if g_.find('turnGravityOff') is not None
                         or (g_.find('gravity') is not None and (g_.findtext('gravity') or '').strip() in ('0', 'false'))})
    wheels = []
    for n, l in links.items():
        if n.endswith('_wheel'):
            p = pose(n)[:3, 3]
            cyl = l.find('collision/geometry/cylinder')
            r = float(cyl.get('radius'))
            m = float(l.find('inertial/mass').get('value'))
            wheels.append(dict(name=n[:-6], x=float(p[0]), z_bottom=float(p[2] - r), r=r, mass=m))
    ground = min(w['z_bottom'] for w in wheels)
    for w in wheels:
        w['loaded'] = w['z_bottom'] - ground < 0.10          # lifted wheels sit ~0.3 m up
    loaded = [w for w in wheels if w['loaded']]
    x_rear = min(w['x'] for w in loaded)
    x_front = max(w['x'] for w in loaded)
    h = float(com[2] - np.mean([w['z_bottom'] for w in loaded]))
    spin = sum(0.5 * w['mass'] * w['r'] ** 2 / w['r'] for w in wheels)   # sum I/r, kg.m
    lever_back = float(com[0] - x_rear)
    lever_front = float(x_front - com[0])
    wheelie = {}
    for s in slopes:
        t = math.radians(s)
        wheelie[s] = max(0.0, g * M * (lever_back * math.cos(t) - h * math.sin(t)) / (M * h + spin))
    return dict(mass=M, com=com, masses=sorted(masses, key=lambda r: -r[1]), links_with_mass=len(masses),
                no_gravity=no_gravity, wheels=wheels, com_height=h, x_rear=x_rear, x_front=x_front,
                lever_back=lever_back, lever_front=lever_front, wheelie=wheelie,
                nose_over=g * M * lever_front / (M * h + spin),
                tip_back_deg=math.degrees(math.atan2(lever_back, h)),
                tip_side_deg=math.degrees(math.atan2(min(abs(float(pose(n)[1, 3])) for n in links
                                                         if n.endswith('_wheel')), h)))


def report(r, drive=None):
    c = r['com']
    out = [f"mass {r['mass']:.1f} kg in {r['links_with_mass']} links; centre of mass x {c[0]:+.3f} y {c[1]:+.3f} m, "
           f"{r['com_height']:.3f} m above the tyre contacts",
           'heaviest links: ' + ', '.join(f'{n} {m:.1f} kg' for n, m, _ in r['masses'][:6]),
           'gravity switched off on: ' + (', '.join(r['no_gravity']) if r['no_gravity'] else 'NO link (gravity acts on all)'),
           'wheels: ' + ', '.join(f"{w['name']} {'loaded' if w['loaded'] else 'LIFTED'} (x {w['x']:+.2f})"
                                  for w in sorted(r['wheels'], key=lambda w: w['name'])),
           f"rearmost loaded contact x {r['x_rear']:+.3f} (COM {r['lever_back']:.3f} m ahead of it), "
           f"front contact x {r['x_front']:+.3f}",
           f"static tip-back angle {r['tip_back_deg']:.1f} deg, braking nose-over above {r['nose_over']:.1f} m/s2"]
    for s, a in r['wheelie'].items():
        line = f'  wheelie (front lifts) on a {s:4.1f} deg climb above {a:5.2f} m/s2'
        if drive:
            line += f"   drive.max_accel {drive['max_accel']:.2f} -> margin x{a / drive['max_accel']:.1f}"
        out.append(line)
    return '\n'.join(out)


def main():
    pkg = package_dir()
    drive = None
    try:
        import yaml
        d = yaml.safe_load((pkg / 'config/physics.yaml').read_text())
        drive = d['/**']['ros__parameters']['physics']['drive']
    except Exception:
        pass
    r = analyse(build_urdf(pkg))
    print(report(r, drive))
    ok = not r['no_gravity'] and (drive is None or drive['max_accel'] < min(r['wheelie'].values()))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
