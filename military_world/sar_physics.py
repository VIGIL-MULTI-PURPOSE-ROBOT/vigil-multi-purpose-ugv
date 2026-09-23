"""Configure the walking people in an exported SDF, without re-exporting meshes.

Two controllers are available, chosen with --controller:

  1  kinematic     the person is placed on its route pose every step. Always
                   works, never falls over, but walks through walls and
                   through the robot. This is the default.
  2  contact-aware forces drive a dynamic body, so the person collides with
                   terrain, props, walls and the robot and stops when
                   blocked. Needs a physics build that is happy with the
                   inertials and friction written below.

Usage:  python3 sar_physics.py [--controller 1|2] WORLD.sdf [WORLD.sdf ...]
"""
from pathlib import Path
import math
import sys
import xml.etree.ElementTree as ET

PLUGIN = "sar::WaypointSystem"
DEFAULT_CONTROLLER = 2


def value(parent, tag, text):
    node = parent.find(tag)
    if node is None:
        node = ET.SubElement(parent, tag)
    node.text = str(text)
    return node


def configure_world(root, controller=DEFAULT_CONTROLLER):
    world = root.find('world')
    if world is None:
        raise ValueError('SDF has no <world> element')
    if controller not in (1, 2):
        raise ValueError(f'controller must be 1 or 2, got {controller!r}')
    physics = world.find('physics')
    if physics is None:
        physics = ET.SubElement(world, 'physics', name='default', type='dart')
        value(physics, 'real_time_factor', '1.0')
    value(physics, 'max_step_size', '0.01')
    # Only the contact controller needs the sturdier collision detector and
    # solver. Leaving the stock DART configuration alone for the kinematic
    # controller keeps that path on the settings Gazebo ships and tests.
    dart = physics.find('dart')
    if controller == 2:
        if dart is None:
            dart = ET.SubElement(physics, 'dart')
        value(dart, 'collision_detector', 'bullet')
        solver = dart.find('solver')
        if solver is None:
            solver = ET.SubElement(dart, 'solver')
        value(solver, 'solver_type', 'pgs')
    elif dart is not None:
        physics.remove(dart)

    movers = []
    for model in world.findall('model'):
        plugin = model.find(f"plugin[@name='{PLUGIN}']")
        if plugin is None:
            continue
        carried = model.get('name', '').startswith('SAR_CarriedStretcher')
        offset = list(map(float, plugin.findtext('offset', '0 0 0').split()))
        in_formation = carried or math.hypot(*offset[:2]) > 0.01
        mass = 15.0 if carried else 75.0
        link = model.find('link')
        if link is None:
            continue  # nothing to drive; leave the model exactly as exported
        if (controller == 2 and not carried
                and model.findtext('static') == 'true'
                and model.findtext('pose')):
            pose = list(map(float, model.findtext('pose').split()))
            pose[2] += 0.04  # Initial clearance; gravity puts the feet on terrain.
            value(model, 'pose', ' '.join(map(str, pose)))
        value(model, 'static', 'false')
        value(model, 'self_collide', 'false')
        # The kinematic controller writes the pose every step, so gravity would
        # only fight it. Contact people stand on the terrain under gravity --
        # except the stretcher, which is held up by its bearers.
        falls = controller == 2 and not carried
        value(link, 'gravity', 'true' if falls else 'false')
        old = link.find('inertial')
        if old is not None:
            link.remove(old)
        inertial = ET.SubElement(link, 'inertial')
        value(inertial, 'mass', mass)
        value(inertial, 'pose', '0 0 0 0 0 0' if carried else '0 0 0.85 0 0 0')
        inertia = ET.SubElement(inertial, 'inertia')
        for tag, val in [('ixx', 5 if carried else 20), ('iyy', 1 if carried else 20),
                         ('izz', 6 if carried else 4), ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
            value(inertia, tag, val)
        for collision in link.findall('collision'):
            surface = collision.find('surface')
            if surface is None:
                surface = ET.SubElement(collision, 'surface')
            friction = surface.find('friction')
            if friction is None:
                friction = ET.SubElement(surface, 'friction')
            ode = friction.find('ode')
            if ode is None:
                ode = ET.SubElement(friction, 'ode')
            value(ode, 'mu', 0.08)
            value(ode, 'mu2', 0.08)
        value(plugin, 'controller_version', controller)
        value(plugin, 'mass', mass)
        value(plugin, 'carried', str(carried).lower())
        # Fixed realistic cruise targets; improve wall-clock pace via physics
        # performance, instead of arbitrarily multiplying pedestrian speed.
        value(plugin, 'cruise_speed', 1.8 if in_formation else 2.2)
        old = plugin.find('speed_multiplier')
        if old is not None:
            plugin.remove(old)
        for member in plugin.findall('member'):
            plugin.remove(member)
        movers.append((model, plugin, in_formation))

    # Members sharing a centreline wait together if any bearer is blocked.
    # Only the contact controller can be blocked, so only it needs formations.
    if controller == 2:
        groups = {}
        for model, plugin, grouped in movers:
            if grouped:
                signature = tuple((w.findtext('time'), w.findtext('pose'))
                                  for w in plugin.findall('waypoint'))
                groups.setdefault(signature, []).append((model, plugin))
        for group in groups.values():
            for owner, plugin in group:
                for model, peer in group:
                    if model is owner:
                        continue  # a person does not wait on themselves
                    member = ET.SubElement(plugin, 'member')
                    value(member, 'name', model.get('name'))
                    value(member, 'offset', peer.findtext('offset', '0 0 0'))
    return root


def update_file(path, controller=DEFAULT_CONTROLLER):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(path, parser=parser)
    configure_world(tree.getroot(), controller)
    tree.write(path, encoding='utf-8', xml_declaration=True)


def main(argv):
    controller = DEFAULT_CONTROLLER
    paths = []
    it = iter(argv)
    for arg in it:
        if arg == '--controller':
            controller = int(next(it, DEFAULT_CONTROLLER))
        elif arg.startswith('--controller='):
            controller = int(arg.split('=', 1)[1])
        else:
            paths.append(arg)
    if not paths:
        print(__doc__.strip())
        return 2
    label = 'contact-aware' if controller == 2 else 'kinematic'
    for arg in paths:
        update_file(Path(arg), controller)
        print(f'Configured {label} people (controller {controller}): {arg}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
