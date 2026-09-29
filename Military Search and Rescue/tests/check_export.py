"""Check the generated world, including the GLB parent-transform regression."""
import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET


def rotate(point, angles):
    x, y, z = point
    roll, pitch, yaw = angles
    y, z = math.cos(roll) * y - math.sin(roll) * z, math.sin(roll) * y + math.cos(roll) * z
    x, z = math.cos(pitch) * x + math.sin(pitch) * z, -math.sin(pitch) * x + math.cos(pitch) * z
    return (math.cos(yaw) * x - math.sin(yaw) * y,
            math.sin(yaw) * x + math.cos(yaw) * y, z)


def check(directory):
    world = ET.parse(directory / 'military_world.sdf').getroot().find('world')
    models = world.findall('model')
    assert len({m.get('name') for m in models}) == len(models), 'Duplicate models'
    assert not world.findall('actor'), 'Unrigged actors must not be used'
    for uri in world.findall('.//mesh/uri'):
        assert (directory / uri.text).is_file(), f'Missing mesh: {uri.text}'
    movers = [m for m in models if m.find("plugin[@name='sar::WaypointSystem']") is not None]
    assert len(movers) == 20, f'Expected 20 movers, got {len(movers)}'
    checked = 0
    for model in movers:
        assert model.findtext('static') == 'false', 'Movers must participate in physics'
        assert model.find('.//inertial/mass') is not None
        assert model.findall('.//collision'), model.get('name')
        waypoints = model.findall("plugin[@name='sar::WaypointSystem']/waypoint")
        times = [float(w.findtext('time')) for w in waypoints]
        assert times[0] == 0 and all(a < b for a, b in zip(times, times[1:]))
        assert waypoints[0].findtext('pose') == waypoints[-1].findtext('pose')
        for visual in model.findall('.//visual'):
            uri = visual.findtext('.//mesh/uri')
            if not uri.endswith('.glb'):
                continue
            data = (directory / uri).read_bytes()
            length, kind = struct.unpack_from('<II', data, 12)
            assert kind == 0x4E4F534A
            gltf = json.loads(data[20:20 + length])
            for node in gltf.get('nodes', []):
                for key, default in [('translation', [0, 0, 0]),
                                     ('rotation', [0, 0, 0, 1]),
                                     ('scale', [1, 1, 1]),
                                     ('matrix', [1, 0, 0, 0, 0, 1, 0, 0,
                                                 0, 0, 1, 0, 0, 0, 0, 1])]:
                    assert all(abs(a - b) < 1e-5 for a, b in zip(node.get(key, default), default)), (uri, key)
            # Verify that the primitive collision follows the mesh's center,
            # including parent scale, instead of being stacked at the feet.
            positions = [gltf['accessors'][p['attributes']['POSITION']]
                         for mesh in gltf['meshes'] for p in mesh['primitives']]
            lo = [min(p['min'][i] for p in positions) for i in range(3)]
            hi = [max(p['max'][i] for p in positions) for i in range(3)]
            scale = list(map(float, visual.findtext('.//scale', '1 1 1').split()))
            pose = list(map(float, visual.findtext('pose', '0 0 0 0 0 0').split()))
            center = rotate([(a + b) / 2 * s for a, b, s in zip(lo, hi, scale)], pose[3:])
            expected = [a + b for a, b in zip(center, pose[:3])]
            cname = 'c' + visual.get('name')[1:]
            collision = model.find(f".//collision[@name='{cname}']")
            assert collision is not None
            actual = list(map(float, collision.findtext('pose', '0 0 0 0 0 0').split()))
            assert max(abs(a - b) for a, b in zip(expected, actual[:3])) < 0.003, (uri, expected, actual)
            checked += 1
    print(f'PASS: 20 movers; {checked} meshes have local transforms and aligned collisions; all assets exist.')


if __name__ == '__main__':
    check(Path(sys.argv[1] if len(sys.argv) > 1 else 'gazebo_export'))
