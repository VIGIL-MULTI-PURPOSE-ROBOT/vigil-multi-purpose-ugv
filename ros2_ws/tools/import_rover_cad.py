#!/usr/bin/python3
"""Split the reviewed 2026-09-20 STL into articulated visual meshes in metres.

Component IDs are bound to a source hash: never apply this map to another export.
The STL supplies geometry only. Masses and joint dynamics remain assumptions.
"""
import argparse
import hashlib
import json
import struct
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

SOURCE_SHA256 = '660776500e243604f17f388921b051b83f0a0db94dda5d01ccfc34054a9ba37e'
DTYPE = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
# CAD +X maps to ROS +Y, CAD -Y to ROS +X; dimensions are millimetres.
ROTATION = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
OFFSET = np.array([-.5697, .511263443, 0.])
WHEELS = {
    'R1': ([0], [1, 2], 12), 'R2': ([3], [6, 9], 13),
    'R3': ([4], [7, 10], 14), 'R4': ([5], [8, 11], 15),
    'L1': ([84], [83, 88], 68), 'L2': ([80], [77, 85], 69),
    'L3': ([81], [78, 86], 70), 'L4': ([82], [79, 87], 71),
}
ROCKERS = {'R': [12, 13, 14, 15, 16, 18, 20, 21, 23, 24],
           'L': [67, 68, 69, 70, 71, 72, 73, 74, 75, 76]}


def split(data):
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError('Expected binary STL')
    triangles = np.frombuffer(data, dtype=DTYPE, offset=84)['v'].astype(float)
    vertices, inverse = np.unique(np.round(triangles.reshape(-1, 3), 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(len(vertices), len(vertices))).tocsr()
    _, labels = connected_components(graph, directed=False)
    return triangles, labels[faces[:, 0]]


def write_stl(path, triangles):
    out = np.zeros(len(triangles), dtype=DTYPE)
    out['v'] = triangles
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    norms = np.linalg.norm(normals, axis=1)
    out['normal'] = normals / np.maximum(norms[:, None], 1e-15)
    path.write_bytes(b'CAD-derived visual mesh, metres'.ljust(80, b'\0') + struct.pack('<I', len(out)) + out.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stl', type=Path)
    args = parser.parse_args()
    data = args.stl.read_bytes()
    if hashlib.sha256(data).hexdigest() != SOURCE_SHA256:
        raise ValueError('Unreviewed STL revision: component mapping must be checked before import')
    ws = Path(__file__).resolve().parents[1]
    package = ws / 'src/agri_ugv_description'
    raw, labels = split(data)
    triangles = raw @ ROTATION.T * .001 + OFFSET
    records = []
    used = set()

    def export(name, ids, origin, link, material):
        if used.intersection(ids):
            raise ValueError('Duplicate component assignment')
        used.update(ids)
        part = triangles[np.isin(labels, ids)]
        filename = 'cad_' + name + '.stl'
        write_stl(package / 'meshes' / filename, part - origin)
        records.append(dict(mesh=filename, components=ids, triangles=len(part), origin_m=list(origin), link=link, material=material))

    wheel_data = {}
    for name, (tyre, hub, axle) in WHEELS.items():
        t = triangles[np.isin(labels, tyre)]
        a = triangles[labels == axle]
        origin = (a.min((0, 1)) + a.max((0, 1))) / 2
        origin[1] = (t[:, :, 1].min() + t[:, :, 1].max()) / 2
        export(name + '_tyre', tyre, origin, name + '_wheel', 'rubber')
        export(name + '_hub', hub, origin, name + '_wheel', 'aluminum')
        radius = .23463 if name[1] == '1' else .17655
        wheel_data[name] = dict(origin_m=origin.tolist(), radius_m=radius,
                               width_m=float(np.ptp(t[:, :, 1])), mass_kg=35.2 if name[1] == '1' else 15.3)
    pivots = {}
    for side, ids in ROCKERS.items():
        origin = np.array([-.5697, .30 if side == 'L' else -.30, -.135])
        pivots[side] = origin.tolist()
        export(side + '_rocker', ids, origin, side + '_rocker_link', 'aluminum')
    export('camera', [29, 30], np.array([.4672, 0., .0119]), 'camera_link', 'rubber')
    export('lidar', list(range(31, 45)) + list(range(46, 60)), np.array([-.5865, 0., .497]), 'lidar_link', 'rubber')
    export('body', sorted(set(np.unique(labels).tolist()) - used), np.zeros(3), 'base_link', 'aluminum')
    assert used == set(np.unique(labels))
    ns = 'http://www.ros.org/wiki/xacro'
    ET.register_namespace('xacro', ns)
    root = ET.Element('robot')
    macro = ET.SubElement(root, '{'+ns+'}macro', name='cad_visual', params='part material:=aluminum pitch:=0')
    visual = ET.SubElement(macro, 'visual')
    ET.SubElement(visual, 'origin', rpy='0 ${pitch} 0')
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(geometry, 'mesh', filename='package://agri_ugv_description/meshes/cad_${part}.stl')
    ET.SubElement(visual, 'material', name='${material}')
    for side in ['L', 'R']:
        macro = ET.SubElement(root, '{'+ns+'}macro', name='cad_'+side+'_wheels')
        for index in range(1, 5):
            wheel = wheel_data[side+str(index)]
            rel = np.array(wheel['origin_m']) - pivots[side]
            ET.SubElement(macro, '{'+ns+'}wheel', side=side, index=str(index), xyz=' '.join(f'{v:.9f}' for v in rel),
                          radius=str(wheel['radius_m']), width=str(wheel['width_m']), mass=str(wheel['mass_kg']))
    ET.indent(root)
    ET.ElementTree(root).write(package/'urdf/cad_geometry.xacro', encoding='utf-8', xml_declaration=True)
    audit = dict(source=str(args.stl.resolve()), sha256=SOURCE_SHA256, triangles=len(triangles),
                 components=len(used), units='metres', cad_to_ros_rotation=ROTATION.tolist(), offset_m=OFFSET.tolist(),
                 wheels=wheel_data, rocker_pivots_m=pivots, parts=records,
                 limitations=['STL has no kinematics, masses or materials.',
                              'Two open-chain rocker pivots inferred at the rear cross-shaft.',
                              'Hydraulic cylinder visuals follow their rocker; no closed-loop hydraulic dynamics.',
                              'As-exported left/right articulation and raised rear wheels retained.',
                              'Collision cylinders and box proxies approximate CAD surfaces.'])
    (package/'config/cad_geometry.json').write_text(json.dumps(audit, indent=2)+'\n')
    print(f'Exported {len(records)} meshes; all {len(triangles)} triangles assigned exactly once.')


if __name__ == '__main__':
    main()
