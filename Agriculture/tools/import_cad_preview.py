#!/usr/bin/python3
"""Import the unarticulated STL faithfully for inspection, not dynamics."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stl', type=Path)
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[1]
    package = ws / 'src/agri_ugv_description'
    meshes = package / 'meshes'
    meshes.mkdir(parents=True, exist_ok=True)
    reports = ws / 'reports'
    reports.mkdir(exist_ok=True)
    data = args.stl.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError('Expected an uncompressed binary STL')
    dtype = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, offset=84)['v']
    # Round solely for connectivity detection; export the original coordinates.
    vertices, inverse = np.unique(np.round(triangles.reshape(-1, 3), 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                       shape=(len(vertices), len(vertices))).tocsr()
    component_count, labels = connected_components(graph, directed=False)
    face_labels = labels[faces[:, 0]]
    records = []
    for i in range(component_count):
        points = triangles[face_labels == i].reshape(-1, 3)
        records.append(dict(id=i, triangles=int(len(points) // 3),
                            min_mm=points.min(0).astype(float).tolist(),
                            max_mm=points.max(0).astype(float).tolist()))
    shutil.copyfile(args.stl, meshes / 'assembly_preview.stl')
    # Coordinate convention follows the earlier CAD-derived description, with
    # the measured chassis centreline refined to -511.26 mm. No part is moved
    # relative to another. Millimetres are interpreted from the 500 mm shell.
    robot = ET.Element('robot', name='agri_cad_preview')
    ET.SubElement(robot, 'link', name='base_footprint')
    body = ET.SubElement(robot, 'link', name='cad_assembly')
    visual = ET.SubElement(body, 'visual')
    ET.SubElement(visual, 'origin', xyz='-0.5697 0.51126 0.5872', rpy='0 0 1.5707963267948966')
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(geometry, 'mesh', filename='package://agri_ugv_description/meshes/assembly_preview.stl', scale='0.001 0.001 0.001')
    material = ET.SubElement(visual, 'material', name='preview_aluminum')
    ET.SubElement(material, 'color', rgba='0.62 0.65 0.69 1')
    joint = ET.SubElement(robot, 'joint', name='preview_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='base_footprint')
    ET.SubElement(joint, 'child', link='cad_assembly')
    ET.indent(robot)
    (package / 'urdf').mkdir(exist_ok=True)
    ET.ElementTree(robot).write(package / 'urdf/cad_preview.urdf', encoding='utf-8', xml_declaration=True)
    audit = dict(source=str(args.stl.resolve()), sha256=hashlib.sha256(data).hexdigest(),
                 triangles=count, connected_components=component_count,
                 bounds_mm=[triangles.reshape(-1, 3).min(0).tolist(), triangles.reshape(-1, 3).max(0).tolist()],
                 coordinate_assumption='STL millimetres; CAD -Y forward, +X left, +Z up',
                 status='static geometry preview only; no inferred physical joints or inertia',
                 missing=['original joint graph, axes and joint types', 'joint limits',
                          'as-built standing angle', 'component material and mass assignments',
                          'actuator attachment and force/stroke specifications', 'IMU mounting frame'],
                 components=records)
    (reports / 'cad_import.json').write_text(json.dumps(audit, indent=2) + '\n')
    print(f'Imported {count} triangles in {component_count} components; geometry preserved.')
    print('This is a static preview, not an articulated simulation robot.')


if __name__ == '__main__':
    main()
