#!/usr/bin/python3
"""Check the converted physics model, attachment geometry and preserved interfaces."""
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import xacro

PACKAGE = Path(__file__).resolve().parents[1]

class ModelContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = ET.fromstring(xacro.process_file(str(PACKAGE / 'urdf/agri_ugv.urdf.xacro'), mappings={'simulation': 'true'}).toxml())
        with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
            f.write(ET.tostring(cls.urdf)); f.flush()
            cls.sdf = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', f.name], text=True)).find('model')

    def test_preserved_chassis_wheels_and_sensor_mounts(self):
        u = self.urdf
        self.assertEqual(u.find("joint[@name='body_reference']/origin").get('xyz'), '0 0 0.5872')
        self.assertEqual(u.find("link[@name='base_link']/collision/geometry/box").get('size'), '1.0737 0.5 0.3523')
        for side in 'LR':
            for i in range(1, 5):
                n = f'{side}{i}'
                j = u.find(f"joint[@name='{n}_joint']")
                self.assertEqual(j.get('type'), 'continuous')
                self.assertEqual(j.find('axis').get('xyz'), '0 1 0')
                cyl = u.find(f"link[@name='{n}_wheel']/collision/geometry/cylinder")
                self.assertAlmostEqual(float(cyl.get('radius')), .23463 if i == 1 else .17655)
        for name in ['camera_mount', 'lidar_mount', 'imu_mount', 'camera_optical_joint']:
            self.assertEqual(u.find(f"joint[@name='{name}']").get('type'), 'fixed')
        self.assertEqual({s.get('type') for s in self.sdf.findall('.//sensor')} - {'contact'}, {'imu', 'rgbd_camera', 'gpu_lidar'})

    def test_native_passive_springs_and_collision_references(self):
        hw = self.urdf.find('ros2_control')
        for side in 'LR':
            for i in range(1, 5):
                n = f'{side}{i}'
                j = self.sdf.find(f"joint[@name='{n}_suspension']")
                self.assertEqual(j.get('type'), 'prismatic')
                self.assertAlmostEqual(float(j.findtext('axis/limit/lower')), -.10)
                self.assertAlmostEqual(float(j.findtext('axis/limit/upper')), .12)
                self.assertEqual(float(j.findtext('axis/dynamics/spring_stiffness')), 8000)
                self.assertEqual(float(j.findtext('axis/dynamics/damping')), 650)
                self.assertIsNone(hw.find(f"joint[@name='{n}_suspension']/command_interface"))
                link = self.sdf.find(f"link[@name='{n}_wheel']")
                collision = link.find('collision')
                sensor = link.find('sensor/contact')
                self.assertEqual(sensor.findtext('collision'), collision.get('name'))
                self.assertEqual(sensor.findtext('topic'), f'/suspension/contacts/{n}')
                self.assertAlmostEqual(float(collision.findtext('surface/friction/ode/mu')), .85)
                # Guide and slider retain overlap at BOTH hard stops.
                for q in [-.10, .12]:
                    self.assertGreater(min(.23, q+.11)-max(-.07, q-.11), 0)
            j = self.sdf.find(f"joint[@name='{side}_rocker']")
            self.assertEqual(float(j.findtext('axis/dynamics/spring_stiffness')), 3500)
            self.assertEqual(float(j.findtext('axis/dynamics/damping')), 300)
            self.assertIsNone(hw.find(f"joint[@name='{side}_rocker']/command_interface"))

    def test_positive_inertia(self):
        for link in self.sdf.findall('link'):
            mass = float(link.findtext('inertial/mass'))
            self.assertGreater(mass, 0)
            inertia = link.find('inertial/inertia')
            a = np.array([[float(inertia.findtext('i'+x+y if x <= y else 'i'+y+x)) for y in 'xyz'] for x in 'xyz'])
            eig = np.linalg.eigvalsh(a)
            self.assertTrue(np.all(eig > 0), link.get('name'))
            self.assertLessEqual(eig[-1], eig[:2].sum()+1e-8, link.get('name'))

    def test_original_cad_axles_are_preserved(self):
        dtype = np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','u2')])
        def mesh(name):
            return np.frombuffer((PACKAGE / f'meshes/cad_{name}.stl').read_bytes(), dtype=dtype, offset=84)['v'].astype(float)
        for side in 'LR':
            original = mesh(side+'_rocker')
            parts = [mesh(side+'_suspended_rocker')]
            for i in range(1, 5):
                j = self.urdf.find(f"joint[@name='{side}{i}_guide_mount']/origin")
                parts.append(mesh(f'{side}{i}_axle') + np.fromstring(j.get('xyz'), sep=' '))
            combined = np.concatenate(parts)
            self.assertEqual(combined.shape, original.shape)
            # Every triangle centroid recovers the original mesh, allowing float32 export rounding.
            from scipy.spatial import cKDTree
            distance, _ = cKDTree(original.mean(1)).query(combined.mean(1))
            self.assertLess(distance.max(), 2e-7)

if __name__ == '__main__':
    unittest.main()
