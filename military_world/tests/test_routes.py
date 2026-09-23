import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sar_motion import timed_route  # noqa: E402
from sar_physics import configure_world  # noqa: E402


class RoutesTest(unittest.TestCase):
    def test_closes_position_and_heading_without_zero_time_segments(self):
        keys = timed_route([(0, 0, 0), (0, 0, 0), (3, 4, 0)], 1.0)
        self.assertEqual(keys[0][1], keys[-1][1])
        self.assertAlmostEqual(keys[-1][0], 10.0)
        self.assertTrue(all(a[0] < b[0] for a, b in zip(keys, keys[1:])))

    def test_terrain_height_contributes_to_speed_and_dwell(self):
        keys = timed_route([(0, 0, 0), (3, 0, 4)], 2.0, 1.0)
        self.assertEqual(keys[0][1], keys[1][1])
        self.assertEqual(keys[1][0], 1.0)
        self.assertAlmostEqual(keys[-1][0], 7.0)

    def test_invalid_routes_fail_export(self):
        for points, speed in [([(0, 0, 0)], 1),
                              ([(0, 0, 0), (1, 0, 0)], 0),
                              ([(0, 0, 0), (math.nan, 0, 0)], 1)]:
            with self.assertRaises(ValueError):
                timed_route(points, speed)


class PhysicsTest(unittest.TestCase):
    """The controller the SDF asks for is the controller the plugin reads."""

    import xml.etree.ElementTree as ET

    WORLD = """<sdf version='1.10'><world name='w'>
      <model name='SAR_DynamicPerson_001'><static>true</static>
        <pose>0 0 0 0 0 0</pose>
        <link name='body'><collision name='c00'><geometry><box>
          <size>1 1 1</size></box></geometry></collision></link>
        <plugin filename='libsar-waypoint-system.so' name='sar::WaypointSystem'>
          <offset>0 0 0</offset>
          <waypoint><time>0</time><pose>0 0 0 0 0 0</pose></waypoint>
          <waypoint><time>4</time><pose>4 0 0 0 0 0</pose></waypoint>
        </plugin></model></world></sdf>"""

    def world(self, controller):
        root = self.ET.fromstring(self.WORLD)
        configure_world(root, controller)
        return root

    def test_controller_version_is_written_through(self):
        for controller in (1, 2):
            root = self.world(controller)
            plugin = root.find(".//plugin[@name='sar::WaypointSystem']")
            self.assertEqual(plugin.findtext('controller_version'),
                             str(controller))

    def test_kinematic_people_do_not_fall_and_skip_dart_tuning(self):
        root = self.world(1)
        self.assertEqual(root.findtext('.//link/gravity'), 'false')
        self.assertIsNone(root.find('.//physics/dart'))

    def test_contact_people_stand_under_gravity_with_dart_tuning(self):
        root = self.world(2)
        self.assertEqual(root.findtext('.//link/gravity'), 'true')
        self.assertEqual(root.findtext('.//physics/dart/collision_detector'),
                         'bullet')

    def test_repeated_configuration_does_not_duplicate_elements(self):
        root = self.ET.fromstring(self.WORLD)
        for _ in range(3):
            configure_world(root, 2)
        model = root.find('.//model')
        self.assertEqual(len(model.findall('.//inertial')), 1)
        self.assertEqual(len(root.findall('.//physics/dart')), 1)

    def test_invalid_controller_is_rejected(self):
        for bad in (0, 3, '2'):
            with self.assertRaises(ValueError):
                configure_world(self.ET.fromstring(self.WORLD), bad)


if __name__ == '__main__':
    unittest.main()
