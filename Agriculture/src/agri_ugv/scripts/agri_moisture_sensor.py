#!/usr/bin/python3
"""Simulated soil-moisture probe (there is no real moisture sensor in Gazebo).

The probe is the moisture_probe_link on the robot (agri_ugv.urdf.xacro): under the chassis,
between the crop row and the left tyre, so it touches soil, not plants. At update_rate it reads the
seeded moisture field (agri_moisture_core.MoistureField) at the probe's world position, adds
measurement noise, and publishes

  /agriculture/moisture   std_msgs/String JSON
      {moisture, x, y, stamp, row, category, valid}      (moisture in %, x/y world metres)

Readings are published only over the crop field (valid soil). Settings: config/agriculture.yaml
(moisture:). Same seed -> same field; change moisture.seed for another field.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

from agri_mission_core import FieldGeometry, find_config, load_yaml, package_root  # noqa: E402
from agri_moisture_core import DEFAULTS, MoistureField, MoistureSensor  # noqa: E402


class MoistureSensorNode(Node):
    def __init__(self):
        super().__init__('agri_moisture_sensor')
        path = self.declare_parameter('config', find_config(__file__)).value
        cfg = load_yaml(path) if path else {}
        cfg.setdefault('field', dict(first_row_y=-13.418, row_spacing=1.22, field_width=26.84,
                                     crop_area_start=-13.9, crop_area_end=14.4, auto_row_detection='dimensions'))
        self.geo = FieldGeometry(cfg, package_root(path) if path else None)
        self.c = dict(DEFAULTS, **cfg.get('moisture', {}))
        self.bounds = self.geo.bounds()
        self.field = MoistureField(self.bounds, self.c)
        self.sensor = MoistureSensor(self.field, self.c)
        self.offset = None
        self.pose = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(String, 'agriculture/moisture', 20)
        self.create_subscription(Odometry, 'sim/ground_truth', self.on_odom, qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.c['update_rate']), self.sample)
        self.get_logger().info(f"moisture field: seed {self.c['seed']}, {self.c['min_moisture']}-{self.c['max_moisture']} %, "
                               f"field x {self.bounds[0]:.1f}..{self.bounds[1]:.1f} y {self.bounds[2]:.1f}..{self.bounds[3]:.1f}")

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def probe_offset(self):
        if self.offset is None:
            try:
                tr = self.tf.lookup_transform('base_footprint', 'moisture_probe_link', rclpy.time.Time()).transform
                self.offset = (tr.translation.x, tr.translation.y)
                self.get_logger().info(f'moisture probe from TF at {np.round(self.offset, 3).tolist()} m')
            except Exception:
                return float(self.c['probe_x']), float(self.c['probe_y'])
        return self.offset

    def sample(self):
        if self.pose is None or not self.c.get('enabled', True):
            return
        x, y, yaw = self.pose
        ox, oy = self.probe_offset()
        px, py = x + ox * math.cos(yaw) - oy * math.sin(yaw), y + ox * math.sin(yaw) + oy * math.cos(yaw)
        x0, x1, y0, y1 = self.bounds
        if not (x0 <= px <= x1 and y0 <= py <= y1):
            return                                   # over the headland / outside the crop field
        value, _ = self.sensor.read(px, py)
        lo, hi = float(self.c['low_threshold']), float(self.c['high_threshold'])
        self.pub.publish(String(data=json.dumps(dict(
            moisture=round(value, 2), x=round(px, 3), y=round(py, 3),
            stamp=self.get_clock().now().nanoseconds * 1e-9, row=int(self.geo.nearest_row(y)) + 1,
            category='DRY' if value < lo else ('WET' if value > hi else 'NORMAL'), valid=True))))


def main():
    rclpy.init()
    node = MoistureSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
