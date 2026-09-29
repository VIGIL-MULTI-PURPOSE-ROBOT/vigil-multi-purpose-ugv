#!/usr/bin/python3
"""Live soil-moisture map of the crop field, built while the robot drives its rows.

In : /agriculture/moisture (agri_moisture_sensor.py), /crop_row/progress (mission state)
Out: /agriculture/moisture_map              nav_msgs/OccupancyGrid  moisture % per cell (-1 = unknown)
     /agriculture/moisture_map/confidence   nav_msgs/OccupancyGrid  0 = interpolated at the edge ..
                                                                     100 = at a measured sample
     /agriculture/moisture_map/status       std_msgs/String JSON    coverage %, complete, statistics,
                                                                     latest reading, sample points
     /agriculture/moisture_events           std_msgs/String JSON    meaningful events only
Only the crop field is mapped (field rows +- half a row spacing, planted extent in x).
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from agri_mission_core import FieldGeometry, find_config, load_yaml, package_root  # noqa: E402
from agri_moisture_core import DEFAULTS, MoistureMapper  # noqa: E402


def grid_msg(values, bounds, res, stamp):
    g = OccupancyGrid()
    g.header.frame_id = 'world'
    g.header.stamp = stamp
    g.info.resolution = float(res)
    g.info.height, g.info.width = values.shape
    g.info.origin.position.x, g.info.origin.position.y = float(bounds[0]), float(bounds[2])
    g.info.origin.orientation.w = 1.0
    g.data = values.astype(np.int8).ravel().tolist()
    return g


class MoistureMapNode(Node):
    def __init__(self):
        super().__init__('agri_moisture_map')
        path = self.declare_parameter('config', find_config(__file__)).value
        cfg = load_yaml(path) if path else {}
        cfg.setdefault('field', dict(first_row_y=-13.418, row_spacing=1.22, field_width=26.84,
                                     crop_area_start=-13.9, crop_area_end=14.4, auto_row_detection='dimensions'))
        geo = FieldGeometry(cfg, package_root(path) if path else None)
        self.c = dict(DEFAULTS, **cfg.get('moisture', {}))
        self.bounds = geo.bounds()
        self.m = MoistureMapper(self.bounds, self.c)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map = self.create_publisher(OccupancyGrid, 'agriculture/moisture_map', latched)
        self.pub_conf = self.create_publisher(OccupancyGrid, 'agriculture/moisture_map/confidence', latched)
        self.pub_status = self.create_publisher(String, 'agriculture/moisture_map/status', latched)
        self.pub_events = self.create_publisher(String, 'agriculture/moisture_events', 50)
        self.create_subscription(String, 'agriculture/moisture', self.on_reading, 50)
        self.create_subscription(String, 'crop_row/progress', self.on_progress, latched)
        self.create_timer(1.0 / float(self.c['map_publish_rate']), self.publish)
        self.dirty = True

    def on_reading(self, msg):
        try:
            r = json.loads(msg.data)
        except ValueError:
            return
        for e in self.m.add(r):
            self.pub_events.publish(String(data=json.dumps(e)))
        self.dirty = True

    def on_progress(self, msg):
        try:
            done = json.loads(msg.data).get('done')
        except ValueError:
            return
        if done:
            for e in self.m.finish():
                self.pub_events.publish(String(data=json.dumps(e)))
            self.dirty = True

    def publish(self):
        if not self.dirty:
            return
        self.dirty = False
        v, conf = self.m.map.grid()
        vals = np.where(np.isfinite(v), np.clip(np.round(v), 0, 100), -1)
        stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(grid_msg(vals, self.bounds, self.m.map.res, stamp))
        self.pub_conf.publish(grid_msg(np.round(conf * 100), self.bounds, self.m.map.res, stamp))
        self.pub_status.publish(String(data=json.dumps(self.m.status())))


def main():
    rclpy.init()
    node = MoistureMapNode()
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
