#!/usr/bin/python3
"""A-to-B navigation with cliff-aware replanning.

Replaces the fixed waypoint list of comparison_driver.py with an A* planner on the
vision traversability map. Output is the same /cmd_vel that drive.py already
converts into the eight wheel velocities (drive.py is unchanged).

Subscribes: /vision/terrain_classes, /vision/slope, /vision/roughness, /sim/ground_truth,
            /navigation/goal and /goal_pose (geometry_msgs/PoseStamped, e.g. RViz 2D goal)
Publishes:  /cmd_vel, /navigation/path, /navigation/previous_path,
            /navigation/status (JSON string)
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from geometry_msgs.msg import Twist, PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry, OccupancyGrid, Path  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from terrain_core import Params  # noqa: E402
from planner_core import NavigatorCore  # noqa: E402
from ros_common import nested_params, odom_to_pose, grid_to_numpy, path_msg  # noqa: E402


class CliffNavigator(Node):
    def __init__(self):
        super().__init__('cliff_navigator', automatically_declare_parameters_from_overrides=True)
        cfg = nested_params(self)
        self.tp = Params.from_nested(cfg)
        self.nav = NavigatorCore(self.tp, cfg['navigation'])
        self.pose = None
        self.speed = 0.0
        self.published_version = -1
        self.published_prev = None
        self.slope = self.rough = None
        self.create_subscription(OccupancyGrid, '/vision/slope',
                                 lambda m: setattr(self, 'slope', grid_to_numpy(m).astype(np.float32)), 2)
        self.create_subscription(OccupancyGrid, '/vision/roughness',
                                 lambda m: setattr(self, 'rough', grid_to_numpy(m).astype(np.float32) / 100.0), 2)
        self.create_subscription(OccupancyGrid, '/vision/terrain_classes', self.on_map, 2)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/navigation/goal', self.on_goal, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/navigation/path', 2)
        self.pub_prev = self.create_publisher(Path, '/navigation/previous_path', 2)
        self.pub_status = self.create_publisher(String, '/navigation/status', 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.republish_paths)
        g = self.nav.goal
        self.get_logger().info(f'cliff_navigator: goal B = ({g[0]:.2f}, {g[1]:.2f}); '
                               f'send a new one on /navigation/goal or /goal_pose')

    def on_map(self, m):
        cls = grid_to_numpy(m)
        ok = lambda a: a if a is not None and a.shape == cls.shape else None  # noqa: E731
        self.nav.update_map(np.where(cls < 0, 0, cls).astype(np.uint8), ok(self.slope), ok(self.rough))

    def on_odom(self, m):
        self.pose = odom_to_pose(m)
        t = m.twist.twist.linear
        self.speed = float(np.hypot(t.x, t.y))

    def on_goal(self, m):
        self.nav.set_goal(m.pose.position.x, m.pose.position.y, self.now())
        self.get_logger().info(f'new goal B = ({m.pose.position.x:.2f}, {m.pose.position.y:.2f})')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.pose is None:
            return
        v, w = self.nav.tick(self.now(), self.pose)
        tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        self.cmd.publish(tw)
        st = self.nav.status(self.pose)
        st.update(x=round(self.pose.x, 2), y=round(self.pose.y, 2), heading_deg=round(np.degrees(self.pose.yaw), 1),
                  speed=round(self.speed, 3), tilt=round(self.pose.tilt_deg, 1))
        self.pub_status.publish(String(data=json.dumps(st)))
        if self.nav.path_version != self.published_version or self.nav.previous_path is not self.published_prev:
            self.republish_paths()

    def republish_paths(self):
        stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(path_msg(self.nav.path, stamp))
        self.pub_prev.publish(path_msg(self.nav.previous_path, stamp))
        self.published_version = self.nav.path_version
        self.published_prev = self.nav.previous_path


def main():
    rclpy.init()
    node = CliffNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
