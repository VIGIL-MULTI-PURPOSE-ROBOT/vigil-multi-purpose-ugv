#!/usr/bin/python3
"""Agriculture autonomy supervisor node (the reasoning is agri_autonomy_core.AgriSupervisor).

Observes the existing field-coverage mission and only acts through crop_row_driver's hook.

Subscribes (existing topics):
  sim/ground_truth (Odometry)         rover pose            imu/data (Imu)          yaw rate
  encoders (JointState)               wheel speeds          camera/image, camera/depth_image, lidar/points
                                                            (sensor health only - arrival times)
  cmd_vel (Twist)                     the command drive.py receives (commanded motion)
  crop_row/progress, exploration/status   the row mission    agri_dashboard/robot_state, agri_dashboard/command
  agriculture/obstacles               tracked obstacles     agriculture/moisture(+ /moisture_map/status)
  agri_vision/tracks                  perception timeliness
Publishes:
  agriculture/autonomy/status (String JSON, latched)   state, decision, confidences, field, metrics, log
  agriculture/autonomy/command (String)                RECOVER | SPEED <v> | HOLD <why> | RESUME
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image, Imu, JointState, PointCloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from agri_autonomy_core import AgriSupervisor  # noqa: E402
from agri_mission_core import find_config, load_yaml  # noqa: E402

# ground-bearing wheels (L1-L3, R1-R3) and radii: odometry.py's six-encoder model
WHEEL_R = {'1': 0.23463, '2': 0.17655, '3': 0.17655}


class AgriSupervisorNode(Node):
    def __init__(self):
        super().__init__('agri_supervisor')
        cfg_path = self.declare_parameter('config', find_config(__file__)).value
        cfg = load_yaml(cfg_path) if cfg_path else {}
        a = dict(cfg.get('autonomy', {}))
        ob = cfg.get('obstacles', {})
        a.setdefault('robot_half_length', ob.get('robot_half_length', 0.85))
        a.setdefault('robot_half_width', ob.get('robot_half_width', 0.55))
        self.sup = AgriSupervisor(a)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        s = qos_profile_sensor_data
        self.pub = self.create_publisher(String, 'agriculture/autonomy/status', latched)
        self.cmd_pub = self.create_publisher(String, 'agriculture/autonomy/command', 10)
        self.create_subscription(Odometry, 'sim/ground_truth', self.on_odom, s)
        self.create_subscription(Imu, 'imu/data', lambda m: self.sup.on_imu(self.now(), m.angular_velocity.z), s)
        self.create_subscription(JointState, 'encoders', self.on_encoders, s)
        self.create_subscription(Image, 'camera/image', lambda m: self.sup.on_sensor(self.now(), 'rgb'), s)
        self.create_subscription(Image, 'camera/depth_image', lambda m: self.sup.on_sensor(self.now(), 'depth'), s)
        self.create_subscription(PointCloud2, 'lidar/points', lambda m: self.sup.on_sensor(self.now(), 'lidar'), s)
        # no-progress uses what drive.py actually receives (/cmd_vel), not the request before the Nav2
        # chain: a silent chain is not a stuck rover (the START gate handles it)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self.sup.on_cmd(self.now(), m.linear.x, m.angular.z), 10)
        self.create_subscription(String, 'crop_row/progress', lambda m: self._json(m, self.sup.on_progress), latched)
        self.create_subscription(String, 'exploration/status', lambda m: self._json(m, self.sup.on_driver), latched)
        self.create_subscription(String, 'agri_dashboard/robot_state',
                                 lambda m: self._json(m, lambda t, d: self.sup.on_gate(t, d.get('state'))), latched)
        self.create_subscription(String, 'agri_dashboard/command', lambda m: self.sup.on_operator(self.now(), m.data), 10)
        self.create_subscription(String, 'agriculture/obstacles',
                                 lambda m: self._json(m, lambda t, d: self.sup.on_obstacles(t, d.get('obstacles', []))), 10)
        self.create_subscription(String, 'agriculture/moisture', lambda m: self._json(m, self.sup.on_moisture), 20)
        self.create_subscription(String, 'agriculture/moisture_map/status',
                                 lambda m: self._json(m, self.sup.on_moisture_status), latched)
        self.create_subscription(String, 'agri_vision/tracks', lambda m: self._json(m, self.sup.on_vision), latched)
        self.create_timer(float(a.get('period', 0.25)), self.step)
        self.get_logger().info('agri_supervisor: field-coverage autonomy supervisor running')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _json(self, m, fn):
        try:
            fn(self.now(), json.loads(m.data))
        except (ValueError, TypeError):
            pass

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.sup.on_pose(self.now(), p.x, p.y, yaw)

    def on_encoders(self, m):
        rates = dict(zip(m.name, m.velocity))
        v = []
        for side in 'LR':
            for i, r in WHEEL_R.items():
                w = rates.get(f'{side}{i}_joint')
                if w is not None and math.isfinite(w):
                    v.append(w * r)
        if v:
            v.sort()
            self.sup.on_encoders(self.now(), v[len(v) // 2])

    def step(self):
        st = self.sup.step(self.now())
        for c in self.sup.commands:
            self.cmd_pub.publish(String(data=c))
            self.get_logger().info(f'autonomy command -> driver: {c}')
        self.pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = AgriSupervisorNode()
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
