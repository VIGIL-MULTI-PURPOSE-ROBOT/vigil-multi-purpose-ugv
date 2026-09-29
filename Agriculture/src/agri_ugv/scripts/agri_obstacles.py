#!/usr/bin/python3
"""Obstacle tracker node for the crop-row mission (observes only; never commands the robot).

In : /lidar/points (360 deg, 16 beams, existing), /camera/depth_image + /camera/camera_info
     (existing, near field in front), /sim/ground_truth (existing pose), /agri_vision/tracks (labels)
Out: /agriculture/obstacles  std_msgs/String JSON {t, robot, obstacles:[{id, x, y, vx, vy, speed,
     dynamic, points, height, size, cls, distance, bearing}], counts}
     /agriculture/obstacle_events  std_msgs/String JSON, one per newly confirmed obstacle
Logic: agri_obstacle_core.py.  Settings: config/agriculture.yaml (obstacles:).
"""
import bisect
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

from agri_mission_core import FieldGeometry, find_config, load_yaml, package_root  # noqa: E402
from agri_obstacle_core import ObstacleDetector, ObstacleTracker  # noqa: E402

LIDAR_FALLBACK = (-0.5865, 0.0, 1.0842)     # base_footprint -> lidar_link (URDF)


def quat_to_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class AgriObstacles(Node):
    def __init__(self):
        super().__init__('agri_obstacles')
        path = self.declare_parameter('config', find_config(__file__)).value
        cfg = load_yaml(path) if path else {}
        cfg.setdefault('field', dict(first_row_y=-13.418, row_spacing=1.22, field_width=26.84,
                                     crop_area_start=-13.9, crop_area_end=14.4, auto_row_detection='dimensions'))
        self.geo = FieldGeometry(cfg, package_root(path) if path else None)
        crop = cfg.get('crop', {})
        band = float(crop.get('row_position_tolerance', 0.30)) + float(crop.get('guard_band', 0.12))
        self.ob = cfg.get('obstacles', {})
        self.det = ObstacleDetector(self.ob, self.geo, band)
        self.trk = ObstacleTracker(self.ob)
        self.use_depth = bool(self.ob.get('use_depth', True))
        self.odom = []
        self.labels = []
        self.depth_pts = None
        self.T = {}
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(String, 'agriculture/obstacles', 10)
        self.events = self.create_publisher(String, 'agriculture/obstacle_events', 20)
        self.create_subscription(Odometry, 'sim/ground_truth', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'lidar/points', self.on_lidar, qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/depth_image', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'agri_vision/tracks', self.on_labels, latched)
        self.K = None
        self.last_depth = 0.0
        self.get_logger().info(f'obstacle tracker: {self.geo.n} crop rows, band {band:.2f} m excluded as crop')

    # ------------------------------------------------------------------ inputs
    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom.append((t, p.x, p.y, p.z, quat_to_matrix(q.x, q.y, q.z, q.w)))
        if len(self.odom) > 200:
            self.odom = self.odom[-100:]

    def pose_at(self, t):
        if not self.odom:
            return None
        ts = [o[0] for o in self.odom]
        i = min(bisect.bisect_left(ts, t), len(ts) - 1)
        if i > 0 and abs(ts[i - 1] - t) < abs(ts[i] - t):
            i -= 1
        return self.odom[i] if abs(ts[i] - t) < 0.3 else None

    def base_to(self, frame, fallback):
        if frame in self.T:
            return self.T[frame]
        try:
            tr = self.tf.lookup_transform('base_footprint', frame, rclpy.time.Time()).transform
            T = np.eye(4)
            T[:3, :3] = quat_to_matrix(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
            T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            self.T[frame] = T
            return T
        except Exception:
            return fallback

    def on_labels(self, m):
        try:
            items = json.loads(m.data).get('items', [])
        except ValueError:
            return
        self.labels = [dict(x=i['x'], y=i['y'], cls=('person' if i['cls'] == 'HUMAN' else i.get('sub', 'object')))
                       for i in items if i['cls'] in ('HUMAN', 'OTHER_OBJECT') and i.get('active', True)]

    def on_info(self, m):
        self.K = np.array(m.k, float).reshape(3, 3)

    def on_depth(self, m):
        """Near-field points in front of the robot (every 4th pixel, <= 6 m)."""
        if not self.use_depth or self.K is None or time.monotonic() - self.last_depth < 0.2:
            return
        self.last_depth = time.monotonic()
        if m.encoding.lower() != '32fc1':
            return
        d = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.step // 4)[:, :m.width][::4, ::4]
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pose = self.pose_at(t)
        if pose is None:
            return
        vv, uu = np.mgrid[0:m.height:4, 0:m.width:4]
        ok = np.isfinite(d) & (d > 0.25) & (d < 6.0)
        K = self.K
        pc = np.stack(((uu[ok] - K[0, 2]) / K[0, 0] * d[ok], (vv[ok] - K[1, 2]) / K[1, 1] * d[ok], d[ok]), 1)
        fallback = np.eye(4)
        fallback[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
        fallback[:3, 3] = [0.4672, 0.0, 0.5991]
        T = self.base_to(m.header.frame_id or 'camera_optical_frame', fallback)
        self.depth_pts = (t, self.to_world(pc, T, pose))

    @staticmethod
    def to_world(pts, T_base, pose):
        _, x, y, z, R = pose
        pb = pts @ T_base[:3, :3].T + T_base[:3, 3]
        return pb @ R.T + np.array([x, y, z])

    def on_lidar(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pose = self.pose_at(t)
        if pose is None:
            return
        try:
            p = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:
            return
        fallback = np.eye(4)
        fallback[:3, 3] = LIDAR_FALLBACK
        P = self.to_world(np.asarray(p, float), self.base_to(m.header.frame_id or 'lidar_link', fallback), pose)
        if self.depth_pts is not None and abs(self.depth_pts[0] - t) < 0.5:
            P = np.vstack((P, self.depth_pts[1]))
        _, x, y, z, R = pose
        yaw = math.atan2(R[1, 0], R[0, 0])
        robot = (x, y, yaw, z)
        clusters = self.det.detect(P, robot)
        self.trk.update(clusters, t, robot)
        obs = self.trk.export(t, robot, self.labels)
        for tr in self.trk.tracks:
            if tr.hits >= 3 and not tr.announced:
                tr.announced = True
                o = next((o for o in obs if o['id'] == f'OB{tr.id}'), None)
                if o:
                    self.events.publish(String(data=json.dumps(dict(t=t, **o))))
        self.pub.publish(String(data=json.dumps(dict(
            t=t, robot=[round(x, 2), round(y, 2), round(yaw, 3)], obstacles=obs,
            counts=dict(tracked=len(obs), dynamic=sum(o['dynamic'] for o in obs))))))


def main():
    rclpy.init()
    node = AgriObstacles()
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
