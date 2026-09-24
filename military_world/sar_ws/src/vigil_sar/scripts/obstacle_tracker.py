#!/usr/bin/python3
"""Dynamic-obstacle detection and tracking node (obstacle_core does the work).

Existing sensors only:
  /camera/depth_image               sensor_msgs/Image 32FC1   (rgbd_camera)
  /camera/segmentation/labels_map   sensor_msgs/Image         (label per pixel, same mount/FOV/resolution)
  /camera/camera_info               sensor_msgs/CameraInfo
  /sim/ground_truth                 nav_msgs/Odometry         (rover pose)
Publishes:
  /perception/obstacles             std_msgs/String JSON: tracks (id, class, position, velocity, speed,
                                    direction, distance, dynamic), counts, timing

Timestamps: every depth frame is paired with the segmentation frame of the SAME stamp (or the
nearest one within obstacles.sync_tolerance) and with the rover pose interpolated at that stamp, so
the world points stay right at any simulation speed (at 4x a 10 Hz camera is 40 frames per wall
second and the pose moves 4x as far per wall second). All times are simulation time.
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
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Image, CameraInfo  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from terrain_core import Params, CameraModel, Pose, quat_to_matrix  # noqa: E402
from obstacle_core import ObstacleParams, ObstacleTracker, extract_points, cluster, describe  # noqa: E402
from ros_common import nested_params, image_to_numpy, node_time  # noqa: E402


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class ObstacleTrackerNode(Node):
    def __init__(self):
        super().__init__('obstacle_tracker', automatically_declare_parameters_from_overrides=True)
        cfg = nested_params(self)
        self.tp = Params.from_nested(cfg)
        self.p = ObstacleParams.from_dict(cfg.get('obstacles', {}))
        self.tracker = ObstacleTracker(self.p)
        self.sync_tol = float(cfg.get('obstacles', {}).get('sync_tolerance', 0.06))
        self.cam = None
        self.K = None
        self.depth = None                      # (stamp, array)
        self.segs = []                         # [(stamp, array)] newest last
        self.poses = []                        # [(stamp, Pose)] newest last, ~2 s
        self.last_depth_t = -1.0
        self.ms = 0.0
        self.frames = 0
        self.seg_frames = 0
        s = qos_profile_sensor_data
        self.create_subscription(Image, '/camera/depth_image', self.on_depth, s)
        self.create_subscription(Image, '/camera/segmentation/labels_map', self.on_seg, s)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, s)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, s)
        self.pub = self.create_publisher(String, '/perception/obstacles', 10)
        rate = float(cfg.get('obstacles', {}).get('rate', 10.0))
        self.create_timer(1.0 / max(1.0, rate), self.step)
        self.get_logger().info(
            f'obstacle_tracker: range {self.p.tracking_range} m, safety human {self.p.human_safety_distance} / vehicle '
            f'{self.p.vehicle_safety_distance} / dynamic {self.p.dynamic_obstacle_distance} / static '
            f'{self.p.static_obstacle_distance} m, prediction {self.p.collision_prediction_time} s')

    # ------------------------------------------------------------ inputs
    def on_depth(self, m):
        try:
            self.depth = (stamp_s(m.header), image_to_numpy(m))
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

    def on_seg(self, m):
        try:
            a = image_to_numpy(m)
        except ValueError:
            return
        self.segs.append((stamp_s(m.header), a[..., 0] if a.ndim == 3 else a))
        del self.segs[:-6]

    def on_info(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = list(m.k)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.poses.append((stamp_s(m.header), Pose(x=p.x, y=p.y, z=p.z, R=quat_to_matrix(q.x, q.y, q.z, q.w))))
        if len(self.poses) > 400:
            del self.poses[:-300]

    def pose_at(self, t):
        """Rover pose at the image stamp: linear interpolation of position, nearest rotation."""
        if not self.poses:
            return None
        ts = [s for s, _ in self.poses]
        k = bisect.bisect_left(ts, t)
        if k <= 0:
            return self.poses[0][1]
        if k >= len(ts):
            return self.poses[-1][1]
        (t0, a), (t1, b) = self.poses[k - 1], self.poses[k]
        f = (t - t0) / max(1e-9, t1 - t0)
        return Pose(x=a.x + f * (b.x - a.x), y=a.y + f * (b.y - a.y), z=a.z + f * (b.z - a.z),
                    R=(a.R if f < 0.5 else b.R))

    # ------------------------------------------------------------ loop
    def step(self):
        if self.depth is None or not self.poses:
            return
        t_img, depth = self.depth
        if t_img <= self.last_depth_t:
            return                                        # no new frame
        self.last_depth_t = t_img
        w0 = time.perf_counter()
        pose = self.pose_at(t_img)
        if self.cam is None or self.cam.w != depth.shape[1]:
            self.cam = CameraModel(self.tp, K=self.K, width=depth.shape[1], height=depth.shape[0])
        seg = None
        if self.segs:
            ts, arr = min(self.segs, key=lambda s: abs(s[0] - t_img))
            if abs(ts - t_img) <= self.sync_tol and arr.shape[:2] == depth.shape[:2]:
                seg = arr
                self.seg_frames += 1
        pts, valid, _, _, _ = self.cam.backproject(depth, pose, self.tp)
        xyz, labels = extract_points(pts, valid, seg, pose, self.p, ground_z=pose.ground_z(self.tp))
        dets = cluster(xyz, labels, self.p)
        tracks = self.tracker.update(t_img, dets)
        self.frames += 1
        self.ms = 0.8 * self.ms + 0.2 * (time.perf_counter() - w0) * 1e3 if self.ms else (time.perf_counter() - w0) * 1e3
        items = [describe(tr, pose, self.p, t_img) for tr in tracks]
        items.sort(key=lambda d: d['distance'])
        counts = {}
        for d in items:
            counts[d['cls']] = counts.get(d['cls'], 0) + 1
        msg = dict(stamp=round(t_img, 3), now=round(node_time(self), 3), robot=[round(pose.x, 2), round(pose.y, 2)],
                   tracks=items, n=len(items), dynamic=sum(1 for d in items if d['dynamic']), counts=counts,
                   detections=len(dets), segmentation=seg is not None, processing_ms=round(self.ms, 1),
                   frames=self.frames, params=dict(
                       dynamic_obstacle_distance=self.p.dynamic_obstacle_distance,
                       human_safety_distance=self.p.human_safety_distance,
                       vehicle_safety_distance=self.p.vehicle_safety_distance,
                       static_obstacle_distance=self.p.static_obstacle_distance,
                       collision_prediction_time=self.p.collision_prediction_time,
                       robot_footprint_margin=self.p.robot_footprint_margin))
        self.pub.publish(String(data=json.dumps(msg)))


def main():
    rclpy.init()
    node = ObstacleTrackerNode()
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
