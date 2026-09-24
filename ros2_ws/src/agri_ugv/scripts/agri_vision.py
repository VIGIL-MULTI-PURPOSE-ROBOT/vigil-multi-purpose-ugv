#!/usr/bin/python3
"""Agriculture perception node (observe only - it never publishes a motion command).

In : /camera/image, /camera/depth_image, /camera/camera_info   (existing RGB-D camera, unchanged)
     /sim/ground_truth                                          (existing robot pose, as the row driver uses)
     TF base_footprint -> camera_optical_frame                  (robot_state_publisher)
Out: /agri_vision/overlay     sensor_msgs/Image  camera + CROP / WEED / HUMAN / UNKNOWN / object overlay
     /agri_vision/detections  std_msgs/String    JSON, this frame's detections
     /agri_vision/tracks      std_msgs/String    JSON, de-duplicated counts, IDs, world positions (1 Hz)
     /agri_vision/events      std_msgs/String    JSON, one message per event (WEED W1 DETECTED, HUMAN H1 ...)
Logic: agri_vision_core.py.  Settings: config/agriculture.yaml (one file for the whole agriculture project).
"""
import bisect
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from message_filters import ApproximateTimeSynchronizer, Subscriber  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

from agri_vision_core import (CROP, HUMAN, OTHER, UNKNOWN, WEED, Detector, Frame, Tracker,  # noqa: E402
                              default_base_to_optical, draw_overlay, load_config, project_world_line,
                              quat_to_matrix, robot_matrix)


def image_to_numpy(msg):
    enc = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    if enc == '32fc1':
        arr = buf.view(np.float32).reshape(h, step // 4)[:, :w]
        return arr.byteswap() if msg.is_bigendian else arr
    if enc in ('16uc1', 'mono16'):
        return buf.view(np.uint16).reshape(h, step // 2)[:, :w].astype(np.float32) / 1000.0
    if enc in ('rgb8', 'bgr8'):
        arr = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return arr[..., ::-1].copy() if enc == 'rgb8' else arr.copy()
    if enc in ('rgba8', 'bgra8'):
        arr = buf.reshape(h, step)[:, :w * 4].reshape(h, w, 4)[..., :3]
        return arr[..., ::-1].copy() if enc == 'rgba8' else arr.copy()
    raise ValueError(f'unsupported encoding {msg.encoding}')


def bgr_msg(img, header):
    m = Image()
    m.header = header
    m.height, m.width = img.shape[:2]
    m.encoding, m.is_bigendian, m.step = 'bgr8', 0, img.shape[1] * 3
    m.data = np.ascontiguousarray(img).tobytes()
    return m


def load_yaml(path):
    try:
        return yaml.safe_load(Path(path).read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}


def default_config_path():
    here = Path(os.path.realpath(__file__)).parent
    cands = [here.parent / 'config/agriculture.yaml', here.parent / 'config/agri_vision.yaml']
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('agri_ugv'))
        cands += [share / 'config/agriculture.yaml', share / 'config/agri_vision.yaml']
    except Exception:
        pass
    return next((str(c) for c in cands if c.exists()), '')


def agri_config(path):
    """agriculture.yaml -> perception config; the crop rows come from the same field definition
    (and row detection) the row mission uses."""
    data = load_yaml(path) if path else {}
    cfg = load_config(data)
    if 'row_spacing' in data.get('field', {}) or 'auto_row_detection' in data.get('field', {}):
        from agri_mission_core import FieldGeometry, package_root
        cfg['field'] = dict(cfg['field'], **FieldGeometry(data, package_root(path)).as_vision_field())
    return cfg


class AgriVision(Node):
    def __init__(self):
        super().__init__('agri_vision')
        path = self.declare_parameter('config', default_config_path()).value
        self.cfg = agri_config(path)
        self.get_logger().info(f'config: {path or "(defaults)"}')
        self.det = Detector(self.cfg)
        self.trk = Tracker(self.cfg)
        self.period = 1.0 / float(self.cfg['performance']['process_rate_hz'])
        self.last_proc = 0.0
        self.odom = []                          # (t, x, y, z, R)
        self.T_base_cam = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_overlay = self.create_publisher(Image, 'agri_vision/overlay', 2)
        self.pub_det = self.create_publisher(String, 'agri_vision/detections', 5)
        self.pub_tracks = self.create_publisher(String, 'agri_vision/tracks', latched)
        self.pub_events = self.create_publisher(String, 'agri_vision/events', 50)
        self.create_subscription(Odometry, 'sim/ground_truth', self.on_odom, qos_profile_sensor_data)
        subs = [Subscriber(self, Image, 'camera/image', qos_profile=qos_profile_sensor_data),
                Subscriber(self, Image, 'camera/depth_image', qos_profile=qos_profile_sensor_data),
                Subscriber(self, CameraInfo, 'camera/camera_info', qos_profile=qos_profile_sensor_data)]
        self.sync = ApproximateTimeSynchronizer(subs, 10, 0.05)
        self.sync.registerCallback(self.on_rgbd)
        self.create_timer(1.0, self.publish_tracks)
        self.frames, self.ms = 0, 0.0

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
        return None if abs(ts[i] - t) > 0.25 else self.odom[i]

    def base_to_cam(self, frame_id):
        if self.T_base_cam is not None:
            return self.T_base_cam
        try:
            tr = self.tf.lookup_transform('base_footprint', frame_id, rclpy.time.Time()).transform
            T = np.eye(4)
            T[:3, :3] = quat_to_matrix(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
            T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
            self.T_base_cam = T
            self.get_logger().info(f'camera pose from TF: {np.round(T[:3, 3], 3).tolist()}')
        except Exception:
            return default_base_to_optical(self.cfg['camera'])
        return self.T_base_cam

    # ------------------------------------------------------------------ processing
    def on_rgbd(self, rgb, depth, info):
        now = time.monotonic()
        if now - self.last_proc < self.period:
            return
        self.last_proc = now
        t = rgb.header.stamp.sec + rgb.header.stamp.nanosec * 1e-9
        pose = self.pose_at(t)
        try:
            bgr, d = image_to_numpy(rgb), image_to_numpy(depth)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=10.0)
            return
        if pose is None:
            # no pose -> no world positions -> nothing is classified (never a guessed weed)
            self.pub_overlay.publish(bgr_msg(draw_overlay(bgr, [], dict(band=np.zeros(d.shape, bool),
                                     guard=np.zeros(d.shape, bool)), self.cfg['performance']['overlay_width'],
                                     dict(status_line='waiting for /sim/ground_truth pose')), rgb.header))
            return
        _, x, y, z, R = pose
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        T = robot_matrix(x, y, z, R) @ self.base_to_cam(depth.header.frame_id or info.header.frame_id)
        K = np.array(info.k, float).reshape(3, 3)
        fr = Frame(bgr, d, K, T, (x, y, yaw), t, robot_z=z)
        t0 = time.perf_counter()
        dets, masks, stats = self.det.process(fr)
        events = self.trk.update(dets, t)
        self.ms = 0.8 * self.ms + 0.2 * (time.perf_counter() - t0) * 1000
        self.frames += 1
        for e in events:
            self.pub_events.publish(String(data=json.dumps(e)))
            if not e['quiet']:
                self.get_logger().info(f"{e['msg']} at ({e['x']}, {e['y']})")
        # label each box with its track id when it has one
        for dd in dets:
            tr = self.trk._match(dd)
            if tr is not None and tr.id and tr.cls in (WEED, HUMAN, OTHER):
                dd['label'] = f'{tr.cls if tr.cls != OTHER else tr.sub.upper()} {tr.id}'
                dd['track'] = tr.id
        counts, items, _ = self.trk.summary(t, self.cfg['human']['active_seconds'])
        f = self.det.field
        k = int(f.row_index(y))
        lines = []
        if abs(np.cos(yaw)) > 0.5:                       # along a row (not in a headland turn)
            xs = x + np.sign(np.cos(yaw)) * np.linspace(0.6, 14.0, 40)
            g = self.det.ground
            for kk in (k - 1, k, k + 1):
                if 0 <= kk < f.n:
                    ry = float(f.row_y(kk))
                    uv = project_world_line(T, K, [(xx, ry, g[0] * xx + g[1] * ry + g[2]) for xx in xs], bgr.shape)
                    if uv:
                        lines.append((uv, (80, 230, 90) if kk == k else (60, 140, 60)))
        info_d = dict(row_lines=lines, show_crop_boxes=True,
                      human_active=any(i['cls'] == HUMAN and i['active'] for i in items),
                      weed_now=any(dd['cls'] == WEED and dd.get('track') for dd in dets),
                      status_line=(f"ROW {k + 1}/{f.n}  CROP {counts[CROP]}  WEED {counts[WEED]}  "
                                   f"HUMAN {counts[HUMAN]}  UNKNOWN {counts[UNKNOWN]}  OBJ {counts[OTHER]}  "
                                   f"{self.ms:.0f} ms"))
        over = draw_overlay(bgr, dets, masks, self.cfg['performance']['overlay_width'], info_d)
        self.pub_overlay.publish(bgr_msg(over, rgb.header))
        self.pub_det.publish(String(data=json.dumps(dict(t=t, robot=[round(x, 2), round(y, 2), round(yaw, 3)],
                                                         detections=dets, stats=stats, ms=round(self.ms, 1)))))

    def publish_tracks(self):
        if not self.odom:
            return
        t = self.odom[-1][0]
        counts, items, crops = self.trk.summary(t, self.cfg['human']['active_seconds'])
        self.pub_tracks.publish(String(data=json.dumps(dict(
            t=t, counts=counts, items=items[-500:], crops=crops[-6000:], frames=self.frames,
            ms=round(self.ms, 1), crop_model=None if self.det.crop_chroma is None else dict(
                height=round(float(self.det.crop_height), 3), samples=self.det.crop_samples)))))


def main():
    rclpy.init()
    node = AgriVision()
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
