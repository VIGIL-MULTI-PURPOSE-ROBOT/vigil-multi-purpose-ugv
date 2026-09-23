#!/usr/bin/python3
"""Thermal human detection node (primary SAR sensor).

Subscribes:
  /thermal/image_raw     sensor_msgs/Image mono16 (Gazebo thermal_camera, L16 = K / 0.01)
  /thermal/camera_info   sensor_msgs/CameraInfo
  /camera/depth_image    sensor_msgs/Image 32FC1 (existing rgbd camera, used for range)
  /camera/camera_info    sensor_msgs/CameraInfo
  /sim/ground_truth      nav_msgs/Odometry (robot pose, same source as the navigator)
  /sar/humans            std_msgs/String JSON (tracker output, only for H-labels in the image)
Publishes:
  /vision/thermal_human   std_msgs/String JSON  per-frame candidates (accepted + rejected, with reason)
  /vision/thermal_overlay sensor_msgs/Image bgr8 colourised thermal image, boxes, HUMAN DETECTED banner

Humans are found ONLY from temperature + geometry (thermal_core). RGB is not used.
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Image, CameraInfo  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from ros_common import nested_params, image_to_numpy, bgr_to_image_msg, odom_to_pose, node_time  # noqa: E402
from thermal_core import ThermalParams, ThermalHumanDetector, PinholeCamera, raw_to_kelvin  # noqa: E402


class ThermalHumanDetectorNode(Node):
    def __init__(self):
        super().__init__('thermal_human_detector', automatically_declare_parameters_from_overrides=True)
        cfg = nested_params(self)
        self.cfg = cfg
        tc, hd = cfg['thermal_camera'], cfg['human_detection']
        self.enabled = bool(tc.get('enabled', True))
        self.p = ThermalParams.from_cfg(tc, hd, cfg['robot'], cfg.get('camera'))
        self.p.body_reference = float(cfg['human_thermal'].get('body_temperature', 305.15))
        self.det = ThermalHumanDetector(self.p)
        c = cfg['camera']
        self.depth_cfg = c
        self.depth_cam = None
        self.depth = None
        self.depth_t = 0.0
        self.pose = None
        self.frame = None
        self.frame_enc = ''
        self.frame_t = 0.0
        self.humans = []
        self.out_w = int(cfg['dashboard'].get('thermal_width', 640))
        self.ambient = float(cfg['world'].get('ambient_temperature_k', 293.15))
        # Sensor noise is added here, not in the sensor SDF: a <noise> element on a thermal_camera
        # segfaults the Gazebo server on gz-sensors 8 (see urdf/sar_sensors.xacro). Set
        # thermal_camera.noise_in_sdf: true only together with the xacro arg thermal_noise_sdf:=true.
        self.noise_k = 0.0 if bool(tc.get('noise_in_sdf', False)) else float(tc.get('noise_stddev', 0.0))
        if self.noise_k > 0.0:
            self.get_logger().info(f'adding {self.noise_k:.3f} K gaussian sensor noise in software')
        self.frames = 0
        self.proc_ms = 0.0
        vf = self.det.cam.vfov
        cfg_vf = float(tc.get('vertical_fov', vf))
        self.get_logger().info(
            f'thermal camera {self.p.width}x{self.p.height} hfov {math.degrees(self.p.hfov):.1f} deg -> '
            f'vfov {math.degrees(vf):.1f} deg (config {math.degrees(cfg_vf):.1f}); human band '
            f'{self.p.t_min:.2f}-{self.p.t_max:.2f} K, range {self.p.detection_range:.0f} m')
        if abs(cfg_vf - vf) > math.radians(2.0):
            self.get_logger().warn('thermal_camera.vertical_fov does not match width/height/hfov; '
                                   'Gazebo uses hfov + aspect, the config value is informational')
        s = qos_profile_sensor_data
        self.create_subscription(Image, '/thermal/image_raw', self.on_thermal, s)
        self.create_subscription(CameraInfo, '/thermal/camera_info', self.on_tinfo, s)
        self.create_subscription(Image, '/camera/depth_image', self.on_depth, s)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_dinfo, s)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, s)
        self.create_subscription(String, '/sar/humans', self.on_humans, 10)
        self.pub = self.create_publisher(String, '/vision/thermal_human', 10)
        self.pub_img = self.create_publisher(Image, '/vision/thermal_overlay', 2)
        rate = float(hd.get('processing_rate', 5.0))
        self.create_timer(1.0 / rate, self.step)
        self.tinfo_ok = False

    # ------------------------------------------------------------ inputs
    def on_thermal(self, m):
        self.frame = m
        self.frame_t = node_time(self)

    def on_tinfo(self, m):
        if not self.tinfo_ok and m.k[0] > 0:
            self.det.set_intrinsics(list(m.k), m.width, m.height)
            self.tinfo_ok = True
            self.get_logger().info(f'thermal intrinsics fx={m.k[0]:.1f} {m.width}x{m.height}')

    def on_dinfo(self, m):
        if self.depth_cam is None and m.k[0] > 0:
            c = self.depth_cfg
            self.depth_cam = PinholeCamera(m.width, m.height, float(c.get('horizontal_fov', 1.5184)),
                                           float(c.get('x', 0.4672)), float(c.get('y', 0.0)),
                                           float(c.get('z', 0.5991)), float(c.get('pitch', 0.1745)), K=list(m.k))

    def on_depth(self, m):
        try:
            self.depth = image_to_numpy(m)
            self.depth_t = node_time(self)
        except ValueError:
            pass

    def on_odom(self, m):
        self.pose = odom_to_pose(m)

    def on_humans(self, m):
        try:
            self.humans = json.loads(m.data).get('humans', [])
        except ValueError:
            pass

    # ------------------------------------------------------------ processing
    def step(self):
        if not self.enabled or self.frame is None or self.pose is None:
            return
        # simulation time: military_world runs far below real time, so a 10 Hz thermal frame can
        # be seconds of wall clock old and still be the newest one there is.
        if node_time(self) - self.frame_t > 2.0:
            return
        t0 = time.monotonic()
        m = self.frame
        try:
            raw = image_to_numpy(m, raw16=True)
            temp = raw_to_kelvin(raw, m.encoding.lower())
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=10.0)
            return
        if self.noise_k > 0.0:
            temp = temp + np.random.normal(0.0, self.noise_k, temp.shape)
        pose = self.pose
        depth = self.depth if (self.depth is not None and node_time(self) - self.depth_t < 1.0) else None
        cands, bg, band = self.det.detect(temp, pose, depth, self.depth_cam)
        accepted = [c for c in cands if c.accepted]
        now = self.get_clock().now()
        msg = dict(stamp=now.nanoseconds * 1e-9, frame=self.frames, background_k=round(bg, 2),
                   robot=dict(x=round(pose.x, 2), y=round(pose.y, 2), yaw=round(pose.yaw, 3)),
                   human_detected=bool(accepted), count=len(accepted),
                   candidates=[c.as_dict() for c in cands])
        self.pub.publish(String(data=json.dumps(msg)))
        if self.pub_img.get_subscription_count() > 0:
            img = self.render(temp, cands, bg)
            self.pub_img.publish(bgr_to_image_msg(img, now.to_msg(), 'thermal_optical_frame'))
        self.frames += 1
        dt = (time.monotonic() - t0) * 1000
        self.proc_ms = 0.8 * self.proc_ms + 0.2 * dt if self.proc_ms else dt

    # ------------------------------------------------------------ visualisation
    def render(self, temp, cands, bg):
        p = self.p
        lo, hi = self.ambient - 10.0, p.t_max + 6.0
        g = np.clip((np.nan_to_num(temp, nan=lo) - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
        img = cv2.applyColorMap(g, cv2.COLORMAP_INFERNO)
        H, W = temp.shape
        S = self.out_w / float(W)
        img = cv2.resize(img, (self.out_w, int(round(H * S))), interpolation=cv2.INTER_NEAREST)
        f = S / 2.0
        # hot but NOT human (engine, generator, warm case): outlined, labelled as rejected
        hot = (temp > p.t_max).astype(np.uint8)
        if hot.sum() > 4:
            cnts, _ = cv2.findContours(hot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < 4:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                tmax = float(temp[y:y + h, x:x + w].max())
                x, y, w, h = [int(round(v * S)) for v in (x, y, w, h)]
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 160, 255), max(1, int(f)))
                _label(img, f'HOT {tmax - 273.15:.0f}C - not human', (x, max(14, y - 4)), (0, 160, 255), f * 0.8)
        n_h = 0
        for c in cands:
            x, y, w, h = [int(round(v * S)) for v in c.bbox]
            if not c.accepted:
                cv2.rectangle(img, (x, y), (x + w, y + h), (150, 150, 150), 1)
                _label(img, f'rejected: {c.reject}', (x, y + h + int(14 * f)), (170, 170, 170), f * 0.7)
                continue
            n_h += 1
            hid = ''
            for hm in self.humans:
                if c.x is not None and math.hypot(hm['x'] - c.x, hm['y'] - c.y) < 2.5:
                    hid = hm['human_id'] + ' '
                    break
            col = (60, 60, 255) if hid else (0, 220, 255)
            cv2.rectangle(img, (x, y), (x + w, y + h), col, max(2, int(round(2 * f))))
            txt = f'{hid}HUMAN {c.confidence:.2f}  {c.distance:.1f} m  {c.mean_temp - 273.15:.1f}C'
            if not hid:
                txt = 'CANDIDATE ' + txt[6:]
            _label(img, txt, (x, max(int(16 * f), y - int(5 * f))), col, f * 0.8)
        banner = 'THERMAL: HUMAN DETECTED' if n_h else 'THERMAL: NO HUMAN DETECTED'
        bh = int(26 * f)
        img[:bh] = (img[:bh] * 0.3).astype(np.uint8)
        cv2.putText(img, banner, (int(8 * f), int(19 * f)), cv2.FONT_HERSHEY_SIMPLEX, 0.6 * f,
                    (60, 60, 255) if n_h else (180, 220, 180), max(1, int(round(2 * f))), cv2.LINE_AA)
        info = f'bg {bg - 273.15:.1f}C  band {p.t_min - 273.15:.1f}-{p.t_max - 273.15:.1f}C  {self.proc_ms:.0f} ms'
        (tw, _), _ = cv2.getTextSize(info, cv2.FONT_HERSHEY_SIMPLEX, 0.42 * f, 1)
        cv2.putText(img, info, (img.shape[1] - tw - int(8 * f), int(18 * f)), cv2.FONT_HERSHEY_SIMPLEX, 0.42 * f,
                    (220, 220, 220), 1, cv2.LINE_AA)
        return img


def _label(img, text, org, color, f=1.0):
    fs, th_ = 0.5 * f, max(1, int(round(f)))
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, th_)
    x, y = org
    pad = max(1, int(3 * f))
    cv2.rectangle(img, (x - pad, y - th - 2 * pad), (x + tw + pad, y + pad), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, fs, color, th_, cv2.LINE_AA)


def main():
    rclpy.init()
    node = ThermalHumanDetectorNode()
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
