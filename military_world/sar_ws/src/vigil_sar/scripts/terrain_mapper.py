#!/usr/bin/python3
"""Vision node: RGB + depth (+ segmentation) + ground-truth pose -> terrain classes,
cliff detection, traversability grid and the camera overlay.

Copied from vigil_rough_terrain. vigil_sar changes (behaviour of the detection is unchanged):
  * WindowedTerrainMapper: only the area around the camera is re-classified per frame
    (the military_world map is 170 m; full-map analysis took 1.1 s per frame)
  * the four grids are published at terrain.grid_publish_rate (2.9 M cells each)
  * the 4K display frame is decoded only when the overlay is drawn, and the overlay is
    drawn at rgb_camera.overlay_width (the 4K stream itself stays on /camera/hd/image)

Subscribes (existing topics, unchanged):
  /camera/depth_image  sensor_msgs/Image 32FC1   (rgbd_camera)
  /camera/image        sensor_msgs/Image rgb8
  /camera/camera_info  sensor_msgs/CameraInfo
  /sim/ground_truth    nav_msgs/Odometry         (base_footprint pose)
  /camera/segmentation/labels_map  sensor_msgs/Image (optional, vision_sensors:=true)
  /camera/hd/image     sensor_msgs/Image rgb8    (optional HD display camera, used for the overlay)
  /navigation/path, /navigation/previous_path, /navigation/status (for the overlay)
Publishes (new):
  /vision/terrain_classes  nav_msgs/OccupancyGrid   class id per cell (see terrain_core)
  /vision/traversability   nav_msgs/OccupancyGrid   0-100 cost, -1 unknown (RViz/Nav2)
  /vision/slope            nav_msgs/OccupancyGrid   footprint-scale slope, whole degrees
  /vision/roughness        nav_msgs/OccupancyGrid   roughness around the local plane, cm
  /vision/terrain          std_msgs/String (JSON)   corridor summary ahead of the robot
  /vision/cliff            std_msgs/String (JSON)   cliff detection summary
  /vision/overlay          sensor_msgs/Image bgr8   annotated camera view
"""
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
from nav_msgs.msg import Odometry, OccupancyGrid, Path  # noqa: E402
from sensor_msgs.msg import Image, CameraInfo  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from terrain_core import Params, CameraModel, CLIFF  # noqa: E402
from terrain_window import WindowedTerrainMapper  # noqa: E402
import vision_overlay  # noqa: E402
from ros_common import (nested_params, image_to_numpy, bgr_to_image_msg, odom_to_pose,  # noqa: E402
                        grid_msg, path_from_msg, node_time)


class TerrainMapperNode(Node):
    def __init__(self):
        super().__init__('terrain_mapper', automatically_declare_parameters_from_overrides=True)
        cfg = nested_params(self)
        self.p = Params.from_nested(cfg)
        for line in self.p.sanity_report():
            self.get_logger().info(line)
        self.mapper = WindowedTerrainMapper(self.p, window=float(cfg['terrain'].get('analysis_window', 12.0)))
        self.overlay_width = int(cfg['rgb_camera'].get('overlay_width', 1920))
        self.grid_period = 1.0 / max(0.1, float(cfg['terrain'].get('grid_publish_rate', 2.0)))
        self.last_grid = -1e9
        self.rgb_hd_msg = None
        self.cam = None
        self.K = None
        self.depth = self.rgb = self.seg = None
        self.rgb_hd, self.rgb_hd_t = None, 0.0
        self.pose = None
        self.path = self.prev_path = None
        self.nav_status = {}
        self.last_process = -1.0
        self.processing_ms = 0.0
        s = qos_profile_sensor_data
        self.create_subscription(Image, '/camera/depth_image', self.on_depth, s)
        self.create_subscription(Image, '/camera/image', self.on_rgb, s)
        self.create_subscription(Image, '/camera/hd/image', self.on_rgb_hd, s)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, s)
        self.create_subscription(Image, '/camera/segmentation/labels_map', self.on_seg, s)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, s)
        self.create_subscription(Path, '/navigation/path', lambda m: setattr(self, 'path', path_from_msg(m)), 10)
        self.create_subscription(Path, '/navigation/previous_path',
                                 lambda m: setattr(self, 'prev_path', path_from_msg(m)), 10)
        self.create_subscription(String, '/navigation/status', self.on_status, 10)
        self.pub_cls = self.create_publisher(OccupancyGrid, '/vision/terrain_classes', 2)
        self.pub_cost = self.create_publisher(OccupancyGrid, '/vision/traversability', 2)
        self.pub_slope = self.create_publisher(OccupancyGrid, '/vision/slope', 2)
        self.pub_rough = self.create_publisher(OccupancyGrid, '/vision/roughness', 2)
        self.pub_ter = self.create_publisher(String, '/vision/terrain', 10)
        self.pub_cliff = self.create_publisher(String, '/vision/cliff', 10)
        self.pub_overlay = self.create_publisher(Image, '/vision/overlay', 2)
        rate = float(cfg['terrain'].get('publish_rate', 5.0))
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info('terrain_mapper ready: waiting for /camera/depth_image and /sim/ground_truth')

    def on_depth(self, m):
        try:
            self.depth = image_to_numpy(m)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

    def on_rgb(self, m):
        try:
            self.rgb = image_to_numpy(m)
        except ValueError:
            pass

    def on_rgb_hd(self, m):
        # keep the message; a 4K frame (25 MB) is only decoded when the overlay is drawn
        self.rgb_hd_msg = m
        self.rgb_hd_t = node_time(self)

    def on_seg(self, m):
        try:
            a = image_to_numpy(m)
            self.seg = a[..., 0] if a.ndim == 3 else a   # semantic label id
        except ValueError:
            pass

    def on_info(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = list(m.k)
            self.cam = CameraModel(self.p, K=self.K, width=m.width, height=m.height)
            self.get_logger().info(f'camera intrinsics fx={m.k[0]:.1f} {m.width}x{m.height}')

    def on_odom(self, m):
        self.pose = odom_to_pose(m)

    def on_status(self, m):
        try:
            self.nav_status = json.loads(m.data)
        except ValueError:
            pass

    def step(self):
        if self.depth is None or self.pose is None:
            return
        if self.cam is None or self.cam.w != self.depth.shape[1]:
            self.cam = CameraModel(self.p, K=self.K, width=self.depth.shape[1], height=self.depth.shape[0])
        t0 = self.get_clock().now()
        depth, pose = self.depth, self.pose
        frame = self.mapper.process(depth, pose, self.cam, seg=self.seg)
        stamp = self.get_clock().now().to_msg()
        now = node_time(self)          # simulation time: grid_publish_rate is a sim-time rate
        if now - self.last_grid >= self.grid_period:
            self.last_grid = now
            cls = self.mapper.classes.astype(np.int8)
            # slope / roughness first, so the navigator has them when the class grid arrives
            self.pub_slope.publish(grid_msg(np.clip(np.round(self.mapper.slope_deg), 0, 90).astype(np.int8),
                                            self.p, stamp))
            self.pub_rough.publish(grid_msg(np.clip(np.round(self.mapper.roughness * 100), 0, 100).astype(np.int8),
                                            self.p, stamp))
            self.pub_cls.publish(grid_msg(cls, self.p, stamp))
            self.pub_cost.publish(grid_msg(self.mapper.cost_grid(), self.p, stamp))
        ter = self.mapper.status_ahead(pose)
        ter['processing_ms'] = round(self.processing_ms, 1)
        self.pub_ter.publish(String(data=json.dumps(ter)))
        cliff_mask = self.mapper.classes == CLIFF
        cliff = dict(detected=bool(ter['blocking'] == 'HIGH CLIFF'), cliff_cells=int(cliff_mask.sum()),
                     drop=ter['drop'], distance=ter['safe_distance'] if ter['blocking'] == 'HIGH CLIFF' else None,
                     max_safe_drop=self.p.max_safe_drop)
        self.pub_cliff.publish(String(data=json.dumps(cliff)))
        if self.pub_overlay.get_subscription_count() > 0:
            target = None
            if self.path is not None and len(self.path) > 1:
                d = np.hypot(self.path[:, 0] - pose.x, self.path[:, 1] - pose.y)
                k = min(len(self.path) - 1, int(np.argmin(d)) + 8)
                target = self.path[k]
            # HD display camera if it is streaming, otherwise the 320x240 rgbd colour image
            rgb = self.rgb
            if self.rgb_hd_msg is not None and node_time(self) - self.rgb_hd_t < 2.0:
                try:
                    rgb = image_to_numpy(self.rgb_hd_msg)
                except ValueError:
                    pass
            img = vision_overlay.draw(rgb, frame, self.mapper, pose, self.cam, self.path, self.prev_path,
                                      self.nav_status, ter, target, out_width=self.overlay_width)
            self.pub_overlay.publish(bgr_to_image_msg(img, stamp, 'camera_optical_frame'))
        dt = (self.get_clock().now() - t0).nanoseconds * 1e-6
        self.processing_ms = 0.8 * self.processing_ms + 0.2 * dt if self.processing_ms else dt


def main():
    rclpy.init()
    node = TerrainMapperNode()
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
