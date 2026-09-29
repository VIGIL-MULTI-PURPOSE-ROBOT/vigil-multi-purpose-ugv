#!/usr/bin/python3
"""Small helpers shared by the vision/navigation/dashboard ROS 2 nodes."""
import math

import numpy as np

SECTIONS = ('robot', 'camera', 'terrain', 'navigation', 'dashboard')


def nested_params(node):
    """Read the nested vision_nav.yaml sections into plain dicts."""
    cfg = {}
    for s in SECTIONS:
        cfg[s] = {k: v.value for k, v in node.get_parameters_by_prefix(s).items()}
    return cfg


def image_to_numpy(msg):
    """sensor_msgs/Image -> numpy without cv_bridge (keeps dependencies minimal)."""
    enc = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in ('32fc1',):
        arr = buf.view(np.float32).reshape(h, step // 4)[:, :w]
        return arr.byteswap() if msg.is_bigendian else arr
    if enc in ('16uc1', 'mono16'):
        return buf.view(np.uint16).reshape(h, step // 2)[:, :w].astype(np.float32) / 1000.0
    if enc in ('rgb8', 'bgr8'):
        arr = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return arr[..., ::-1].copy() if enc == 'rgb8' else arr.copy()   # always return BGR
    if enc in ('rgba8', 'bgra8'):
        arr = buf.reshape(h, step)[:, :w * 4].reshape(h, w, 4)[..., :3]
        return arr[..., ::-1].copy() if enc == 'rgba8' else arr.copy()
    if enc in ('mono8', '8uc1'):
        return buf.reshape(h, step)[:, :w].copy()
    raise ValueError(f'unsupported encoding {msg.encoding}')


def bgr_to_image_msg(img, stamp, frame_id):
    from sensor_msgs.msg import Image
    m = Image()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.height, m.width = img.shape[:2]
    m.encoding = 'bgr8'
    m.is_bigendian = 0
    m.step = m.width * 3
    m.data = np.ascontiguousarray(img).tobytes()
    return m


def odom_to_pose(msg):
    from terrain_core import Pose, quat_to_matrix
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return Pose(x=p.x, y=p.y, z=p.z, R=quat_to_matrix(q.x, q.y, q.z, q.w))


def grid_msg(values_int8, p, stamp, frame='world'):
    from nav_msgs.msg import OccupancyGrid
    g = OccupancyGrid()
    g.header.stamp = stamp
    g.header.frame_id = frame
    g.info.map_load_time = stamp
    g.info.resolution = float(p.resolution)
    n = values_int8.shape[0]
    g.info.width = g.info.height = n
    g.info.origin.position.x = float(p.map_origin_x)
    g.info.origin.position.y = float(p.map_origin_y)
    g.info.origin.orientation.w = 1.0
    g.data = values_int8.astype(np.int8).ravel().tolist()
    return g


def grid_to_numpy(msg):
    n_w, n_h = msg.info.width, msg.info.height
    return np.array(msg.data, dtype=np.int16).reshape(n_h, n_w)


def path_msg(path, stamp, frame='world'):
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    m = Path()
    m.header.stamp = stamp
    m.header.frame_id = frame
    if path is None:
        return m
    for k, (x, y) in enumerate(path):
        ps = PoseStamped()
        ps.header = m.header
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        k2 = min(len(path) - 1, k + 1)
        k1 = max(0, k2 - 1)
        yaw = math.atan2(path[k2][1] - path[k1][1], path[k2][0] - path[k1][0])
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.poses.append(ps)
    return m


def path_from_msg(msg):
    if not msg.poses:
        return None
    return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
