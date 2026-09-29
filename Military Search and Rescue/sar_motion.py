"""Portable waypoint generation shared by the Blender exporter and tests."""
import math
import xml.etree.ElementTree as ET

PLUGIN = "sar::WaypointSystem"


def timed_route(points, speed, dwell=0.0):
    """Closed route with continuous loop poses; all distances are in metres."""
    points = [tuple(map(float, p)) for p in points]
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("Route speed must be positive and finite")
    if not math.isfinite(dwell) or dwell < 0:
        raise ValueError("Route dwell must be finite and nonnegative")
    if any(len(p) != 3 or not all(map(math.isfinite, p)) for p in points):
        raise ValueError("Waypoints must contain finite XYZ coordinates")
    # Adjacent duplicates would produce repeated timestamps.
    points = [p for i, p in enumerate(points) if i == 0 or p != points[i - 1]]
    if len(points) < 2:
        raise ValueError("Route needs at least two distinct positions")
    if points[-1] != points[0]:
        points.append(points[0])
    poses = []
    for p, q in zip(points, points[1:]):
        yaw = math.atan2(q[1] - p[1], q[0] - p[0]) - math.pi / 2
        poses.append((*p, 0.0, 0.0, yaw))  # Blender people face local +Y.
    poses.append(poses[0])
    keys, time = [], 0.0
    for i, pose in enumerate(poses[:-1]):
        keys.append((time, pose))
        if dwell:
            time += dwell
            keys.append((time, pose))
        time += math.dist(points[i], points[i + 1]) / speed
    keys.append((time, poses[-1]))
    return keys


def plugin_element(keys, offset=(0, 0, 0)):
    plugin = ET.Element("plugin", filename="libsar-waypoint-system.so", name=PLUGIN)
    ET.SubElement(plugin, "offset").text = " ".join(map(str, offset))
    for time, pose in keys:
        waypoint = ET.SubElement(plugin, "waypoint")
        ET.SubElement(waypoint, "time").text = f"{time:.9f}"
        ET.SubElement(waypoint, "pose").text = " ".join(f"{v:.9f}" for v in pose)
    return plugin
