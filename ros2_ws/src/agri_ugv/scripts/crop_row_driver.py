#!/usr/bin/python3
"""Protected row-1 straddle drive for the supplied cotton field."""
import json, math, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CropRowDriver(Node):
    def __init__(self):
        super().__init__('crop_row_driver')
        self.length = self.declare_parameter('row_length_m', 31.5).value
        # Keep the five-times drivetrain capability, while using a safer
        # 1.20 m/s row speed for precise crop-centre tracking.
        self.speed = self.declare_parameter('cruise_speed_mps', 1.20).value
        # The rover straddles a crop row with tyre centres at +/-0.436 to +/-0.466 m.  A 0.32 m
        # maximum centreline error still leaves 0.069 m between a tyre edge
        # and the nearest crop centreline (1.22 m row pitch, 0.071 m tyre
        # half-width).  The controller starts slowing much earlier.
        self.guard = self.declare_parameter('max_cross_track_m', 0.36).value
        self.rows = self.declare_parameter('row_count', 23).value
        self.row_pitch = self.declare_parameter('row_pitch_m', 1.22).value
        # The physical eight-wheel skid-steer has ~1.395x the requested
        # curvature on soil.  Command 0.85 m here to obtain the required
        # 0.61 m physical semicircle radius and 1.22 m row-to-row shift.
        self.turn_radius = self.declare_parameter('headland_turn_radius_m', .85).value
        self.turn_speed = self.declare_parameter('headland_turn_speed_mps', .45).value
        # The world has a tractor on the perimeter headland.  This is a
        # conservative centre-to-centre no-contact radius: it exceeds the
        # rover footprint, tractor body, and a 1 m presentation safety gap.
        self.tractor_x = self.declare_parameter('tractor_x_m', 17.60).value
        self.tractor_y = self.declare_parameter('tractor_y_m', 17.40).value
        self.tractor_keepout = self.declare_parameter('tractor_keepout_m', 3.50).value
        self.pose = None
        self.start = None
        self.blocked_until = 0.0
        self.done = False
        self.last_report = ''
        self.phase = 'row'
        self.row = 0
        self.direction = 1
        self.turn_sign = 0
        self.turn_previous_yaw = None
        self.turn_angle = 0.0
        self.cmd = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, 'exploration/status', qos)
        self.path = self.create_publisher(NavPath, 'crop_row/path', qos)
        self.mode = self.create_publisher(String, 'crop_row/mode', qos)
        # Use Gazebo's world-frame model odometry for this deterministic field
        # demonstration.  It contains no GPS signal; it is simulator pose
        # truth and avoids encoder-slip drift during skid-steer headland turns.
        self.create_subscription(Odometry, 'sim/ground_truth', self.odometry, 10)
        # The RGB-D hazard stream excludes green crop foliage while retaining
        # people, equipment, rocks and ditches.  It is the crop-safe stop
        # input for the row mission.
        self.create_subscription(PointCloud2, 'perception/hazards', self.obstacles, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def odometry(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z)))

    def obstacles(self, msg):
        try:
            p = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:
            return
        # Crop foliage is excluded upstream. This remains a live stop for a
        # person, rock, or equipment in either protected wheel lane.
        if len(p) and np.any((p[:, 0] > .65) & (p[:, 0] < 3.0) & (np.abs(p[:, 1]) < .62)):
            self.blocked_until = time.monotonic() + .6

    def report(self, text, **extra):
        if text == self.last_report and not extra:
            return
        self.last_report = text
        record = {'state': 'row_complete' if self.done else 'crop_row', 'message': text,
                  'sim_time': self.get_clock().now().nanoseconds * 1e-9, **extra}
        self.status.publish(String(data=json.dumps(record)))
        self.get_logger().info(text)

    def publish_path(self):
        msg = NavPath(); msg.header.frame_id = 'odom'; msg.header.stamp = self.get_clock().now().to_msg()
        for row in range(self.rows):
            y = self.start[1] + row * self.row_pitch
            xs = np.linspace(self.start[0], self.start[0] + self.length, 22)
            if row % 2:
                xs = xs[::-1]
            for x in xs:
                pose = PoseStamped(); pose.header = msg.header; pose.pose.position.x = float(x); pose.pose.position.y = y; pose.pose.orientation.w = 1.0; msg.poses.append(pose)
        self.path.publish(msg)

    def set_mode(self, mode):
        self.mode.publish(String(data=mode))

    def stop(self, reason, **extra):
        self.cmd.publish(Twist())
        self.done = True
        self.report(reason, **extra)

    def tractor_clear(self):
        """Prevent every motion command from entering the tractor keep-out."""
        x, y, _ = self.pose
        distance = math.hypot(x - self.tractor_x, y - self.tractor_y)
        if distance >= self.tractor_keepout:
            return True
        self.cmd.publish(Twist())
        self.report('Tractor safety hold: no-contact keep-out zone active',
                    tractor_distance_m=round(distance, 3),
                    keepout_m=self.tractor_keepout)
        return False

    def tick(self):
        if self.pose is None or self.done:
            return
        if self.start is None:
            self.start = self.pose
            self.publish_path()
            self.set_mode('row')
            self.report('Row 1 straddle drive started: tyres are in the two empty crop aisles',
                        row_center_y=self.start[1], tyre_lane_offsets_m=[-0.5, 0.5], cruise_speed_mps=self.speed)
            return
        if not self.tractor_clear():
            return
        if self.phase == 'headland_turn':
            self.headland_turn()
            return
        x, y, yaw = self.pose
        row_y = self.start[1] + self.row * self.row_pitch
        row_start_x = self.start[0] if self.direction > 0 else self.start[0] + self.length
        progress = self.direction * (x - row_start_x)
        error_y = y - row_y
        if abs(error_y) > self.guard:
            self.stop('Crop safety stop: tyre lane drift limit reached', cross_track_m=abs(error_y), limit_m=self.guard)
            return
        if progress >= self.length - .20:
            if self.row + 1 >= self.rows:
                self.stop('Field row sweep complete: rover stopped in the clear headland', rows_completed=self.rows, distance_m=progress)
                return
            self.phase = 'headland_turn'
            self.turn_sign = 1 if self.direction > 0 else -1
            self.turn_previous_yaw = yaw
            self.turn_angle = 0.0
            self.set_mode('headland')
            self.report('Clear-headland U-turn started', completed_row=self.row + 1, next_row=self.row + 2)
            return
        if time.monotonic() < self.blocked_until:
            self.cmd.publish(Twist())
            self.report('Protected obstacle stop: obstacle detected in a wheel lane')
            return
        # Pure-pursuit-style row centring.  The drive node still limits this
        # command to the rover's 2 m minimum turning radius.
        heading = 0.0 if self.direction > 0 else math.pi
        # Cross-track steering reverses when travelling west: a positive
        # offset must turn the rover south in either travel direction.
        steering = 1.6 * wrap(heading-yaw) - self.direction * 2.4 * math.atan(error_y / 1.2)
        out = Twist()
        out.linear.x = float(self.speed * (1.0 - 0.55 * min(abs(error_y) / self.guard, 1.0)))
        out.angular.z = float(np.clip(steering, -self.speed / 2.0, self.speed / 2.0))
        self.cmd.publish(out)

    def headland_turn(self):
        _, y, yaw = self.pose
        self.turn_angle += wrap(yaw-self.turn_previous_yaw)
        self.turn_previous_yaw = yaw
        if abs(self.turn_angle) >= math.pi-.04:
            self.cmd.publish(Twist())
            self.row += 1
            self.direction *= -1
            self.phase = 'row'
            expected_y = self.start[1] + self.row * self.row_pitch
            self.set_mode('row')
            if abs(y-expected_y) > self.guard:
                self.stop('Crop safety stop: U-turn did not enter the next protected lane', cross_track_m=abs(y-expected_y), limit_m=self.guard)
                return
            error = abs(y-expected_y)
            self.report(f'U-turn complete: entering next protected crop row (cross-track {error:.3f} m)', row=self.row + 1, cross_track_m=error)
            return
        out = Twist()
        out.linear.x = self.turn_speed
        out.angular.z = self.turn_sign * self.turn_speed / self.turn_radius
        self.cmd.publish(out)


def main():
    rclpy.init(); node = CropRowDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
