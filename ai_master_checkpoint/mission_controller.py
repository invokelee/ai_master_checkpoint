"""Mission controller: drive to the target, save an arrival image and a report."""

import datetime
import getpass
import math
import os
import platform
import socket
import sys
import time

import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from .navigation_algorithm import FastObstacleNavigator, normalize_angle

TARGET_X = -5.62021446228027
TARGET_Y = -3.63326478004456

PACKAGE_NAME = 'ai_master_checkpoint'


def find_package_dir():
    """Locate the source package directory (the one containing package.xml)."""
    roots = [os.getcwd(), os.path.expanduser('~/ros2_ws')]
    for root in roots:
        for candidate in (
            root,
            os.path.join(root, PACKAGE_NAME),
            os.path.join(root, 'src', PACKAGE_NAME),
        ):
            if os.path.isfile(os.path.join(candidate, 'package.xml')) and \
                    os.path.isfile(os.path.join(candidate, 'setup.py')):
                return candidate
    return os.path.expanduser(f'~/ros2_ws/src/{PACKAGE_NAME}')


def yaw_from_quaternion(q):
    """Extract yaw (rad) from a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def image_to_bgr(msg):
    """Convert a sensor_msgs/Image to a BGR numpy array without cv_bridge."""
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}
    enc = msg.encoding
    if enc not in channels:
        raise ValueError(f'unsupported image encoding: {enc}')
    c = channels[enc]
    data = np.frombuffer(msg.data, dtype=np.uint8)
    img = data.reshape(msg.height, msg.step)[:, :msg.width * c]
    img = img.reshape(msg.height, msg.width, c)
    if enc == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if enc == 'mono8':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img.copy()


class MissionController(Node):
    """Navigate to the target using /odom and /scan, then save an image."""

    # (name, min_angle_deg, max_angle_deg), angles positive to the left.
    SECTORS = (
        ('front', -15.0, 15.0),
        ('front_left', 15.0, 60.0),
        ('front_right', -60.0, -15.0),
        ('left', 60.0, 120.0),
        ('right', -120.0, -60.0),
    )

    def __init__(self):
        super().__init__('mission_controller')

        self.declare_parameter('target_x', TARGET_X)
        self.declare_parameter('target_y', TARGET_Y)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('package_dir', '')

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        package_dir = self.get_parameter('package_dir').value or find_package_dir()
        self.report_path = os.path.join(package_dir, 'checkpoint_report.txt')
        self.image_path = os.path.join(
            package_dir, 'checkpoint_result', 'arrival_image.png')

        self.navigator = FastObstacleNavigator()
        self.odom = None
        self.sectors = None
        self.latest_image = None

        self.counts = {'/scan': 0, '/odom': 0, '/camera/image_raw': 0}
        self.first_seen = {}

        self.start_time = None
        self.start_xy = None
        self.path_length = 0.0
        self.last_xy = None
        self.reached = False
        self.finished = False
        self.arrival_time = None
        self.last_state = ''

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
        self.create_timer(1.0 / self.control_rate_value(), self.control_loop)

        self.get_logger().info(
            f'Mission started: target=({self.target_x:.2f}, {self.target_y:.2f})')

    def control_rate_value(self):
        """Return the control loop rate in Hz."""
        return float(self.get_parameter('control_rate').value)

    def odom_callback(self, msg):
        """Track pose and travelled distance."""
        self.count('/odom')
        self.odom = msg
        p = msg.pose.pose.position
        if self.start_xy is None:
            self.start_xy = (p.x, p.y)
            self.start_time = time.time()
        if self.last_xy is not None:
            self.path_length += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def scan_callback(self, msg):
        """Reduce the scan to the minimum distance in each sector."""
        self.count('/scan')
        mins = {name: float('inf') for name, _, _ in self.SECTORS}
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = math.degrees(
                normalize_angle(msg.angle_min + i * msg.angle_increment))
            for name, lo, hi in self.SECTORS:
                if lo <= angle < hi and r < mins[name]:
                    mins[name] = r
        self.sectors = mins

    def image_callback(self, msg):
        """Keep the most recent camera frame."""
        self.count('/camera/image_raw')
        self.latest_image = msg

    def count(self, topic):
        """Count a received message for the report statistics."""
        self.counts[topic] += 1
        self.first_seen.setdefault(topic, time.time())

    def topic_rate(self, topic):
        """Return the average receive rate of a topic in Hz."""
        n = self.counts[topic]
        if n < 2:
            return 0.0
        return (n - 1) / max(time.time() - self.first_seen[topic], 1e-6)

    def publish_cmd(self, linear, angular):
        """Publish a velocity command."""
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        """Run one control cycle."""
        if self.finished:
            return
        if self.odom is None or self.sectors is None:
            return

        pose = self.odom.pose.pose
        x, y = pose.position.x, pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)

        if not self.reached:
            if math.hypot(self.target_x - x, self.target_y - y) <= self.goal_tolerance:
                self.reached = True
                self.arrival_time = time.time()
                self.get_logger().info('Target reached, stopping.')
            else:
                s = self.sectors
                linear, angular, state = self.navigator.compute(
                    x, y, yaw, self.target_x, self.target_y,
                    s['front'], s['front_left'], s['front_right'],
                    s['left'], s['right'])
                if state != self.last_state:
                    self.get_logger().info(f'Navigation state: {state}')
                    self.last_state = state
                self.publish_cmd(linear, angular)
                return

        # Reached: keep stopped until a fresh camera frame is available.
        self.publish_cmd(0.0, 0.0)
        if self.latest_image is not None:
            self.finish(x, y)

    def build_report(self, x, y, error, saved):
        """Build the checkpoint report text."""
        types = dict(self.get_topic_names_and_types())

        def type_of(name):
            return ', '.join(types.get(name, ['(not found)']))

        topics = [f'{name} [{", ".join(t)}]' for name, t in sorted(types.items())]
        status = 'SUCCESS - target reached' if saved else 'target reached, image NOT saved'
        out = [
            'AI MASTER PREREQUISITES CHECKPOINT', '',
            '', '--- SYSTEM INFORMATION ---', '',
            f'Username: {getpass.getuser()}',
            f'Hostname: {socket.gethostname()}',
            f'Current directory: {os.getcwd()}',
            f'Linux kernel: {platform.release()}',
            f'Python version: {sys.version.split()[0]}',
            f'Current date: {datetime.datetime.now():%Y-%m-%d %H:%M:%S}',
            '', '', '--- ROS 2 TOPICS ---', '', *topics,
            '', '', '--- ROS 2 MESSAGE TYPES ---', '',
        ]
        for name in ('/camera/image_raw', '/cmd_vel', '/odom', '/scan', '/tf'):
            out.append(f'{name}: {type_of(name)}')
        out += ['', '', '--- TOPIC FREQUENCIES ---', '']
        for name in ('/scan', '/odom', '/camera/image_raw'):
            out.append(f'{name}: {self.topic_rate(name):.1f} Hz')
        out += [
            '', '', '--- TOPIC INSPECTION ---', '',
            'Odometry position field: pose.pose.position (x, y, z in meters)',
            'Odometry orientation field: pose.pose.orientation '
            '(quaternion x, y, z, w)',
            'LaserScan ranges field: ranges (float32[], one distance per beam '
            'from angle_min to angle_max)',
            'LaserScan distance unit: meters',
            '', '', '--- RVIZ2 INSPECTION ---', '',
            'Fixed Frame: odom',
            '', '', '--- TF FRAMES ---', '',
            'Frame 1: odom',
            'Frame 2: base_footprint',
            'Frame 3: base_link',
            '', '', '--- TF KNOWLEDGE ---', '',
            'Purpose of /tf: publishes the dynamic (changing) coordinate frame '
            'transforms, e.g. odom -> base_footprint as the robot moves.',
            '',
            'Purpose of /tf_static: publishes fixed transforms that never change '
            '(latched), e.g. base_link -> camera and base_link -> lidar.',
            '',
            'Reason RViz2 requires a Fixed Frame: all data is displayed in one '
            'common, non-moving reference frame; RViz2 transforms every message '
            'into it using TF.',
            '', '', '--- MISSION RESULT ---', '',
            f'Mission status: {status}',
            f'Target X: {self.target_x}',
            f'Target Y: {self.target_y}',
            f'Final X: {x:.3f}',
            f'Final Y: {y:.3f}',
            f'Final distance to target: {error:.3f} m',
            f'Scans processed: {self.counts["/scan"]}',
            f'Odometry messages processed: {self.counts["/odom"]}',
            f'Camera messages received: {self.counts["/camera/image_raw"]}',
            f'Arrival image: {self.image_path if saved else "none"}',
        ]
        return out

    def finish(self, x, y):
        """Save the arrival image and the report, then request shutdown."""
        self.finished = True
        saved = False
        try:
            img = image_to_bgr(self.latest_image)
            os.makedirs(os.path.dirname(self.image_path), exist_ok=True)
            saved = bool(cv2.imwrite(self.image_path, img))
        except Exception as exc:  # noqa: B902
            self.get_logger().error(f'Failed to save arrival image: {exc}')

        error = math.hypot(self.target_x - x, self.target_y - y)
        lines = self.build_report(x, y, error, saved)
        try:
            with open(self.report_path, 'w') as f:
                f.write('\n'.join(lines) + '\n')
        except OSError as exc:
            self.get_logger().error(f'Failed to write report: {exc}')

        print(
            f'[mission_controller] The robot has reached the destination '
            f'({x:.2f}, {y:.2f}). Arrival image saved to {self.image_path}',
            flush=True)
        self.get_logger().info(f'Report written to {self.report_path}')
        raise SystemExit


def main(args=None):
    """Run the mission controller node."""
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
