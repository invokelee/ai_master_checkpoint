"""Robot monitor node: subscribes to /odom and /scan and reports diagnostics."""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    """Extract yaw (rad) from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotMonitor(Node):
    """Monitor robot state from /odom and /scan and log it periodically."""

    def __init__(self):
        super().__init__('robot_monitor')

        self.declare_parameter('report_period', 1.0)
        self.declare_parameter('stale_timeout', 2.0)

        report_period = self.get_parameter('report_period').value
        self.stale_timeout = self.get_parameter('stale_timeout').value

        self.odom = None
        self.scan_stats = None
        self.odom_count = 0
        self.scan_count = 0
        self.last_odom_time = None
        self.last_scan_time = None
        self._last_report_odom = 0
        self._last_report_scan = 0
        self._last_report_time = self.get_clock().now()

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_timer(report_period, self.report)

        self.get_logger().info('Robot monitor started.')

    def odom_callback(self, msg):
        """Store the latest odometry."""
        self.odom = msg
        self.odom_count += 1
        self.last_odom_time = self.get_clock().now()

    def scan_callback(self, msg):
        """Compute summary statistics for the latest scan."""
        self.scan_count += 1
        self.last_scan_time = self.get_clock().now()

        valid = [
            (r, i) for i, r in enumerate(msg.ranges)
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        if not valid:
            self.scan_stats = {'valid': 0, 'total': len(msg.ranges)}
            return

        min_range, min_index = min(valid)
        min_angle = msg.angle_min + min_index * msg.angle_increment
        self.scan_stats = {
            'valid': len(valid),
            'total': len(msg.ranges),
            'min_range': min_range,
            'min_angle': min_angle,
        }

    def _age(self, stamp, now):
        """Return seconds since stamp, or None if never received."""
        if stamp is None:
            return None
        return (now - stamp).nanoseconds * 1e-9

    def report(self):
        """Log a diagnostic summary."""
        now = self.get_clock().now()
        dt = (now - self._last_report_time).nanoseconds * 1e-9
        self._last_report_time = now

        odom_rate = (self.odom_count - self._last_report_odom) / dt if dt else 0.0
        scan_rate = (self.scan_count - self._last_report_scan) / dt if dt else 0.0
        self._last_report_odom = self.odom_count
        self._last_report_scan = self.scan_count

        log = self.get_logger()
        log.info('----- Robot diagnostics -----')

        odom_age = self._age(self.last_odom_time, now)
        if self.odom is None:
            log.warn('/odom: no data received yet')
        else:
            pose = self.odom.pose.pose
            twist = self.odom.twist.twist
            yaw = yaw_from_quaternion(pose.orientation)
            log.info(
                f'/odom: x={pose.position.x:.2f} y={pose.position.y:.2f} '
                f'yaw={math.degrees(yaw):.1f}deg | '
                f'v={twist.linear.x:.2f}m/s w={twist.angular.z:.2f}rad/s | '
                f'{odom_rate:.1f}Hz')
            if odom_age > self.stale_timeout:
                log.warn(f'/odom is stale ({odom_age:.1f}s since last msg)')

        scan_age = self._age(self.last_scan_time, now)
        if self.scan_stats is None:
            log.warn('/scan: no data received yet')
        else:
            s = self.scan_stats
            if s['valid'] == 0:
                log.warn(f'/scan: no valid ranges (0/{s["total"]})')
            else:
                log.info(
                    f'/scan: min={s["min_range"]:.2f}m at '
                    f'{math.degrees(s["min_angle"]):.1f}deg | '
                    f'valid={s["valid"]}/{s["total"]} | {scan_rate:.1f}Hz')
            if scan_age > self.stale_timeout:
                log.warn(f'/scan is stale ({scan_age:.1f}s since last msg)')


def main(args=None):
    """Run the robot monitor node."""
    rclpy.init(args=args)
    node = RobotMonitor()
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
