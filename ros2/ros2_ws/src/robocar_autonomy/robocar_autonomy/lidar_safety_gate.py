"""Final LiDAR safety gate between ackermann_mux and the VESC adapter."""

import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarSafetyGate(Node):
    """Overrides manual and autonomous motion when the scan is unsafe or stale."""

    def __init__(self):
        super().__init__('lidar_safety_gate')
        self.declare_parameter('command_topic', '/ackermann_cmd')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('safe_command_topic', '/ackermann_cmd_safe')
        self.declare_parameter('stop_distance_m', 0.70)
        self.declare_parameter('release_distance_m', 0.95)
        self.declare_parameter('slow_distance_m', 1.50)
        self.declare_parameter('front_half_angle_deg', 25.0)
        self.declare_parameter('rear_half_angle_deg', 25.0)
        # Mask the camera body immediately behind the centered LiDAR. These
        # bounds are expressed in the LaserScan frame (x forward, y left).
        self.declare_parameter('self_filter_x_min_m', -0.15)
        self.declare_parameter('self_filter_x_max_m', -0.02)
        self.declare_parameter('self_filter_y_half_width_m', 0.075)
        self.declare_parameter('scan_timeout_sec', 0.30)
        self.declare_parameter('command_timeout_sec', 0.20)

        self.stop_distance = float(self.get_parameter('stop_distance_m').value)
        self.release_distance = float(self.get_parameter('release_distance_m').value)
        self.slow_distance = float(self.get_parameter('slow_distance_m').value)
        self.front_half_angle = float(self.get_parameter('front_half_angle_deg').value)
        self.rear_half_angle = float(self.get_parameter('rear_half_angle_deg').value)
        self.self_filter_x_min = float(
            self.get_parameter('self_filter_x_min_m').value)
        self.self_filter_x_max = float(
            self.get_parameter('self_filter_x_max_m').value)
        self.self_filter_y_half_width = float(
            self.get_parameter('self_filter_y_half_width_m').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self.command_timeout = float(self.get_parameter('command_timeout_sec').value)

        if (self.self_filter_x_min >= self.self_filter_x_max or
                self.self_filter_y_half_width < 0.0):
            raise ValueError('Invalid LiDAR self-filter bounds')

        command_topic = self.get_parameter('command_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        safe_topic = self.get_parameter('safe_command_topic').value
        self.last_command = None
        self.last_command_time = None
        self.last_scan_time = None
        self.scan_ranges = None
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0
        self.obstacle_latched = True

        self.safe_pub = self.create_publisher(AckermannDriveStamped, safe_topic, 10)
        self.create_subscription(AckermannDriveStamped, command_topic, self.command_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback,
                                 qos_profile_sensor_data)
        # 20 Hz keeps the downstream VESC command watchdog refreshed.
        self.create_timer(0.05, self.output_timer)
        self.get_logger().info(
            f'LiDAR safety gate active: {command_topic} -> {safe_topic}, scan={scan_topic}.')

    @staticmethod
    def _now():
        return time.monotonic()

    def command_callback(self, msg):
        self.last_command = msg
        self.last_command_time = self._now()

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_increment = float(msg.angle_increment)
        self.last_scan_time = self._now()

        front = self.sector_min(-self.front_half_angle, self.front_half_angle)
        if front <= self.stop_distance:
            self.obstacle_latched = True
        elif front >= self.release_distance:
            self.obstacle_latched = False

    def sector_min(self, center_deg, half_angle_deg):
        if self.scan_ranges is None or self.scan_angle_increment == 0.0:
            return math.inf
        center = math.radians(center_deg)
        half_width = math.radians(half_angle_deg)
        distances = []
        for index, distance in enumerate(self.scan_ranges):
            angle = self.scan_angle_min + index * self.scan_angle_increment
            delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
            value = float(distance)
            if abs(delta) > half_width or not math.isfinite(value) or value < 0.0:
                continue

            # Ignore returns that land inside the camera's physical footprint.
            # This removes the car's own camera echo while keeping rear obstacle
            # checks active outside its outline.
            point_x = value * math.cos(angle)
            point_y = value * math.sin(angle)
            inside_self_filter = (
                self.self_filter_x_min <= point_x <= self.self_filter_x_max and
                abs(point_y) <= self.self_filter_y_half_width
            )
            if not inside_self_filter:
                distances.append(value)
        return min(distances, default=math.inf)

    def output_timer(self):
        now = self._now()
        output = AckermannDriveStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_link'

        scan_fresh = (
            self.last_scan_time is not None and
            now - self.last_scan_time <= self.scan_timeout
        )
        command_fresh = (
            self.last_command is not None and
            self.last_command_time is not None and
            now - self.last_command_time <= self.command_timeout
        )
        if not scan_fresh or not command_fresh:
            self.safe_pub.publish(output)
            return

        output.drive = self.last_command.drive
        speed = float(output.drive.speed)
        if not math.isfinite(speed) or not math.isfinite(float(output.drive.steering_angle)):
            output.drive.speed = 0.0
            self.safe_pub.publish(output)
            return

        half_angle = self.front_half_angle if speed >= 0.0 else self.rear_half_angle
        center = 0.0 if speed >= 0.0 else 180.0
        clearance = self.sector_min(center, half_angle)
        blocked = self.obstacle_latched if speed >= 0.0 else clearance <= self.stop_distance
        if blocked:
            output.drive.speed = 0.0
        elif clearance < self.slow_distance:
            scale = max(0.0, min(1.0,
                                 (clearance - self.stop_distance) /
                                 max(self.slow_distance - self.stop_distance, 0.01)))
            output.drive.speed = speed * scale
        self.safe_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_pub.publish(AckermannDriveStamped())
        node.destroy_node()
        rclpy.shutdown()
