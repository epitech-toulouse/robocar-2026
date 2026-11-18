"""Follow white tape arrows, with LiDAR wall avoidance and fail-safe stops."""

import math
import time

import cv2
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class ArrowFollowNode(Node):
    """A deliberately slow arrow follower for floor arrows made from white tape."""

    def __init__(self):
        super().__init__('arrow_follow_node')

        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('command_topic', '/drive')
        self.declare_parameter('status_topic', '/autonomy/status')
        self.declare_parameter('debug_mask_topic', '/autonomy/arrow_mask')
        self.declare_parameter('speed_mps', 0.10)
        self.declare_parameter('max_steering_angle', 0.34)
        self.declare_parameter('arrow_memory_sec', 2.0)
        self.declare_parameter('camera_timeout_sec', 1.5)
        self.declare_parameter('scan_timeout_sec', 0.30)
        self.declare_parameter('roi_top_ratio', 0.45)
        self.declare_parameter('white_saturation_max', 85)
        self.declare_parameter('white_value_min', 175)
        self.declare_parameter('min_arrow_area_px', 70)
        self.declare_parameter('heading_gain', 0.55)
        self.declare_parameter('wall_clearance_m', 0.60)
        self.declare_parameter('wall_avoidance_gain', 0.22)
        self.declare_parameter('front_slow_distance_m', 1.40)
        self.declare_parameter('front_corridor_half_width_m', 0.30)

        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.arrow_memory_sec = float(self.get_parameter('arrow_memory_sec').value)
        self.camera_timeout_sec = float(self.get_parameter('camera_timeout_sec').value)
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)

        self.speed_mps = float(self.get_parameter('speed_mps').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.wall_clearance_m = float(self.get_parameter('wall_clearance_m').value)
        self.wall_avoidance_gain = float(self.get_parameter('wall_avoidance_gain').value)
        self.front_slow_distance_m = float(self.get_parameter('front_slow_distance_m').value)
        self.front_corridor_half_width = float(
            self.get_parameter('front_corridor_half_width_m').value)
        self.roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
        self.white_saturation_max = int(self.get_parameter('white_saturation_max').value)
        self.white_value_min = int(self.get_parameter('white_value_min').value)
        self.min_arrow_area_px = int(self.get_parameter('min_arrow_area_px').value)

        self.bridge = CvBridge()
        self.last_image_time = None
        self.last_scan_time = None
        self.last_arrow_time = None
        self.last_arrow_steering = 0.0
        self.scan_ranges = None
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.command_topic, 10)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.mask_pub = self.create_publisher(
            Image, self.get_parameter('debug_mask_topic').value, 2)
        self.create_subscription(Image, self.image_topic, self.image_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback,
                                 qos_profile_sensor_data)
        # Publish above the VESC adapter's 10 Hz command watchdog.
        self.create_timer(0.05, self.command_timer)

        self.get_logger().info(
            f'Arrow follower ready: image={self.image_topic}, scan={self.scan_topic}, '
            f'command={self.command_topic}; RB must be held to unlock autonomous drive.')

    @staticmethod
    def _now():
        return time.monotonic()

    def scan_callback(self, msg):
        self.scan_ranges = np.asarray(msg.ranges, dtype=np.float32)
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_increment = float(msg.angle_increment)
        self.last_scan_time = self._now()

    def image_callback(self, msg):
        now = self._now()
        self.last_image_time = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detected, steering, mask = self.detect_white_arrow(image)
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
            if detected:
                self.last_arrow_steering = steering
                self.last_arrow_time = now
        except Exception as exc:  # A bad frame must result in a stop, not a stale command.
            self.get_logger().warning(f'Could not process camera frame: {exc}')

    def detect_white_arrow(self, image):
        height, width = image.shape[:2]
        top = int(height * clamp(self.roi_top_ratio, 0.0, 0.9))
        left = int(width * 0.08)
        right = int(width * 0.92)
        roi = image[top:, left:right]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([0, 0, self.white_value_min], dtype=np.uint8),
            np.array([180, self.white_saturation_max, 255], dtype=np.uint8),
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, np.ones((7, 7), dtype=np.uint8), iterations=2)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8), iterations=1)

        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        min_area = max(self.min_arrow_area_px, int(roi.shape[0] * roi.shape[1] * 0.0005))
        candidates = []
        for label in range(1, count):
            x, y, component_width, component_height, area = stats[label]
            if area < min_area:
                continue
            if max(component_width, component_height) < 16:
                continue
            if min(component_width, component_height) == 0:
                continue
            if max(component_width, component_height) / min(component_width, component_height) < 1.35:
                continue
            candidates.append((area, label))

        if not candidates:
            return False, 0.0, mask

        _, best_label = max(candidates)
        ys, xs = np.where(labels == best_label)
        points = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        if points.shape[0] < min_area:
            return False, 0.0, mask

        center = points.mean(axis=0)
        centered = points - center
        covariance = np.cov(centered, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        if not np.all(np.isfinite(axis)):
            return False, 0.0, mask

        projections = centered @ axis
        lower = float(np.min(projections))
        upper = float(np.max(projections))
        span = upper - lower
        if span < 12.0:
            return False, 0.0, mask

        # The arrow tip extends farther from the shape's center than the flat tail.
        positive_reach = upper - float(np.mean(projections))
        negative_reach = float(np.mean(projections)) - lower
        reach_ratio = max(positive_reach, negative_reach) / max(
            min(positive_reach, negative_reach), 1e-3)
        if reach_ratio < 1.10:
            return False, 0.0, mask
        direction = axis if positive_reach > negative_reach else -axis

        # Image x grows to the right and image y grows downward. Positive
        # Ackermann steering is left, hence the minus sign.
        heading_error = math.atan2(float(direction[0]), -float(direction[1]))
        steering = clamp(-self.heading_gain * heading_error,
                         -self.max_steering_angle, self.max_steering_angle)
        return True, steering, mask

    def _sector_min(self, angle_low_deg, angle_high_deg, max_abs_y=None,
                    center_deg=0.0):
        if self.scan_ranges is None or self.scan_angle_increment == 0.0:
            return math.inf
        low = math.radians(angle_low_deg)
        high = math.radians(angle_high_deg)
        center = math.radians(center_deg)
        valid = []
        for index, distance in enumerate(self.scan_ranges):
            angle = self.scan_angle_min + index * self.scan_angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            relative_angle = math.atan2(
                math.sin(angle - center), math.cos(angle - center))
            value = float(distance)
            if low <= relative_angle <= high and math.isfinite(value) and value >= 0.0:
                point_x = value * math.cos(angle)
                point_y = value * math.sin(angle)
                corridor_y = -point_x * math.sin(center) + point_y * math.cos(center)
                if max_abs_y is None or abs(corridor_y) <= max_abs_y:
                    valid.append(value)
        return min(valid, default=math.inf)

    def command_timer(self):
        now = self._now()
        camera_fresh = (
            self.last_image_time is not None and
            now - self.last_image_time <= self.camera_timeout_sec
        )
        scan_fresh = (
            self.last_scan_time is not None and
            now - self.last_scan_time <= self.scan_timeout_sec
        )
        arrow_fresh = (
            self.last_arrow_time is not None and
            now - self.last_arrow_time <= self.arrow_memory_sec
        )

        speed = 0.0
        steering = 0.0
        status = 'stopped: waiting for fresh camera, LiDAR, and a white arrow'
        if camera_fresh and scan_fresh and arrow_fresh:
            left_clearance = self._sector_min(18.0, 70.0)
            right_clearance = self._sector_min(-70.0, -18.0)
            wall_correction = 0.0
            if (left_clearance < self.wall_clearance_m or
                    right_clearance < self.wall_clearance_m):
                left_value = min(left_clearance, self.wall_clearance_m)
                right_value = min(right_clearance, self.wall_clearance_m)
                # Positive steering turns left; steer toward the side with more room.
                wall_correction = self.wall_avoidance_gain * (left_value - right_value)

            steering = clamp(
                self.last_arrow_steering + wall_correction,
                -self.max_steering_angle,
                self.max_steering_angle,
            )
            front = self._sector_min(
                -25.0, 25.0,
                max_abs_y=self.front_corridor_half_width,
                center_deg=math.degrees(steering),
            )
            if front <= self.front_slow_distance_m:
                speed_scale = clamp(
                    (front - 0.45) / max(self.front_slow_distance_m - 0.45, 0.01),
                    0.0, 1.0)
            else:
                speed_scale = 1.0
            speed = self.speed_mps * speed_scale
            status = f'following arrow; front={front:.2f}m speed={speed:.2f}m/s'
        elif not scan_fresh:
            status = 'stopped: LiDAR scan is missing or stale'
        elif not camera_fresh:
            status = 'stopped: camera image is missing or stale'
        elif not arrow_fresh:
            status = 'stopped: no recent white arrow detected'

        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.drive.speed = float(speed)
        command.drive.steering_angle = float(steering)
        self.drive_pub.publish(command)

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArrowFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = AckermannDriveStamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        node.drive_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()
