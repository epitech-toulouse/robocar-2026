"""Record camera, LiDAR, and the final safe driving command for BC training."""

from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from .bc_preprocessing import (
    SCAN_BINS,
    image_to_model_input,
    scan_to_model_input,
)


class DemonstrationRecorder(Node):
    """Save aligned model inputs and safe Ackermann labels without publishing."""

    def __init__(self):
        super().__init__('record_behavioral_demonstrations')
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('label_topic', '/ackermann_cmd_safe')
        self.declare_parameter('output_dir', '/dataset')
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('record_rate_hz', 5.0)
        self.declare_parameter('image_timeout_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 0.35)
        self.declare_parameter('command_timeout_sec', 0.25)

        self.max_range_m = float(self.get_parameter('max_range_m').value)
        rate = float(self.get_parameter('record_rate_hz').value)
        if self.max_range_m <= 0.0 or rate <= 0.0:
            raise ValueError('max_range_m and record_rate_hz must be positive')
        self.image_timeout = float(self.get_parameter('image_timeout_sec').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self.command_timeout = float(self.get_parameter('command_timeout_sec').value)

        self.output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        self.image_dir = self.output_dir / 'images'
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / 'demonstrations.csv'
        self.csv_file = self.csv_path.open('a', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_path.stat().st_size == 0:
            self.csv_writer.writerow([
                'image_path', 'scan_normalized', 'steering_angle_rad',
                'speed_mps', 'max_range_m', 'recorded_unix_sec',
            ])
            self.csv_file.flush()

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None
        self.latest_command = None
        self.image_time = None
        self.scan_time = None
        self.command_time = None
        self.last_record_time = 0.0
        with self.csv_path.open(encoding='utf-8') as existing:
            self.sample_count = max(0, sum(1 for _ in existing) - 1)

        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AckermannDriveStamped, self.get_parameter('label_topic').value,
            self.command_callback, 10,
        )
        self.create_timer(1.0 / rate, self.record_sample)
        self.get_logger().info(
            f'Recording demonstrations to {self.csv_path}; '
            'labels include safe zero-speed stops. This node does not publish drive commands.'
        )

    @staticmethod
    def now():
        return time.monotonic()

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            model_input = image_to_model_input(image)
            self.latest_image = np.clip(model_input[0] * 255.0, 0, 255).astype(np.uint8)
            self.image_time = self.now()
        except Exception as exc:
            self.get_logger().warning(f'Ignoring invalid camera frame: {exc}')

    def scan_callback(self, msg):
        self.latest_scan = scan_to_model_input(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            self.max_range_m,
            SCAN_BINS,
        )
        self.scan_time = self.now()

    def command_callback(self, msg):
        self.latest_command = msg
        self.command_time = self.now()

    def record_sample(self):
        now = self.now()
        if (now - self.last_record_time) < 1.0 / float(
                self.get_parameter('record_rate_hz').value):
            return
        if self.latest_image is None or self.latest_scan is None or self.latest_command is None:
            return
        if now - self.image_time > self.image_timeout:
            return
        if now - self.scan_time > self.scan_timeout:
            return
        if now - self.command_time > self.command_timeout:
            return

        self.last_record_time = now
        self.sample_count += 1
        image_name = f'{self.sample_count:08d}.png'
        image_path = self.image_dir / image_name
        if not cv2.imwrite(str(image_path), self.latest_image):
            self.get_logger().error(f'Could not write demonstration image {image_path}')
            return

        drive = self.latest_command.drive
        self.csv_writer.writerow([
            f'images/{image_name}',
            json.dumps(self.latest_scan.astype(float).tolist(), separators=(',', ':')),
            f'{float(drive.steering_angle):.8f}',
            f'{float(drive.speed):.8f}',
            f'{self.max_range_m:.6f}',
            f'{time.time():.6f}',
        ])
        self.csv_file.flush()
        if self.sample_count % 25 == 0:
            self.get_logger().info(
                f'Saved {self.sample_count} samples; '
                f'latest speed label={float(drive.speed):.3f} m/s.'
            )

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DemonstrationRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
