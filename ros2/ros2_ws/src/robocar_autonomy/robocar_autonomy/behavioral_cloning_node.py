"""Camera + LiDAR behavioral cloning controller with explicit fail-safe stops."""

from __future__ import annotations

import json
import math
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
from std_msgs.msg import String

from .bc_preprocessing import (
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    SCAN_BINS,
    image_to_model_input,
    scan_to_model_input,
)


class BehavioralCloningNode(Node):
    """Run a trained ONNX policy; missing inputs or model always produce zero speed."""

    def __init__(self):
        super().__init__('behavioral_cloning_node')
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('command_topic', '/drive')
        self.declare_parameter('status_topic', '/autonomy/status')
        self.declare_parameter('model_path', '/models/behavioral_cloning.onnx')
        self.declare_parameter('max_speed_mps', 0.10)
        self.declare_parameter('max_steering_angle_rad', 0.34)
        self.declare_parameter('camera_timeout_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 0.30)
        self.declare_parameter('stop_confidence_threshold', -1.0)
        self.declare_parameter('stop_release_threshold', -1.0)

        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.max_steering = float(self.get_parameter('max_steering_angle_rad').value)
        self.camera_timeout = float(self.get_parameter('camera_timeout_sec').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self.bridge = CvBridge()
        self.image_input = None
        self.scan_input = None
        self.image_time = None
        self.scan_time = None
        self.last_status = None
        self.model = None
        self.metadata = None
        self.stop_threshold = 0.70
        self.stop_release_threshold = 0.35
        self.model_stopped = False

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('command_topic').value, 10
        )
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10
        )
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_callback,
            qos_profile_sensor_data,
        )
        self._load_model()
        self.create_timer(0.05, self.control_tick)

    @staticmethod
    def now():
        return time.monotonic()

    def _load_model(self):
        model_path = Path(self.get_parameter('model_path').value).expanduser()
        metadata_path = model_path.with_suffix(model_path.suffix + '.json')
        try:
            metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
            if (metadata.get('input_image_width') != IMAGE_WIDTH or
                    metadata.get('input_image_height') != IMAGE_HEIGHT or
                    metadata.get('scan_bins') != SCAN_BINS):
                raise ValueError('model input dimensions do not match ROS preprocessing')
            max_range = float(metadata['max_range_m'])
            trained_speed = float(metadata['max_speed_mps'])
            trained_steering = float(metadata['max_steering_angle_rad'])
            if min(max_range, trained_speed, trained_steering,
                   self.max_speed, self.max_steering,
                   self.camera_timeout, self.scan_timeout) <= 0.0:
                raise ValueError('model metadata has non-positive output limits')
            net = cv2.dnn.readNetFromONNX(str(model_path))
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            configured_threshold = float(
                self.get_parameter('stop_confidence_threshold').value
            )
            stop_threshold = (
                configured_threshold if 0.0 < configured_threshold < 1.0
                else float(metadata.get('stop_confidence_threshold', 0.70))
            )
            configured_release = float(self.get_parameter('stop_release_threshold').value)
            release_threshold = (
                configured_release if 0.0 < configured_release < self.stop_threshold
                else float(metadata.get('stop_release_threshold', 0.35))
            )
            if not 0.0 < release_threshold < stop_threshold < 1.0:
                raise ValueError('model stop thresholds must satisfy 0 < release < stop < 1')
            self.model = net
            self.metadata = metadata
            self.max_range = max_range
            self.max_speed = min(self.max_speed, trained_speed)
            self.max_steering = min(self.max_steering, trained_steering)
            self.stop_threshold = stop_threshold
            self.stop_release_threshold = release_threshold
            self.get_logger().info(
                f'Loaded BC model {model_path}; stop threshold={self.stop_threshold:.2f}, '
                f'speed limit={self.max_speed:.2f} m/s.'
            )
        except Exception as exc:
            self.get_logger().error(f'Behavioral cloning unavailable; holding stop: {exc}')

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_input = np.expand_dims(image_to_model_input(image), axis=0)
            self.image_time = self.now()
        except Exception as exc:
            self.image_input = None
            self.get_logger().warning(f'Ignoring invalid camera frame: {exc}')

    def scan_callback(self, msg):
        if self.metadata is None:
            return
        try:
            self.scan_input = scan_to_model_input(
                msg.ranges,
                msg.angle_min,
                msg.angle_increment,
                self.max_range,
                SCAN_BINS,
            )[None, :]
            self.scan_time = self.now()
        except Exception as exc:
            self.scan_input = None
            self.get_logger().warning(f'Ignoring invalid LiDAR scan: {exc}')

    def _publish_command(self, speed, steering, status):
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.drive.speed = float(speed)
        command.drive.steering_angle = float(steering)
        self.drive_pub.publish(command)
        if status != self.last_status:
            message = String()
            message.data = status
            self.status_pub.publish(message)
            self.last_status = status

    @staticmethod
    def _sigmoid(value):
        value = max(-60.0, min(60.0, float(value)))
        return 1.0 / (1.0 + math.exp(-value))

    def control_tick(self):
        now = self.now()
        if self.model is None:
            self._publish_command(0.0, 0.0, 'stopped: behavioral cloning model unavailable')
            return
        if (self.image_time is None or now - self.image_time > self.camera_timeout or
                self.image_input is None):
            self._publish_command(0.0, 0.0, 'stopped: camera image missing or stale')
            return
        if (self.scan_time is None or now - self.scan_time > self.scan_timeout or
                self.scan_input is None):
            self._publish_command(0.0, 0.0, 'stopped: LiDAR scan missing or stale')
            return

        try:
            self.model.setInput(self.image_input, 'image')
            self.model.setInput(self.scan_input, 'scan')
            output = np.asarray(self.model.forward(), dtype=np.float32).reshape(-1)
            if output.size != 3 or not np.all(np.isfinite(output)):
                raise ValueError(f'expected three finite outputs, got {output}')

            steering_norm = float(np.clip(output[0], -1.0, 1.0))
            speed_norm = float(np.clip(output[1], -1.0, 1.0))
            stop_probability = self._sigmoid(output[2])
            if self.model_stopped:
                self.model_stopped = stop_probability >= self.stop_release_threshold
            else:
                self.model_stopped = stop_probability >= self.stop_threshold
            if self.model_stopped:
                self._publish_command(0.0, 0.0, f'stopped: model stop={stop_probability:.2f}')
                return

            speed = float(np.clip(speed_norm * self.max_speed, -self.max_speed, self.max_speed))
            steering = float(np.clip(
                steering_norm * self.max_steering,
                -self.max_steering,
                self.max_steering,
            ))
            self._publish_command(
                speed,
                steering,
                f'behavioral cloning: stop={stop_probability:.2f} speed={speed:.2f} m/s',
            )
        except Exception as exc:
            self.get_logger().error(f'Inference failed; publishing stop: {exc}')
            self._publish_command(0.0, 0.0, 'stopped: behavioral cloning inference error')


def main(args=None):
    rclpy.init(args=args)
    node = BehavioralCloningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_command(0.0, 0.0, 'stopped: behavioral cloning shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
