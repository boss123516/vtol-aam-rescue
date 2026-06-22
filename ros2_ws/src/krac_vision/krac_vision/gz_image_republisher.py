#!/usr/bin/env python3

import re
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


DEFAULT_GZ_TOPIC = '/world/default/model/amsr_vtol_0/link/camera_link/sensor/camera/image'


class GzImageRepublisher(Node):
    """Republish a Gazebo Transport camera topic as ROS2 sensor_msgs/Image.

    This is a fallback bridge for environments where ros_gz_bridge does not decode
    the AMSR camera topic correctly. It shells out to `gz topic -e`, so it is not
    ideal for high FPS, but it is convenient for YOLO pipeline debugging.
    """

    def __init__(self):
        super().__init__('gz_image_republisher')

        self.declare_parameter('gz_topic', DEFAULT_GZ_TOPIC)
        self.declare_parameter('ros_topic', '/image_raw')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('timeout_sec', 1.0)
        self.declare_parameter('encoding', 'rgb8')

        self.gz_topic = self.get_parameter('gz_topic').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value
        self.encoding = self.get_parameter('encoding').get_parameter_value().string_value

        self.pub = self.create_publisher(Image, self.ros_topic, 10)
        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f'Gazebo image republisher: {self.gz_topic} -> {self.ros_topic} @ {self.rate_hz:.1f} Hz')

    def timer_callback(self):
        try:
            result = subprocess.run(
                ['gz', 'topic', '-e', '-t', self.gz_topic, '-n', '1'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=False,
                timeout=self.timeout_sec,
            )
        except subprocess.TimeoutExpired:
            return
        except Exception as exc:
            self.get_logger().warn(f'Failed to execute gz topic: {exc}')
            return

        raw = result.stdout
        if not raw:
            err = result.stderr.decode('utf-8', errors='ignore').strip()
            if err:
                self.get_logger().debug(f'gz topic stderr: {err}')
            return

        msg = self._parse_gz_image(raw)
        if msg is None:
            return
        self.pub.publish(msg)

    def _parse_gz_image(self, raw: bytes):
        text = raw.decode('latin1', errors='ignore')
        width_match = re.search(r'width:\s*(\d+)', text)
        height_match = re.search(r'height:\s*(\d+)', text)
        step_match = re.search(r'step:\s*(\d+)', text)

        if not width_match or not height_match or not step_match:
            return None

        width = int(width_match.group(1))
        height = int(height_match.group(1))
        step = int(step_match.group(1))

        data_match = re.search(rb'data:\s*"((?:[^"\\]|\\.)*)"', raw, re.DOTALL)
        if not data_match:
            return None

        escaped = data_match.group(1)
        try:
            img_bytes = bytes(escaped.decode('latin1').encode('latin1').decode('unicode_escape'), 'latin1')
        except Exception as exc:
            self.get_logger().warn(f'Failed to decode image bytes: {exc}')
            return None

        expected = height * step
        if len(img_bytes) < expected:
            self.get_logger().warn(f'Image bytes too short: {len(img_bytes)} < {expected}')
            return None

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = self.encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = img_bytes[:expected]
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = GzImageRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
