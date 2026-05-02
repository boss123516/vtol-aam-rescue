#!/usr/bin/env python3

import subprocess
import re
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


GZ_TOPIC = "/world/default/model/amsr_vtol_0/link/camera_link/sensor/camera/image"


class GzImageRepublisher(Node):
    def __init__(self):
        super().__init__("gz_image_republisher")
        self.pub = self.create_publisher(Image, "/image_raw", 10)
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)
        self.get_logger().info("GZ image republisher started: Gazebo camera -> /image_raw")

    def timer_callback(self):
        try:
            result = subprocess.run(
                ["gz", "topic", "-e", "-t", GZ_TOPIC, "-n", "1"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=False,
                timeout=1.0,
            )

            raw = result.stdout
            if not raw:
                return

            text = raw.decode("latin1", errors="ignore")

            width_match = re.search(r"width:\s*(\d+)", text)
            height_match = re.search(r"height:\s*(\d+)", text)
            step_match = re.search(r"step:\s*(\d+)", text)

            if not width_match or not height_match or not step_match:
                return

            width = int(width_match.group(1))
            height = int(height_match.group(1))
            step = int(step_match.group(1))

            # gz topic -e 출력에서 data: "..." 내부 바이트 문자열 추출
            data_match = re.search(rb'data:\s*"((?:[^"\\]|\\.)*)"', raw, re.DOTALL)
            if not data_match:
                return

            escaped = data_match.group(1)
            img_bytes = bytes(escaped.decode("latin1").encode("latin1").decode("unicode_escape"), "latin1")

            expected = height * step
            if len(img_bytes) < expected:
                self.get_logger().warn(f"Image bytes too short: {len(img_bytes)} < {expected}")
                return

            img_bytes = img_bytes[:expected]

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            msg.height = height
            msg.width = width
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = step
            msg.data = img_bytes

            self.pub.publish(msg)

        except subprocess.TimeoutExpired:
            return
        except Exception as e:
            self.get_logger().warn(f"Failed to republish image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GzImageRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
