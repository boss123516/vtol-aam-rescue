#!/usr/bin/env python3

import re
import subprocess
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


DEFAULT_GZ_TOPIC = '/world/default/model/amsr_vtol_0/link/camera_link/sensor/camera/image'

# Each gz.msgs.Image DebugString printed by `gz topic -e` starts with the
# `header {` field, so consecutive messages in the stream are delimited by a
# newline immediately followed by `header {`. The escaped binary `data:` payload
# never contains a raw newline (non-printable bytes are octal-escaped), so this
# delimiter is safe to split on.
_MSG_DELIM = b'\nheader {'
_MAX_BUFFER_BYTES = 64 * 1024 * 1024  # safety cap; ~13 full 1280x960 frames


class GzImageRepublisher(Node):
    """Republish a Gazebo Transport camera topic as ROS2 sensor_msgs/Image.

    This is a fallback bridge for environments where ros_gz_bridge does not decode
    the AMSR camera topic correctly.

    A single long-lived `gz topic -e` subprocess streams the camera continuously;
    a reader thread drains it (keeping only the newest complete frame) and a timer
    republishes that frame at ``rate_hz``. This replaces the previous design that
    spawned a fresh `gz topic -e -n 1` process every tick: those short-lived
    subscribers were routinely SIGKILLed on the 1s timeout mid-handshake, leaving
    orphaned subscriptions whose send queues accumulated 30 Hz camera frames
    inside the gz server (~110 MB/s RSS growth -> eventual OOM/freeze). One steady,
    promptly-drained subscription lets gz's normal flow control reclaim frames.
    """

    def __init__(self):
        super().__init__('gz_image_republisher')

        self.declare_parameter('gz_topic', DEFAULT_GZ_TOPIC)
        self.declare_parameter('ros_topic', '/image_raw')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('encoding', 'rgb8')

        self.gz_topic = self.get_parameter('gz_topic').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.encoding = self.get_parameter('encoding').get_parameter_value().string_value

        self.pub = self.create_publisher(Image, self.ros_topic, 10)

        # Newest complete raw message block, guarded by _lock. Set by the reader
        # thread, consumed (and cleared) by the publish timer so we only ever
        # publish fresh frames.
        self._lock = threading.Lock()
        self._latest_raw = None
        self._stop = threading.Event()

        self._proc = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', self.gz_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Gazebo image republisher (persistent stream): '
            f'{self.gz_topic} -> {self.ros_topic} @ {self.rate_hz:.1f} Hz')

    def _reader_loop(self):
        """Continuously drain the gz stream, retaining only the newest frame.

        The stream is a run of concatenated message DebugStrings delimited by
        ``\\nheader {``. ``_parse_gz_image`` locates width/height/step/data by
        regex, so a complete block need not carry its leading ``header {`` prefix
        for parsing to succeed.
        """
        buffer = bytearray()
        stream = self._proc.stdout
        while not self._stop.is_set():
            chunk = stream.read(262144)
            if not chunk:
                break  # gz topic exited / EOF
            buffer.extend(chunk)

            # Locate the last two delimiters via C-level rfind (cheap even on a
            # multi-MB buffer); the newest *complete* message lies between them.
            last = buffer.rfind(_MSG_DELIM)
            if last > 0:
                prev = buffer.rfind(_MSG_DELIM, 0, last)
                start = 0 if prev < 0 else prev + 1  # drop the delimiter's leading '\n'
                with self._lock:
                    self._latest_raw = bytes(buffer[start:last])
                # Keep only the still-streaming tail (from the last delimiter on).
                del buffer[:last + 1]

            if len(buffer) > _MAX_BUFFER_BYTES:
                # Delimiter never matched (unexpected format); don't grow forever.
                del buffer[:-1024]

    def timer_callback(self):
        with self._lock:
            raw = self._latest_raw
            self._latest_raw = None
        if not raw:
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

    def destroy_node(self):
        self._stop.set()
        try:
            if self._proc and self._proc.poll() is None:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
        except Exception:
            pass
        super().destroy_node()


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
