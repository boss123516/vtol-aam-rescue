#!/usr/bin/env python3
"""Gimbal relay: ROS gimbal commands (deg) <-> Gazebo joint controllers (rad).

The rescue controller (`rescue_controller_placeholder.py`) publishes desired
gimbal angles on `/gimbal/angle_cmd` (geometry_msgs/Vector3, x=yaw_deg,
y=pitch_deg, controller convention: pitch=-90 -> straight down / nadir,
pitch=0 -> forward). The standard_vtol Gazebo model now carries a 2-DOF gimbal
whose two `gz-sim-joint-position-controller` plugins listen (via ros_gz_bridge)
on:
    /model/<GZ_MODEL_NAME>/gimbal_yaw    (std_msgs/Float64 -> gz.msgs.Double, rad)
    /model/<GZ_MODEL_NAME>/gimbal_pitch  (std_msgs/Float64 -> gz.msgs.Double, rad)

Joint <-> command mapping (see model.sdf comment):
    yaw_joint_rad   = radians(YAW_SIGN   * yaw_cmd_deg)
    pitch_joint_rad = radians(PITCH_SIGN * pitch_cmd_deg)   # PITCH_SIGN=-1 ->
                      pitch_cmd -90 maps to +1.5708 rad (camera looks down)

The signs are exposed as parameters so they can be flipped after a visual
check in Gazebo without touching the mapping logic.

This node also re-publishes the commanded angles on `/gimbal/attitude`
(geometry_msgs/Vector3Stamped, x=yaw_deg, y=pitch_deg) so the rescue
controller's visual-servo math has a gimbal-attitude feedback source. The
JointPositionController tracks the command tightly, so an open-loop echo is a
good approximation; if true joint feedback is ever needed, bridge the model's
`joint_state` topic instead.
"""

from __future__ import annotations

import math
import os

import rclpy
from geometry_msgs.msg import Vector3, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64


class GimbalRelay(Node):
    def __init__(self) -> None:
        super().__init__("gimbal_relay")

        default_model = os.environ.get("GZ_MODEL_NAME", "standard_vtol_0")
        self.declare_parameter("gz_model_name", default_model)
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("pitch_sign", -1.0)
        self.declare_parameter("republish_hz", 10.0)
        # Startup default so the camera points straight down (nadir) during the
        # whole outbound flight, matching the previous fixed-down camera.
        self.declare_parameter("startup_yaw_deg", 0.0)
        self.declare_parameter("startup_pitch_deg", -90.0)

        model = str(self.get_parameter("gz_model_name").value)
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.pitch_sign = float(self.get_parameter("pitch_sign").value)
        republish_hz = max(1.0, float(self.get_parameter("republish_hz").value))

        self.cmd_yaw_deg = float(self.get_parameter("startup_yaw_deg").value)
        self.cmd_pitch_deg = float(self.get_parameter("startup_pitch_deg").value)

        yaw_topic = f"/model/{model}/gimbal_yaw"
        pitch_topic = f"/model/{model}/gimbal_pitch"
        self.yaw_pub = self.create_publisher(Float64, yaw_topic, 10)
        self.pitch_pub = self.create_publisher(Float64, pitch_topic, 10)
        self.att_pub = self.create_publisher(Vector3Stamped, "/gimbal/attitude", 10)

        self.cmd_sub = self.create_subscription(
            Vector3, "/gimbal/angle_cmd", self._cmd_cb, 10
        )

        self.timer = self.create_timer(1.0 / republish_hz, self._tick)

        self.get_logger().info(
            "gimbal_relay up: /gimbal/angle_cmd(deg) -> %s , %s (rad); "
            "yaw_sign=%.0f pitch_sign=%.0f; startup yaw=%.0f pitch=%.0f"
            % (
                yaw_topic,
                pitch_topic,
                self.yaw_sign,
                self.pitch_sign,
                self.cmd_yaw_deg,
                self.cmd_pitch_deg,
            )
        )

    def _cmd_cb(self, msg: Vector3) -> None:
        self.cmd_yaw_deg = float(msg.x)
        self.cmd_pitch_deg = float(msg.y)
        self._publish_joint_cmds()

    def _publish_joint_cmds(self) -> None:
        y = Float64()
        y.data = math.radians(self.yaw_sign * self.cmd_yaw_deg)
        p = Float64()
        p.data = math.radians(self.pitch_sign * self.cmd_pitch_deg)
        self.yaw_pub.publish(y)
        self.pitch_pub.publish(p)

    def _tick(self) -> None:
        # Keepalive republish (the joint controller holds the last value, but a
        # steady stream survives any transient bridge drop) + attitude echo.
        self._publish_joint_cmds()

        att = Vector3Stamped()
        att.header.stamp = self.get_clock().now().to_msg()
        att.header.frame_id = "base_link"
        att.vector.x = self.cmd_yaw_deg
        att.vector.y = self.cmd_pitch_deg
        att.vector.z = 0.0
        self.att_pub.publish(att)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
