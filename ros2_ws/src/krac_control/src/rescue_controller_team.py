#!/usr/bin/env python3
"""
Control-team rescue controller template v1.

Purpose
-------
One shared controller used by both the standalone control-team BT and
the original split mission. Edit this file repeatedly during development.

Runtime sequence
----------------
enable=true
  -> publish local hold setpoints continuously
  -> ready=true
  -> AUTO.LAND
  -> confirm landing
  -> disarm
  -> wait 5 seconds on the ground
  -> pre-stream the original handoff position
  -> enter OFFBOARD and arm
  -> climb back to the original handoff height
  -> confirm stable hover
  -> result="SUCCESS"
  -> keep holding until enable=false

Important:
- MAVROS local pose/velocity/relative-altitude publishers use sensor-data QoS.
- SUCCESS is published only after the vehicle has actually climbed and
  stabilized. The return mission therefore starts from a safe airborne state.
"""

from __future__ import annotations

from copy import deepcopy
from enum import Enum, auto
from math import hypot
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64, String


MAV_LANDED_STATE_ON_GROUND = 1


class Phase(Enum):
    IDLE = auto()
    PRESTREAM = auto()
    LANDING = auto()
    GROUND_WAIT = auto()
    RESTART_PRESTREAM = auto()
    REARM_OFFBOARD = auto()
    CLIMB = auto()
    RESULT_SENT = auto()


class RescueControllerTeam(Node):
    def __init__(self) -> None:
        super().__init__("rescue_controller_team")

        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("handover_prestream_sec", 1.5)
        self.declare_parameter("restart_prestream_sec", 1.5)
        self.declare_parameter("landing_stable_sec", 1.0)
        self.declare_parameter("ground_wait_sec", 5.0)
        self.declare_parameter("climb_stable_sec", 1.0)
        self.declare_parameter("landing_timeout_sec", 120.0)
        self.declare_parameter("restart_timeout_sec", 30.0)
        self.declare_parameter("climb_timeout_sec", 45.0)
        self.declare_parameter("max_landed_altitude_m", 0.30)
        self.declare_parameter("max_landed_vertical_speed_mps", 0.20)
        self.declare_parameter("climb_z_tolerance_m", 0.35)
        self.declare_parameter("max_stable_vertical_speed_mps", 0.30)
        self.declare_parameter("max_stable_horizontal_speed_mps", 0.50)

        self.setpoint_rate_hz = max(
            10.0, float(self.get_parameter("setpoint_rate_hz").value)
        )
        self.handover_prestream_sec = max(
            1.0, float(self.get_parameter("handover_prestream_sec").value)
        )
        self.restart_prestream_sec = max(
            1.0, float(self.get_parameter("restart_prestream_sec").value)
        )
        self.landing_stable_sec = max(
            0.5, float(self.get_parameter("landing_stable_sec").value)
        )
        self.ground_wait_sec = max(
            0.0, float(self.get_parameter("ground_wait_sec").value)
        )
        self.climb_stable_sec = max(
            0.5, float(self.get_parameter("climb_stable_sec").value)
        )
        self.landing_timeout_sec = max(
            10.0, float(self.get_parameter("landing_timeout_sec").value)
        )
        self.restart_timeout_sec = max(
            10.0, float(self.get_parameter("restart_timeout_sec").value)
        )
        self.climb_timeout_sec = max(
            10.0, float(self.get_parameter("climb_timeout_sec").value)
        )
        self.max_landed_altitude_m = float(
            self.get_parameter("max_landed_altitude_m").value
        )
        self.max_landed_vertical_speed_mps = float(
            self.get_parameter("max_landed_vertical_speed_mps").value
        )
        self.climb_z_tolerance_m = float(
            self.get_parameter("climb_z_tolerance_m").value
        )
        self.max_stable_vertical_speed_mps = float(
            self.get_parameter("max_stable_vertical_speed_mps").value
        )
        self.max_stable_horizontal_speed_mps = float(
            self.get_parameter("max_stable_horizontal_speed_mps").value
        )

        # BT handshake topics use reliable/default QoS.
        self.enable_sub = self.create_subscription(
            Bool,
            "/krac/rescue_module/enable",
            self._enable_cb,
            10,
        )
        self.ready_pub = self.create_publisher(
            Bool, "/krac/rescue_module/ready", 10
        )
        self.result_pub = self.create_publisher(
            String, "/krac/rescue_module/result", 10
        )

        # MAVROS telemetry uses sensor-data QoS (BEST_EFFORT).
        # The previous control-team controller used default RELIABLE subscriptions, so it
        # never received pose/velocity/rel_alt and never sent ready=true.
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_cb,
            qos_profile_sensor_data,
        )
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self._velocity_cb,
            qos_profile_sensor_data,
        )
        self.rel_alt_sub = self.create_subscription(
            Float64,
            "/mavros/global_position/rel_alt",
            self._rel_alt_cb,
            qos_profile_sensor_data,
        )
        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self._extended_state_cb,
            qos_profile_sensor_data,
        )
        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            qos_profile_sensor_data,
        )

        self.hold_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self.set_mode_client = self.create_client(
            SetMode, "/mavros/set_mode"
        )
        self.arming_client = self.create_client(
            CommandBool, "/mavros/cmd/arming"
        )

        self.phase = Phase.IDLE
        self.active = False
        self.latest_pose: Optional[PoseStamped] = None
        self.handoff_pose: Optional[PoseStamped] = None
        self.command_pose: Optional[PoseStamped] = None

        self.relative_altitude_m = float("inf")
        self.vertical_speed_mps = float("inf")
        self.horizontal_speed_mps = float("inf")
        self.landed_state = 0
        self.current_mode = ""
        self.armed = False

        self.phase_started_ns = 0
        self.stable_started_ns = 0
        self.last_mode_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_disarm_request_ns = 0
        self.last_diag_ns = 0
        self.phase_setpoint_count = 0

        self.timer = self.create_timer(
            1.0 / self.setpoint_rate_hz, self._tick
        )

        self.get_logger().info(
            "Control-team rescue controller ready: land -> wait 5 s -> "
            "OFFBOARD climb -> stable hover -> SUCCESS."
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _elapsed_sec(self, started_ns: int) -> float:
        if started_ns <= 0:
            return 0.0
        return (self._now_ns() - started_ns) / 1e9

    def _set_phase(self, phase: Phase) -> None:
        self.phase = phase
        self.phase_started_ns = self._now_ns()
        self.stable_started_ns = 0
        self.phase_setpoint_count = 0

    def _enable_cb(self, msg: Bool) -> None:
        if msg.data:
            if not self.active:
                self._start_request()
            return

        if self.active:
            self.get_logger().info(
                "enable=false received: stopping control-team setpoints."
            )
        self._reset_idle()

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _velocity_cb(self, msg: TwistStamped) -> None:
        self.vertical_speed_mps = msg.twist.linear.z
        self.horizontal_speed_mps = hypot(
            msg.twist.linear.x, msg.twist.linear.y
        )

    def _rel_alt_cb(self, msg: Float64) -> None:
        self.relative_altitude_m = msg.data

    def _extended_state_cb(self, msg: ExtendedState) -> None:
        self.landed_state = int(msg.landed_state)

    def _state_cb(self, msg: State) -> None:
        self.current_mode = msg.mode
        self.armed = bool(msg.armed)

    def _start_request(self) -> None:
        self.active = True
        self.handoff_pose = (
            deepcopy(self.latest_pose)
            if self.latest_pose is not None
            else None
        )
        self.command_pose = (
            deepcopy(self.handoff_pose)
            if self.handoff_pose is not None
            else None
        )
        self.last_mode_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_disarm_request_ns = 0
        self._set_phase(Phase.PRESTREAM)
        self.get_logger().info(
            "enable=true: starting external hold pre-stream."
        )

    def _reset_idle(self) -> None:
        self.active = False
        self.phase = Phase.IDLE
        self.handoff_pose = None
        self.command_pose = None
        self.phase_started_ns = 0
        self.stable_started_ns = 0
        self.phase_setpoint_count = 0

    def _publish_setpoint(self) -> None:
        if self.command_pose is None and self.latest_pose is not None:
            self.command_pose = deepcopy(self.latest_pose)

        if self.command_pose is None:
            return

        msg = deepcopy(self.command_pose)
        msg.header.stamp = self.get_clock().now().to_msg()
        if not msg.header.frame_id:
            msg.header.frame_id = "map"
        self.hold_pub.publish(msg)
        self.phase_setpoint_count += 1

    def _publish_ready(self) -> None:
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)
        self.get_logger().info(
            "ready=true: external setpoint stream active."
        )

    def _publish_result(self, result: str) -> None:
        msg = String()
        msg.data = result
        self.result_pub.publish(msg)
        self.get_logger().info("result=%s" % result)
        self._set_phase(Phase.RESULT_SENT)

    def _service_period_elapsed(self, last_ns: int, period: float = 1.0) -> bool:
        return last_ns <= 0 or (self._now_ns() - last_ns) / 1e9 >= period

    def _request_mode(self, mode: str) -> None:
        if not self._service_period_elapsed(self.last_mode_request_ns):
            return
        if not self.set_mode_client.service_is_ready():
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        self.set_mode_client.call_async(request)
        self.last_mode_request_ns = self._now_ns()
        self.get_logger().info(
            "Requested mode=%s current=%s" % (mode, self.current_mode)
        )

    def _request_arm(self, arm: bool) -> None:
        last_ns = (
            self.last_arm_request_ns if arm
            else self.last_disarm_request_ns
        )
        if not self._service_period_elapsed(last_ns):
            return
        if not self.arming_client.service_is_ready():
            return

        request = CommandBool.Request()
        request.value = arm
        self.arming_client.call_async(request)
        if arm:
            self.last_arm_request_ns = self._now_ns()
        else:
            self.last_disarm_request_ns = self._now_ns()
        self.get_logger().info(
            "Requested arm=%s current=%s"
            % (str(arm).lower(), str(self.armed).lower())
        )

    def _landing_condition(self) -> bool:
        px4_grounded = self.landed_state == MAV_LANDED_STATE_ON_GROUND
        kinematic_grounded = (
            self.relative_altitude_m <= self.max_landed_altitude_m
            and abs(self.vertical_speed_mps)
            <= self.max_landed_vertical_speed_mps
        )
        return px4_grounded or kinematic_grounded

    def _stable_for(self, condition: bool, duration_sec: float) -> bool:
        if not condition:
            self.stable_started_ns = 0
            return False
        if self.stable_started_ns == 0:
            self.stable_started_ns = self._now_ns()
        return self._elapsed_sec(self.stable_started_ns) >= duration_sec

    def _diag(self, text: str) -> None:
        if self._service_period_elapsed(self.last_diag_ns, 2.0):
            self.last_diag_ns = self._now_ns()
            self.get_logger().info(text)

    # =====================================================================
    # CONTROL / VISION TEAM EDIT ZONE
    # ---------------------------------------------------------------------
    # 현재 기본 동작:
    #   AUTO.LAND -> 착륙 확인 -> 5초 대기 -> OFFBOARD 재이륙
    #   -> 인계 고도 안정화 -> SUCCESS
    #
    # 실제 구조 알고리즘으로 교체할 때 이 _tick() 상태 머신 내부를
    # 비전 검출/정렬/하강/파지/검증/상승 로직으로 수정하면 된다.
    # 아래 ROS 인터페이스는 유지할 것:
    #   subscribe /krac/rescue_module/enable  (Bool)
    #   publish   /krac/rescue_module/ready   (Bool)
    #   publish   /krac/rescue_module/result  (String)
    # SUCCESS는 구조 작업과 상승 및 안정 hover가 모두 끝난 뒤 보낸다.
    # =====================================================================

    def _tick(self) -> None:
        if not self.active:
            return

        # Continuous stream for seamless BT <-> external-controller handoff.
        self._publish_setpoint()

        if self.phase == Phase.PRESTREAM:
            if self.handoff_pose is None and self.latest_pose is not None:
                self.handoff_pose = deepcopy(self.latest_pose)
                self.command_pose = deepcopy(self.latest_pose)

            required = int(
                self.setpoint_rate_hz * self.handover_prestream_sec
            )
            if self.command_pose is not None and self.phase_setpoint_count >= required:
                self._publish_ready()
                self.last_mode_request_ns = 0
                self._set_phase(Phase.LANDING)
                self._request_mode("AUTO.LAND")
            else:
                self._diag(
                    "Waiting for valid MAVROS pose/setpoint pre-stream: "
                    "pose=%s samples=%d/%d"
                    % (
                        "yes" if self.command_pose is not None else "no",
                        self.phase_setpoint_count,
                        required,
                    )
                )
            return

        if self.phase == Phase.LANDING:
            if self.current_mode != "AUTO.LAND":
                self._request_mode("AUTO.LAND")

            if self._stable_for(
                self._landing_condition(),
                self.landing_stable_sec,
            ):
                self._set_phase(Phase.GROUND_WAIT)
                self.get_logger().info(
                    "Landing confirmed: landed_state=%d alt=%.2f "
                    "vz=%.2f. Waiting %.1f seconds."
                    % (
                        self.landed_state,
                        self.relative_altitude_m,
                        self.vertical_speed_mps,
                        self.ground_wait_sec,
                    )
                )
                self._request_arm(False)
                return

            if self._elapsed_sec(self.phase_started_ns) > self.landing_timeout_sec:
                self._publish_result("FAILURE:landing_timeout")
            return

        if self.phase == Phase.GROUND_WAIT:
            if self.armed:
                self._request_arm(False)

            if self._elapsed_sec(self.phase_started_ns) >= self.ground_wait_sec:
                if self.handoff_pose is None:
                    self._publish_result("FAILURE:handoff_pose_missing")
                    return

                # Return to the exact position/height at which BT handed over.
                self.command_pose = deepcopy(self.handoff_pose)
                self.last_mode_request_ns = 0
                self.last_arm_request_ns = 0
                self._set_phase(Phase.RESTART_PRESTREAM)
                self.get_logger().info(
                    "Ground wait complete. Pre-streaming climb target "
                    "z=%.2f m."
                    % self.command_pose.pose.position.z
                )
            return

        if self.phase == Phase.RESTART_PRESTREAM:
            required = int(
                self.setpoint_rate_hz * self.restart_prestream_sec
            )
            if self.phase_setpoint_count >= required:
                self.last_mode_request_ns = 0
                self.last_arm_request_ns = 0
                self._set_phase(Phase.REARM_OFFBOARD)
            return

        if self.phase == Phase.REARM_OFFBOARD:
            if self.current_mode != "OFFBOARD":
                self._request_mode("OFFBOARD")
            if not self.armed:
                self._request_arm(True)

            if self.current_mode == "OFFBOARD" and self.armed:
                self._set_phase(Phase.CLIMB)
                self.get_logger().info(
                    "OFFBOARD and armed. Climbing to handoff height."
                )
                return

            if self._elapsed_sec(self.phase_started_ns) > self.restart_timeout_sec:
                self._publish_result("FAILURE:restart_timeout")
            return

        if self.phase == Phase.CLIMB:
            if self.current_mode != "OFFBOARD":
                self._request_mode("OFFBOARD")

            if self.latest_pose is None or self.command_pose is None:
                climb_ok = False
            else:
                z_error = abs(
                    self.latest_pose.pose.position.z
                    - self.command_pose.pose.position.z
                )
                climb_ok = (
                    z_error <= self.climb_z_tolerance_m
                    and abs(self.vertical_speed_mps)
                    <= self.max_stable_vertical_speed_mps
                    and self.horizontal_speed_mps
                    <= self.max_stable_horizontal_speed_mps
                )

            if self._stable_for(climb_ok, self.climb_stable_sec):
                self.get_logger().info(
                    "Climb and hover confirmed: z=%.2f target=%.2f "
                    "vz=%.2f vxy=%.2f."
                    % (
                        self.latest_pose.pose.position.z,
                        self.command_pose.pose.position.z,
                        self.vertical_speed_mps,
                        self.horizontal_speed_mps,
                    )
                )
                self._publish_result("SUCCESS")
                return

            if self._elapsed_sec(self.phase_started_ns) > self.climb_timeout_sec:
                self._publish_result("FAILURE:climb_timeout")
            return

        # RESULT_SENT: keep the final command setpoint alive until BT publishes
        # enable=false after restoring its own hold stream.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RescueControllerTeam()
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
