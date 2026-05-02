#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import time
import os
from mavros_msgs.srv import WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint

class MissionLoader(Node):
    def __init__(self):
        super().__init__('mission_loader')
        
        # .plan 파일 경로 파라미터
        self.declare_parameter('plan_file', '')
        plan_path = self.get_parameter('plan_file').value

        self.client_wp_push = self.create_client(WaypointPush, 'mavros/mission/push')
        self.client_wp_clear = self.create_client(WaypointClear, 'mavros/mission/clear')

        if not plan_path or not os.path.exists(plan_path):
            self.get_logger().error(f"Plan file not found: {plan_path}")
            return

        self.upload_mission(plan_path)

    def upload_mission(self, file_path):
        # 1. 서비스 대기
        self.get_logger().info("Waiting for MAVROS services...")
        self.client_wp_push.wait_for_service()
        self.client_wp_clear.wait_for_service()

        # 2. 기존 미션 클리어
        req_clear = WaypointClear.Request()
        future_clear = self.client_wp_clear.call_async(req_clear)
        rclpy.spin_until_future_complete(self, future_clear)
        self.get_logger().info("Old mission cleared.")
        time.sleep(1)

        # 3. .plan 파일 파싱
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load JSON: {e}")
            return

        mission_items = data['mission']['items']
        waypoints = []

        # 4. Waypoint 메시지 변환
        for i, item in enumerate(mission_items):
            wp = Waypoint()
            
            # QGC .plan 파일 구조: params = [p1, p2, p3, p4, lat, lon, alt]
            # null 값은 0.0으로 변환
            params = [0.0 if p is None else float(p) for p in item['params']]

            wp.frame = item.get('frame', 3) # 기본값: GLOBAL_REL_ALT (3)
            wp.command = item['command']
            wp.is_current = (i == 0)      # 첫 번째만 True
            wp.autocontinue = True
            
            # 파라미터 매핑
            wp.param1 = params[0]
            wp.param2 = params[1]
            wp.param3 = params[2]
            wp.param4 = params[3]
            wp.x_lat = params[4]
            wp.y_long = params[5]
            wp.z_alt = params[6]

            waypoints.append(wp)

        # 5. 미션 전송
        req_push = WaypointPush.Request()
        req_push.start_index = 0
        req_push.waypoints = waypoints

        self.get_logger().info(f"Uploading {len(waypoints)} items from .plan file...")
        future_push = self.client_wp_push.call_async(req_push)
        rclpy.spin_until_future_complete(self, future_push)

        if future_push.result().success:
            self.get_logger().info("✅ Mission Upload SUCCESS!")
        else:
            self.get_logger().error(f"❌ Mission Upload FAILED! Sent: {future_push.result().wp_transfered}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionLoader()
    # 업로드만 하고 노드는 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
