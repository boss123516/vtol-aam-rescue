import cv2
import torch
import random
import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from cv_bridge import CvBridge
from ultralytics import YOLO

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class YoloDetector(Node):

    def __init__(self) -> None:
        super().__init__("yolo_detector_node")

        # 1. 파라미터 설정
        self.declare_parameter("model", "/home/boss/vtol-aam-rescue/ros2_ws/src/krac_vision/weights/best.pt")
        model_path = self.get_parameter("model").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value

        self.declare_parameter("device", "cpu")
        device = self.get_parameter("device").get_parameter_value().string_value

        self._class_to_color = {}
        self.cv_bridge = CvBridge()
        self.enable = True
        
        self.target_class = "" 

        # 2. 모델 로드
        try:
            self.yolo = YOLO(model_path)
            self.yolo.to(device)
            self.get_logger().info(f"YOLOv8 Model loaded: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Model load failed: {e}")

        # 3. Topic 설정
        self._pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)
        self._error_pub = self.create_publisher(Point, "camera/target_error", 10)
        
        self._target_sub = self.create_subscription(String, "camera/set_target", self.target_cb, 10)
        
        self._sub = self.create_subscription(
            Image, 
            "/image_raw", 
            self.image_cb, 
            qos_profile_sensor_data
        )

    def target_cb(self, msg: String) -> None:
        self.target_class = msg.data
        self.get_logger().info(f"Target Class Changed to: '{self.target_class}'")

    def image_cb(self, msg: Image) -> None:
        if not self.enable:
            return

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            return

        h, w, _ = cv_image.shape
        cx, cy = w // 2, h // 2

        # 화면 중심 십자가
        cv2.line(cv_image, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
        cv2.line(cv_image, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)

        # YOLO 추론
        results = self.yolo.track(source=cv_image, conf=self.threshold, persist=True, verbose=False)
        results = results[0].cpu()

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        best_box = None
        max_score = -1.0

        if results.boxes:
            for b in results.boxes:
                label_id = int(b.cls)
                label_name = self.yolo.names[label_id]
                score = float(b.conf)
                
                # 타겟 필터링
                if self.target_class != "" and label_name != self.target_class:
                    continue

                # [추가됨] 터미널에 탐지된 물체 정보 출력 (필터링 통과한 것만)
                self.get_logger().info(f"Detected: {label_name} (Score: {score:.2f})")

                box = b.xywh[0]

                # 메시지 생성
                detection = Detection2D()
                detection.bbox.center.position.x = float(box[0])
                detection.bbox.center.position.y = float(box[1])
                detection.bbox.size_x = float(box[2])
                detection.bbox.size_y = float(box[3])
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = label_name
                hypothesis.hypothesis.score = score
                detection.results.append(hypothesis)
                detections_msg.detections.append(detection)

                # 시각화
                color = self._get_color(label_name)
                min_pt = (int(box[0] - box[2]/2), int(box[1] - box[3]/2))
                max_pt = (int(box[0] + box[2]/2), int(box[1] + box[3]/2))
                cv2.rectangle(cv_image, min_pt, max_pt, color, 2)
                cv2.putText(cv_image, f"{label_name} {score:.2f}", (min_pt[0], min_pt[1]-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                if score > max_score:
                    max_score = score
                    best_box = box

        # 오차 계산 및 발행
        if best_box is not None:
            obj_x, obj_y = int(best_box[0]), int(best_box[1])
            error_x = float(obj_x - cx)
            error_y = float(obj_y - cy)

            err_msg = Point()
            err_msg.x = error_x
            err_msg.y = error_y
            err_msg.z = 1.0 
            self._error_pub.publish(err_msg)

            # 시각화
            cv2.circle(cv_image, (obj_x, obj_y), 5, (0, 0, 255), -1)
            cv2.line(cv_image, (cx, cy), (obj_x, obj_y), (0, 255, 255), 2)
            cv2.putText(cv_image, f"Target: {self.target_class}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            err_msg = Point()
            err_msg.x = 0.0
            err_msg.y = 0.0
            err_msg.z = 0.0
            self._error_pub.publish(err_msg)
            
            if self.target_class != "":
                 cv2.putText(cv_image, f"Searching: {self.target_class}...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self._pub.publish(detections_msg)
        self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

    def _get_color(self, label):
        if label not in self._class_to_color:
            self._class_to_color[label] = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
        return self._class_to_color[label]

def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
