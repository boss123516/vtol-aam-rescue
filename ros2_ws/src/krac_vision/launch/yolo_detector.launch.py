import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 학습한 모델의 절대 경로 설정
    model_path = '/home/boss/vtol-aam-rescue/ros2_ws/src/krac_vision/weights/best.pt'

    return LaunchDescription([
        Node(
            package='krac_vision',
            executable='yolo_node', # setup.py에 등록될 이름
            name='yolo_detector_node',
            output='screen',
            parameters=[{
                'model': model_path,
                'threshold': 0.5,
                'device': 'cpu' # GPU 사용 시 'cuda:0'
            }],
            # Use default YOLO input topic: /image_raw
            remappings=[]
        )
    ])
