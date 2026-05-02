from setuptools import setup
import os
from glob import glob

package_name = 'krac_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. [가중치] weights 폴더 안의 모든 .pt 파일을 설치 폴더로 복사
        (os.path.join('share', package_name, 'weights'), glob('weights/*.pt')),
        
        # 2. [추가] launch 폴더 안의 모든 .launch.py 파일을 설치 폴더로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kch',
    maintainer_email='kch@todo.todo',
    description='Package for drone vision using YOLOv8',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run krac_vision [이름] 명령어로 실행할 파일들 등록
            'survivor_yolo = krac_vision.survivor_yolo:main',
            'landing_marker = krac_vision.landing_marker:main',
            'yolo_node = krac_vision.yolo_detector:main', # 런치파일의 executable과 맞춤
            'gz_image_republisher = krac_vision.gz_image_republisher:main',
            'yolo_gazebo = krac_vision.yolo_gazebo:main',
        ],
    },
)
