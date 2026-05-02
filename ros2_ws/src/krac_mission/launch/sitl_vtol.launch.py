import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'krac_mission' # 패키지 이름 확인 필요 (설정파일 위치)
    control_package_name = 'krac_control' # 제어 노드 패키지 이름
    home_dir = os.environ.get('HOME')
    px4_dir = os.path.join(home_dir, 'PX4-Autopilot')

    # 1. 설정 파일 경로들
    mavros_config = os.path.join(
        get_package_share_directory(package_name), 'config', 'mavros_params.yaml'
    )

    waypoints_config = os.path.join(
        get_package_share_directory(package_name), 'config', 'waypoints.yaml'
    )

    # 2. PX4 + Gazebo 실행
    vehicle_model = 'gz_amsr_vtol'
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', vehicle_model],
        cwd=px4_dir,
        output='screen'
    )

    # 3. MAVROS 노드
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[mavros_config],
        namespace='mavros'
    )

    # 4. 제어 노드 (vtol_fsm_P 실행)
    # [설명] 하이브리드 핸드오버 로직이 들어있는 메인 제어 노드
    control_node = Node(
        package=control_package_name,
        executable='vtol_fsm_P', 
        output='screen',
        parameters=[waypoints_config]
    )
    
    # 5. [수정됨] 미션 로더 노드 (변수에 할당)
    # .plan 파일을 읽어서 PX4에 업로드하는 역할
    mission_loader_node = Node(
        package=control_package_name,
        executable='mission_loader.py', # CMakeLists.txt install 확인 필수
        name='mission_loader',
        output='screen',
        parameters=[{'plan_file': '/home/boss/vtol-aam-rescue/ros2_ws/src/krac_control/src/way3.plan'}]
    )
    
    # 6. 데이터 로거 노드
    logger_node = Node(
        package='krac_utils',
        executable='competition_logger',
        output='screen',
        parameters=[waypoints_config]
    )
    
    # 7. 실행 목록 반환
    return LaunchDescription([
        px4_sitl,
        mavros_node,
        control_node,
        mission_loader_node, # [중요] 여기에 추가되어야 실제로 실행됩니다.
        logger_node
    ])
