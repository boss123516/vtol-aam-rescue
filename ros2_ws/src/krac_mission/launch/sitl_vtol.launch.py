import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_px4_dir():
    return os.path.join(os.environ.get('HOME', ''), 'PX4-Autopilot')


def generate_launch_description():
    mission_pkg = 'krac_mission'
    control_pkg = 'krac_control'

    mission_share = get_package_share_directory(mission_pkg)
    control_share = get_package_share_directory(control_pkg)

    default_mavros_config = os.path.join(mission_share, 'config', 'mavros_params.yaml')
    default_waypoints_config = os.path.join(mission_share, 'config', 'waypoints.yaml')
    default_plan_file = os.path.join(control_share, 'mission', 'way3.plan')

    px4_dir = LaunchConfiguration('px4_dir')
    vehicle_model = LaunchConfiguration('vehicle_model')
    mavros_config = LaunchConfiguration('mavros_config')
    waypoints_config = LaunchConfiguration('waypoints_config')
    plan_file = LaunchConfiguration('plan_file')
    start_px4 = LaunchConfiguration('start_px4')
    start_mavros = LaunchConfiguration('start_mavros')
    start_fsm = LaunchConfiguration('start_fsm')
    start_mission_loader = LaunchConfiguration('start_mission_loader')
    start_logger = LaunchConfiguration('start_logger')

    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', vehicle_model],
        cwd=px4_dir,
        output='screen',
        condition=IfCondition(start_px4),
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[mavros_config],
        namespace='mavros',
        condition=IfCondition(start_mavros),
    )

    control_node = Node(
        package=control_pkg,
        executable='vtol_fsm_P',
        name='vtol_fsm_P',
        output='screen',
        parameters=[waypoints_config],
        condition=IfCondition(start_fsm),
    )

    mission_loader_node = Node(
        package=control_pkg,
        executable='mission_loader.py',
        name='mission_loader',
        output='screen',
        parameters=[{'plan_file': plan_file}],
        condition=IfCondition(start_mission_loader),
    )

    logger_node = Node(
        package='krac_utils',
        executable='competition_logger',
        name='competition_logger',
        output='screen',
        parameters=[waypoints_config],
        condition=IfCondition(start_logger),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=_default_px4_dir(),
            description='PX4-Autopilot root directory',
        ),
        DeclareLaunchArgument(
            'vehicle_model',
            default_value='gz_standard_vtol',
            description='PX4 SITL make target, e.g. gz_standard_vtol or gz_amsr_vtol',
        ),
        DeclareLaunchArgument(
            'mavros_config',
            default_value=default_mavros_config,
            description='MAVROS parameter YAML',
        ),
        DeclareLaunchArgument(
            'waypoints_config',
            default_value=default_waypoints_config,
            description='KRAC waypoint/mission parameter YAML',
        ),
        DeclareLaunchArgument(
            'plan_file',
            default_value=default_plan_file,
            description='QGroundControl .plan file to upload through MAVROS',
        ),
        DeclareLaunchArgument('start_px4', default_value='true'),
        DeclareLaunchArgument('start_mavros', default_value='true'),
        DeclareLaunchArgument('start_fsm', default_value='true'),
        DeclareLaunchArgument('start_mission_loader', default_value='true'),
        DeclareLaunchArgument('start_logger', default_value='true'),
        px4_sitl,
        mavros_node,
        control_node,
        mission_loader_node,
        logger_node,
    ])
