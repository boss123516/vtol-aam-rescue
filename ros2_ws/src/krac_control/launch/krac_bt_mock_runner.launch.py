from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_xml = PathJoinSubstitution([
        FindPackageShare('krac_control'),
        'bt',
        'krac_mission_bt_robust.xml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('bt_xml_path', default_value=default_xml),
        DeclareLaunchArgument('tick_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('max_ticks', default_value='0'),
        DeclareLaunchArgument('verbose_mock_ticks', default_value='false'),
        DeclareLaunchArgument('default_action_ticks', default_value='1'),
        DeclareLaunchArgument('fail_node', default_value=''),
        DeclareLaunchArgument('fail_after_tick', default_value='0'),
        DeclareLaunchArgument('fail_once', default_value='true'),
        DeclareLaunchArgument('emergency_at_tick', default_value='0'),
        Node(
            package='krac_control',
            executable='krac_bt_mock_runner',
            name='krac_bt_mock_runner',
            output='screen',
            parameters=[{
                'bt_xml_path': LaunchConfiguration('bt_xml_path'),
                'tick_rate_hz': LaunchConfiguration('tick_rate_hz'),
                'max_ticks': LaunchConfiguration('max_ticks'),
                'verbose_mock_ticks': LaunchConfiguration('verbose_mock_ticks'),
                'default_action_ticks': LaunchConfiguration('default_action_ticks'),
                'fail_node': LaunchConfiguration('fail_node'),
                'fail_after_tick': LaunchConfiguration('fail_after_tick'),
                'fail_once': LaunchConfiguration('fail_once'),
                'emergency_at_tick': LaunchConfiguration('emergency_at_tick'),
            }]
        )
    ])