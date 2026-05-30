from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('crawler_controllers'),
        'config',
        'all_controllers.yaml'
    )
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(package='crawler_controllers', executable='soft_force_pid_node', name='soft_force_pid', output='screen', parameters=[config]),
        Node(package='crawler_controllers', executable='cable_force_pid_node', name='cable_force_pid', output='screen', parameters=[config]),
        Node(package='crawler_controllers', executable='soft_pressure_combiner_node', name='soft_pressure_combiner', output='screen', parameters=[config]),
        Node(package='crawler_controllers', executable='pump_pid_controller', name='pump_pid_controller', output='screen', parameters=[config]),
        Node(package='crawler_controllers', executable='pump_bangbang_controller', name='pump_bangbang_controller', output='screen', parameters=[config]),
    ])
