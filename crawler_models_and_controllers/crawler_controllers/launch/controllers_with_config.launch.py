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
    use_bangbang = LaunchConfiguration('use_bangbang')

    nodes = [
        # Unified cable force controller
        Node(package='crawler_controllers', executable='cable_force_pid_node', name='cable_force_pid', output='screen', parameters=[config]),

        # Front and rear soft force controllers
        Node(package='crawler_controllers', executable='soft_force_pid_node', name='soft_force_pid_front', output='screen', parameters=[config]),
        Node(package='crawler_controllers', executable='soft_force_pid_node', name='soft_force_pid_rear', output='screen', parameters=[config]),

        # Front and rear pressure command combiners
        Node(package='crawler_controllers', executable='soft_pressure_combiner_node', name='soft_pressure_combiner_front', output='screen', parameters=[config]),
        Node(package='crawler_controllers', executable='soft_pressure_combiner_node', name='soft_pressure_combiner_rear', output='screen', parameters=[config]),

        # Front and rear pump PID controllers
        Node(package='crawler_controllers', executable='pump_pid_controller', name='pump_pid_front', output='screen', parameters=[config]),
        Node(package='crawler_controllers', executable='pump_pid_controller', name='pump_pid_rear', output='screen', parameters=[config]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_bangbang', default_value='false', description='Currently PID pump nodes are launched by default; use manual launch for bang-bang if needed.'),
        *nodes,
    ])
