from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = LaunchConfiguration('config')
    # parameters=[config] works if provided a path via --ros-args --params-file
    nodes = [
        Node(package='crawler_controllers', executable='soft_force_pid_node', name='soft_force_pid', parameters=[config]),
        Node(package='crawler_controllers', executable='cable_force_pid_node', name='cable_force_pid', parameters=[config]),
        Node(package='crawler_controllers', executable='pump_pid_controller', name='pump_pid_controller', parameters=[config]),
        Node(package='crawler_controllers', executable='pump_bangbang_controller', name='pump_bangbang_controller', parameters=[config]),
    ]
    return LaunchDescription([DeclareLaunchArgument('config', default_value=''), *nodes])
