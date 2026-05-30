from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('deformation_model'), 'config', 'deformation_model.yaml')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(package='deformation_model', executable='cable_ik_node', name='cable_ik_node', output='screen', parameters=[config]),
        Node(package='deformation_model', executable='soft_forward_statics_node', name='soft_forward_statics_front', output='screen', parameters=[config]),
        Node(package='deformation_model', executable='soft_forward_statics_node', name='soft_forward_statics_rear', output='screen', parameters=[config]),
    ])
