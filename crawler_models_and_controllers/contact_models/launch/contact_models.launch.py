from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('contact_models'), 'config', 'contact_models.yaml')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(package='contact_models', executable='cable_contact_model_node', name='cable_contact_model', output='screen', parameters=[config]),
        Node(package='contact_models', executable='soft_contact_model_node', name='soft_contact_model_front', output='screen', parameters=[config]),
        Node(package='contact_models', executable='soft_contact_model_node', name='soft_contact_model_rear', output='screen', parameters=[config]),
    ])
