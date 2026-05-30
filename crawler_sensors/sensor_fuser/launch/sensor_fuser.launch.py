from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_fuser')
    cfg = os.path.join(pkg_share, 'config', 'sensor_fuser.yaml')
    return LaunchDescription([
        Node(package='sensor_fuser', executable='sensor_fuser', name='sensor_fuser', output='screen', parameters=[cfg])
    ])
