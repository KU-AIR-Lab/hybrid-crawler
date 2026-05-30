from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('teensy_serial_bridge')
    cfg = os.path.join(pkg_share, 'config', 'teensy_serial_bridge.yaml')
    return LaunchDescription([
        Node(package='teensy_serial_bridge', executable='teensy_serial_bridge', name='teensy_serial_bridge', output='screen', parameters=[cfg])
    ])
