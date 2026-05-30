from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('crawler_tasks'), 'config', 'gaits_phased.yaml')
    return LaunchDescription([
        Node(package='crawler_tasks', executable='locomotion_task_server',
             name='locomotion_task_server',
             parameters=[cfg]),
    ])
