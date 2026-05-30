from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crawler_control',
            executable='joint_controller',
            name='joint_controller',
            output='screen',
            parameters=[{'rate_hz': 100.0}],
        ),
        Node(
            package='crawler_control',
            executable='maestro_bridge',
            name='maestro_bridge',
            output='screen',
            parameters=[
                # Loads $(share)/crawler_control/config/maestro.yaml after install
                PathJoinSubstitution([
                    FindPackageShare('crawler_control'),
                    'config',
                    'maestro.yaml'
                ])
            ],
        ),
        # If you ever need to go back to Arduino, comment the Maestro node above
        # and uncomment the block below:
        # Node(
        #     package='crawler_control',
        #     executable='arduino_bridge',
        #     name='arduino_bridge',
        #     output='screen',
        #     parameters=[{'port': '/dev/ttyACM0', 'baud': 115200, 'rate_hz': 100.0, 'watchdog_ms': 200}],
        # ),
    ])

