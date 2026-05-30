from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Rate + namespace (used by joint_controller)
    rate_arg = DeclareLaunchArgument('rate_hz', default_value='100.0',
                                     description='Controller publish/update rate (Hz)')
    ns_arg   = DeclareLaunchArgument('ns', default_value='',
                                     description='Optional ROS namespace')

    # Gaits YAML
    default_gaits = os.path.join(
        get_package_share_directory('crawler_tasks'), 'config', 'gaits_phased.yaml'
    )
    gaits_arg = DeclareLaunchArgument('gaits_file', default_value=default_gaits,
                                      description='Path to gaits.yaml')

    # Maestro YAML (installed with crawler_control)
    default_maestro = PathJoinSubstitution([
        FindPackageShare('crawler_control'), 'config', 'maestro.yaml'
    ])
    maestro_arg = DeclareLaunchArgument('maestro_cfg', default_value=default_maestro,
                                        description='Path to maestro.yaml')

    rate  = LaunchConfiguration('rate_hz')
    ns    = LaunchConfiguration('ns')
    gaits = LaunchConfiguration('gaits_file')
    mcfg  = LaunchConfiguration('maestro_cfg')

    joint_controller = Node(
        package='crawler_control',
        executable='joint_controller',
        name='joint_controller',
        namespace=ns,
        output='screen',
        respawn=True,
        parameters=[{'rate_hz': rate}],
    )

    maestro_bridge = Node(
        package='crawler_control',
        executable='maestro_bridge',
        name='maestro_bridge',
        namespace=ns,
        output='screen',
        respawn=True,
        parameters=[mcfg],
    )

    locomotion_server = Node(
        package='crawler_tasks',
        executable='locomotion_task_server',
        name='locomotion_task_server',
        namespace=ns,
        output='screen',
        respawn=True,
        parameters=[gaits],
    )
    
    relay_node = Node(
    package='crawler_control',
    executable='relay_goal_to_setpoint',
    name='relay_goal_to_setpoint',
    parameters=[{'rate_hz': 50.0}],
    output='screen'
    )

    reached_node = Node(
    package='crawler_control',
    executable='reached_from_commands',
    name='reached_from_commands',
    parameters=[
        {'rate_hz': 50.0, 'min_dwell_s': 0.05,
         'tol.J1': 0.002, 'tol.J2': 0.002,
         'tol.J3': 0.01,  'tol.J4': 0.01, 'tol.J5': 0.01, 'tol.J6': 0.01, 'tol.J7': 0.01}
    ],
    output='screen'
    )    

    return LaunchDescription([
        rate_arg, ns_arg, gaits_arg, maestro_arg,
        joint_controller,
        maestro_bridge,
        locomotion_server,
        relay_node,
        reached_node
    ])

