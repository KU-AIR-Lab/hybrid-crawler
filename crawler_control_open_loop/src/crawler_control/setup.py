from setuptools import setup
import os

package_name = 'crawler_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # make sure crawler_control/__init__.py exists
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/crawler_control']),
        ('share/crawler_control', ['package.xml']),
        ('share/crawler_control/launch', ['launch/bringup.launch.py']),
        ('share/crawler_control/config', ['config/maestro.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Abdur Rosyid',
    maintainer_email='abdoorasheed@gmail.com',
    description='Joint setpoint fanout and serial bridges.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_controller       = crawler_control.joint_controller:main',
            'maestro_bridge         = crawler_control.maestro_bridge:main',
            # helpers to keep old joint control + new task-space in sync:
            'relay_goal_to_setpoint = crawler_control.relay_goal_to_setpoint:main',
            'reached_from_commands  = crawler_control.reached_from_commands:main',
            # 'arduino_bridge        = crawler_control.arduino_bridge:main',  # if you use it
        ],
    },
)

