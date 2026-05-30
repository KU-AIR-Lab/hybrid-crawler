from setuptools import setup
package_name = 'teensy_serial_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/teensy_serial_bridge.yaml']),
        ('share/' + package_name + '/launch', ['launch/teensy_serial_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdur Rosyid',
    maintainer_email='abdoorasheed@gmail.com',
    description='Reads 18 pressure sensors from Teensy over USB and publishes ROS arrays.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['teensy_serial_bridge = teensy_serial_bridge.teensy_serial_bridge_node:main']},
)
