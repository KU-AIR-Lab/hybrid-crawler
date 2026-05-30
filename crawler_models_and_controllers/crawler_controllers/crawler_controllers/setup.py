from setuptools import setup, find_packages
package_name = 'crawler_controllers'
setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(include=('crawler_controllers*',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['crawler_controllers/launch/controllers_with_config.launch.py']),
        ('share/' + package_name + '/config', [
            'crawler_controllers/config/all_controllers.yaml',
            'crawler_controllers/config/soft_force_pid.yaml',
            'crawler_controllers/config/cable_force_pid.yaml',
            'crawler_controllers/config/pump_pid.yaml',
            'crawler_controllers/config/pump_bangbang.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdur Rosyid',
    maintainer_email='abdoorasheed@gmail.com',
    description='Controllers (PID and bang-bang) with config folder; renamed pump PID files.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'soft_force_pid_node = crawler_controllers.nodes.soft_force_pid_node:main',
            'cable_force_pid_node = crawler_controllers.nodes.cable_force_pid_node:main',
            'pump_pid_controller = crawler_controllers.nodes.pump_pid_controller:main',
            'pump_bangbang_controller = crawler_controllers.nodes.pump_bangbang_controller:main',
        ],
    },
)
