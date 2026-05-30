from setuptools import setup
package_name = 'sensor_fuser'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sensor_fuser.yaml']),
        ('share/' + package_name + '/launch', ['launch/sensor_fuser.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdur Rosyid',
    maintainer_email='abdoorasheed@gmail.com',
    description='Fuses raw sensor arrays into scalar feedback topics for controllers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['sensor_fuser = sensor_fuser.sensor_fuser_node:main']},
)
