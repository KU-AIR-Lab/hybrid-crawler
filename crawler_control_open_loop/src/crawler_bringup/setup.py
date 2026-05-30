from setuptools import setup

package_name = 'crawler_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/maestro.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdur Rosyid',
    maintainer_email='abdoorasheed@gmail.com',
    description='Top-level bringup launch for my robot.',
    license='Apache-2.0',
)
