from setuptools import setup

package_name = 'deformation_model'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/deformation_model.launch.py']),
        ('share/' + package_name + '/config', ['config/deformation_model.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdur Rosyid',
    maintainer_email='abdoorasheed@gmail.com',
    description='Cable IK and soft arm forward statics for unified control groups.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cable_ik_node = deformation_model.cable_ik_node:main',
            'soft_forward_statics_node = deformation_model.soft_forward_statics_node:main',
        ],
    },
)
