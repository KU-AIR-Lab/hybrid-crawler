from setuptools import setup

package_name = 'contact_models'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/contact_models.launch.py']),
        ('share/' + package_name + '/config', ['config/contact_models.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdur Rosyid',
    maintainer_email='abdoorasheed@gmail.com',
    description='Contact models for unified cable and front/rear soft arm control.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cable_contact_model_node = contact_models.cable_contact_model_node:main',
            'soft_contact_model_node = contact_models.soft_contact_model_node:main',
        ],
    },
)
