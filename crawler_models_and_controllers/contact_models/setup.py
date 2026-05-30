
from setuptools import setup
package_name = 'contact_models'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/contact_models.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdur Rosyid',
    maintainer_email='abdoorasheed@gmail.com',
    description='Contact models for cable and soft arms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ]
    },
)
