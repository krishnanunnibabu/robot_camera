from setuptools import setup
import os
from glob import glob

package_name = 'robot_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krish',
    maintainer_email='your_email@example.com',
    description='ROS2 Camera Publisher and Subscriber optimized for Raspberry Pi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = robot_camera.camera_node:main',
            'camera_subscriber = robot_camera.camera_subscriber:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
)
