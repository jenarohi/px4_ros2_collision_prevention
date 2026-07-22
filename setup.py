from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_ros2_collision_prevention'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='PX4 ROS2 Collision Prevention via RealSense D435',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_obstacle_node = '
            'px4_ros2_collision_prevention.realsense_obstacle_node:main',
            'depth_replay_node = '
            'px4_ros2_collision_prevention.depth_replay_node:main',
        ],
    },
)
