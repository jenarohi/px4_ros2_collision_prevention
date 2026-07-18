#!/usr/bin/env python3
"""
Launch file: px4_ros2_collision_prevention
==========================================
Starts the realsense_obstacle_node with parameters loaded from config/params.yaml.

Usage:
  ros2 launch px4_ros2_collision_prevention obstacle_avoidance.launch.py
  ros2 launch px4_ros2_collision_prevention obstacle_avoidance.launch.py simulate_depth:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('px4_ros2_collision_prevention')
    params_file = os.path.join(pkg, 'config', 'params.yaml')

    # Allow overriding simulate_depth from command line
    simulate_arg = DeclareLaunchArgument(
        'simulate_depth',
        default_value='false',
        description='Set true to inject synthetic obstacle (SITL testing)',
    )

    publish_hz_arg = DeclareLaunchArgument(
        'publish_hz',
        default_value='15.0',
        description='ObstacleDistance publish rate in Hz',
    )

    node = Node(
        package='px4_ros2_collision_prevention',
        executable='realsense_obstacle_node',
        name='realsense_obstacle_node',
        output='screen',
        parameters=[
            params_file,
            {
                'simulate_depth': LaunchConfiguration('simulate_depth'),
                'publish_hz': LaunchConfiguration('publish_hz'),
            },
        ],
        # Remapping is not needed — topic name matches PX4 default.
        # If your DDS namespace is different, add remappings here:
        # remappings=[('/fmu/in/obstacle_distance', '/fmu/in/obstacle_distance')],
    )

    return LaunchDescription([
        simulate_arg,
        publish_hz_arg,
        node,
    ])
