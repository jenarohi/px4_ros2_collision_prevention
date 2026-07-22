#!/usr/bin/env python3
"""
depth_replay.launch.py
======================
Launch file for the depth_replay_node.

Replays pre-recorded RealSense depth frames (depthdata_filter_N.xlsx)
as px4_msgs/ObstacleDistance messages to PX4 Collision Prevention.

Usage:
  ros2 launch px4_ros2_collision_prevention depth_replay.launch.py \
      data_dir:=/path/to/depth/frames

  ros2 launch px4_ros2_collision_prevention depth_replay.launch.py \
      data_dir:=/path/to/frames loop:=true publish_hz:=15.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('px4_ros2_collision_prevention')
    replay_params_file = os.path.join(pkg, 'config', 'replay_params.yaml')

    # ── Launch arguments (all overridable from command line) ─────────────────
    args = [
        DeclareLaunchArgument(
            'data_dir',
            default_value='',
            description='[REQUIRED] Path to folder containing depthdata_filter_*.xlsx files',
        ),
        DeclareLaunchArgument(
            'publish_hz',
            default_value='10.0',
            description='ObstacleDistance publish rate in Hz (default: 10.0)',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Loop the dataset indefinitely (default: false)',
        ),
        DeclareLaunchArgument(
            'fov_h_deg',
            default_value='87.0',
            description='Camera horizontal FOV in degrees (default: 87.0 for D435)',
        ),
        DeclareLaunchArgument(
            'min_depth_m',
            default_value='0.20',
            description='Minimum valid depth in metres (default: 0.20)',
        ),
        DeclareLaunchArgument(
            'max_depth_m',
            default_value='8.00',
            description='Maximum valid depth in metres (default: 8.00)',
        ),
        DeclareLaunchArgument(
            'num_bins',
            default_value='9',
            description='Angular bins across the camera FOV (default: 9)',
        ),
    ]

    node = Node(
        package    = 'px4_ros2_collision_prevention',
        executable = 'depth_replay_node',
        name       = 'depth_replay_node',
        output     = 'screen',
        parameters = [
            replay_params_file,
            {
                'data_dir':    LaunchConfiguration('data_dir'),
                'publish_hz':  LaunchConfiguration('publish_hz'),
                'loop':        LaunchConfiguration('loop'),
                'fov_h_deg':   LaunchConfiguration('fov_h_deg'),
                'min_depth_m': LaunchConfiguration('min_depth_m'),
                'max_depth_m': LaunchConfiguration('max_depth_m'),
                'num_bins':    LaunchConfiguration('num_bins'),
            },
        ],
    )

    return LaunchDescription(args + [node])
