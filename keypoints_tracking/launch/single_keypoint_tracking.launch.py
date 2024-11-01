"""
TU Delft Swarm Lab - Human Swarm Interaction Project
Description: Track and filter a single keypoint linearly.
Author: Alexander James Becoy
Date: 2024-10-28
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the launch file directory
    keypoint_tracking_dir = get_package_share_directory('keypoints_tracking')

    # Define the path to the config file
    config_file = os.path.join(keypoint_tracking_dir, 'config', 'single_keypoint_tracking_params.yaml')

    return LaunchDescription([
        Node(
            package='keypoints_tracking',
            executable='single_keypoint_tracking_node',
            name='single_keypoint_tracking_node',
            output='screen',
            parameters=[config_file]
        )
    ])
