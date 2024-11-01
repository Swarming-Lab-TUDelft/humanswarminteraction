"""
TU Delft Swarm Lab - Human Swarm Interaction Project
Description: Track and filter a single keypoint linearly.
Author: Alexander James Becoy
Date: 2024-10-28
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Get the launch file directory
    keypoint_tracking_dir = get_package_share_directory('keypoints_tracking')
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')

    # Define the path to the config file
    config_file = os.path.join(keypoint_tracking_dir, 'config', 'single_keypoint_tracking_params.yaml')

    # Path to the Orbbec camera launch file
    orbbec_camera_launch = os.path.join(orbbec_camera_dir, 'launch', 'ob_camera.launch.py')

    return LaunchDescription([
        # Launch single keypoint tracking node
        Node(
            package='keypoints_tracking',
            executable='single_keypoint_tracking_node',
            name='single_keypoint_tracking_node',
            output='screen',
            parameters=[config_file]
        ),
        
        # Include the Orbbec camera launch file with specified arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_camera_launch),
            launch_arguments={
                'color_width': '1920',
                'color_height': '1080',
                'color_fps': '15',
                'color_format': 'MJPG',
                'depth_width': '640',
                'depth_height': '576',
                'depth_fps': '30',
                'depth_format': 'Y16',
                'ir_width': '640',
                'ir_height': '576',
                'ir_fps': '30',
                'ir_format': 'Y16'
            }.items()
        ),

        # Run PoseNet node
        Node(
            package='posenet_orbbec',
            executable='posenet_node',
            name='posenet_node',
            output='screen'
        )
    ])
