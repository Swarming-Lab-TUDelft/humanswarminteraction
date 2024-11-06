"""
TU Delft Swarming Lab - Human Swarm Interaction project
Description: Launch Orbbec SDK and PoseNet node
Author: Alexander James Becoy
Date: 2024-11-06
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    orbbec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'ob_camera.launch.py')
        ),
        launch_arguments=[
            ('color_width', '1920'), ('color_height', '1080'), ('color_fps', '15'), ('color_format', 'MJPG'),
            ('depth_width', '640'), ('depth_height', '576'), ('depth_fps', '30'), ('depth_format', 'Y16'),
            ('ir_width', '640'), ('ir_height', '576'), ('ir_fps', '30'), ('ir_format', 'Y16'),
        ],
    )

    posenet_node = Node(
        package='posenet_orbbec',
        executable='posenet_node',
        name='posenet_node',
        output='screen'
    )

    return LaunchDescription([
        orbbec_node,
        posenet_node,
    ])
