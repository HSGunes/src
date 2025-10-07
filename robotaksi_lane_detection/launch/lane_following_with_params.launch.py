#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robotaksi_lane_detection')
    params_file = os.path.join(pkg_dir, 'config', 'lane_following_params.yaml')
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/camera/color/image_raw',
        description='Camera topic for lane detection'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to parameters file'
    )
    
    # Lane detection node
    lane_detection_node = Node(
        package='robotaksi_lane_detection',
        executable='lane_detection_node',
        name='lane_detection_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/camera/camera/color/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    

    return LaunchDescription([
        camera_topic_arg,
        params_file_arg,
        lane_detection_node,
    ]) 