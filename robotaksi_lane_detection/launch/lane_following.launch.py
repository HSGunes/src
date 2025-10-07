#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
  
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera_rgb/image_raw',
        description='Camera topic for lane detection'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='10.0',
        description='Control command publishing rate (Hz)'
    )
    
    # Lane detection node
    lane_detection_node = Node(
        package='robotaksi_lane_detection',
        executable='lane_detection_node',
        name='lane_detection_node',
        output='screen',
        parameters=[{
            'kp': 1.2,
            'ki': 0.1,
            'kd': 0.4,
            'base_speed': 100.0,    # Set to exactly 100
            'max_speed': 120.0,     # Slightly higher max
            'min_speed': 50.0,      # Reasonable minimum
            'max_angular_velocity': 1.0,
            'enable_speed_control': True,
            'enable_temporal_smoothing': True,
            'enable_adaptive_threshold': True,
            'control_rate': LaunchConfiguration('control_rate'),
        }],
        remappings=[
            ('/camera_rgb/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
   
    return LaunchDescription([
        #serial_port_arg,
        camera_topic_arg,
        control_rate_arg,
        lane_detection_node,
        #serial_controller_node,
    ]) 