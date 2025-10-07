from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directories
    rplidar_share = get_package_share_directory('rplidar_slam')
    arduino_share = get_package_share_directory('arduino_ros2_hello')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # RPLidar S2 node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'frame_id': 'laser',
            'serial_port': '/dev/ttyUSB0',
        }]
    )
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(rplidar_share, 'config'),
            '-configuration_basename', 'rplidar.lua'
        ]
    )
    
    # Occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.03']
    )
    
    # Path tracker node
    path_tracker_node = Node(
        package='rplidar_slam',
        executable='path_tracker.py',
        name='path_tracker',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Arduino Bridge node
    arduino_bridge_node = Node(
        package='arduino_ros2_hello',
        executable='rplidar_arduino_bridge',
        name='rplidar_arduino_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Static transforms
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )
    
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        rplidar_node,
        cartographer_node,
        occupancy_grid_node,
        path_tracker_node,
        arduino_bridge_node,
        base_to_laser_tf,
        odom_to_base_tf
    ]) 