import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('robotaksi_mapping')
    slam_config_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file, {'use_sim_time': True}],
    )

    return LaunchDescription([
        start_async_slam_toolbox_node
    ]) 