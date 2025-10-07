from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Arduino Bridge node (RPLIDAR olmadan)
    arduino_bridge_node = Node(
        package='arduino_ros2_hello',
        executable='rplidar_arduino_bridge',
        name='rplidar_arduino_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Static transforms (basit)
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
        arduino_bridge_node,
        base_to_laser_tf,
        odom_to_base_tf
    ]) 