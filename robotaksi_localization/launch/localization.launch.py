from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Launch arguments
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [FindPackageShare('robotaksi_localization'),
             'maps',
             'map.yaml']),
        description='Full path to map yaml file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            'xacro ',
            PathJoinSubstitution(
                [FindPackageShare('robotaksi_description'), 'urdf', 'robotaksi.xacro']
            )
        ]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # AMCL parameters
    amcl_params = PathJoinSubstitution(
        [FindPackageShare('robotaksi_localization'),
         'config',
         'amcl_params.yaml'])

    # Start AMCL node
    start_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params,
                   {'use_sim_time': use_sim_time}],
        remappings=[('scan', 'scan'),
                   ('map', 'map')]
    )

    # Start map server
    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file,
                    'use_sim_time': use_sim_time}]
    )

    # Create and return launch description
    ld = LaunchDescription()
    
    # Add declared arguments
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_amcl_cmd)
    
    return ld 