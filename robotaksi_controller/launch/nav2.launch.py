from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Nav2 parameters
    nav2_params = PathJoinSubstitution(
        [FindPackageShare('robotaksi_controller'),
         'config',
         'nav2.yaml'])

    # Include localization launch file
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('robotaksi_localization'), 'launch', 'localization.launch.py']
            )]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Start Nav2 controller server
    start_nav2_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Start Nav2 planner server
    start_nav2_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Start Nav2 lifecycle manager for navigation
    start_navigation_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['planner_server',
                                  'controller_server',
                                  'behavior_server',
                                  'smoother_server',
                                  'bt_navigator',
                                  'waypoint_follower'],
                     }]
    )

    # RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('robotaksi_localization'),
             'rviz',
             'nav2.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    # Start Nav2 behavior server
    start_nav2_behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    # Start Nav2 bt_navigator
    start_nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    # Start Nav2 smoother
    start_nav2_smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Start Nav2 waypoint follower
    start_nav2_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params]
    )

    # Start the cmd_vel to motor command converter
    start_cmd_vel_converter = Node(
        package='robotaksi_controller',
        executable='cmd_vel_converter',
        name='cmd_vel_converter',
        output='screen'
    )

    # Create and return launch description
    ld = LaunchDescription()
    
    # Add declared arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add Nav2 nodes
    ld.add_action(localization_launch)
    ld.add_action(start_nav2_controller_server)
    ld.add_action(start_nav2_planner_server)
    ld.add_action(start_nav2_behavior_server)
    ld.add_action(start_nav2_bt_navigator)
    ld.add_action(start_nav2_smoother)
    ld.add_action(start_nav2_waypoint_follower)
    ld.add_action(start_navigation_lifecycle_manager)
    ld.add_action(start_cmd_vel_converter)
    ld.add_action(start_rviz_cmd)
    
    return ld 