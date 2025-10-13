import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- Paths ---
    pkg_robotaksi_controller = get_package_share_directory('robotaksi_controller')
    pkg_robotaksi_description = get_package_share_directory('robotaksi_description')

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Simulation ---
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robotaksi_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- Controller Config ---
    controllers_yaml = os.path.join(pkg_robotaksi_controller, 'config', 'controllers.yaml')

    # Robotu Gazebo'da oluştur (spawn)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'robotaksi',
                   '-z', '0.3'],
        output='screen'
    )

    # --- Controller Spawners ---
    # `spawn_entity` düğümü bittiğinde kontrolcüleri başlat.
    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            # Bu dosya içinde tanımladığımız `spawn_entity` düğümünü hedef alıyoruz.
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'))
    ld.add_action(simulation_launch)
    ld.add_action(spawn_entity)
    ld.add_action(spawn_controllers) # Bu, spawner'ları doğru zamanda başlatacak.

    return ld 