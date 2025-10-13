import os
from os import pathsep

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('robotaksi_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'robotaksi.xacro')
    
    # Gazebo'nun kendi modellerini ve bizim modellerimizi bulabilmesi için ortam değişkenini ayarla
    # Bu, 'libgz_ros2_control.so' gibi eklentilerin bulunması için KRİTİKTİR.
    pkg_path = get_package_share_directory('robotaksi_description')
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_path, '..') + pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    robot_description_config = xacro.process_file(xacro_file, mappings={'use_sim_time': 'true'})
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': True}
        ]
    )

    world_path = PathJoinSubstitution([
        FindPackageShare('robotaksi_description'),
        'worlds',
        'robotaksi.world'
    ])
    
    # Standart Gazebo launch dosyasını, dünya dosyası argümanı ile çağır
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )

    # Gazebo ve ROS 2 arasında konuları köprüle
    # Bu, sensör verilerini (LIDAR, IMU, Kamera) ve saati ROS'a aktarır
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Lidar
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Camera
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    ld = LaunchDescription([
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_ros2_bridge
    ])

    return ld