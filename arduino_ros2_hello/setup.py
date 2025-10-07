from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arduino_ros2_hello'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyalarını ekle
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='gunes',
    maintainer_email='gunes@todo.todo',
    description='Arduino ROS2 Serial Bridge Package with RPLIDAR support',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = arduino_ros2_hello.arduino_bridge:main',
            'rplidar_arduino_bridge = arduino_ros2_hello.rplidar_arduino_bridge:main',
        ],
    },
)