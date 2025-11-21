# my_robot/launch/lidar_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen'
        )
    ])
