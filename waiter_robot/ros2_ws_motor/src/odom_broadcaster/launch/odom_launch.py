from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      Node(package='robot_sensors', executable='sensor_node', output='screen'),
      Node(package='odom_broadcaster', executable='odom_broadcaster', output='screen'),
      Node(package='slam_toolbox',
           executable='sync_slam_toolbox_node',
           name='slam_toolbox',
           output='screen',
           parameters=[{'mode':'mapping'}]),
    ])

