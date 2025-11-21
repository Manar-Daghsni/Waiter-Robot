# my_robot/launch/bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Packages
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_sensors_pkg = get_package_share_directory('robot_sensors')
    # Files
    urdf_file = os.path.join(robot_description_pkg, 'urdf', 'robot.urdf')
    rviz_config = os.path.join(robot_sensors_pkg, 'rviz', 'robot.rviz')
    

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # -----------------------
        # Sensors
        # -----------------------


        Node(
            package='robot_sensors',
            executable='sensor_node',
            name='sensor_node',
            parameters=[{
                'ignore_O_for_raw_odom': False,  # Publish odom directly
                'e_is_delta': False
            }],
            output='screen'
        ),

        # -----------------------
        # Robot description (URDF)
        # -----------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen'
        ),

        # -----------------------
        # Visualization (RViz2)
        # -----------------------
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
