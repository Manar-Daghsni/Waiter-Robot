from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    nav2_params = "/home/adminrobot/ros2_ws_lidar/src/my_nav2/config/nav2_params.yaml"

    # try find default nav2 rviz config path (installed with nav2_bringup)
    default_rviz = "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
    rviz_config = default_rviz if os.path.exists(default_rviz) else ""

    nodes = []

    # map server
    nodes.append(Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": "/home/adminrobot/ros2_ws_lidar/src/maps/map.yaml", "use_sim_time": False}]
    ))

    # amcl
    nodes.append(Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params]
    ))

    # planner_server
    nodes.append(Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params]
    ))

    # controller_server
    nodes.append(Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params]
    ))

    # smoother_server
    nodes.append(Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params]
    ))

    # behavior_server
    nodes.append(Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params]
    ))

    # bt_navigator
    nodes.append(Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params]
    ))

    # after bt_navigator
    nodes.append(
        ExecuteProcess(
            cmd=['python3', '/home/adminrobot/ros2_ws_lidar/src/scripts/amcl_initializer.py', 'cafemocha_map'],
            output='screen'
        )
    )

    # lifecycle manager
    nodes.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "map_server",
                "amcl",
                "planner_server",
                "controller_server",
                "smoother_server",
                "behavior_server",
                "bt_navigator"
            ]
        }]
    ))
    nodes.append(Node(
        package="robot_call",
        executable="ble_scanner",
        name="ble_scanner",
        output="screen"
    ))
    nodes.append(Node(
        package="robot_call",
        executable="mqtt_bridge",
        name="mqtt_bridge",
        output="screen"
    ))

    nodes.append(Node(
        package="robot_call",
        executable="call_manager",
        name="call_manager",
        output="screen"
    ))

    # rviz2 node
    rviz_node_kwargs = {
        "package": "rviz2",
        "executable": "rviz2",
        "name": "rviz2",
        "output": "screen",
        "arguments": []
    }
    if rviz_config:
        rviz_node_kwargs["arguments"] = ["-d", rviz_config]
    nodes.append(Node(**rviz_node_kwargs))

    return LaunchDescription(nodes)
