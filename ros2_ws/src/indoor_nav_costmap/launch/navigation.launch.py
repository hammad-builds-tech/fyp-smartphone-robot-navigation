"""Phase 3 — Nav2 navigation stack.

Starts controller/planner/behavior servers, the BT navigator and their
lifecycle manager. Only start this phase after localization is healthy
(map -> odom available and /scan flowing), otherwise the global costmap
cannot resolve the robot pose in the map frame and activation fails — the
historical bringup failure of this project.
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = (
        "/home/hammad/FYP/ros2_ws/src/"
        "indoor_nav_costmap/config/nav2_params.yaml"
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file],
        remappings=[("cmd_vel", "/cmd_vel")],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file],
        remappings=[("cmd_vel", "/cmd_vel")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file],
    )

    navigation_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            params_file,
            {
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription(
        [
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            navigation_manager,
        ]
    )
