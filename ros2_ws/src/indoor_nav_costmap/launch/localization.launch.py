"""Phase 2 — Smartphone depth pipeline + AMCL localization.

Starts the smartphone depth bridge (FastAPI -> ROS), the depth-to-scan
converter, the map server and AMCL. Readiness gate for this phase (checked by
run_system.sh): /scan flowing from live MiDaS depth AND map -> odom
broadcast by AMCL.

Note for bringup order: /scan only flows once an Android phone (or the test
phone simulator) streams frames to the backend and MiDaS produces depth.
Starting this phase without any camera stream still brings AMCL up with its
configured initial pose, but /scan stays silent and the navigation phase must
not be started before depth data appears.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = (
        "/home/hammad/FYP/ros2_ws/src/"
        "indoor_nav_costmap/config/nav2_params.yaml"
    )
    map_file = (
        "/home/hammad/FYP/ros2_ws/src/"
        "indoor_nav_costmap/maps/indoor_map.yaml"
    )

    backend_url = os.environ.get(
        "FYP_BACKEND_URL",
        "http://127.0.0.1:8000/latest-depth-image",
    )

    depth_bridge = Node(
        package="smartphone_depth_bridge",
        executable="depth_bridge_node",
        output="screen",
        parameters=[
            {
                "backend_url": backend_url,
                "depth_topic": "/smartphone/depth",
                "depth_frame_id": "camera_depth_frame",
                "use_sim_time": True,
            }
        ],
    )

    depth_to_scan = Node(
        package="indoor_nav_costmap",
        executable="depth_to_scan",
        output="screen",
        parameters=[
            {
                "depth_topic": "/smartphone/depth",
                "scan_topic": "/scan",
                "scan_frame_id": "base_link",
                "inverse_depth": True,
                "min_range": 0.15,
                "obstacle_max_range": 3.0,
                "clearing_max_range": 3.5,
                # MiDaS depth is relative; anchor the nearest visible surface
                # at this distance instead of min_range so walls are not
                # permanently reported as bumper-distance obstacles.
                "near_range_anchor": 2.00,
                "use_sim_time": True,
            }
        ],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_file,
                "use_sim_time": True,
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file],
        remappings=[("scan", "/scan")],
    )

    localization_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server", "amcl"],
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            depth_bridge,
            depth_to_scan,
            map_server,
            amcl,
            localization_manager,
        ]
    )
