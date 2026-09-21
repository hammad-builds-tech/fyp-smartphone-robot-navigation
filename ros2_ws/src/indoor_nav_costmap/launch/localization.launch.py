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
    # Map can be overridden, e.g. a map generated from the real-room
    # reconstruction: FYP_MAP=~/FYP/realroom/realroom_map.yaml
    map_file = os.environ.get(
        "FYP_MAP",
        "/home/hammad/FYP/ros2_ws/src/"
        "indoor_nav_costmap/maps/indoor_map.yaml",
    )
    # AMCL initial pose override for non-default maps: FYP_INITIAL_POSE="x,y"
    _ip = os.environ.get("FYP_INITIAL_POSE", "0.0,-2.0").split(",")
    initial_pose = (float(_ip[0]), float(_ip[1]))

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
        parameters=[params_file, {"set_initial_pose": True, "initial_pose.x": initial_pose[0], "initial_pose.y": initial_pose[1]}],
        remappings=[("scan", "/scan")],
    )

    # Report Stage 3 + 4: depth -> 3D point cloud -> filtering -> 2D
    # occupancy grid. This is the report-defined perception representation.
    depth_pipeline = Node(
        package="indoor_nav_costmap",
        executable="depth_pipeline_node",
        output="screen",
        parameters=[
            {
                "depth_topic": "/smartphone/depth",
                "pointcloud_topic": "/camera/depth/points",
                "grid_topic": "/depth_occupancy_grid",
                "frame_id": "camera_depth_frame",
                "use_sim_time": True,
            }
        ],
    )

    # Report representation -> Nav2: republishes the depth-derived occupancy
    # grid as /smartphone_map (static-layer wire format + updates stream) so
    # both costmaps consume the report pipeline's map instead of /scan.
    grid_bridge = Node(
        package="indoor_nav_costmap",
        executable="pointcloud_costmap_layer",
        output="screen",
        parameters=[
            {
                "grid_topic": "/depth_occupancy_grid",
                "pointcloud_topic": "/camera/depth/points",
                "map_topic": "/smartphone_map",
                "use_sim_time": True,
            }
        ],
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
            depth_pipeline,
            grid_bridge,
            map_server,
            amcl,
            localization_manager,
        ]
    )
