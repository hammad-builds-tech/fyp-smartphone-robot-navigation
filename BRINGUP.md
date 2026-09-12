# FYP Bringup Guide

## Current Baseline

Date: 2026-09-12

This guide documents the current verified integration path for the ROS2 → costmap → Nav2 → Gazebo bringup workflow.

## Topic Contract

- Smartphone depth bridge publishes `/smartphone/depth` as an `Image` message from the FastAPI depth PNG endpoint.
- `depth_to_occupancy_grid` subscribes to `/smartphone/depth` and publishes `/depth_occupancy_grid`.
- `depth_to_scan` subscribes to `/smartphone/depth` and publishes `/scan`.
- `depth_to_pointcloud` subscribes to `/smartphone/depth` and publishes `/camera/depth/points`.
- Gazebo receives final robot commands on `/cmd_vel` through the existing `ros_gz_bridge` topic mapping.

## Environment

Use:

```bash
source /opt/ros/lyrical/setup.bash
source ~/FYP/ros2_ws/install/setup.bash
source ~/FYP/nav2_src/install/setup.bash
```

Set the backend endpoint locally without committing secrets:

```bash
export FYP_BACKEND_URL=http://127.0.0.1:8000/latest-depth-image
```

## Start the Backend

Start the FastAPI backend in the backend Python environment that contains FastAPI, MiDaS, and the depth-processing dependencies. Do not force that backend into the ROS Python environment.

## Start the ROS/Gazebo/Nav2 Bringup

```bash
ros2 launch indoor_nav_gazebo fyp_bringup.launch.py
```

This launch file starts the Gazebo world, the ROS-Gazebo bridges, the ROS depth bridge, and the depth conversion nodes that output the occupancy and scan data used in the current integration path.

## Nav2

The existing configuration file used by the launch path is:

```text
/home/hammad/FYP/ros2_ws/src/indoor_nav_costmap/config/nav2_params.yaml
```

The active Nav2 server components in the current source-built environment are expected to include:

- `map_server`
- `planner_server`
- `controller_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`
- `collision_monitor`

## Known Limitation

The source depth is a derived pseudo-sensor stream from smartphone monocular depth and is not a LiDAR scan.
