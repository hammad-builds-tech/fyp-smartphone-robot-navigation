# Architecture

## Current Baseline

Date: 2026-09-12

The current architecture is organized as a staged pipeline:

1. Smartphone or Android camera video stream
2. FastAPI backend receives the stream and invokes MiDaS depth inference
3. `smartphone_depth_bridge` republishes depth data as `/smartphone/depth`
4. `indoor_nav_costmap` nodes convert the depth signal into occupancy and scan information
5. Gazebo simulation consumes `/cmd_vel` through the `ros_gz_bridge` path

## Verified Components

- `backend/api/main.py` accepts a WebSocket video stream and receives frames.
- `backend/depth/midas_processor.py` loads MiDaS and produces `numpy` depth arrays.
- `smartphone_depth_bridge/depth_bridge_node.py` fetches the latest depth PNG and republishes a ROS `Image` message.
- `indoor_nav_costmap` includes conversion nodes:
  - `depth_to_occupancy_grid`
  - `depth_to_scan`
  - `depth_to_pointcloud`
  - `pointcloud_to_grid`
  - `depth_obstacle_node`

## Pending Integration

The architecture now aligns the existing verified package components around a single trusted topic contract:

- Bridge publishes `/smartphone/depth` from the FastAPI depth PNG endpoint.
- `depth_to_occupancy_grid` consumes `/smartphone/depth` and publishes `/depth_occupancy_grid`.
- `depth_to_scan` consumes `/smartphone/depth` and publishes `/scan`.
- `depth_to_pointcloud` consumes `/smartphone/depth` for a point-cloud view.
- `depth_obstacle_node` now consumes `/smartphone/depth` and publishes no longer directly to the robot.

The new `fyp_bringup.launch.py` file provides a single integrated Gazebo + depth bridge + occupancy and scan node + Nav2 control chain.

## Next Task

The next implementation milestone is ROS2 → costmap → Nav2 → Gazebo integration.
