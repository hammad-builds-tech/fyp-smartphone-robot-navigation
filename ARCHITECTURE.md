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

The architecture should move from these verified component packages to one consistent ROS 2 launch and Nav2 navigation stack.

## Next Task

The next implementation milestone is ROS2 → costmap → Nav2 → Gazebo integration.
