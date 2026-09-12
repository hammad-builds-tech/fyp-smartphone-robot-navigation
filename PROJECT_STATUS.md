# Project Status

## Current Baseline

Date: 2026-09-12

The workspace has a verified implementation structure in four major layers:

1. Smartphone Video and Backend
   - FastAPI backend receives webcam/smartphone JPEG frames and emits depth telemetry.
   - MiDaS depth output is produced by the depth processor in `backend/depth/midas_processor.py`.

2. ROS 2 Smartphone Depth Bridge
   - `smartphone_depth_bridge` republishes a depth image to `/smartphone/depth`.

3. Costmap and Occupancy Generation
   - `indoor_nav_costmap` contains nodes for converting depth into occupancy, scan, and point-cloud data.

4. Gazebo and Robot World
   - `indoor_nav_gazebo` contains `indoor_world.launch.py` for the Gazebo world and the custom `fyp_robot` model.

## Verified State

- MiDaS is installed locally and depth inference is available.
- `dpt_hybrid_384` weights are present in the expected location.
- FastAPI backend endpoints are available and process depth frames.
- Android application package exists in the workspace.
- Gazebo world and robot model are present.
- `ros2_ws` packages build with `colcon`.

## Current Limitations

- The stack is now aligned to a single ROS 2 topic contract using `/smartphone/depth` for the bridge and all current depth-facing subscribers.
- The backend URL in the ROS depth bridge is now represented via the `FYP_BACKEND_URL` environment variable defaulting to a local loopback address and must remain out of the public repository.

## Integration Contract

- Smartphone depth image: `/smartphone/depth`
- Occupancy grid: `/depth_occupancy_grid`
- Scan topic: `/scan`
- Gazebo command velocity: `/cmd_vel`

## Next Task

The next implementation milestone is ROS2 → costmap → Nav2 → Gazebo integration.
