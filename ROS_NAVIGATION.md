# ROS Navigation Baseline

## Current Baseline

Date: 2026-09-12

This document captures the current verified ROS and Nav2-related workspace baseline without starting a new Nav2 implementation.

## Verified ROS Components

- `ros2_ws/src/smartphone_depth_bridge/` publishes `/smartphone/depth`.
- `ros2_ws/src/indoor_nav_costmap/` provides depth conversion nodes to occupancy grid, scan, and point cloud data.
- `ros2_ws/src/indoor_nav_gazebo/` provides an indoor Gazebo world and `fyp_robot` SDF model.

## Verified ROS Build State

The following packages were verified via `colcon build --packages-select indoor_nav_costmap smartphone_depth_bridge indoor_nav_gazebo`:

- `indoor_nav_costmap`
- `smartphone_depth_bridge`
- `indoor_nav_gazebo`

## Current Gap

The current verified bridge contract is now implemented as:

- `/smartphone/depth` from `smartphone_depth_bridge`
- `/depth_occupancy_grid` from the depth-to-occupancy conversion node
- `/scan` from the depth-to-scan conversion node
- `/cmd_vel` from the Nav2 controller chain into Gazebo

The previous mismatch that pulled from `/camera/depth/image_raw` was corrected in the existing node sources to the real source topic.

## Next Task

The next implementation milestone is ROS2 → costmap → Nav2 → Gazebo integration.
