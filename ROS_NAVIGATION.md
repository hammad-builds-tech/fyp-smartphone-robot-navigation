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

The current gap is the missing structured ROS 2 → depth/costmap → Nav2 integration chain and Gazebo command velocity handoff.

## Next Task

The next implementation milestone is ROS2 → costmap → Nav2 → Gazebo integration.
