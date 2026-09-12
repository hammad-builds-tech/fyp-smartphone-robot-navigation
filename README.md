# FYP Smartphone Robot Navigation

This repository captures the current baseline for the FYP project: a low-cost indoor robot navigation system that uses smartphone monocular video plus monocular depth estimation, without LiDAR.

## Current Baseline

- Date: 2026-09-12
- Verified state only.
- MiDaS depth processing and the dpt_hybrid_384 weight path are present and locally usable.
- FastAPI backend endpoints exist for frame upload, depth processing, and latest depth image retrieval.
- ROS 2 smartphone depth bridge publishes `/smartphone/depth`.
- ROS 2 costmap conversion nodes exist for depth-to-occupancy-grid, depth-to-scan, pointcloud conversion, and obstacle control logic.
- Gazebo indoor world and custom `fyp_robot` model exist in the ROS workspace.
- The ROS workspace has already built successfully for `indoor_nav_costmap`, `smartphone_depth_bridge`, and `indoor_nav_gazebo`.

## Next Task

The next implementation milestone is ROS2 → costmap → Nav2 → Gazebo integration.

## Repository Notes

- The root workspace is intentionally backed up from the current local workspace.
- The nested upstream repositories `MiDaS` and `nav2_src` are not modified by this baseline repository and are excluded from this repository by the root ignore file.
- Generated data, build outputs, logs, and model weights are excluded from version control.
