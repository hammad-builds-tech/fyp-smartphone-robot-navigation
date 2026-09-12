# Development Log

## Current Baseline

Date: 2026-09-12

This document records the verified state from the current workspace before starting the next implementation milestone.

## Verified Entries

- `MiDaS` depth inference is working locally.
- The `dpt_hybrid_384` weight artifact is available.
- The FastAPI backend receives and processes webcam or smartphone frames.
- A ROS 2 depth bridge republishes `/smartphone/depth`.
- The `indoor_nav_costmap` package provides depth conversion and scan/point-cloud generation nodes.
- The `indoor_nav_gazebo` package contains the `fyp_robot` model and an indoor world launch path.
- The ROS workspace packages `indoor_nav_costmap`, `smartphone_depth_bridge`, and `indoor_nav_gazebo` built successfully with `colcon`.

## Planned Future Work

The verified baseline is now updated with an integrated launch path and a single topic contract for the ROS2 → costmap → Nav2 → Gazebo integration milestone.

## Verification

- Existing topic contract corrected: `/smartphone/depth` is the source topic for depth conversion nodes.
- Backend URL in the depth bridge is configured via the `backend_url` ROS parameter and the `FYP_BACKEND_URL` environment variable default.
- `colcon build --packages-select indoor_nav_costmap smartphone_depth_bridge indoor_nav_gazebo` verified the package build after the selected source adjustments.
