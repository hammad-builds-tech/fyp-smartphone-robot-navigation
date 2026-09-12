# Setup

## Current Baseline

Date: 2026-09-12

This repository captures the current local setup baseline only. It documents the verified components that have been observed to work in the workspace.

## Local System Expectations

- Ubuntu-like host environment
- ROS 2 Lyrical
- Gazebo
- Python/FastAPI
- Android project structure inside the workspace

## Verified Source Inputs

- MiDaS repository is present as an upstream nested repository.
- `nav2_src` is present as an upstream nested ROS navigation source tree.
- The ROS package workspace is under `ros2_ws/src`.

## Public Repository Safety

The following are intentionally excluded from the Git baseline:

- model weights
- build/install/log folders
- generated datasets and outputs
- private IPs and credentials
- local environment files

## Next Task

The next implementation milestone is ROS2 → costmap → Nav2 → Gazebo integration.
