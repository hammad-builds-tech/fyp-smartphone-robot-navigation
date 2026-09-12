# Troubleshooting

## Current Baseline

Date: 2026-09-12

The baseline repository must not include generated artifacts, weights, credentials, or build outputs.

## Known Safe Guidance

- If MiDaS cannot load, verify that the weight file is available locally, but do not commit the model file.
- If ROS package discovery fails, verify the workspace `install/setup.bash` scripts are sourced and rebuild fresh from `ros2_ws`.
- If the backend does not return the latest depth image, verify that the FastAPI server has a frame and that the depth image output path exists.
- If the ROS bridge cannot fetch the depth image, confirm the private local backend IP is converted to an environment setting and not stored in the repository.
- If Gazebo import fails, verify the `ros2_ws/src/indoor_nav_gazebo` package and model resources locally.

## Next Task

The next implementation milestone is ROS2 → costmap → Nav2 → Gazebo integration.
