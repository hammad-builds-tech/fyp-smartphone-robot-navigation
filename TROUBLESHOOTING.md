# Troubleshooting

## Current Baseline

Date: 2026-09-12

The baseline repository must not include generated artifacts, weights, credentials, or build outputs.

## Known Safe Guidance

- If MiDaS cannot load, verify that the weight file is available locally, but do not commit the model file.
- If ROS package discovery fails, verify the workspace `install/setup.bash` scripts are sourced and rebuild fresh from `ros2_ws`.
- If the backend does not return the latest depth image, verify that the FastAPI server has a frame and that the depth image output path exists.
- If the ROS bridge cannot fetch the depth image, set `FYP_BACKEND_URL` or pass the ROS parameter `backend_url` rather than embedding a private LAN IP in the repository.
- If Gazebo import fails, verify the `ros2_ws/src/indoor_nav_gazebo` package and model resources locally.
- If depth-to-occupancy and depth-to-scan do not receive data, verify the topic contract remains `/smartphone/depth` and not the camera-native `/camera/depth/image_raw` topic.

## Next Task

The next implementation milestone is ROS2 → costmap → Nav2 → Gazebo integration.
