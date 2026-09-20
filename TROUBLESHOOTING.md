# Troubleshooting

## Gazebo/ROS data suddenly absent (no /clock, /odom, /tf)

Symptom: `gz topic -l` lists topics, `ros2 topic list` shows them, but no data flows;
the simulation log shows `parameter_bridge: Exception sending a multicast message:
Network is unreachable`.

Cause: Gazebo Transport picked a DOWN interface (e.g. unplugged ethernet) for its
discovery multicast. The fix is already applied project-wide: `GZ_IP=127.0.0.1` is set
in `simulation.launch.py`, `scripts/fyp_phase.sh` and `run_system.sh`. If you launch
Gazebo manually, export `GZ_IP=127.0.0.1` yourself.

## Navigation goal aborted with "Failed to make progress"

With the recorded-scene test streamer (`scripts/test_backend_pipeline.py --loop`),
MiDaS obstacles describe a fixed recording, not the simulated room. As the robot
drives, those obstacle marks drift relative to the map and cannot be ray-cleared
behind the robot; eventually the robot sits inside stale inflated marks and path
validation fails. This is a test-harness limitation, not a system bug: with a live
phone camera the marks correspond to real obstacles. Remedies: use a live phone, or
keep test goals short (within ~2 m of the spawn in the guaranteed-clear forward zone).

## CLI tools show topics as silent

`ros2 topic hz/echo` default to reliable QoS; sensor topics here are best-effort.
A silent-looking `/scan` under CLI tools is usually a QoS artifact. Verify with a
rclpy probe using `qos_profile_sensor_data` (see `scripts/` and PROJECT_STATUS.md).

## Costmaps fail activation on startup

They need TF and /scan ready before activation. The phased bringup
(`run_system.sh`) gates each phase on readiness. If starting phases manually,
wait for `map -> base_link` TF before launching `navigation.launch.py`.

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
