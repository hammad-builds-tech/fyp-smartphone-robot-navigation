# FYP Project Baseline

## Project
Low-cost indoor robot navigation using smartphone monocular camera,
MiDaS depth estimation, ROS2/Gazebo and Nav2. No LiDAR.

## Main paths
- Project: ~/FYP
- ROS workspace: ~/FYP/ros2_ws
- Nav2 source: ~/FYP/nav2_src
- MiDaS: ~/FYP/MiDaS
- Backend: ~/FYP/backend
- Android: ~/FYP/MiDaS/mobile/android

## Working pipeline
Android Camera
 -> FastAPI WebSocket
 -> MiDaS depth
 -> /camera/depth/image_raw
 -> /camera/depth/points
 -> Nav2 costmaps
 -> Nav2 planner/controller
 -> Gazebo fyp_robot

## ROS2
Distro: Lyrical

Important ROS topics:
- /camera/depth/image_raw
- /camera/depth/points
- /map
- /odom
- /tf
- /cmd_vel
- /clock

## TF
map -> odom -> base_link -> camera_depth_frame

## Nav2
Active components:
- controller_server
- planner_server
- behavior_server
- bt_navigator
- global_costmap
- local_costmap

## Map
~/FYP/ros2_ws/src/indoor_nav_costmap/maps/indoor_map.yaml

Current temporary map:
resolution = 0.05
size = 120 x 120
origin = [-3.0, -3.0, 0.0]

## Current blocker
NavigateToPose accepts goals, but NavFn/GridBased currently fails:
error_code: 208
"Failed to create plan with tolerance of 0.500000"

Next task:
Make temporary map/path guaranteed traversable, verify
(0,0) -> (1.5,0), then restore/integrate depth-based obstacle avoidance.

## Important
Do not reinstall completed dependencies.
Do not delete:
- MiDaS weights
- Android source
- backend source
- ROS source packages
- Nav2 source/install
- maps
- Gazebo robot/world files
- configuration YAML files
