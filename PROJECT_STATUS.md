# Project Status

## 2026-09-20 — End-to-end navigation verified

Full pipeline demonstrated live in one sitting:

- Backend + MiDaS: `/health` OK, frames streamed over `/ws/video`, depth PNG served with
  `X-FYP-Depth-Sequence` header (atomic in-memory serve, no read/write race).
- Depth chain: `depth_bridge_node` -> `/smartphone/depth` -> `depth_to_scan` -> `/scan`
  (~4 Hz, 120 beams, sensor QoS). Bridge survives backend outages (404/empty body ->
  warn + retry, no crash).
- Gazebo: robot spawns at (0, -2), stable physics, /odom 50 Hz, /clock flows.
  **`GZ_IP=127.0.0.1` is now pinned** — with the ethernet interface DOWN, gz transport
  previously selected it for multicast and silently dropped every gz->ROS message.
- TF: map -> odom (AMCL, automated initial pose) -> base_link -> camera_depth_frame all resolve.
- Nav2: all six nodes (controller, planner, bt_navigator, behavior, both costmaps) reach
  active via the phased bringup; goal accepted, path planned, controller drove the robot,
  **bt_navigator logged `Goal succeeded`** on a 1.4 m goal (spawn -> (1.4, 0), finished at
  (2.11, -0.04) by odom).
- `depth_to_scan` gains `near_range_anchor`: MiDaS depth is relative, so the nearest visible
  surface is anchored at a configurable distance (2.0 m in sim bringup) instead of hard 0.15 m,
  which made every wall a bumper-distance obstacle.
- Known limitation (recorded-scene test harness): the streamed scene is a fixed recording, so
  depth-derived obstacle marks drift relative to the map as the robot moves and can never be
  ray-cleared behind the robot; long-range goals eventually enter stale inflated zones and
  abort ("Failed to make progress"). With a live phone camera the marks correspond to real
  world obstacles and this loop disappears. Short-range goals in the guaranteed-clear forward
  zone complete reliably.

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
