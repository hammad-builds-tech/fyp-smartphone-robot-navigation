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

## 2026-09-20 — End-to-end navigation achieved

Fixes this session:

- depth_to_scan: added `near_range_anchor` parameter. MiDaS depth is relative, so the
  nearest visible surface is now anchored at 2.0 m (bringup default) instead of
  hard-coding it to min_range (0.15 m), which made every wall a bumper-distance obstacle
  and stalled navigation. Unit compatibility preserved (default = legacy behavior).
- Gazebo transport pinned to loopback (`GZ_IP=127.0.0.1`) in simulation.launch.py,
  scripts/fyp_phase.sh and run_system.sh. Root cause of intermittent "everything silent":
  with the ethernet link DOWN, gz transport chose it for discovery multicast and dropped
  all gz->ROS data ("Exception sending a multicast message: Network is unreachable").
- backend /latest-depth-image now serves immutable in-memory PNG bytes; the previous
  read-while-replace race produced empty bodies and IncompleteRead errors in the bridge.
- depth_bridge_node hardened: any transient HTTP/decode error logs a warning and retries;
  nothing can kill the node (verified against backend outage: 404 -> warn -> reconnect).
- Bringup restructured into three phase launches (simulation / localization / navigation)
  with a compatibility wrapper (fyp_bringup.launch.py) and readiness-gated run_system.sh.
  Added scripts/fyp_phase.sh (detached phase runner) and scripts/fyp_teardown.sh
  (self-safe teardown).
- Spawn pose moved to (0, -2) in open floor; AMCL initial pose set to match.
- nav2_params: AMCL `set_initial_pose: true`, `scan_topic` wiring, global costmap
  obstacle layer over /scan, corrected parameter types for the Lyrical (1.5.x) stack.

Verification (all commands run in-session):

- Backend: /health OK; WebSocket stream accepted; depth served with advancing
  X-FYP-Depth-Sequence; MiDaS ~1-2 s/frame on CPU.
- ROS chain: /clock ~1 kHz, /odom ~50 Hz, /smartphone/depth + /scan ~4 Hz at sensor QoS,
  map->odom->base_link->camera_depth_frame all resolve.
- Nav2: controller/planner/bt_navigator/behavior + both costmaps reach active.
- Navigation: bt_navigator logged `Goal succeeded` for a 1.4 m goal; robot odom moved
  (0,-2) -> (2.11,-0.04).
- Known harness limitation (documented in TROUBLESHOOTING.md): with the static recorded
  scene, long goals eventually enter stale inflated obstacle marks ("Failed to make
  progress"); requires live phone camera for full-room goals.
