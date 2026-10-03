# FYP: Smartphone-Video → Indoor Robot Navigation (Generic Pipeline)

## Project Overview

This Final Year Project demonstrates a **low-cost indoor robot navigation system** that builds its
world from a **single smartphone video** instead of expensive LiDAR or RGB-D sensors. Any recorded
indoor video is turned into a 3D reconstruction, an occupancy map, and a Gazebo world in which a
simulated robot navigates with Nav2 and avoids the obstacles that were actually reconstructed —
no synthetic rooms, no hardcoded maps.

```
ANY INPUT VIDEO (30 s / 1 min / 2 min … any length)
    ↓  ffmpeg frame extraction (fps configurable)
    ↓  useful-frame selection (blur + near-duplicate removal, frame cap)
MiDaS DPT-Hybrid-384 monocular depth  (CUDA if available, else CPU)
    ↓
COLMAP camera-pose reconstruction (SIFT, sequential+exhaustive matching)
    ↓  MiDaS depth × COLMAP poses
3D FUSION → filtered point cloud (voxel/SOR/RANSAC)
    ↓
OCCUPANCY MAP (carved free space + occupied components)
    ↓
GAZEBO WORLD (exact rectangle decomposition of reconstructed obstacles)
    ↓  world boxes baked back into the map (world-consistent Nav2 map)
SPAWN + GOAL AUTO-SELECTION (from the reconstruction's own free space)
    ↓
GAZEBO + NAV2 (visible-speed robot, obstacle-crossing goals, detours)
```

**Pipeline entry point:** `scripts/fyp30_run_pipeline.sh <path/to/any_video.mp4>`

## Hardware

- **Development machine**: Ubuntu 26.04.1 LTS
- **GPU (optional)**: NVIDIA CUDA device used automatically by MiDaS when the driver is
  available; the pipeline transparently falls back to CPU (the current dev machine has no
  usable NVIDIA driver loaded, so results here are CPU-measured).
- **Smartphone**: any Android device for recording the input video (or live streaming, see below)
- **Robot**: simulated differential-drive robot in Gazebo (visible ~0.20–0.25 m/s)

## Software Stack

- **ROS2**: Lyrical · **Gazebo**: Harmonic (10.5) · **Nav2**: full stack
- **MiDaS**: DPT-Hybrid-384 (local `MiDaS/` repo + weights) · **PyTorch** (CUDA-capable)
- **COLMAP** (CPU recipe), **ffmpeg/ffprobe**, **OpenCV**, **FastAPI** (live bridge only)

## Repository Structure

```
~/FYP/
├── backend/                  # FastAPI live depth bridge (live-input extension point)
│   ├── api/main.py           # WebSocket + REST endpoints
│   └── depth/midas_processor.py  # MiDaS inference (auto CUDA/CPU)
├── ros2_ws/src/
│   ├── indoor_nav_costmap/   # depth pipeline nodes, nav2_params.yaml, launch
│   ├── indoor_nav_gazebo/    # robot model, world/spawn launch files
│   └── smartphone_depth_bridge/  # backend → ROS2 depth bridge
├── scripts/
│   ├── fyp30_run_pipeline.sh     # ★ GENERIC PIPELINE ORCHESTRATOR
│   ├── fyp30_select_frames.py    # useful-frame selection (blur + dedup + cap)
│   ├── fyp30_midas_batch.py      # MiDaS batch depth (PFM output)
│   ├── realroom_fuse.py          # MiDaS depth × COLMAP poses → fused cloud
│   ├── realroom_cloud_to_map.py  # filtered cloud → occupancy map
│   ├── realroom30_world2.py      # occupancy map → Gazebo world SDF
│   ├── fyp30_bake_world_boxes.py # world obstacle boxes → baked map (Nav2-safe)
│   ├── fyp30_pick_goal.py        # auto spawn/goal from real free space
│   ├── fyp30_multi_goal_demo.py  # multi-goal obstacle-crossing demo oracle
│   ├── fyp30_nav_driver.py       # per-goal navigation driver + trajectory log
│   ├── fyp_realroom30_up.sh      # generic bring-up (all params from env)
│   └── fyp30_fixrun.sh           # demo supervisor (bringup→gate→demo)
├── realroom/capture_fresh/   # example dataset outputs (maps, world, nav_targets.env)
├── MiDaS/                    # MiDaS repo + weights (gitignored, see setup)
├── checkpoints/              # model weights (gitignored)
├── vocab_tree.bin            # COLMAP vocabulary tree (gitignored)
└── docs/                     # PIPELINE.md and other documentation
```

## Setup

```bash
# 1. ROS2 + workspace
source /opt/ros/lyrical/setup.bash
cd ~/FYP/ros2_ws && colcon build --symlink-install && source install/setup.bash

# 2. Python env (MiDaS/COLMAP tooling) — or point FYP_PY at your own env
#    default: ~/miniconda3/envs/fyp/bin/python with torch + opencv-python + numpy

# 3. MiDaS weights (gitignored): place DPT weights at MiDaS/weights/dpt_hybrid_384.pt
#    from https://github.com/isl-org/MiDaS (dpt_hybrid_384 ≈ 400 MB)

# 4. COLMAP + ffmpeg (system packages)
sudo apt install colmap ffmpeg

# 5. COLMAP vocabulary tree (gitignored): vocab_tree.bin in the repo root
#    (optional; pipeline works with sequential+exhaustive matching without it)
```

## Run the Generic Pipeline (any video)

```bash
# 1. Reconstruct: frames → MiDaS → COLMAP → fusion → map → world → bake → spawn/goal
~/FYP/scripts/fyp30_run_pipeline.sh ~/Videos/my_room.mp4
#    → output dataset: ~/FYP/realroom/runs/my_room/  (nav_targets.env at the end)

# 2. Bring up Gazebo + Nav2 on the reconstruction
set -a; source ~/FYP/realroom/runs/my_room/nav_targets.env; set +a
~/FYP/scripts/fyp_realroom30_up.sh

# 3. (Optional) full supervised multi-goal avoidance demo
~/FYP/scripts/fyp30_fixrun.sh ~/FYP/realroom/runs/my_room
```

### Useful options (environment variables, no source edits)

| Variable | Default | Meaning |
|----------|---------|---------|
| `FYP_FPS` | `2` | frame extraction rate |
| `FYP_MAX_FRAMES` | `140` | cap on frames fed to MiDaS+COLMAP |
| `FYP_PY` | conda fyp env | Python used for MiDaS/fusion/map stages |
| `FYP_ROOT` | `~/FYP` | project root |

A 30-second, 1-minute, or 2-minute capture all take the same bounded processing path:
extraction → sharp/duplicate filtering → capped frame set → reconstruction. Longer videos
add extraction time only.

## Failure Handling (the pipeline never fabricates a room)

| Failure | Behaviour |
|---------|-----------|
| unreadable/corrupt video | `ffprobe` check fails fast (`FAIL: unreadable or corrupt video`) |
| too few extracted/usable frames | exits with frame count (`need >= 5`) |
| blurred/repetitive capture | frame selection reports kept/rejected; fails if < 5 usable |
| COLMAP registers too few images | exits (rotation-dominant or textureless capture) |
| fusion/map/world stage fails | stage log tail printed, exit 2 |
| no walkable free space / no obstacle-crossing pair | `fyp30_pick_goal.py` exits honestly |
| world/map inconsistency | bake step fails loudly (Nav2 must see the same geometry as Gazebo) |

Stages are idempotent: completed stage outputs in the dataset dir are reused, so a failed
run resumes where it stopped (delete the dataset dir to force a full rerun).

## Live-Input Extension Point

The core pipeline is stage-based and dataset-dir-driven: a live stream adapter can simply
write RGB frames into `<dataset>/img/` while capture is running, then invoke the same
`fyp30_run_pipeline.sh <dataset_dir>` for reconstruction. `backend/` (FastAPI + MiDaS +
ROS2 bridge) already provides the live depth path used by the Nav2 obstacle layer; the
reconstruction side is identical for recorded and streamed input.

## Demo Verification

The multi-goal demo (`fyp30_multi_goal_demo.py`) auto-picks goals whose straight line crosses
real reconstructed geometry, navigates each leg with `fyp30_nav_driver.py`, and verifies the
executed trajectory bends around every threatened obstacle (detour ratio + clearance). Run-1
evidence on the `capture_fresh` dataset: 3/3 legs reached with visible detours —
see `realroom/capture_fresh/evidence/`.

## License

Developed as a Final Year Project for educational purposes. Third-party components:
MiDaS (Intel ISL), Nav2 (Open Navigation), ROS2, Gazebo, COLMAP.

---
**Project Status**: generic pipeline complete and demo-ready
**Last Updated**: October 3, 2026
