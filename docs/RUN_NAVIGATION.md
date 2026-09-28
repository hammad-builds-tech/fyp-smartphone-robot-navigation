# FYP — Generic Recorded-Video Navigation (frozen workflow)

## One command

```bash
cd ~/FYP
./run_navigation.sh /path/to/video.mp4        # any recorded video
./run_navigation.sh ~/FYP/realroom/runs/X     # or a prepared frame dataset (X/img/*.jpg)
```

Everything below happens automatically — **no source-code edits, no manual
coordinates, no per-video constants**:

| Stage | Implementation | Output |
|---|---|---|
| frames | ffmpeg @2 fps (video input) or existing `img/` | `<DS>/img/frame_*.jpg` |
| MiDaS depth | `scripts/fyp30_midas_batch.py` (report Stage 2, frozen recipe) | `<DS>/depth_midas/*.pfm` |
| poses | COLMAP CPU SIFT: feature_extractor → sequential+exhaustive matching → mapper (`FeatureExtraction/FeatureMatching.use_gpu 0`, `Mapper.min_model_size 3`, etc.) | `<DS>/colmap/sparse/txt/` |
| fusion | `scripts/realroom_fuse.py` (report Stage 3) | `<DS>/fused_cloud.ply` |
| obstacle map | `scripts/realroom_cloud_to_map.py` (Stage 4: voxel+SOR+RANSAC+band+ray carve) | `<DS>/realroom_<DS>.pgm/.yaml` |
| Gazebo world | `scripts/realroom30_world2.py` (generic map-driven generator) | `<DS>/realroom_<DS>_world.sdf` |
| spawn + goal | `scripts/fyp30_pick_goal.py` (walkable cells near camera path ≥0.45 m clearance, 4–9 m pairs whose line crosses ≥2 cells of a ≥25-cell real component) | `<DS>/nav_targets.env` |
| bring-up | `scripts/fyp_realroom30_up.sh` (env-parameterized; RViz off by default: `FYP_NO_RVIZ=1`) | Gazebo + Nav2 |
| navigation | `scripts/fyp30_nav_driver.py validate/navigate` | `<DS>/trajectory_odom.log` |
| avoidance proof | `scripts/fyp30_detour_analysis.py <DS>` (9 gates incl. cloud provenance) | `<DS>/evidence/` |

Stages are idempotent — delete the dataset dir (or `rm -rf <DS>/depth_midas
<DS>/colmap <DS>/fused_cloud.ply <DS>/realroom_*.yaml <DS>/realroom_*_world.sdf
<DS>/nav_targets.env`) to force a full rerun. Honesty guarantees: degenerate
recordings (no COLMAP registration, no walkable pair, no crossing obstacle)
**fail with a clear error instead of fabricating a room**.

## Environment overrides (bring-up only)

`SPAWN_X SPAWN_Y FYP_MAP FYP_WORLD FYP_NO_RVIZ FYP_VIDEO_STREAM_LOCK`
default to the values `nav_targets.env` produced; the defaults reproduce the
archived capture_fresh test when the pipeline stages are already cached.

## Freeze verification (2026-09-28)

| | Run 1: `realroom/capture_fresh` (30-s phone capture) | Run 2: `realroom/runs/capture60` (different recording) |
|---|---|---|
| command | `./run_navigation.sh <dir>` | `./run_navigation.sh <dir>` (identical) |
| source changes between runs | none | none |
| auto spawn | (8.01, 13.73) | (−4.60, 7.49) |
| auto goal | (0.81, 17.58), blocker 765 cells | (0.60, 3.24), blocker 895 cells |
| Nav2 result | status 4 SUCCEEDED | status 4 SUCCEEDED |
| executed / straight | 8.95 m / 8.16 m (ratio 1.096) | 7.18 m / 6.72 m (ratio 1.069) |
| path keeps from blocker | 0.78 m (line grazed at 0.03 m) | 0.70 m (line grazed at 0.00 m) |
| provenance vs fused cloud | median 0.015 m | median 0.016 m |
| detour gates | 9/9 PASS | 9/9 PASS |
| wall time | 521 s (cached stages) | 321 s (20 frames, full recon ~90 s) |

Evidence per run: `<DS>/evidence/` (overlays on recorded RGB, detour_metrics.json,
VERDICT logs), `<DS>/trajectory_odom.log`, `<DS>/run.log`.

Phase 2 (not yet enabled by design): live WebSocket depth + reactive obstacle
layer + real-time replanning. Offline/map-based avoidance is the frozen,
proven behaviour.
