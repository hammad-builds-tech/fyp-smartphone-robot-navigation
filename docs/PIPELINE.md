# Generic Pipeline Reference (any input video)

Entry point: `scripts/fyp30_run_pipeline.sh <dataset_dir | video.(mp4|mov|avi|mkv|webm|...)>`

All stages are idempotent — existing stage outputs in the dataset dir are reused, so a
failed run resumes where it stopped. Every stage FAILS LOUDLY (exit 2) rather than
fabricating data.

## Stages

| # | Stage | Script | Input → Output | Failure handling |
|---|-------|--------|----------------|------------------|
| 0 | video → frames | ffmpeg/ffprobe | video → `img/frame_*.jpg` (fps = `FYP_FPS`, default 2) | ffprobe rejects unreadable/corrupt video; `need >= 5` frames |
| 0b | useful-frame selection | `fyp30_select_frames.py` | drops blurred + near-duplicate frames, caps at `FYP_MAX_FRAMES` (default 140) | fails if < 5 usable frames remain |
| 1 | MiDaS depth | `fyp30_midas_batch.py` | frames → `depth_midas/*.pfm` | per-frame skip only for unreadable images |
| 2 | COLMAP poses | colmap CLI | frames → `colmap/sparse/txt/` | fails if no model or < 3 registered images |
| 3 | 3D fusion | `realroom_fuse.py` | depth × poses → `fused_cloud.ply` | stage log tail printed, exit 2 |
| 4 | cloud → occupancy map | `realroom_cloud_to_map.py` | cloud → `realroom_<name>.pgm/.yaml` | stage log tail printed, exit 2 |
| 5 | map → Gazebo world | `realroom30_world2.py` | map → `realroom_<name>_world.sdf` | XML-validated output; exit 2 on failure |
| 5b | bake world boxes → map | `fyp30_bake_world_boxes.py` | world SDF + map → `<name>_baked.pgm/.yaml` | fails on missing boxes / unreadable map |
| 6 | auto spawn + goal | `fyp30_pick_goal.py` | map + poses → `SPAWN_X/Y`, `GOAL_X/Y` | exits honestly when no walkable free space / no obstacle-crossing pair |
| 7 | env handoff | (script) | writes `<dataset>/nav_targets.env` | parse failure exits 2 |

`nav_targets.env` contains: `FYP_MAP` (baked, world-consistent), `FYP_WORLD`,
`FYP_WORLD_NAME`, `SPAWN_X/Y`, `GOAL_X/Y` — everything the generic bring-up
(`fyp_realroom30_up.sh`) and demo (`fyp30_fixrun.sh`, `fyp30_multi_goal_demo.py`)
need. No coordinates, filenames, or dataset paths are hardcoded in the workflow.

## Why the bake stage exists

The raw occupancy map is lethal only where capture rays stopped; the Gazebo world
rendered from the same reconstruction is solid. Nav2 planning on the raw map can cut
through map gaps that physically exist in the world (the robot wedges). Baking every
`obstacle_*`/`wall_*` box (inflated by half the robot footprint, 0.25 m) onto the map
makes the Nav2 map world-consistent using only reconstruction data.

## Environment variables (no source edits needed)

| Variable | Default | Purpose |
|----------|---------|---------|
| `FYP_FPS` | `2` | extraction fps |
| `FYP_MAX_FRAMES` | `140` | frame cap for MiDaS+COLMAP |
| `FYP_PY` | `~/miniconda3/envs/fyp/bin/python` | python for MiDaS/fusion/map stages |
| `FYP_ROOT` | `~/FYP` | project root |

## Live-input extension point

A live adapter writes RGB frames into `<dataset>/img/`; then
`fyp30_run_pipeline.sh <dataset_dir>` runs unchanged. The `backend/` FastAPI+MiDaS+ROS2
bridge already provides the live depth path used by the Nav2 obstacle layer — recorded
and streamed input share the same reconstruction stages.
