#!/bin/bash
# =============================================================================
# fyp30_run_pipeline.sh — GENERIC offline reconstruction pipeline (frozen recipe)
#
# Usage: fyp30_run_pipeline.sh <dataset_dir | video.mp4>
#
#   video/dataset -> MiDaS depth -> COLMAP poses -> 3D fusion -> filtering
#                 -> occupancy map -> Gazebo world -> spawn/goal selection
#
# Idempotent: existing stage outputs are reused (delete the dataset dir to
# force a full rerun). No stage fabricates data: if COLMAP cannot register the
# frames the script FAILS with exit 2 (never invents a room).
# Output: <dataset>/nav_targets.env with SPAWN_X/SPAWN_Y/GOAL_X/GOAL_Y +
#         FYP_MAP / FYP_WORLD / FYP_WORLD_NAME for fyp_realroom30_up.sh.
# =============================================================================
set -o pipefail
DS=$(realpath "$1") || { echo "usage: $0 <dataset_dir|video.mp4>"; exit 1; }
[ -d "$DS" ] || { echo "dataset dir not created"; exit 1; }

FYP_PY=/home/hammad/miniconda3/envs/fyp/bin/python
cd /home/hammad/FYP
T0=$(date +%s)
step() { echo "== [$(( $(date +%s) - T0 ))s] $*"; }

# ---- 0. input: video file -> frames, or existing frame dir -----------------
if [ -f "$DS" ] && [[ "$DS" == *.mp4 || "$DS" == *.mov || "$DS" == *.avi ]]; then
    VID="$DS"
    SAFE=$(basename "$VID" | sed 's/\.[^.]*$//' | tr -c 'a-zA-Z0-9_' '_' | sed 's/_*$//')
    DS=/home/hammad/FYP/realroom/runs/$SAFE
    mkdir -p "$DS/img"
    if [ -z "$(ls -A "$DS/img" 2>/dev/null)" ]; then
        step "extracting frames from $VID (2 fps)"
        ffmpeg -hide_banner -loglevel error -y -i "$VID" -vf fps=2 "$DS/img/frame_%04d.jpg" \
            || { echo "FAIL: ffmpeg frame extraction"; exit 2; }
    fi
elif [ -d "$DS" ]; then
    SAFE=$(basename "$DS" | tr -c 'a-zA-Z0-9_' '_' | sed 's/_*$//')
else
    echo "usage: $0 <dataset_dir|video.mp4>"; exit 1
fi
NIMG=$(ls "$DS"/img/*.jpg 2>/dev/null | wc -l)
[ "$NIMG" -ge 5 ] || { echo "FAIL: only $NIMG frames in $DS/img (need >= 5)"; exit 2; }
step "dataset $DS ($SAFE), $NIMG frames"

# ---- 1. MiDaS depth (report Stage 2) ---------------------------------------
if [ -z "$(ls "$DS"/depth_midas/*.pfm 2>/dev/null)" ]; then
    step "MiDaS batch depth ($NIMG frames, CPU)"
    "$FYP_PY" scripts/fyp30_midas_batch.py --img-dir "$DS/img" --out-dir "$DS/depth_midas" \
        || { echo "FAIL: MiDaS batch"; exit 2; }
fi

# ---- 2. COLMAP poses (report Stage 2, proven CPU SIFT recipe) --------------
if [ ! -f "$DS/colmap/sparse/txt/images.txt" ]; then
    step "COLMAP feature extraction"
    mkdir -p "$DS/colmap"
    colmap feature_extractor \
        --database_path "$DS/colmap/database.db" --image_path "$DS/img" \
        --ImageReader.camera_model OPENCV --ImageReader.single_camera 1 \
        --FeatureExtraction.use_gpu 0 > "$DS/colmap/feature_extractor.log" 2>&1 \
        || { tail -5 "$DS/colmap/feature_extractor.log"; echo "FAIL: colmap feature_extractor"; exit 2; }
    step "COLMAP matching (sequential + exhaustive)"
    colmap sequential_matcher --database_path "$DS/colmap/database.db" \
        --SequentialMatching.overlap 5 --SequentialMatching.quadratic_overlap 1 \
        --FeatureMatching.use_gpu 0 > "$DS/colmap/matching.log" 2>&1 \
        || { tail -5 "$DS/colmap/matching.log"; echo "FAIL: colmap sequential_matcher"; exit 2; }
    colmap exhaustive_matcher --database_path "$DS/colmap/database.db" \
        --FeatureMatching.use_gpu 0 >> "$DS/colmap/matching.log" 2>&1 \
        || { tail -5 "$DS/colmap/matching.log"; echo "FAIL: colmap exhaustive_matcher"; exit 2; }
    step "COLMAP mapper"
    mkdir -p "$DS/colmap/sparse"
    colmap mapper --database_path "$DS/colmap/database.db" --image_path "$DS/img" \
        --output_path "$DS/colmap/sparse" \
        --Mapper.min_model_size 3 --Mapper.init_min_num_inliers 15 \
        --Mapper.abs_pose_min_num_inliers 6 --Mapper.init_min_tri_angle 2 \
        --Mapper.filter_min_tri_angle 0.5 > "$DS/colmap/mapper.log" 2>&1 \
        || { tail -5 "$DS/colmap/mapper.log"; echo "FAIL: colmap mapper"; exit 2; }
    # pick the largest reconstructed model and convert to TXT
    BEST=$(ls -d "$DS"/colmap/sparse/*/ 2>/dev/null | while read d; do
               echo "$(stat -c%s "$d/images.bin" 2>/dev/null) $d"; done | sort -rn | head -1 | cut -d' ' -f2)
    [ -n "$BEST" ] || { echo "FAIL: no COLMAP model reconstructed"; exit 2; }
    mkdir -p "$DS/colmap/sparse/txt"
    colmap model_converter --input_path "$BEST" --output_path "$DS/colmap/sparse/txt" \
        --output_type TXT > /dev/null 2>&1 || { echo "FAIL: model_converter"; exit 2; }
fi
NREG=$(grep -cE '\.(jpg|JPG|jpeg|JPEG)' "$DS/colmap/sparse/txt/images.txt" 2>/dev/null)
[ "${NREG:-0}" -ge 3 ] || { echo "FAIL: only $NREG images registered (rotation-dominant or textureless capture)"; exit 2; }
step "COLMAP: $NREG/$NIMG images registered"

# ---- 3. 3D fusion (report Stage 3) ------------------------------------------
if [ ! -f "$DS/fused_cloud.ply" ]; then
    step "MiDaS+COLMAP fusion"
    "$FYP_PY" scripts/realroom_fuse.py --img-dir "$DS/img" --depth-dir "$DS/depth_midas" \
        --poses "$DS/colmap/sparse/txt/images.txt" --cameras "$DS/colmap/sparse/txt/cameras.txt" \
        --out "$DS/fused_cloud.ply" > "$DS/fusion.log" 2>&1 \
        || { tail -5 "$DS/fusion.log"; echo "FAIL: fusion"; exit 2; }
    tail -2 "$DS/fusion.log"
fi

# ---- 4. occupancy map (report Stage 4: filter + ray carve) ------------------
MAPBASE="$DS/realroom_$SAFE"
if [ ! -f "$MAPBASE.yaml" ]; then
    step "cloud -> occupancy map"
    "$FYP_PY" scripts/realroom_cloud_to_map.py --cloud "$DS/fused_cloud.ply" \
        --poses "$DS/colmap/sparse/txt/images.txt" --out "$MAPBASE" > "$DS/map.log" 2>&1 \
        || { tail -5 "$DS/map.log"; echo "FAIL: cloud_to_map"; exit 2; }
    grep -E "MAP WRITTEN|WARNING" "$DS/map.log"
fi

# ---- 5. Gazebo world (generic map-driven generator) -------------------------
WORLD="$DS/realroom_${SAFE}_world.sdf"
if [ ! -f "$WORLD" ]; then
    step "map -> Gazebo world"
    "$FYP_PY" scripts/realroom30_world2.py --map "$MAPBASE.yaml" --out "$WORLD" \
        --world-name "${SAFE}" > "$DS/world.log" 2>&1 \
        || { tail -5 "$DS/world.log"; echo "FAIL: world generation"; exit 2; }
    grep -E "kept|WORLD" "$DS/world.log" | tail -2
fi

# ---- 6. automatic spawn + goal (no manual coordinates) ----------------------
step "selecting spawn/goal from captured free space"
"$FYP_PY" scripts/fyp30_pick_goal.py "$MAPBASE.yaml" "$DS/colmap/sparse/txt/images.txt" \
    > "$DS/pick_goal.log" 2>&1 \
    || { cat "$DS/pick_goal.log"; echo "FAIL: no valid spawn/goal pair in this reconstruction"; exit 2; }
cat "$DS/pick_goal.log"

SPAWN_X=$(grep -oP 'SPAWN_X=\K-?[0-9.]+' "$DS/pick_goal.log")
SPAWN_Y=$(grep -oP 'SPAWN_Y=\K-?[0-9.]+' "$DS/pick_goal.log")
GOAL_X=$(grep -oP 'GOAL_X=\K-?[0-9.]+' "$DS/pick_goal.log")
GOAL_Y=$(grep -oP 'GOAL_Y=\K-?[0-9.]+' "$DS/pick_goal.log")
[ -n "$SPAWN_X" ] && [ -n "$GOAL_Y" ] || { echo "FAIL: could not parse spawn/goal"; exit 2; }

cat > "$DS/nav_targets.env" <<EOF
FYP_MAP=$MAPBASE.yaml
FYP_WORLD=$WORLD
FYP_WORLD_NAME=${SAFE}
SPAWN_X=$SPAWN_X
SPAWN_Y=$SPAWN_Y
GOAL_X=$GOAL_X
GOAL_Y=$GOAL_Y
EOF
step "PIPELINE OK -> $DS/nav_targets.env"
exit 0
