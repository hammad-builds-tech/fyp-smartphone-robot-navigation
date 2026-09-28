#!/bin/bash
# =============================================================================
# run_navigation.sh — ONE-COMMAND generic recorded-video navigation (FYP)
#
#   ./run_navigation.sh /path/to/video.mp4      (or an existing dataset dir)
#
# Automatically:
#   1. extracts frames (video) / validates a frame dir
#   2. MiDaS depth (frozen report recipe)
#   3. COLMAP poses + MiDaS+COLMAP fusion (frozen report recipe)
#   4. obstacle occupancy map + generic Gazebo world generation
#   5. automatic valid spawn + goal selection (fyp30_pick_goal.py oracle)
#   6. Gazebo + Nav2 bring-up (no manual coordinates, RViz off by default)
#   7. validates the plan, then navigates and reports the outcome
#
# No source-code modification per video. Fails honestly (exit != 0) if the
# recording cannot be reconstructed instead of inventing geometry.
# =============================================================================
set -o pipefail
cd /home/hammad/FYP
FYP_PY=/home/hammad/miniconda3/envs/fyp/bin/python

INPUT="${1:?usage: ./run_navigation.sh <video.mp4|dataset_dir>}"
GOAL_OVERRIDE="${GOAL_X:+set}"          # optional manual override (testing only)
T0=$(date +%s)
step() { echo; echo "===== [$(( $(date +%s) - T0 ))s] $* ====="; }

step "STAGE A — reconstruction pipeline (MiDaS -> COLMAP -> fusion -> map -> world -> spawn/goal)"
bash scripts/fyp30_run_pipeline.sh "$INPUT" || {
    echo "RECONSTRUCTION FAILED — see logs in the dataset dir"; exit 2; }
# locate the nav_targets.env produced above (input may be a pre-existing dir)
if [ -d "$INPUT" ]; then DS=$(realpath "$INPUT"); else
    SAFE=$(basename "$INPUT" | sed 's/\.[^.]*$//' | tr -c 'a-zA-Z0-9_' '_' | sed 's/_*$//')
    DS=/home/hammad/FYP/realroom/runs/$SAFE; fi
set -a; source "$DS/nav_targets.env"; set +a
echo "dataset: $DS"
echo "spawn ($SPAWN_X,$SPAWN_Y)  goal ($GOAL_X,$GOAL_Y)"

step "STAGE B — Gazebo + Nav2 bring-up (map $FYP_MAP)"
export FYP_NO_RVIZ=${FYP_NO_RVIZ:-1}
bash scripts/fyp_realroom30_up.sh 2>&1 | tail -25 || {
    echo "BRING-UP FAILED"; exit 2; }

step "STAGE C — verify plan, then navigate"
source /opt/ros/lyrical/setup.bash
source /home/hammad/FYP/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export DISPLAY=:0
export FYP_TRAJ_LOG="$DS/trajectory_odom.log"   # per-dataset evidence

if timeout 150 /usr/bin/python3 -u scripts/fyp30_nav_driver.py validate "$GOAL_X" "$GOAL_Y" 2>&1 | tail -3; then :; else
    echo "PLAN VALIDATION FAILED"; exit 2; fi

timeout 400 /usr/bin/python3 -u scripts/fyp30_nav_driver.py navigate "$GOAL_X" "$GOAL_Y" 2>&1 | tail -12
NAV_RC=$?

step "STAGE D — result"
FINAL=$(timeout 15 /usr/bin/python3 -u - <<'PY'
import math, os
path = os.environ.get('FYP_TRAJ_LOG', '/tmp/robot_traj_final_full.log')
segs, pts = [], []
for line in open(path):
    if line.startswith('# --- run start'):
        if pts: segs.append(pts)
        pts = []
        continue
    p = line.split()
    if len(p) >= 3 and not line.startswith('#'):
        pts.append((float(p[1]), float(p[2])))
if pts: segs.append(pts)
pts = segs[-1] if segs else []
if len(pts) < 10:
    print("NO_TRAJECTORY"); raise SystemExit
moved = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
print("MOVED %.2f m" % moved)
PY
)
echo "trajectory: $FINAL"
echo "total wall time: $(( $(date +%s) - T0 )) s"
[ "$NAV_RC" -eq 0 ] && echo "RUN OK (goal reached)" || echo "RUN FAILED (nav exit $NAV_RC)"
exit $NAV_RC
