#!/bin/bash
# =============================================================================
# FYP complete-system launcher (phased, readiness-gated)
#
# Starts the full stack in the order the data actually flows, waiting for
# each phase to be genuinely ready before the next one starts:
#
#   Phase 0  FastAPI backend + MiDaS        (wait for /health)
#   Phase 0b Android phone OR test streamer (SIM_PHONE=0 to skip; wait for
#            depth sequence to advance)
#   Phase 1  Gazebo + robot + ROS bridge    (wait for /odom + /clock)
#   Phase 2  Depth bridge + depth->scan +
#            map server + AMCL              (wait for /scan and map->odom TF)
#   Phase 3  Nav2 navigation stack          (wait for all lifecycle nodes
#                                            to reach the active state)
#
# Environment switches:
#   SIM_PHONE=0      do not start the test phone simulator (use when a real
#                    Android phone streams frames)
#   SKIP_CLEANUP=1   do not kill stale FYP ROS/Gazebo processes first
#   KEEP_BACKEND=1   never touch a running backend (default behaviour)
#
# Only ever kills processes belonging to this project (FYP paths, the indoor
# world, launch_params-scope ROS nodes started by this project's launches).
# =============================================================================

# NOTE: deliberately no `set -u` — it breaks sourcing ROS 2 setup.bash
# (AMENT_TRACE_SETUP_FILES is referenced unbound there). Careful quoting is
# used instead, and every environment switch gets a default via ${VAR:-x}.
set -o pipefail
cd /home/hammad/FYP

RED='\033[0;31m'; GREEN='\033[0;32m'; YEL='\033[1;33m'; NC='\033[0m'
info() { echo -e "${GREEN}[FYP]${NC} $*"; }
warn() { echo -e "${YEL}[FYP]${NC} $*"; }
fail() { echo -e "${RED}[FYP]${NC} $*"; exit 1; }

if [ -f /opt/ros/lyrical/setup.bash ]; then
    source /opt/ros/lyrical/setup.bash
else
    fail "ROS 2 Lyrical not found at /opt/ros/lyrical"
fi
[ -f ros2_ws/install/setup.bash ] || fail "ros2_ws not built. Run: cd ros2_ws && colcon build --symlink-install"
source ros2_ws/install/setup.bash

# Load conda env for backend/phone-sim commands
FYP_PY=/home/hammad/miniconda3/envs/fyp/bin/python
[ -x "$FYP_PY" ] || FYP_PY=$(command -v python3)

SIM_PHONE="${SIM_PHONE:-1}"
SKIP_CLEANUP="${SKIP_CLEANUP:-0}"

echo "======================================="
echo " FYP smartphone-depth navigation stack"
echo "======================================="

# -----------------------------------------------------------------------------
# Cleanup: only processes that belong to this project
# -----------------------------------------------------------------------------
cleanup_stale() {
    [ "$SKIP_CLEANUP" = "1" ] && return 0
    warn "Killing stale FYP ROS/Gazebo processes (project-scoped only)"
    pkill -f "ros2 launch indoor_nav"           2>/dev/null
    pkill -f "fyp_bringup.launch.py"            2>/dev/null
    pkill -f "simulation.launch.py"             2>/dev/null
    pkill -f "localization.launch.py"           2>/dev/null
    pkill -f "navigation.launch.py"             2>/dev/null
    pkill -f "indoor_world.sdf"                 2>/dev/null   # our gz sim server
    pkill -f "gz sim -r -s"                     2>/dev/null
    pkill -f "ros_gz_bridge.*parameter_bridge"  2>/dev/null
    pkill -f "ros_gz_sim.*create"               2>/dev/null
    pkill -f "depth_bridge_node"                2>/dev/null
    pkill -f "depth_to_scan"                    2>/dev/null
    pkill -f "nav2_lifecycle_manager"           2>/dev/null
    pkill -f "nav2_controller"                  2>/dev/null
    pkill -f "nav2_planner"                     2>/dev/null
    pkill -f "nav2_behaviors"                   2>/dev/null
    pkill -f "nav2_bt_navigator"                2>/dev/null
    pkill -f "nav2_map_server"                  2>/dev/null
    pkill -f "nav2_amcl"                        2>/dev/null
    pkill -f "base_to_camera_static"            2>/dev/null
    pkill -f "test_backend_pipeline"            2>/dev/null
    sleep 2
}
cleanup_stale

# -----------------------------------------------------------------------------
# Generic wait helpers
# -----------------------------------------------------------------------------
wait_for() {  # wait_for <description> <timeout_s> <probe-command...>
    local desc="$1" timeout_s="$2"; shift 2
    local t0=$SECONDS
    while :; do
        if "$@" >/dev/null 2>&1; then
            info "$desc ready ($((SECONDS - t0))s)"
            return 0
        fi
        if [ $((SECONDS - t0)) -ge "$timeout_s" ]; then
            warn "$desc NOT ready after ${timeout_s}s"
            return 1
        fi
        sleep 2
    done
}

# /scan is ~1 Hz on CPU MiDaS, so a small window is required for the rate
# estimate to appear inside the probe timeout.
topic_flowing() { timeout 15 ros2 topic hz "$1" --window 3 2>/dev/null | grep -q "average rate"; }
tf_available()  { timeout 12 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -m1 -q "Rotation"; }
lifecycle_active() { timeout 10 ros2 lifecycle get "/$1" 2>/dev/null | grep -q "active"; }
backend_healthy() { curl -s -m 3 http://127.0.0.1:8000/health | grep -q '"status"'; }

# =============================================================================
# Phase 0 — backend
# =============================================================================
info "Phase 0: FastAPI backend + MiDaS"
if backend_healthy; then
    info "Backend already healthy on :8000"
else
    info "Starting backend (log: /tmp/fyp_backend.log)"
    setsid nohup env PYTHONPATH=/home/hammad/FYP "$FYP_PY" -m uvicorn \
        backend.api.main:app --host 0.0.0.0 --port 8000 \
        > /tmp/fyp_backend.log 2>&1 < /dev/null &
    wait_for "backend /health" 120 bash -c 'curl -s -m 3 http://127.0.0.1:8000/health | grep -q status' \
        || fail "Backend did not become healthy. See /tmp/fyp_backend.log"
fi

# =============================================================================
# Phase 0b — camera stream (real phone or test simulator)
# =============================================================================
if [ "$SIM_PHONE" = "1" ]; then
    info "Phase 0b: test phone simulator (real MiDaS frames via WebSocket)"
    info "         (SIM_PHONE=0 skips this when your Android streams instead)"
    setsid nohup "$FYP_PY" scripts/test_backend_pipeline.py --loop=86400 \
        > /tmp/fyp_phone_sim.log 2>&1 < /dev/null &
    # Depth is usable as soon as the backend sequence header advances.
    seq0=$(curl -s -m 3 -D - -o /dev/null http://127.0.0.1:8000/latest-depth-image | tr -d '\r' | awk -F': ' '/x-fyp-depth-sequence/{print $2}')
    if [ -z "$seq0" ]; then
        seq0=0
    fi
    t0=$SECONDS
    while :; do
        seq=$(curl -s -m 3 -D - -o /dev/null http://127.0.0.1:8000/latest-depth-image | tr -d '\r' | awk -F': ' '/x-fyp-depth-sequence/{print $2}')
        [ -n "$seq" ] && [ "$seq" -gt "$seq0" ] && break
        [ $((SECONDS - t0)) -ge 60 ] && warn "Depth sequence not advancing (phone simulator failed? see /tmp/fyp_phone_sim.log)"
        [ $((SECONDS - t0)) -ge 60 ] && break
        sleep 3
    done
else
    info "Phase 0b: skipped (SIM_PHONE=0 — make sure the Android app is streaming)"
fi

# =============================================================================
# Phase 1 — Gazebo simulation
# =============================================================================
info "Phase 1: Gazebo world + robot + ROS bridge (log: /tmp/fyp_sim.launch.log)"
setsid nohup ros2 launch indoor_nav_gazebo simulation.launch.py \
    > /tmp/fyp_sim.launch.log 2>&1 < /dev/null &

wait_for "/clock flowing"       60 bash -c "source /opt/ros/lyrical/setup.bash && source /home/hammad/FYP/ros2_ws/install/setup.bash && timeout 12 ros2 topic hz /clock --window 10 2>/dev/null | grep -q 'average rate'" \
    || warn "/clock not flowing yet"
wait_for "/odom flowing"        60 topic_flowing /odom \
    || warn "/odom not flowing yet (robot spawn failed? see /tmp/fyp_sim.launch.log)"

# =============================================================================
# Phase 2 — depth pipeline + localization
# =============================================================================
info "Phase 2: depth bridge + depth->scan + map server + AMCL (log: /tmp/fyp_loc.launch.log)"
setsid nohup ros2 launch indoor_nav_costmap localization.launch.py \
    > /tmp/fyp_loc.launch.log 2>&1 < /dev/null &

wait_for "/scan flowing (MiDaS depth -> LaserScan)" 90 topic_flowing /scan \
    || warn "/scan silent — without a camera stream Nav2 costmaps will see no obstacles"
wait_for "AMCL map->odom transform" 60 tf_available map odom \
    || warn "map->odom missing — AMCL did not localize"

# =============================================================================
# Phase 3 — Nav2 navigation
# =============================================================================
info "Phase 3: Nav2 navigation stack (log: /tmp/fyp_nav.launch.log)"
setsid nohup ros2 launch indoor_nav_costmap navigation.launch.py \
    > /tmp/fyp_nav.launch.log 2>&1 < /dev/null &

NAV_OK=1
for node in map_server amcl controller_server planner_server behavior_server bt_navigator; do
    wait_for "lifecycle: $node active" 60 lifecycle_active "$node" || NAV_OK=0
done

echo
if [ "$NAV_OK" = "1" ]; then
    info "SYSTEM UP — all lifecycle nodes active."
    info "  Send a goal:        ./send_test_goal.sh"
    info "  RViz:               ./run_rviz.sh"
    info "  Logs: /tmp/fyp_backend.log /tmp/fyp_phone_sim.log /tmp/fyp_sim.launch.log"
    info "        /tmp/fyp_loc.launch.log /tmp/fyp_nav.launch.log"
else
    warn "Some nodes did not reach active state — check /tmp/fyp_nav.launch.log"
    warn "Most common cause: no /scan (phone not streaming) or map->odom missing."
fi
