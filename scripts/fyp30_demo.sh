#!/bin/bash
# fyp30_demo.sh — ONE-COMMAND launcher for the FYP supervisor demo
#
# Runs the COMPLETE recorded-video navigation demo from a cold laptop:
#   clean slate -> full ROS2/Nav2/Gazebo bring-up (fresh 30-s reconstruction,
#   Gazebo GUI only, NO RViz) -> evidence cameras + recorders -> traction gate
#   -> multi-obstacle AGENDA tour at 0.85 m/s:
#   for EACH substantial reconstructed obstacle: fast APPROACH to ~1.2 m off
#   its face -> visible slow-down -> turn/clear around it (CROSS to its far
#   side) -> accelerate on to the next obstacle.
#
# Usage:   bash ~/FYP/scripts/fyp30_demo.sh              # latest reconstruction
#          FYP_DEMO_DS=~/FYP/realroom/runs/<name> bash ~/FYP/scripts/fyp30_demo.sh
#          bash ~/FYP/scripts/fyp30_demo.sh ~/FYP/realroom/runs/<name>
# Stop:    Ctrl-C (leaves Gazebo open; close its window manually)
# Logs:    /tmp/fyp30_demo_onecmd.log  (demo verdict at the end)
# =============================================================================
set -u
FYP="$HOME/FYP"
# GENERIC: any reconstruction run produced by fyp30_run_pipeline.sh works.
# Default = the most recent run; override with arg 1 or FYP_DEMO_DS env.
if [ $# -ge 1 ]; then DS="$(realpath "$1")"
elif [ -n "${FYP_DEMO_DS:-}" ]; then DS="$(realpath "$FYP_DEMO_DS")"
else DS=$(ls -dt "$FYP"/realroom/runs/*/ 2>/dev/null | head -1)
fi
DS=${DS%/}
[ -n "$DS" ] || { echo "FATAL: no reconstruction under $FYP/realroom/runs — run the pipeline first"; exit 1; }

[ -r "$DS/nav_targets.env" ] || { echo "FATAL: $DS/nav_targets.env missing — run the pipeline first"; exit 1; }

# ROS setup.bash scripts are not set-u safe: disable nounset around sourcing.
# Sourcing HERE means the launcher works from any fresh terminal.
set +u; source /opt/ros/lyrical/setup.bash; source "$FYP/ros2_ws/install/setup.bash"; set -u
# some stripped environments do not get the gz tools vendor dir from ament
export PATH="/opt/ros/lyrical/opt/gz_tools_vendor/bin:$PATH"
command -v gz >/dev/null || { echo "FATAL: gz not found after sourcing ROS lyrical"; exit 1; }

echo "== [0/5] stopping anything left over from a previous run =="
pkill -f 'fyp30[_]multi_goal_demo' 2>/dev/null
pkill -f 'fyp[_]nav[_]final'       2>/dev/null
pkill -f 'nav2[_]'                 2>/dev/null
pkill -f 'ros_gz[_]'               2>/dev/null
pkill -f 'gz[ ]sim'                2>/dev/null
pkill -f 'depth[_]pipeline'        2>/dev/null
pkill -f 'pointcloud[_]costmap'    2>/dev/null
pkill -f 'depth[_]bridge'          2>/dev/null
pkill -f 'map[_]server'            2>/dev/null
pkill -f 'parameter[_]bridge'      2>/dev/null
sleep 5

echo "== [0.5/5] 3D redesign pass (coherent muted furniture blobs + materials) =="
set +u; set -a; source "$DS/nav_targets.env"; set +a; set -u
WORLD_SDF="$FYP_WORLD"
# STEP A: reset to pristine .orig (guarantees both passes below start from a
# KNOWN base; hand-edited restyles are wiped, collisions/poses never change).
# STEP B: geometry REDESIGN v2 -- depth cells are BFS-merged with a STRICT
# 0.10 m gap rule so each piece of FURNITURE is one readable solid
# (cabinet/crate/pillar/cylinder) in a muted palette; sparse cells become
# clean 0.4 m cubes; ALL collision <size> blocks stay byte-identical
# (verified offline via bake-replay), so the Nav2 map and planner behavior
# are UNCHANGED.
# STEP C: material polish (warm walls, lit floor, ambient lift, 2 ceiling
# lights) on top of the redesigned shapes.
if [ -n "${FYP_WORLD:-}" ] && [ -f "$FYP_WORLD" ]; then
  WORLD_SDF="$FYP_WORLD"
  if [ -f "$WORLD_SDF.orig" ]; then
    cp -f "$WORLD_SDF.orig" "$WORLD_SDF"
  fi
  python3 "$FYP/scripts/fyp30_redesign_world.py" "$WORLD_SDF" --backup \
      > /tmp/fyp30_redesign.log 2>&1
  RED_RC=$?
  echo "   redesign rc=$RED_RC ($(tail -1 /tmp/fyp30_redesign.log 2>/dev/null))"
  if [ "$RED_RC" -ne 0 ]; then
    cp -f "$WORLD_SDF.orig" "$WORLD_SDF" 2>/dev/null
    echo "   redesign FAILED -> reverted to .orig; aborting"
    exit 4
  fi
  python3 "$FYP/scripts/fyp30_decorate_world.py" "$WORLD_SDF" --backup \
      > /tmp/fyp30_decorate.log 2>&1
  echo "   decorate rc=$? (materials only; $(grep -c obstacle_ "$WORLD_SDF" 2>/dev/null || echo 0) obstacle models present)"
else
  echo "   SKIP: FYP_WORLD not set; using world as-is"
fi

echo "== [1/5] full bring-up (Gazebo GUI + Nav2, NO RViz) =="
set -a; source "$DS/nav_targets.env"; set +a
export ROS_DOMAIN_ID=0 GZ_IP=127.0.0.1 FYP_NO_RVIZ=1
bash "$FYP/scripts/fyp_realroom30_up.sh" > /tmp/fyp30_demo_bringup.log 2>&1
echo "   bringup rc=$?"

echo "== [2/5] instruments: evidence cameras (IN-WORLD) + recorders + pose logger =="
rm -rf /tmp/ve_final /tmp/ve_final_cam2; mkdir -p /tmp/ve_final /tmp/ve_final_cam2
# Both evidence cameras are now INSIDE the world SDF (see .orig injection):
# sensors spawned via `ros_gz_sim create` AFTER sim-start render clear color
# (verified live several times), while world-loaded sensors render correctly.
timeout 15 ros2 topic echo /evidence_cam/camera_info --once > /dev/null 2>&1 \
    && echo "   evidence_cam streaming (world-loaded)" || echo "   WARN: evidence_cam not yet streaming"
sleep 2
timeout 15 ros2 topic echo /evidence_cam2/camera_info --once > /dev/null 2>&1 \
    && echo "   evidence_cam2 streaming (world-loaded)" || echo "   WARN: evidence_cam2 not yet streaming"
sleep 2
# Close-up evidence camera (obstacle-level view matching the reference image).
# The camera MODELS themselves are injected into the world SDF by the
# decorate pass (poses come from the reconstruction dataset itself, not
# hardcoded here), so nothing needs spawning after sim-start.
timeout 20 ros2 topic echo /evidence_cam2/camera_info --once > /dev/null 2>&1 || true
sleep 2
setsid nohup ros2 run ros_gz_bridge parameter_bridge \
    '/evidence_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' \
    > /tmp/evcam_bridge.log 2>&1 < /dev/null &
sleep 3
setsid nohup ros2 run ros_gz_bridge parameter_bridge \
    '/evidence_cam2/image@sensor_msgs/msg/Image[gz.msgs.Image' \
    > /tmp/evcam2_bridge.log 2>&1 < /dev/null &
sleep 3
# Recorder A films the OVERVIEW cam, Recorder B films the CLOSE-UP cam:
# two independent frame sets so any single frozen sensor can't blind us.
setsid nohup /usr/bin/python3 -u "$FYP/scripts/fyp30_visual_recorder.py" \
    /tmp/ve_final 3600 /evidence_cam/image \
    > /tmp/ve_final/recorder.log 2>&1 < /dev/null &
setsid nohup /usr/bin/python3 -u "$FYP/scripts/fyp30_visual_recorder.py" \
    /tmp/ve_final_cam2 3600 /evidence_cam2/image \
    > /tmp/ve_final_cam2_recorder.log 2>&1 < /dev/null &
setsid nohup bash "$FYP/scripts/fyp30_gz_pose_logger.sh" \
    > /tmp/ve_final/gz_pose_logger.log 2>&1 < /dev/null &
sleep 3
# CPU headroom for RTF: perception nodes are prefetch/stale-republish only
for p in $(pgrep -f 'depth_pipeline_node|pointcloud_costmap_layer|depth_bridge_node'); do renice 19 -p "$p" >/dev/null 2>&1; done

echo "== [2.5/5] VERIFY: evidence cameras render real scene (not clear color) =="
export PATH="/opt/ros/lyrical/opt/gz_tools_vendor/bin:$PATH"; export GZ_IP=127.0.0.1
sleep 12
for CAM in evidence_cam evidence_cam2; do
  timeout 30 gz topic -e -t /$CAM/image -n 1 2>/dev/null > /tmp/${CAM}_check.txt
  NPX=$(/usr/bin/python3 - <<PYX
import re
raw=open('/tmp/${CAM}_check.txt','rb').read()
i=raw.find(b'data: "')
vals=[int(x) for x in re.findall(rb'\\\((\d{3})', raw[i+7:i+7+3000])]
pix=[tuple(vals[k:k+3]) for k in range(0,len(vals)-2,3)]
print(len(set(pix)))
PYX
)
  echo "   $CAM: unique-sample-pixels=$NPX $([ "$NPX" -gt 5 ] 2>/dev/null && echo RENDER_OK || echo RENDER_BROKEN)"
done

echo "== [3/5] sim-time traction gate (proves real wheel traction) =="
timeout 150 /usr/bin/python3 "$FYP/scripts/fyp30_gate2.py" > /tmp/gate2_run.log 2>&1
GATE_RC=$?
cat /tmp/gate2_run.log
if [ "$GATE_RC" -ne 0 ]; then
  echo "GATE FAILED — demo aborted (see /tmp/gate2_run.log)"; exit 3
fi

echo "== [4/5] teleport robot back to spawn =="
timeout 15 gz service -s "/world/$FYP_WORLD_NAME/set_pose" \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
    --req "name: \"fyp_robot\", position: {x: $SPAWN_X, y: $SPAWN_Y, z: 0.28}, orientation: {x: 0, y: 0, z: 0, w: 1.0}" \
    > /tmp/tp_spawn.log 2>&1
echo "   teleport rc=$?"
sleep 3

echo "== [5/5] multi-obstacle AGENDA tour @0.85 m/s (watch the Gazebo GUI!) =="
echo "   APPROACH each obstacle -> slow near its face -> clear around it ->"
echo "   accelerate to the next obstacle. ~10 legs over 5 obstacles."
cd "$DS"
timeout 5400 /usr/bin/python3 -u "$FYP/scripts/fyp30_multi_goal_demo.py" \
    "$DS" 10 2>&1 | tee multi_demo_final.log
DEMO_RC=${PIPESTATUS[0]}

echo
echo "== demo finished (rc=$DEMO_RC) — verdict + evidence: =="
tail -6 multi_demo_final.log
cp -r /tmp/ve_final "$DS/evidence/video_$(date +%H%M%S)" 2>/dev/null || true
cp -r /tmp/ve_final_cam2 "$DS/evidence/cam2_$(date +%H%M%S)" 2>/dev/null || true
echo "   dataset: $DS"
echo "   video evidence copied into $DS/evidence/"
exit $DEMO_RC
