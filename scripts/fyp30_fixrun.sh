#!/bin/bash
# Supervisor: clean bringup -> evidence instruments -> sim-time traction gate
# -> 3-leg multi-goal demo. Run via setsid nohup; poll the log.
# Uses ONLY the dataset reconstruction via nav_targets.env (any dataset).
DS="${1:-}"
if [ -z "$DS" ] || [ ! -d "$DS" ]; then
    echo "usage: fyp30_fixrun.sh <dataset_dir>   (dir containing nav_targets.env)"
    exit 1
fi
DS=$(realpath "$DS")
if [ ! -r "$DS/nav_targets.env" ]; then
    echo "FAIL: $DS/nav_targets.env missing - run scripts/fyp30_run_pipeline.sh <video> first"
    exit 1
fi
set -x
# set -a auto-exports every var sourced from nav_targets.env so the child
# bring-up and nav2-restart scripts inherit the CURRENT reconstruction.
set -a
source "$DS/nav_targets.env"
set +a
export ROS_DOMAIN_ID=0
export GZ_IP=127.0.0.1

rm -rf /tmp/ve_final
mkdir -p /tmp/ve_final

echo "== [1/6] full bringup =="
FYP_NO_RVIZ=1 bash "$HOME/FYP/scripts/fyp_realroom30_up.sh" > /tmp/bringup.log 2>&1
echo "bringup rc=$?"
# CPU headroom for RTF: perception nodes are prefetch/stale-republish only.
for p in $(pgrep -f 'depth_pipeline_node|pointcloud_costmap_layer|depth_bridge_node'); do renice 19 -p "$p" >/dev/null 2>&1; done

echo "== [2/6] evidence camera into running world =="
timeout 120 ros2 run ros_gz_sim create \
    -file "$HOME/FYP/scripts/fyp30_evidence_cam.sdf" -name evidence_cam \
    > /tmp/evcam_create.log 2>&1
echo "evcam create rc=$?"
sleep 3
setsid nohup ros2 run ros_gz_bridge parameter_bridge \
    '/evidence_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' \
    > /tmp/evcam_bridge.log 2>&1 &
sleep 3

echo "== [3/6] recorder + gz pose logger =="
setsid nohup /usr/bin/python3 -u "$HOME/FYP/scripts/fyp30_visual_recorder.py" \
    /tmp/ve_final 3600 > /tmp/ve_final/recorder.log 2>&1 &
setsid nohup bash "$HOME/FYP/scripts/fyp30_gz_pose_logger.sh" \
    > /tmp/ve_final/gz_pose_logger.log 2>&1 &
sleep 3

echo "== [4/6] sim-time traction gate =="
timeout 150 /usr/bin/python3 "$HOME/FYP/scripts/fyp30_gate2.py" > /tmp/gate2_run.log 2>&1
GATE_RC=$?
cat /tmp/gate2_run.log
if [ "$GATE_RC" -ne 0 ]; then
  echo "GATE FAILED (rc=$GATE_RC) - NOT running demo"
  echo "FIXRUN: DONE (gate-failed)"
  exit 3
fi

echo "== [4b/6] teleport robot back to spawn (gate drove it 0.87 m; demo goal ring needs spawn origin) =="
timeout 15 gz service -s /world/$FYP_WORLD_NAME/set_pose \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
    --req "name: \"fyp_robot\", position: {x: $SPAWN_X, y: $SPAWN_Y, z: 0.28}, orientation: {x: 0, y: 0, z: 0, w: 1.0}" \
    > /tmp/tp_spawn.log 2>&1
echo "teleport rc=$? -> $(head -1 /tmp/tp_spawn.log)"
sleep 3

echo "== [5/6] 3-leg multi-goal demo =="
cd "$DS"
timeout 5400 /usr/bin/python3 -u "$HOME/FYP/scripts/fyp30_multi_goal_demo.py" \
    "$DS" 3 > multi_demo.log 2>&1
echo "demo rc=$?"

echo "== [6/6] done =="
echo "FIXRUN: DONE (demo rc=$?)"
