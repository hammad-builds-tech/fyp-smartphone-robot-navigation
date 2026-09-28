#!/bin/bash
# Full offline bring-up for the 30-second recorded-video test (realroom30).
# Localization = wheel odometry via static map->odom TF (phone disconnected).
# Environment = the captured realroom30 map/world only. No AMCL.
# All kill patterns in-file (pkill trap safety).
pkill -f 'simulation.launch.py' 2>/dev/null
pkill -f 'localization.launch.py' 2>/dev/null
pkill -f 'navigation.launch.py' 2>/dev/null
pkill -f 'gz-sim-main' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'ros_gz_create' 2>/dev/null
pkill -f 'amcl' 2>/dev/null
pkill -f 'map_server' 2>/dev/null
pkill -f 'planner_server' 2>/dev/null
pkill -f 'controller_server' 2>/dev/null
pkill -f 'behavior_server' 2>/dev/null
pkill -f 'smoother_server' 2>/dev/null
pkill -f 'lifecycle_manager' 2>/dev/null
pkill -f 'depth_to_scan' 2>/dev/null
pkill -f 'depth_bridge_node' 2>/dev/null
pkill -f 'depth_pipeline_node' 2>/dev/null
pkill -f 'pointcloud_costmap_layer' 2>/dev/null
pkill -f 'static_transform_publisher' 2>/dev/null
pkill -f 'rviz2' 2>/dev/null
sleep 5

# ---------------------------------------------------------------------------
# GENERIC MODE: every dataset-specific value comes from the environment.
# fyp30_run_pipeline.sh exports them via <dataset>/nav_targets.env; the
# defaults below reproduce the archived capture_fresh test when unset.
# ---------------------------------------------------------------------------
SPAWN_X=${SPAWN_X:-8.01}
SPAWN_Y=${SPAWN_Y:-13.73}
FYP_MAP=${FYP_MAP:-$HOME/FYP/realroom/capture_fresh/realroom_fresh.yaml}
FYP_WORLD=${FYP_WORLD:-$HOME/FYP/realroom/capture_fresh/realroom_fresh_world.sdf}
FYP_NO_RVIZ=${FYP_NO_RVIZ:-0}

echo "== killed; starting backend (video stream locked: offline) =="
FYP_VIDEO_STREAM_LOCK=1 setsid nohup bash "$HOME/FYP/run_backend.sh" > /tmp/fyp_backend.log 2>&1 &
sleep 15

echo "== sim (world $FYP_WORLD, spawn $SPAWN_X,$SPAWN_Y) =="
setsid env FYP_WORLD=$FYP_WORLD \
    FYP_SPAWN_X=$SPAWN_X FYP_SPAWN_Y=$SPAWN_Y \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_gazebo simulation.launch.py \
    > /tmp/fyp_sim30.log 2>&1 &
sleep 30

echo "== static map->odom TF =="
setsid nohup /opt/ros/lyrical/lib/tf2_ros/static_transform_publisher \
    --x $SPAWN_X --y $SPAWN_Y --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id map --child-frame-id odom > /tmp/fyp_static_tf30.log 2>&1 &
sleep 2

echo "== map server (BEFORE nav2 so costmaps init with the map) =="
source /opt/ros/lyrical/setup.bash
source "$HOME/FYP/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
setsid nohup ros2 run nav2_map_server map_server \
    --ros-args -r __node:=map_server \
    -p yaml_filename:=$FYP_MAP \
    -p use_sim_time:=true -p frame_id:=map \
    > /tmp/fyp_mapserver30.log 2>&1 &
sleep 3
for st in configure activate; do
  for i in 1 2 3 4 5 6; do
    CUR=$(timeout 6 ros2 lifecycle get /map_server 2>/dev/null)
    case "$CUR" in
      active*) break 2 ;;
      inactive*) [ "$st" = "activate" ] && timeout 8 ros2 lifecycle set /map_server activate >/dev/null 2>&1; sleep 1 ;;
      *) [ "$st" = "configure" ] && timeout 8 ros2 lifecycle set /map_server configure >/dev/null 2>&1; sleep 1 ;;
    esac
  done
done
echo "map_server: $(timeout 10 ros2 lifecycle get /map_server 2>/dev/null)"

echo "== perception pipeline (offline stale republish + grid bridge) =="
setsid nohup ros2 run smartphone_depth_bridge depth_bridge_node \
    --ros-args -p backend_url:=http://127.0.0.1:8000/latest-depth-image \
    -p depth_topic:=/smartphone/depth -p depth_frame_id:=camera_depth_frame \
    -p use_sim_time:=true -p republish_stale:=true \
    > /tmp/fyp_bridge30.log 2>&1 &
sleep 2
setsid nohup ros2 run indoor_nav_costmap depth_pipeline_node \
    --ros-args -p depth_topic:=/smartphone/depth \
    -p pointcloud_topic:=/camera/depth/points \
    -p grid_topic:=/depth_occupancy_grid \
    -p frame_id:=camera_depth_frame -p use_sim_time:=true \
    > /tmp/fyp_pipeline30.log 2>&1 &
sleep 2
setsid nohup ros2 run indoor_nav_costmap pointcloud_costmap_layer \
    --ros-args -p grid_topic:=/depth_occupancy_grid \
    -p pointcloud_topic:=/camera/depth/points \
    -p map_topic:=/smartphone_map -p use_sim_time:=true \
    > /tmp/fyp_gridbridge30.log 2>&1 &
sleep 6

echo "== nav2 on $FYP_MAP =="
setsid env FYP_MAP=$FYP_MAP \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap navigation.launch.py \
    > /tmp/fyp_nav30_launch.log 2>&1 &
sleep 35

# wait for each managed node to reach active; nudge transitions lost under load
for node in controller_server planner_server behavior_server bt_navigator; do
  for try in 1 2 3 4 5 6; do
    ST=$(timeout 20 ros2 lifecycle get /$node 2>/dev/null)
    case "$ST" in
      active*) echo "$node: $ST"; break ;;
      inactive*)
        timeout 30 ros2 lifecycle set /$node activate 2>/dev/null; sleep 2 ;;
      *)
        timeout 30 ros2 lifecycle set /$node configure 2>/dev/null
        sleep 2
        timeout 30 ros2 lifecycle set /$node activate 2>/dev/null
        sleep 2 ;;
    esac
  done
done
echo "== final nav states =="
for node in controller_server planner_server behavior_server bt_navigator; do
  echo "$node: $(timeout 20 ros2 lifecycle get /$node 2>/dev/null)"
done

echo "== GUIs =="
setsid bash -c 'export DISPLAY=:0; exec gz sim -g -v 2' > /tmp/fyp_gzgui30.log 2>&1 &
sleep 6
if [ "$FYP_NO_RVIZ" != "1" ]; then
  setsid bash -c 'source /opt/ros/lyrical/setup.bash; source '$HOME'/FYP/ros2_ws/install/setup.bash; export DISPLAY=:0; exec rviz2 -d '$HOME'/FYP/rviz_live_view.rviz' > /tmp/fyp_rviz30.log 2>&1 &
  sleep 3
fi

# --- AUTO-UNPAUSE + FLOW VERIFICATION -------------------------------------
# The sim should start running (gz sim -r), but some sessions come up paused
# (clock frozen at 0 => no /odom, no gz /tf). Force-unpause via the world
# control service, then verify clock + odom + gz-side /tf actually flow.
export GZ_IP=127.0.0.1
WNAME=$(grep -o '<world name="[^"]*"' "$FYP_WORLD" | head -1 | sed 's/.*name="//;s/"//')
echo "== unpause world '$WNAME' (if paused) =="
setsid nohup gz service -s /world/$WNAME/control \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 \
    --req 'pause: false' > /tmp/fyp_unpause30.log 2>&1 &
sleep 3
CLK=$(timeout 8 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec)
echo "clock: ${CLK:-NONE}"
ODOM=$(timeout 8 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | tr '\n' ' ')
echo "odom: ${ODOM:-NONE}"
GZTF=$(timeout 6 gz topic -i -t /tf 2>/dev/null | grep -m1 publisher)
echo "gz /tf: ${GZTF:-NO PUBLISHER}"
echo "DONE bringup (spawn $SPAWN_X,$SPAWN_Y)"
