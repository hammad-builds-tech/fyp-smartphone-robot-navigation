#!/bin/bash
# Safe cleanup + relaunch of localization & navigation phases.
# All kill patterns live in this FILE (not in the caller's command line),
# so pkill can never match the invoking shell.
set -u
LOGDIR=/tmp

echo "== stopping localization/navigation =="
pkill -f 'localization.launch.py' 2>/dev/null
pkill -f 'navigation.launch.py' 2>/dev/null
pkill -f 'nav2_amcl' 2>/dev/null
pkill -f 'amcl' 2>/dev/null
pkill -f 'nav2_map_server' 2>/dev/null
pkill -f 'map_server' 2>/dev/null
pkill -f 'nav2_planner' 2>/dev/null
pkill -f 'planner_server' 2>/dev/null
pkill -f 'nav2_controller' 2>/dev/null
pkill -f 'controller_server' 2>/dev/null
pkill -f 'nav2_bt_navigator' 2>/dev/null
pkill -f 'bt_navigator' 2>/dev/null
pkill -f 'nav2_behaviors' 2>/dev/null
pkill -f 'behavior_server' 2>/dev/null
pkill -f 'lifecycle_manager' 2>/dev/null
pkill -f 'depth_to_scan' 2>/dev/null
pkill -f 'depth_pipeline_node' 2>/dev/null
pkill -f 'pointcloud_costmap_layer' 2>/dev/null
pkill -f 'depth_bridge_node' 2>/dev/null
sleep 4

LEFT=$(pgrep -f 'amcl|map_server|planner_server|controller_server|bt_navigator|behavior_server|lifecycle_manager|depth_to_scan|depth_pipeline_node|pointcloud_costmap_layer|depth_bridge_node' | wc -l)
echo "remaining loc/nav processes: $LEFT"

echo "== relaunching localization (capture30 map) =="
setsid env FYP_MAP=/home/hammad/FYP/realroom/capture30/realroom30_map.yaml \
    FYP_INITIAL_POSE="-3.3,4.65" \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap localization.launch.py \
    > $LOGDIR/fyp_loc30.log 2>&1 &
sleep 25

echo "== relaunching navigation =="
setsid "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap navigation.launch.py \
    > $LOGDIR/fyp_nav30.log 2>&1 &
sleep 35

echo "== done =="
