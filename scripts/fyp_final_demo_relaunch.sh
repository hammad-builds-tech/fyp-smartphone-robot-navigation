#!/bin/bash
# Kill and relaunch sim + localization + nav for the final video60 demo.
# All kill patterns live in this FILE (never pkill from the agent shell).
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
pkill -f 'bt_navigator' 2>/dev/null
pkill -f 'behavior_server' 2>/dev/null
pkill -f 'lifecycle_manager' 2>/dev/null
pkill -f 'depth_to_scan' 2>/dev/null
pkill -f 'depth_bridge_node' 2>/dev/null
pkill -f 'depth_pipeline_node' 2>/dev/null
sleep 5
echo "killed; gz procs left: $(pgrep -c -f 'gz-sim' || echo 0)"

SPAWN_X=1.58
SPAWN_Y=-1.93

echo "== sim (world2) =="
setsid env FYP_WORLD=$HOME/FYP/realroom/video60/video60_world2.sdf \
    FYP_SPAWN_X=$SPAWN_X FYP_SPAWN_Y=$SPAWN_Y \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_gazebo simulation.launch.py \
    > /tmp/fyp_sim60.log 2>&1 &
sleep 30

echo "== localization =="
setsid env FYP_MAP=$HOME/FYP/realroom/video60/video60_map.yaml \
    FYP_INITIAL_POSE="$SPAWN_X,$SPAWN_Y" \
    FYP_DEPTH_REPUBLISH_STALE=1 \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap localization.launch.py \
    > /tmp/fyp_loc60.log 2>&1 &
sleep 30

echo "== navigation =="
setsid env FYP_MAP=$HOME/FYP/realroom/video60/video60_map.yaml \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap navigation.launch.py \
    > /tmp/fyp_nav60_launch.log 2>&1 &
sleep 30
echo "DONE relaunch (spawn $SPAWN_X,$SPAWN_Y)"
