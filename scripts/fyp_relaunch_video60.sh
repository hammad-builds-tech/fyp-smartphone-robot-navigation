#!/bin/bash
# Full clean relaunch for the video60 demo (world2 clean geometry).
# All kill patterns live in this FILE so pkill can never match the caller.
set -u

echo "== stopping sim/loc/nav =="
pkill -f 'simulation.launch.py' 2>/dev/null
pkill -f 'localization.launch.py' 2>/dev/null
pkill -f 'navigation.launch.py' 2>/dev/null
pkill -f 'gz-sim-main' 2>/dev/null
pkill -f 'ruby' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'create' 2>/dev/null
pkill -f 'amcl' 2>/dev/null
pkill -f 'map_server' 2>/dev/null
pkill -f 'planner_server' 2>/dev/null
pkill -f 'controller_server' 2>/dev/null
pkill -f 'bt_navigator' 2>/dev/null
pkill -f 'behavior_server' 2>/dev/null
pkill -f 'lifecycle_manager' 2>/dev/null
pkill -f 'depth_to_scan' 2>/dev/null
pkill -f 'depth_pipeline_node' 2>/dev/null
pkill -f 'pointcloud_costmap_layer' 2>/dev/null
pkill -f 'depth_bridge_node' 2>/dev/null
pkill -f 'rviz2' 2>/dev/null
sleep 5
LEFT=$(pgrep -f 'gz-sim|amcl|nav2|rviz2|depth_bridge' | wc -l)
echo "remaining sim/loc/nav processes: $LEFT"

SPAWN_X=1.58
SPAWN_Y=-1.93

echo "== relaunching simulation (world2) =="
setsid env FYP_WORLD=$HOME/FYP/realroom/video60/video60_world2.sdf \
    FYP_SPAWN_X=$SPAWN_X FYP_SPAWN_Y=$SPAWN_Y \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_gazebo simulation.launch.py \
    > /tmp/fyp_sim60.log 2>&1 &
sleep 25

echo "== relaunching localization (video60 map, republish_stale=1) =="
setsid env FYP_MAP=$HOME/FYP/realroom/video60/video60_map.yaml \
    FYP_INITIAL_POSE="$SPAWN_X,$SPAWN_Y" \
    FYP_DEPTH_REPUBLISH_STALE=1 \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap localization.launch.py \
    > /tmp/fyp_loc60.log 2>&1 &
sleep 25

echo "== relaunching navigation =="
setsid env FYP_MAP=$HOME/FYP/realroom/video60/video60_map.yaml \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap navigation.launch.py \
    > /tmp/fyp_nav60_launch.log 2>&1 &
sleep 25

echo "== relaunching Gazebo GUI + RViz =="
setsid bash -c 'export DISPLAY=:0; exec gz sim -g -v 2' \
    > /tmp/fyp_gzgui.log 2>&1 &
sleep 6
setsid bash -c 'source /opt/ros/lyrical/setup.bash; source '$HOME'/FYP/ros2_ws/install/setup.bash; export DISPLAY=:0; exec rviz2 -d '$HOME'/FYP/rviz_live_view.rviz' \
    > /tmp/fyp_rviz60.log 2>&1 &

echo "DONE relaunch (spawn $SPAWN_X,$SPAWN_Y)"
