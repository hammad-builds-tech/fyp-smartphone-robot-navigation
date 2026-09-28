#!/bin/bash
# Stop the stale-sensor feeds (rotating with base_link -> smear costmaps) and
# restart Nav2 so costmaps come up fresh from the static phone-derived map.
pkill -f 'depth_to_scan' 2>/dev/null
pkill -f 'depth_pipeline_node' 2>/dev/null
pkill -f 'depth_bridge_node' 2>/dev/null
pkill -f 'pointcloud_costmap_layer' 2>/dev/null
pkill -f 'navigation.launch.py' 2>/dev/null
pkill -f 'bt_navigator' 2>/dev/null
pkill -f 'planner_server' 2>/dev/null
pkill -f 'controller_server' 2>/dev/null
pkill -f 'behavior_server' 2>/dev/null
pkill -f 'smoother_server' 2>/dev/null
pkill -f 'lifecycle_manager' 2>/dev/null
pkill -f 'nav2_collision_monitor' 2>/dev/null
pkill -f 'velocity_smoother' 2>/dev/null
sleep 5
echo "feeds stopped; restarting navigation"
setsid env FYP_MAP=$HOME/FYP/realroom/video60/video60_map.yaml \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap navigation.launch.py \
    > /tmp/fyp_nav60_launch.log 2>&1 &
sleep 40
echo "navigation restarted (spawn-time AMCL replaced by static map->odom)"
