#!/bin/bash
# Stop all FYP ROS/Gazebo processes cleanly (backend and phone-sim untouched).
# Patterns live in this file, so callers never self-match.
for pat in \
    "ros2 launch indoor_nav_gazebo simulation" \
    "ros2 launch indoor_nav_costmap localization" \
    "ros2 launch indoor_nav_costmap navigation" \
    "nav2_" \
    "depth_to_scan" \
    "depth_bridge_node" \
    "gz-sim-main" \
    "parameter_bridge" \
    "transform_publisher" \
    "ros_gz_sim"; do
    pkill -KILL -f "$pat" 2>/dev/null
    sleep 0.5
done
sleep 2
remaining=$(pgrep -cf "gz-sim-main|nav2_|parameter_bridge|depth_bridge_node|depth_to_scan" 2>/dev/null)
echo "remaining sim processes: ${remaining:-0}"
