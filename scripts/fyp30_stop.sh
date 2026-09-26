#!/bin/bash
# Stop all FYP ROS/Gazebo processes cleanly. Patterns live in this FILE to
# avoid the pkill self-match trap (inline bash -c strings match themselves).
pkill -f 'fyp_realroom30_up.sh' 2>/dev/null
pkill -f 'fyp_phase.sh' 2>/dev/null
pkill -f 'gz-sim-main' 2>/dev/null
pkill -f 'gz-sim-gui-client' 2>/dev/null
pkill -x rviz2 2>/dev/null
pkill -f 'simulation.launch.py' 2>/dev/null
pkill -f 'localization.launch.py' 2>/dev/null
pkill -f 'navigation.launch.py' 2>/dev/null
pkill -f 'controller_server' 2>/dev/null
pkill -f 'planner_server' 2>/dev/null
pkill -f 'bt_navigator' 2>/dev/null
pkill -f 'behavior_server' 2>/dev/null
pkill -f 'smoother_server' 2>/dev/null
pkill -f 'lifecycle_manager' 2>/dev/null
pkill -f 'map_server' 2>/dev/null
pkill -f 'amcl' 2>/dev/null
pkill -f 'depth_bridge_node' 2>/dev/null
pkill -f 'depth_pipeline_node' 2>/dev/null
pkill -f 'pointcloud_costmap_layer' 2>/dev/null
pkill -f 'static_transform_publisher' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'ros_gz_create' 2>/dev/null
pkill -f 'depth_to_scan' 2>/dev/null
sleep 3
echo "remaining ROS/Gazebo procs:"
pgrep -af "gz-sim|rviz|nav2|depth_bridge|static_transform" | grep -v "fyp30_stop" | head -5 || true
echo "done"
