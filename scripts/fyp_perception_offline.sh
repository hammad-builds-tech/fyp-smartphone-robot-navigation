#!/bin/bash
# Offline-demo perception nodes: keep the phone-derived (saved video) depth
# -> point cloud -> occupancy grid -> /smartphone_map pipeline running,
# WITHOUT AMCL (pose comes from the static map->odom TF) and without the
# base_link-authored rotating stale scan (it corrupted AMCL + costmaps).
source /opt/ros/lyrical/setup.bash
source "$HOME/FYP/ros2_ws/install/setup.bash"

setsid nohup ros2 run smartphone_depth_bridge depth_bridge_node \
    --ros-args -p backend_url:=http://127.0.0.1:8000/latest-depth-image \
    -p depth_topic:=/smartphone/depth -p depth_frame_id:=camera_depth_frame \
    -p use_sim_time:=true -p republish_stale:=true \
    > /tmp/fyp_bridge_manual.log 2>&1 &

sleep 2
setsid nohup ros2 run indoor_nav_costmap depth_pipeline_node \
    --ros-args -p depth_topic:=/smartphone/depth \
    -p pointcloud_topic:=/camera/depth/points \
    -p grid_topic:=/depth_occupancy_grid \
    -p frame_id:=camera_depth_frame -p use_sim_time:=true \
    > /tmp/fyp_pipeline_manual.log 2>&1 &

sleep 2
setsid nohup ros2 run indoor_nav_costmap pointcloud_costmap_layer \
    --ros-args -p grid_topic:=/depth_occupancy_grid \
    -p pointcloud_topic:=/camera/depth/points \
    -p map_topic:=/smartphone_map -p use_sim_time:=true \
    > /tmp/fyp_gridbridge_manual.log 2>&1 &

sleep 8
echo "bridge=$(pgrep -c -f depth_bridge_nod[e]) pipeline=$(pgrep -c -f depth_pipeline_nod[e]) grid_bridge=$(pgrep -c -f pointcloud_costmap_lay[e]r)"
