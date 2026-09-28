#!/bin/bash
# Offline-demo localization fix:
# The phone is DISCONNECTED, so there is no live sensor. AMCL keeps trying to
# scan-match the republished stale scan (authored in base_link, so it sweeps
# with the robot), corrupting both the pose and the costmaps.
# For the offline demo: pose = wheel odometry (exact in Gazebo) via a static
# map->odom TF; environment = the phone-derived static map only.
# Stale /scan keeps flowing ONLY as the compatibility feed (harmless now that
# nothing scan-matches it), and the depth->pointcloud pipeline stays up.
pkill -f 'amcl' 2>/dev/null
pkill -f 'localization_manager' 2>/dev/null
sleep 3
source /opt/ros/lyrical/setup.bash
source "$HOME/FYP/ros2_ws/install/setup.bash"
# map = odom (robot starts at odom (0,0) == map (1.58,-1.93) spawn offset)
setsid nohup ros2 run tf2_ros static_transform_publisher \
    1.58 -1.93 0 0 0 0 map odom \
    > /tmp/fyp_static_tf.log 2>&1 &
sleep 3
echo "static map->odom publisher running: $(pgrep -c -f static_transform_publisher)"
