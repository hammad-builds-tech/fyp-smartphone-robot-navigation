#!/bin/bash

# Send a test navigation goal to Nav2

set -e

cd /home/hammad/FYP

# Source ROS2
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash

echo "======================================="
echo "Sending Test Navigation Goal"
echo "======================================="
echo

# Wait for Nav2 to be ready
echo "Waiting for Nav2 BT Navigator..."
timeout 30s bash -c 'until ros2 service list | grep -q navigate_to_pose; do sleep 1; done' || {
    echo "ERROR: Nav2 navigate_to_pose service not available"
    echo "Make sure the system is running: ./run_system.sh"
    exit 1
}

echo "Nav2 is ready!"
echo

# Send a simple navigation goal (2 meters forward)
echo "Sending goal: x=2.0, y=0.0"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
  pose:
    position:
      x: 2.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"

echo
echo "Goal sent! Watch the robot move in Gazebo."
