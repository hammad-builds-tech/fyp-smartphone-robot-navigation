#!/bin/bash

# FYP Complete System Launch Script

set -e

cd /home/hammad/FYP

echo "======================================="
echo "FYP Indoor Navigation System"
echo "======================================="
echo

# Source ROS2
if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    source /opt/ros/lyrical/setup.bash
else
    echo "ERROR: ROS2 Lyrical not found"
    exit 1
fi

# Source workspace
if [ -f "ros2_ws/install/setup.bash" ]; then
    source ros2_ws/install/setup.bash
else
    echo "ERROR: ROS workspace not built"
    echo "Run: cd ros2_ws && colcon build --symlink-install"
    exit 1
fi

echo "Launching complete system:"
echo "  - Gazebo simulation"
echo "  - ROS2 bridge"
echo "  - Robot spawn"
echo "  - Smartphone depth bridge"
echo "  - Depth-to-scan converter"
echo "  - Map server"
echo "  - Nav2 stack (controller, planner, behavior, BT navigator)"
echo

echo "IMPORTANT: Make sure the backend is running in another terminal:"
echo "  ./run_backend.sh"
echo

echo "After launch, you can:"
echo "  1. Send smartphone video to: ws://localhost:8000/ws/video"
echo "  2. Set navigation goals in RViz2 with '2D Goal Pose'"
echo "  3. Monitor topics: ros2 topic list"
echo "  4. View TF tree: ros2 run tf2_tools view_frames"
echo

read -p "Press ENTER to launch (Ctrl+C to cancel)..."

# Launch the integrated system
ros2 launch indoor_nav_gazebo fyp_bringup.launch.py
