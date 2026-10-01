#!/bin/bash
# Restart ONLY nav2 (controller/planner/behavior/bt_navigator + their launch),
# leaving sim, robot, map_server, perception and TF untouched. Nudges each
# managed node to active with retries (lifecycle calls are slow under load).
pkill -f 'navigation.launch.py' 2>/dev/null
sleep 2
pkill -f 'lifecycle_manager' 2>/dev/null
pkill -f 'controller_server' 2>/dev/null
pkill -f 'planner_server' 2>/dev/null
pkill -f 'behavior_server' 2>/dev/null
pkill -f 'bt_navigator' 2>/dev/null
pkill -f 'smoother_server' 2>/dev/null
sleep 5

source /opt/ros/lyrical/setup.bash
source "$HOME/FYP/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
FYP_MAP=${FYP_MAP:-/home/hammad/FYP/realroom/capture_fresh/realroom_capture_fresh.yaml}

setsid env FYP_MAP="$FYP_MAP" \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap navigation.launch.py \
    > /tmp/fyp_nav30_launch.log 2>&1 &
sleep 35

for node in controller_server planner_server behavior_server bt_navigator; do
  for try in 1 2 3 4 5 6; do
    ST=$(timeout 20 ros2 lifecycle get /$node 2>/dev/null)
    case "$ST" in
      active*) echo "$node: $ST"; break ;;
      inactive*) timeout 30 ros2 lifecycle set /$node activate 2>/dev/null; sleep 2 ;;
      *) timeout 30 ros2 lifecycle set /$node configure 2>/dev/null
         sleep 2
         timeout 30 ros2 lifecycle set /$node activate 2>/dev/null
         sleep 2 ;;
    esac
  done
done
echo "== final nav states =="
for node in controller_server planner_server behavior_server bt_navigator; do
  echo "$node: $(timeout 20 ros2 lifecycle get /$node 2>/dev/null)"
done
