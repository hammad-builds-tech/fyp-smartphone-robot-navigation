#!/bin/bash
# Clean Nav2 restart for the offline demo; sim/static-TF/backend stay up.
# All kill patterns in-file (pkill trap safety).
pkill -f 'navigation.launch.py' 2>/dev/null
pkill -f 'bt_navigator' 2>/dev/null
pkill -f 'planner_server' 2>/dev/null
pkill -f 'controller_server' 2>/dev/null
pkill -f 'behavior_server' 2>/dev/null
pkill -f 'smoother_server' 2>/dev/null
pkill -f 'lifecycle_manager' 2>/dev/null
pkill -f 'nav2_collision_monitor' 2>/dev/null
pkill -f 'velocity_smoother' 2>/dev/null
pkill -f 'waypoint_follower' 2>/dev/null
sleep 5

echo "== launching navigation =="
setsid env FYP_MAP=$HOME/FYP/realroom/video60/video60_map.yaml \
    "$HOME/FYP/scripts/fyp_phase.sh" indoor_nav_costmap navigation.launch.py \
    > /tmp/fyp_nav60_launch.log 2>&1 &

sleep 30
source /opt/ros/lyrical/setup.bash
source "$HOME/FYP/ros2_ws/install/setup.bash"

# Wait for each Nav2 managed node to reach active, retrying activation if the
# manager's transition response was lost under load.
for node in controller_server planner_server behavior_server bt_navigator; do
  for try in 1 2 3 4 5 6; do
    ST=$(timeout 20 ros2 lifecycle get /$node 2>/dev/null)
    case "$ST" in
      active*) echo "$node: $ST"; break ;;
      inactive*)
        timeout 30 ros2 lifecycle set /$node activate 2>/dev/null && sleep 2 ;;
      *)
        timeout 30 ros2 lifecycle set /$node configure 2>/dev/null
        sleep 2
        timeout 30 ros2 lifecycle set /$node activate 2>/dev/null
        sleep 2 ;;
    esac
  done
done
echo "== final states =="
for node in controller_server planner_server behavior_server bt_navigator; do
  echo "$node: $(timeout 20 ros2 lifecycle get /$node 2>/dev/null)"
done
