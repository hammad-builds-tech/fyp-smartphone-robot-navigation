#!/bin/bash
# Standalone map_server for the offline realroom30 test (navigation.launch.py has no map_server).
# Idempotent: safe to re-run.
source /opt/ros/lyrical/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle
MAP_YAML="${MAP_YAML:-/home/hammad/FYP/realroom/capture30/realroom30_map.yaml}"

if pgrep -f "nav2_map_server map_server" >/dev/null; then
  echo "map_server already running"
else
  echo "starting map_server for $MAP_YAML"
  setsid nohup ros2 run nav2_map_server map_server \
    --ros-args -r __node:=map_server \
    -p yaml_filename:="$MAP_YAML" -p use_sim_time:=true -p frame_id:="map" \
    >> /tmp/fyp30_mapserver.log 2>&1 &
  sleep 3
fi

# Lifecycle: configure then activate, with retries
for st in configure activate; do
  for i in 1 2 3 4 5 6; do
    cur=$(timeout 6 ros2 lifecycle get /map_server 2>/dev/null)
    echo "attempt $i $st (current: $cur)"
    case "$cur" in
      *"unconfigured"*) [ "$st" = "configure" ] && timeout 8 ros2 lifecycle set /map_server configure >/dev/null 2>&1;;
      *"inactive"*)     [ "$st" = "activate" ] && timeout 8 ros2 lifecycle set /map_server activate >/dev/null 2>&1;;
      *"active"*)       [ "$st" = "activate" ] && { echo "map_server active"; exit 0; } || exit 0;;
    esac
    sleep 2
  done
done
echo "FAILED to activate map_server"
exit 1
